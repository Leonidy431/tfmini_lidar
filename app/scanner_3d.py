"""
3D Object Scanner Module

Turns the single-point TFmini-S into a 3D object scanner: the carrier vehicle
(surface boat, drone, or ROV) orbits a target object in a circle with the
sensor pointing at the object center. Each range reading is converted to a
surface point; a full orbit produces a ring of points, and repeating the orbit
at different depths/altitudes stacks rings into a dense 3D point cloud —
an inverted turntable scanner (the sensor moves, the object stays still).

Orbit-scan geometry
-------------------
Let C be the object center, R the orbit radius, h the vehicle COMPASS
heading (degrees, clockwise-positive from North) and
u(h) = [sin h, cos h, 0] the sensor direction (aimed at C) in the app's ENU
world frame (x=East, y=North). The carrier position is derived as:

    P = C - R * u(h)

and a range reading d maps to the surface point:

    S = P + d * u(h) = C + (d - R) * u(h)      (at the current layer depth z)

Readings with d >= R passed the object center (a miss) and are rejected.

Coverage is tracked per angular bin per layer so the operator can see when a
ring is complete before moving to the next depth.

The module is stdlib-only (no numpy/Open3D) so it stays testable in minimal CI
environments; conversion to numpy happens at the API boundary.
"""

import math
import threading
import time
import logging
from typing import Dict, List, Optional, Tuple

from app.config import Config

logger = logging.getLogger(__name__)


class Scanner3D:
    """
    Orbit-scan engine building a 3D point cloud of a single object.

    Usage:
        scanner.start_scan(center=(0, 0, 0), orbit_radius=3.0, initial_z=0.0)
        # vehicle orbits; feed readings:
        scanner.add_reading(distance=2.1, heading_deg=137.0, signal_strength=250)
        # operator moves to the next depth ring:
        scanner.set_layer(-0.5)
        ...
        stats = scanner.stop_scan()
    """

    def __init__(self, config=None):
        self.config = config or Config.scanner

        # Thread safety: readings arrive on the processing worker while the
        # API reads status from Flask request threads.
        self.lock = threading.Lock()

        self._reset_state()

    def _reset_state(self):
        self.is_scanning = False
        self.center: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.orbit_radius: float = self.config.orbit_radius
        self.current_z: float = 0.0
        self.started_at: Optional[float] = None
        self.finished_at: Optional[float] = None

        # Point cloud: list of [x, y, z]
        self.points: List[List[float]] = []

        # Angular coverage per layer: layer_index -> set(bin_index)
        self.layers: Dict[int, set] = {}

        # Statistics
        self.total_readings = 0
        self.accepted = 0
        self.rejected = 0
        self.rejected_by = {
            'not_scanning': 0,
            'low_signal': 0,
            'too_close': 0,
            'beyond_center': 0,
            'capacity': 0,
        }

    # ---------------- Lifecycle ----------------

    def start_scan(self,
                   center: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                   orbit_radius: Optional[float] = None,
                   initial_z: float = 0.0) -> bool:
        """Begin a new scan around `center` at orbit radius `orbit_radius`."""
        radius = orbit_radius if orbit_radius is not None else self.config.orbit_radius

        # Physical sanity: orbit must be within sensor range and leave room
        # for an object between carrier and center.
        if not (self.config.min_distance < radius <= 12.0):
            logger.error(f"Invalid orbit radius: {radius}")
            return False

        with self.lock:
            self._reset_state()
            self.center = (float(center[0]), float(center[1]), float(center[2]))
            self.orbit_radius = float(radius)
            self.current_z = float(initial_z)
            self.is_scanning = True
            self.started_at = time.time()

        logger.info(
            f"3D scan started: center={self.center}, R={self.orbit_radius}m, "
            f"z={self.current_z}m"
        )
        return True

    def stop_scan(self) -> dict:
        """Stop scanning and return final statistics."""
        with self.lock:
            if self.is_scanning:
                self.is_scanning = False
                self.finished_at = time.time()
                logger.info(
                    f"3D scan stopped: {len(self.points)} points, "
                    f"{len(self.layers)} layers"
                )
        return self.get_statistics()

    def set_layer(self, z: float):
        """Set the current scan depth/altitude (operator moved the vehicle)."""
        with self.lock:
            self.current_z = float(z)
        logger.info(f"Scan layer set to z={z}m")

    def clear(self):
        """Discard all scan data."""
        with self.lock:
            self._reset_state()
        logger.info("3D scanner cleared")

    # ---------------- Data path ----------------

    def add_reading(self, distance: float, heading_deg: float,
                    signal_strength: int) -> Optional[dict]:
        """
        Convert one range reading into a surface point.

        Returns a result dict {accepted, reason?, point?, layer, coverage} or
        None when not scanning.
        """
        with self.lock:
            if not self.is_scanning:
                self.rejected_by['not_scanning'] += 1
                return None

            self.total_readings += 1

            if signal_strength < self.config.signal_threshold:
                return self._reject('low_signal')

            if distance < self.config.min_distance:
                return self._reject('too_close')

            # d >= R means the beam passed the object center without a hit —
            # there is no object surface on this bearing (or the center /
            # radius is misconfigured).
            if distance >= self.orbit_radius:
                return self._reject('beyond_center')

            if len(self.points) >= self.config.max_points:
                return self._reject('capacity')

            # S = C + (d - R) * u(h). heading_deg is a COMPASS heading
            # (clockwise-positive from North); u(h) must be expressed in the
            # same ENU convention used everywhere else in the app (Physics
            # Audit C4) so scanner geometry stays consistent with mapping.
            h = math.radians(heading_deg)
            ux, uy = math.sin(h), math.cos(h)
            offset = distance - self.orbit_radius  # negative: carrier side
            x = self.center[0] + offset * ux
            y = self.center[1] + offset * uy
            z = self.center[2] + self.current_z

            self.points.append([x, y, z])
            self.accepted += 1

            # Coverage: carrier azimuth around the center is heading + 180deg
            layer = self._layer_index(self.current_z)
            theta = (heading_deg + 180.0) % 360.0
            bin_idx = int(theta // self.config.angular_resolution_deg) % self._n_bins()
            self.layers.setdefault(layer, set()).add(bin_idx)

            return {
                'accepted': True,
                'point': [round(x, 4), round(y, 4), round(z, 4)],
                'layer': layer,
                'layer_coverage': self._layer_coverage(layer),
            }

    def _reject(self, reason: str) -> dict:
        self.rejected += 1
        self.rejected_by[reason] += 1
        return {'accepted': False, 'reason': reason}

    # ---------------- Coverage helpers ----------------

    def _n_bins(self) -> int:
        return max(1, int(360.0 / self.config.angular_resolution_deg))

    def _layer_index(self, z: float) -> int:
        if self.config.layer_height <= 0:
            return 0
        return int(round(z / self.config.layer_height))

    def _layer_coverage(self, layer: int) -> float:
        bins = self.layers.get(layer)
        if not bins:
            return 0.0
        return round(len(bins) / self._n_bins(), 3)

    # ---------------- Queries ----------------

    def get_points(self, max_points: Optional[int] = None) -> List[List[float]]:
        """Return the point cloud, optionally strided down to max_points."""
        with self.lock:
            pts = self.points
            if max_points and len(pts) > max_points:
                stride = math.ceil(len(pts) / max_points)
                pts = pts[::stride]
            return [list(p) for p in pts]

    def get_statistics(self) -> dict:
        """Scan progress and golden-signal metrics."""
        with self.lock:
            # Compute each layer's coverage once instead of 2-3x
            # (Blind Spot Audit R3, R3-PERF-6) -- this is broadcast every
            # progress_emit_every accepted readings during an active scan.
            layer_coverages = {layer: self._layer_coverage(layer) for layer in self.layers}
            layers = {
                str(layer * self.config.layer_height): {
                    'coverage': coverage,
                    'complete': coverage >= self.config.min_coverage_complete,
                }
                for layer, coverage in sorted(layer_coverages.items())
            }
            coverages = list(layer_coverages.values())
            duration = 0.0
            if self.started_at:
                end = self.finished_at or time.time()
                duration = round(end - self.started_at, 1)

            return {
                'is_scanning': self.is_scanning,
                'center': list(self.center),
                'orbit_radius': self.orbit_radius,
                'current_z': self.current_z,
                'point_count': len(self.points),
                'layer_count': len(self.layers),
                'layers': layers,
                'overall_coverage': round(sum(coverages) / len(coverages), 3)
                                    if coverages else 0.0,
                'total_readings': self.total_readings,
                'accepted': self.accepted,
                'rejected': self.rejected,
                'rejected_by': dict(self.rejected_by),
                'duration_seconds': duration,
                'capacity_used': round(len(self.points) / self.config.max_points, 4),
            }
