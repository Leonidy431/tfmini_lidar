"""
Tests for the 3D orbit scanner: point-placement geometry, coverage tracking,
layers, rejection gates, and capacity limits.

The scanner engine is stdlib-only, so this suite runs without numpy/Open3D.
"""

import math
import sys
import os
import threading

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.scanner_3d import Scanner3D
from app.config import ScannerConfig


def make_scanner(**overrides) -> Scanner3D:
    cfg = ScannerConfig(**overrides)
    return Scanner3D(cfg)


class TestGeometry:
    """S = C + (d - R) * u(heading), at the current layer z."""

    def test_point_on_heading_zero(self):
        s = make_scanner(orbit_radius=5.0)
        s.start_scan(center=(10.0, 0.0, 0.0), orbit_radius=5.0)

        # heading 0 -> u = (1, 0); d=3, R=5 -> S = C + (3-5)*(1,0) = (8, 0)
        result = s.add_reading(distance=3.0, heading_deg=0.0, signal_strength=200)
        assert result['accepted'] is True
        x, y, z = result['point']
        assert abs(x - 8.0) < 1e-6
        assert abs(y - 0.0) < 1e-6
        assert abs(z - 0.0) < 1e-6

    def test_point_on_heading_ninety(self):
        s = make_scanner(orbit_radius=4.0)
        s.start_scan(center=(0.0, 0.0, 0.0), orbit_radius=4.0)

        # heading 90 -> u = (0, 1); d=1, R=4 -> S = (0, -3)
        result = s.add_reading(distance=1.0, heading_deg=90.0, signal_strength=200)
        x, y, _ = result['point']
        assert abs(x - 0.0) < 1e-6
        assert abs(y + 3.0) < 1e-6

    def test_point_between_carrier_and_center(self):
        """Surface points must lie inside the orbit circle."""
        s = make_scanner(orbit_radius=3.0)
        s.start_scan(center=(0.0, 0.0, 0.0), orbit_radius=3.0)

        for heading in range(0, 360, 15):
            result = s.add_reading(distance=2.0, heading_deg=float(heading),
                                   signal_strength=200)
            x, y, _ = result['point']
            r = math.hypot(x, y)
            assert r < 3.0 + 1e-9  # strictly inside the orbit

    def test_layer_z_applied(self):
        s = make_scanner(orbit_radius=3.0, layer_height=0.5)
        s.start_scan(center=(0.0, 0.0, -2.0), orbit_radius=3.0, initial_z=0.0)
        s.set_layer(-1.0)

        result = s.add_reading(distance=1.5, heading_deg=0.0, signal_strength=200)
        _, _, z = result['point']
        # center z (-2.0) + layer z (-1.0)
        assert abs(z + 3.0) < 1e-6


class TestRejectionGates:
    def test_beyond_center_rejected(self):
        """d >= R means the beam missed the object entirely."""
        s = make_scanner(orbit_radius=3.0)
        s.start_scan(orbit_radius=3.0)
        result = s.add_reading(distance=3.5, heading_deg=0.0, signal_strength=200)
        assert result['accepted'] is False
        assert result['reason'] == 'beyond_center'

    def test_weak_signal_rejected(self):
        s = make_scanner(orbit_radius=3.0, signal_threshold=100)
        s.start_scan(orbit_radius=3.0)
        result = s.add_reading(distance=1.0, heading_deg=0.0, signal_strength=10)
        assert result['accepted'] is False
        assert result['reason'] == 'low_signal'

    def test_too_close_rejected(self):
        s = make_scanner(orbit_radius=3.0, min_distance=0.2)
        s.start_scan(orbit_radius=3.0)
        result = s.add_reading(distance=0.05, heading_deg=0.0, signal_strength=200)
        assert result['accepted'] is False
        assert result['reason'] == 'too_close'

    def test_not_scanning_returns_none(self):
        s = make_scanner()
        assert s.add_reading(distance=1.0, heading_deg=0.0, signal_strength=200) is None

    def test_capacity_cap(self):
        s = make_scanner(orbit_radius=3.0, max_points=10)
        s.start_scan(orbit_radius=3.0)
        for i in range(15):
            s.add_reading(distance=1.0, heading_deg=float(i), signal_strength=200)
        assert len(s.points) == 10
        assert s.rejected_by['capacity'] == 5

    def test_invalid_orbit_radius_refused(self):
        s = make_scanner()
        assert s.start_scan(orbit_radius=0.0) is False
        assert s.start_scan(orbit_radius=50.0) is False
        assert s.is_scanning is False


class TestCoverage:
    def test_full_ring_reaches_full_coverage(self):
        s = make_scanner(orbit_radius=3.0, angular_resolution_deg=5.0)
        s.start_scan(orbit_radius=3.0)

        for heading in range(0, 360):
            s.add_reading(distance=1.5, heading_deg=float(heading),
                          signal_strength=200)

        stats = s.get_statistics()
        assert stats['overall_coverage'] >= 0.99
        assert stats['layer_count'] == 1

    def test_half_ring_partial_coverage(self):
        s = make_scanner(orbit_radius=3.0, angular_resolution_deg=5.0)
        s.start_scan(orbit_radius=3.0)

        for heading in range(0, 180):
            s.add_reading(distance=1.5, heading_deg=float(heading),
                          signal_strength=200)

        cov = s.get_statistics()['overall_coverage']
        assert 0.4 < cov < 0.6

    def test_multiple_layers_tracked(self):
        s = make_scanner(orbit_radius=3.0, layer_height=0.5)
        s.start_scan(orbit_radius=3.0, initial_z=0.0)

        for heading in range(0, 360, 10):
            s.add_reading(distance=1.5, heading_deg=float(heading),
                          signal_strength=200)
        s.set_layer(-0.5)
        for heading in range(0, 360, 10):
            s.add_reading(distance=1.5, heading_deg=float(heading),
                          signal_strength=200)

        stats = s.get_statistics()
        assert stats['layer_count'] == 2


class TestLifecycle:
    def test_stop_returns_stats_and_keeps_points(self):
        s = make_scanner(orbit_radius=3.0)
        s.start_scan(orbit_radius=3.0)
        s.add_reading(distance=1.0, heading_deg=0.0, signal_strength=200)

        stats = s.stop_scan()
        assert stats['is_scanning'] is False
        assert stats['point_count'] == 1
        assert len(s.get_points()) == 1  # data survives stop for saving

    def test_clear_resets_everything(self):
        s = make_scanner(orbit_radius=3.0)
        s.start_scan(orbit_radius=3.0)
        s.add_reading(distance=1.0, heading_deg=0.0, signal_strength=200)
        s.clear()

        stats = s.get_statistics()
        assert stats['point_count'] == 0
        assert stats['layer_count'] == 0
        assert stats['is_scanning'] is False

    def test_restart_resets_previous_scan(self):
        s = make_scanner(orbit_radius=3.0)
        s.start_scan(orbit_radius=3.0)
        s.add_reading(distance=1.0, heading_deg=0.0, signal_strength=200)

        s.start_scan(orbit_radius=2.0)
        assert s.get_statistics()['point_count'] == 0
        assert s.orbit_radius == 2.0

    def test_get_points_stride(self):
        s = make_scanner(orbit_radius=3.0)
        s.start_scan(orbit_radius=3.0)
        for i in range(100):
            s.add_reading(distance=1.0, heading_deg=float(i % 360),
                          signal_strength=200)
        pts = s.get_points(max_points=10)
        assert len(pts) <= 10


class TestConcurrency:
    def test_concurrent_readings_and_status(self):
        s = make_scanner(orbit_radius=3.0)
        s.start_scan(orbit_radius=3.0)
        errors = []

        def producer():
            try:
                for i in range(300):
                    s.add_reading(distance=1.0 + (i % 10) * 0.1,
                                  heading_deg=float(i % 360),
                                  signal_strength=200)
            except Exception as e:  # noqa: BLE001
                errors.append(e)

        def reader():
            try:
                for _ in range(300):
                    s.get_statistics()
                    s.get_points(max_points=100)
            except Exception as e:  # noqa: BLE001
                errors.append(e)

        threads = [threading.Thread(target=producer) for _ in range(2)]
        threads += [threading.Thread(target=reader) for _ in range(2)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert errors == []


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
