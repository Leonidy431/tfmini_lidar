"""
Localization Engine Module

Provides real-time position estimation within a known map.
Uses scan matching techniques to determine current position.
"""

import numpy as np
import open3d as o3d
from typing import Tuple, Optional, List
from datetime import datetime
from collections import deque
import threading
import logging

from app.config import Config

logger = logging.getLogger(__name__)


class LocalizationEngine:
    """
    Localization engine using scan-to-map matching

    For single-point LiDAR (TFmini-S):
    - Accumulates readings to form pseudo-scans
    - Matches against reference map using ICP
    - Tracks position confidence over time
    """

    def __init__(self, config=None):
        self.config = config or Config.localization

        # Reference map
        self.reference_map: Optional[o3d.geometry.PointCloud] = None
        self.reference_kdtree: Optional[o3d.geometry.KDTreeFlann] = None

        # Current state
        self.current_pose = np.eye(4)
        self.last_pose = np.eye(4)
        self.confidence = 0.0

        # Position history
        self.pose_history = deque(maxlen=1000)
        self.confidence_history = deque(maxlen=100)

        # Statistics
        self.localization_count = 0
        self.lost_count = 0
        self.consecutive_failures = 0

        # Scan accumulation
        self.scan_buffer: List[np.ndarray] = []
        self.buffer_size = 100

        # Thread safety
        self.lock = threading.Lock()

        # State
        self.is_initialized = False
        self.is_lost = False

    def set_reference_map(self, map_points: np.ndarray) -> bool:
        """
        Set the reference map for localization

        Args:
            map_points: Nx3 numpy array of map points
        """
        try:
            with self.lock:
                self.reference_map = o3d.geometry.PointCloud()
                self.reference_map.points = o3d.utility.Vector3dVector(map_points)

                # Estimate normals for better matching
                self.reference_map.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(
                        radius=0.1, max_nn=30
                    )
                )

                # Build KD-tree for fast nearest neighbor queries
                self.reference_kdtree = o3d.geometry.KDTreeFlann(self.reference_map)

                self.is_initialized = True
                logger.info(f"Reference map set ({len(map_points)} points)")
                return True

        except Exception as e:
            logger.error(f"Failed to set reference map: {e}")
            return False

    def add_point(self, x: float, y: float, z: float) -> Optional[dict]:
        """Add a point and return localization result if available"""
        with self.lock:
            self.scan_buffer.append(np.array([x, y, z]))

            if len(self.scan_buffer) >= self.buffer_size:
                points = np.array(self.scan_buffer)
                self.scan_buffer.clear()
                return self._localize(points)

        return None

    def localize(self, scan_points: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        Localize using a scan

        Args:
            scan_points: Nx3 numpy array of current scan points

        Returns:
            (pose_4x4, confidence)
        """
        with self.lock:
            return self._localize(scan_points)

    def _localize(self, scan_points: np.ndarray) -> dict:
        """Internal localization method (assumes lock is held)"""
        if not self.is_initialized or self.reference_map is None:
            return {
                'success': False,
                'error': 'Map not initialized',
                'position': (0, 0, 0),
                'confidence': 0
            }

        if len(scan_points) < 10:
            return {
                'success': False,
                'error': 'Insufficient points',
                'position': tuple(self.current_pose[:3, 3]),
                'confidence': self.confidence
            }

        try:
            # Create point cloud from scan
            scan_pcd = o3d.geometry.PointCloud()
            scan_pcd.points = o3d.utility.Vector3dVector(scan_points)

            # Downsample for efficiency
            scan_down = scan_pcd.voxel_down_sample(voxel_size=0.05)

            # Initial guess from last pose
            init_transform = self.current_pose

            # ICP registration
            result = o3d.pipelines.registration.registration_icp(
                scan_down,
                self.reference_map,
                self.config.map_matching_distance,
                init_transform,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=100,
                    relative_fitness=1e-6,
                    relative_rmse=1e-6
                )
            )

            fitness = result.fitness
            rmse = result.inlier_rmse

            # Update state based on result
            if fitness >= self.config.confidence_threshold:
                # Good localization
                self.last_pose = self.current_pose.copy()
                self.current_pose = result.transformation
                self.confidence = fitness
                self.consecutive_failures = 0
                self.is_lost = False

                # Record history
                self.pose_history.append({
                    'pose': self.current_pose.copy(),
                    'confidence': fitness,
                    'timestamp': datetime.now()
                })
                self.confidence_history.append(fitness)
                self.localization_count += 1

                position = tuple(self.current_pose[:3, 3])
                logger.debug(f"Localized: {position}, confidence: {fitness:.3f}")

                return {
                    'success': True,
                    'position': position,
                    'orientation': self._pose_to_euler(self.current_pose),
                    'confidence': round(fitness, 3),
                    'rmse': round(rmse, 4),
                    'transformation': self.current_pose.tolist()
                }
            else:
                # Poor localization
                self.consecutive_failures += 1

                if self.consecutive_failures >= self.config.lost_threshold:
                    self.is_lost = True
                    self.lost_count += 1
                    logger.warning("Localization lost!")

                return {
                    'success': False,
                    'error': 'Low confidence match',
                    'position': tuple(self.current_pose[:3, 3]),
                    'confidence': round(fitness, 3),
                    'is_lost': self.is_lost
                }

        except Exception as e:
            logger.error(f"Localization error: {e}")
            return {
                'success': False,
                'error': str(e),
                'position': tuple(self.current_pose[:3, 3]),
                'confidence': 0
            }

    def _pose_to_euler(self, pose: np.ndarray) -> Tuple[float, float, float]:
        """Convert pose matrix to Euler angles (roll, pitch, yaw) in degrees"""
        R = pose[:3, :3]

        # Extract Euler angles (ZYX convention)
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0

        return (
            round(np.degrees(roll), 2),
            round(np.degrees(pitch), 2),
            round(np.degrees(yaw), 2)
        )

    def get_position(self) -> Tuple[float, float, float]:
        """Get current estimated position"""
        with self.lock:
            return tuple(self.current_pose[:3, 3])

    def get_orientation(self) -> Tuple[float, float, float]:
        """Get current estimated orientation (roll, pitch, yaw)"""
        with self.lock:
            return self._pose_to_euler(self.current_pose)

    def get_pose(self) -> np.ndarray:
        """Get full 4x4 pose matrix"""
        with self.lock:
            return self.current_pose.copy()

    def get_trajectory(self) -> np.ndarray:
        """Get position history as Nx3 array"""
        with self.lock:
            if not self.pose_history:
                return np.array([])
            positions = [h['pose'][:3, 3] for h in self.pose_history]
            return np.array(positions)

    def reset(self, initial_pose: np.ndarray = None):
        """Reset localization state"""
        with self.lock:
            self.current_pose = initial_pose if initial_pose is not None else np.eye(4)
            self.last_pose = self.current_pose.copy()
            self.confidence = 0.0
            self.pose_history.clear()
            self.confidence_history.clear()
            self.scan_buffer.clear()
            self.consecutive_failures = 0
            self.is_lost = False
            logger.info("Localization reset")

    def get_statistics(self) -> dict:
        """Get localization statistics"""
        with self.lock:
            avg_confidence = (
                sum(self.confidence_history) / len(self.confidence_history)
                if self.confidence_history else 0
            )

            return {
                'initialized': self.is_initialized,
                'is_lost': self.is_lost,
                'localization_count': self.localization_count,
                'lost_count': self.lost_count,
                'consecutive_failures': self.consecutive_failures,
                'current_confidence': round(self.confidence, 3),
                'average_confidence': round(avg_confidence, 3),
                'position': {
                    'x': round(self.current_pose[0, 3], 3),
                    'y': round(self.current_pose[1, 3], 3),
                    'z': round(self.current_pose[2, 3], 3)
                },
                'orientation': {
                    'roll': self._pose_to_euler(self.current_pose)[0],
                    'pitch': self._pose_to_euler(self.current_pose)[1],
                    'yaw': self._pose_to_euler(self.current_pose)[2]
                },
                'buffer_size': len(self.scan_buffer),
                'history_size': len(self.pose_history)
            }


class ParticleFilterLocalizer:
    """
    Alternative localizer using particle filter

    Better for global localization (when position is unknown)
    but more computationally expensive.
    """

    def __init__(self, num_particles: int = 500, config=None):
        self.num_particles = num_particles
        self.config = config or Config.localization

        # Particles: Nx3 array of [x, y, theta]
        self.particles = None
        self.weights = None

        # Reference map
        self.reference_map: Optional[np.ndarray] = None

        # State
        self.is_initialized = False

    def set_reference_map(self, map_points: np.ndarray):
        """Set reference map"""
        self.reference_map = map_points

        # Initialize particles around map center
        map_center = np.mean(map_points, axis=0)
        map_std = np.std(map_points, axis=0)

        self.particles = np.random.normal(
            loc=[map_center[0], map_center[1], 0],
            scale=[map_std[0], map_std[1], np.pi],
            size=(self.num_particles, 3)
        )
        self.weights = np.ones(self.num_particles) / self.num_particles
        self.is_initialized = True

    def update(self, distance_reading: float, motion_delta: Tuple[float, float, float] = None):
        """
        Update particle filter with new measurement

        Args:
            distance_reading: Current LiDAR reading
            motion_delta: Optional (dx, dy, dtheta) motion estimate
        """
        if not self.is_initialized:
            return

        # Motion update
        if motion_delta is not None:
            dx, dy, dtheta = motion_delta
            noise = np.random.normal(0, 0.1, (self.num_particles, 3))
            self.particles[:, 0] += dx + noise[:, 0]
            self.particles[:, 1] += dy + noise[:, 1]
            self.particles[:, 2] += dtheta + noise[:, 2]

        # Measurement update
        for i in range(self.num_particles):
            expected_distance = self._ray_cast(
                self.particles[i, 0],
                self.particles[i, 1],
                self.particles[i, 2]
            )
            # Gaussian likelihood
            diff = distance_reading - expected_distance
            self.weights[i] *= np.exp(-0.5 * (diff / 0.3) ** 2)

        # Normalize weights
        self.weights /= np.sum(self.weights) + 1e-10

        # Resample if effective particle count is low
        neff = 1.0 / np.sum(self.weights ** 2)
        if neff < self.num_particles / 2:
            self._resample()

    def _ray_cast(self, x: float, y: float, theta: float) -> float:
        """Simple ray casting for expected distance"""
        if self.reference_map is None:
            return 10.0  # Default distance

        # Direction vector
        dx = np.cos(theta)
        dy = np.sin(theta)

        # Find intersection with map points
        min_dist = 12.0  # Max range

        for point in self.reference_map[::10]:  # Subsample for speed
            # Vector from position to point
            vx = point[0] - x
            vy = point[1] - y

            # Project onto ray direction
            proj = vx * dx + vy * dy

            if proj > 0:
                # Distance to point
                dist = np.sqrt(vx**2 + vy**2)
                if dist < min_dist:
                    min_dist = dist

        return min_dist

    def _resample(self):
        """Resample particles based on weights"""
        indices = np.random.choice(
            self.num_particles,
            self.num_particles,
            p=self.weights
        )
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles

    def get_estimate(self) -> Tuple[float, float, float, float]:
        """Get position estimate and confidence"""
        if not self.is_initialized:
            return 0, 0, 0, 0

        # Weighted mean
        x = np.average(self.particles[:, 0], weights=self.weights)
        y = np.average(self.particles[:, 1], weights=self.weights)
        theta = np.average(self.particles[:, 2], weights=self.weights)

        # Confidence based on particle spread
        spread = np.std(self.particles[:, :2], axis=0).mean()
        confidence = max(0, 1 - spread / 2)

        return x, y, theta, confidence
