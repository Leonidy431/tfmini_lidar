"""
SLAM Engine Module

Implements scan-to-scan ICP registration for mapping with single-point LiDAR.
Since TFmini-S is a single-point sensor, we accumulate readings with motion
estimation to create pseudo-scans.
"""

import numpy as np
import open3d as o3d
from typing import Optional, Tuple, List
from datetime import datetime
from collections import deque
import logging
import threading

from app.config import Config

logger = logging.getLogger(__name__)


class PointCloud:
    """Point cloud wrapper with metadata"""

    def __init__(self, points: np.ndarray, timestamp: datetime):
        self.points = points
        self.timestamp = timestamp
        self.pcd: Optional[o3d.geometry.PointCloud] = None
        self._create_o3d_cloud()

    def _create_o3d_cloud(self):
        """Create Open3D point cloud"""
        if len(self.points) > 0:
            self.pcd = o3d.geometry.PointCloud()
            self.pcd.points = o3d.utility.Vector3dVector(self.points)

    @property
    def size(self) -> int:
        return len(self.points)


class SLAMEngine:
    """
    SLAM Engine using ICP-based scan matching

    For single-point LiDAR (TFmini-S), this works by:
    1. Accumulating readings over time with estimated motion
    2. Creating pseudo-scans from accumulated points
    3. Using ICP to register successive scans
    """

    def __init__(self, config=None):
        self.config = config or Config.slam

        # Point cloud storage
        self.reference_cloud: Optional[PointCloud] = None
        self.accumulated_cloud: Optional[o3d.geometry.PointCloud] = None
        self.scan_buffer: List[np.ndarray] = []

        # Pose tracking
        self.poses: List[np.ndarray] = []
        self.current_pose = np.eye(4)

        # Statistics
        self.total_points = 0
        self.total_scans = 0
        self.drift_estimate = 0.0

        # Thread safety
        self.lock = threading.Lock()

        # Map bounds
        self.map_bounds = {
            'min': np.array([0.0, 0.0, 0.0]),
            'max': np.array([0.0, 0.0, 0.0])
        }

    def add_point(self, x: float, y: float, z: float):
        """Add a single point to the scan buffer"""
        with self.lock:
            self.scan_buffer.append(np.array([x, y, z]))

            if len(self.scan_buffer) >= self.config.buffer_size:
                self._process_buffer()

    def add_points(self, points: np.ndarray):
        """Add multiple points at once"""
        with self.lock:
            for point in points:
                self.scan_buffer.append(point)

            if len(self.scan_buffer) >= self.config.buffer_size:
                self._process_buffer()

    def _process_buffer(self):
        """Process accumulated buffer into a scan"""
        if len(self.scan_buffer) == 0:
            return

        points_array = np.array(self.scan_buffer)
        self.scan_buffer.clear()

        success, transformation = self.process_scan(points_array, datetime.now())
        return success

    def process_scan(self, points_3d: np.ndarray, timestamp: datetime) -> Tuple[bool, np.ndarray]:
        """
        Process a single scan

        Args:
            points_3d: Nx3 array of points
            timestamp: Scan timestamp

        Returns:
            (success, transformation_matrix)
        """
        if len(points_3d) == 0:
            return False, np.eye(4)

        # Filter invalid points
        valid_mask = np.all(np.isfinite(points_3d), axis=1)
        points_3d = points_3d[valid_mask]

        if len(points_3d) < 10:
            logger.warning("Not enough valid points for scan processing")
            return False, np.eye(4)

        # Create point cloud
        current_cloud = PointCloud(points_3d, timestamp)

        if self.reference_cloud is None:
            # First scan - use as reference
            self.reference_cloud = current_cloud
            self.accumulated_cloud = current_cloud.pcd
            self.poses.append(np.eye(4))
            self.total_scans = 1
            self.total_points = len(points_3d)
            self._update_bounds(points_3d)
            logger.info(f"Initialized first scan ({len(points_3d)} points)")
            return True, np.eye(4)

        # ICP registration
        transformation, success = self._register_clouds(
            current_cloud.pcd,
            self.reference_cloud.pcd
        )

        if success:
            # Transform current cloud to global frame
            current_cloud.pcd.transform(transformation)

            # Update current pose
            self.current_pose = transformation @ self.current_pose

            # Add to accumulated cloud
            self.accumulated_cloud = self.accumulated_cloud + current_cloud.pcd

            # Downsample if too large
            if len(self.accumulated_cloud.points) > 500000:
                self.accumulated_cloud = self.accumulated_cloud.voxel_down_sample(
                    voxel_size=self.config.voxel_size * 2
                )

            # Update reference
            self.reference_cloud = current_cloud

            # Store pose
            self.poses.append(self.current_pose.copy())
            self.total_scans += 1
            self.total_points = len(self.accumulated_cloud.points)

            # Update bounds
            self._update_bounds(np.asarray(current_cloud.pcd.points))

            # Estimate drift
            self.drift_estimate = np.linalg.norm(transformation[:3, 3])

            logger.debug(f"Scan registered (displacement: {self.drift_estimate:.3f}m)")
            return True, transformation
        else:
            logger.warning("Scan registration failed")
            return False, np.eye(4)

    def _register_clouds(self, source: o3d.geometry.PointCloud,
                        target: o3d.geometry.PointCloud) -> Tuple[np.ndarray, bool]:
        """
        Register source to target using ICP

        Returns:
            (transformation_matrix, success)
        """
        try:
            # Downsample for faster registration
            source_down = source.voxel_down_sample(voxel_size=self.config.voxel_size)
            target_down = target.voxel_down_sample(voxel_size=self.config.voxel_size)

            # Estimate normals for point-to-plane ICP
            source_down.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.config.voxel_size * 2,
                    max_nn=30
                )
            )
            target_down.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.config.voxel_size * 2,
                    max_nn=30
                )
            )

            # ICP registration
            result = o3d.pipelines.registration.registration_icp(
                source_down,
                target_down,
                self.config.max_correspondence_distance,
                np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=self.config.max_iterations,
                    relative_fitness=1e-6,
                    relative_rmse=1e-6
                )
            )

            success = result.fitness > self.config.icp_threshold
            return result.transformation, success

        except Exception as e:
            logger.error(f"ICP registration error: {e}")
            return np.eye(4), False

    def _update_bounds(self, points: np.ndarray):
        """Update map bounds"""
        if len(points) > 0:
            point_min = points.min(axis=0)
            point_max = points.max(axis=0)

            self.map_bounds['min'] = np.minimum(self.map_bounds['min'], point_min)
            self.map_bounds['max'] = np.maximum(self.map_bounds['max'], point_max)

    def get_map(self) -> Optional[np.ndarray]:
        """Get current map as numpy array"""
        with self.lock:
            if self.accumulated_cloud is None:
                return None
            return np.asarray(self.accumulated_cloud.points)

    def get_map_downsampled(self, voxel_size: float = None) -> Optional[np.ndarray]:
        """Get downsampled map"""
        with self.lock:
            if self.accumulated_cloud is None:
                return None

            voxel = voxel_size or self.config.voxel_size
            downsampled = self.accumulated_cloud.voxel_down_sample(voxel_size=voxel)
            return np.asarray(downsampled.points)

    def get_trajectory(self) -> np.ndarray:
        """Get trajectory as array of positions"""
        positions = []
        for pose in self.poses:
            positions.append(pose[:3, 3])
        return np.array(positions) if positions else np.array([])

    def get_current_position(self) -> Tuple[float, float, float]:
        """Get current position"""
        return tuple(self.current_pose[:3, 3])

    def clear(self):
        """Clear all accumulated data"""
        with self.lock:
            self.reference_cloud = None
            self.accumulated_cloud = None
            self.scan_buffer.clear()
            self.poses.clear()
            self.current_pose = np.eye(4)
            self.total_points = 0
            self.total_scans = 0
            self.drift_estimate = 0.0
            self.map_bounds = {
                'min': np.array([0.0, 0.0, 0.0]),
                'max': np.array([0.0, 0.0, 0.0])
            }
        logger.info("SLAM engine cleared")

    def save_map(self, filepath: str) -> bool:
        """Save map to file"""
        try:
            with self.lock:
                if self.accumulated_cloud is None:
                    logger.error("No map to save")
                    return False

                o3d.io.write_point_cloud(filepath, self.accumulated_cloud)
                logger.info(f"Map saved to {filepath}")
                return True

        except Exception as e:
            logger.error(f"Failed to save map: {e}")
            return False

    def load_map(self, filepath: str) -> bool:
        """Load map from file"""
        try:
            with self.lock:
                self.accumulated_cloud = o3d.io.read_point_cloud(filepath)
                self.total_points = len(self.accumulated_cloud.points)

                # Update bounds
                points = np.asarray(self.accumulated_cloud.points)
                if len(points) > 0:
                    self.map_bounds['min'] = points.min(axis=0)
                    self.map_bounds['max'] = points.max(axis=0)

                logger.info(f"Map loaded from {filepath} ({self.total_points} points)")
                return True

        except Exception as e:
            logger.error(f"Failed to load map: {e}")
            return False

    def get_statistics(self) -> dict:
        """Get engine statistics"""
        return {
            'total_points': self.total_points,
            'total_scans': self.total_scans,
            'buffer_size': len(self.scan_buffer),
            'drift_estimate': round(self.drift_estimate, 4),
            'current_position': {
                'x': round(self.current_pose[0, 3], 3),
                'y': round(self.current_pose[1, 3], 3),
                'z': round(self.current_pose[2, 3], 3)
            },
            'map_bounds': {
                'min': self.map_bounds['min'].tolist(),
                'max': self.map_bounds['max'].tolist()
            },
            'map_size': {
                'x': round(self.map_bounds['max'][0] - self.map_bounds['min'][0], 2),
                'y': round(self.map_bounds['max'][1] - self.map_bounds['min'][1], 2),
                'z': round(self.map_bounds['max'][2] - self.map_bounds['min'][2], 2)
            }
        }
