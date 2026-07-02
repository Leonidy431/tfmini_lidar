"""
SLAM Engine Module

Implements scan-to-scan ICP registration for mapping with single-point LiDAR.
Since TFmini-S is a single-point sensor, we accumulate readings with motion
estimation to create pseudo-scans.

Algorithm attribution (prior art):
- ICP registration: P. Besl and N. McKay, "A Method for Registration of 3-D
  Shapes", IEEE PAMI, 1992. Implementation via Open3D (MIT license).
See docs/ALGORITHMS.md and LICENSES.md.
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
        # Accumulated registration-error proxy (Physics Audit: dead
        # reckoning drift semantics) -- grows with mission length/scan
        # count, unlike last_displacement below.
        self.drift_estimate = 0.0
        # Most recent single-scan translation magnitude, reported
        # separately so it is never confused with accumulated drift.
        self.last_displacement = 0.0

        # Thread safety
        self.lock = threading.Lock()

        # Map bounds. Seeded at +/-infinity (the identity elements for
        # running min/max), not the zero vector: seeding at zero forces the
        # origin into every bounding box regardless of where the data
        # actually is (Physics Audit H6).
        self.map_bounds = {
            'min': np.full(3, np.inf),
            'max': np.full(3, -np.inf)
        }

        # Re-orthonormalize the accumulated pose's rotation block every N
        # scans (Physics Audit M1): repeated float64 matrix products do not
        # preserve orthogonality, so a long chain of composed transforms
        # accumulates spurious scale/shear on the SO(3) block.
        self._scans_since_reortho = 0

    # Hard cap to bound memory if processing ever stalls (Performance #1).
    MAX_BUFFER_MULTIPLIER = 2

    # Re-orthonormalization cadence (Physics Audit M1).
    REORTHONORMALIZE_EVERY = 50

    # Degeneracy threshold for point-to-plane ICP eligibility (Physics
    # Audit C6): ratio of 2nd-largest to largest covariance eigenvalue.
    # Below this the scan is effectively collinear/planar and normals are
    # numerically arbitrary.
    PLANARITY_EIGENVALUE_RATIO = 0.01

    def add_point(self, x: float, y: float, z: float):
        """Add a single point to the scan buffer"""
        with self.lock:
            self.scan_buffer.append(np.array([x, y, z]))

            if len(self.scan_buffer) >= self.config.buffer_size:
                self._process_buffer()
            else:
                self._enforce_buffer_cap()

    def add_points(self, points: np.ndarray):
        """Add multiple points at once"""
        with self.lock:
            for point in points:
                self.scan_buffer.append(point)

            if len(self.scan_buffer) >= self.config.buffer_size:
                self._process_buffer()
            else:
                self._enforce_buffer_cap()

    def _enforce_buffer_cap(self):
        """Trim the oldest points if the buffer grows beyond its hard cap."""
        cap = self.config.buffer_size * self.MAX_BUFFER_MULTIPLIER
        if len(self.scan_buffer) > cap:
            overflow = len(self.scan_buffer) - cap
            del self.scan_buffer[:overflow]
            logger.warning(f"Scan buffer capped, dropped {overflow} old points")

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

        # Motion pre-check (Performance #2): if the scan centroid has barely
        # moved since the reference, skip the expensive ICP and reuse identity.
        # Centroid displacement ALONE is blind to pure rotation -- a vehicle
        # yawing in place in front of a roughly symmetric scene can leave the
        # centroid essentially unchanged while the sampled geometry changes
        # completely (Physics Audit H4). Also compare scan covariance so
        # rotation-dominated motion is not misclassified as "no motion".
        motion_threshold = getattr(self.config, 'motion_threshold', 0.01)
        ref_points = np.asarray(self.reference_cloud.pcd.points)
        ref_centroid = np.mean(ref_points, axis=0)
        cur_centroid = np.mean(points_3d, axis=0)
        centroid_shift = np.linalg.norm(cur_centroid - ref_centroid)

        cov_shift = 0.0
        if len(ref_points) >= 3 and len(points_3d) >= 3:
            cov_diff = np.cov(points_3d.T) - np.cov(ref_points.T)
            cov_shift = float(np.linalg.norm(cov_diff, ord='fro'))

        if centroid_shift < motion_threshold and cov_shift < motion_threshold:
            logger.debug("Motion below threshold (centroid+covariance), skipping ICP")
            return True, np.eye(4)

        # ICP registration
        transformation, success, inlier_rmse = self._register_clouds(
            current_cloud.pcd,
            self.reference_cloud.pcd,
            points_3d
        )

        if success:
            # Transform current cloud to global frame
            current_cloud.pcd.transform(transformation)

            # Compose the scan-frame delta onto the world pose. ICP returns
            # a transform mapping the current (scan-frame) cloud into the
            # reference frame -- a relative delta expressed in the previous
            # scan's frame must be RIGHT-multiplied onto the world pose
            # (T_w_cur = T_w_prev @ T_prev_cur). Left-multiplying (as before)
            # is only valid for a delta already expressed in world frame,
            # and silently corrupts the trajectory on any non-straight-line
            # motion (Physics Audit C5).
            self.current_pose = self.current_pose @ transformation
            self._scans_since_reortho += 1
            if self._scans_since_reortho >= self.REORTHONORMALIZE_EVERY:
                self._reorthonormalize_pose()
                self._scans_since_reortho = 0

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

            # Accumulate drift as a registration-error proxy, not the raw
            # per-scan displacement (Physics Audit H-dead-reckoning): drift
            # is the growing, unbounded integration error of chained
            # relative transforms, not an instantaneous velocity proxy. A
            # vehicle moving fast with perfect registration should not
            # report large "drift"; a vehicle that has chained many noisy
            # registrations should, even while momentarily stationary.
            self.drift_estimate += inlier_rmse
            self.last_displacement = float(np.linalg.norm(transformation[:3, 3]))

            logger.debug(
                f"Scan registered (displacement: {self.last_displacement:.3f}m, "
                f"inlier_rmse: {inlier_rmse:.4f}m)"
            )
            return True, transformation
        else:
            logger.warning("Scan registration failed")
            return False, np.eye(4)

    def _reorthonormalize_pose(self):
        """Re-project the accumulated pose's rotation block onto SO(3).

        Chained float64 matrix products drift off orthogonality over a long
        mission (Physics Audit M1); SVD re-projection is the standard fix.
        """
        R = self.current_pose[:3, :3]
        U, _, Vt = np.linalg.svd(R)
        Rn = U @ Vt
        if np.linalg.det(Rn) < 0:
            Rn = U @ np.diag([1, 1, -1]) @ Vt
        self.current_pose[:3, :3] = Rn

    def _is_geometrically_degenerate(self, points_3d: np.ndarray) -> bool:
        """
        Check whether a point set is too close to collinear/planar for
        point-to-plane ICP to be well-posed (Physics Audit C6).

        Single-point-LiDAR pseudo-scans are frequently near-collinear
        (readings along one beam sweep). Surface-normal estimation on such
        data returns a numerically arbitrary normal (rank-deficient local
        covariance), and point-to-plane ICP then solves an under-constrained
        system that can report high fitness for an arbitrary transform.
        """
        if len(points_3d) < 3:
            return True
        try:
            eigenvalues = np.linalg.eigvalsh(np.cov(points_3d.T))
        except np.linalg.LinAlgError:
            return True
        eigenvalues = np.sort(np.abs(eigenvalues))
        if eigenvalues[-1] < 1e-12:
            return True
        return bool((eigenvalues[1] / eigenvalues[-1]) < self.PLANARITY_EIGENVALUE_RATIO)

    def _register_clouds(self, source: o3d.geometry.PointCloud,
                        target: o3d.geometry.PointCloud,
                        source_points: np.ndarray = None
                        ) -> Tuple[np.ndarray, bool, float]:
        """
        Register source to target using ICP

        Returns:
            (transformation_matrix, success, inlier_rmse)
        """
        try:
            # Downsample for faster registration
            source_down = source.voxel_down_sample(voxel_size=self.config.voxel_size)
            target_down = target.voxel_down_sample(voxel_size=self.config.voxel_size)

            # Degenerate (near-collinear/planar) geometry makes normal
            # estimation and point-to-plane ICP numerically unreliable;
            # fall back to point-to-point, which needs no normals and is
            # well-posed on any non-degenerate point count (Physics Audit
            # C6).
            degenerate = self._is_geometrically_degenerate(
                source_points if source_points is not None
                else np.asarray(source_down.points)
            )

            if degenerate:
                estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()
            else:
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
                estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()

            # ICP registration
            result = o3d.pipelines.registration.registration_icp(
                source_down,
                target_down,
                self.config.max_correspondence_distance,
                np.eye(4),
                estimation,
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=self.config.max_iterations,
                    relative_fitness=1e-6,
                    relative_rmse=1e-6
                )
            )

            # Fitness alone is an inlier-COUNT ratio at max_correspondence_
            # distance; two scans misaligned by a large fraction of that
            # distance can still report fitness near 1.0. Gate on the
            # actual residual (inlier_rmse) too (Physics Audit H3).
            rmse_ok = result.inlier_rmse <= 2.5 * self.config.voxel_size
            success = (result.fitness > self.config.icp_threshold) and rmse_ok
            return result.transformation, success, result.inlier_rmse

        except Exception as e:
            logger.error(f"ICP registration error: {e}")
            return np.eye(4), False, 0.0

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
        with self.lock:
            positions = [pose[:3, 3] for pose in self.poses]
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
            self.last_displacement = 0.0
            self._scans_since_reortho = 0
            self.map_bounds = {
                'min': np.full(3, np.inf),
                'max': np.full(3, -np.inf)
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
        has_bounds = bool(np.all(np.isfinite(self.map_bounds['min']))
                          and np.all(np.isfinite(self.map_bounds['max'])))
        bounds_min = self.map_bounds['min'] if has_bounds else np.zeros(3)
        bounds_max = self.map_bounds['max'] if has_bounds else np.zeros(3)

        return {
            'total_points': self.total_points,
            'total_scans': self.total_scans,
            'buffer_size': len(self.scan_buffer),
            # Accumulated registration-error proxy (grows with mission
            # length), NOT the last per-scan displacement -- see
            # last_displacement for that (Physics Audit: drift semantics).
            'drift_estimate': round(self.drift_estimate, 4),
            'last_displacement': round(self.last_displacement, 4),
            'current_position': {
                'x': round(self.current_pose[0, 3], 3),
                'y': round(self.current_pose[1, 3], 3),
                'z': round(self.current_pose[2, 3], 3)
            },
            'map_bounds': {
                'min': bounds_min.tolist(),
                'max': bounds_max.tolist()
            },
            'map_size': {
                'x': round(bounds_max[0] - bounds_min[0], 2),
                'y': round(bounds_max[1] - bounds_min[1], 2),
                'z': round(bounds_max[2] - bounds_min[2], 2)
            } if has_bounds else {'x': 0, 'y': 0, 'z': 0}
        }
