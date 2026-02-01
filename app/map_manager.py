"""
Map Manager Module

Handles saving, loading, and managing map data.
Supports multiple map formats and metadata storage.
"""

import os
import json
import shutil
import numpy as np
import open3d as o3d
from typing import List, Dict, Optional, Tuple
from datetime import datetime
from dataclasses import dataclass, field
import logging
import h5py

from app.config import Config

logger = logging.getLogger(__name__)


@dataclass
class MapMetadata:
    """Map metadata"""
    name: str
    created: datetime
    modified: datetime = None
    point_count: int = 0
    bounds_min: List[float] = field(default_factory=lambda: [0, 0, 0])
    bounds_max: List[float] = field(default_factory=lambda: [0, 0, 0])
    description: str = ""
    version: str = "1.0"
    format: str = "ply"  # ply, pcd, npy, h5
    tags: List[str] = field(default_factory=list)
    trajectory_count: int = 0
    total_scans: int = 0

    def to_dict(self) -> dict:
        return {
            'name': self.name,
            'created': self.created.isoformat(),
            'modified': self.modified.isoformat() if self.modified else self.created.isoformat(),
            'point_count': self.point_count,
            'bounds': {
                'min': self.bounds_min,
                'max': self.bounds_max
            },
            'size': {
                'x': round(self.bounds_max[0] - self.bounds_min[0], 2),
                'y': round(self.bounds_max[1] - self.bounds_min[1], 2),
                'z': round(self.bounds_max[2] - self.bounds_min[2], 2)
            },
            'description': self.description,
            'version': self.version,
            'format': self.format,
            'tags': self.tags,
            'trajectory_count': self.trajectory_count,
            'total_scans': self.total_scans
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'MapMetadata':
        bounds = data.get('bounds', {})
        return cls(
            name=data['name'],
            created=datetime.fromisoformat(data['created']),
            modified=datetime.fromisoformat(data['modified']) if data.get('modified') else None,
            point_count=data.get('point_count', 0),
            bounds_min=bounds.get('min', [0, 0, 0]),
            bounds_max=bounds.get('max', [0, 0, 0]),
            description=data.get('description', ''),
            version=data.get('version', '1.0'),
            format=data.get('format', 'ply'),
            tags=data.get('tags', []),
            trajectory_count=data.get('trajectory_count', 0),
            total_scans=data.get('total_scans', 0)
        )


class MapManager:
    """
    Map storage and retrieval manager

    Supports:
    - Multiple map formats (PLY, PCD, NPY, H5)
    - Metadata management
    - Map versioning
    - Trajectory storage
    """

    SUPPORTED_FORMATS = ['ply', 'pcd', 'npy', 'h5']

    def __init__(self, maps_dir: str = None):
        self.maps_dir = maps_dir or Config.MAPS_DIR
        os.makedirs(self.maps_dir, exist_ok=True)

    def save_map(self, name: str, points: np.ndarray,
                 description: str = "",
                 trajectory: np.ndarray = None,
                 format: str = 'ply',
                 tags: List[str] = None,
                 total_scans: int = 0) -> bool:
        """
        Save map to disk

        Args:
            name: Map name (used as directory name)
            points: Nx3 numpy array of points
            description: Map description
            trajectory: Optional Mx3 array of trajectory points
            format: Output format ('ply', 'pcd', 'npy', 'h5')
            tags: Optional list of tags
            total_scans: Number of scans used to create map
        """
        if format not in self.SUPPORTED_FORMATS:
            logger.error(f"Unsupported format: {format}")
            return False

        try:
            # Create map directory
            map_dir = os.path.join(self.maps_dir, name)
            os.makedirs(map_dir, exist_ok=True)

            # Save points
            points_file = self._save_points(map_dir, points, format)
            if not points_file:
                return False

            # Save trajectory if provided
            trajectory_count = 0
            if trajectory is not None and len(trajectory) > 0:
                traj_file = os.path.join(map_dir, 'trajectory.npy')
                np.save(traj_file, trajectory)
                trajectory_count = len(trajectory)

            # Create and save metadata
            metadata = MapMetadata(
                name=name,
                created=datetime.now(),
                modified=datetime.now(),
                point_count=len(points),
                bounds_min=points.min(axis=0).tolist() if len(points) > 0 else [0, 0, 0],
                bounds_max=points.max(axis=0).tolist() if len(points) > 0 else [0, 0, 0],
                description=description,
                format=format,
                tags=tags or [],
                trajectory_count=trajectory_count,
                total_scans=total_scans
            )

            metadata_file = os.path.join(map_dir, 'metadata.json')
            with open(metadata_file, 'w') as f:
                json.dump(metadata.to_dict(), f, indent=2)

            logger.info(f"Map '{name}' saved ({len(points)} points, format: {format})")
            return True

        except Exception as e:
            logger.error(f"Failed to save map: {e}")
            return False

    def _save_points(self, map_dir: str, points: np.ndarray, format: str) -> Optional[str]:
        """Save points in specified format"""
        try:
            if format == 'ply':
                filepath = os.path.join(map_dir, 'points.ply')
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points)
                o3d.io.write_point_cloud(filepath, pcd)

            elif format == 'pcd':
                filepath = os.path.join(map_dir, 'points.pcd')
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points)
                o3d.io.write_point_cloud(filepath, pcd)

            elif format == 'npy':
                filepath = os.path.join(map_dir, 'points.npy')
                np.save(filepath, points)

            elif format == 'h5':
                filepath = os.path.join(map_dir, 'points.h5')
                with h5py.File(filepath, 'w') as f:
                    f.create_dataset('points', data=points, compression='gzip')

            return filepath

        except Exception as e:
            logger.error(f"Failed to save points: {e}")
            return None

    def load_map(self, name: str) -> Optional[Tuple[np.ndarray, MapMetadata]]:
        """
        Load map from disk

        Returns:
            Tuple of (points_array, metadata) or None if failed
        """
        try:
            map_dir = os.path.join(self.maps_dir, name)

            if not os.path.exists(map_dir):
                logger.error(f"Map '{name}' not found")
                return None

            # Load metadata
            metadata_file = os.path.join(map_dir, 'metadata.json')
            if os.path.exists(metadata_file):
                with open(metadata_file, 'r') as f:
                    metadata = MapMetadata.from_dict(json.load(f))
            else:
                metadata = MapMetadata(name=name, created=datetime.now())

            # Load points
            points = self._load_points(map_dir, metadata.format)
            if points is None:
                return None

            logger.info(f"Map '{name}' loaded ({len(points)} points)")
            return points, metadata

        except Exception as e:
            logger.error(f"Failed to load map: {e}")
            return None

    def _load_points(self, map_dir: str, format: str = None) -> Optional[np.ndarray]:
        """Load points from map directory"""
        # Try different formats
        formats_to_try = [format] if format else self.SUPPORTED_FORMATS

        for fmt in formats_to_try:
            try:
                if fmt == 'ply':
                    filepath = os.path.join(map_dir, 'points.ply')
                    if os.path.exists(filepath):
                        pcd = o3d.io.read_point_cloud(filepath)
                        return np.asarray(pcd.points)

                elif fmt == 'pcd':
                    filepath = os.path.join(map_dir, 'points.pcd')
                    if os.path.exists(filepath):
                        pcd = o3d.io.read_point_cloud(filepath)
                        return np.asarray(pcd.points)

                elif fmt == 'npy':
                    filepath = os.path.join(map_dir, 'points.npy')
                    if os.path.exists(filepath):
                        return np.load(filepath)

                elif fmt == 'h5':
                    filepath = os.path.join(map_dir, 'points.h5')
                    if os.path.exists(filepath):
                        with h5py.File(filepath, 'r') as f:
                            return np.array(f['points'])

            except Exception as e:
                logger.debug(f"Failed to load as {fmt}: {e}")
                continue

        logger.error("Could not load points in any format")
        return None

    def load_trajectory(self, name: str) -> Optional[np.ndarray]:
        """Load trajectory for a map"""
        try:
            traj_file = os.path.join(self.maps_dir, name, 'trajectory.npy')
            if os.path.exists(traj_file):
                return np.load(traj_file)
            return None
        except Exception as e:
            logger.error(f"Failed to load trajectory: {e}")
            return None

    def list_maps(self) -> List[Dict]:
        """List all saved maps with metadata"""
        maps = []

        try:
            for map_name in os.listdir(self.maps_dir):
                map_dir = os.path.join(self.maps_dir, map_name)

                if not os.path.isdir(map_dir):
                    continue

                metadata_file = os.path.join(map_dir, 'metadata.json')

                if os.path.exists(metadata_file):
                    with open(metadata_file, 'r') as f:
                        maps.append(json.load(f))
                else:
                    # Create basic metadata from directory
                    maps.append({
                        'name': map_name,
                        'created': datetime.fromtimestamp(
                            os.path.getctime(map_dir)
                        ).isoformat(),
                        'point_count': 0,
                        'description': ''
                    })

        except Exception as e:
            logger.error(f"Failed to list maps: {e}")

        return sorted(maps, key=lambda x: x.get('created', ''), reverse=True)

    def delete_map(self, name: str) -> bool:
        """Delete a map"""
        try:
            map_dir = os.path.join(self.maps_dir, name)

            if os.path.exists(map_dir):
                shutil.rmtree(map_dir)
                logger.info(f"Map '{name}' deleted")
                return True

            logger.warning(f"Map '{name}' not found")
            return False

        except Exception as e:
            logger.error(f"Failed to delete map: {e}")
            return False

    def update_metadata(self, name: str, **kwargs) -> bool:
        """Update map metadata"""
        try:
            metadata_file = os.path.join(self.maps_dir, name, 'metadata.json')

            if not os.path.exists(metadata_file):
                logger.error(f"Map '{name}' metadata not found")
                return False

            with open(metadata_file, 'r') as f:
                data = json.load(f)

            # Update fields
            for key, value in kwargs.items():
                if key in data:
                    data[key] = value

            data['modified'] = datetime.now().isoformat()

            with open(metadata_file, 'w') as f:
                json.dump(data, f, indent=2)

            logger.info(f"Map '{name}' metadata updated")
            return True

        except Exception as e:
            logger.error(f"Failed to update metadata: {e}")
            return False

    def export_map(self, name: str, output_path: str, format: str = 'ply') -> bool:
        """Export map to specified path and format"""
        try:
            result = self.load_map(name)
            if result is None:
                return False

            points, _ = result

            if format == 'ply' or format == 'pcd':
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points)
                o3d.io.write_point_cloud(output_path, pcd)

            elif format == 'npy':
                np.save(output_path, points)

            elif format == 'csv':
                np.savetxt(output_path, points, delimiter=',',
                          header='x,y,z', comments='')

            elif format == 'xyz':
                np.savetxt(output_path, points, delimiter=' ')

            else:
                logger.error(f"Unsupported export format: {format}")
                return False

            logger.info(f"Map '{name}' exported to {output_path}")
            return True

        except Exception as e:
            logger.error(f"Failed to export map: {e}")
            return False

    def get_map_info(self, name: str) -> Optional[Dict]:
        """Get detailed map information"""
        try:
            map_dir = os.path.join(self.maps_dir, name)

            if not os.path.exists(map_dir):
                return None

            # Load metadata
            metadata_file = os.path.join(map_dir, 'metadata.json')
            if os.path.exists(metadata_file):
                with open(metadata_file, 'r') as f:
                    info = json.load(f)
            else:
                info = {'name': name}

            # Get file sizes
            info['files'] = {}
            for filename in os.listdir(map_dir):
                filepath = os.path.join(map_dir, filename)
                if os.path.isfile(filepath):
                    info['files'][filename] = os.path.getsize(filepath)

            info['total_size'] = sum(info['files'].values())

            return info

        except Exception as e:
            logger.error(f"Failed to get map info: {e}")
            return None

    def get_statistics(self) -> dict:
        """Get map storage statistics"""
        maps = self.list_maps()
        total_points = sum(m.get('point_count', 0) for m in maps)

        total_size = 0
        for map_info in maps:
            map_dir = os.path.join(self.maps_dir, map_info['name'])
            if os.path.exists(map_dir):
                for f in os.listdir(map_dir):
                    filepath = os.path.join(map_dir, f)
                    if os.path.isfile(filepath):
                        total_size += os.path.getsize(filepath)

        return {
            'map_count': len(maps),
            'total_points': total_points,
            'total_size_bytes': total_size,
            'total_size_mb': round(total_size / (1024 * 1024), 2),
            'maps_dir': self.maps_dir
        }
