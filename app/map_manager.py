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
from app.security import safe_join, validate_path_component

logger = logging.getLogger(__name__)


def _fsync_path(path: str):
    """Flush a file's contents to disk. Best-effort: a platform without
    fsync semantics must not break the save."""
    try:
        fd = os.open(path, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except OSError as e:
        logger.debug(f"fsync of {path} skipped: {e}")


def _fsync_dir(dirpath: str):
    """Flush a directory entry so a rename/create is durable across power
    loss (ext4 can otherwise lose the rename itself). Blind Spot Audit R2
    domain 17 #2."""
    try:
        fd = os.open(dirpath, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except OSError as e:
        logger.debug(f"fsync of dir {dirpath} skipped: {e}")


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

        if not validate_path_component(name):
            logger.error(f"Invalid map name: {name}")
            return False

        # Build the whole map in a staging directory, fsync every file, then
        # atomically swap it into place (Blind Spot Audit R2 domain 17 #1/#2/
        # #6). On an ROV, power loss is routine; the previous good map must
        # survive a save that is cut off partway, and a half-written map must
        # never appear in list_maps().
        staging_dir = None
        try:
            map_dir = safe_join(self.maps_dir, name)
            if map_dir is None:
                logger.error(f"Path traversal blocked for map: {name}")
                return False

            staging_dir = map_dir + '.staging'
            if os.path.exists(staging_dir):
                shutil.rmtree(staging_dir)
            os.makedirs(staging_dir)

            # Save points; abort the whole save if the point-cloud write
            # fails, so metadata never claims points that aren't on disk
            # (domain 17 #3).
            points_file = self._save_points(staging_dir, points, format)
            if not points_file:
                return False

            trajectory_count = 0
            if trajectory is not None and len(trajectory) > 0:
                traj_file = os.path.join(staging_dir, 'trajectory.npy')
                with open(traj_file, 'wb') as f:
                    np.save(f, trajectory)
                    f.flush()
                    os.fsync(f.fileno())
                trajectory_count = len(trajectory)

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

            metadata_file = os.path.join(staging_dir, 'metadata.json')
            with open(metadata_file, 'w') as f:
                json.dump(metadata.to_dict(), f, indent=2)
                f.flush()
                os.fsync(f.fileno())

            _fsync_dir(staging_dir)

            # Atomic swap: remove any prior map only once the new one is fully
            # written and fsynced. A power cut before this point leaves the old
            # map untouched and only an orphan .staging dir (cleaned below).
            if os.path.exists(map_dir):
                shutil.rmtree(map_dir)
            os.replace(staging_dir, map_dir)
            staging_dir = None
            _fsync_dir(self.maps_dir)

            logger.info(f"Map '{name}' saved ({len(points)} points, format: {format})")
            return True

        except Exception as e:
            logger.error(f"Failed to save map: {e}")
            return False
        finally:
            if staging_dir is not None and os.path.exists(staging_dir):
                shutil.rmtree(staging_dir, ignore_errors=True)

    def _save_points(self, map_dir: str, points: np.ndarray, format: str) -> Optional[str]:
        """Save points in the given format via a temp file + atomic rename.

        Each format writes to a hidden temp file that keeps the real
        extension (so Open3D can still infer the format), fsyncs it, then
        os.replace()s it into place. Returns the final path, or None if the
        underlying writer reported/raised a failure (Blind Spot Audit R2
        domain 17 #1/#3)."""
        try:
            if format in ('ply', 'pcd'):
                filepath = os.path.join(map_dir, f'points.{format}')
                tmp = os.path.join(map_dir, f'.points.tmp.{format}')
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points)
                if not o3d.io.write_point_cloud(tmp, pcd):
                    logger.error(f"Open3D failed to write {format} point cloud")
                    return None

            elif format == 'npy':
                filepath = os.path.join(map_dir, 'points.npy')
                tmp = os.path.join(map_dir, '.points.tmp.npy')
                with open(tmp, 'wb') as f:
                    np.save(f, points)
                    f.flush()
                    os.fsync(f.fileno())

            elif format == 'h5':
                filepath = os.path.join(map_dir, 'points.h5')
                tmp = os.path.join(map_dir, '.points.tmp.h5')
                with h5py.File(tmp, 'w') as f:
                    f.create_dataset('points', data=points, compression='gzip')
            else:
                logger.error(f"Unsupported format in _save_points: {format}")
                return None

            _fsync_path(tmp)
            os.replace(tmp, filepath)
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
        if not validate_path_component(name):
            logger.error(f"Invalid map name: {name}")
            return None

        try:
            map_dir = safe_join(self.maps_dir, name)
            if map_dir is None:
                logger.error(f"Path traversal blocked for map: {name}")
                return None

            if not os.path.exists(map_dir):
                logger.error(f"Map '{name}' not found")
                return None

            # Load metadata. A corrupt/half-written metadata.json (routine
            # after power loss) must not make an otherwise-intact points file
            # unloadable -- fall through to synthesized metadata instead
            # (Blind Spot Audit R2 domain 17 #5).
            metadata_file = os.path.join(map_dir, 'metadata.json')
            metadata = None
            if os.path.exists(metadata_file):
                try:
                    with open(metadata_file, 'r') as f:
                        metadata = MapMetadata.from_dict(json.load(f))
                except (json.JSONDecodeError, KeyError, ValueError, OSError) as e:
                    logger.warning(f"Map '{name}' metadata unreadable ({e}); "
                                   f"synthesizing from directory")
            metadata_synthesized = metadata is None
            if metadata_synthesized:
                metadata = MapMetadata(name=name, created=datetime.now())

            # Load points. When metadata was synthesized we don't know the
            # real format, so try all of them rather than trusting the
            # placeholder default (domain 17 #5 -- an npy map with a corrupt
            # metadata.json must still load).
            points = self._load_points(
                map_dir, None if metadata_synthesized else metadata.format)
            if points is None:
                return None

            # Validate geometry before handing the map to localization: a
            # truncated point file loads as an empty/partial or wrong-shape
            # array without raising, and navigating against a phantom map is
            # worse than reporting a load failure (domain 17 #4).
            if points.ndim != 2 or points.shape[1] != 3:
                logger.error(f"Map '{name}' points have invalid shape "
                             f"{points.shape}; refusing to load")
                return None
            if metadata.point_count > 0 and len(points) < metadata.point_count * 0.5:
                logger.error(f"Map '{name}' has {len(points)} points but "
                             f"metadata claims {metadata.point_count}; likely "
                             f"truncated, refusing to load")
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
        if not validate_path_component(name):
            return None

        try:
            map_dir = safe_join(self.maps_dir, name)
            if map_dir is None:
                return None
            traj_file = os.path.join(map_dir, 'trajectory.npy')
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
            entries = os.listdir(self.maps_dir)
        except OSError as e:
            logger.error(f"Failed to list maps: {e}")
            return maps

        for map_name in entries:
            # Skip staging/temp directories left by an interrupted save so a
            # half-written map is never advertised as loadable (Blind Spot
            # Audit R2 domain 17 #6).
            if map_name.endswith('.staging'):
                continue
            map_dir = os.path.join(self.maps_dir, map_name)
            if not os.path.isdir(map_dir):
                continue

            # Per-entry error isolation: one corrupt metadata.json must not
            # blank the entire listing and hide every healthy map after it
            # (domain 17 #8).
            try:
                metadata_file = os.path.join(map_dir, 'metadata.json')
                if os.path.exists(metadata_file):
                    with open(metadata_file, 'r') as f:
                        maps.append(json.load(f))
                else:
                    maps.append({
                        'name': map_name,
                        'created': datetime.fromtimestamp(
                            os.path.getctime(map_dir)
                        ).isoformat(),
                        'point_count': 0,
                        'description': '',
                        'status': 'incomplete'
                    })
            except (json.JSONDecodeError, OSError) as e:
                logger.warning(f"Skipping unreadable map '{map_name}': {e}")
                continue

        return sorted(maps, key=lambda x: x.get('created', ''), reverse=True)

    def delete_map(self, name: str) -> bool:
        """Delete a map"""
        if not validate_path_component(name):
            logger.error(f"Invalid map name: {name}")
            return False

        try:
            map_dir = safe_join(self.maps_dir, name)
            if map_dir is None:
                logger.error(f"Path traversal blocked for map: {name}")
                return False

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
        if not validate_path_component(name):
            logger.error(f"Invalid map name: {name}")
            return False

        try:
            map_dir = safe_join(self.maps_dir, name)
            if map_dir is None:
                return False
            metadata_file = os.path.join(map_dir, 'metadata.json')

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
        if not validate_path_component(name):
            return None

        try:
            map_dir = safe_join(self.maps_dir, name)
            if map_dir is None:
                return None

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
