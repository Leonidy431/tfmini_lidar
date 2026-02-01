"""
Configuration Module for BlueOS LiDAR SLAM Extension
"""

import os
from dataclasses import dataclass, field
from typing import List


@dataclass
class LiDARConfig:
    """TFmini-S LiDAR Configuration"""
    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    timeout: float = 1.0
    frequency: int = 100  # Hz (1-1000)
    max_range: float = 12.0  # meters
    min_range: float = 0.1  # meters
    signal_threshold: int = 100  # Minimum signal strength


@dataclass
class SLAMConfig:
    """SLAM Algorithm Configuration"""
    voxel_size: float = 0.05  # Voxel size in meters
    max_depth: float = 12.0
    min_depth: float = 0.1
    icp_threshold: float = 0.6  # ICP convergence threshold (fitness)
    max_correspondence_distance: float = 0.5
    max_iterations: int = 50
    buffer_size: int = 1000  # Points buffer before processing


@dataclass
class NavigationConfig:
    """Navigation Profile Configuration"""
    profile_sample_rate: float = 10.0  # Hz
    waypoint_distance_threshold: float = 0.5  # meters
    heading_tolerance: float = 5.0  # degrees
    speed_limit: float = 1.0  # m/s
    obstacle_distance_warning: float = 2.0  # meters
    obstacle_distance_critical: float = 0.5  # meters


@dataclass
class ObjectDetectionConfig:
    """Object Detection Configuration"""
    enabled: bool = True
    min_object_size: float = 0.1  # meters
    max_object_size: float = 5.0  # meters
    detection_threshold: float = 0.3  # confidence
    clustering_eps: float = 0.2  # DBSCAN eps parameter
    clustering_min_samples: int = 5  # DBSCAN min_samples
    classes: List[str] = field(default_factory=lambda: [
        "obstacle",
        "wall",
        "pipe",
        "rock",
        "debris",
        "unknown"
    ])


@dataclass
class LocalizationConfig:
    """Localization Configuration"""
    enable_icp_refinement: bool = True
    confidence_threshold: float = 0.7
    map_matching_distance: float = 2.0
    update_frequency: int = 10  # Hz
    lost_threshold: int = 10  # Number of failed localizations before "lost"


@dataclass
class BlueOSConfig:
    """BlueOS Integration Configuration"""
    vehicle_host: str = "localhost"
    vehicle_port: int = 14550
    telemetry_enabled: bool = True
    mavlink_heartbeat_rate: int = 1  # Hz
    send_distance_sensor: bool = True
    send_obstacle_distance: bool = True


class Config:
    """Main Configuration Class"""

    # Environment settings
    DEBUG = os.getenv("DEBUG", "False").lower() == "true"

    # Directory paths
    DATA_DIR = os.getenv("DATA_DIR", "/app/data")
    MAPS_DIR = os.path.join(DATA_DIR, "maps")
    PROFILES_DIR = os.path.join(DATA_DIR, "profiles")
    LOGS_DIR = os.path.join(DATA_DIR, "logs")
    OBJECTS_DIR = os.path.join(DATA_DIR, "objects")

    # Web server settings
    WEB_HOST = os.getenv("WEB_HOST", "0.0.0.0")
    WEB_PORT = int(os.getenv("WEB_PORT", "5000"))

    # Component configurations
    lidar = LiDARConfig(
        port=os.getenv("LIDAR_PORT", "/dev/ttyUSB0")
    )
    slam = SLAMConfig()
    navigation = NavigationConfig()
    object_detection = ObjectDetectionConfig()
    localization = LocalizationConfig()
    blueos = BlueOSConfig()

    @classmethod
    def init_directories(cls):
        """Initialize all required directories"""
        for dir_path in [cls.MAPS_DIR, cls.PROFILES_DIR, cls.LOGS_DIR, cls.OBJECTS_DIR]:
            os.makedirs(dir_path, exist_ok=True)

    @classmethod
    def to_dict(cls) -> dict:
        """Export configuration as dictionary"""
        return {
            "debug": cls.DEBUG,
            "data_dir": cls.DATA_DIR,
            "web": {
                "host": cls.WEB_HOST,
                "port": cls.WEB_PORT
            },
            "lidar": {
                "port": cls.lidar.port,
                "baudrate": cls.lidar.baudrate,
                "frequency": cls.lidar.frequency,
                "max_range": cls.lidar.max_range,
                "min_range": cls.lidar.min_range
            },
            "slam": {
                "voxel_size": cls.slam.voxel_size,
                "icp_threshold": cls.slam.icp_threshold
            },
            "navigation": {
                "sample_rate": cls.navigation.profile_sample_rate,
                "obstacle_warning": cls.navigation.obstacle_distance_warning,
                "obstacle_critical": cls.navigation.obstacle_distance_critical
            },
            "object_detection": {
                "enabled": cls.object_detection.enabled,
                "classes": cls.object_detection.classes
            }
        }


# Initialize directories on module load
Config.init_directories()
