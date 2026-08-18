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
    # 12.0 m is the TFmini-S IN-AIR spec. Underwater at 850nm, two-way Beer-
    # Lambert absorption (alpha ~= 4.3 /m in clear water) makes any return
    # past a few meters physically impossible -- readings beyond this are
    # guaranteed backscatter/multipath artifacts, not real targets (Physics
    # Audit C2). 4.0 m is a conservative compromise for moderate-clarity
    # water; tune down for turbid water, up only for bench/air testing.
    # Override via LIDAR_MAX_RANGE_M.
    max_range: float = float(os.getenv("LIDAR_MAX_RANGE_M", "4.0"))
    min_range: float = 0.1  # meters
    signal_threshold: int = 100  # Minimum signal strength
    # ToF range assumes propagation at c/n. n_air ~= 1.0003 (no-op); at
    # 850nm n_water ~= 1.333 (fresh) / 1.339 (sea). Uncorrected, underwater
    # ranges read ~33% long (Physics Audit C1). Set to 1.0 for bench/air
    # testing via LIDAR_MEDIUM_INDEX.
    medium_refractive_index: float = float(os.getenv("LIDAR_MEDIUM_INDEX", "1.333"))


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
    motion_threshold: float = 0.01  # Min centroid displacement (m) to run ICP


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
class ScannerConfig:
    """3D Object Scanner (orbit scan) Configuration"""
    orbit_radius: float = 3.0  # meters, default carrier-to-center distance
    angular_resolution_deg: float = 5.0  # coverage bin size (72 bins per ring)
    layer_height: float = 0.5  # meters between vertical scan rings
    min_distance: float = 0.2  # meters, reject readings closer than this
    max_points: int = 500000  # hard cap on accumulated cloud
    signal_threshold: int = 100  # minimum signal strength to accept
    min_coverage_complete: float = 0.95  # ring considered complete above this
    progress_emit_every: int = 25  # WebSocket progress cadence (accepted points)


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


@dataclass
class MAVLinkAttitudeConfig:
    """MAVLink 3D Attitude Configuration (Physics Audit D1).

    Disabled by default: with no flight controller wired up, the app keeps
    the existing 1D compass-heading-only beam projection (Physics Audit
    C4). Enabling this lets full roll/pitch/yaw correct the beam
    projection when the ROV is not level.
    """
    enabled: bool = os.getenv("ENABLE_MAVLINK_3D_ATTITUDE", "false").lower() == "true"
    connection_string: str = os.getenv("MAVLINK_CONNECTION_STRING", "udp:127.0.0.1:14550")
    baudrate: int = int(os.getenv("MAVLINK_BAUDRATE", "115200"))
    timeout_s: float = float(os.getenv("MAVLINK_TIMEOUT_S", "1.0"))


@dataclass
class MultipathConfig:
    """Multipath/Turbidity Detection Configuration (Physics Audit D2).

    Disabled by default: the 2-component mixture model needs a warm-up
    window of clean readings before it can discriminate scattered-light
    returns, and is only worth the extra CPU in turbid water.
    """
    enabled: bool = os.getenv("ENABLE_MULTIPATH_DETECTION", "false").lower() == "true"
    window_size: int = int(os.getenv("MULTIPATH_WINDOW_SIZE", "50"))
    min_samples: int = int(os.getenv("MULTIPATH_MIN_SAMPLES", "20"))
    signal_strength_threshold: int = int(os.getenv("MULTIPATH_SIGNAL_THRESHOLD", "100"))


@dataclass
class EnvironmentalCorrectionConfig:
    """Depth-Dependent Refractive Index + Temperature Compensation
    (Physics Audit D3/D4).

    Disabled by default: default coefficients (b=c=0, no temperature
    slope disabled) are a strict no-op over the driver's existing
    constant-n correction (Physics Audit C1) until D3/D4 P9 lab/field
    calibration provides fitted coefficients for the actual deployment.
    """
    enabled: bool = os.getenv("ENABLE_DEPTH_CORRECTION", "false").lower() == "true"
    depth_coeff_a: float = float(os.getenv("DEPTH_REFRACTIVE_A", "1.333"))
    depth_coeff_b: float = float(os.getenv("DEPTH_REFRACTIVE_B", "0.0"))
    depth_coeff_c: float = float(os.getenv("DEPTH_REFRACTIVE_C", "0.0"))
    temperature_enabled: bool = os.getenv("ENABLE_TEMPERATURE_CORRECTION", "false").lower() == "true"
    temperature_ref_c: float = float(os.getenv("TEMPERATURE_REF_C", "20.0"))
    temperature_slope_per_degree: float = float(os.getenv("TEMPERATURE_SLOPE", "0.0005"))


@dataclass
class EKFConfig:
    """9-DOF Position+Attitude EKF Fusion Configuration (Physics Audit D8).

    Disabled by default; depends on MAVLinkAttitudeConfig for attitude
    measurements to be useful (position-only fusion still works without
    it, degrading gracefully to a constant-velocity position filter).
    """
    enabled: bool = os.getenv("ENABLE_EKF_FUSION", "false").lower() == "true"
    process_noise_position: float = float(os.getenv("EKF_PROCESS_NOISE_POSITION", "0.01"))
    process_noise_attitude: float = float(os.getenv("EKF_PROCESS_NOISE_ATTITUDE", "0.05"))
    process_noise_velocity: float = float(os.getenv("EKF_PROCESS_NOISE_VELOCITY", "0.1"))
    measurement_noise_position: float = float(os.getenv("EKF_MEASUREMENT_NOISE_POSITION", "0.15"))
    measurement_noise_attitude: float = float(os.getenv("EKF_MEASUREMENT_NOISE_ATTITUDE", "0.02"))


class Config:
    """Main Configuration Class"""

    # Environment settings
    DEBUG = os.getenv("DEBUG", "False").lower() == "true"

    # The Werkzeug interactive debugger allows arbitrary code execution from
    # any client that can trigger an unhandled exception. It must never be
    # derived from DEBUG alone (DEBUG is also used for verbose app logging,
    # which operators reasonably enable for field troubleshooting while
    # WEB_HOST is 0.0.0.0). Requires an explicit, separate opt-in AND is
    # still refused unless the server only listens on loopback.
    # (Blind Spot Audit R3, R3-SEC-1)
    ALLOW_WERKZEUG_DEBUGGER = os.getenv("ALLOW_WERKZEUG_DEBUGGER", "False").lower() == "true"

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
    scanner = ScannerConfig()
    blueos = BlueOSConfig()
    mavlink_attitude = MAVLinkAttitudeConfig()
    multipath = MultipathConfig()
    environmental_correction = EnvironmentalCorrectionConfig()
    ekf = EKFConfig()

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
                "min_range": cls.lidar.min_range,
                "medium_refractive_index": cls.lidar.medium_refractive_index
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
            },
            "scanner": {
                "orbit_radius": cls.scanner.orbit_radius,
                "angular_resolution_deg": cls.scanner.angular_resolution_deg,
                "layer_height": cls.scanner.layer_height
            },
            "deferred_decisions": {
                "mavlink_3d_attitude_enabled": cls.mavlink_attitude.enabled,
                "multipath_detection_enabled": cls.multipath.enabled,
                "depth_correction_enabled": cls.environmental_correction.enabled,
                "ekf_fusion_enabled": cls.ekf.enabled
            }
        }


# Initialize directories on module load
Config.init_directories()
