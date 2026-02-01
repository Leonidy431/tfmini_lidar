"""
BlueOS LiDAR SLAM Extension - Main Application

Integrates all modules and provides REST API for web interface.
"""

import logging
import threading
import time
import numpy as np
from datetime import datetime
from typing import Dict, List, Optional

from flask import Flask, jsonify, request, render_template, send_from_directory
from flask_cors import CORS
from flask_socketio import SocketIO, emit

from app.config import Config
from app.lidar_driver import TFminiSDriver, LiDARReading
from app.slam_engine import SLAMEngine
from app.localization import LocalizationEngine
from app.map_manager import MapManager
from app.profile_recorder import ProfileRecorder, ProfileNavigator
from app.object_detection import ObjectDetector

# Configure logging
logging.basicConfig(
    level=logging.DEBUG if Config.DEBUG else logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler(f"{Config.LOGS_DIR}/app.log")
    ]
)
logger = logging.getLogger(__name__)


class LiDARSLAMApplication:
    """
    Main application class

    Manages all components:
    - LiDAR driver
    - SLAM engine
    - Localization
    - Navigation profile recording/playback
    - Object detection
    """

    # Operating modes
    MODE_IDLE = 'idle'
    MODE_MAPPING = 'mapping'
    MODE_LOCALIZING = 'localizing'
    MODE_NAVIGATING = 'navigating'
    MODE_RECORDING = 'recording'

    def __init__(self):
        self.config = Config

        # Initialize components
        self.driver: Optional[TFminiSDriver] = None
        self.slam_engine = SLAMEngine(self.config.slam)
        self.localization_engine = LocalizationEngine(self.config.localization)
        self.map_manager = MapManager(self.config.MAPS_DIR)
        self.profile_recorder = ProfileRecorder(self.config.PROFILES_DIR)
        self.profile_navigator = ProfileNavigator(self.config.navigation)
        self.object_detector = ObjectDetector(self.config.object_detection)

        # State
        self.mode = self.MODE_IDLE
        self.is_running = False
        self.current_heading = 0.0  # Will be updated from MAVLink if available

        # Real-time data
        self.last_reading: Optional[LiDARReading] = None
        self.readings_per_second = 0
        self._reading_count = 0
        self._last_rate_check = time.time()

        # WebSocket callback
        self.websocket_callback = None

    def initialize(self) -> bool:
        """Initialize all components"""
        logger.info("=" * 60)
        logger.info("BlueOS LiDAR SLAM Extension v1.0.0")
        logger.info("=" * 60)

        # Initialize LiDAR driver
        self.driver = TFminiSDriver(
            port=self.config.lidar.port,
            baudrate=self.config.lidar.baudrate,
            timeout=self.config.lidar.timeout
        )

        if not self.driver.connect():
            logger.error("Failed to connect to LiDAR")
            return False

        # Set frequency
        self.driver.set_framerate(self.config.lidar.frequency)

        # Add data callback
        self.driver.add_callback(self._on_lidar_reading)

        logger.info("Application initialized successfully")
        return True

    def start(self) -> bool:
        """Start the application"""
        if self.is_running:
            logger.warning("Application already running")
            return True

        if not self.driver:
            if not self.initialize():
                return False

        try:
            self.driver.start()
            self.is_running = True
            logger.info("Application started")
            return True
        except Exception as e:
            logger.error(f"Failed to start: {e}")
            return False

    def stop(self):
        """Stop the application"""
        self.is_running = False
        self.mode = self.MODE_IDLE

        if self.driver:
            self.driver.stop()

        logger.info("Application stopped")

    def set_mode(self, mode: str) -> bool:
        """Set operating mode"""
        valid_modes = [self.MODE_IDLE, self.MODE_MAPPING, self.MODE_LOCALIZING,
                      self.MODE_NAVIGATING, self.MODE_RECORDING]

        if mode not in valid_modes:
            logger.error(f"Invalid mode: {mode}")
            return False

        # Handle mode transitions
        if self.mode == self.MODE_RECORDING and mode != self.MODE_RECORDING:
            # Stop recording
            self.profile_recorder.stop_recording()

        if self.mode == self.MODE_NAVIGATING and mode != self.MODE_NAVIGATING:
            # Stop navigation
            self.profile_navigator.stop_navigation()

        self.mode = mode
        logger.info(f"Mode changed to: {mode}")
        return True

    def _on_lidar_reading(self, reading: LiDARReading):
        """Callback for new LiDAR readings"""
        if not reading.valid:
            return

        self.last_reading = reading
        self._reading_count += 1

        # Calculate readings per second
        now = time.time()
        if now - self._last_rate_check >= 1.0:
            self.readings_per_second = self._reading_count
            self._reading_count = 0
            self._last_rate_check = now

        # Convert distance to 3D point (assuming forward-facing sensor)
        # In real application, this would use IMU data for proper transformation
        x = reading.distance
        y = 0.0
        z = 0.0

        # Get current position estimate
        position = self._get_current_position()

        # Transform point to world frame (simplified)
        world_x = position[0] + x * np.cos(np.radians(self.current_heading))
        world_y = position[1] + x * np.sin(np.radians(self.current_heading))
        world_z = position[2] + z

        # Process based on mode
        if self.mode == self.MODE_MAPPING:
            self.slam_engine.add_point(world_x, world_y, world_z)

        elif self.mode == self.MODE_LOCALIZING:
            result = self.localization_engine.add_point(world_x, world_y, world_z)
            if result:
                self._broadcast_localization(result)

        elif self.mode == self.MODE_RECORDING:
            self.profile_recorder.add_waypoint(
                position=(world_x, world_y, world_z),
                heading=self.current_heading,
                distance_reading=reading.distance,
                signal_strength=reading.signal_strength
            )

        elif self.mode == self.MODE_NAVIGATING:
            guidance = self.profile_navigator.update(
                current_position=position,
                current_heading=self.current_heading,
                current_distance_reading=reading.distance
            )
            self._broadcast_navigation(guidance)

        # Object detection (always active if enabled)
        if self.config.object_detection.enabled:
            detection = self.object_detector.process_reading(
                distance=reading.distance,
                strength=reading.signal_strength,
                position=(world_x, world_y, world_z),
                timestamp=reading.timestamp
            )
            if detection:
                self._broadcast_detection(detection)

        # Broadcast reading via WebSocket
        self._broadcast_reading(reading)

    def _get_current_position(self) -> tuple:
        """Get current position estimate"""
        if self.mode == self.MODE_LOCALIZING:
            return self.localization_engine.get_position()
        elif self.mode == self.MODE_MAPPING:
            return self.slam_engine.get_current_position()
        else:
            return (0.0, 0.0, 0.0)

    def _broadcast_reading(self, reading: LiDARReading):
        """Broadcast reading via WebSocket"""
        if self.websocket_callback:
            self.websocket_callback('lidar_reading', reading.to_dict())

    def _broadcast_localization(self, result: dict):
        """Broadcast localization result"""
        if self.websocket_callback:
            self.websocket_callback('localization', result)

    def _broadcast_navigation(self, guidance: dict):
        """Broadcast navigation guidance"""
        if self.websocket_callback:
            self.websocket_callback('navigation', guidance)

    def _broadcast_detection(self, detection):
        """Broadcast object detection"""
        if self.websocket_callback:
            self.websocket_callback('detection', detection.to_dict())

    def get_status(self) -> Dict:
        """Get comprehensive application status"""
        lidar_stats = self.driver.get_statistics() if self.driver else {}
        slam_stats = self.slam_engine.get_statistics()
        localization_stats = self.localization_engine.get_statistics()
        detection_stats = self.object_detector.get_statistics()
        recording_status = self.profile_recorder.get_recording_status()
        navigation_status = self.profile_navigator.get_status()

        return {
            'running': self.is_running,
            'mode': self.mode,
            'readings_per_second': self.readings_per_second,
            'last_reading': self.last_reading.to_dict() if self.last_reading else None,
            'lidar': lidar_stats,
            'slam': slam_stats,
            'localization': localization_stats,
            'object_detection': detection_stats,
            'recording': recording_status,
            'navigation': navigation_status,
            'config': self.config.to_dict()
        }


# Create Flask application
app = Flask(__name__,
           template_folder='web/templates',
           static_folder='web/static')
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

# Create application instance
lidar_app = LiDARSLAMApplication()


# WebSocket callback
def ws_emit(event: str, data: dict):
    socketio.emit(event, data)


lidar_app.websocket_callback = ws_emit


# ============ REST API Routes ============

@app.route('/')
def index():
    """Serve main page"""
    return render_template('index.html')


@app.route('/api/register_service')
def register_service():
    """BlueOS service registration"""
    return jsonify({
        "name": "LiDAR SLAM Module",
        "description": "Real-time mapping, navigation and object detection with TFmini-S LiDAR",
        "version": "1.0.0",
        "company": "BlueOS Extensions",
        "webpage": "/",
        "api": "/api/"
    })


@app.route('/api/status')
def get_status():
    """Get application status"""
    return jsonify(lidar_app.get_status())


@app.route('/api/start', methods=['POST'])
def start():
    """Start application"""
    success = lidar_app.start()
    return jsonify({'success': success}), 200 if success else 500


@app.route('/api/stop', methods=['POST'])
def stop():
    """Stop application"""
    lidar_app.stop()
    return jsonify({'success': True})


@app.route('/api/mode/<mode>', methods=['POST'])
def set_mode(mode: str):
    """Set operating mode"""
    success = lidar_app.set_mode(mode)
    return jsonify({'success': success, 'mode': lidar_app.mode}), 200 if success else 400


# ============ Mapping API ============

@app.route('/api/mapping/start', methods=['POST'])
def start_mapping():
    """Start mapping mode"""
    lidar_app.slam_engine.clear()
    lidar_app.set_mode(LiDARSLAMApplication.MODE_MAPPING)
    return jsonify({'success': True, 'mode': 'mapping'})


@app.route('/api/mapping/stop', methods=['POST'])
def stop_mapping():
    """Stop mapping"""
    lidar_app.set_mode(LiDARSLAMApplication.MODE_IDLE)
    return jsonify({
        'success': True,
        'statistics': lidar_app.slam_engine.get_statistics()
    })


@app.route('/api/mapping/clear', methods=['POST'])
def clear_mapping():
    """Clear current map"""
    lidar_app.slam_engine.clear()
    return jsonify({'success': True})


@app.route('/api/mapping/statistics')
def mapping_statistics():
    """Get mapping statistics"""
    return jsonify(lidar_app.slam_engine.get_statistics())


@app.route('/api/mapping/points')
def get_map_points():
    """Get current map points (downsampled for web)"""
    points = lidar_app.slam_engine.get_map_downsampled(voxel_size=0.1)
    if points is None:
        return jsonify({'points': []})
    return jsonify({'points': points.tolist(), 'count': len(points)})


@app.route('/api/mapping/trajectory')
def get_trajectory():
    """Get mapping trajectory"""
    trajectory = lidar_app.slam_engine.get_trajectory()
    return jsonify({'trajectory': trajectory.tolist(), 'count': len(trajectory)})


# ============ Maps API ============

@app.route('/api/maps')
def list_maps():
    """List all saved maps"""
    return jsonify(lidar_app.map_manager.list_maps())


@app.route('/api/maps/<name>')
def get_map_info(name: str):
    """Get map information"""
    info = lidar_app.map_manager.get_map_info(name)
    if info:
        return jsonify(info)
    return jsonify({'error': 'Map not found'}), 404


@app.route('/api/maps/<name>/save', methods=['POST'])
def save_map(name: str):
    """Save current map"""
    data = request.json or {}
    description = data.get('description', '')
    tags = data.get('tags', [])

    points = lidar_app.slam_engine.get_map()
    if points is None or len(points) == 0:
        return jsonify({'success': False, 'error': 'No map data'}), 400

    trajectory = lidar_app.slam_engine.get_trajectory()
    stats = lidar_app.slam_engine.get_statistics()

    success = lidar_app.map_manager.save_map(
        name=name,
        points=points,
        description=description,
        trajectory=trajectory if len(trajectory) > 0 else None,
        tags=tags,
        total_scans=stats.get('total_scans', 0)
    )

    return jsonify({'success': success}), 200 if success else 500


@app.route('/api/maps/<name>/load', methods=['POST'])
def load_map(name: str):
    """Load map for localization"""
    result = lidar_app.map_manager.load_map(name)
    if result is None:
        return jsonify({'success': False, 'error': 'Map not found'}), 404

    points, metadata = result

    # Set as reference for localization
    lidar_app.localization_engine.set_reference_map(points)
    lidar_app.set_mode(LiDARSLAMApplication.MODE_LOCALIZING)

    return jsonify({
        'success': True,
        'point_count': len(points),
        'metadata': metadata.to_dict()
    })


@app.route('/api/maps/<name>/delete', methods=['DELETE'])
def delete_map(name: str):
    """Delete a map"""
    success = lidar_app.map_manager.delete_map(name)
    return jsonify({'success': success}), 200 if success else 404


# ============ Localization API ============

@app.route('/api/localization/position')
def get_position():
    """Get current position"""
    stats = lidar_app.localization_engine.get_statistics()
    return jsonify(stats)


@app.route('/api/localization/reset', methods=['POST'])
def reset_localization():
    """Reset localization"""
    lidar_app.localization_engine.reset()
    return jsonify({'success': True})


# ============ Profile Recording API ============

@app.route('/api/profiles')
def list_profiles():
    """List all navigation profiles"""
    return jsonify(lidar_app.profile_recorder.list_profiles())


@app.route('/api/profiles/<name>/record/start', methods=['POST'])
def start_recording(name: str):
    """Start recording a navigation profile"""
    data = request.json or {}
    description = data.get('description', '')

    success = lidar_app.profile_recorder.start_recording(name, description)
    if success:
        lidar_app.set_mode(LiDARSLAMApplication.MODE_RECORDING)

    return jsonify({'success': success}), 200 if success else 400


@app.route('/api/profiles/record/stop', methods=['POST'])
def stop_recording():
    """Stop recording"""
    profile = lidar_app.profile_recorder.stop_recording()
    if profile:
        lidar_app.profile_recorder.save_profile(profile)

    lidar_app.set_mode(LiDARSLAMApplication.MODE_IDLE)

    return jsonify({
        'success': profile is not None,
        'profile': profile.to_dict() if profile else None
    })


@app.route('/api/profiles/<name>/navigate/start', methods=['POST'])
def start_navigation(name: str):
    """Start navigating with a profile"""
    profile = lidar_app.profile_recorder.load_profile(name)
    if not profile:
        return jsonify({'success': False, 'error': 'Profile not found'}), 404

    success = lidar_app.profile_navigator.start_navigation(profile)
    if success:
        lidar_app.set_mode(LiDARSLAMApplication.MODE_NAVIGATING)

    return jsonify({'success': success})


@app.route('/api/profiles/navigate/stop', methods=['POST'])
def stop_navigation():
    """Stop navigation"""
    lidar_app.profile_navigator.stop_navigation()
    lidar_app.set_mode(LiDARSLAMApplication.MODE_IDLE)
    return jsonify({'success': True})


@app.route('/api/profiles/<name>/delete', methods=['DELETE'])
def delete_profile(name: str):
    """Delete a profile"""
    success = lidar_app.profile_recorder.delete_profile(name)
    return jsonify({'success': success}), 200 if success else 404


# ============ Object Detection API ============

@app.route('/api/objects')
def get_objects():
    """Get all detected objects"""
    return jsonify({
        'objects': lidar_app.object_detector.get_objects(),
        'statistics': lidar_app.object_detector.get_statistics()
    })


@app.route('/api/objects/nearby')
def get_nearby_objects():
    """Get objects near a position"""
    x = float(request.args.get('x', 0))
    y = float(request.args.get('y', 0))
    z = float(request.args.get('z', 0))
    radius = float(request.args.get('radius', 5.0))

    objects = lidar_app.object_detector.get_nearby_objects((x, y, z), radius)
    return jsonify({'objects': objects})


@app.route('/api/objects/clear', methods=['POST'])
def clear_objects():
    """Clear detected objects"""
    lidar_app.object_detector.clear_objects()
    return jsonify({'success': True})


@app.route('/api/objects/save', methods=['POST'])
def save_objects():
    """Save detected objects"""
    success = lidar_app.object_detector.save_objects()
    return jsonify({'success': success})


# ============ WebSocket Events ============

@socketio.on('connect')
def on_connect():
    logger.info("WebSocket client connected")
    emit('status', lidar_app.get_status())


@socketio.on('disconnect')
def on_disconnect():
    logger.info("WebSocket client disconnected")


@socketio.on('get_status')
def on_get_status():
    emit('status', lidar_app.get_status())


@socketio.on('get_map_points')
def on_get_map_points():
    points = lidar_app.slam_engine.get_map_downsampled(voxel_size=0.1)
    if points is not None:
        emit('map_points', {'points': points.tolist()})


# ============ Main Entry Point ============

def main():
    """Main entry point"""
    logger.info("Starting BlueOS LiDAR SLAM Extension...")

    # Initialize application
    if not lidar_app.initialize():
        logger.warning("LiDAR initialization failed - running in demo mode")

    # Run Flask server with SocketIO
    socketio.run(
        app,
        host=Config.WEB_HOST,
        port=Config.WEB_PORT,
        debug=Config.DEBUG,
        allow_unsafe_werkzeug=True
    )


if __name__ == '__main__':
    main()
