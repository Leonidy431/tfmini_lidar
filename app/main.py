"""
BlueOS LiDAR SLAM Extension - Main Application

Integrates all modules and provides REST API for web interface.
"""

import logging
import os
import queue
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
from app.scanner_3d import Scanner3D
from app.data_quality import DataQualityValidator
from app.security import (
    require_auth, require_rate_limit, init_default_token,
    is_public_route, validate_path_component, validate_token
)

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
    MODE_SCANNING = 'scanning'  # 3D object scan: carrier orbits the target

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
        self.scanner = Scanner3D(self.config.scanner)

        # Data quality validation (Rule 4: IQR + Z-score outlier filtering)
        self.data_quality = DataQualityValidator(
            signal_threshold=self.config.lidar.signal_threshold,
            max_range=self.config.lidar.max_range,
            min_range=self.config.lidar.min_range
        )

        # Thread safety locks
        self._state_lock = threading.RLock()
        self._reading_lock = threading.Lock()

        # State (protected by _state_lock)
        self._mode = self.MODE_IDLE
        self._is_running = False
        # Compass heading in degrees, clockwise-positive from North (NED/
        # MAVLink convention). No attitude source is wired up in this build
        # (Physics Audit C3/D1) -- the value stays at the placeholder 0.0.
        # heading_ever_set tracks whether anything has ever updated it, so
        # health reporting can surface "no heading source" honestly instead
        # of silently mapping every reading onto a single world axis.
        self.current_heading = 0.0
        self.heading_ever_set = False
        self.heading_last_update = 0.0

        # Real-time data (protected by _reading_lock)
        self.last_reading: Optional[LiDARReading] = None
        self.readings_per_second = 0
        self._reading_count = 0
        self._last_rate_check = time.time()

        # Real-time decoupling: the serial read thread only enqueues readings;
        # a dedicated worker thread runs the heavy SLAM/localization/detection
        # so a slow pipeline can never back up the UART buffer. When the queue
        # is full the oldest frames are dropped (tracked as saturation metric).
        self._processing_queue: "queue.Queue[LiDARReading]" = queue.Queue(maxsize=200)
        self._processing_thread: Optional[threading.Thread] = None
        self._processing_active = False
        self._dropped_frames = 0

        # WebSocket callback
        self.websocket_callback = None

    @property
    def mode(self):
        with self._state_lock:
            return self._mode

    @mode.setter
    def mode(self, value):
        with self._state_lock:
            self._mode = value

    @property
    def is_running(self):
        with self._state_lock:
            return self._is_running

    @is_running.setter
    def is_running(self, value):
        with self._state_lock:
            self._is_running = value

    def initialize(self) -> bool:
        """Initialize all components"""
        logger.info("=" * 60)
        logger.info("BlueOS LiDAR SLAM Extension v1.0.0")
        logger.info("=" * 60)

        # Initialize LiDAR driver
        self.driver = TFminiSDriver(
            port=self.config.lidar.port,
            baudrate=self.config.lidar.baudrate,
            timeout=self.config.lidar.timeout,
            min_signal=self.config.lidar.signal_threshold,
            min_range_m=self.config.lidar.min_range,
            max_range_m=self.config.lidar.max_range,
            medium_refractive_index=self.config.lidar.medium_refractive_index
        )

        if not self.driver.connect():
            logger.error("Failed to connect to LiDAR")
            return False

        # Set frequency
        self.driver.set_framerate(self.config.lidar.frequency)

        # Add data callback
        self.driver.add_callback(self._on_lidar_reading)
        # Propagate driver errors to clients for visibility (Error Handling #3)
        self.driver.add_error_callback(self._on_driver_error)

        logger.info("Application initialized successfully")
        return True

    # Modes where losing the sensor is a safety concern
    ACTIVE_MODES = (MODE_MAPPING, MODE_LOCALIZING, MODE_NAVIGATING,
                    MODE_RECORDING, MODE_SCANNING)

    def _on_driver_error(self, context: str, exc: Exception):
        """Broadcast driver errors and enforce a safe state (Maritime #2).

        If the sensor has failed while an active guidance/mapping mode is
        running, transition to IDLE and raise an alarm so stale data cannot
        drive navigation.
        """
        health = self.get_health()

        if health['state'] == 'failed' and self.mode in self.ACTIVE_MODES:
            previous_mode = self.mode
            self.set_mode(self.MODE_IDLE)
            logger.error(
                f"SAFETY: sensor failed during {previous_mode}; forced IDLE"
            )
            if self.websocket_callback:
                self.websocket_callback('safety_alarm', {
                    'priority': 'alarm',
                    'message': f'Sensor failure during {previous_mode}; system moved to IDLE',
                    'previous_mode': previous_mode
                })

        if self.websocket_callback:
            self.websocket_callback('driver_error', {
                'context': context,
                'message': str(exc),
                'health': health
            })

    def start(self) -> bool:
        """Start the application"""
        if self.is_running:
            logger.warning("Application already running")
            return True

        if not self.driver:
            if not self.initialize():
                return False

        try:
            self._start_processing_thread()
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

        self._stop_processing_thread()

        logger.info("Application stopped")

    def _start_processing_thread(self):
        """Start the background reading-processing worker."""
        if self._processing_thread and self._processing_thread.is_alive():
            return
        self._processing_active = True
        self._processing_thread = threading.Thread(
            target=self._processing_loop, daemon=True, name="reading-processor"
        )
        self._processing_thread.start()

    def _stop_processing_thread(self):
        """Stop the background worker and drain the queue."""
        self._processing_active = False
        if self._processing_thread:
            self._processing_thread.join(timeout=2.0)
        # Drain any leftover queued readings
        try:
            while True:
                self._processing_queue.get_nowait()
        except queue.Empty:
            pass

    def _processing_loop(self):
        """Consume queued readings and run the heavy processing pipeline."""
        while self._processing_active:
            try:
                reading = self._processing_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._process_reading(reading)
            except Exception as e:
                logger.error(f"Processing error: {e}")

    def set_mode(self, mode: str) -> bool:
        """Set operating mode"""
        valid_modes = [self.MODE_IDLE, self.MODE_MAPPING, self.MODE_LOCALIZING,
                      self.MODE_NAVIGATING, self.MODE_RECORDING, self.MODE_SCANNING]

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

        if self.mode == self.MODE_SCANNING and mode != self.MODE_SCANNING:
            # Stop the 3D scan (data is kept until cleared/saved)
            self.scanner.stop_scan()

        self.mode = mode
        logger.info(f"Mode changed to: {mode}")
        return True

    def _on_lidar_reading(self, reading: LiDARReading):
        """Lightweight driver callback: validate and enqueue only.

        Runs on the serial read thread, so it must stay fast. Heavy work is
        deferred to the processing worker (_processing_loop).
        """
        if not reading.valid:
            return

        # Rule 4: reject statistical outliers before any downstream processing.
        # Turbidity/multipath in the underwater environment produces spurious
        # spikes that would otherwise corrupt the SLAM map.
        # Use the monotonic capture timestamp for the rate-of-change gate
        # (Physics Audit H9): wall-clock datetime.now() is subject to NTP
        # steps on an RTC-less companion computer, which can silently
        # disable or falsely trigger the gate. Fall back to wall-clock only
        # if a reading somehow arrives without mono_timestamp set.
        rate_ts = reading.mono_timestamp if reading.mono_timestamp is not None \
            else reading.timestamp.timestamp()
        quality = self.data_quality.validate(
            distance=reading.distance,
            signal_strength=reading.signal_strength,
            timestamp=rate_ts
        )
        if not quality.accepted:
            logger.debug(f"Reading rejected ({quality.reason}): {reading.distance}m")
            return

        with self._reading_lock:
            self.last_reading = reading
            self._reading_count += 1

            # Calculate readings per second: divide by the ACTUAL elapsed
            # window, not a raw count (Physics Audit H5). The window is
            # ">= 1.0s" by an unbounded amount when readings are sparse or
            # after a dropout, so treating the count as if the window were
            # exactly 1.000s biases the reported rate.
            now = time.time()
            elapsed = now - self._last_rate_check
            if elapsed >= 1.0:
                self.readings_per_second = round(self._reading_count / elapsed, 1)
                self._reading_count = 0
                self._last_rate_check = now

        # Enqueue for processing; drop oldest when saturated so the read thread
        # never blocks on a slow pipeline.
        try:
            self._processing_queue.put_nowait(reading)
        except queue.Full:
            try:
                self._processing_queue.get_nowait()  # evict oldest
                self._processing_queue.put_nowait(reading)
            except queue.Empty:
                pass
            self._dropped_frames += 1

    def _process_reading(self, reading: LiDARReading):
        """Heavy processing pipeline, runs on the worker thread."""
        # Convert distance to 3D point (assuming forward-facing sensor).
        # Full attitude (pitch/roll) is not fused -- see PHYSICS_AUDIT.md D2;
        # this remains a yaw-only projection until a real attitude source
        # (MAVLink ATTITUDE) is wired in (D1).
        d = reading.distance

        # Get current position estimate
        position = self._get_current_position()

        # World frame is ENU (x=East, y=North, z=Up). current_heading is a
        # COMPASS heading in degrees, clockwise-positive from North -- the
        # opposite handedness of the math convention (CCW-positive from
        # +X). Using cos()->x / sin()->y directly (as before) silently
        # mirrors the reconstructed geometry about the NE diagonal (Physics
        # Audit C4). Converting explicitly here keeps the single point of
        # truth for the heading convention in one place.
        heading_rad = np.radians(self.current_heading)
        world_x = position[0] + d * np.sin(heading_rad)
        world_y = position[1] + d * np.cos(heading_rad)
        world_z = position[2]

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

        elif self.mode == self.MODE_SCANNING:
            result = self.scanner.add_reading(
                distance=reading.distance,
                heading_deg=self.current_heading,
                signal_strength=reading.signal_strength
            )
            if (result and result.get('accepted') and self.websocket_callback
                    and self.scanner.accepted % self.config.scanner.progress_emit_every == 0):
                self.websocket_callback('scanner_progress', self.scanner.get_statistics())

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

    def get_health(self) -> Dict:
        """Classify overall system health for graceful degradation (#2).

        States:
          - healthy:  sensor connected, low error rate, good data quality
          - degraded: running but a subsystem is impaired (recovering,
                      elevated errors, or poor data quality)
          - failed:   sensor unavailable while the app is meant to be running
        """
        connected = bool(self.driver and self.driver.is_connected)
        lidar_stats = self.driver.get_statistics() if self.driver else {}
        error_rate = lidar_stats.get('error_rate', 0.0)
        dq = self.data_quality.quality_score
        reconnecting = lidar_stats.get('reconnect_attempts', 0) > 0

        # Temperature envelope check (IEC 60945 Category D: -15C..+55C)
        temp_out_of_range = False
        last = self.last_reading
        if last is not None:
            if last.temperature < -15.0 or last.temperature > 55.0:
                temp_out_of_range = True

        # No attitude/heading source is wired up in this build (Physics
        # Audit C3/D1): current_heading stays at its 0.0 placeholder. In
        # modes that project readings into world coordinates, that silently
        # collapses every beam onto a single axis. Surface this honestly
        # instead of hiding it.
        heading_dependent_mode = self.mode in (
            self.MODE_MAPPING, self.MODE_RECORDING,
            self.MODE_NAVIGATING, self.MODE_SCANNING
        )
        heading_missing = heading_dependent_mode and not self.heading_ever_set

        reasons = []
        state = 'healthy'

        if self.is_running and not connected:
            state = 'failed'
            reasons.append('sensor_disconnected')
        elif reconnecting or error_rate > 0.1 or dq < 0.7 or temp_out_of_range or heading_missing:
            state = 'degraded'
            if reconnecting:
                reasons.append('reconnecting')
            if error_rate > 0.1:
                reasons.append('high_error_rate')
            if dq < 0.7:
                reasons.append('low_data_quality')
            if temp_out_of_range:
                reasons.append('temperature_out_of_range')
            if heading_missing:
                reasons.append('no_heading_source')

        return {
            'state': state,
            'reasons': reasons,
            'sensor_connected': connected,
            'error_rate': round(error_rate, 4),
            'data_quality_score': dq,
            'heading_source_active': self.heading_ever_set,
            'last_error': lidar_stats.get('last_error') if self.driver else None
        }

    def set_heading(self, heading_deg: float):
        """Update the compass heading (degrees, clockwise-positive from
        North). Extension point for a future attitude source (MAVLink
        ATTITUDE/VFR_HUD, see PHYSICS_AUDIT.md D1) -- not called anywhere
        in this build, so heading_ever_set stays honestly False until one
        is wired in."""
        with self._state_lock:
            self.current_heading = heading_deg % 360.0
            self.heading_ever_set = True
            self.heading_last_update = time.time()

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
            'scanner': self.scanner.get_statistics(),
            'data_quality': self.data_quality.get_statistics(),
            'health': self.get_health(),
            'pipeline': {
                'queue_depth': self._processing_queue.qsize(),
                'queue_capacity': self._processing_queue.maxsize,
                'dropped_frames': self._dropped_frames
            },
            'config': self.config.to_dict()
        }


# Create Flask application
app = Flask(__name__,
           template_folder='web/templates',
           static_folder='web/static')

# Restricted CORS - only allow same-origin and BlueOS hosts
ALLOWED_ORIGINS = os.environ.get('CORS_ORIGINS', 'http://localhost:5000,http://127.0.0.1:5000,http://blueos.local').split(',')
CORS(app, origins=ALLOWED_ORIGINS, supports_credentials=True)
# async_mode='threading' makes socketio.emit() safe to call from the LiDAR
# background thread (Concurrency finding #2).
socketio = SocketIO(app, cors_allowed_origins=ALLOWED_ORIGINS, async_mode='threading')

# When enabled, WebSocket clients must present a valid token on connect.
# Off by default for local BlueOS use; enable on shared networks.
REQUIRE_WS_AUTH = os.environ.get('REQUIRE_WS_AUTH', 'false').lower() == 'true'

# Create application instance
lidar_app = LiDARSLAMApplication()


# WebSocket callback
def ws_emit(event: str, data: dict):
    socketio.emit(event, data)


lidar_app.websocket_callback = ws_emit


# ============ Standard Error Handling ============

def error_response(code: str, message: str, status: int, details: dict = None):
    """Build a standardized error response body (API Design #2).

    Shape: {"error": {"code": str, "message": str, "details"?: object}}
    """
    body = {'error': {'code': code, 'message': message}}
    if details:
        body['error']['details'] = details
    return jsonify(body), status


@app.errorhandler(400)
def handle_400(e):
    return error_response('bad_request', getattr(e, 'description', 'Bad request'), 400)


@app.errorhandler(401)
def handle_401(e):
    return error_response('unauthorized', 'Authentication required', 401)


@app.errorhandler(404)
def handle_404(e):
    return error_response('not_found', 'Resource not found', 404)


@app.errorhandler(405)
def handle_405(e):
    return error_response('method_not_allowed', 'Method not allowed', 405)


@app.errorhandler(429)
def handle_429(e):
    return error_response('rate_limited', 'Rate limit exceeded', 429)


@app.errorhandler(500)
def handle_500(e):
    logger.exception("Unhandled server error")
    return error_response('internal_error', 'Internal server error', 500)


# ============ REST API Routes ============

@app.before_request
def check_auth():
    """Check authentication for non-public routes"""
    if request.method == 'OPTIONS':
        return None
    if is_public_route(request.path):
        return None
    if request.path.startswith('/static'):
        return None
    # WebSocket upgrade requests handled separately
    if request.environ.get('HTTP_UPGRADE', '').lower() == 'websocket':
        return None
    return None  # Auth enforced per-route with @require_auth


@app.route('/')
def index():
    """Serve main page"""
    return render_template('index.html')


@app.route('/api/health')
def health_check():
    """Health check endpoint for Docker HEALTHCHECK and monitoring.

    Returns 200 when healthy/degraded (process is alive) and 503 when the
    sensor has failed while the app is meant to be running.
    """
    health = lidar_app.get_health()
    status_code = 503 if health['state'] == 'failed' else 200
    return jsonify({
        'status': health['state'],
        'timestamp': datetime.now().isoformat(),
        'lidar_connected': health['sensor_connected'],
        'reasons': health['reasons']
    }), status_code


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
@require_rate_limit
def get_status():
    """Get application status"""
    return jsonify(lidar_app.get_status())


@app.route('/api/start', methods=['POST'])
@require_auth
@require_rate_limit
def start():
    """Start application"""
    success = lidar_app.start()
    return jsonify({'success': success}), 200 if success else 500


@app.route('/api/stop', methods=['POST'])
@require_auth
@require_rate_limit
def stop():
    """Stop application"""
    lidar_app.stop()
    return jsonify({'success': True})


@app.route('/api/mode/<mode>', methods=['POST'])
@require_auth
@require_rate_limit
def set_mode(mode: str):
    """Set operating mode"""
    success = lidar_app.set_mode(mode)
    return jsonify({'success': success, 'mode': lidar_app.mode}), 200 if success else 400


# ============ Mapping API ============

@app.route('/api/mapping/start', methods=['POST'])
@require_auth
@require_rate_limit
def start_mapping():
    """Start mapping mode"""
    lidar_app.slam_engine.clear()
    lidar_app.set_mode(LiDARSLAMApplication.MODE_MAPPING)
    return jsonify({'success': True, 'mode': 'mapping'})


@app.route('/api/mapping/stop', methods=['POST'])
@require_auth
@require_rate_limit
def stop_mapping():
    """Stop mapping"""
    lidar_app.set_mode(LiDARSLAMApplication.MODE_IDLE)
    return jsonify({
        'success': True,
        'statistics': lidar_app.slam_engine.get_statistics()
    })


@app.route('/api/mapping/clear', methods=['POST'])
@require_auth
@require_rate_limit
def clear_mapping():
    """Clear current map"""
    lidar_app.slam_engine.clear()
    return jsonify({'success': True})


@app.route('/api/mapping/statistics')
@require_rate_limit
def mapping_statistics():
    """Get mapping statistics"""
    return jsonify(lidar_app.slam_engine.get_statistics())


@app.route('/api/mapping/points')
@require_rate_limit
def get_map_points():
    """Get current map points (downsampled for web)"""
    points = lidar_app.slam_engine.get_map_downsampled(voxel_size=0.1)
    if points is None:
        return jsonify({'points': []})
    return jsonify({'points': points.tolist(), 'count': len(points)})


@app.route('/api/mapping/trajectory')
@require_rate_limit
def get_trajectory():
    """Get mapping trajectory"""
    trajectory = lidar_app.slam_engine.get_trajectory()
    return jsonify({'trajectory': trajectory.tolist(), 'count': len(trajectory)})


# ============ Maps API ============

@app.route('/api/maps')
@require_rate_limit
def list_maps():
    """List all saved maps"""
    return jsonify(lidar_app.map_manager.list_maps())


@app.route('/api/maps/<name>')
@require_rate_limit
def get_map_info(name: str):
    """Get map information"""
    if not validate_path_component(name):
        return jsonify({'error': 'Invalid map name'}), 400
    info = lidar_app.map_manager.get_map_info(name)
    if info:
        return jsonify(info)
    return jsonify({'error': 'Map not found'}), 404


@app.route('/api/maps/<name>/save', methods=['POST'])
@require_auth
@require_rate_limit
def save_map(name: str):
    """Save current map"""
    if not validate_path_component(name):
        return jsonify({'success': False, 'error': 'Invalid map name'}), 400

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
@require_auth
@require_rate_limit
def load_map(name: str):
    """Load map for localization"""
    if not validate_path_component(name):
        return jsonify({'success': False, 'error': 'Invalid map name'}), 400

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
@require_auth
@require_rate_limit
def delete_map(name: str):
    """Delete a map"""
    if not validate_path_component(name):
        return jsonify({'success': False, 'error': 'Invalid map name'}), 400
    success = lidar_app.map_manager.delete_map(name)
    return jsonify({'success': success}), 200 if success else 404


# ============ Localization API ============

@app.route('/api/localization/position')
@require_rate_limit
def get_position():
    """Get current position"""
    stats = lidar_app.localization_engine.get_statistics()
    return jsonify(stats)


@app.route('/api/localization/reset', methods=['POST'])
@require_auth
@require_rate_limit
def reset_localization():
    """Reset localization"""
    lidar_app.localization_engine.reset()
    return jsonify({'success': True})


# ============ Profile Recording API ============

@app.route('/api/profiles')
@require_rate_limit
def list_profiles():
    """List all navigation profiles"""
    return jsonify(lidar_app.profile_recorder.list_profiles())


@app.route('/api/profiles/<name>/record/start', methods=['POST'])
@require_auth
@require_rate_limit
def start_recording(name: str):
    """Start recording a navigation profile"""
    if not validate_path_component(name):
        return jsonify({'success': False, 'error': 'Invalid profile name'}), 400

    data = request.json or {}
    description = data.get('description', '')

    success = lidar_app.profile_recorder.start_recording(name, description)
    if success:
        lidar_app.set_mode(LiDARSLAMApplication.MODE_RECORDING)

    return jsonify({'success': success}), 200 if success else 400


@app.route('/api/profiles/record/stop', methods=['POST'])
@require_auth
@require_rate_limit
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
@require_auth
@require_rate_limit
def start_navigation(name: str):
    """Start navigating with a profile"""
    if not validate_path_component(name):
        return jsonify({'success': False, 'error': 'Invalid profile name'}), 400

    profile = lidar_app.profile_recorder.load_profile(name)
    if not profile:
        return jsonify({'success': False, 'error': 'Profile not found'}), 404

    success = lidar_app.profile_navigator.start_navigation(profile)
    if success:
        lidar_app.set_mode(LiDARSLAMApplication.MODE_NAVIGATING)

    return jsonify({'success': success})


@app.route('/api/profiles/navigate/stop', methods=['POST'])
@require_auth
@require_rate_limit
def stop_navigation():
    """Stop navigation"""
    lidar_app.profile_navigator.stop_navigation()
    lidar_app.set_mode(LiDARSLAMApplication.MODE_IDLE)
    return jsonify({'success': True})


@app.route('/api/profiles/<name>/delete', methods=['DELETE'])
@require_auth
@require_rate_limit
def delete_profile(name: str):
    """Delete a profile"""
    if not validate_path_component(name):
        return jsonify({'success': False, 'error': 'Invalid profile name'}), 400
    success = lidar_app.profile_recorder.delete_profile(name)
    return jsonify({'success': success}), 200 if success else 404


# ============ Object Detection API ============

@app.route('/api/objects')
@require_rate_limit
def get_objects():
    """Get all detected objects"""
    return jsonify({
        'objects': lidar_app.object_detector.get_objects(),
        'statistics': lidar_app.object_detector.get_statistics()
    })


@app.route('/api/objects/nearby')
@require_rate_limit
def get_nearby_objects():
    """Get objects near a position"""
    try:
        x = float(request.args.get('x', 0))
        y = float(request.args.get('y', 0))
        z = float(request.args.get('z', 0))
        radius = min(float(request.args.get('radius', 5.0)), 100.0)  # Max 100m radius
    except (ValueError, TypeError):
        return jsonify({'error': 'Invalid coordinates'}), 400

    objects = lidar_app.object_detector.get_nearby_objects((x, y, z), radius)
    return jsonify({'objects': objects})


@app.route('/api/objects/clear', methods=['POST'])
@require_auth
@require_rate_limit
def clear_objects():
    """Clear detected objects"""
    lidar_app.object_detector.clear_objects()
    return jsonify({'success': True})


@app.route('/api/objects/save', methods=['POST'])
@require_auth
@require_rate_limit
def save_objects():
    """Save detected objects"""
    success = lidar_app.object_detector.save_objects()
    return jsonify({'success': success})


# ============ 3D Scanner API ============

@app.route('/api/scanner/start', methods=['POST'])
@require_auth
@require_rate_limit
def scanner_start():
    """Start a 3D orbit scan.

    Body: {center: [x,y,z]?, orbit_radius: float?, initial_z: float?}
    The vehicle should orbit the object with the sensor aimed at its center.
    """
    data = request.get_json(silent=True) or {}

    center = data.get('center', [0.0, 0.0, 0.0])
    if (not isinstance(center, (list, tuple)) or len(center) != 3
            or not all(isinstance(v, (int, float)) for v in center)):
        return jsonify({'success': False, 'error': 'center must be [x, y, z]'}), 400

    orbit_radius = data.get('orbit_radius')
    if orbit_radius is not None:
        try:
            orbit_radius = float(orbit_radius)
        except (TypeError, ValueError):
            return jsonify({'success': False, 'error': 'Invalid orbit_radius'}), 400
        if not (0.2 < orbit_radius <= 12.0):
            return jsonify({'success': False,
                            'error': 'orbit_radius must be within (0.2, 12.0] m'}), 400

    try:
        initial_z = float(data.get('initial_z', 0.0))
    except (TypeError, ValueError):
        return jsonify({'success': False, 'error': 'Invalid initial_z'}), 400

    if not lidar_app.scanner.start_scan(center=tuple(center),
                                        orbit_radius=orbit_radius,
                                        initial_z=initial_z):
        return jsonify({'success': False, 'error': 'Failed to start scan'}), 400

    lidar_app.set_mode(LiDARSLAMApplication.MODE_SCANNING)
    return jsonify({'success': True, 'scanner': lidar_app.scanner.get_statistics()})


@app.route('/api/scanner/stop', methods=['POST'])
@require_auth
@require_rate_limit
def scanner_stop():
    """Stop the scan (point cloud is kept until cleared or saved)."""
    lidar_app.set_mode(LiDARSLAMApplication.MODE_IDLE)
    return jsonify({'success': True, 'scanner': lidar_app.scanner.get_statistics()})


@app.route('/api/scanner/layer', methods=['POST'])
@require_auth
@require_rate_limit
def scanner_set_layer():
    """Set the current scan layer depth/altitude. Body: {z: float}"""
    data = request.get_json(silent=True) or {}
    try:
        z = float(data['z'])
    except (KeyError, TypeError, ValueError):
        return jsonify({'success': False, 'error': 'z (float) is required'}), 400

    lidar_app.scanner.set_layer(z)
    return jsonify({'success': True, 'current_z': z})


@app.route('/api/scanner/status')
@require_rate_limit
def scanner_status():
    """Scan progress: per-layer coverage, point count, rejection stats."""
    return jsonify(lidar_app.scanner.get_statistics())


@app.route('/api/scanner/points')
@require_rate_limit
def scanner_points():
    """Current scan point cloud (strided to at most 50k points for the web)."""
    points = lidar_app.scanner.get_points(max_points=50000)
    return jsonify({'points': points, 'count': len(points)})


@app.route('/api/scanner/clear', methods=['POST'])
@require_auth
@require_rate_limit
def scanner_clear():
    """Discard the current scan data."""
    lidar_app.scanner.clear()
    return jsonify({'success': True})


@app.route('/api/scanner/save/<name>', methods=['POST'])
@require_auth
@require_rate_limit
def scanner_save(name: str):
    """Save the scan point cloud as a named map (PLY by default)."""
    if not validate_path_component(name):
        return jsonify({'success': False, 'error': 'Invalid scan name'}), 400

    points = lidar_app.scanner.get_points()
    if not points:
        return jsonify({'success': False, 'error': 'No scan data'}), 400

    data = request.get_json(silent=True) or {}
    description = data.get('description', '3D object scan')
    stats = lidar_app.scanner.get_statistics()

    success = lidar_app.map_manager.save_map(
        name=name,
        points=np.array(points, dtype=float),
        description=description,
        tags=['3d_scan'] + list(data.get('tags', [])),
        total_scans=stats.get('layer_count', 0)
    )
    return jsonify({'success': success,
                    'point_count': len(points)}), 200 if success else 500


# ============ WebSocket Events ============

@socketio.on('connect')
def on_connect(auth=None):
    """Handle a new WebSocket client.

    When REQUIRE_WS_AUTH is set, the client must supply a valid token via the
    Socket.IO auth payload ({token: ...}) or an api_key query parameter.
    Returning False rejects the connection.
    """
    if REQUIRE_WS_AUTH:
        token = None
        if isinstance(auth, dict):
            token = auth.get('token') or auth.get('api_key')
        if not token:
            token = request.args.get('api_key')
        if not validate_token(token):
            logger.warning("WebSocket connection rejected: invalid token")
            return False

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


# ============ API Versioning ============

def _register_versioned_aliases():
    """Expose every /api/<x> route also as /api/v1/<x> (API Design #1).

    The unversioned paths remain for backward compatibility; /api/v1 is the
    canonical prefix going forward.
    """
    for rule in list(app.url_map.iter_rules()):
        if rule.rule.startswith('/api/') and not rule.rule.startswith('/api/v1/'):
            versioned = rule.rule.replace('/api/', '/api/v1/', 1)
            view = app.view_functions[rule.endpoint]
            methods = sorted(rule.methods - {'HEAD', 'OPTIONS'})
            app.add_url_rule(
                versioned,
                endpoint=f'v1_{rule.endpoint}',
                view_func=view,
                methods=methods
            )


_register_versioned_aliases()


# ============ Main Entry Point ============

def main():
    """Main entry point"""
    logger.info("Starting BlueOS LiDAR SLAM Extension...")

    # Initialize API token
    api_token = init_default_token()
    logger.info(f"API Token: {api_token}")

    # Initialize application
    if not lidar_app.initialize():
        logger.warning("LiDAR initialization failed - running in demo mode")

    # Run Flask server with SocketIO.
    # allow_unsafe_werkzeug is only permitted in DEBUG; production deployments
    # should front this with a proper WSGI server (see DEPLOYMENT.md /
    # docker-compose gunicorn command). Never ship with DEBUG=true.
    if Config.DEBUG:
        logger.warning("DEBUG mode is ON - do not use in production")

    socketio.run(
        app,
        host=Config.WEB_HOST,
        port=Config.WEB_PORT,
        debug=Config.DEBUG,
        allow_unsafe_werkzeug=Config.DEBUG
    )


if __name__ == '__main__':
    main()
