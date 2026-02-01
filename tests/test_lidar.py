"""
Tests for TFmini-S LiDAR driver and SLAM components
"""

import pytest
import numpy as np
from datetime import datetime
from unittest.mock import Mock, patch, MagicMock

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.lidar_driver import TFminiSDriver, LiDARReading
from app.slam_engine import SLAMEngine, PointCloud
from app.profile_recorder import ProfileRecorder, Waypoint, NavigationProfile
from app.object_detection import ObjectDetector, DistancePatternAnalyzer
from app.map_manager import MapManager
from app.localization import LocalizationEngine


class TestLiDARReading:
    """Test LiDARReading dataclass"""

    def test_reading_creation(self):
        reading = LiDARReading(
            distance=1.5,
            signal_strength=200,
            timestamp=datetime.now(),
            temperature=25.0,
            valid=True
        )
        assert reading.distance == 1.5
        assert reading.signal_strength == 200
        assert reading.valid is True

    def test_reading_to_dict(self):
        reading = LiDARReading(
            distance=2.0,
            signal_strength=150,
            timestamp=datetime(2024, 1, 1, 12, 0, 0),
            temperature=22.5,
            valid=True
        )
        d = reading.to_dict()
        assert d['distance'] == 2.0
        assert d['signal_strength'] == 150
        assert d['temperature'] == 22.5
        assert d['valid'] is True


class TestTFminiSDriver:
    """Test TFmini-S driver"""

    def test_frame_parsing(self):
        driver = TFminiSDriver('/dev/ttyUSB0')

        # Valid frame: header(2) + dist_L + dist_H + strength_L + strength_H + temp_L + temp_H + checksum
        # Distance = 150cm (0x96, 0x00), Strength = 200 (0xC8, 0x00), Temp raw = 400 (0x90, 0x01)
        frame = bytes([
            0x59, 0x59,  # Header
            0x96, 0x00,  # Distance: 150cm
            0xC8, 0x00,  # Strength: 200
            0x90, 0x01,  # Temperature raw: 400
            0x00         # Placeholder for checksum (will calculate)
        ])

        # Calculate correct checksum
        checksum = sum(frame[:8]) & 0xFF
        frame = frame[:8] + bytes([checksum])

        reading = driver._parse_frame(frame)

        assert reading is not None
        assert reading.distance == 1.5  # 150cm = 1.5m
        assert reading.signal_strength == 200
        assert reading.valid is True

    def test_invalid_checksum(self):
        driver = TFminiSDriver('/dev/ttyUSB0')

        # Frame with wrong checksum
        frame = bytes([
            0x59, 0x59,
            0x96, 0x00,
            0xC8, 0x00,
            0x90, 0x01,
            0xFF  # Wrong checksum
        ])

        reading = driver._parse_frame(frame)
        assert reading is None


class TestSLAMEngine:
    """Test SLAM engine"""

    def test_initialization(self):
        engine = SLAMEngine()
        assert engine.total_points == 0
        assert engine.total_scans == 0

    def test_first_scan_processing(self):
        engine = SLAMEngine()

        points = np.random.rand(100, 3)
        success, transform = engine.process_scan(points, datetime.now())

        assert success is True
        assert engine.total_scans == 1
        assert engine.total_points == 100

    def test_get_map(self):
        engine = SLAMEngine()

        # No map initially
        assert engine.get_map() is None

        # Add points
        points = np.random.rand(50, 3)
        engine.process_scan(points, datetime.now())

        map_points = engine.get_map()
        assert map_points is not None
        assert len(map_points) == 50

    def test_clear(self):
        engine = SLAMEngine()

        points = np.random.rand(50, 3)
        engine.process_scan(points, datetime.now())
        assert engine.total_points > 0

        engine.clear()
        assert engine.total_points == 0
        assert engine.get_map() is None


class TestProfileRecorder:
    """Test navigation profile recording"""

    def test_start_recording(self, tmp_path):
        recorder = ProfileRecorder(str(tmp_path))

        success = recorder.start_recording("test_profile", "Test description")
        assert success is True
        assert recorder.is_recording is True

    def test_add_waypoint(self, tmp_path):
        recorder = ProfileRecorder(str(tmp_path))
        recorder.start_recording("test_profile")

        # Add first waypoint
        success = recorder.add_waypoint(
            position=(0, 0, 0),
            heading=0,
            distance_reading=1.0,
            signal_strength=200
        )
        assert success is True
        assert recorder.waypoint_counter == 1

        # Add second waypoint (far enough)
        success = recorder.add_waypoint(
            position=(1.0, 0, 0),
            heading=0,
            distance_reading=1.5,
            signal_strength=180
        )
        assert success is True
        assert recorder.waypoint_counter == 2

    def test_stop_recording(self, tmp_path):
        recorder = ProfileRecorder(str(tmp_path))
        recorder.start_recording("test_profile")

        profile = recorder.stop_recording()
        assert profile is not None
        assert profile.name == "test_profile"
        assert recorder.is_recording is False


class TestObjectDetector:
    """Test object detection"""

    def test_pattern_analyzer(self):
        analyzer = DistancePatternAnalyzer(window_size=20)

        # Add readings
        for i in range(20):
            analyzer.add_reading(
                distance=2.0 + np.random.normal(0, 0.01),
                strength=200,
                timestamp=datetime.now()
            )

        result = analyzer.analyze()
        assert result['status'] == 'analyzed'
        # Should detect flat surface with low variance
        assert result['is_flat'] is True

    def test_detector_initialization(self, tmp_path):
        detector = ObjectDetector(objects_dir=str(tmp_path))
        assert detector.detected_objects == {}

    def test_get_objects(self, tmp_path):
        detector = ObjectDetector(objects_dir=str(tmp_path))
        objects = detector.get_objects()
        assert isinstance(objects, list)
        assert len(objects) == 0


class TestMapManager:
    """Test map management"""

    def test_save_and_load_map(self, tmp_path):
        manager = MapManager(str(tmp_path))

        # Create test points
        points = np.random.rand(100, 3)

        # Save map
        success = manager.save_map("test_map", points, "Test map")
        assert success is True

        # Load map
        result = manager.load_map("test_map")
        assert result is not None

        loaded_points, metadata = result
        assert len(loaded_points) == 100
        assert metadata.name == "test_map"

    def test_list_maps(self, tmp_path):
        manager = MapManager(str(tmp_path))

        # Initially empty
        maps = manager.list_maps()
        assert len(maps) == 0

        # Save a map
        points = np.random.rand(50, 3)
        manager.save_map("map1", points)

        maps = manager.list_maps()
        assert len(maps) == 1
        assert maps[0]['name'] == "map1"

    def test_delete_map(self, tmp_path):
        manager = MapManager(str(tmp_path))

        points = np.random.rand(50, 3)
        manager.save_map("to_delete", points)

        success = manager.delete_map("to_delete")
        assert success is True

        result = manager.load_map("to_delete")
        assert result is None


class TestLocalizationEngine:
    """Test localization engine"""

    def test_initialization(self):
        engine = LocalizationEngine()
        assert engine.is_initialized is False

    def test_set_reference_map(self):
        engine = LocalizationEngine()

        map_points = np.random.rand(1000, 3) * 10
        success = engine.set_reference_map(map_points)

        assert success is True
        assert engine.is_initialized is True

    def test_get_position(self):
        engine = LocalizationEngine()
        pos = engine.get_position()
        assert len(pos) == 3


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
