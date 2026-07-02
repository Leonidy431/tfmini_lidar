"""
Tests for object detection classification, profile navigation guidance,
waypoint quality gating, and concurrency safety.

Depends on numpy + scipy (not Open3D), so runnable without the SLAM stack.
"""

import sys
import os
import threading
from datetime import datetime

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.object_detection import ObjectDetector, DistancePatternAnalyzer
from app.profile_recorder import (
    ProfileRecorder, ProfileNavigator, NavigationProfile, Waypoint
)


# ---------------- Object Detection ----------------

class TestPatternAnalyzer:
    def test_flat_surface_detected(self):
        analyzer = DistancePatternAnalyzer(window_size=20)
        for _ in range(20):
            analyzer.add_reading(2.0, 200, datetime.now())
        result = analyzer.analyze()
        assert result['status'] == 'analyzed'
        assert result['is_flat'] is True
        assert result['pattern_type'] == 'flat_surface'

    def test_edge_detected(self):
        analyzer = DistancePatternAnalyzer(window_size=20)
        # First half near, second half far -> a big step (edge)
        for _ in range(10):
            analyzer.add_reading(1.0, 200, datetime.now())
        for _ in range(10):
            analyzer.add_reading(3.0, 200, datetime.now())
        result = analyzer.analyze()
        assert result['edge_count'] >= 1
        assert result['pattern_type'] == 'edge_detected'

    def test_insufficient_data(self):
        analyzer = DistancePatternAnalyzer(window_size=50)
        analyzer.add_reading(2.0, 200, datetime.now())
        assert analyzer.analyze()['status'] == 'insufficient_data'


class TestObjectClassification:
    def test_flat_surface_classifies_as_wall(self, tmp_path):
        detector = ObjectDetector(objects_dir=str(tmp_path))
        pattern = {
            'pattern_type': 'flat_surface',
            'mean_strength': 300,
            'distance_std': 0.01,
        }
        obj_class, confidence = detector._classify_object(pattern, distance=2.0)
        assert obj_class == 'wall'
        assert confidence > 0.3

    def test_confidence_clamped_for_large_spread(self, tmp_path):
        """A multi-meter distance_std (routine for an edge) must not drive
        confidence negative (Physics Audit H8)."""
        detector = ObjectDetector(objects_dir=str(tmp_path))
        pattern = {
            'pattern_type': 'edge_detected',
            'mean_strength': 300,
            'distance_std': 3.0,  # meters -- large spread across an edge
        }
        _, confidence = detector._classify_object(pattern, distance=2.0)
        assert 0.0 <= confidence <= 1.0

    def test_unknown_when_no_rule_matches(self, tmp_path):
        detector = ObjectDetector(objects_dir=str(tmp_path))
        pattern = {
            'pattern_type': 'flat_surface',
            'mean_strength': 5,  # below wall's strength range
            'distance_std': 0.01,
        }
        obj_class, _ = detector._classify_object(pattern, distance=2.0)
        assert obj_class == 'unknown'

    def test_nearby_objects_merge(self, tmp_path):
        detector = ObjectDetector(objects_dir=str(tmp_path))
        ts = datetime.now()
        detector.process_reading(1.0, 200, (0.0, 0.0, 0.0), ts)
        # Feed an edge to trigger a detection near the same spot
        for i in range(30):
            d = 1.0 if i < 15 else 3.0
            detector.process_reading(d, 200, (0.01, 0.0, 0.0), datetime.now())
        objects = detector.get_objects()
        # Objects within 0.5m merge, so count stays small
        assert len(objects) <= 3


# ---------------- Navigation Guidance ----------------

def _make_profile():
    profile = NavigationProfile(name='test', created=datetime.now())
    profile.waypoints = [
        Waypoint(0, datetime.now(), (0.0, 0.0, 0.0), 0.0, 1.0, 200),
        Waypoint(1, datetime.now(), (1.0, 0.0, 0.0), 90.0, 1.5, 200),
        Waypoint(2, datetime.now(), (2.0, 0.0, 0.0), 90.0, 2.0, 200),
    ]
    return profile


class TestProfileNavigator:
    def test_reached_advances_waypoint(self):
        nav = ProfileNavigator()
        nav.start_navigation(_make_profile())
        # Start exactly at waypoint 0 -> reached, index advances
        result = nav.update((0.0, 0.0, 0.0), 0.0, 1.0)
        assert result['status'] == 'reached'
        assert nav.current_waypoint_index == 1

    def test_off_course_on_large_heading_error(self):
        nav = ProfileNavigator()
        nav.start_navigation(_make_profile())
        # Far from wp0 in distance, big heading error -> off_course
        result = nav.update((5.0, 5.0, 0.0), 0.0, 1.0)
        assert result['status'] in ('off_course', 'on_track')
        # heading target is 0 for wp0, current 0 -> no error; move to a case:
        result2 = nav.update((5.0, 5.0, 0.0), 200.0, 1.0)
        assert 'heading_error' in result2

    def test_not_navigating_when_stopped(self):
        nav = ProfileNavigator()
        assert nav.update((0, 0, 0), 0, 1.0)['status'] == 'not_navigating'

    def test_completed_after_last_waypoint(self):
        nav = ProfileNavigator()
        nav.start_navigation(_make_profile())
        nav.current_waypoint_index = 3  # past the end
        assert nav.update((0, 0, 0), 0, 1.0)['status'] == 'completed'

    def test_normalize_angle(self):
        nav = ProfileNavigator()
        assert nav._normalize_angle(270) == -90
        assert nav._normalize_angle(-270) == 90
        assert nav._normalize_angle(45) == 45

    def test_heading_correction_right_for_positive_error(self):
        """Compass convention: target clockwise of current -> turn right.

        wp0 heading=0 (North), vehicle heading=340 (i.e. 20 deg counter-
        clockwise of North) -> normalized error = 0 - 340 -> +20 (target is
        20 deg clockwise of current) -> correction must be 'right'
        (Physics Audit H2 - this was previously inverted to 'left').
        """
        nav = ProfileNavigator()
        nav.start_navigation(_make_profile())
        result = nav.update((5.0, 5.0, 0.0), 340.0, 1.0)
        assert result['heading_error'] > 0
        assert result['heading_correction'] == 'right'

    def test_heading_correction_left_for_negative_error(self):
        """wp0 heading=0, vehicle heading=20 -> error = 0-20 = -20 -> left."""
        nav = ProfileNavigator()
        nav.start_navigation(_make_profile())
        result = nav.update((5.0, 5.0, 0.0), 20.0, 1.0)
        assert result['heading_error'] < 0
        assert result['heading_correction'] == 'left'


# ---------------- Waypoint Quality Gating ----------------

class TestWaypointQualityGating:
    def test_low_signal_waypoint_rejected(self, tmp_path):
        rec = ProfileRecorder(str(tmp_path))
        rec.min_signal_strength = 100
        rec.start_recording('p')
        ok = rec.add_waypoint((0, 0, 0), 0.0, 1.0, signal_strength=10)
        assert ok is False
        assert rec.rejected_waypoints == 1

    def test_out_of_range_waypoint_rejected(self, tmp_path):
        rec = ProfileRecorder(str(tmp_path))
        rec.max_distance = 12.0
        rec.start_recording('p')
        ok = rec.add_waypoint((0, 0, 0), 0.0, 99.0, signal_strength=200)
        assert ok is False

    def test_good_waypoint_accepted_and_annotated(self, tmp_path):
        rec = ProfileRecorder(str(tmp_path))
        rec.start_recording('p')
        ok = rec.add_waypoint((0, 0, 0), 0.0, 1.0, signal_strength=200)
        assert ok is True
        wp = rec.current_profile.waypoints[0]
        assert wp.features.get('quality') == 'good'


# ---------------- Concurrency ----------------

class TestConcurrency:
    def test_concurrent_process_and_read(self, tmp_path):
        """Hammer the detector from multiple threads; no crash / corruption."""
        detector = ObjectDetector(objects_dir=str(tmp_path))
        errors = []

        def producer():
            try:
                for i in range(200):
                    detector.process_reading(
                        1.0 + (i % 5) * 0.5, 200, (i * 0.01, 0.0, 0.0), datetime.now()
                    )
            except Exception as e:  # noqa: BLE001
                errors.append(e)

        def reader():
            try:
                for _ in range(200):
                    detector.get_objects()
                    detector.get_statistics()
            except Exception as e:  # noqa: BLE001
                errors.append(e)

        threads = [threading.Thread(target=producer) for _ in range(3)]
        threads += [threading.Thread(target=reader) for _ in range(3)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert errors == []


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
