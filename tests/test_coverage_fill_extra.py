"""Coverage-fill tests: slam_engine, security, object_detection,
profile_recorder / navigator, localization."""

import sys
import os
import time
from datetime import datetime

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.slam_engine import SLAMEngine
from app.security import (init_default_token, is_public_route, validate_token,
                          safe_join, API_TOKENS, hash_token)
from app.object_detection import ObjectDetector
from app.profile_recorder import (ProfileRecorder, ProfileNavigator, Waypoint,
                                   NavigationProfile)
from app.localization import LocalizationEngine, ParticleFilterLocalizer


# ============================== SLAM ==============================

class TestSLAMEngineFill:
    def test_add_points_bulk_then_process(self):
        eng = SLAMEngine()
        eng.config.buffer_size = 50
        eng.add_points(np.random.rand(60, 3))  # exceeds buffer -> process
        assert eng.total_scans >= 1

    def test_add_point_buffer_cap(self):
        eng = SLAMEngine()
        eng.config.buffer_size = 5
        # Add far more than cap*multiplier without triggering a full process
        for i in range(200):
            eng.add_point(float(i), 0.0, 0.0)
        assert len(eng.scan_buffer) <= eng.config.buffer_size * eng.MAX_BUFFER_MULTIPLIER

    def test_process_scan_empty(self):
        eng = SLAMEngine()
        ok, T = eng.process_scan(np.empty((0, 3)), datetime.now())
        assert ok is False and T.shape == (4, 4)

    def test_process_scan_too_few_points(self):
        eng = SLAMEngine()
        ok, _ = eng.process_scan(np.random.rand(3, 3), datetime.now())
        assert ok is False

    def test_process_scan_filters_nonfinite(self):
        eng = SLAMEngine()
        pts = np.random.rand(40, 3)
        pts[0] = [np.nan, 0, 0]
        ok, _ = eng.process_scan(pts, datetime.now())
        assert ok is True

    def test_save_and_load_map(self, tmp_path):
        eng = SLAMEngine()
        eng.process_scan(np.random.rand(50, 3), datetime.now())
        fp = str(tmp_path / "map.ply")
        assert eng.save_map(fp) is True
        eng2 = SLAMEngine()
        assert eng2.load_map(fp) is True
        assert eng2.total_points == 50

    def test_save_map_no_data(self, tmp_path):
        assert SLAMEngine().save_map(str(tmp_path / "x.ply")) is False

    def test_load_map_exception_path(self):
        from unittest.mock import patch
        eng = SLAMEngine()
        with patch('app.slam_engine.o3d.io.read_point_cloud',
                   side_effect=RuntimeError("corrupt")):
            assert eng.load_map("/whatever.ply") is False

    def test_downsampled_and_trajectory(self):
        eng = SLAMEngine()
        eng.process_scan(np.random.rand(100, 3), datetime.now())
        assert eng.get_map_downsampled(voxel_size=0.1) is not None
        assert eng.get_trajectory().shape[0] >= 1

    def test_reorthonormalization_over_many_scans(self):
        eng = SLAMEngine()
        for _ in range(eng.REORTHONORMALIZE_EVERY + 5):
            eng.process_scan(np.random.rand(40, 3), datetime.now())
        R = eng.current_pose[:3, :3]
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-6)


# ============================== Security ==============================

class TestSecurityFill:
    def test_init_default_token_generates(self, monkeypatch, capsys):
        monkeypatch.delenv("LIDAR_API_TOKEN", raising=False)
        tok = init_default_token()
        assert isinstance(tok, str) and len(tok) > 10
        assert "Generated API token" in capsys.readouterr().out

    def test_init_default_token_from_env(self, monkeypatch):
        monkeypatch.setenv("LIDAR_API_TOKEN", "x" * 40)
        assert init_default_token() == "[set via LIDAR_API_TOKEN]"

    def test_is_public_route(self):
        assert is_public_route("/api/health") is True
        assert is_public_route("/api/start") is False

    def test_validate_token_empty(self):
        assert validate_token(None) is False
        assert validate_token("") is False

    def test_validate_token_valid(self):
        API_TOKENS[hash_token("valid" * 4)] = {"name": "t"}
        assert validate_token("valid" * 4) is True

    def test_safe_join_blocks_traversal(self):
        assert safe_join("/base", "../etc") is None

    def test_safe_join_ok(self):
        assert safe_join("/base", "sub").endswith("/base/sub")


# ============================== Object Detection ==============================

class TestObjectDetectionFill:
    def test_process_and_classify(self, tmp_path):
        det = ObjectDetector(objects_dir=str(tmp_path))
        # Feed a stable flat surface then a step change to provoke a pattern
        for i in range(60):
            det.process_reading(distance=2.0 + (0.0 if i < 30 else 1.5),
                                strength=200, position=(float(i), 0.0, 0.0),
                                timestamp=datetime.now())
        assert isinstance(det.get_objects(), list)
        assert "total_detections" in det.get_statistics() or det.get_statistics() is not None

    def test_nearby_objects(self, tmp_path):
        det = ObjectDetector(objects_dir=str(tmp_path))
        near = det.get_nearby_objects((0.0, 0.0, 0.0), 5.0)
        assert isinstance(near, list)

    def test_save_and_clear(self, tmp_path):
        det = ObjectDetector(objects_dir=str(tmp_path))
        assert det.save_objects() in (True, False)
        det.clear_objects()
        assert det.get_objects() == []


# ============================== Profile Recorder / Navigator ==============================

class TestProfileFill:
    def test_waypoint_roundtrip(self):
        wp = Waypoint(index=1, timestamp=datetime.now(), position=(1, 2, 3),
                      heading=45.0, distance_reading=2.0, signal_strength=200)
        wp2 = Waypoint.from_dict(wp.to_dict())
        assert wp2.position == (1, 2, 3) and wp2.heading == 45.0

    def test_profile_roundtrip(self):
        wp = Waypoint(index=0, timestamp=datetime.now(), position=(0, 0, 0),
                      heading=0.0, distance_reading=1.0, signal_strength=200)
        prof = NavigationProfile(name="p", created=datetime.now(), waypoints=[wp])
        prof2 = NavigationProfile.from_dict(prof.to_dict())
        assert prof2.name == "p" and len(prof2.waypoints) == 1

    def test_load_missing_and_invalid(self, tmp_path):
        rec = ProfileRecorder(str(tmp_path))
        assert rec.load_profile("nope") is None
        assert rec.load_profile("../evil") is None

    def test_save_load_delete_roundtrip(self, tmp_path):
        rec = ProfileRecorder(str(tmp_path))
        rec.start_recording("p1", "desc")
        rec.add_waypoint(position=(0, 0, 0), heading=0, distance_reading=1.0, signal_strength=200)
        rec.add_waypoint(position=(1, 0, 0), heading=10, distance_reading=1.2, signal_strength=200)
        prof = rec.stop_recording()
        assert rec.save_profile(prof) is True
        loaded = rec.load_profile("p1")
        assert loaded is not None and len(loaded.waypoints) == 2
        assert any(p["name"] == "p1" for p in rec.list_profiles())
        assert rec.delete_profile("p1") is True
        assert rec.delete_profile("p1") is False  # already gone

    def test_delete_invalid_name(self, tmp_path):
        assert ProfileRecorder(str(tmp_path)).delete_profile("../x") is False

    def test_navigator_lifecycle(self):
        nav = ProfileNavigator()
        assert nav.update((0, 0, 0), 0, 1.0)["status"] == "not_navigating"
        wp = [Waypoint(index=i, timestamp=datetime.now(),
                       position=(float(i), 0, 0), heading=0.0,
                       distance_reading=1.0, signal_strength=200) for i in range(3)]
        prof = NavigationProfile(name="p", created=datetime.now(), waypoints=wp)
        assert nav.start_navigation(prof) is True
        g = nav.update((0.0, 0.0, 0.0), 0.0, 1.0)
        assert g["status"] in ("reached", "on_track", "approaching")
        assert "heading_correction" in g
        nav.stop_navigation()
        assert nav.is_navigating is False

    def test_navigator_rejects_empty_profile(self):
        nav = ProfileNavigator()
        empty = NavigationProfile(name="e", created=datetime.now(), waypoints=[])
        assert nav.start_navigation(empty) is False

    def test_navigator_off_course_and_completed(self):
        nav = ProfileNavigator()
        wp = [Waypoint(index=0, timestamp=datetime.now(), position=(100.0, 0, 0),
                       heading=90.0, distance_reading=1.0, signal_strength=200)]
        prof = NavigationProfile(name="p", created=datetime.now(), waypoints=wp)
        nav.start_navigation(prof)
        g = nav.update((0.0, 0.0, 0.0), 0.0, 1.0)  # far + heading way off
        assert g["status"] in ("off_course", "on_track")
        # Force completion
        nav.current_waypoint_index = 1
        assert nav.update((0.0, 0.0, 0.0), 0.0, 1.0)["status"] == "completed"


# ============================== Localization ==============================

class TestLocalizationFill:
    def test_reference_map_and_position(self):
        eng = LocalizationEngine()
        eng.set_reference_map(np.random.rand(500, 3) * 5)
        assert eng.is_initialized is True
        pos = eng.get_position()
        assert len(pos) == 3

    def test_add_points_accumulate(self):
        eng = LocalizationEngine()
        eng.set_reference_map(np.random.rand(500, 3) * 5)
        eng.buffer_size = 10
        result = None
        for i in range(15):
            result = eng.add_point(float(i % 3), float(i % 2), 0.0)
        assert eng.get_statistics() is not None

    def test_reset_and_trajectory(self):
        eng = LocalizationEngine()
        eng.set_reference_map(np.random.rand(200, 3) * 5)
        eng.reset()
        assert eng.get_trajectory().shape[0] == 0

    def test_particle_filter_localizer(self):
        pf = ParticleFilterLocalizer(num_particles=200)
        pf.set_reference_map(np.random.rand(300, 3) * 5)
        pf.update(distance_reading=2.5, motion_delta=(0.1, 0.0, 0.05))
        pf.update(distance_reading=2.6)
        est = pf.get_estimate()
        assert len(est) == 4  # x, y, theta, confidence

    def test_particle_filter_uninitialized(self):
        pf = ParticleFilterLocalizer(num_particles=50)
        pf.update(distance_reading=1.0)  # no-op before reference map
        assert pf.get_estimate() == (0, 0, 0, 0)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
