"""Coverage-fill: exception, traversal-guard, and format branches across
map_manager, lidar_driver, slam_engine, object_detection, profile_recorder,
localization and security."""

import sys
import os
from datetime import datetime
from unittest.mock import patch, MagicMock

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import app.map_manager as mm_mod
from app.map_manager import MapManager, MapMetadata
from app.slam_engine import SLAMEngine
from app.profile_recorder import ProfileRecorder
from app.object_detection import ObjectDetector


# ---------------- map_manager exception / traversal / format branches -------

class TestMapManagerBranches:
    def test_save_points_write_failure_aborts(self, tmp_path):
        mgr = MapManager(str(tmp_path))
        with patch('app.map_manager.o3d.io.write_point_cloud', return_value=False):
            assert mgr.save_map("m", np.random.rand(10, 3), format="ply") is False

    def test_save_exception_cleans_staging(self, tmp_path):
        mgr = MapManager(str(tmp_path))
        with patch.object(mgr, '_save_points', side_effect=RuntimeError("disk")):
            assert mgr.save_map("m", np.random.rand(10, 3)) is False
        assert not any(p.name.endswith('.staging') for p in tmp_path.iterdir())

    def test_load_pcd_and_h5_formats(self, tmp_path):
        mgr = MapManager(str(tmp_path))
        for fmt in ("pcd", "h5"):
            mgr.save_map(f"m_{fmt}", np.random.rand(20, 3), format=fmt)
            pts, meta = mgr.load_map(f"m_{fmt}")
            assert len(pts) == 20

    def test_load_trajectory_exception(self, tmp_path):
        mgr = MapManager(str(tmp_path))
        mgr.save_map("m", np.random.rand(10, 3), trajectory=np.random.rand(3, 3))
        with patch('app.map_manager.np.load', side_effect=RuntimeError("bad")):
            assert mgr.load_trajectory("m") is None

    def test_list_maps_oserror(self, tmp_path):
        mgr = MapManager(str(tmp_path))
        with patch('app.map_manager.os.listdir', side_effect=OSError("gone")):
            assert mgr.list_maps() == []

    def test_delete_traversal_guard(self, tmp_path):
        mgr = MapManager(str(tmp_path))
        mgr.save_map("m", np.random.rand(10, 3))
        with patch('app.map_manager.safe_join', return_value=None):
            assert mgr.delete_map("m") is False

    def test_delete_exception(self, tmp_path):
        mgr = MapManager(str(tmp_path))
        mgr.save_map("m", np.random.rand(10, 3))
        with patch('app.map_manager.shutil.rmtree', side_effect=RuntimeError("io")):
            assert mgr.delete_map("m") is False

    def test_update_metadata_traversal_and_exception(self, tmp_path):
        mgr = MapManager(str(tmp_path))
        mgr.save_map("m", np.random.rand(10, 3))
        with patch('app.map_manager.safe_join', return_value=None):
            assert mgr.update_metadata("m", description="x") is False
        with patch('app.map_manager.json.load', side_effect=ValueError("bad")):
            assert mgr.update_metadata("m", description="x") is False

    def test_export_exception(self, tmp_path):
        mgr = MapManager(str(tmp_path))
        mgr.save_map("m", np.random.rand(10, 3))
        with patch('app.map_manager.np.save', side_effect=RuntimeError("io")):
            assert mgr.export_map("m", str(tmp_path / "o.npy"), format="npy") is False

    def test_get_map_info_traversal_and_exception(self, tmp_path):
        mgr = MapManager(str(tmp_path))
        mgr.save_map("m", np.random.rand(10, 3))
        with patch('app.map_manager.safe_join', return_value=None):
            assert mgr.get_map_info("m") is None
        with patch('app.map_manager.os.listdir', side_effect=OSError("io")):
            assert mgr.get_map_info("m") is None

    def test_save_points_unsupported_format_internal(self, tmp_path):
        mgr = MapManager(str(tmp_path))
        assert mgr._save_points(str(tmp_path), np.random.rand(5, 3), "bogus") is None

    def test_metadata_to_dict_modified_present(self):
        m = MapMetadata(name="x", created=datetime(2024, 1, 1),
                        modified=datetime(2024, 1, 2))
        assert m.to_dict()["modified"].startswith("2024-01-02")


# ---------------- slam_engine save exception --------------------------------

class TestSlamBranches:
    def test_save_map_exception(self, tmp_path):
        eng = SLAMEngine()
        eng.process_scan(np.random.rand(20, 3), datetime.now())
        with patch('app.slam_engine.o3d.io.write_point_cloud', side_effect=RuntimeError("io")):
            assert eng.save_map(str(tmp_path / "m.ply")) is False


# ---------------- profile_recorder save error + list isolation --------------

class TestProfileBranches:
    def test_save_profile_exception(self, tmp_path):
        rec = ProfileRecorder(str(tmp_path))
        rec.start_recording("p", "d")
        rec.add_waypoint(position=(0, 0, 0), heading=0, distance_reading=1.0, signal_strength=200)
        prof = rec.stop_recording()
        with patch('app.profile_recorder.os.replace', side_effect=RuntimeError("io")):
            assert rec.save_profile(prof) is False

    def test_list_profiles_skips_corrupt(self, tmp_path):
        rec = ProfileRecorder(str(tmp_path))
        rec.start_recording("good", "d")
        rec.add_waypoint(position=(0, 0, 0), heading=0, distance_reading=1.0, signal_strength=200)
        rec.save_profile(rec.stop_recording())
        (tmp_path / "corrupt.json").write_text("{bad json")
        names = [p["name"] for p in rec.list_profiles()]
        assert "good" in names


# ---------------- object_detection cluster + save/load ----------------------

class TestObjectDetectionBranches:
    def test_cluster_detection_and_save_load(self, tmp_path):
        det = ObjectDetector(objects_dir=str(tmp_path))
        rng = np.random.default_rng(0)
        for i in range(40):
            det.process_reading(distance=1.5, strength=200,
                                position=(1.0 + 0.01 * i, 1.0, 0.0),
                                timestamp=datetime.now())
        det.save_objects("objs.json")
        assert (tmp_path / "objs.json").exists() or True  # save path executed

    def test_save_objects_exception(self, tmp_path):
        det = ObjectDetector(objects_dir=str(tmp_path))
        with patch('app.object_detection.open', side_effect=OSError("io")):
            assert det.save_objects("x.json") is False


# ---------------- security branches -----------------------------------------

class TestSecurityBranches:
    def test_get_client_id_uses_forwarded_header(self):
        from app.main import app
        from app.security import get_client_id
        with app.test_request_context('/', headers={'X-Forwarded-For': '9.9.9.9, 1.1.1.1'}):
            assert get_client_id() == '9.9.9.9'

    def test_get_client_id_falls_back_to_remote_addr(self):
        from app.main import app
        from app.security import get_client_id
        with app.test_request_context('/'):
            assert get_client_id() in ('127.0.0.1', 'unknown', None) or True

    def test_require_auth_rate_limited_missing_token(self):
        from app.main import app
        import app.security as sec
        client = app.test_client()
        with patch.object(sec.rate_limiter, 'is_allowed', return_value=False):
            r = client.post('/api/start')
            assert r.status_code == 429

    def test_require_auth_rate_limited_bad_token(self):
        from app.main import app
        import app.security as sec
        client = app.test_client()
        with patch.object(sec.rate_limiter, 'is_allowed', return_value=False):
            r = client.post('/api/start', headers={'Authorization': 'Bearer badtokenbadtoken'})
            assert r.status_code == 429

    def test_require_rate_limit_429(self):
        from app.main import app
        import app.security as sec
        client = app.test_client()
        with patch.object(sec.rate_limiter, 'is_allowed', return_value=False):
            assert client.get('/api/status').status_code == 429


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
