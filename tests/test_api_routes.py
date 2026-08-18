"""
End-to-end REST + WebSocket route coverage for app/main.py.

Uses Flask's test client (and flask-socketio's test client) against the real
module-level `app`/`socketio`/`lidar_app`, with a known token registered in
security.API_TOKENS and the hardware driver mocked so start/stop exercise the
real code paths without a serial port.
"""

import sys
import os
import time
from datetime import datetime
from unittest.mock import MagicMock

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app import main as main_mod
from app.main import app, socketio, lidar_app, LiDARSLAMApplication
from app.security import API_TOKENS, hash_token
from app.lidar_driver import LiDARReading

TOKEN = "test-token-abcdefghijklmnop"
AUTH = {"Authorization": f"Bearer {TOKEN}"}


@pytest.fixture(autouse=True)
def _register_token():
    API_TOKENS[hash_token(TOKEN)] = {"name": "test", "created": time.time(),
                                     "permissions": ["all"]}
    yield


@pytest.fixture
def client():
    app.config["TESTING"] = True
    return app.test_client()


@pytest.fixture(autouse=True)
def _reset_app_state():
    """Return lidar_app to a clean IDLE/stopped state around each test."""
    lidar_app.mode = LiDARSLAMApplication.MODE_IDLE
    lidar_app.is_running = False
    lidar_app.slam_engine.clear()
    yield
    lidar_app.is_running = False
    lidar_app.mode = LiDARSLAMApplication.MODE_IDLE


def _mock_driver():
    drv = MagicMock()
    drv.is_connected = True
    drv.get_statistics.return_value = {
        "error_rate": 0.0, "reconnect_attempts": 0,
        "seconds_since_last_read": 0.1, "last_error": None,
    }
    return drv


# --------------------------------------------------------------------------
# Public / unauthenticated routes
# --------------------------------------------------------------------------

class TestPublicRoutes:
    def test_index(self, client):
        assert client.get("/").status_code == 200

    def test_health_ok(self, client):
        r = client.get("/api/health")
        assert r.status_code in (200, 503)
        assert "status" in r.get_json()

    def test_health_failed_returns_503(self, client):
        lidar_app.driver = MagicMock(is_connected=False)
        lidar_app.driver.get_statistics.return_value = {"error_rate": 0.0}
        lidar_app.is_running = True
        try:
            r = client.get("/api/health")
            assert r.status_code == 503
            assert r.get_json()["status"] == "failed"
        finally:
            lidar_app.driver = None
            lidar_app.is_running = False

    def test_register_service(self, client):
        assert client.get("/api/register_service").get_json()["name"]

    def test_status(self, client):
        body = client.get("/api/status").get_json()
        assert "mode" in body and "health" in body and "sensor_fusion" in body

    def test_versioned_alias(self, client):
        """/api/v1/<x> aliases must resolve (API versioning)."""
        assert client.get("/api/v1/status").status_code == 200


# --------------------------------------------------------------------------
# Auth enforcement
# --------------------------------------------------------------------------

class TestAuth:
    def test_missing_token_401(self, client):
        assert client.post("/api/start").status_code == 401

    def test_bad_token_401(self, client):
        r = client.post("/api/start", headers={"Authorization": "Bearer nope-nope-nope-nope"})
        assert r.status_code == 401

    def test_x_api_key_header_accepted(self, client):
        lidar_app.driver = _mock_driver()
        try:
            r = client.post("/api/stop", headers={"X-API-Key": TOKEN})
            assert r.status_code == 200
        finally:
            lidar_app.driver = None


# --------------------------------------------------------------------------
# Lifecycle: start / stop / mode
# --------------------------------------------------------------------------

class TestLifecycle:
    def test_start_and_stop(self, client):
        lidar_app.driver = _mock_driver()
        try:
            r = client.post("/api/start", headers=AUTH)
            assert r.status_code == 200 and r.get_json()["success"] is True
            assert lidar_app.is_running is True
            r = client.post("/api/stop", headers=AUTH)
            assert r.get_json()["success"] is True
            assert lidar_app.is_running is False
        finally:
            lidar_app.driver = None

    def test_set_mode_valid(self, client):
        r = client.post("/api/mode/mapping", headers=AUTH)
        assert r.status_code == 200 and r.get_json()["mode"] == "mapping"

    def test_set_mode_invalid(self, client):
        r = client.post("/api/mode/bogus", headers=AUTH)
        assert r.status_code == 400 and r.get_json()["success"] is False


# --------------------------------------------------------------------------
# Mapping
# --------------------------------------------------------------------------

class TestMapping:
    def test_start_stop_clear(self, client):
        assert client.post("/api/mapping/start", headers=AUTH).get_json()["success"]
        assert lidar_app.mode == "mapping"
        assert client.post("/api/mapping/stop", headers=AUTH).get_json()["success"]
        assert client.post("/api/mapping/clear", headers=AUTH).get_json()["success"]

    def test_statistics_points_trajectory(self, client):
        assert client.get("/api/mapping/statistics").status_code == 200
        assert "points" in client.get("/api/mapping/points").get_json()
        assert "trajectory" in client.get("/api/mapping/trajectory").get_json()

    def test_points_after_scan(self, client):
        lidar_app.slam_engine.process_scan(np.random.rand(60, 3), datetime.now())
        body = client.get("/api/mapping/points").get_json()
        assert body["count"] > 0


# --------------------------------------------------------------------------
# Maps CRUD
# --------------------------------------------------------------------------

class TestMaps:
    def test_list_empty(self, client):
        assert isinstance(client.get("/api/maps").get_json(), list)

    def test_save_requires_map_data(self, client):
        r = client.post("/api/maps/emptymap/save", headers=AUTH, json={})
        assert r.status_code == 400  # no map data

    def test_save_load_info_delete_roundtrip(self, client):
        lidar_app.slam_engine.process_scan(np.random.rand(80, 3), datetime.now())
        r = client.post("/api/maps/rt_map/save", headers=AUTH,
                        json={"description": "d", "tags": ["t"]})
        assert r.get_json()["success"] is True

        assert client.get("/api/maps/rt_map").status_code == 200
        assert client.post("/api/maps/rt_map/load", headers=AUTH).get_json()["success"]
        assert lidar_app.mode == "localizing"
        assert client.delete("/api/maps/rt_map/delete", headers=AUTH).get_json()["success"]

    def test_invalid_name_rejected(self, client):
        assert client.get("/api/maps/..%2Fetc").status_code in (400, 404)
        assert client.post("/api/maps/bad$name/save", headers=AUTH, json={}).status_code == 400
        assert client.post("/api/maps/bad$name/load", headers=AUTH).status_code == 400
        assert client.delete("/api/maps/bad$name/delete", headers=AUTH).status_code == 400

    def test_info_not_found(self, client):
        assert client.get("/api/maps/does_not_exist").status_code == 404

    def test_load_not_found(self, client):
        assert client.post("/api/maps/nope/load", headers=AUTH).status_code == 404

    def test_delete_not_found(self, client):
        assert client.delete("/api/maps/nope/delete", headers=AUTH).status_code == 404


# --------------------------------------------------------------------------
# Localization
# --------------------------------------------------------------------------

class TestLocalization:
    def test_position(self, client):
        assert client.get("/api/localization/position").status_code == 200

    def test_reset(self, client):
        assert client.post("/api/localization/reset", headers=AUTH).get_json()["success"]


# --------------------------------------------------------------------------
# Profiles
# --------------------------------------------------------------------------

class TestProfiles:
    def test_list(self, client):
        assert isinstance(client.get("/api/profiles").get_json(), list)

    def test_record_start_stop(self, client):
        r = client.post("/api/profiles/prof_rt/record/start", headers=AUTH,
                        json={"description": "d"})
        assert r.get_json()["success"] is True
        assert lidar_app.mode == "recording"
        r = client.post("/api/profiles/record/stop", headers=AUTH)
        assert r.get_json()["success"] is True

    def test_record_invalid_name(self, client):
        assert client.post("/api/profiles/bad$name/record/start", headers=AUTH,
                           json={}).status_code == 400

    def test_navigate_not_found(self, client):
        assert client.post("/api/profiles/nope/navigate/start", headers=AUTH).status_code == 404

    def test_navigate_invalid_name(self, client):
        assert client.post("/api/profiles/bad$name/navigate/start", headers=AUTH).status_code == 400

    def test_navigate_roundtrip(self, client):
        client.post("/api/profiles/nav_rt/record/start", headers=AUTH, json={})
        # add a couple waypoints so the profile is navigable
        lidar_app.profile_recorder.add_waypoint(position=(0, 0, 0), heading=0,
                                                 distance_reading=1.0, signal_strength=200)
        lidar_app.profile_recorder.add_waypoint(position=(1, 0, 0), heading=0,
                                                 distance_reading=1.0, signal_strength=200)
        client.post("/api/profiles/record/stop", headers=AUTH)
        r = client.post("/api/profiles/nav_rt/navigate/start", headers=AUTH)
        assert r.status_code == 200
        assert client.post("/api/profiles/navigate/stop", headers=AUTH).get_json()["success"]

    def test_delete_not_found(self, client):
        assert client.delete("/api/profiles/nope/delete", headers=AUTH).status_code == 404

    def test_delete_invalid_name(self, client):
        assert client.delete("/api/profiles/bad$name/delete", headers=AUTH).status_code == 400


# --------------------------------------------------------------------------
# Objects
# --------------------------------------------------------------------------

class TestObjects:
    def test_get(self, client):
        body = client.get("/api/objects").get_json()
        assert "objects" in body and "statistics" in body

    def test_nearby(self, client):
        r = client.get("/api/objects/nearby?x=0&y=0&z=0&radius=5")
        assert r.status_code == 200 and "objects" in r.get_json()

    def test_nearby_invalid_coords(self, client):
        assert client.get("/api/objects/nearby?x=abc").status_code == 400

    def test_clear(self, client):
        assert client.post("/api/objects/clear", headers=AUTH).get_json()["success"]

    def test_save(self, client):
        assert "success" in client.post("/api/objects/save", headers=AUTH).get_json()


# --------------------------------------------------------------------------
# 3D Scanner
# --------------------------------------------------------------------------

class TestScanner:
    def test_start_default(self, client):
        r = client.post("/api/scanner/start", headers=AUTH, json={})
        assert r.get_json()["success"] is True
        assert lidar_app.mode == "scanning"

    def test_start_with_params(self, client):
        r = client.post("/api/scanner/start", headers=AUTH,
                        json={"center": [1.0, 2.0, 0.0], "orbit_radius": 3.0, "initial_z": 0.5})
        assert r.get_json()["success"] is True

    def test_start_bad_center(self, client):
        assert client.post("/api/scanner/start", headers=AUTH,
                           json={"center": [1, 2]}).status_code == 400

    def test_start_bad_radius(self, client):
        assert client.post("/api/scanner/start", headers=AUTH,
                           json={"orbit_radius": 99.0}).status_code == 400
        assert client.post("/api/scanner/start", headers=AUTH,
                           json={"orbit_radius": "x"}).status_code == 400

    def test_start_bad_initial_z(self, client):
        assert client.post("/api/scanner/start", headers=AUTH,
                           json={"initial_z": "x"}).status_code == 400

    def test_start_rejects_non_finite_center(self, client):
        """Regression test for Blind Spot Audit R3 R3-SEC-8: isinstance
        check alone accepts NaN/Infinity (Python's json module parses those
        non-standard literals by default), silently poisoning the
        accumulated point cloud / saved map bounds."""
        for bad_center in (
            [float('nan'), 0.0, 0.0],
            [0.0, float('inf'), 0.0],
            [0.0, 0.0, float('-inf')],
        ):
            r = client.post("/api/scanner/start", headers=AUTH,
                            json={"center": bad_center})
            assert r.status_code == 400, bad_center

    def test_start_rejects_non_finite_initial_z(self, client):
        r = client.post("/api/scanner/start", headers=AUTH,
                        json={"initial_z": float('nan')})
        assert r.status_code == 400

    def test_stop_status_points_clear(self, client):
        client.post("/api/scanner/start", headers=AUTH, json={})
        assert client.post("/api/scanner/stop", headers=AUTH).get_json()["success"]
        assert client.get("/api/scanner/status").status_code == 200
        assert "points" in client.get("/api/scanner/points").get_json()
        assert client.post("/api/scanner/clear", headers=AUTH).get_json()["success"]

    def test_layer(self, client):
        r = client.post("/api/scanner/layer", headers=AUTH, json={"z": 1.5})
        assert r.get_json()["current_z"] == 1.5

    def test_layer_missing_z(self, client):
        assert client.post("/api/scanner/layer", headers=AUTH, json={}).status_code == 400

    def test_save_no_data(self, client):
        client.post("/api/scanner/clear", headers=AUTH)
        assert client.post("/api/scanner/save/emptyscan", headers=AUTH, json={}).status_code == 400

    def test_save_invalid_name(self, client):
        assert client.post("/api/scanner/save/bad$name", headers=AUTH, json={}).status_code == 400

    def test_save_roundtrip(self, client):
        client.post("/api/scanner/start", headers=AUTH, json={})
        # distance must be < orbit_radius (default 3.0m) or it's rejected as
        # 'beyond_center'.
        for _ in range(30):
            lidar_app.scanner.add_reading(distance=2.0, heading_deg=float(_ * 12),
                                          signal_strength=200)
        r = client.post("/api/scanner/save/scan_rt", headers=AUTH, json={"description": "s"})
        assert r.status_code == 200
        lidar_app.map_manager.delete_map("scan_rt")


# --------------------------------------------------------------------------
# WebSocket handlers
# --------------------------------------------------------------------------

class TestWebSocket:
    def test_connect_emits_status(self):
        c = socketio.test_client(app)
        assert c.is_connected()
        received = c.get_received()
        assert any(m["name"] == "status" for m in received)
        c.disconnect()

    def test_get_status_event(self):
        c = socketio.test_client(app)
        c.get_received()  # drain connect
        c.emit("get_status")
        assert any(m["name"] == "status" for m in c.get_received())
        c.disconnect()

    def test_get_map_points_event(self):
        lidar_app.slam_engine.process_scan(np.random.rand(40, 3), datetime.now())
        c = socketio.test_client(app)
        c.get_received()
        c.emit("get_map_points")
        assert any(m["name"] == "map_points" for m in c.get_received())
        c.disconnect()


class TestRequestSizeLimit:
    """Regression test for Blind Spot Audit R3 R3-SEC-7: no
    MAX_CONTENT_LENGTH was set, so any POST route would buffer an
    arbitrarily large body into memory before parsing."""

    def test_max_content_length_configured(self):
        assert app.config.get("MAX_CONTENT_LENGTH") is not None
        assert app.config["MAX_CONTENT_LENGTH"] <= 4 * 1024 * 1024

    def test_oversized_body_rejected(self, client):
        limit = app.config["MAX_CONTENT_LENGTH"]
        oversized = b"{" + b'"description": "' + b"x" * (limit + 1024) + b'"}'
        r = client.post(
            "/api/maps/some-map/save",
            headers={**AUTH, "Content-Type": "application/json"},
            data=oversized,
        )
        assert r.status_code == 413


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
