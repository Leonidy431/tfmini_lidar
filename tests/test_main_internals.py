"""Coverage-fill tests for LiDARSLAMApplication internals: initialize,
driver-error safety, processing thread, broadcasts, health branches, main()."""

import sys
import os
import time
import queue
from datetime import datetime
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import app.main as main_mod
from app.main import LiDARSLAMApplication
from app.lidar_driver import LiDARReading


def make_reading(distance=2.0, strength=200, mono_ts=None):
    return LiDARReading(distance=distance, signal_strength=strength,
                        timestamp=datetime.now(), temperature=20.0, valid=True,
                        mono_timestamp=mono_ts or time.monotonic())


class TestInitialize:
    def test_initialize_success(self):
        app = LiDARSLAMApplication()
        fake = MagicMock()
        fake.connect.return_value = True
        with patch('app.main.TFminiSDriver', return_value=fake):
            assert app.initialize() is True
            fake.set_framerate.assert_called_once()
            assert fake.add_callback.called

    def test_initialize_connect_fails(self):
        app = LiDARSLAMApplication()
        fake = MagicMock()
        fake.connect.return_value = False
        with patch('app.main.TFminiSDriver', return_value=fake):
            assert app.initialize() is False


class TestStartStop:
    def test_start_initializes_when_no_driver(self):
        app = LiDARSLAMApplication()
        fake = MagicMock()
        fake.connect.return_value = True
        with patch('app.main.TFminiSDriver', return_value=fake):
            assert app.start() is True
            assert app.is_running is True
        app.stop()
        assert app.is_running is False

    def test_start_already_running(self):
        app = LiDARSLAMApplication()
        app.is_running = True
        assert app.start() is True  # early-return branch

    def test_start_init_failure_returns_false(self):
        app = LiDARSLAMApplication()
        with patch.object(app, 'initialize', return_value=False):
            assert app.start() is False

    def test_start_exception_path(self):
        app = LiDARSLAMApplication()
        app.driver = MagicMock()
        app.driver.start.side_effect = RuntimeError("boom")
        assert app.start() is False


class TestDriverErrorSafety:
    def test_sensor_failure_forces_idle_and_alarms(self):
        app = LiDARSLAMApplication()
        app.mode = app.MODE_MAPPING
        app.is_running = True
        app.driver = MagicMock(is_connected=False)
        app.driver.get_statistics.return_value = {"error_rate": 1.0}
        events = []
        app.websocket_callback = lambda name, data: events.append(name)

        app._on_driver_error("serial", RuntimeError("cable"))

        assert app.mode == app.MODE_IDLE
        assert "safety_alarm" in events
        assert "driver_error" in events

    def test_error_without_failure_state_just_notifies(self):
        app = LiDARSLAMApplication()
        app.mode = app.MODE_IDLE
        app.driver = MagicMock(is_connected=True)
        app.driver.get_statistics.return_value = {"error_rate": 0.0,
                                                  "reconnect_attempts": 0,
                                                  "seconds_since_last_read": 0.1}
        events = []
        app.websocket_callback = lambda name, data: events.append(name)
        app._on_driver_error("read_loop", RuntimeError("x"))
        assert events == ["driver_error"]


class TestProcessingThread:
    def test_queue_drained_and_processed(self):
        app = LiDARSLAMApplication()
        app.mode = app.MODE_MAPPING
        processed = []
        app._process_reading = lambda r: processed.append(r)
        app._start_processing_thread()
        app._processing_queue.put(make_reading())
        time.sleep(0.1)
        app._stop_processing_thread()
        assert len(processed) >= 1

    def test_start_processing_thread_idempotent(self):
        app = LiDARSLAMApplication()
        app._start_processing_thread()
        t1 = app._processing_thread
        app._start_processing_thread()  # already alive -> no new thread
        assert app._processing_thread is t1
        app._stop_processing_thread()

    def test_processing_loop_swallows_errors(self):
        app = LiDARSLAMApplication()
        def boom(r): raise ValueError("bad")
        app._process_reading = boom
        app._start_processing_thread()
        app._processing_queue.put(make_reading())
        time.sleep(0.1)
        app._stop_processing_thread()  # must not raise


class TestBroadcastsPerMode:
    def _app_with_cb(self, mode):
        app = LiDARSLAMApplication()
        app.mode = mode
        app.events = []
        app.websocket_callback = lambda name, data: app.events.append(name)
        return app

    def test_mapping_broadcast(self):
        app = self._app_with_cb(LiDARSLAMApplication.MODE_MAPPING)
        app._process_reading(make_reading())
        assert "lidar_reading" in app.events

    def test_localizing_broadcast(self):
        app = self._app_with_cb(LiDARSLAMApplication.MODE_LOCALIZING)
        app.localization_engine.set_reference_map(np.random.rand(200, 3) * 5)
        app.localization_engine.buffer_size = 5
        for _ in range(8):
            app._process_reading(make_reading())
        assert "lidar_reading" in app.events

    def test_recording_broadcast(self):
        app = self._app_with_cb(LiDARSLAMApplication.MODE_RECORDING)
        app.profile_recorder.start_recording("p", "d")
        app._process_reading(make_reading())
        assert "lidar_reading" in app.events

    def test_navigating_broadcast(self):
        app = self._app_with_cb(LiDARSLAMApplication.MODE_NAVIGATING)
        app._process_reading(make_reading())
        assert "navigation" in app.events or "lidar_reading" in app.events

    def test_scanning_broadcast(self):
        app = self._app_with_cb(LiDARSLAMApplication.MODE_SCANNING)
        app.scanner.start_scan(center=(0, 0, 0), orbit_radius=3.0, initial_z=0.0)
        for h in range(0, 360, 5):
            app._process_reading(make_reading(distance=2.0))
        assert "lidar_reading" in app.events


class TestHealthBranches:
    def test_temperature_out_of_range(self):
        app = LiDARSLAMApplication()
        app.driver = MagicMock(is_connected=True)
        app.driver.get_statistics.return_value = {"error_rate": 0.0,
                                                  "reconnect_attempts": 0,
                                                  "seconds_since_last_read": 0.1}
        app.last_reading = make_reading()
        app.last_reading.temperature = 80.0  # above IEC envelope
        health = app.get_health()
        assert "temperature_out_of_range" in health["reasons"]

    def test_heading_missing_in_mapping(self):
        app = LiDARSLAMApplication()
        app.mode = app.MODE_MAPPING
        app.driver = MagicMock(is_connected=True)
        app.driver.get_statistics.return_value = {"error_rate": 0.0,
                                                  "reconnect_attempts": 0,
                                                  "seconds_since_last_read": 0.1}
        assert "no_heading_source" in app.get_health()["reasons"]

    def test_reconnecting_and_high_error(self):
        app = LiDARSLAMApplication()
        app.driver = MagicMock(is_connected=True)
        app.driver.get_statistics.return_value = {"error_rate": 0.5,
                                                  "reconnect_attempts": 2,
                                                  "seconds_since_last_read": 0.1}
        health = app.get_health()
        assert "reconnecting" in health["reasons"]
        assert "high_error_rate" in health["reasons"]


class TestSetModeTransitions:
    def test_recording_to_idle_stops_recorder(self):
        app = LiDARSLAMApplication()
        app.profile_recorder.start_recording("p", "d")
        app.set_mode(app.MODE_RECORDING)
        app.set_mode(app.MODE_IDLE)
        assert app.profile_recorder.is_recording is False

    def test_scanning_to_idle_stops_scanner(self):
        app = LiDARSLAMApplication()
        app.scanner.start_scan(center=(0, 0, 0), orbit_radius=3.0, initial_z=0.0)
        app.set_mode(app.MODE_SCANNING)
        app.set_mode(app.MODE_IDLE)
        assert app.scanner.is_scanning is False


class TestWsAuthConnect:
    def test_connect_rejected_without_token(self):
        with patch.object(main_mod, "REQUIRE_WS_AUTH", True):
            client = main_mod.socketio.test_client(main_mod.app)
            assert client.is_connected() is False

    def test_connect_accepted_with_valid_token(self):
        from app.security import API_TOKENS, hash_token
        tok = "wsvalidtoken1234567890"
        API_TOKENS[hash_token(tok)] = {"name": "ws"}
        with patch.object(main_mod, "REQUIRE_WS_AUTH", True):
            client = main_mod.socketio.test_client(main_mod.app, auth={"token": tok})
            assert client.is_connected() is True
            client.disconnect()


class TestMainEntry:
    def test_main_runs_server(self):
        with patch.object(main_mod.lidar_app, "initialize", return_value=True), \
             patch.object(main_mod, "init_default_token", return_value="tok"), \
             patch.object(main_mod.socketio, "run") as mock_run:
            main_mod.main()
            mock_run.assert_called_once()

    def test_main_demo_mode_when_init_fails(self):
        with patch.object(main_mod.lidar_app, "initialize", return_value=False), \
             patch.object(main_mod, "init_default_token", return_value="tok"), \
             patch.object(main_mod.socketio, "run") as mock_run:
            main_mod.main()
            mock_run.assert_called_once()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
