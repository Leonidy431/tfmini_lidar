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

    def test_start_calls_mavlink_attitude_start(self):
        """Regression test for Blind Spot Audit R3 R3-REL-1:
        MAVLinkAttitudeReader.start()/.stop() were never called anywhere in
        main.py, so D1 3D-attitude fusion was a silent no-op even when
        explicitly enabled."""
        app = LiDARSLAMApplication()
        fake_driver = MagicMock()
        fake_driver.connect.return_value = True
        fake_mavlink = MagicMock()
        fake_mavlink.start.return_value = True
        app.mavlink_attitude = fake_mavlink
        with patch('app.main.TFminiSDriver', return_value=fake_driver):
            assert app.start() is True
        fake_mavlink.start.assert_called_once()
        app.stop()
        fake_mavlink.stop.assert_called_once()

    def test_start_continues_when_mavlink_attitude_fails(self):
        """start() must degrade gracefully (1D heading only) rather than
        aborting the whole application when the MAVLink source fails."""
        app = LiDARSLAMApplication()
        fake_driver = MagicMock()
        fake_driver.connect.return_value = True
        fake_mavlink = MagicMock()
        fake_mavlink.start.return_value = False
        fake_mavlink.last_error = "connection refused"
        app.mavlink_attitude = fake_mavlink
        with patch('app.main.TFminiSDriver', return_value=fake_driver):
            assert app.start() is True
        assert app.is_running is True


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


class TestAttitudeAndEKFHealthSignals:
    """Regression tests for Blind Spot Audit R3 R3-COMP-2/R3-COMP-3, see
    docs/ALGORITHM_DECISION_LOG.md Decision 4."""

    def _idle_driver(self, app):
        app.driver = MagicMock(is_connected=True)
        app.driver.get_statistics.return_value = {"error_rate": 0.0,
                                                  "reconnect_attempts": 0,
                                                  "seconds_since_last_read": 0.1}

    def test_attitude_3d_counts_as_heading_source(self):
        """A deployment using only D1 (never calling the legacy
        set_heading()) must not report no_heading_source while 3D attitude
        is actively driving beam projection."""
        app = LiDARSLAMApplication()
        app.mode = app.MODE_MAPPING
        self._idle_driver(app)
        app.mavlink_attitude = MagicMock()
        app.mavlink_attitude.get_attitude.return_value = MagicMock(roll=0.0, pitch=0.0, yaw=0.0)
        app._project_beam(1.0, (0.0, 0.0, 0.0))  # observes the attitude sample
        health = app.get_health()
        assert "no_heading_source" not in health["reasons"]
        assert "attitude_3d_lost" not in health["reasons"]

    def test_attitude_3d_lost_after_transition(self):
        """3D attitude active, then drops (mid-mission MAVLink dropout) ->
        distinct attitude_3d_lost reason, not conflated with
        no_heading_source (which means 'never had any heading source')."""
        app = LiDARSLAMApplication()
        app.mode = app.MODE_MAPPING
        self._idle_driver(app)
        app.mavlink_attitude = MagicMock()
        app.mavlink_attitude.get_attitude.return_value = MagicMock(roll=0.0, pitch=0.0, yaw=0.0)
        app._project_beam(1.0, (0.0, 0.0, 0.0))  # attitude active once

        app.mavlink_attitude.get_attitude.return_value = None  # then drops
        health = app.get_health()
        assert "attitude_3d_lost" in health["reasons"]
        assert "no_heading_source" not in health["reasons"]

    def test_no_attitude_3d_lost_when_never_configured(self):
        """No false positive: attitude_3d_lost must not fire for a build
        that never had a MAVLink attitude source at all."""
        app = LiDARSLAMApplication()
        app.mode = app.MODE_MAPPING
        self._idle_driver(app)
        assert app.mavlink_attitude is None
        health = app.get_health()
        assert "attitude_3d_lost" not in health["reasons"]

    def test_no_attitude_3d_lost_on_first_read_without_prior_activity(self):
        """attitude_3d_lost requires an active->inactive TRANSITION, not
        just 'currently inactive' -- must not fire on the very first health
        check before _project_beam has ever run."""
        app = LiDARSLAMApplication()
        app.mode = app.MODE_MAPPING
        self._idle_driver(app)
        app.mavlink_attitude = MagicMock()
        app.mavlink_attitude.get_attitude.return_value = None
        health = app.get_health()
        assert "attitude_3d_lost" not in health["reasons"]

    def test_ekf_diverged_above_threshold(self):
        app = LiDARSLAMApplication()
        self._idle_driver(app)
        app.ekf = MagicMock()
        app.ekf.get_statistics.return_value = {"covariance_trace": 999.0}
        health = app.get_health()
        assert "ekf_diverged" in health["reasons"]
        assert health["state"] == "degraded"

    def test_ekf_not_diverged_below_threshold(self):
        app = LiDARSLAMApplication()
        self._idle_driver(app)
        app.ekf = MagicMock()
        app.ekf.get_statistics.return_value = {"covariance_trace": 0.5}
        health = app.get_health()
        assert "ekf_diverged" not in health["reasons"]

    def test_ekf_diverged_absent_when_ekf_disabled(self):
        app = LiDARSLAMApplication()
        self._idle_driver(app)
        assert app.ekf is None
        health = app.get_health()
        assert "ekf_diverged" not in health["reasons"]


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

    def test_concurrent_set_mode_calls_do_not_skip_cleanup(self):
        """Regression test for Blind Spot Audit R3 R3-TEST-2/R3-CONC-5:
        set_mode()'s read-check-transition-write sequence had no lock
        spanning it, so two threads racing here could both observe a stale
        self.mode and one transition's cleanup (stop_recording/
        stop_navigation/stop_scan) would silently never run."""
        import concurrent.futures

        app = LiDARSLAMApplication()
        app.profile_recorder.start_recording("p", "d")
        app.set_mode(app.MODE_RECORDING)

        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            futures = [pool.submit(app.set_mode, app.MODE_IDLE) for _ in range(2)]
            for f in futures:
                f.result()

        assert app.profile_recorder.is_recording is False
        assert app.mode == app.MODE_IDLE


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

    def test_main_registers_sigterm_handler(self):
        """Regression test for Blind Spot Audit R3 R3-DEVOPS-2: Docker's
        default SIGTERM previously hit Python's default handler and killed
        the process with no chance to close the UART port or flush
        in-progress state."""
        import signal as signal_mod
        with patch.object(main_mod.lidar_app, "initialize", return_value=True), \
             patch.object(main_mod, "init_default_token", return_value="tok"), \
             patch.object(main_mod.socketio, "run"):
            main_mod.main()
            assert signal_mod.getsignal(signal_mod.SIGTERM) is main_mod._handle_shutdown_signal

    def test_shutdown_signal_handler_stops_app_and_exits(self):
        with patch.object(main_mod.lidar_app, "stop") as mock_stop:
            with pytest.raises(SystemExit):
                main_mod._handle_shutdown_signal(15, None)
            mock_stop.assert_called_once()

    def test_shutdown_signal_handler_exits_even_if_stop_raises(self):
        """Shutdown must still terminate the process even if stop() itself
        errors -- must not hang past the container's stop grace period."""
        with patch.object(main_mod.lidar_app, "stop", side_effect=RuntimeError("boom")):
            with pytest.raises(SystemExit):
                main_mod._handle_shutdown_signal(15, None)


class TestWerkzeugDebuggerGate:
    """Regression tests for Blind Spot Audit R3 R3-SEC-1: the Werkzeug
    interactive debugger (allow_unsafe_werkzeug) was previously tied to the
    same DEBUG flag as app logging, and WEB_HOST defaults to 0.0.0.0 --
    setting DEBUG=true for field troubleshooting would expose unauthenticated
    RCE to the whole LAN."""

    def test_debugger_disabled_by_default(self):
        with patch.object(main_mod.lidar_app, "initialize", return_value=True), \
             patch.object(main_mod, "init_default_token", return_value="tok"), \
             patch.object(main_mod.Config, "ALLOW_WERKZEUG_DEBUGGER", False), \
             patch.object(main_mod.socketio, "run") as mock_run:
            main_mod.main()
            assert mock_run.call_args.kwargs["allow_unsafe_werkzeug"] is False

    def test_debugger_refused_when_host_not_loopback(self):
        with patch.object(main_mod.lidar_app, "initialize", return_value=True), \
             patch.object(main_mod, "init_default_token", return_value="tok"), \
             patch.object(main_mod.Config, "ALLOW_WERKZEUG_DEBUGGER", True), \
             patch.object(main_mod.Config, "WEB_HOST", "0.0.0.0"), \
             patch.object(main_mod.socketio, "run") as mock_run:
            main_mod.main()
            assert mock_run.call_args.kwargs["allow_unsafe_werkzeug"] is False

    def test_debugger_enabled_when_explicitly_opted_in_and_loopback(self):
        with patch.object(main_mod.lidar_app, "initialize", return_value=True), \
             patch.object(main_mod, "init_default_token", return_value="tok"), \
             patch.object(main_mod.Config, "ALLOW_WERKZEUG_DEBUGGER", True), \
             patch.object(main_mod.Config, "WEB_HOST", "127.0.0.1"), \
             patch.object(main_mod.socketio, "run") as mock_run:
            main_mod.main()
            assert mock_run.call_args.kwargs["allow_unsafe_werkzeug"] is True


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
