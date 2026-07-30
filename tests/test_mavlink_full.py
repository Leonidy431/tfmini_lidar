"""Coverage-fill tests for app/mavlink_imu.py connect/start/stop/read-loop/ingest."""

import sys
import os
import time
from unittest.mock import MagicMock, patch

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import app.mavlink_imu as mav_mod
from app.mavlink_imu import MAVLinkAttitudeReader, euler_to_quaternion


class TestConnectGuards:
    def test_connect_without_pymavlink(self):
        reader = MAVLinkAttitudeReader()
        with patch.object(mav_mod, 'PYMAVLINK_AVAILABLE', False):
            assert reader.connect() is False
            assert reader.last_error == "pymavlink not installed"

    def test_connect_success(self):
        reader = MAVLinkAttitudeReader()
        fake_conn = MagicMock()
        with patch.object(mav_mod, 'PYMAVLINK_AVAILABLE', True), \
             patch.object(mav_mod, 'mavutil', MagicMock(
                 mavlink_connection=MagicMock(return_value=fake_conn))):
            assert reader.connect() is True
            assert reader._mav is fake_conn

    def test_connect_failure(self):
        reader = MAVLinkAttitudeReader()
        bad = MagicMock()
        bad.mavlink_connection.side_effect = RuntimeError("no device")
        with patch.object(mav_mod, 'PYMAVLINK_AVAILABLE', True), \
             patch.object(mav_mod, 'mavutil', bad):
            assert reader.connect() is False
            assert "no device" in reader.last_error


class TestStartStop:
    def test_start_returns_false_when_connect_fails(self):
        reader = MAVLinkAttitudeReader()
        with patch.object(reader, 'connect', return_value=False):
            assert reader.start() is False

    def test_start_launches_thread_then_stop(self):
        reader = MAVLinkAttitudeReader()
        fake_conn = MagicMock()
        fake_conn.recv_match.return_value = None  # loop just idles
        with patch.object(reader, 'connect', return_value=True):
            reader._mav = fake_conn
            assert reader.start() is True
            time.sleep(0.05)
            reader.stop()
            assert reader._running is False


class TestIngest:
    def test_ingest_attitude_message(self):
        reader = MAVLinkAttitudeReader(timeout_s=5.0)
        msg = MagicMock()
        msg.get_type.return_value = "ATTITUDE"
        msg.roll, msg.pitch, msg.yaw = 0.1, -0.2, 0.3
        reader._ingest(msg)
        s = reader.get_attitude()
        assert abs(s.roll - 0.1) < 1e-9 and abs(s.yaw - 0.3) < 1e-9

    def test_ingest_attitude_quaternion_message(self):
        reader = MAVLinkAttitudeReader(timeout_s=5.0)
        x, y, z, w = euler_to_quaternion(0.05, 0.1, -0.15)
        msg = MagicMock()
        msg.get_type.return_value = "ATTITUDE_QUATERNION"
        msg.q1, msg.q2, msg.q3, msg.q4 = w, x, y, z  # wire order q1=w
        reader._ingest(msg)
        s = reader.get_attitude()
        assert abs(s.pitch - 0.1) < 1e-6

    def test_ingest_ignores_unknown_type(self):
        reader = MAVLinkAttitudeReader()
        msg = MagicMock()
        msg.get_type.return_value = "HEARTBEAT"
        reader._ingest(msg)
        assert reader.get_attitude() is None


class TestReadLoop:
    def test_read_loop_ingests_then_exits(self):
        reader = MAVLinkAttitudeReader(timeout_s=5.0)
        msg = MagicMock()
        msg.get_type.return_value = "ATTITUDE"
        msg.roll, msg.pitch, msg.yaw = 0.0, 0.0, 1.0
        fake_conn = MagicMock()
        # First call returns a message, subsequent return None
        fake_conn.recv_match.side_effect = [msg] + [None] * 100
        reader._mav = fake_conn
        reader._running = True
        import threading
        t = threading.Thread(target=reader._read_loop, daemon=True)
        t.start()
        time.sleep(0.05)
        reader._running = False
        t.join(timeout=1.0)
        assert reader.messages_received >= 1

    def test_read_loop_counts_parse_errors(self):
        reader = MAVLinkAttitudeReader()
        fake_conn = MagicMock()
        fake_conn.recv_match.side_effect = RuntimeError("bad frame")
        reader._mav = fake_conn
        reader._running = True
        import threading
        t = threading.Thread(target=reader._read_loop, daemon=True)
        t.start()
        time.sleep(0.05)
        reader._running = False
        t.join(timeout=1.0)
        assert reader.parse_errors >= 1


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
