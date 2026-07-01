"""
Tests for the TFmini-S driver using a mocked serial port.

These exercise the read loop, frame parsing over a byte stream, and the
auto-reconnection logic without any real hardware.
"""

import sys
import os
import time
import threading
from unittest.mock import MagicMock, patch

import pytest
import serial

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.lidar_driver import TFminiSDriver, LiDARReading


def build_frame(distance_cm: int, strength: int = 200, temp_raw: int = 400) -> bytes:
    """Build a valid 9-byte TFmini-S frame with correct checksum."""
    frame = bytes([
        0x59, 0x59,
        distance_cm & 0xFF, (distance_cm >> 8) & 0xFF,
        strength & 0xFF, (strength >> 8) & 0xFF,
        temp_raw & 0xFF, (temp_raw >> 8) & 0xFF,
    ])
    checksum = sum(frame[:8]) & 0xFF
    return frame + bytes([checksum])


class FakeSerial:
    """
    Minimal stand-in for serial.Serial that yields a scripted byte stream.

    Set `fail_after` to raise SerialException after N reads to simulate a
    disconnect.
    """

    def __init__(self, stream: bytes = b'', fail_after=None):
        self._stream = bytearray(stream)
        self.is_open = True
        self.fail_after = fail_after
        self._reads = 0
        self.written = bytearray()

    @property
    def in_waiting(self):
        if self.fail_after is not None and self._reads >= self.fail_after:
            raise serial.SerialException("simulated disconnect")
        return len(self._stream)

    def read(self, n):
        self._reads += 1
        chunk = bytes(self._stream[:n])
        del self._stream[:n]
        return chunk

    def write(self, data):
        self.written.extend(data)
        return len(data)

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass

    def close(self):
        self.is_open = False

    def feed(self, data: bytes):
        self._stream.extend(data)


class TestFrameStreamParsing:
    def test_process_buffer_single_frame(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        readings = []
        driver.add_callback(lambda r: readings.append(r))

        driver.buffer.extend(build_frame(150))
        driver._process_buffer()

        assert len(readings) == 1
        assert readings[0].distance == 1.5

    def test_process_buffer_with_leading_garbage(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        readings = []
        driver.add_callback(lambda r: readings.append(r))

        driver.buffer.extend(b'\x00\x11\x22' + build_frame(200))
        driver._process_buffer()

        assert len(readings) == 1
        assert readings[0].distance == 2.0

    def test_process_buffer_multiple_frames(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        readings = []
        driver.add_callback(lambda r: readings.append(r))

        driver.buffer.extend(build_frame(100) + build_frame(300) + build_frame(500))
        driver._process_buffer()

        assert len(readings) == 3
        assert [round(r.distance, 1) for r in readings] == [1.0, 3.0, 5.0]

    def test_callback_exception_isolated(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        good = []

        def bad_cb(r):
            raise ValueError("boom")

        driver.add_callback(bad_cb)
        driver.add_callback(lambda r: good.append(r))

        driver.buffer.extend(build_frame(150))
        driver._process_buffer()

        # Second callback still runs despite first raising
        assert len(good) == 1


class TestReadLoopWithMock:
    def test_read_loop_produces_readings(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.reconnect_enabled = False
        readings = []
        driver.add_callback(lambda r: readings.append(r))

        fake = FakeSerial(build_frame(150) * 5)
        driver.serial_conn = fake
        driver.is_running = True

        t = threading.Thread(target=driver._read_loop, daemon=True)
        t.start()
        time.sleep(0.2)
        driver.is_running = False
        t.join(timeout=1.0)

        assert len(readings) >= 1
        assert readings[0].distance == 1.5

    def test_connection_error_triggers_cleanup(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.reconnect_enabled = False

        fake = FakeSerial(build_frame(150), fail_after=1)
        driver.serial_conn = fake
        driver.is_running = True

        t = threading.Thread(target=driver._read_loop, daemon=True)
        t.start()
        time.sleep(0.2)
        driver.is_running = False
        t.join(timeout=1.0)

        # After a serial failure the connection is dropped
        assert driver.serial_conn is None


class TestReconnection:
    def test_reconnect_success(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.reconnect_delay_base = 0.001
        driver.reconnect_delay_max = 0.01

        with patch.object(driver, 'connect', return_value=True) as mock_connect:
            driver._attempt_reconnect()
            mock_connect.assert_called_once()
            assert driver.reconnect_attempts == 0  # reset on success

    def test_reconnect_backoff_increments(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.reconnect_delay_base = 0.001
        driver.reconnect_delay_max = 0.01

        with patch.object(driver, 'connect', return_value=False):
            driver._attempt_reconnect()
            assert driver.reconnect_attempts == 1
            driver._attempt_reconnect()
            assert driver.reconnect_attempts == 2

    def test_max_attempts_resets(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.reconnect_delay_max = 0.01
        driver.max_reconnect_attempts = 3
        driver.reconnect_attempts = 3

        with patch.object(driver, 'connect', return_value=False):
            driver._attempt_reconnect()
            # Hitting the cap resets the counter to allow future retries
            assert driver.reconnect_attempts == 0

    def test_set_reconnect_enabled(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.set_reconnect_enabled(False)
        assert driver.reconnect_enabled is False
        driver.set_reconnect_enabled(True)
        assert driver.reconnect_enabled is True


class TestStatistics:
    def test_statistics_include_reconnect_fields(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        stats = driver.get_statistics()
        assert 'reconnect_attempts' in stats
        assert 'reconnect_enabled' in stats
        assert 'seconds_since_last_read' in stats


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
