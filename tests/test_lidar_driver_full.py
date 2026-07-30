"""Coverage-fill tests for app/lidar_driver.py (connect, commands, single read)."""

import sys
import os
import time
from unittest.mock import MagicMock, patch

import pytest
import serial

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.lidar_driver import TFminiSDriver


def build_frame(distance_cm, strength=200, temp_raw=400):
    frame = bytes([0x59, 0x59, distance_cm & 0xFF, (distance_cm >> 8) & 0xFF,
                   strength & 0xFF, (strength >> 8) & 0xFF,
                   temp_raw & 0xFF, (temp_raw >> 8) & 0xFF])
    return frame + bytes([sum(frame[:8]) & 0xFF])


class FakeSerial:
    def __init__(self, stream=b""):
        self._stream = bytearray(stream)
        self.is_open = True
        self.written = bytearray()

    @property
    def in_waiting(self):
        return len(self._stream)

    def read(self, n):
        chunk = bytes(self._stream[:n])
        del self._stream[:n]
        return chunk

    def write(self, data):
        self.written.extend(data)
        return len(data)

    def reset_input_buffer(self): pass
    def reset_output_buffer(self): pass
    def close(self): self.is_open = False


class TestConnect:
    def test_connect_success(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        with patch('serial.Serial', return_value=FakeSerial()):
            assert driver.connect() is True
            assert driver.is_connected is True

    def test_connect_serial_exception(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        with patch('serial.Serial', side_effect=serial.SerialException("no port")):
            assert driver.connect() is False

    def test_connect_generic_exception(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        with patch('serial.Serial', side_effect=RuntimeError("boom")):
            assert driver.connect() is False

    def test_disconnect(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.serial_conn = FakeSerial()
        driver.disconnect()
        assert driver.serial_conn.is_open is False


class TestStartStop:
    def test_start_connect_failure_raises(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.reconnect_enabled = False
        with patch.object(driver, 'connect', return_value=False):
            with pytest.raises(RuntimeError):
                driver.start()

    def test_start_then_stop(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.serial_conn = FakeSerial(build_frame(150) * 3)
        driver.start()
        time.sleep(0.05)
        driver.stop()
        assert driver.is_running is False


class TestSingleReading:
    def test_get_single_reading_returns_valid(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.serial_conn = FakeSerial(build_frame(200, strength=200))
        reading = driver.get_single_reading(timeout=0.5)
        assert reading is not None and reading.distance == 2.0

    def test_get_single_reading_connects_if_needed(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        with patch.object(driver, 'connect', return_value=False):
            assert driver.get_single_reading(timeout=0.1) is None

    def test_get_single_reading_timeout(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.serial_conn = FakeSerial(b'')  # no data
        assert driver.get_single_reading(timeout=0.05) is None


class TestCommands:
    def _connected_driver(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.serial_conn = FakeSerial()
        return driver

    def test_set_framerate_valid(self):
        driver = self._connected_driver()
        assert driver.set_framerate(100) is True
        assert len(driver.serial_conn.written) == 6

    def test_set_framerate_out_of_range(self):
        driver = self._connected_driver()
        assert driver.set_framerate(0) is False
        assert driver.set_framerate(2000) is False

    def test_set_framerate_write_error(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.serial_conn = MagicMock()
        driver.serial_conn.write.side_effect = RuntimeError("io")
        assert driver.set_framerate(100) is False

    def test_set_output_format(self):
        driver = self._connected_driver()
        assert driver.set_output_format(1) is True

    def test_set_output_format_error(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.serial_conn = MagicMock()
        driver.serial_conn.write.side_effect = RuntimeError("io")
        assert driver.set_output_format() is False

    def test_save_settings(self):
        driver = self._connected_driver()
        assert driver.save_settings() is True

    def test_save_settings_error(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.serial_conn = MagicMock()
        driver.serial_conn.write.side_effect = RuntimeError("io")
        assert driver.save_settings() is False


class TestReadLoopErrorPaths:
    def test_read_loop_handles_serial_exception(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.reconnect_enabled = False
        errors = []
        driver.add_error_callback(lambda ctx, exc: errors.append(ctx))

        fake = MagicMock()
        fake.is_open = True
        type(fake).in_waiting = property(
            lambda self: (_ for _ in ()).throw(serial.SerialException("lost")))
        driver.serial_conn = fake
        driver.is_running = True

        import threading
        t = threading.Thread(target=driver._read_loop, daemon=True)
        t.start()
        time.sleep(0.1)
        driver.is_running = False
        t.join(timeout=1.0)
        assert 'serial' in errors

    def test_handle_connection_error_clears_conn(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.serial_conn = FakeSerial(build_frame(150))
        driver.buffer.extend(b'\x01\x02')
        driver._handle_connection_error()
        assert driver.serial_conn is None
        assert len(driver.buffer) == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
