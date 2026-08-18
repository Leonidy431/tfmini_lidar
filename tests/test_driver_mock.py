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


class TestPhysicsCorrections:
    """Physics Audit C1/H1/H9: medium refraction, sentinel decoding, and
    driver-level datasheet defaults."""

    def test_medium_refractive_index_default_is_noop(self):
        """Library-level default must stay 1.0 (air/bench) so raw driver
        instantiation and existing unit tests are unaffected; the app
        applies the water correction via LiDARConfig, not this default."""
        driver = TFminiSDriver('/dev/ttyUSB0')
        assert driver.medium_refractive_index == 1.0

        reading = driver._parse_frame(build_frame(150))
        assert reading.distance == 1.5

    def test_medium_refractive_index_applied(self):
        """distance_m = (distance_cm/100) / n -- underwater a reported
        1.5m (n=1.0 raw) should resolve to the true ~1.125m range at
        n=1.333."""
        driver = TFminiSDriver('/dev/ttyUSB0', medium_refractive_index=1.333)
        reading = driver._parse_frame(build_frame(150))
        assert abs(reading.distance - (1.5 / 1.333)) < 1e-6

    def test_weak_signal_sentinel_rejected(self):
        """distance_cm=65535 is the TFmini-S 'pulse timing invalid'
        sentinel, not a real 655m measurement."""
        driver = TFminiSDriver('/dev/ttyUSB0')
        reading = driver._parse_frame(build_frame(65535, strength=50))
        assert reading is not None
        assert reading.valid is False

    def test_saturation_sentinel_rejected_by_distance_code(self):
        """distance_cm=65532 is the 'receiver saturated' sentinel."""
        driver = TFminiSDriver('/dev/ttyUSB0')
        reading = driver._parse_frame(build_frame(65532, strength=200))
        assert reading is not None
        assert reading.valid is False

    def test_saturated_strength_rejected_even_with_plausible_distance(self):
        """strength==65535 (saturated receiver) must invalidate the frame
        even when the accompanying distance looks like a normal reading --
        a saturated pulse has unreliable timing walk (Physics Audit H1)."""
        driver = TFminiSDriver('/dev/ttyUSB0')
        reading = driver._parse_frame(build_frame(150, strength=65535))
        assert reading is not None
        assert reading.valid is False

    def test_normal_frame_still_valid(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        reading = driver._parse_frame(build_frame(150, strength=200))
        assert reading.valid is True

    def test_datasheet_floor_defaults(self):
        """Constructor defaults raised to the TFmini-S datasheet floor so a
        driver built with no explicit config still rejects noise-floor and
        blind-zone readings (Physics Audit H1)."""
        driver = TFminiSDriver('/dev/ttyUSB0')
        assert driver.min_signal == 100
        assert driver.min_range_m == 0.1

        weak = driver._parse_frame(build_frame(150, strength=50))
        assert weak.valid is False

        blind_zone = driver._parse_frame(build_frame(5, strength=200))  # 0.05m
        assert blind_zone.valid is False

    def test_mono_timestamp_populated(self):
        """A monotonic capture time must be attached to every reading so
        the rate-of-change gate is immune to wall-clock/NTP steps
        (Physics Audit H9)."""
        driver = TFminiSDriver('/dev/ttyUSB0')
        reading = driver._parse_frame(build_frame(150, strength=200))
        assert reading.mono_timestamp is not None
        assert reading.mono_timestamp > 0

    def test_mono_timestamp_monotonic_across_frames(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        r1 = driver._parse_frame(build_frame(150, strength=200))
        r2 = driver._parse_frame(build_frame(150, strength=200))
        assert r2.mono_timestamp >= r1.mono_timestamp


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
        assert 'last_error' in stats

    def test_concurrent_frame_processing_and_statistics_read(self):
        """Regression test for Blind Spot Audit R3 R3-CONC-2:
        readings_history is appended to from the serial-read thread
        (_process_buffer) and filtered from the Flask request thread
        (get_statistics, polled by /api/status and /api/health) with no
        lock -- a poll landing mid-append could raise 'deque mutated
        during iteration' and 500 an unrelated request."""
        import concurrent.futures

        driver = TFminiSDriver('/dev/ttyUSB0')
        errors = []

        def writer():
            for i in range(300):
                driver.buffer.extend(build_frame(100 + (i % 50)))
                driver._process_buffer()

        def reader():
            for _ in range(300):
                try:
                    driver.get_statistics()
                except RuntimeError as exc:
                    errors.append(exc)

        # Only one writer thread: production has exactly one serial-read
        # thread calling _process_buffer; the real race under test is
        # between that thread and multiple concurrent Flask readers.
        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
            futures = [pool.submit(writer), pool.submit(reader), pool.submit(reader)]
            for f in futures:
                f.result()

        assert errors == []


class TestErrorPropagation:
    def test_error_callback_invoked(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        seen = []
        driver.add_error_callback(lambda ctx, exc: seen.append((ctx, str(exc))))

        driver._notify_error('serial', RuntimeError("cable unplugged"))

        assert len(seen) == 1
        assert seen[0][0] == 'serial'
        assert 'cable unplugged' in seen[0][1]
        assert driver.last_error is not None

    def test_error_callback_isolated(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        good = []
        driver.add_error_callback(lambda ctx, exc: (_ for _ in ()).throw(ValueError("bad")))
        driver.add_error_callback(lambda ctx, exc: good.append(ctx))

        driver._notify_error('read_loop', RuntimeError("x"))

        # Second callback runs despite the first raising
        assert good == ['read_loop']


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
