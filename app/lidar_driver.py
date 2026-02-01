"""
TFmini-S LiDAR Driver Module

Benewake TFmini-S is a single-point ToF LiDAR sensor.
Communication: UART at 115200 baud
Data format: 9-byte frame with 0x59 0x59 header
"""

import serial
import threading
import time
import struct
from typing import Optional, Callable, List
from dataclasses import dataclass
from datetime import datetime
from collections import deque
import logging

logger = logging.getLogger(__name__)


@dataclass
class LiDARReading:
    """Single LiDAR measurement"""
    distance: float  # meters
    signal_strength: int  # 0-65535
    timestamp: datetime
    temperature: float  # Celsius
    valid: bool = True

    def to_dict(self) -> dict:
        return {
            'distance': round(self.distance, 3),
            'signal_strength': self.signal_strength,
            'timestamp': self.timestamp.isoformat(),
            'temperature': round(self.temperature, 1),
            'valid': self.valid
        }


class TFminiSDriver:
    """
    Driver for Benewake TFmini-S LiDAR sensor

    Protocol:
    - Frame: [0x59, 0x59, Dist_L, Dist_H, Strength_L, Strength_H, Temp_L, Temp_H, Checksum]
    - Distance: Dist_H << 8 | Dist_L (in cm)
    - Strength: Strength_H << 8 | Strength_L
    - Temperature: (Temp_H << 8 | Temp_L) / 8 - 256
    - Checksum: Low byte of sum of first 8 bytes
    """

    FRAME_HEADER = bytes([0x59, 0x59])
    FRAME_LENGTH = 9
    DEFAULT_BAUDRATE = 115200

    # Command bytes
    CMD_HEADER = bytes([0x5A, 0x04])
    CMD_GET_VERSION = bytes([0x01])
    CMD_SOFT_RESET = bytes([0x02])
    CMD_SET_FRAMERATE = bytes([0x03])
    CMD_TRIGGER_ONCE = bytes([0x04])
    CMD_OUTPUT_FORMAT = bytes([0x05])
    CMD_SET_BAUDRATE = bytes([0x06])
    CMD_OUTPUT_ENABLE = bytes([0x07])
    CMD_RESTORE_DEFAULT = bytes([0x10])
    CMD_SAVE_SETTINGS = bytes([0x11])

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.serial_conn: Optional[serial.Serial] = None
        self.is_running = False
        self.thread: Optional[threading.Thread] = None

        self.callbacks: List[Callable[[LiDARReading], None]] = []
        self.buffer = bytearray()

        # Statistics
        self.readings_count = 0
        self.errors_count = 0
        self.last_reading: Optional[LiDARReading] = None
        self.readings_history = deque(maxlen=100)

    @property
    def is_connected(self) -> bool:
        return self.serial_conn is not None and self.serial_conn.is_open

    def connect(self) -> bool:
        """Connect to the LiDAR sensor"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )

            # Clear buffers
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            time.sleep(0.5)

            logger.info(f"Connected to TFmini-S on {self.port}")
            return True

        except serial.SerialException as e:
            logger.error(f"Failed to connect to TFmini-S: {e}")
            return False
        except Exception as e:
            logger.error(f"Unexpected error connecting: {e}")
            return False

    def disconnect(self):
        """Disconnect from the sensor"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            logger.info("Disconnected from TFmini-S")

    def start(self):
        """Start continuous reading thread"""
        if not self.is_connected:
            if not self.connect():
                raise RuntimeError("Cannot start: failed to connect to TFmini-S")

        self.is_running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        logger.info("TFmini-S reading thread started")

    def stop(self):
        """Stop reading thread"""
        self.is_running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        self.disconnect()
        logger.info("TFmini-S reading thread stopped")

    def add_callback(self, callback: Callable[[LiDARReading], None]):
        """Add callback for new readings"""
        self.callbacks.append(callback)

    def remove_callback(self, callback: Callable[[LiDARReading], None]):
        """Remove callback"""
        if callback in self.callbacks:
            self.callbacks.remove(callback)

    def _read_loop(self):
        """Main reading loop"""
        while self.is_running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    self.buffer.extend(data)
                    self._process_buffer()
                else:
                    time.sleep(0.001)  # 1ms sleep to prevent CPU spinning

            except serial.SerialException as e:
                logger.error(f"Serial error: {e}")
                self.errors_count += 1
                time.sleep(0.1)
            except Exception as e:
                logger.error(f"Read loop error: {e}")
                self.errors_count += 1
                time.sleep(0.1)

    def _process_buffer(self):
        """Process accumulated buffer data"""
        while len(self.buffer) >= self.FRAME_LENGTH:
            # Find frame header
            header_pos = self.buffer.find(self.FRAME_HEADER)

            if header_pos == -1:
                # No header found, clear buffer
                self.buffer.clear()
                return

            if header_pos > 0:
                # Discard bytes before header
                del self.buffer[:header_pos]

            if len(self.buffer) < self.FRAME_LENGTH:
                return

            # Extract frame
            frame = bytes(self.buffer[:self.FRAME_LENGTH])

            # Verify checksum
            checksum = sum(frame[:8]) & 0xFF
            if checksum != frame[8]:
                # Invalid checksum, skip this byte
                del self.buffer[0]
                self.errors_count += 1
                continue

            # Parse valid frame
            reading = self._parse_frame(frame)
            del self.buffer[:self.FRAME_LENGTH]

            if reading:
                self.last_reading = reading
                self.readings_count += 1
                self.readings_history.append(reading)

                # Call all callbacks
                for callback in self.callbacks:
                    try:
                        callback(reading)
                    except Exception as e:
                        logger.error(f"Callback error: {e}")

    def _parse_frame(self, frame: bytes) -> Optional[LiDARReading]:
        """Parse TFmini-S data frame"""
        try:
            # Distance in cm (convert to meters)
            distance_cm = frame[2] | (frame[3] << 8)
            distance_m = distance_cm / 100.0

            # Signal strength
            strength = frame[4] | (frame[5] << 8)

            # Temperature
            temp_raw = frame[6] | (frame[7] << 8)
            temperature = temp_raw / 8.0 - 256.0

            # Validate reading
            valid = (distance_cm > 0 and
                    distance_cm < 1200 and  # Max 12m
                    strength > 0)

            return LiDARReading(
                distance=distance_m,
                signal_strength=strength,
                timestamp=datetime.now(),
                temperature=temperature,
                valid=valid
            )

        except Exception as e:
            logger.error(f"Frame parse error: {e}")
            return None

    def get_single_reading(self, timeout: float = 1.0) -> Optional[LiDARReading]:
        """Get a single reading synchronously"""
        if not self.is_connected:
            if not self.connect():
                return None

        start_time = time.time()
        temp_buffer = bytearray()

        while time.time() - start_time < timeout:
            if self.serial_conn.in_waiting > 0:
                temp_buffer.extend(self.serial_conn.read(self.serial_conn.in_waiting))

                # Look for frame
                while len(temp_buffer) >= self.FRAME_LENGTH:
                    header_pos = temp_buffer.find(self.FRAME_HEADER)

                    if header_pos == -1:
                        temp_buffer.clear()
                        break

                    if header_pos > 0:
                        del temp_buffer[:header_pos]

                    if len(temp_buffer) >= self.FRAME_LENGTH:
                        frame = bytes(temp_buffer[:self.FRAME_LENGTH])
                        checksum = sum(frame[:8]) & 0xFF

                        if checksum == frame[8]:
                            reading = self._parse_frame(frame)
                            if reading and reading.valid:
                                return reading

                        del temp_buffer[:1]
            else:
                time.sleep(0.001)

        return None

    def set_framerate(self, fps: int) -> bool:
        """
        Set output framerate (1-1000 Hz)
        Command: 0x5A 0x06 0x03 [fps_L] [fps_H] [checksum]
        """
        if not 1 <= fps <= 1000:
            logger.error("Framerate must be 1-1000 Hz")
            return False

        try:
            cmd = bytearray([0x5A, 0x06, 0x03, fps & 0xFF, (fps >> 8) & 0xFF])
            cmd.append(sum(cmd) & 0xFF)

            self.serial_conn.write(cmd)
            time.sleep(0.1)

            logger.info(f"Set framerate to {fps} Hz")
            return True

        except Exception as e:
            logger.error(f"Failed to set framerate: {e}")
            return False

    def set_output_format(self, format_type: int = 1) -> bool:
        """
        Set output format
        0: Standard 9-byte
        1: PIX format (Standard for TFmini-S)
        """
        try:
            cmd = bytearray([0x5A, 0x05, 0x05, format_type])
            cmd.append(sum(cmd) & 0xFF)

            self.serial_conn.write(cmd)
            time.sleep(0.1)

            logger.info(f"Set output format to {format_type}")
            return True

        except Exception as e:
            logger.error(f"Failed to set output format: {e}")
            return False

    def save_settings(self) -> bool:
        """Save current settings to sensor EEPROM"""
        try:
            cmd = bytearray([0x5A, 0x04, 0x11])
            cmd.append(sum(cmd) & 0xFF)

            self.serial_conn.write(cmd)
            time.sleep(0.1)

            logger.info("Settings saved to sensor")
            return True

        except Exception as e:
            logger.error(f"Failed to save settings: {e}")
            return False

    def get_statistics(self) -> dict:
        """Get driver statistics"""
        avg_distance = 0
        avg_strength = 0

        if self.readings_history:
            valid_readings = [r for r in self.readings_history if r.valid]
            if valid_readings:
                avg_distance = sum(r.distance for r in valid_readings) / len(valid_readings)
                avg_strength = sum(r.signal_strength for r in valid_readings) / len(valid_readings)

        return {
            'connected': self.is_connected,
            'running': self.is_running,
            'total_readings': self.readings_count,
            'total_errors': self.errors_count,
            'error_rate': self.errors_count / max(1, self.readings_count),
            'average_distance': round(avg_distance, 3),
            'average_strength': round(avg_strength, 1),
            'last_reading': self.last_reading.to_dict() if self.last_reading else None
        }
