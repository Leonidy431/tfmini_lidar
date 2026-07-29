"""
MAVLink 3D Attitude Module (Physics Audit D1)

Fuses MAVLink ATTITUDE_QUATERNION telemetry into a SE(3) rotation usable by
the SLAM/localization pipeline, replacing the 1D compass-heading-only
projection with full roll/pitch/yaw when a flight controller is present.

Decision record: docs/ALGORITHM_DECISION_LOG.md (Decision 4: MAVLink 3D
Attitude). Scientific basis: Diebel (2006) "Representing Attitude", Beard &
McLain (2012) "Small Unmanned Aircraft" Ch. 2, PX4 ATTITUDE_QUATERNION spec.

Feature-flagged via Config.mavlink_attitude.enabled (env
ENABLE_MAVLINK_3D_ATTITUDE). Graceful degradation (P10): if no message has
arrived within timeout_s, get_se3_rotation()/get_attitude() return None and
callers fall back to the existing 1D compass-heading projection in
app/main.py -- this module never raises on a missing/stale connection.
"""

import time
import math
import logging
import threading
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

try:
    from pymavlink import mavutil
    PYMAVLINK_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised on hosts without pymavlink
    mavutil = None
    PYMAVLINK_AVAILABLE = False


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Unit quaternion (x, y, z, w) -> 3x3 rotation matrix.

    See Diebel (2006) eq. 125. Non-unit input is re-normalized defensively
    (MAVLink links can deliver a slightly denormalized quaternion).
    """
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """roll/pitch/yaw (radians, aerospace ZYX convention) -> (x, y, z, w)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """(x, y, z, w) -> (roll, pitch, yaw) radians.

    Gimbal-lock-safe: the pitch asin() argument is clamped to [-1, 1]
    (Physics Audit D1 P6 adversarial test: pitch -> +/-90 deg must not
    raise a domain error from floating-point overshoot).
    """
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


@dataclass
class AttitudeSample:
    roll: float           # radians
    pitch: float          # radians
    yaw: float            # radians
    quaternion: Tuple[float, float, float, float]  # (x, y, z, w)
    mono_timestamp: float  # time.monotonic(), immune to NTP steps (H9 convention)


class MAVLinkAttitudeReader:
    """Background reader for MAVLink ATTITUDE / ATTITUDE_QUATERNION messages.

    Runs its own thread once started; get_attitude() / get_se3_rotation()
    are the only methods most callers need, and both return None if no
    fresh sample is available -- signaling "fall back to 1D heading".
    """

    def __init__(self, connection_string: str = "udp:127.0.0.1:14550",
                 baudrate: int = 115200, timeout_s: float = 1.0):
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.timeout_s = timeout_s

        self._mav = None
        self._lock = threading.Lock()
        self._latest: Optional[AttitudeSample] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self.messages_received = 0
        self.parse_errors = 0
        self.last_error: Optional[str] = None

    @property
    def is_available(self) -> bool:
        """False if pymavlink isn't installed -- caller should not attempt connect()."""
        return PYMAVLINK_AVAILABLE

    def connect(self) -> bool:
        if not PYMAVLINK_AVAILABLE:
            self.last_error = "pymavlink not installed"
            logger.warning("MAVLink attitude disabled: %s", self.last_error)
            return False
        try:
            self._mav = mavutil.mavlink_connection(self.connection_string, baud=self.baudrate)
            return True
        except Exception as exc:
            self.last_error = str(exc)
            logger.error("MAVLink connect failed: %s", exc)
            return False

    def start(self) -> bool:
        """Connect and start the background read thread. Returns False (no
        raise) if pymavlink is unavailable or the connection fails -- caller
        proceeds with 1D heading only."""
        if not self.connect():
            return False
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _read_loop(self):
        while self._running:
            try:
                msg = self._mav.recv_match(
                    type=["ATTITUDE_QUATERNION", "ATTITUDE"], blocking=True, timeout=0.5)
                if msg is None:
                    continue
                self._ingest(msg)
            except Exception as exc:
                self.parse_errors += 1
                self.last_error = str(exc)
                logger.debug("MAVLink read error: %s", exc)

    def _ingest(self, msg):
        mtype = msg.get_type()
        if mtype == "ATTITUDE_QUATERNION":
            # MAVLink wire order is q1=w, q2=x, q3=y, q4=z.
            q = (msg.q2, msg.q3, msg.q4, msg.q1)
            roll, pitch, yaw = quaternion_to_euler(*q)
        elif mtype == "ATTITUDE":
            roll, pitch, yaw = msg.roll, msg.pitch, msg.yaw
            q = euler_to_quaternion(roll, pitch, yaw)
        else:
            return
        self._store(roll, pitch, yaw, q, time.monotonic())

    def ingest_raw(self, roll: float, pitch: float, yaw: float,
                    mono_timestamp: Optional[float] = None):
        """Test/emulation hook: inject an attitude sample without a real
        MAVLink connection (see tests/emulation_server.py MAVLinkEmulator,
        and unit tests in tests/test_mavlink_imu.py)."""
        q = euler_to_quaternion(roll, pitch, yaw)
        ts = mono_timestamp if mono_timestamp is not None else time.monotonic()
        self._store(roll, pitch, yaw, q, ts)

    def _store(self, roll, pitch, yaw, quaternion, mono_timestamp):
        sample = AttitudeSample(roll=roll, pitch=pitch, yaw=yaw,
                                 quaternion=quaternion, mono_timestamp=mono_timestamp)
        with self._lock:
            self._latest = sample
            self.messages_received += 1

    def get_attitude(self) -> Optional[AttitudeSample]:
        """Latest attitude sample, or None if missing/stale (older than
        timeout_s -- Physics Audit D1 P6: MAVLink dropout must fail safe,
        not silently serve a frozen orientation)."""
        with self._lock:
            sample = self._latest
        if sample is None:
            return None
        if (time.monotonic() - sample.mono_timestamp) > self.timeout_s:
            return None
        return sample

    def get_se3_rotation(self) -> Optional[np.ndarray]:
        """3x3 rotation matrix from the latest attitude, or None if stale."""
        sample = self.get_attitude()
        if sample is None:
            return None
        return quaternion_to_rotation_matrix(*sample.quaternion)

    def get_statistics(self) -> dict:
        sample = self.get_attitude()
        return {
            "available": PYMAVLINK_AVAILABLE,
            "connected": self._mav is not None,
            "messages_received": self.messages_received,
            "parse_errors": self.parse_errors,
            "last_error": self.last_error,
            "has_fresh_attitude": sample is not None,
            "roll_deg": math.degrees(sample.roll) if sample else None,
            "pitch_deg": math.degrees(sample.pitch) if sample else None,
            "yaw_deg": math.degrees(sample.yaw) if sample else None,
        }
