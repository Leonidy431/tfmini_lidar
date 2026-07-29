"""
Tests for the MAVLink 3D Attitude module (Physics Audit D1).

Covers the pure-math quaternion/Euler conversions and the
MAVLinkAttitudeReader's fresh/stale sample lifecycle -- all exercised via
ingest_raw() so the suite runs without pymavlink or a live connection.
"""

import sys
import os
import math
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.mavlink_imu import (
    MAVLinkAttitudeReader,
    quaternion_to_rotation_matrix,
    euler_to_quaternion,
    quaternion_to_euler,
)


class TestQuaternionMath:
    def test_identity_rotation(self):
        R = quaternion_to_rotation_matrix(0.0, 0.0, 0.0, 1.0)
        assert np.allclose(R, np.eye(3), atol=1e-9)

    def test_rotation_matrix_orthonormal(self):
        x, y, z, w = euler_to_quaternion(math.radians(15), math.radians(-25), math.radians(200))
        R = quaternion_to_rotation_matrix(x, y, z, w)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
        assert abs(np.linalg.det(R) - 1.0) < 1e-9

    def test_zero_quaternion_falls_back_to_identity(self):
        """Degenerate all-zero quaternion must not raise or divide by zero."""
        R = quaternion_to_rotation_matrix(0.0, 0.0, 0.0, 0.0)
        assert np.allclose(R, np.eye(3))

    @pytest.mark.parametrize("roll_deg,pitch_deg,yaw_deg", [
        (0, 0, 0), (30, 0, 0), (0, 20, 0), (0, 0, 45),
        (15, -25, 200), (-89, 10, -10),
    ])
    def test_euler_quaternion_round_trip(self, roll_deg, pitch_deg, yaw_deg):
        roll, pitch, yaw = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
        q = euler_to_quaternion(roll, pitch, yaw)
        roll2, pitch2, yaw2 = quaternion_to_euler(*q)
        assert abs(roll - roll2) < 1e-6
        assert abs(pitch - pitch2) < 1e-6
        assert abs(math.sin(yaw) - math.sin(yaw2)) < 1e-6
        assert abs(math.cos(yaw) - math.cos(yaw2)) < 1e-6

    def test_gimbal_lock_pitch_90_does_not_raise(self):
        """Physics Audit D1 adversarial test: pitch = +90 deg must not
        raise a math domain error from floating-point asin() overshoot."""
        q = euler_to_quaternion(0.0, math.radians(90.0), math.radians(30.0))
        roll, pitch, yaw = quaternion_to_euler(*q)
        assert abs(pitch - math.radians(90.0)) < 1e-3

    def test_gimbal_lock_pitch_negative_90(self):
        q = euler_to_quaternion(0.0, math.radians(-90.0), 0.0)
        roll, pitch, yaw = quaternion_to_euler(*q)
        assert abs(pitch - math.radians(-90.0)) < 1e-3


class TestMAVLinkAttitudeReaderLifecycle:
    def test_no_attitude_before_first_sample(self):
        reader = MAVLinkAttitudeReader(timeout_s=1.0)
        assert reader.get_attitude() is None
        assert reader.get_se3_rotation() is None

    def test_ingest_raw_produces_fresh_sample(self):
        reader = MAVLinkAttitudeReader(timeout_s=1.0)
        reader.ingest_raw(roll=0.1, pitch=-0.05, yaw=1.2)
        sample = reader.get_attitude()
        assert sample is not None
        assert abs(sample.roll - 0.1) < 1e-9
        assert abs(sample.pitch - (-0.05)) < 1e-9
        assert abs(sample.yaw - 1.2) < 1e-9

    def test_se3_rotation_is_orthonormal(self):
        reader = MAVLinkAttitudeReader(timeout_s=1.0)
        reader.ingest_raw(roll=0.2, pitch=0.1, yaw=-0.3)
        R = reader.get_se3_rotation()
        assert R is not None
        assert R.shape == (3, 3)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)

    def test_stale_sample_returns_none(self):
        """Physics Audit D1 P6/P10: a sample older than timeout_s must be
        treated as missing so callers fall back to 1D heading, not serve a
        frozen orientation indefinitely."""
        reader = MAVLinkAttitudeReader(timeout_s=0.05)
        t0 = time.monotonic()
        reader.ingest_raw(roll=0.0, pitch=0.0, yaw=0.0, mono_timestamp=t0 - 1.0)
        assert reader.get_attitude() is None
        assert reader.get_se3_rotation() is None

    def test_fresh_sample_within_timeout(self):
        reader = MAVLinkAttitudeReader(timeout_s=5.0)
        reader.ingest_raw(roll=0.0, pitch=0.0, yaw=0.0)
        assert reader.get_attitude() is not None

    def test_messages_received_counter_increments(self):
        reader = MAVLinkAttitudeReader()
        reader.ingest_raw(0.0, 0.0, 0.0)
        reader.ingest_raw(0.1, 0.0, 0.0)
        assert reader.messages_received == 2

    def test_statistics_shape(self):
        reader = MAVLinkAttitudeReader()
        reader.ingest_raw(roll=0.1, pitch=0.2, yaw=0.3)
        stats = reader.get_statistics()
        assert "available" in stats
        assert "has_fresh_attitude" in stats
        assert stats["has_fresh_attitude"] is True
        assert stats["roll_deg"] is not None

    def test_statistics_without_sample(self):
        reader = MAVLinkAttitudeReader()
        stats = reader.get_statistics()
        assert stats["has_fresh_attitude"] is False
        assert stats["roll_deg"] is None

    def test_connect_without_pymavlink_is_graceful(self):
        """If pymavlink isn't installed, connect()/start() must return False
        rather than raising -- the caller falls back to 1D heading."""
        reader = MAVLinkAttitudeReader()
        if not reader.is_available:
            assert reader.connect() is False
            assert reader.last_error is not None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
