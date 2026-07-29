"""
Tests for the 9-DOF position+attitude EKF fusion module (Physics Audit D8).
"""

import sys
import os
import math

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.ekf_3d_attitude import EKF3DAttitude, wrap_angle, IDX_X, IDX_ROLL


class TestAngleWrap:
    def test_wrap_identity_within_range(self):
        assert wrap_angle(0.5) == pytest.approx(0.5)

    def test_wrap_positive_overflow(self):
        assert wrap_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)

    def test_wrap_negative_overflow(self):
        assert wrap_angle(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)

    def test_wrap_result_always_in_range(self):
        for angle in np.linspace(-10 * math.pi, 10 * math.pi, 50):
            wrapped = wrap_angle(angle)
            assert -math.pi <= wrapped < math.pi


class TestPredict:
    def test_predict_moves_position_by_velocity(self):
        ekf = EKF3DAttitude()
        ekf.state[6:9] = [1.0, 0.5, 0.0]  # vx, vy, vz
        ekf.predict(dt=2.0)
        assert ekf.state[IDX_X] == pytest.approx(2.0)
        assert ekf.state[1] == pytest.approx(1.0)

    def test_predict_zero_dt_is_noop(self):
        ekf = EKF3DAttitude()
        ekf.state[6] = 5.0
        state_before = ekf.state.copy()
        ekf.predict(dt=0.0)
        assert np.array_equal(ekf.state, state_before)
        assert ekf.predict_count == 0

    def test_predict_negative_dt_is_noop(self):
        ekf = EKF3DAttitude()
        ekf.predict(dt=-1.0)
        assert ekf.predict_count == 0

    def test_predict_increases_covariance(self):
        ekf = EKF3DAttitude()
        trace_before = np.trace(ekf.covariance)
        ekf.predict(dt=1.0)
        trace_after = np.trace(ekf.covariance)
        assert trace_after > trace_before

    def test_repeated_predict_keeps_covariance_finite(self):
        ekf = EKF3DAttitude()
        for _ in range(1000):
            ekf.predict(dt=0.1)
        assert np.all(np.isfinite(ekf.covariance))
        assert np.all(np.isfinite(ekf.state))


class TestPositionUpdate:
    def test_update_pulls_state_toward_measurement(self):
        ekf = EKF3DAttitude()
        ekf.predict(dt=1.0)
        ekf.update_position([1.0, 2.0, 0.5])
        assert ekf.state[IDX_X] != 0.0
        assert ekf.position_update_count == 1

    def test_update_reduces_position_uncertainty(self):
        ekf = EKF3DAttitude()
        ekf.predict(dt=1.0)
        std_before = ekf.get_state()["position_std"][0]
        ekf.update_position([1.0, 1.0, 1.0])
        std_after = ekf.get_state()["position_std"][0]
        assert std_after < std_before

    def test_repeated_updates_converge_to_measurement(self):
        ekf = EKF3DAttitude(measurement_noise_position=0.05)
        target = np.array([3.0, -1.0, 0.2])
        for _ in range(50):
            ekf.predict(dt=0.1)
            ekf.update_position(target)
        assert np.allclose(ekf.state[0:3], target, atol=0.05)


class TestAttitudeUpdate:
    def test_update_pulls_attitude_toward_measurement(self):
        ekf = EKF3DAttitude()
        ekf.predict(dt=1.0)
        ekf.update_attitude(roll=0.1, pitch=-0.05, yaw=1.0)
        assert ekf.attitude_update_count == 1
        assert ekf.state[IDX_ROLL] != 0.0

    def test_wraps_correctly_across_pi_seam(self):
        """State near +pi, measurement near -pi (physically 0.1 rad apart
        going the short way around the circle) must not be treated as a
        ~2*pi discontinuity -- Physics Audit D8 adversarial test."""
        ekf = EKF3DAttitude(measurement_noise_attitude=0.01)
        ekf.state[5] = math.pi - 0.05  # yaw near +pi
        for _ in range(20):
            ekf.predict(dt=0.05)
            ekf.update_attitude(roll=0.0, pitch=0.0, yaw=-math.pi + 0.05)
        # Should converge near the wrapped target, not swing to 0
        yaw = wrap_angle(ekf.state[5])
        target = wrap_angle(-math.pi + 0.05)
        diff = abs(wrap_angle(yaw - target))
        assert diff < 0.2

    def test_repeated_updates_converge(self):
        ekf = EKF3DAttitude(measurement_noise_attitude=0.01)
        target = (0.2, -0.15, 1.3)
        for _ in range(50):
            ekf.predict(dt=0.05)
            ekf.update_attitude(*target)
        attitude = ekf.get_state()["attitude_rad"]
        assert np.allclose(attitude, target, atol=0.05)


class TestNumericalStability:
    def test_singular_innovation_covariance_does_not_crash(self):
        ekf = EKF3DAttitude()
        H = np.zeros((3, 9))
        innovation = np.zeros(3)
        R = np.zeros((3, 3))  # S = H P H^T + R = zero matrix -> singular
        applied = ekf._kalman_update(H, innovation, R)
        assert applied is False
        assert ekf.skipped_singular_updates == 1
        assert np.all(np.isfinite(ekf.state))
        assert np.all(np.isfinite(ekf.covariance))

    def test_covariance_stays_symmetric(self):
        ekf = EKF3DAttitude()
        for _ in range(100):
            ekf.predict(dt=0.1)
            ekf.update_position(np.random.default_rng(1).normal(0, 1, 3))
            ekf.update_attitude(0.1, 0.1, 0.1)
        assert np.allclose(ekf.covariance, ekf.covariance.T, atol=1e-9)

    def test_covariance_stays_positive_semidefinite(self):
        ekf = EKF3DAttitude()
        rng = np.random.default_rng(3)
        for _ in range(200):
            ekf.predict(dt=0.05)
            ekf.update_position(rng.normal(0, 0.5, 3))
            ekf.update_attitude(*rng.normal(0, 0.1, 3))
        eigenvalues = np.linalg.eigvalsh(ekf.covariance)
        assert np.all(eigenvalues > -1e-6)


class TestSE3Pose:
    def test_se3_pose_shape_and_orthonormal(self):
        ekf = EKF3DAttitude()
        ekf.state[3:6] = [0.1, -0.2, 0.5]
        T = ekf.get_se3_pose()
        assert T.shape == (4, 4)
        R = T[:3, :3]
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)

    def test_se3_pose_translation_matches_state(self):
        ekf = EKF3DAttitude()
        ekf.state[0:3] = [1.0, 2.0, 3.0]
        T = ekf.get_se3_pose()
        assert np.allclose(T[:3, 3], [1.0, 2.0, 3.0])


class TestStatistics:
    def test_statistics_fields(self):
        ekf = EKF3DAttitude()
        ekf.predict(dt=0.1)
        ekf.update_position([1.0, 0.0, 0.0])
        ekf.update_attitude(0.0, 0.0, 0.0)
        stats = ekf.get_statistics()
        assert stats["predict_count"] == 1
        assert stats["position_update_count"] == 1
        assert stats["attitude_update_count"] == 1
        assert stats["covariance_trace"] > 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
