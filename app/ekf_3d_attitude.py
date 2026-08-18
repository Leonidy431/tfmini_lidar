"""
3D-Attitude EKF Module (Physics Audit D8)

Fuses LiDAR/SLAM-derived position with MAVLink attitude (D1) into a single
9-DOF state estimate: [x, y, z, roll, pitch, yaw, vx, vy, vz].

The process model is linear (constant-velocity on position; attitude and
velocity are random walks corrected by measurements), so the predict/update
math is a standard Kalman filter. The one nonlinearity is angle wrapping on
the three attitude channels -- both in state propagation and in the
measurement innovation (a naive z - Hx breaks near the +/-pi seam) -- which
is what makes this an "extended" filter despite constant F and H matrices.

Decision record: docs/ALGORITHM_DECISION_LOG.md (Decision 6: 3D-Attitude
EKF). Scientific basis: Bar-Shalom, Li & Kirubarajan (2001) "Estimation
with Applications to Tracking and Navigation" Ch. 5; Beard & McLain (2012)
"Small Unmanned Aircraft" Ch. 2.

Numerical stability: the covariance update uses the Joseph form
(P = (I-KH)P(I-KH)^T + KRK^T), which stays positive semi-definite under
floating-point rounding where the textbook P=(I-KH)P form can drift
indefinite after many iterations (Physics Audit D8 P6 adversarial finding).
"""

import logging
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)

STATE_DIM = 9

IDX_X, IDX_Y, IDX_Z = 0, 1, 2
IDX_ROLL, IDX_PITCH, IDX_YAW = 3, 4, 5
IDX_VX, IDX_VY, IDX_VZ = 6, 7, 8
ANGLE_INDICES = (IDX_ROLL, IDX_PITCH, IDX_YAW)


def wrap_angle(angle: float) -> float:
    """Wrap radians to [-pi, pi)."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class EKF3DAttitude:
    """9-DOF position+attitude+velocity EKF.

    Usage: call predict(dt) once per cycle, then update_position()
    and/or update_attitude() with whatever measurements arrived that cycle
    (either, both, or neither -- missing sensors just skip their update).
    """

    def __init__(self,
                 process_noise_position: float = 0.01,
                 process_noise_attitude: float = 0.05,
                 process_noise_velocity: float = 0.1,
                 measurement_noise_position: float = 0.15,
                 measurement_noise_attitude: float = 0.02,
                 initial_covariance: float = 1.0):
        self.state = np.zeros(STATE_DIM)
        self.covariance = np.eye(STATE_DIM) * initial_covariance

        q_diag = np.array(
            [process_noise_position] * 3 +
            [process_noise_attitude] * 3 +
            [process_noise_velocity] * 3
        )
        self.Q_base = np.diag(q_diag)

        self.R_position = np.eye(3) * (measurement_noise_position ** 2)
        self.R_attitude = np.eye(3) * (measurement_noise_attitude ** 2)

        self.predict_count = 0
        self.position_update_count = 0
        self.attitude_update_count = 0
        self.skipped_singular_updates = 0

    def predict(self, dt: float):
        """Propagate state and covariance forward by dt seconds. No-op for
        dt <= 0 (e.g. a duplicate or out-of-order timestamp)."""
        if dt <= 0:
            return

        F = np.eye(STATE_DIM)
        F[IDX_X, IDX_VX] = dt
        F[IDX_Y, IDX_VY] = dt
        F[IDX_Z, IDX_VZ] = dt

        self.state = F @ self.state
        for idx in ANGLE_INDICES:
            self.state[idx] = wrap_angle(self.state[idx])

        Q = self.Q_base * dt
        self.covariance = F @ self.covariance @ F.T + Q
        self._symmetrize()
        self.predict_count += 1

    def update_position(self, position_xyz) -> bool:
        """position_xyz: (x, y, z) from LiDAR/SLAM, in the same world frame
        as the EKF state. Returns False (no-op) on a non-finite input."""
        z = np.asarray(position_xyz, dtype=float)
        if not np.all(np.isfinite(z)):
            # A degenerate SLAM/localization result feeding this call would
            # otherwise poison self.state/self.covariance with NaN
            # permanently -- no subsequent predict/update recovers, since
            # NaN propagates through every further matrix op.
            # (Blind Spot Audit R3, R3-TEST-1)
            self.skipped_singular_updates += 1
            logger.warning("EKF position update skipped: non-finite input %s", position_xyz)
            return False

        H = np.zeros((3, STATE_DIM))
        H[0, IDX_X] = 1.0
        H[1, IDX_Y] = 1.0
        H[2, IDX_Z] = 1.0

        innovation = z - H @ self.state
        if self._kalman_update(H, innovation, self.R_position):
            self.position_update_count += 1
            return True
        return False

    def update_attitude(self, roll: float, pitch: float, yaw: float) -> bool:
        """roll/pitch/yaw in radians, e.g. from MAVLinkAttitudeReader.
        Returns False (no-op) on a non-finite input."""
        z = np.array([roll, pitch, yaw])
        if not np.all(np.isfinite(z)):
            # Same NaN-poisoning hazard as update_position.
            # (Blind Spot Audit R3, R3-TEST-1)
            self.skipped_singular_updates += 1
            logger.warning("EKF attitude update skipped: non-finite input (%s, %s, %s)",
                            roll, pitch, yaw)
            return False

        H = np.zeros((3, STATE_DIM))
        H[0, IDX_ROLL] = 1.0
        H[1, IDX_PITCH] = 1.0
        H[2, IDX_YAW] = 1.0

        predicted = H @ self.state
        innovation = np.array([wrap_angle(z[i] - predicted[i]) for i in range(3)])
        if self._kalman_update(H, innovation, self.R_attitude):
            for idx in ANGLE_INDICES:
                self.state[idx] = wrap_angle(self.state[idx])
            self.attitude_update_count += 1
            return True
        return False

    def _kalman_update(self, H: np.ndarray, innovation: np.ndarray, R: np.ndarray) -> bool:
        """Returns True if the update was applied, False if skipped due to
        a singular innovation covariance (Physics Audit D8 P6: must not
        propagate NaN/Inf into the state on a degenerate measurement)."""
        S = H @ self.covariance @ H.T + R
        try:
            K = self.covariance @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.skipped_singular_updates += 1
            logger.warning("EKF update skipped: singular innovation covariance")
            return False

        self.state = self.state + K @ innovation
        I_KH = np.eye(STATE_DIM) - K @ H
        # Joseph form: numerically stable, guarantees PSD covariance even
        # when K is not the exact optimal gain (floating-point rounding).
        self.covariance = I_KH @ self.covariance @ I_KH.T + K @ R @ K.T
        self._symmetrize()
        return True

    def _symmetrize(self):
        self.covariance = 0.5 * (self.covariance + self.covariance.T)

    def get_state(self) -> dict:
        std = np.sqrt(np.clip(np.diag(self.covariance), 0.0, None))
        return {
            "position": tuple(self.state[IDX_X:IDX_Z + 1]),
            "attitude_rad": tuple(self.state[IDX_ROLL:IDX_YAW + 1]),
            "velocity": tuple(self.state[IDX_VX:IDX_VZ + 1]),
            "position_std": tuple(std[IDX_X:IDX_Z + 1]),
            "attitude_std_rad": tuple(std[IDX_ROLL:IDX_YAW + 1]),
        }

    def get_se3_pose(self) -> np.ndarray:
        """4x4 SE(3) matrix combining the fused position and attitude."""
        from app.mavlink_imu import euler_to_quaternion, quaternion_to_rotation_matrix
        roll, pitch, yaw = self.state[IDX_ROLL:IDX_YAW + 1]
        R = quaternion_to_rotation_matrix(*euler_to_quaternion(roll, pitch, yaw))
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = self.state[IDX_X:IDX_Z + 1]
        return T

    def get_statistics(self) -> dict:
        eigenvalues = np.linalg.eigvalsh(self.covariance)
        return {
            "predict_count": self.predict_count,
            "position_update_count": self.position_update_count,
            "attitude_update_count": self.attitude_update_count,
            "skipped_singular_updates": self.skipped_singular_updates,
            "covariance_trace": float(np.trace(self.covariance)),
            "covariance_min_eigenvalue": float(np.min(eigenvalues)),
            "covariance_max_eigenvalue": float(np.max(eigenvalues)),
        }
