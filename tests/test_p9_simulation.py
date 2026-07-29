"""
P9 Simulation Validation Campaign (Physics Audit D1/D2/D3/D4/D8).

TECHNICAL_SPECIFICATION.md defines P9 as "Empirical tuning on ROV hardware
OR VALIDATED SIMULATION". Real hardware (IMU, turbidity tank, depth pool,
oven) is not available in this environment, so this suite runs the
simulation half of P9 against the physically-grounded environment model in
tests/emulation_server.py (Beer-Lambert attenuation, n(depth, salinity,
temperature), signal-strength degradation), which acts as ground truth.

Each test both ASSERTS a quantitative acceptance threshold and PRINTS the
measured metric (run with `pytest -s` to see the numbers; they are also
transcribed into docs/P9_SIMULATION_VALIDATION.md).

Field P9 on real hardware remains open and is tracked in
DEVELOPMENT_BACKLOG.md -- this suite validates the calibration PROCEDURES
and the end-to-end error-reduction claims, not the real ocean.
"""

import sys
import os
import math

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.emulation_server import UnderwaterEnvironment
from app.environmental_correction import (
    EnvironmentalCorrector, DepthCorrectedRefractive, TemperatureCorrection
)
from app.multipath_detector import MultipathDetector
from app.ekf_3d_attitude import EKF3DAttitude, wrap_angle
from app.mavlink_imu import MAVLinkAttitudeReader
from app.main import LiDARSLAMApplication


# ============================================================================
# D3: Depth-dependent refractive index -- calibration against emulator truth
# ============================================================================

class TestD3DepthCalibrationSim:
    """P9-sim: fit n(z) from emulator-generated calibration data, then show
    the fitted model beats the constant-n correction on held-out depths."""

    SALINITY = 35.0
    TEMP_C = 15.0

    def _true_n(self, depth_m: float) -> float:
        env = UnderwaterEnvironment(depth_m=depth_m, temperature_c=self.TEMP_C,
                                    salinity_ppt=self.SALINITY)
        return env.refractive_index()

    def test_calibration_recovers_emulator_model(self):
        cal_depths = [0.0, 5.0, 10.0, 20.0, 30.0, 50.0]
        n_measured = [self._true_n(z) for z in cal_depths]

        fitted = EnvironmentalCorrector().calibrate_depth_model(cal_depths, n_measured)

        held_out = [2.5, 15.0, 25.0, 40.0]
        residuals = [abs(fitted.get_refractive_index(z) - self._true_n(z))
                     for z in held_out]
        max_residual = max(residuals)
        print(f"\n[D3] fitted n(z): a={fitted.a:.6f} b={fitted.b:.2e} c={fitted.c:.2e}; "
              f"max held-out |n_fit - n_true| = {max_residual:.2e}")
        # Emulator model is quadratic-or-lower in z, so degree-2 fit must
        # recover it to numerical precision.
        assert max_residual < 1e-6

    def test_fitted_model_beats_constant_n_on_distance(self):
        cal_depths = [0.0, 5.0, 10.0, 20.0, 30.0, 50.0]
        fitted = EnvironmentalCorrector().calibrate_depth_model(
            cal_depths, [self._true_n(z) for z in cal_depths])

        true_distance_m = 3.0
        errors_constant, errors_fitted = [], []
        for z in [2.5, 15.0, 25.0, 40.0, 50.0]:
            n_true = self._true_n(z)
            raw_cm = true_distance_m * n_true * 100.0  # what the ToF sensor reports
            d_constant = (raw_cm / 100.0) / 1.333
            d_fitted = fitted.correct_distance(raw_cm, z)
            errors_constant.append(abs(d_constant - true_distance_m))
            errors_fitted.append(abs(d_fitted - true_distance_m))

        mean_const = float(np.mean(errors_constant))
        mean_fit = float(np.mean(errors_fitted))
        print(f"[D3] mean |distance error| at 3.0m: constant-n={mean_const*1000:.3f}mm, "
              f"fitted n(z)={mean_fit*1000:.5f}mm")
        assert mean_fit < mean_const
        assert mean_fit < 1e-4  # < 0.1 mm residual on noiseless sim data


# ============================================================================
# D4: Temperature compensation -- calibration procedure on simulated drift
# ============================================================================

class TestD4TemperatureCalibrationSim:
    """P9-sim: simulate the TFmini-S ToF drift envelope (datasheet
    ~+/-0.05%/degC), run the oven-calibration procedure from
    TECHNICAL_SPECIFICATION.md D4-P9 on synthetic (temp, distance) pairs,
    and verify the fitted TemperatureCorrection recovers true distance."""

    TRUE_SENSOR_SLOPE = -0.0004  # sensor reads short when cold (per degC)
    TRUE_DISTANCE_M = 1.0        # oven reference marker at 1 m

    def _measured(self, temp_c: float) -> float:
        return self.TRUE_DISTANCE_M * (1.0 + self.TRUE_SENSOR_SLOPE * (temp_c - 20.0))

    def test_oven_procedure_recovers_correction_slope(self):
        oven_temps = [0.0, 10.0, 15.0, 20.0, 25.0, 30.0]
        # Calibration: fit ratio true/measured = 1 + c*(T - 20) by least squares
        ratios = np.array([self.TRUE_DISTANCE_M / self._measured(t) for t in oven_temps])
        dT = np.array(oven_temps) - 20.0
        c_fitted = float(np.polyfit(dT, ratios - 1.0, deg=1)[0])

        model = TemperatureCorrection(ref_temp_c=20.0, slope_per_degree=c_fitted)
        residuals = [abs(model.correct_distance(self._measured(t), t) - self.TRUE_DISTANCE_M)
                     for t in [5.0, 12.5, 27.5]]  # held-out temperatures
        max_residual_mm = max(residuals) * 1000.0
        print(f"\n[D4] true sensor slope={self.TRUE_SENSOR_SLOPE:+.4f}/degC, "
              f"fitted correction slope={c_fitted:+.6f}/degC, "
              f"max held-out residual={max_residual_mm:.4f}mm at 1.0m")
        # First-order inversion of a 0.04%/degC drift: residual is O(drift^2)
        assert max_residual_mm < 0.5
        assert abs(c_fitted - (-self.TRUE_SENSOR_SLOPE)) < 5e-5


# ============================================================================
# D2: Multipath detection -- detection/false-positive rates on emulator-style
# turbid-water streams
# ============================================================================

class TestD2MultipathRatesSim:
    """P9-sim: measure detection and false-positive rates on a synthetic
    turbid-water stream shaped like the emulator's physics (weak scattered
    returns at short range, strong direct returns at true range).

    Acceptance (TECHNICAL_SPECIFICATION.md D2): detection > 85% at 2-5 NTU
    equivalent, false positives < 5% on clean direct-path readings."""

    def _warmed_detector(self, rng):
        det = MultipathDetector(window_size=300, min_samples=30, refit_every=5,
                                signal_strength_threshold=100)
        env = UnderwaterEnvironment(depth_m=10.0, turbidity_ntu=3.0)
        for _ in range(200):
            det.check(env.get_range_with_noise(3.0), env.get_signal_strength(3.0))
        for _ in range(40):
            det.check(float(rng.normal(1.0, 0.06)), int(rng.integers(20, 60)))
        return det

    def test_detection_and_false_positive_rates(self):
        rng = np.random.default_rng(42)
        np.random.seed(42)  # emulator uses global np.random
        det = self._warmed_detector(rng)

        n_trials = 200
        detected = sum(
            det.check(float(rng.normal(1.0, 0.06)), int(rng.integers(20, 60))).is_multipath
            for _ in range(n_trials)
        )
        env = UnderwaterEnvironment(depth_m=10.0, turbidity_ntu=3.0)
        false_pos = sum(
            det.check(env.get_range_with_noise(3.0), env.get_signal_strength(3.0)).is_multipath
            for _ in range(n_trials)
        )

        detection_rate = detected / n_trials
        fp_rate = false_pos / n_trials
        print(f"\n[D2] detection rate={detection_rate:.1%}, "
              f"false-positive rate={fp_rate:.1%} (n={n_trials} each, 3 NTU)")
        assert detection_rate > 0.85
        assert fp_rate < 0.05


# ============================================================================
# D8: EKF fusion -- RMSE improvement vs raw measurements on a simulated
# constant-velocity trajectory
# ============================================================================

class TestD8EKFTrajectorySim:
    """P9-sim: constant-velocity 3D trajectory with Gaussian measurement
    noise matched to the EKF's R. Acceptance: fused position RMSE at least
    15% below raw-measurement RMSE (TECHNICAL_SPECIFICATION.md D8 target),
    measured after a convergence burn-in."""

    def test_position_rmse_improvement(self):
        rng = np.random.default_rng(7)
        ekf = EKF3DAttitude(measurement_noise_position=0.15)
        velocity = np.array([0.5, 0.2, -0.1])
        dt, n_steps, burn_in = 0.1, 400, 100
        meas_sigma = 0.15

        errors_raw, errors_ekf = [], []
        for k in range(n_steps):
            true_pos = velocity * (k * dt)
            meas = true_pos + rng.normal(0.0, meas_sigma, 3)
            ekf.predict(dt)
            ekf.update_position(meas)
            if k >= burn_in:
                est = np.array(ekf.get_state()["position"])
                errors_raw.append(np.sum((meas - true_pos) ** 2))
                errors_ekf.append(np.sum((est - true_pos) ** 2))

        rmse_raw = math.sqrt(float(np.mean(errors_raw)))
        rmse_ekf = math.sqrt(float(np.mean(errors_ekf)))
        improvement = 1.0 - rmse_ekf / rmse_raw
        print(f"\n[D8] position RMSE: raw={rmse_raw:.3f}m, fused={rmse_ekf:.3f}m, "
              f"improvement={improvement:.1%} (target >15%)")
        assert improvement > 0.15

    def _run_attitude_sim(self, process_noise_attitude: float, seed: int = 11):
        """Returns (rmse_raw, rmse_fused), both 3-axis combined, for a
        near-constant attitude with sigma=0.02 measurement noise."""
        rng = np.random.default_rng(seed)
        ekf = EKF3DAttitude(measurement_noise_attitude=0.02,
                             process_noise_attitude=process_noise_attitude)
        true_att = np.array([0.1, -0.05, 2.8])
        att_sigma = 0.02
        errors_raw, errors_ekf = [], []
        for k in range(400):
            ekf.predict(0.1)
            meas = true_att + rng.normal(0.0, att_sigma, 3)
            ekf.update_attitude(*meas)
            if k >= 100:
                est = np.array(ekf.get_state()["attitude_rad"])
                err = np.array([wrap_angle(e - t) for e, t in zip(est, true_att)])
                errors_ekf.append(np.sum(err ** 2))
                errors_raw.append(np.sum((meas - true_att) ** 2))
        return (math.sqrt(float(np.mean(errors_raw))),
                math.sqrt(float(np.mean(errors_ekf))))

    def test_attitude_fusion_beats_raw_measurements(self):
        """Like-for-like: 3-axis fused RMSE vs 3-axis raw-measurement RMSE.
        With the DEFAULT process noise (0.05, tuned to track maneuvers) the
        smoothing margin is small by design; it must still not be worse."""
        rmse_raw, rmse_fused = self._run_attitude_sim(process_noise_attitude=0.05)
        print(f"\n[D8] attitude RMSE (default q=0.05): raw={rmse_raw:.4f}rad, "
              f"fused={rmse_fused:.4f}rad")
        assert rmse_fused < rmse_raw

    def test_attitude_smoothing_with_tuned_process_noise(self):
        """P8 hyperparameter guidance measured in sim: for a slowly-varying
        attitude (station-keeping ROV), lowering process_noise_attitude to
        0.001 buys substantial smoothing. This is the tuning lever field
        P8/P9 will set from real maneuver data."""
        rmse_raw, rmse_fused = self._run_attitude_sim(process_noise_attitude=0.001)
        improvement = 1.0 - rmse_fused / rmse_raw
        print(f"[D8] attitude RMSE (tuned q=0.001): raw={rmse_raw:.4f}rad, "
              f"fused={rmse_fused:.4f}rad, improvement={improvement:.1%}")
        assert improvement > 0.30


# ============================================================================
# D1: 3D beam projection -- geometric ground truth for a pitched sensor
# ============================================================================

class TestD1PitchedBeamGeometrySim:
    """P9-sim: closed-form geometric truth for a pitched/yawed beam, plus
    quantification of the error the 1D yaw-only projection would make on a
    tilted ROV (the improvement D1 exists to deliver)."""

    def test_pitched_beam_matches_closed_form(self):
        d, pitch_deg, yaw_deg = 3.0, 30.0, 90.0
        pitch, yaw = math.radians(pitch_deg), math.radians(yaw_deg)
        east, north, up = LiDARSLAMApplication._compute_3d_beam_offset(
            d, roll=0.0, pitch=pitch, yaw_compass_rad=yaw)
        # Aerospace convention: positive pitch = nose up. Closed form:
        # horizontal = d*cos(pitch) along heading, vertical = +d*sin(pitch).
        assert abs(east - d * math.cos(pitch) * math.sin(yaw)) < 1e-9
        assert abs(north - d * math.cos(pitch) * math.cos(yaw)) < 1e-9
        assert abs(up - d * math.sin(pitch)) < 1e-9

    def test_1d_projection_error_on_tilted_rov_quantified(self):
        """The 1D fallback places a pitched beam's endpoint at full range in
        the horizontal plane. At 30 deg pitch and 3 m range this is a 1.55 m
        3D endpoint error -- the error budget D1 removes."""
        d, pitch = 3.0, math.radians(30.0)
        endpoint_3d = np.array([0.0, d * math.cos(pitch), d * math.sin(pitch)])
        endpoint_1d = np.array([0.0, d, 0.0])
        error_m = float(np.linalg.norm(endpoint_3d - endpoint_1d))
        print(f"\n[D1] 1D-projection endpoint error at 30deg pitch, 3.0m range: "
              f"{error_m:.2f}m ({error_m/d:.0%} of range); 3D projection error: 0.00m")
        assert error_m > 1.5  # material error, justifying D1
        # End-to-end: the wired-in path reproduces the closed form
        app = LiDARSLAMApplication()
        app.mode = LiDARSLAMApplication.MODE_MAPPING
        app.mavlink_attitude = MAVLinkAttitudeReader(timeout_s=5.0)
        app.mavlink_attitude.ingest_raw(roll=0.0, pitch=pitch, yaw=0.0)
        wx, wy, wz = app._project_beam(d, (0.0, 0.0, 0.0))
        assert abs(wy - d * math.cos(pitch)) < 1e-9
        assert abs(wz - d * math.sin(pitch)) < 1e-9


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
