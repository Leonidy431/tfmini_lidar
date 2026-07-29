"""
Integration tests for the D1/D2/D3-D4/D8 wiring into app/main.py.

These exercise the glue code in LiDARSLAMApplication (_project_beam,
_apply_environmental_correction, _update_ekf, and the multipath gate in
_on_lidar_reading) directly against the new modules, without going through
Config env vars -- tests attach the optional components post-construction,
which is equivalent to what happens when the corresponding ENABLE_* flag is
set before the app starts.
"""

import sys
import os
import math
import time
from datetime import datetime

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.main import LiDARSLAMApplication
from app.lidar_driver import LiDARReading
from app.mavlink_imu import MAVLinkAttitudeReader
from app.multipath_detector import MultipathDetector
from app.environmental_correction import (
    EnvironmentalCorrector, DepthCorrectedRefractive, TemperatureCorrection
)
from app.ekf_3d_attitude import EKF3DAttitude


def make_reading(distance=3.0, strength=200, temperature=20.0, mono_ts=None):
    return LiDARReading(
        distance=distance,
        signal_strength=strength,
        timestamp=datetime.now(),
        temperature=temperature,
        valid=True,
        mono_timestamp=mono_ts if mono_ts is not None else time.monotonic()
    )


@pytest.fixture
def mapping_app():
    app = LiDARSLAMApplication()
    app.mode = LiDARSLAMApplication.MODE_MAPPING
    return app


class TestDefaultsDisabled:
    def test_all_deferred_modules_none_by_default(self, mapping_app):
        assert mapping_app.mavlink_attitude is None
        assert mapping_app.multipath_detector is None
        assert mapping_app.environmental_corrector is None
        assert mapping_app.ekf is None

    def test_health_reports_deferred_fields_inactive(self, mapping_app):
        health = mapping_app.get_health()
        assert health['attitude_3d_active'] is False
        assert health['multipath_rejected_count'] == 0
        assert health['ekf_fusion_active'] is False


class TestBeam3DProjection:
    def test_reduces_to_1d_formula_at_zero_roll_pitch(self, mapping_app):
        for heading_deg in [0.0, 45.0, 90.0, 180.0, 270.0]:
            heading_rad = math.radians(heading_deg)
            east, north, up = LiDARSLAMApplication._compute_3d_beam_offset(
                3.0, roll=0.0, pitch=0.0, yaw_compass_rad=heading_rad)
            assert abs(east - 3.0 * math.sin(heading_rad)) < 1e-9
            assert abs(north - 3.0 * math.cos(heading_rad)) < 1e-9
            assert abs(up - 0.0) < 1e-9

    def test_project_beam_uses_fresh_3d_attitude(self, mapping_app):
        mapping_app.set_heading(0.0)  # fallback would point north
        mapping_app.mavlink_attitude = MAVLinkAttitudeReader(timeout_s=5.0)
        # Pitch the sensor down 30deg with yaw=90 (east): should now have
        # a nonzero (negative) Up component in addition to East.
        mapping_app.mavlink_attitude.ingest_raw(
            roll=0.0, pitch=math.radians(30.0), yaw=math.radians(90.0))

        world_x, world_y, world_z = mapping_app._project_beam(3.0, (0.0, 0.0, 0.0))
        assert world_z != 0.0  # pitch contributes vertical component
        assert world_x > 0.0   # still generally eastward

    def test_project_beam_falls_back_when_attitude_stale(self, mapping_app):
        mapping_app.set_heading(90.0)  # east
        mapping_app.mavlink_attitude = MAVLinkAttitudeReader(timeout_s=0.01)
        mapping_app.mavlink_attitude.ingest_raw(
            roll=0.5, pitch=0.5, yaw=0.0, mono_timestamp=time.monotonic() - 10.0)

        world_x, world_y, world_z = mapping_app._project_beam(3.0, (0.0, 0.0, 0.0))
        # Stale sample -> falls back to 1D heading (east, no vertical component)
        assert abs(world_x - 3.0) < 1e-6
        assert abs(world_z - 0.0) < 1e-9

    def test_end_to_end_process_reading_uses_3d_attitude(self, mapping_app):
        mapping_app.mavlink_attitude = MAVLinkAttitudeReader(timeout_s=5.0)
        mapping_app.mavlink_attitude.ingest_raw(roll=0.0, pitch=0.0, yaw=0.0)
        mapping_app._process_reading(make_reading(distance=3.0))
        point = mapping_app.slam_engine.scan_buffer[0]
        assert abs(point[1] - 3.0) < 1e-6  # yaw=0 -> north, matches 1D convention


class TestMultipathIntegration:
    def test_multipath_reading_not_enqueued(self, mapping_app):
        # Signal thresholds must clear DataQualityValidator's own gate
        # (Config.lidar.signal_threshold=100, checked upstream of the
        # multipath detector) while still tripping the multipath
        # detector's own (higher) threshold, so this test isolates the
        # multipath stage rather than re-testing Rule 4 filtering.
        mapping_app.multipath_detector = MultipathDetector(
            window_size=200, min_samples=30, refit_every=5, signal_strength_threshold=150)
        rng = np.random.default_rng(11)

        # Warm up the detector directly: this test exercises the
        # _on_lidar_reading -> multipath_detector.check() wiring, not the
        # interaction with DataQualityValidator's own adaptive IQR/Z-score
        # state (covered by tests/test_data_quality.py), so avoid routing
        # 180 synthetic warm-up readings through that adaptive gate too.
        for _ in range(150):
            mapping_app.multipath_detector.check(float(rng.normal(3.0, 0.05)), 200)
        for _ in range(30):
            mapping_app.multipath_detector.check(float(rng.normal(1.0, 0.05)), 120)
        assert mapping_app.multipath_detector.warmed_up is True

        queue_size_before = mapping_app._processing_queue.qsize()
        rejected_before = mapping_app._multipath_rejected_count

        # A clear multipath reading: near the scattered cluster, signal
        # strong enough to pass Rule 4 but weak relative to the multipath
        # detector's own threshold.
        mapping_app._on_lidar_reading(make_reading(distance=1.0, strength=120))

        assert mapping_app._multipath_rejected_count == rejected_before + 1
        assert mapping_app._processing_queue.qsize() == queue_size_before

    def test_direct_path_reading_still_enqueued(self, mapping_app):
        mapping_app.multipath_detector = MultipathDetector(min_samples=5, refit_every=1)
        for _ in range(10):
            mapping_app._on_lidar_reading(make_reading(distance=3.0, strength=200))
        assert mapping_app._processing_queue.qsize() > 0


class TestEnvironmentalCorrectionIntegration:
    def test_no_depth_no_temperature_is_noop(self, mapping_app):
        mapping_app.environmental_corrector = EnvironmentalCorrector()
        reading = make_reading(distance=3.0)
        corrected = mapping_app._apply_environmental_correction(reading)
        assert corrected == pytest.approx(reading.distance)

    def test_depth_correction_changes_distance(self, mapping_app):
        mapping_app.environmental_corrector = EnvironmentalCorrector(
            depth_model=DepthCorrectedRefractive(a=1.333, b=0.001, c=0.0))
        mapping_app.set_depth(40.0)
        reading = make_reading(distance=3.0)
        corrected = mapping_app._apply_environmental_correction(reading)
        assert corrected != pytest.approx(reading.distance)

    def test_temperature_correction_requires_flag(self, mapping_app):
        """Reading.temperature is only consulted when
        Config.environmental_correction.temperature_enabled -- this test
        drives the app-level flag directly since it's read from the
        dataclass Config singleton."""
        mapping_app.environmental_corrector = EnvironmentalCorrector()
        mapping_app.config.environmental_correction.temperature_enabled = True
        try:
            reading = make_reading(distance=3.0, temperature=5.0)
            corrected = mapping_app._apply_environmental_correction(reading)
            assert corrected != pytest.approx(reading.distance)
        finally:
            mapping_app.config.environmental_correction.temperature_enabled = False


class TestEKFIntegration:
    def test_ekf_predict_and_update_counts_increase(self, mapping_app):
        mapping_app.ekf = EKF3DAttitude()
        t0 = time.monotonic()
        mapping_app._process_reading(make_reading(distance=3.0, mono_ts=t0))
        mapping_app._process_reading(make_reading(distance=3.0, mono_ts=t0 + 0.1))

        stats = mapping_app.ekf.get_statistics()
        assert stats['position_update_count'] == 2
        assert stats['predict_count'] == 1  # first call has no prior timestamp to diff against

    def test_ekf_fuses_attitude_when_available(self, mapping_app):
        mapping_app.ekf = EKF3DAttitude()
        mapping_app.mavlink_attitude = MAVLinkAttitudeReader(timeout_s=5.0)
        mapping_app.mavlink_attitude.ingest_raw(roll=0.1, pitch=0.05, yaw=0.2)

        mapping_app._process_reading(make_reading(distance=3.0))

        stats = mapping_app.ekf.get_statistics()
        assert stats['attitude_update_count'] == 1

    def test_health_reflects_ekf_active(self, mapping_app):
        mapping_app.ekf = EKF3DAttitude()
        assert mapping_app.get_health()['ekf_fusion_active'] is True


class TestSetDepth:
    def test_set_depth_updates_state(self, mapping_app):
        assert mapping_app.current_depth_m is None
        mapping_app.set_depth(12.5)
        assert mapping_app.current_depth_m == pytest.approx(12.5)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
