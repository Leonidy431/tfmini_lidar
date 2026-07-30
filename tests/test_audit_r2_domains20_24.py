"""
Regression tests for the Blind Spot Audit R2 domains 20-24 mechanical fixes
(backend only; frontend JS fixes are verified by inspection).
"""

import sys
import os
import time
from datetime import datetime

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.main import LiDARSLAMApplication
from app.lidar_driver import TFminiSDriver, LiDARReading
from app.environmental_correction import EnvironmentalCorrector, DepthCorrectedRefractive


def make_reading(distance=3.0, strength=200, temperature=20.0, mono_ts=None):
    return LiDARReading(distance=distance, signal_strength=strength,
                        timestamp=datetime.now(), temperature=temperature,
                        valid=True, mono_timestamp=mono_ts or time.monotonic())


def build_frame(distance_cm, strength=200, temp_raw=400):
    frame = bytes([0x59, 0x59, distance_cm & 0xFF, (distance_cm >> 8) & 0xFF,
                   strength & 0xFF, (strength >> 8) & 0xFF,
                   temp_raw & 0xFF, (temp_raw >> 8) & 0xFF])
    return frame + bytes([sum(frame[:8]) & 0xFF])


class TestDepthStaleness:
    """Domain 22: a stale depth sample must not keep biasing ranges."""

    def _app(self):
        app = LiDARSLAMApplication()
        app.environmental_corrector = EnvironmentalCorrector(
            depth_model=DepthCorrectedRefractive(a=1.333, b=0.001, c=0.0))
        return app

    def test_fresh_depth_applied(self):
        app = self._app()
        app.set_depth(40.0)
        corrected = app._apply_environmental_correction(make_reading(3.0))
        assert corrected != pytest.approx(3.0)  # depth correction active

    def test_stale_depth_falls_back_to_constant_n(self):
        app = self._app()
        app.set_depth(40.0)
        app._depth_timestamp = time.monotonic() - (app.DEPTH_TIMEOUT_S + 1.0)
        corrected = app._apply_environmental_correction(make_reading(3.0))
        # Stale depth -> depth_m=None -> constant-n path (b,c ignored)
        assert corrected == pytest.approx(3.0)

    def test_fresh_depth_helper(self):
        app = self._app()
        assert app._fresh_depth() is None  # nothing set
        app.set_depth(12.0)
        assert app._fresh_depth() == pytest.approx(12.0)


class TestHealthFreshness:
    """Domain 24 #9: connected-but-mute sensor must not read 'healthy'."""

    class _FakeDriver:
        is_connected = True
        def __init__(self, since): self._since = since
        def get_statistics(self):
            return {'error_rate': 0.0, 'reconnect_attempts': 0,
                    'seconds_since_last_read': self._since, 'last_error': None}

    def test_stale_reads_degrade_health(self):
        app = LiDARSLAMApplication()
        app.driver = self._FakeDriver(since=10.0)
        app._is_running = True
        health = app.get_health()
        assert health['state'] == 'degraded'
        assert 'stale_readings' in health['reasons']

    def test_fresh_reads_stay_healthy(self):
        app = LiDARSLAMApplication()
        app.driver = self._FakeDriver(since=0.2)
        app._is_running = True
        app.mode = app.MODE_IDLE  # avoid no_heading_source in mapping modes
        health = app.get_health()
        assert 'stale_readings' not in health['reasons']


class TestDriverObservabilityCounters:
    """Domain 24 #11/#12: separate frame errors and invalid readings."""

    def test_frame_error_counter_on_bad_checksum(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        bad = bytes([0x59, 0x59, 0x96, 0x00, 0xC8, 0x00, 0x90, 0x01, 0xFF])
        driver.buffer.extend(bad)
        driver._process_buffer()
        assert driver.frame_errors > 0
        stats = driver.get_statistics()
        assert 'frame_errors' in stats
        assert 'invalid_readings' in stats

    def test_invalid_reading_counter_on_sentinel(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.add_callback(lambda r: None)
        # 65535cm distance is the weak-signal sentinel -> valid=False
        driver.buffer.extend(build_frame(65535, strength=50))
        driver._process_buffer()
        assert driver.invalid_readings >= 1

    def test_valid_reading_not_counted_invalid(self):
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.add_callback(lambda r: None)
        driver.buffer.extend(build_frame(150, strength=200))
        driver._process_buffer()
        assert driver.invalid_readings == 0


class TestFusionMetricsExposure:
    """Domain 24 #7: enabled D1/D2/D3/D8 modules expose metrics."""

    def test_default_build_empty(self):
        app = LiDARSLAMApplication()
        assert app._get_fusion_metrics() == {}

    def test_enabled_modules_reported(self):
        from app.multipath_detector import MultipathDetector
        from app.ekf_3d_attitude import EKF3DAttitude
        app = LiDARSLAMApplication()
        app.multipath_detector = MultipathDetector()
        app.ekf = EKF3DAttitude()
        metrics = app._get_fusion_metrics()
        assert 'multipath' in metrics and 'rejected_total' in metrics['multipath']
        assert 'ekf' in metrics and 'predict_count' in metrics['ekf']

    def test_status_includes_sensor_fusion_key(self):
        app = LiDARSLAMApplication()
        status = app.get_status()
        assert 'sensor_fusion' in status


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
