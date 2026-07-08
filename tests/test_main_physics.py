"""
Targeted tests for main.py physics/engineering fixes:
- ENU projection with correct compass-heading convention (Physics Audit C4)
- readings_per_second as an actual rate, not a raw count (H5)
- Monotonic timestamp threaded into the rate-of-change gate (H9)
"""

import sys
import os
import time
from datetime import datetime

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.main import LiDARSLAMApplication
from app.lidar_driver import LiDARReading


def make_reading(distance=3.0, strength=200, mono_ts=None):
    return LiDARReading(
        distance=distance,
        signal_strength=strength,
        timestamp=datetime.now(),
        temperature=25.0,
        valid=True,
        mono_timestamp=mono_ts if mono_ts is not None else time.monotonic()
    )


@pytest.fixture
def mapping_app():
    app = LiDARSLAMApplication()
    app.mode = LiDARSLAMApplication.MODE_MAPPING
    return app


class TestENUProjection:
    """Compass heading (clockwise-positive from North) -> ENU world frame
    (x=East, y=North). u(heading) = [sin(h), cos(h)]."""

    def test_heading_north_projects_along_y(self, mapping_app):
        mapping_app.set_heading(0.0)
        mapping_app._process_reading(make_reading(distance=3.0))
        point = mapping_app.slam_engine.scan_buffer[0]
        assert abs(point[0] - 0.0) < 1e-6  # East component
        assert abs(point[1] - 3.0) < 1e-6  # North component

    def test_heading_east_projects_along_x(self, mapping_app):
        mapping_app.set_heading(90.0)
        mapping_app._process_reading(make_reading(distance=3.0))
        point = mapping_app.slam_engine.scan_buffer[0]
        assert abs(point[0] - 3.0) < 1e-6
        assert abs(point[1] - 0.0) < 1e-6

    def test_heading_south_projects_negative_y(self, mapping_app):
        mapping_app.set_heading(180.0)
        mapping_app._process_reading(make_reading(distance=3.0))
        point = mapping_app.slam_engine.scan_buffer[0]
        assert abs(point[0] - 0.0) < 1e-6
        assert abs(point[1] + 3.0) < 1e-6

    def test_heading_west_projects_negative_x(self, mapping_app):
        mapping_app.set_heading(270.0)
        mapping_app._process_reading(make_reading(distance=3.0))
        point = mapping_app.slam_engine.scan_buffer[0]
        assert abs(point[0] + 3.0) < 1e-6
        assert abs(point[1] - 0.0) < 1e-6

    def test_heading_northeast_45_splits_evenly(self, mapping_app):
        """A NE (45deg) heading should split the range evenly between East
        and North components -- a mirrored (wrong) convention would still
        pass this specific case, but it catches gross sign/axis swaps."""
        mapping_app.set_heading(45.0)
        mapping_app._process_reading(make_reading(distance=np.sqrt(2)))
        point = mapping_app.slam_engine.scan_buffer[0]
        assert abs(point[0] - 1.0) < 1e-6
        assert abs(point[1] - 1.0) < 1e-6


class TestReadingsPerSecond:
    def test_rate_divides_by_elapsed_time_not_raw_count(self, mapping_app):
        """Physics Audit H5: a sparse stream (readings arriving over a
        longer-than-1s window) must not report the raw count as if the
        window were exactly 1.000s."""
        mapping_app._last_rate_check = time.time() - 2.0  # pretend 2s elapsed
        mapping_app._reading_count = 20
        # Feed one more reading to trigger the rate-window check
        mapping_app._on_lidar_reading(make_reading(distance=2.0))

        # 21 readings over ~2s -> rate should be roughly 10/s, NOT 21
        assert mapping_app.readings_per_second < 15
        assert mapping_app.readings_per_second > 5

    def test_rate_is_float_not_truncated_count(self, mapping_app):
        mapping_app._last_rate_check = time.time() - 1.5
        mapping_app._reading_count = 14
        mapping_app._on_lidar_reading(make_reading(distance=2.0))
        # 15 readings / 1.5s = 10.0 -- verify it's not just "15"
        assert mapping_app.readings_per_second != 15


class TestMonotonicRateGate:
    def test_mono_timestamp_used_for_rate_gate(self, mapping_app):
        """Physics Audit H9: the data-quality rate gate must key off
        reading.mono_timestamp, not wall-clock datetime, so it stays
        immune to NTP steps."""
        t0 = time.monotonic()
        # Seed the validator with an initial accepted reading via the
        # normal ingestion path. Stay within LiDARConfig.max_range (default
        # 4.0m, Physics Audit C2) so this isn't rejected as out-of-range.
        mapping_app._on_lidar_reading(make_reading(distance=0.5, mono_ts=t0))

        # A reading 10ms later with a large-but-in-range distance jump
        # (3.4m in 10ms = 340 m/s) should be caught by the rate gate using
        # the monotonic clock, independent of whatever the wall-clock
        # datetime says.
        far = make_reading(distance=3.9, mono_ts=t0 + 0.01)
        mapping_app._on_lidar_reading(far)

        stats = mapping_app.data_quality.get_statistics()
        assert stats['rejected_by']['rate'] >= 1


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
