"""
Tests for the data quality validation module (Rule 4).

Pure-stdlib module, so these tests run with no third-party dependencies.
"""

import sys
import os

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.data_quality import DataQualityValidator


class TestPhysicalBounds:
    def test_below_min_range_rejected(self):
        v = DataQualityValidator(min_range=0.1, max_range=12.0)
        result = v.validate(distance=0.05, signal_strength=200)
        assert result.accepted is False
        assert result.reason == 'out_of_range'

    def test_above_max_range_rejected(self):
        v = DataQualityValidator(min_range=0.1, max_range=12.0)
        result = v.validate(distance=15.0, signal_strength=200)
        assert result.accepted is False

    def test_low_signal_rejected(self):
        v = DataQualityValidator(signal_threshold=100)
        result = v.validate(distance=2.0, signal_strength=50)
        assert result.accepted is False
        assert result.reason == 'low_signal'


class TestStatisticalFilters:
    def test_stable_stream_accepted(self):
        v = DataQualityValidator(min_samples=10)
        accepted = 0
        for _ in range(50):
            r = v.validate(distance=2.0, signal_strength=200)
            if r.accepted:
                accepted += 1
        assert accepted == 50

    def test_zscore_outlier_rejected(self):
        v = DataQualityValidator(min_samples=10, z_threshold=3.0)
        # Build a stable window with slight variation so std > 0
        for i in range(30):
            v.validate(distance=2.0 + (0.001 if i % 2 else -0.001), signal_strength=200)
        # A large spike should be rejected as z-score outlier
        result = v.validate(distance=8.0, signal_strength=200)
        assert result.accepted is False
        assert result.reason in ('zscore_outlier', 'iqr_outlier')

    def test_insufficient_history_accepts(self):
        v = DataQualityValidator(min_samples=10)
        # First reading, no history yet -> accepted even if unusual
        result = v.validate(distance=5.0, signal_strength=200)
        assert result.accepted is True
        assert result.reason == 'insufficient_history'

    def test_gradual_drift_accepted(self):
        """Slow legitimate changes should not be rejected as outliers."""
        v = DataQualityValidator(min_samples=10, z_threshold=3.0)
        rejected = 0
        d = 2.0
        for _ in range(100):
            r = v.validate(distance=d, signal_strength=200)
            if not r.accepted:
                rejected += 1
            d += 0.02  # gradual ramp
        # Allow a few edge rejections but the ramp should mostly pass
        assert rejected < 15


class TestRegimeChange:
    """Physics Audit C8: a legitimate step change must not permanently
    lock out the new regime."""

    def test_zscore_lockout_recovers_after_threshold(self):
        v = DataQualityValidator(min_samples=10, z_threshold=3.0,
                                 regime_change_after=5)
        # Establish a stable regime around 2.0m with slight jitter (a
        # perfectly uniform window has std=0/iqr=0, which trivially
        # disables both statistical gates -- not representative of a real
        # sensor).
        for i in range(30):
            v.validate(distance=2.0 + (0.005 if i % 2 else -0.005),
                      signal_strength=200)

        # A real step to 6.0m: first several readings are statistical
        # outliers against the 2.0m window and get rejected...
        results = [v.validate(distance=6.0, signal_strength=200)
                   for _ in range(4)]
        assert all(r.accepted is False for r in results)

        # ...but after regime_change_after consecutive rejections, the
        # filter must recover instead of rejecting forever.
        recovered = [v.validate(distance=6.0, signal_strength=200)
                    for _ in range(10)]
        assert any(r.accepted for r in recovered)
        assert v.regime_changes >= 1

    def test_rate_gate_lockout_recovers(self):
        v = DataQualityValidator(min_samples=5, max_rate_m_per_s=15.0,
                                 regime_change_after=3)
        t = 0.0
        v.validate(distance=2.0, signal_strength=200, timestamp=t)

        # Large instantaneous jump repeatedly rejected by the rate gate...
        for _ in range(2):
            t += 0.01
            r = v.validate(distance=8.0, signal_strength=200, timestamp=t)
            assert r.accepted is False

        # ...must eventually recover rather than reject forever, since
        # _last_distance never advances on a pure rate-based reject.
        recovered = False
        for _ in range(10):
            t += 0.01
            r = v.validate(distance=8.0, signal_strength=200, timestamp=t)
            if r.accepted:
                recovered = True
                break
        assert recovered

    def test_physical_bound_rejects_do_not_trigger_regime_change(self):
        """Out-of-range/low-signal readings are genuinely invalid, not a
        regime change -- repeating them must never force an accept."""
        v = DataQualityValidator(min_range=0.1, max_range=12.0,
                                 regime_change_after=3)
        for _ in range(10):
            r = v.validate(distance=99.0, signal_strength=200)
            assert r.accepted is False
            assert r.reason == 'out_of_range'
        assert v.regime_changes == 0


class TestRateOfChange:
    def test_impossible_jump_rejected(self):
        v = DataQualityValidator(min_samples=5, max_rate_m_per_s=15.0)
        # Seed a timed reading
        v.validate(distance=2.0, signal_strength=200, timestamp=100.0)
        # 10m jump in 1ms -> 10000 m/s, far above the 15 m/s gate
        result = v.validate(distance=12.0, signal_strength=200, timestamp=100.001)
        assert result.accepted is False
        assert result.reason == 'rate_exceeded'

    def test_plausible_change_accepted(self):
        v = DataQualityValidator(min_samples=5, max_rate_m_per_s=15.0)
        v.validate(distance=2.0, signal_strength=200, timestamp=100.0)
        # 0.1m over 100ms = 1 m/s, well within limit
        result = v.validate(distance=2.1, signal_strength=200, timestamp=100.1)
        assert result.accepted is True

    def test_no_timestamp_skips_rate(self):
        v = DataQualityValidator(min_samples=5)
        v.validate(distance=2.0, signal_strength=200)
        # Big jump but no timestamp -> rate filter is skipped (may still pass
        # since history is short)
        result = v.validate(distance=9.0, signal_strength=200)
        assert result.reason != 'rate_exceeded'


class TestTemperatureCompensation:
    def test_noop_by_default(self):
        v = DataQualityValidator()
        assert v.compensate_temperature(2.0, 30.0) == 2.0

    def test_applies_coefficient(self):
        v = DataQualityValidator(temp_coefficient=0.001)
        # First call sets baseline -> no change
        assert v.compensate_temperature(2.0, 20.0) == 2.0
        # 10 degrees above baseline -> 2.0 * (1 + 0.001*10) = 2.02
        assert abs(v.compensate_temperature(2.0, 30.0) - 2.02) < 1e-9


class TestQualityScore:
    def test_perfect_stream_scores_high(self):
        v = DataQualityValidator(min_samples=5)
        for _ in range(50):
            v.validate(distance=2.0, signal_strength=200)
        assert v.quality_score > 0.95

    def test_noisy_stream_scores_lower(self):
        v = DataQualityValidator(min_samples=5, signal_threshold=100)
        for i in range(50):
            # Half the readings are invalid (low signal)
            strength = 200 if i % 2 == 0 else 10
            v.validate(distance=2.0, signal_strength=strength)
        assert v.quality_score < 0.7

    def test_reset_clears_state(self):
        v = DataQualityValidator()
        for _ in range(20):
            v.validate(distance=2.0, signal_strength=200)
        v.reset()
        assert v.total_checked == 0
        assert v.quality_score == 1.0

    def test_concurrent_validate_and_quality_score_read(self):
        """Regression test for Blind Spot Audit R3 R3-CONC-1: quality_score
        raced with _accept()/_reject() appending to the same deque from the
        serial-read thread, while quality_score is polled from the Flask
        thread via get_health()/get_status() -- a poll landing mid-append
        could raise 'deque mutated during iteration' and 500 an unrelated
        request."""
        import concurrent.futures

        v = DataQualityValidator(min_samples=5)
        errors = []

        def writer():
            for i in range(500):
                v.validate(distance=2.0 + (i % 3) * 0.01, signal_strength=200)

        def reader():
            for _ in range(500):
                try:
                    _ = v.quality_score
                except RuntimeError as exc:
                    errors.append(exc)

        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            futures = [pool.submit(writer), pool.submit(reader),
                       pool.submit(reader), pool.submit(writer)]
            for f in futures:
                f.result()

        assert errors == []


class TestStatistics:
    def test_statistics_shape(self):
        v = DataQualityValidator()
        v.validate(distance=2.0, signal_strength=200)
        stats = v.get_statistics()
        assert 'quality_score' in stats
        assert 'rejection_rate' in stats
        assert 'rejected_by' in stats
        assert set(stats['rejected_by'].keys()) == {'range', 'signal', 'iqr', 'zscore', 'rate'}

    def test_rejection_counters(self):
        v = DataQualityValidator(min_range=0.1, max_range=12.0, signal_threshold=100)
        v.validate(distance=100.0, signal_strength=200)   # range
        v.validate(distance=2.0, signal_strength=10)      # signal
        stats = v.get_statistics()
        assert stats['rejected_by']['range'] == 1
        assert stats['rejected_by']['signal'] == 1
        assert stats['total_rejected'] == 2


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
