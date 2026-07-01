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


class TestStatistics:
    def test_statistics_shape(self):
        v = DataQualityValidator()
        v.validate(distance=2.0, signal_strength=200)
        stats = v.get_statistics()
        assert 'quality_score' in stats
        assert 'rejection_rate' in stats
        assert 'rejected_by' in stats
        assert set(stats['rejected_by'].keys()) == {'range', 'signal', 'iqr', 'zscore'}

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
