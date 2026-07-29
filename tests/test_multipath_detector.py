"""
Tests for the multipath/turbidity detector (Physics Audit D2).
"""

import sys
import os

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.multipath_detector import (
    MultipathDetector,
    fit_two_component_gmm,
    _gaussian_pdf,
)


class TestGaussianMixtureFit:
    def test_separates_two_clear_clusters(self):
        rng = np.random.default_rng(42)
        cluster_a = rng.normal(1.0, 0.05, 100)   # scattered/near
        cluster_b = rng.normal(3.0, 0.05, 100)   # direct/far
        samples = np.concatenate([cluster_a, cluster_b])

        means, stds, weights = fit_two_component_gmm(samples, n_iter=20)

        assert means[0] < means[1]
        assert abs(means[0] - 1.0) < 0.15
        assert abs(means[1] - 3.0) < 0.15
        assert abs(weights.sum() - 1.0) < 1e-6

    def test_single_sample_no_crash(self):
        means, stds, weights = fit_two_component_gmm(np.array([2.0]))
        assert len(means) == 2

    def test_gaussian_pdf_peak_at_mean(self):
        peak = _gaussian_pdf(np.array([1.0]), 1.0, 0.1)[0]
        off_peak = _gaussian_pdf(np.array([1.5]), 1.0, 0.1)[0]
        assert peak > off_peak


class TestMultipathDetectorWarmup:
    def test_accepts_everything_before_min_samples(self):
        det = MultipathDetector(min_samples=20)
        for i in range(15):
            verdict = det.check(distance_m=2.0, signal_strength=200)
            assert verdict.is_multipath is False
        assert det.warmed_up is False

    def test_warms_up_after_min_samples(self):
        det = MultipathDetector(min_samples=20, refit_every=1)
        for i in range(25):
            det.check(distance_m=3.0, signal_strength=200)
        assert det.warmed_up is True


class TestMultipathDetectorClassification:
    def _warmed_detector(self, rng):
        """Detector pre-fed with a clean bimodal distribution: mostly
        direct-path readings around 3.0m, occasional scattered readings
        around 1.0m with weak signal (typical turbid-water multipath)."""
        det = MultipathDetector(window_size=200, min_samples=30, refit_every=5,
                                 signal_strength_threshold=100)
        for _ in range(150):
            det.check(distance_m=float(rng.normal(3.0, 0.05)), signal_strength=200)
        for _ in range(30):
            det.check(distance_m=float(rng.normal(1.0, 0.05)), signal_strength=40)
        return det

    def test_scattered_low_signal_flagged(self):
        rng = np.random.default_rng(7)
        det = self._warmed_detector(rng)
        verdict = det.check(distance_m=1.0, signal_strength=30)
        assert verdict.is_multipath is True
        assert verdict.posterior_scattered > 0.5

    def test_direct_path_reading_not_flagged(self):
        rng = np.random.default_rng(7)
        det = self._warmed_detector(rng)
        verdict = det.check(distance_m=3.0, signal_strength=200)
        assert verdict.is_multipath is False

    def test_short_range_but_strong_signal_not_flagged(self):
        """A short, strong-signal reading is a legitimate close obstacle,
        not multipath -- the signal-strength gate must prevent a false
        positive here even if the distance alone looks 'scattered'."""
        rng = np.random.default_rng(7)
        det = self._warmed_detector(rng)
        verdict = det.check(distance_m=1.0, signal_strength=250)
        assert verdict.is_multipath is False

    def test_flagged_readings_excluded_from_window(self):
        rng = np.random.default_rng(7)
        det = self._warmed_detector(rng)
        fill_before = len(det._distances)
        det.check(distance_m=1.0, signal_strength=20)  # should be flagged, not added
        assert len(det._distances) == fill_before


class TestMultipathDetectorStatistics:
    def test_statistics_fields(self):
        det = MultipathDetector(min_samples=5, refit_every=1)
        for _ in range(10):
            det.check(2.0, 200)
        stats = det.get_statistics()
        assert stats["checked_count"] == 10
        assert stats["warmed_up"] is True
        assert stats["component_means"] is not None

    def test_reset_clears_state(self):
        det = MultipathDetector(min_samples=5, refit_every=1)
        for _ in range(10):
            det.check(2.0, 200)
        det.reset()
        assert det.warmed_up is False
        stats = det.get_statistics()
        assert stats["window_fill"] == 0
        assert stats["component_means"] is None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
