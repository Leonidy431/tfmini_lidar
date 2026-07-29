"""
Multipath Detection Module (Physics Audit D2)

Detects scattered-light / multipath returns in turbid underwater conditions
by fitting a lightweight 2-component 1D Gaussian mixture to the recent
distance-reading history and flagging readings that fall in the low-range,
low-signal "scattered" component instead of the "direct path" component.

Decision record: docs/ALGORITHM_DECISION_LOG.md (Decision 5: Multipath
Detection). Scientific basis: Jerlov (1976) Marine Optics (scattering);
mixture-model outlier detection per Carpenter et al. (1999), Thrun et al.
(2005) Probabilistic Robotics Ch. 5 (sensor models with a failure mode).

Implementation note: TECHNICAL_SPECIFICATION.md D2 specifies scikit-learn's
GaussianMixture; this module implements the same 2-component EM fit with
plain NumPy to avoid adding a new dependency for a 1D special case (the
mixture has a closed, well-conditioned E/M step at this scale).

Feature-flagged via Config.multipath.enabled (env ENABLE_MULTIPATH_DETECTION).
Graceful degradation: before `min_samples` readings are collected, every
reading is accepted unfiltered (P10 -- never blocks the pipeline while
warming up).
"""

import math
import logging
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


def _gaussian_pdf(x: np.ndarray, mean: float, std: float) -> np.ndarray:
    std = max(std, 1e-6)
    return (1.0 / (std * math.sqrt(2 * math.pi))) * np.exp(-0.5 * ((x - mean) / std) ** 2)


def fit_two_component_gmm(samples: np.ndarray, n_iter: int = 25,
                           eps: float = 1e-6) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """2-component 1D Gaussian mixture via Expectation-Maximization.

    Returns (means, stds, weights), each a length-2 array sorted by
    ascending mean: index 0 is the "scattered/near" component, index 1 the
    "direct path/far" component (Physics Audit D2 P7 convention).
    """
    x = np.asarray(samples, dtype=float)
    n = len(x)
    if n < 2:
        m = float(x[0]) if n else 0.0
        return np.array([m, m]), np.array([1.0, 1.0]), np.array([0.5, 0.5])

    # Initialize means at the 10th/90th percentiles, NOT a median split.
    # P9-sim finding: median-split initialization converges to a local
    # optimum that splits the majority (direct-path) cluster in two when
    # the scattered cluster is a small minority far from it, driving
    # false positives on direct readings (see tests/test_p9_simulation.py
    # TestD2MultipathRatesSim and docs/P9_SIMULATION_VALIDATION.md).
    means = np.array([np.percentile(x, 10), np.percentile(x, 90)])
    if means[1] - means[0] < eps:
        means = np.array([means[0] - eps, means[1] + eps])
    global_std = max(float(x.std()), eps)
    stds = np.array([global_std / 2.0, global_std / 2.0])
    weights = np.array([0.5, 0.5])

    for _ in range(n_iter):
        resp = np.stack([
            weights[k] * _gaussian_pdf(x, means[k], stds[k]) for k in range(2)
        ], axis=1)
        resp_sum = resp.sum(axis=1, keepdims=True)
        resp_sum[resp_sum < eps] = eps
        resp = resp / resp_sum

        Nk = resp.sum(axis=0)
        Nk_safe = np.maximum(Nk, eps)
        means = (resp * x[:, None]).sum(axis=0) / Nk_safe
        variances = (resp * (x[:, None] - means[None, :]) ** 2).sum(axis=0) / Nk_safe
        stds = np.sqrt(np.maximum(variances, eps))
        weights = Nk / n

    order = np.argsort(means)
    return means[order], stds[order], weights[order]


@dataclass
class MultipathVerdict:
    is_multipath: bool
    posterior_scattered: float          # P(scattered component | reading)
    component_means: Tuple[float, float]  # (scattered_mean, direct_mean)


class MultipathDetector:
    """Rolling-window 2-component mixture detector for scattered-light returns.

    Only readings classified as direct-path are added back to the window,
    so the reference distribution doesn't get contaminated by the outliers
    it's meant to detect.
    """

    def __init__(self, window_size: int = 50, min_samples: int = 20,
                 signal_strength_threshold: int = 100, refit_every: int = 5,
                 posterior_threshold: float = 0.5):
        self.window_size = window_size
        self.min_samples = min_samples
        self.signal_strength_threshold = signal_strength_threshold
        self.refit_every = refit_every
        self.posterior_threshold = posterior_threshold

        self._distances = deque(maxlen=window_size)
        self._means: Optional[np.ndarray] = None
        self._stds: Optional[np.ndarray] = None
        self._weights: Optional[np.ndarray] = None
        self._since_refit = 0

        self.checked_count = 0
        self.flagged_count = 0
        self.warmed_up = False

    def _maybe_refit(self):
        if len(self._distances) < self.min_samples:
            return
        self._since_refit += 1
        if self._means is None or self._since_refit >= self.refit_every:
            means, stds, weights = fit_two_component_gmm(np.array(self._distances))
            self._means, self._stds, self._weights = means, stds, weights
            self._since_refit = 0
            self.warmed_up = True

    def check(self, distance_m: float, signal_strength: int) -> MultipathVerdict:
        """Classify one reading. During warm-up (window below min_samples)
        every reading is accepted and added to the window unconditionally."""
        self.checked_count += 1
        self._maybe_refit()

        if self._means is None:
            self._distances.append(distance_m)
            return MultipathVerdict(False, 0.0, (0.0, 0.0))

        x = np.array([distance_m])
        p_scattered = float(_gaussian_pdf(x, self._means[0], self._stds[0])[0] * self._weights[0])
        p_direct = float(_gaussian_pdf(x, self._means[1], self._stds[1])[0] * self._weights[1])
        total = p_scattered + p_direct
        posterior_scattered = p_scattered / total if total > 1e-12 else 0.0

        # Bimodality guard (Ashman's D): if the two fitted components are
        # not clearly separated (D < 2), the window is effectively
        # unimodal -- there is no scattered-light population to reject, and
        # flagging against a degenerate split of the single direct-path
        # cluster produces false positives (P9-sim finding, see
        # docs/P9_SIMULATION_VALIDATION.md).
        ashman_d = (math.sqrt(2.0) * (self._means[1] - self._means[0])
                    / math.sqrt(self._stds[0] ** 2 + self._stds[1] ** 2 + 1e-12))
        bimodal = ashman_d > 2.0

        is_multipath = (
            bimodal
            and posterior_scattered > self.posterior_threshold
            and distance_m < self._means[1]
            and signal_strength < self.signal_strength_threshold
        )

        if is_multipath:
            self.flagged_count += 1
        else:
            self._distances.append(distance_m)

        return MultipathVerdict(is_multipath, posterior_scattered,
                                 (float(self._means[0]), float(self._means[1])))

    def reset(self):
        self._distances.clear()
        self._means = self._stds = self._weights = None
        self._since_refit = 0
        self.warmed_up = False

    def get_statistics(self) -> dict:
        return {
            "checked_count": self.checked_count,
            "flagged_count": self.flagged_count,
            "flagged_rate": (self.flagged_count / self.checked_count) if self.checked_count else 0.0,
            "warmed_up": self.warmed_up,
            "window_fill": len(self._distances),
            "component_means": (
                (float(self._means[0]), float(self._means[1])) if self._means is not None else None
            ),
        }
