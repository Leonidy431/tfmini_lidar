"""
Data Quality Validation Module for BlueOS LiDAR SLAM Extension

Implements Rule 4 (Data Quality Validation) from the project rules:
- IQR outlier detection on raw LiDAR readings
- Z-score filtering for anomalous points
- Data Quality Score tracking for calibration drift

This runs BEFORE SLAM processing to reject spurious measurements caused by
turbidity, multipath reflections, or sensor glitches in the underwater
environment.
"""

import logging
import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, Deque

logger = logging.getLogger(__name__)


@dataclass
class QualityResult:
    """Result of a data quality check for a single reading."""
    accepted: bool
    reason: str
    z_score: float = 0.0
    quality_score: float = 1.0


class DataQualityValidator:
    """
    Validates raw LiDAR readings using statistical outlier detection.

    Two complementary filters are applied over a sliding window of recent
    readings:

    1. IQR (Interquartile Range) filter: rejects points outside
       [Q1 - k*IQR, Q3 + k*IQR]. Robust to non-Gaussian noise.
    2. Z-score filter: rejects points more than `z_threshold` standard
       deviations from the rolling mean. Catches sudden spikes.

    A rolling Data Quality Score (fraction of accepted readings) is tracked
    as a proxy for calibration drift / environmental degradation.
    """

    def __init__(self,
                 window_size: int = 50,
                 iqr_multiplier: float = 1.5,
                 z_threshold: float = 3.0,
                 min_samples: int = 10,
                 signal_threshold: int = 100,
                 max_range: float = 12.0,
                 min_range: float = 0.1,
                 max_rate_m_per_s: float = 15.0,
                 temp_coefficient: float = 0.0):
        self.window_size = window_size
        self.iqr_multiplier = iqr_multiplier
        self.z_threshold = z_threshold
        self.min_samples = min_samples
        self.signal_threshold = signal_threshold
        self.max_range = max_range
        self.min_range = min_range
        # Rate-of-change gate: reject physically impossible jumps between
        # consecutive readings (Data Quality #2). Default tuned for a slow ROV.
        self.max_rate_m_per_s = max_rate_m_per_s
        # Optional temperature-drift compensation (Data Quality #3). Default 0
        # (no-op) since the TFmini-S applies its own internal compensation;
        # set a coefficient only after empirical calibration.
        self.temp_coefficient = temp_coefficient
        self._baseline_temp: Optional[float] = None

        # Sliding window of accepted distances (meters)
        self._window: Deque[float] = deque(maxlen=window_size)

        # Last accepted reading for rate-of-change checks
        self._last_distance: Optional[float] = None
        self._last_time: Optional[float] = None

        # Quality score tracking (rolling over recent decisions)
        self._decisions: Deque[bool] = deque(maxlen=200)

        # Statistics
        self.total_checked = 0
        self.total_rejected = 0
        self.rejected_range = 0
        self.rejected_signal = 0
        self.rejected_iqr = 0
        self.rejected_zscore = 0
        self.rejected_rate = 0

    def compensate_temperature(self, distance: float, temperature: float) -> float:
        """Apply optional temperature-drift correction.

        corrected = raw * (1 + coeff * (T - T_baseline)). With the default
        coefficient of 0 this returns the distance unchanged.
        """
        if self.temp_coefficient == 0.0:
            return distance
        if self._baseline_temp is None:
            self._baseline_temp = temperature
        return distance * (1 + self.temp_coefficient * (temperature - self._baseline_temp))

    def validate(self, distance: float, signal_strength: int,
                 timestamp: float = None) -> QualityResult:
        """
        Validate a single reading.

        Args:
            distance: measured distance in meters
            signal_strength: sensor signal strength (0-65535)
            timestamp: monotonic seconds; used for rate-of-change gating.
                When None, the rate check is skipped.

        Returns:
            QualityResult indicating whether the reading is accepted.
        """
        self.total_checked += 1

        # Hard physical bounds first (cheap rejects)
        if distance < self.min_range or distance > self.max_range:
            return self._reject('out_of_range', 'range')

        if signal_strength < self.signal_threshold:
            return self._reject('low_signal', 'signal')

        # Rate-of-change gate (needs a previous timed reading)
        if (timestamp is not None and self._last_distance is not None
                and self._last_time is not None):
            dt = timestamp - self._last_time
            if dt > 0:
                rate = abs(distance - self._last_distance) / dt
                if rate > self.max_rate_m_per_s:
                    return self._reject('rate_exceeded', 'rate')

        # Need enough history for statistical tests
        if len(self._window) < self.min_samples:
            self._accept(distance, timestamp)
            return QualityResult(
                accepted=True,
                reason='insufficient_history',
                quality_score=self.quality_score
            )

        # Z-score filter
        mean = sum(self._window) / len(self._window)
        variance = sum((x - mean) ** 2 for x in self._window) / len(self._window)
        std = math.sqrt(variance)

        z_score = 0.0
        if std > 1e-9:
            z_score = abs(distance - mean) / std
            if z_score > self.z_threshold:
                return self._reject('zscore_outlier', 'zscore', z_score)

        # IQR filter
        q1, q3 = self._quartiles(self._window)
        iqr = q3 - q1
        if iqr > 1e-9:
            lower = q1 - self.iqr_multiplier * iqr
            upper = q3 + self.iqr_multiplier * iqr
            if distance < lower or distance > upper:
                return self._reject('iqr_outlier', 'iqr', z_score)

        self._accept(distance, timestamp)
        return QualityResult(
            accepted=True,
            reason='ok',
            z_score=z_score,
            quality_score=self.quality_score
        )

    def _accept(self, distance: float, timestamp: float = None):
        self._window.append(distance)
        self._decisions.append(True)
        self._last_distance = distance
        if timestamp is not None:
            self._last_time = timestamp

    def _reject(self, reason: str, category: str, z_score: float = 0.0) -> QualityResult:
        self.total_rejected += 1
        self._decisions.append(False)

        if category == 'range':
            self.rejected_range += 1
        elif category == 'signal':
            self.rejected_signal += 1
        elif category == 'iqr':
            self.rejected_iqr += 1
        elif category == 'zscore':
            self.rejected_zscore += 1
        elif category == 'rate':
            self.rejected_rate += 1

        return QualityResult(
            accepted=False,
            reason=reason,
            z_score=z_score,
            quality_score=self.quality_score
        )

    @staticmethod
    def _quartiles(values) -> tuple:
        """Compute Q1 and Q3 using linear interpolation (type-7)."""
        data = sorted(values)
        n = len(data)

        def percentile(p: float) -> float:
            if n == 1:
                return data[0]
            rank = p * (n - 1)
            low = int(math.floor(rank))
            high = int(math.ceil(rank))
            if low == high:
                return data[low]
            frac = rank - low
            return data[low] * (1 - frac) + data[high] * frac

        return percentile(0.25), percentile(0.75)

    @property
    def quality_score(self) -> float:
        """
        Data Quality Score: fraction of recently accepted readings.

        A sustained drop indicates calibration drift, sensor fouling, or a
        degraded (e.g. turbid) environment.
        """
        if not self._decisions:
            return 1.0
        accepted = sum(1 for d in self._decisions if d)
        return round(accepted / len(self._decisions), 3)

    def reset(self):
        """Clear all state."""
        self._window.clear()
        self._decisions.clear()
        self._last_distance = None
        self._last_time = None
        self._baseline_temp = None
        self.total_checked = 0
        self.total_rejected = 0
        self.rejected_range = 0
        self.rejected_signal = 0
        self.rejected_iqr = 0
        self.rejected_zscore = 0
        self.rejected_rate = 0

    def get_statistics(self) -> dict:
        """Golden-signal-style metrics for data quality."""
        rejection_rate = self.total_rejected / max(1, self.total_checked)
        return {
            'quality_score': self.quality_score,
            'total_checked': self.total_checked,
            'total_rejected': self.total_rejected,
            'rejection_rate': round(rejection_rate, 4),
            'window_fill': len(self._window),
            'rejected_by': {
                'range': self.rejected_range,
                'signal': self.rejected_signal,
                'iqr': self.rejected_iqr,
                'zscore': self.rejected_zscore,
                'rate': self.rejected_rate
            }
        }
