"""
Environmental Correction Module (Physics Audit D3 + D4)

Refines the constant medium_refractive_index correction already applied in
app/lidar_driver.py (Physics Audit C1, n=1.333 default) with two additional,
optional corrections when telemetry is available:

  D3 - Depth-dependent refractive index n(depth): seawater's refractive
       index rises slightly with pressure/depth (Austin & Halikas 1976).
       A degree-2 polynomial n(z) = a + b*z + c*z^2 is fit empirically
       per-deployment (see TECHNICAL_SPECIFICATION.md D3 P9 calibration
       procedure) and defaults to a flat n=1.333 (b=c=0) until calibrated.

  D4 - Temperature-compensated distance: TFmini-S datasheet specifies a
       ToF drift of roughly +/-0.05%/degC. A linear model referenced to
       20 degC (the sensor's typical calibration temperature) corrects for
       this when the driver's onboard thermometer reading is available.

Both corrections are optional and independently toggleable -- passing
depth_m=None or temperature_c=None to EnvironmentalCorrector.correct_raw_distance_cm()
skips that stage entirely (P10 graceful degradation: missing telemetry never
blocks distance reporting, it just falls back to the existing constant-n
behavior).
"""

import logging
from dataclasses import dataclass, field
from typing import Optional, Tuple

logger = logging.getLogger(__name__)


@dataclass
class DepthCorrectedRefractive:
    """n(depth_m) = a + b*depth_m + c*depth_m^2, clamped to physical bounds.

    Default coefficients (a=1.333, b=c=0) reproduce the existing constant
    correction exactly -- this is a strict refinement, not a behavior
    change, until D3-P9 lab/field calibration provides fitted b, c.
    """
    a: float = 1.333
    b: float = 0.0
    c: float = 0.0
    n_min: float = 1.0
    n_max: float = 1.40

    def get_refractive_index(self, depth_m: float) -> float:
        n = self.a + self.b * depth_m + self.c * (depth_m ** 2)
        return max(self.n_min, min(self.n_max, n))

    def correct_distance(self, distance_cm: float, depth_m: float) -> float:
        n = self.get_refractive_index(depth_m)
        return (distance_cm / 100.0) / n


@dataclass
class TemperatureCorrection:
    """Linear ToF drift compensation referenced to ref_temp_c.

    factor(temp) = 1 + slope_per_degree * (temp - ref_temp_c)
    Default slope (0.0005/degC) is a conservative placeholder pending
    D4-P9 oven calibration; factor(ref_temp_c) == 1.0 (no-op) by construction.
    """
    ref_temp_c: float = 20.0
    slope_per_degree: float = 0.0005

    def get_correction_factor(self, temperature_c: float) -> float:
        return 1.0 + self.slope_per_degree * (temperature_c - self.ref_temp_c)

    def correct_distance(self, distance_m: float, temperature_c: float) -> float:
        return distance_m * self.get_correction_factor(temperature_c)


class EnvironmentalCorrector:
    """Combined depth + temperature correction pipeline.

    Primary entry point is correct_raw_distance_cm(), which takes the
    *uncorrected* TFmini-S distance_cm field directly (bypassing the
    driver's constant-n division) so depth and temperature corrections
    compose cleanly without floating-point round-trip error.
    """

    def __init__(self, depth_model: Optional[DepthCorrectedRefractive] = None,
                 temp_model: Optional[TemperatureCorrection] = None):
        self.depth_model = depth_model or DepthCorrectedRefractive()
        self.temp_model = temp_model or TemperatureCorrection()

    def correct_raw_distance_cm(self, distance_cm: float,
                                 depth_m: Optional[float] = None,
                                 temperature_c: Optional[float] = None) -> float:
        """distance_cm: raw TFmini-S field, pre any refractive correction.

        depth_m=None -> uses depth_model.a as a flat refractive index
        (equivalent to the existing constant-n behavior).
        temperature_c=None -> skips temperature compensation entirely.
        """
        if depth_m is not None:
            distance_m = self.depth_model.correct_distance(distance_cm, depth_m)
        else:
            distance_m = (distance_cm / 100.0) / self.depth_model.a

        if temperature_c is not None:
            distance_m = self.temp_model.correct_distance(distance_m, temperature_c)

        return distance_m

    def rescale_corrected_distance(self, distance_m: float, applied_n: float,
                                    depth_m: Optional[float] = None,
                                    temperature_c: Optional[float] = None) -> float:
        """Fallback path when only an already-n-corrected distance is
        available (e.g. reading.distance from the driver, which already
        divided by the constant medium_refractive_index). Divides out the
        constant-n assumption and reapplies depth/temperature corrections.
        """
        distance_cm = distance_m * applied_n * 100.0
        return self.correct_raw_distance_cm(distance_cm, depth_m, temperature_c)

    def calibrate_depth_model(self, depth_samples_m, refractive_indices) -> DepthCorrectedRefractive:
        """D3-P9: least-squares fit of n(z) = a + b*z + c*z^2 from paired
        (depth_m, measured_n) calibration data. Returns a new model; does
        not mutate self.depth_model (caller decides whether to adopt it)."""
        import numpy as np
        z = np.asarray(depth_samples_m, dtype=float)
        n = np.asarray(refractive_indices, dtype=float)
        if len(z) < 3:
            raise ValueError("calibrate_depth_model requires at least 3 samples")
        coeffs = np.polyfit(z, n, deg=2)  # c, b, a order (numpy convention)
        c, b, a = coeffs
        return DepthCorrectedRefractive(a=float(a), b=float(b), c=float(c))

    def get_statistics(self) -> dict:
        return {
            "depth_model": {"a": self.depth_model.a, "b": self.depth_model.b,
                             "c": self.depth_model.c},
            "temp_model": {"ref_temp_c": self.temp_model.ref_temp_c,
                            "slope_per_degree": self.temp_model.slope_per_degree},
        }
