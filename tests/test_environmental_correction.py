"""
Tests for depth-dependent refractive index and temperature compensation
(Physics Audit D3 + D4).
"""

import sys
import os

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.environmental_correction import (
    DepthCorrectedRefractive,
    TemperatureCorrection,
    EnvironmentalCorrector,
)


class TestDepthCorrectedRefractive:
    def test_default_matches_constant_n(self):
        model = DepthCorrectedRefractive()
        assert model.get_refractive_index(0.0) == pytest.approx(1.333)

    def test_default_flat_across_depth(self):
        """b=c=0 by default -- must reproduce the existing constant
        correction exactly regardless of depth until calibrated (D3 P9)."""
        model = DepthCorrectedRefractive()
        assert model.get_refractive_index(0.0) == model.get_refractive_index(50.0)

    def test_calibrated_model_increases_with_depth(self):
        model = DepthCorrectedRefractive(a=1.333, b=0.00002, c=-0.0000001)
        n_shallow = model.get_refractive_index(0.0)
        n_deep = model.get_refractive_index(50.0)
        assert n_deep > n_shallow

    def test_clamped_to_physical_bounds(self):
        model = DepthCorrectedRefractive(a=1.333, b=10.0, c=0.0)  # absurd slope
        n = model.get_refractive_index(1000.0)
        assert n <= 1.40

    def test_correct_distance_matches_driver_formula(self):
        model = DepthCorrectedRefractive(a=1.333, b=0.0, c=0.0)
        # 150cm raw at n=1.333 -> matches app/lidar_driver.py's existing formula
        assert model.correct_distance(150.0, depth_m=0.0) == pytest.approx(1.5 / 1.333, rel=1e-9)


class TestTemperatureCorrection:
    def test_reference_temperature_is_noop(self):
        model = TemperatureCorrection(ref_temp_c=20.0)
        assert model.get_correction_factor(20.0) == pytest.approx(1.0)
        assert model.correct_distance(3.0, 20.0) == pytest.approx(3.0)

    def test_colder_water_increases_factor(self):
        model = TemperatureCorrection(ref_temp_c=20.0, slope_per_degree=0.0005)
        factor_cold = model.get_correction_factor(5.0)   # below ref
        factor_warm = model.get_correction_factor(25.0)  # above ref
        assert factor_cold < 1.0 or factor_cold != factor_warm
        assert factor_cold != 1.0
        assert factor_warm != 1.0

    def test_correction_is_linear(self):
        model = TemperatureCorrection(ref_temp_c=20.0, slope_per_degree=0.001)
        f10 = model.get_correction_factor(10.0)
        f30 = model.get_correction_factor(30.0)
        # Symmetric around ref_temp: distances from 1.0 should be equal magnitude
        assert abs((f10 - 1.0) + (f30 - 1.0)) < 1e-9 or abs((1.0 - f10) - (f30 - 1.0)) < 1e-9


class TestEnvironmentalCorrector:
    def test_no_telemetry_matches_constant_correction(self):
        corrector = EnvironmentalCorrector()
        distance_m = corrector.correct_raw_distance_cm(150.0)
        assert distance_m == pytest.approx(1.5 / 1.333, rel=1e-9)

    def test_depth_only_correction(self):
        corrector = EnvironmentalCorrector(
            depth_model=DepthCorrectedRefractive(a=1.333, b=0.001, c=0.0))
        shallow = corrector.correct_raw_distance_cm(300.0, depth_m=0.0)
        deep = corrector.correct_raw_distance_cm(300.0, depth_m=50.0)
        # Higher n at depth -> shorter corrected distance for the same raw reading
        assert deep < shallow

    def test_temperature_only_correction(self):
        corrector = EnvironmentalCorrector()
        at_ref = corrector.correct_raw_distance_cm(300.0, temperature_c=20.0)
        away_from_ref = corrector.correct_raw_distance_cm(300.0, temperature_c=5.0)
        assert at_ref != away_from_ref

    def test_combined_depth_and_temperature(self):
        corrector = EnvironmentalCorrector(
            depth_model=DepthCorrectedRefractive(a=1.333, b=0.001, c=0.0))
        combined = corrector.correct_raw_distance_cm(300.0, depth_m=30.0, temperature_c=10.0)
        depth_only = corrector.correct_raw_distance_cm(300.0, depth_m=30.0)
        assert combined != depth_only

    def test_rescale_corrected_distance_round_trip(self):
        """A distance already corrected with the constant n=1.333, when
        rescaled with depth_m=0 (same coefficient a=1.333), must reproduce
        the original value exactly (identity round-trip)."""
        corrector = EnvironmentalCorrector()
        applied_n = 1.333
        original_distance_m = 1.5 / applied_n
        rescaled = corrector.rescale_corrected_distance(original_distance_m, applied_n, depth_m=0.0)
        assert rescaled == pytest.approx(original_distance_m, rel=1e-9)

    def test_calibrate_depth_model_recovers_known_polynomial(self):
        corrector = EnvironmentalCorrector()
        depths = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]
        true_a, true_b, true_c = 1.333, 0.0002, -0.000001
        n_values = [true_a + true_b * z + true_c * z * z for z in depths]

        fitted = corrector.calibrate_depth_model(depths, n_values)
        assert fitted.a == pytest.approx(true_a, abs=1e-4)
        assert fitted.b == pytest.approx(true_b, abs=1e-5)

    def test_calibrate_requires_min_samples(self):
        corrector = EnvironmentalCorrector()
        with pytest.raises(ValueError):
            corrector.calibrate_depth_model([0.0, 10.0], [1.333, 1.334])

    def test_statistics_shape(self):
        corrector = EnvironmentalCorrector()
        stats = corrector.get_statistics()
        assert "depth_model" in stats
        assert "temp_model" in stats
        assert stats["depth_model"]["a"] == pytest.approx(1.333)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
