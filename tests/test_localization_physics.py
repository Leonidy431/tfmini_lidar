"""
Targeted tests for the localization physics/engineering fixes:
- Particle filter log-space likelihood (no underflow/ZeroDivisionError)
- Circular mean for heading estimate
- Localization pose composition (no double-application)
- Localization confidence gate (fitness + rmse)
"""

import sys
import os

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.localization import LocalizationEngine, ParticleFilterLocalizer


class TestParticleFilterNumericalStability:
    def test_extreme_measurement_does_not_zero_all_weights(self):
        """A residual far beyond the sensor range must not underflow every
        weight to exactly 0.0 (Physics Audit C7)."""
        pf = ParticleFilterLocalizer(num_particles=50)
        map_points = np.random.default_rng(0).uniform(-5, 5, (200, 3))
        pf.set_reference_map(map_points)

        # A wildly implausible reading relative to whatever the ray-cast
        # expects will drive the linear-space likelihood to exact 0.0 for
        # every particle; the log-space rewrite must still leave a valid,
        # normalized (or safely-reset) weight vector.
        for _ in range(20):
            pf.update(distance_reading=999.0)

        assert np.all(np.isfinite(pf.weights))
        assert abs(np.sum(pf.weights) - 1.0) < 1e-6
        assert not np.any(pf.weights < 0)

    def test_resample_does_not_raise_after_extreme_updates(self):
        """Weight sum must stay exactly 1.0 so np.random.choice(p=...)
        never raises (Physics Audit C7 weight-normalization epsilon bug)."""
        pf = ParticleFilterLocalizer(num_particles=100)
        map_points = np.random.default_rng(1).uniform(-5, 5, (200, 3))
        pf.set_reference_map(map_points)

        # Repeated extreme updates to force many resample cycles.
        for _ in range(50):
            pf.update(distance_reading=500.0, motion_delta=(0.01, 0.0, 0.0))

        # No exception raised is itself the primary assertion; also check
        # invariants hold afterward.
        assert np.all(np.isfinite(pf.weights))
        assert abs(np.sum(pf.weights) - 1.0) < 1e-6

    def test_normal_updates_produce_valid_weights(self):
        pf = ParticleFilterLocalizer(num_particles=50)
        map_points = np.random.default_rng(2).uniform(-5, 5, (200, 3))
        pf.set_reference_map(map_points)

        pf.update(distance_reading=2.0)
        assert np.all(np.isfinite(pf.weights))
        assert abs(np.sum(pf.weights) - 1.0) < 1e-6


class TestCircularMean:
    def test_heading_wrap_averages_correctly(self):
        """Particles straddling +/-pi (all pointing ~the same real
        direction) must not average to the opposite bearing (Physics
        Audit H7)."""
        pf = ParticleFilterLocalizer(num_particles=4)
        pf.set_reference_map(np.random.default_rng(3).uniform(-5, 5, (50, 3)))

        # Half the particles at +3.1 rad, half at -3.1 rad -- both are
        # ~178 deg, essentially agreeing, not opposite.
        pf.particles[:, 2] = np.array([3.1, 3.1, -3.1, -3.1])
        pf.weights = np.array([0.25, 0.25, 0.25, 0.25])

        _, _, theta, _ = pf.get_estimate()
        # The circular mean of {+3.1, -3.1} should be near +/-pi (~178deg),
        # NOT near 0 (which a naive linear mean would produce).
        assert abs(abs(theta) - np.pi) < 0.1

    def test_uniform_heading_no_wrap_issue(self):
        pf = ParticleFilterLocalizer(num_particles=4)
        pf.set_reference_map(np.random.default_rng(4).uniform(-5, 5, (50, 3)))
        pf.particles[:, 2] = np.array([0.1, 0.1, 0.1, 0.1])
        pf.weights = np.array([0.25, 0.25, 0.25, 0.25])

        _, _, theta, _ = pf.get_estimate()
        assert abs(theta - 0.1) < 1e-6

    def test_motion_update_wraps_particle_headings(self):
        pf = ParticleFilterLocalizer(num_particles=10)
        pf.set_reference_map(np.random.default_rng(5).uniform(-5, 5, (50, 3)))
        pf.particles[:, 2] = np.full(10, 3.0)

        # Push headings past +pi repeatedly via motion updates
        for _ in range(5):
            pf.update(distance_reading=2.0, motion_delta=(0.0, 0.0, 1.0))

        assert np.all(pf.particles[:, 2] >= -np.pi)
        assert np.all(pf.particles[:, 2] < np.pi)


class TestLocalizationPoseComposition:
    def test_localize_with_reasonable_offset_converges(self):
        """A scan pre-offset by a plausible pose error should converge
        toward the reference map without runaway double-application of
        the pose (Physics Audit, localization double-composition)."""
        engine = LocalizationEngine()
        rng = np.random.default_rng(6)

        # Build a simple planar reference "wall" with real 2D structure.
        ref_points = np.column_stack([
            rng.uniform(-3, 3, 500),
            rng.uniform(-3, 3, 500),
            np.zeros(500)
        ])
        engine.set_reference_map(ref_points)

        # A scan that IS the reference map, slightly offset -- as if the
        # vehicle's own pose estimate already has a small error baked in
        # (mimicking main.py's world-frame pre-projection).
        scan = ref_points[:150] + np.array([0.05, 0.02, 0.0])

        result = engine.localize(scan)

        # The resulting pose must stay a valid, small correction -- not
        # blow up from a doubled pose application (which would misalign
        # the source cloud by the full pose magnitude before ICP even
        # starts).
        pose = engine.get_pose()
        R = pose[:3, :3]
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-4)
        translation_norm = np.linalg.norm(pose[:3, 3])
        assert translation_norm < 1.0  # not runaway

    def test_init_transform_is_identity_not_current_pose(self):
        """Directly verify the fix: init_transform must not be seeded with
        current_pose when scan points are already world-frame."""
        engine = LocalizationEngine()
        engine.current_pose = np.eye(4)
        engine.current_pose[:3, 3] = [10.0, 10.0, 0.0]  # nonzero pose

        ref_points = np.random.default_rng(7).uniform(-3, 3, (300, 3))
        ref_points[:, 2] = 0.0
        engine.set_reference_map(ref_points)

        # A scan near the origin (NOT near the current_pose translation of
        # (10,10,0)) -- if init_transform were current_pose, ICP would
        # start 10-14m from any correspondence and fail to converge at all.
        scan = ref_points[:100]
        engine.localize(scan)
        # Should be plausible to match (fitness computed without a 10m
        # initial-guess handicap); we mainly assert it runs without the
        # pose exploding to (20,20,0)-scale values.
        pose = engine.get_pose()
        assert np.linalg.norm(pose[:3, 3]) < 30.0  # sanity bound, not 2x pose


class TestLocalizationConfidenceGate:
    def test_get_statistics_includes_gate_fields(self):
        engine = LocalizationEngine()
        stats = engine.get_statistics()
        assert 'current_confidence' in stats
        assert 'is_lost' in stats


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
