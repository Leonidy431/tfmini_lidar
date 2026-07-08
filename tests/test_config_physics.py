"""
Tests for the physics-driven configuration defaults (Physics Audit C1/C2):
- LiDARConfig.max_range default and LIDAR_MAX_RANGE_M override
- LiDARConfig.medium_refractive_index default and LIDAR_MEDIUM_INDEX override

Config values are read from the environment at class-definition time (module
import), so overrides are verified via a fresh subprocess rather than
mutating os.environ in-process (which would not retroactively change an
already-imported dataclass field default).
"""

import subprocess
import sys
import os

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run_config_snippet(code: str, env_overrides: dict = None) -> str:
    env = os.environ.copy()
    env['DATA_DIR'] = '/tmp/lidar_config_test'
    if env_overrides:
        env.update(env_overrides)
    result = subprocess.run(
        [sys.executable, '-c', code],
        cwd=REPO_ROOT, env=env, capture_output=True, text=True, timeout=30
    )
    assert result.returncode == 0, f"stderr: {result.stderr}"
    return result.stdout.strip()


class TestDefaultValues:
    def test_max_range_default_is_water_realistic(self):
        out = run_config_snippet(
            "from app.config import Config; print(Config.lidar.max_range)"
        )
        # Physics Audit C2: must NOT be the TFmini-S in-air spec (12.0);
        # underwater at 850nm no real return exists past a few meters.
        assert float(out) != 12.0
        assert float(out) == 4.0

    def test_medium_refractive_index_default_is_water(self):
        out = run_config_snippet(
            "from app.config import Config; print(Config.lidar.medium_refractive_index)"
        )
        # Physics Audit C1: default must reflect the project's stated
        # underwater ROV purpose (n~=1.333), not the air no-op (1.0).
        assert float(out) == 1.333


class TestEnvironmentOverrides:
    def test_max_range_overridable(self):
        out = run_config_snippet(
            "from app.config import Config; print(Config.lidar.max_range)",
            env_overrides={'LIDAR_MAX_RANGE_M': '2.0'}
        )
        assert float(out) == 2.0

    def test_medium_index_overridable_to_air(self):
        """Bench/air testing must be able to disable the water correction."""
        out = run_config_snippet(
            "from app.config import Config; print(Config.lidar.medium_refractive_index)",
            env_overrides={'LIDAR_MEDIUM_INDEX': '1.0'}
        )
        assert float(out) == 1.0

    def test_to_dict_includes_medium_index(self):
        out = run_config_snippet(
            "from app.config import Config; import json; "
            "print(json.dumps(Config.to_dict()['lidar']))"
        )
        import json
        lidar_dict = json.loads(out)
        assert 'medium_refractive_index' in lidar_dict
        assert 'max_range' in lidar_dict


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
