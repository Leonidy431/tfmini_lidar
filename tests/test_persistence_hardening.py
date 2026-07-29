"""
Regression tests for the Blind Spot Audit R2 persistence + security fixes
(domains 17 & 18). These lock in the corruption-resilience and rate-limit
behavior so a future change can't silently regress them.
"""

import sys
import os
import json

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.map_manager import MapManager
from app.security import RateLimiter, RATE_LIMIT_STORE, init_default_token, API_TOKENS


class TestMapSaveAtomicity:
    def test_save_leaves_no_staging_dir(self, tmp_path):
        mgr = MapManager(str(tmp_path))
        assert mgr.save_map("m1", np.random.rand(50, 3))
        assert not any(p.name.endswith('.staging') for p in tmp_path.iterdir())

    def test_resave_preserves_loadability(self, tmp_path):
        """Re-saving an existing name must end with a valid, loadable map --
        the atomic swap must not leave it half-written (domain 17 #1)."""
        mgr = MapManager(str(tmp_path))
        mgr.save_map("m1", np.random.rand(50, 3))
        mgr.save_map("m1", np.random.rand(80, 3))
        result = mgr.load_map("m1")
        assert result is not None
        points, meta = result
        assert len(points) == 80

    def test_all_formats_round_trip(self, tmp_path):
        mgr = MapManager(str(tmp_path))
        for fmt in ("ply", "pcd", "npy", "h5"):
            pts = np.random.rand(40, 3)
            assert mgr.save_map(f"map_{fmt}", pts, format=fmt), fmt
            loaded = mgr.load_map(f"map_{fmt}")
            assert loaded is not None, fmt
            assert len(loaded[0]) == 40, fmt


class TestMapLoadValidation:
    def test_corrupt_metadata_does_not_block_load(self, tmp_path):
        """An intact points file must still load when metadata.json is
        garbage (domain 17 #5)."""
        mgr = MapManager(str(tmp_path))
        mgr.save_map("m1", np.random.rand(60, 3), format="npy")
        meta_path = tmp_path / "m1" / "metadata.json"
        meta_path.write_text("{ this is not valid json ")
        result = mgr.load_map("m1")
        assert result is not None
        assert len(result[0]) == 60

    def test_truncated_points_rejected(self, tmp_path):
        """If the points file has far fewer points than metadata claims, the
        map must be refused rather than fed to localization (domain 17 #4)."""
        mgr = MapManager(str(tmp_path))
        mgr.save_map("m1", np.random.rand(100, 3), format="npy")
        # Overwrite points with a tiny fragment, keep metadata's count=100
        np.save(tmp_path / "m1" / "points.npy", np.random.rand(3, 3))
        assert mgr.load_map("m1") is None

    def test_wrong_shape_rejected(self, tmp_path):
        mgr = MapManager(str(tmp_path))
        mgr.save_map("m1", np.random.rand(50, 3), format="npy")
        np.save(tmp_path / "m1" / "points.npy", np.random.rand(50, 2))  # not Nx3
        assert mgr.load_map("m1") is None


class TestMapListIsolation:
    def test_one_corrupt_metadata_does_not_blank_listing(self, tmp_path):
        mgr = MapManager(str(tmp_path))
        mgr.save_map("good1", np.random.rand(10, 3))
        mgr.save_map("bad", np.random.rand(10, 3))
        mgr.save_map("good2", np.random.rand(10, 3))
        (tmp_path / "bad" / "metadata.json").write_text("{corrupt")
        listed = {m["name"] for m in mgr.list_maps()}
        assert "good1" in listed
        assert "good2" in listed  # healthy maps survive the bad one

    def test_staging_dirs_hidden_from_listing(self, tmp_path):
        mgr = MapManager(str(tmp_path))
        mgr.save_map("m1", np.random.rand(10, 3))
        (tmp_path / "m1.staging").mkdir()
        listed = {m["name"] for m in mgr.list_maps()}
        assert "m1.staging" not in listed
        assert "m1" in listed


class TestRateLimiterEviction:
    def setup_method(self):
        RATE_LIMIT_STORE.clear()

    def test_expired_keys_evicted_past_cap(self):
        limiter = RateLimiter(requests_per_minute=1000)
        limiter.MAX_TRACKED_CLIENTS = 5
        for i in range(20):
            RATE_LIMIT_STORE[f"stale_{i}"] = [0.0]  # ancient timestamp
        limiter.is_allowed("fresh_client")
        # Ancient keys should have been swept once the store passed the cap
        assert len(RATE_LIMIT_STORE) < 20

    def test_within_limit_allowed_over_limit_blocked(self):
        limiter = RateLimiter(requests_per_minute=3)
        cid = "1.2.3.4"
        assert limiter.is_allowed(cid)
        assert limiter.is_allowed(cid)
        assert limiter.is_allowed(cid)
        assert not limiter.is_allowed(cid)  # 4th within the window


class TestWeakTokenRejection:
    def test_short_env_token_rejected(self, monkeypatch):
        monkeypatch.setenv("LIDAR_API_TOKEN", "short")
        with pytest.raises(ValueError):
            init_default_token()

    def test_strong_env_token_accepted(self, monkeypatch):
        monkeypatch.setenv("LIDAR_API_TOKEN", "a" * 32)
        before = len(API_TOKENS)
        result = init_default_token()
        assert result == "[set via LIDAR_API_TOKEN]"
        assert len(API_TOKENS) == before + 1


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
