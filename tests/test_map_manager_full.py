"""Coverage-fill tests for app/map_manager.py (export, info, stats, metadata)."""

import sys
import os
import json

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.map_manager import MapManager, MapMetadata


@pytest.fixture
def mgr(tmp_path):
    return MapManager(str(tmp_path))


class TestMetadataDataclass:
    def test_to_dict_and_from_dict_roundtrip(self):
        from datetime import datetime
        m = MapMetadata(name="x", created=datetime(2024, 1, 1), modified=datetime(2024, 1, 2),
                        point_count=5, bounds_min=[0, 0, 0], bounds_max=[1, 2, 3])
        d = m.to_dict()
        assert d["size"] == {"x": 1.0, "y": 2.0, "z": 3.0}
        m2 = MapMetadata.from_dict(d)
        assert m2.name == "x" and m2.point_count == 5

    def test_from_dict_missing_modified(self):
        m = MapMetadata.from_dict({"name": "y", "created": "2024-01-01T00:00:00"})
        assert m.modified is None


class TestUnsupportedFormat:
    def test_save_rejects_unknown_format(self, mgr):
        assert mgr.save_map("m", np.random.rand(5, 3), format="xyz") is False

    def test_save_rejects_bad_name(self, mgr):
        assert mgr.save_map("../evil", np.random.rand(5, 3)) is False


class TestLoadTrajectory:
    def test_with_and_without_trajectory(self, mgr):
        mgr.save_map("m", np.random.rand(10, 3), trajectory=np.random.rand(4, 3))
        assert mgr.load_trajectory("m").shape == (4, 3)
        mgr.save_map("m2", np.random.rand(10, 3))
        assert mgr.load_trajectory("m2") is None

    def test_invalid_name(self, mgr):
        assert mgr.load_trajectory("../x") is None


class TestUpdateMetadata:
    def test_update_existing(self, mgr):
        mgr.save_map("m", np.random.rand(10, 3), description="old")
        assert mgr.update_metadata("m", description="new") is True
        info = mgr.get_map_info("m")
        assert info["description"] == "new"

    def test_update_missing_map(self, mgr):
        assert mgr.update_metadata("nope", description="x") is False

    def test_update_invalid_name(self, mgr):
        assert mgr.update_metadata("../x", description="y") is False


class TestExportMap:
    @pytest.mark.parametrize("fmt", ["ply", "pcd", "npy", "csv", "xyz"])
    def test_export_formats(self, mgr, tmp_path, fmt):
        mgr.save_map("m", np.random.rand(12, 3))
        out = str(tmp_path / f"out.{fmt}")
        assert mgr.export_map("m", out, format=fmt) is True
        assert os.path.exists(out)

    def test_export_unknown_format(self, mgr, tmp_path):
        mgr.save_map("m", np.random.rand(12, 3))
        assert mgr.export_map("m", str(tmp_path / "o.bin"), format="bin") is False

    def test_export_missing_map(self, mgr, tmp_path):
        assert mgr.export_map("nope", str(tmp_path / "o.ply")) is False


class TestGetMapInfo:
    def test_info_includes_files_and_size(self, mgr):
        mgr.save_map("m", np.random.rand(20, 3))
        info = mgr.get_map_info("m")
        assert "files" in info and info["total_size"] > 0

    def test_info_missing_map(self, mgr):
        assert mgr.get_map_info("nope") is None

    def test_info_invalid_name(self, mgr):
        assert mgr.get_map_info("../x") is None

    def test_info_without_metadata(self, mgr, tmp_path):
        d = tmp_path / "raw"
        d.mkdir()
        (d / "points.npy").write_bytes(b"x")
        info = mgr.get_map_info("raw")
        assert info["name"] == "raw"


class TestGetStatistics:
    def test_statistics(self, mgr):
        mgr.save_map("a", np.random.rand(10, 3))
        mgr.save_map("b", np.random.rand(20, 3))
        stats = mgr.get_statistics()
        assert stats["map_count"] == 2
        assert stats["total_points"] == 30
        assert stats["total_size_bytes"] > 0


class TestListIncomplete:
    def test_incomplete_dir_flagged(self, mgr, tmp_path):
        (tmp_path / "half").mkdir()
        listed = {m["name"]: m for m in mgr.list_maps()}
        assert listed["half"].get("status") == "incomplete"

    def test_non_directory_skipped(self, mgr, tmp_path):
        (tmp_path / "loose.txt").write_text("x")
        assert all(m["name"] != "loose.txt" for m in mgr.list_maps())


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
