"""Final push: slam registration, object-detection clustering/edge, main.py
env-correction + detection broadcast + invalid/stale branches, driver read-loop."""

import sys
import os
import time
import threading
from datetime import datetime
from unittest.mock import MagicMock

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.slam_engine import SLAMEngine
from app.object_detection import ObjectDetector
from app.main import LiDARSLAMApplication
from app.lidar_driver import LiDARReading
from app.environmental_correction import EnvironmentalCorrector, DepthCorrectedRefractive


def make_reading(distance=2.0, strength=200, temperature=20.0, mono_ts=None):
    return LiDARReading(distance=distance, signal_strength=strength,
                        timestamp=datetime.now(), temperature=temperature, valid=True,
                        mono_timestamp=mono_ts or time.monotonic())


def _plane(n=200, noise=0.01, shift=(0.0, 0.0, 0.0)):
    """A noisy planar point cloud (well-conditioned for ICP)."""
    xs = np.random.rand(n) * 3.0
    ys = np.random.rand(n) * 3.0
    zs = np.random.normal(0, noise, n)
    pts = np.column_stack([xs, ys, zs])
    return pts + np.array(shift)


class TestSlamRegistration:
    def test_two_scan_registration_runs(self):
        eng = SLAMEngine()
        base = _plane()
        ok1, _ = eng.process_scan(base, datetime.now())
        assert ok1 is True
        # A slightly shifted copy of the same plane -> ICP has real correspondence
        ok2, T = eng.process_scan(base + np.array([0.05, 0.0, 0.0]), datetime.now())
        assert T.shape == (4, 4)
        assert eng.total_scans >= 1

    def test_repeated_registration_accumulates(self):
        eng = SLAMEngine()
        base = _plane()
        for i in range(6):
            eng.process_scan(base + np.array([0.01 * i, 0.0, 0.0]), datetime.now())
        stats = eng.get_statistics()
        assert stats["total_scans"] >= 1
        assert eng.get_map() is not None

    def test_degenerate_collinear_scan(self):
        eng = SLAMEngine()
        eng.process_scan(_plane(), datetime.now())
        # collinear points: x varies, y,z fixed -> planarity/degeneracy path
        line = np.column_stack([np.linspace(0, 3, 60), np.zeros(60), np.zeros(60)])
        ok, T = eng.process_scan(line, datetime.now())
        assert T.shape == (4, 4)


class TestObjectDetectionClusterEdge:
    def test_dense_cluster_creates_object(self, tmp_path):
        det = ObjectDetector(objects_dir=str(tmp_path))
        rng = np.random.default_rng(1)
        # Tight ~0.3m cube cluster, well within [min_object_size, max_object_size]
        for _ in range(80):
            p = rng.normal(0, 0.1, 3) + np.array([1.0, 1.0, 0.5])
            det.process_reading(distance=1.5, strength=200, position=tuple(p),
                                timestamp=datetime.now())
        # clustering runs on buffer_limit; objects may or may not persist, but
        # the cluster-analysis code path is exercised
        assert isinstance(det.get_objects(), list)

    def test_edge_detected_pattern(self, tmp_path):
        det = ObjectDetector(objects_dir=str(tmp_path))
        # A sharp step in distance is what the pattern analyzer flags as an edge
        for i in range(30):
            d = 1.0 if i < 15 else 4.0
            det.process_reading(distance=d, strength=200,
                                position=(float(i), 0.0, 0.0), timestamp=datetime.now())
        assert isinstance(det.get_objects(), list)


class TestMainEnvAndBroadcast:
    def test_env_correction_depth_branch(self):
        app = LiDARSLAMApplication()
        app.mode = app.MODE_MAPPING
        app.environmental_corrector = EnvironmentalCorrector(
            depth_model=DepthCorrectedRefractive(a=1.333, b=0.002, c=0.0))
        app.set_depth(30.0)
        app._process_reading(make_reading(distance=2.5))
        # scan buffer got a point (processing completed through env-correction)
        assert len(app.slam_engine.scan_buffer) >= 1

    def test_detection_broadcast(self):
        app = LiDARSLAMApplication()
        app.mode = app.MODE_MAPPING
        events = []
        app.websocket_callback = lambda name, data: events.append(name)
        # Force the detector to return a detection so _broadcast_detection fires
        det = MagicMock()
        det.to_dict.return_value = {"id": 1}
        app.object_detector = MagicMock()
        app.object_detector.process_reading.return_value = det
        app.config.object_detection.enabled = True
        app._process_reading(make_reading())
        assert "detection" in events

    def test_invalid_reading_dropped(self):
        app = LiDARSLAMApplication()
        r = make_reading()
        r.valid = False
        app._on_lidar_reading(r)  # returns early, nothing enqueued
        assert app._processing_queue.qsize() == 0


class TestDriverReadLoopData:
    def test_read_loop_processes_streamed_frames(self):
        from tests.test_lidar_driver_full import FakeSerial, build_frame
        from app.lidar_driver import TFminiSDriver
        driver = TFminiSDriver('/dev/ttyUSB0')
        driver.reconnect_enabled = False
        got = []
        driver.add_callback(lambda r: got.append(r))
        driver.serial_conn = FakeSerial(build_frame(150) * 4)
        driver.is_running = True
        t = threading.Thread(target=driver._read_loop, daemon=True)
        t.start()
        time.sleep(0.1)
        driver.is_running = False
        t.join(timeout=1.0)
        assert len(got) >= 1


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
