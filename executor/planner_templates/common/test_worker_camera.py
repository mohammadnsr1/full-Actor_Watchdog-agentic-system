"""camera1 decoding and save_frame, offline (no ROS graph): stand-in messages."""

import struct
import sys
import threading
import zlib
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import worker as W  # noqa: E402


class _NoTF:
    def lookup_transform(self, *a, **k):
        raise RuntimeError("no tf in this test")


def _worker(rgb, depth, info):
    w = object.__new__(W.Worker)
    w._cam = {"rgb": rgb, "depth": depth, "camera_info": info}
    w._cam_lock = threading.Lock()
    w._tf = _NoTF()
    w._cam_subscribed = True
    w.world = "world"
    w._saved = 0
    return w


def _msgs(encoding="bgr8", depth_encoding="32FC1"):
    h, wd = 2, 3
    px = np.arange(h * wd * 3, dtype=np.uint8).reshape(h, wd, 3)
    stamp = SimpleNamespace(sec=1, nanosec=500_000_000)
    rgb = SimpleNamespace(encoding=encoding, height=h, width=wd, step=wd * 3, data=px.tobytes(),
                          header=SimpleNamespace(frame_id="camera1_optical_frame", stamp=stamp))
    if depth_encoding == "32FC1":
        d = np.full((h, wd), 0.5, dtype=np.float32).tobytes()
    else:
        d = np.array([[500, 0, 500], [500, 500, 500]], dtype=np.uint16).tobytes()
    depth = SimpleNamespace(encoding=depth_encoding, height=h, width=wd, data=d)
    info = SimpleNamespace(k=[300.0, 0, 1.5, 0, 300.0, 1.0, 0, 0, 1])
    return px, rgb, depth, info


def test_bgr_is_flipped_and_depth_in_metres():
    px, rgb, depth, info = _msgs("bgr8", "32FC1")
    f = _worker(rgb, depth, info).camera1()
    assert f["rgb"].shape == (2, 3, 3) and np.array_equal(f["rgb"], px[:, :, ::-1])
    assert f["depth"].shape == (2, 3) and float(f["depth"][0, 0]) == pytest.approx(0.5)
    assert f["K"][0, 0] == 300.0 and f["world_T_camera"] is None and f["stamp_s"] == pytest.approx(1.5)


def test_16bit_depth_is_millimetres_and_zero_is_nan():
    _, rgb, depth, info = _msgs("rgb8", "16UC1")
    d = _worker(rgb, depth, info).camera1()["depth"]
    assert d[0, 0] == pytest.approx(0.5) and np.isnan(d[0, 1])


def test_save_frame_writes_png_and_npy(tmp_path, monkeypatch):
    pytest.importorskip("robot_backend.utils.png_encode")
    monkeypatch.setenv("CELL_EPISODE_DIR", str(tmp_path))
    _, rgb, depth, info = _msgs()
    w = _worker(rgb, depth, info)
    path = w.save_frame("after insert u_brown")
    assert path.endswith("001_after_insert_u_brown.png")
    data = Path(path).read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    width, height = struct.unpack(">II", data[16:24])
    assert (width, height) == (3, 2)
    assert Path(path[:-4] + ".npy").is_file()
    assert zlib  # png is zlib-compressed; decoding is the viewer's job


def test_joint_path_is_smooth_and_speed_capped():
    from geometry import joint_path

    path = joint_path([0.0, 1.0], [0.5, 1.0], max_speed=0.5)
    t_end, q_end, v_end = path[-1]
    assert q_end == pytest.approx([0.5, 1.0]) and v_end == pytest.approx([0.0, 0.0], abs=1e-9)
    assert t_end == pytest.approx(1.875 * 0.5 / 0.5)
    assert max(abs(v[0]) for _, _, v in path) <= 0.5 + 1e-9
    with pytest.raises(ValueError):
        joint_path([0.0], [1.0], duration_s=0.5, max_speed=0.5)
