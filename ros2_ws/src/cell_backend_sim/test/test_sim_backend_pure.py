"""Tests that need no ROS graph: registry frame parsing, the command-server
client's refusal when unconfigured, and the gripper reading rules."""

from __future__ import annotations

from pathlib import Path

from cell_backend_sim.ground_truth_pose_publisher import _registry_object_frames
from cell_backend_sim.scene import CommandServerClient

REPO = Path(__file__).resolve().parents[4]


def test_registry_frames_parse():
    frames = _registry_object_frames(REPO / "ros2_ws" / "src" / "robot_backend" / "config" / "model_registry.yaml")
    assert frames, "registry has enabled objects with tf_frame/lookup_frame"
    for entry in frames.values():
        assert entry["tf_frame"] and entry["lookup_frame"]


def test_registry_frames_skip_disabled(tmp_path: Path):
    p = tmp_path / "r.yaml"
    p.write_text(
        "objects:\n  a: {tf_frame: a, lookup_frame: world, enabled: false}\n  b: {tf_frame: b, lookup_frame: world}\n  c: {lookup_frame: world}\n",
        encoding="utf-8",
    )
    assert list(_registry_object_frames(p)) == ["b"]


def test_command_server_client_unconfigured():
    client = CommandServerClient("127.0.0.1", 0)
    assert not client.configured
    response = client.call("health")
    assert response["ok"] is False and "port" in response["error"]


def test_command_server_client_connection_refused():
    client = CommandServerClient("127.0.0.1", 1, timeout_s=0.5)  # nothing listens on port 1
    response = client.call("health")
    assert response["ok"] is False and "error" in response
