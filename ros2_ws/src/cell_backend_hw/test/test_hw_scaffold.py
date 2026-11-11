"""Scaffold tests without a ROS graph: fake node objects, config refusals."""

from __future__ import annotations

import io
from pathlib import Path

from cell_backend_hw.gripper import URScriptRG6Gripper
from cell_backend_hw.scene import OperatorScene
from cell_platform_interfaces.platform_config import load_platform_config
from cell_platform_interfaces.types import PoseObservation

REPO = Path(__file__).resolve().parents[4]


class _Logger:
    def __init__(self):
        self.lines = []

    def warning(self, msg):
        self.lines.append(msg)

    info = warning


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg.data)


class _Node:
    def __init__(self):
        self.pub = _Pub()
        self.logger = _Logger()

    def get_logger(self):
        return self.logger

    def create_publisher(self, _type, _topic, _depth):
        return self.pub

    def create_subscription(self, *_a, **_k):
        return object()

    def destroy_publisher(self, _p):
        pass

    def destroy_subscription(self, _s):
        pass


class _Poses:
    def __init__(self, obs):
        self._obs = obs
        self.source = "foundationpose"
        self.camera = "camera2"

    def latest_all(self, max_age_s=None):
        return dict(self._obs)


def test_gripper_refuses_until_todo_filled():
    node = _Node()
    g = URScriptRG6Gripper(node, load_platform_config("hardware", REPO))
    assert g.provided_readings == {"width": "missing", "effort": "missing", "grip_detected": "missing"}
    r = g.read()
    assert r.width_m is None and r.width_source == "missing"
    result = g.open()
    assert result.ok is False and "TODO" in result.detail
    assert node.pub.sent == []


def test_gripper_sends_script_when_configured():
    node = _Node()
    platform = load_platform_config("hardware", REPO)
    platform.raw["gripper"]["urscript"]["grip_function"] = "rg_grip"
    platform.raw["gripper"]["width_range_m"] = [0.0, 0.16]
    g = URScriptRG6Gripper(node, platform)
    result = g.set_width(0.05)
    assert result.ok and node.pub.sent == ["rg_grip(0.0500, 0)"]
    bad = g.set_width(0.5)
    assert not bad.ok and bad.stop_reason == "rejected"


def test_operator_scene_prompt_and_verify():
    node = _Node()
    platform = load_platform_config("hardware", REPO)
    obs = {"base2": PoseObservation("base2", "world", (0.1, 0.2, 0.0), (0, 0, 0, 1), 0.9, "foundationpose", "camera2", 0)}
    scene = OperatorScene(node, platform, _Poses(obs), prompt_in=io.StringIO("done\n"), prompt_out=io.StringIO())
    assert scene.reset({"base2": {"position": [0.1, 0.2, 0.0]}}).ok
    verified = scene.verify_reset({"position_m": 0.01}, {"base2": {"position": [0.1, 0.2, 0.0]}, "hex_blue": {"position": [0, 0, 0]}})
    assert not verified.ok and verified.data["missing_evidence"] == ["hex_blue"]
    scene2 = OperatorScene(node, platform, _Poses(obs), prompt_in=io.StringIO("abort\n"), prompt_out=io.StringIO())
    assert not scene2.reset().ok
