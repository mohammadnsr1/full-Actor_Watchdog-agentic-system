"""Pure-Python tests for the platform config reader (no ROS)."""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from cell_platform_interfaces.platform_config import (
    MissingPlatformValue,
    PlatformConfig,
    find_repo_root,
    load_platform_config,
    resolve_platform_name,
)

REPO = Path(__file__).resolve().parents[4]


@pytest.fixture(autouse=True)
def _clean_env(monkeypatch):
    for var in ("CELL_PLATFORM", "CELL_REPO_ROOT", "CELL_CONFIG_PATH"):
        monkeypatch.delenv(var, raising=False)


def test_repo_root_found():
    assert find_repo_root(REPO) == REPO


def test_sim_platform_loads_and_names_resolve():
    p = load_platform_config("sim", REPO)
    assert p.name == "sim"
    assert p.topic("joint_states") == "/joint_states"
    assert p.frame("world") == "world" and p.frame("tcp") == "tcp"
    assert p.joints("arm")[0] == "shoulder_pan_joint" and len(p.joints("arm")) == 6
    assert p.get("motion", "cancel_service").endswith("/_action/cancel_goal")
    assert p.service("request_go") == "/cell/request_go"
    assert p.use_sim_time is True


def test_missing_value_is_an_error_not_a_fallback():
    p = load_platform_config("hardware", REPO)
    with pytest.raises(MissingPlatformValue) as info:
        p.frame("tcp")  # TODO(mohammad) placeholder is empty
    assert "frames.tcp" in str(info.value)
    assert p.get_optional("frames", "tcp", default="x") == "x"
    assert not p.has("topics", "clock")


def test_platform_name_from_cell_yaml_and_env(monkeypatch):
    assert resolve_platform_name(None, REPO) == "sim"
    monkeypatch.setenv("CELL_PLATFORM", "hardware")
    assert resolve_platform_name(None, REPO) == "hardware"
    with pytest.raises(ValueError):
        resolve_platform_name("moon", REPO)


def test_cameras_and_roles():
    p = load_platform_config("sim", REPO)
    assert p.camera("camera2")["role"] == "table"
    assert p.camera_enabled("camera2") and not p.camera_enabled("camera1")
    assert p.camera_topic("camera2", "rgb") == "/camera/depth/rgb"
    assert p.camera_topic("recorder", "rgb") == "/recorder_rgb"
    with pytest.raises(MissingPlatformValue):
        p.camera_topic("camera2", "thermal")


def test_declared_platform_mismatch_rejected(tmp_path: Path):
    with pytest.raises(ValueError):
        PlatformConfig("sim", {"platform": "hardware"}, tmp_path / "x.yaml", REPO)


def test_resolve_path_relative_to_repo():
    p = load_platform_config("sim", REPO)
    assert p.resolve_path("config/gripper/rg2_kinematics.yaml").is_file()
