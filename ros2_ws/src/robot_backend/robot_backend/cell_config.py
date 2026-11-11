"""Read config/cell.yaml and the platform config for robot_backend processes.

cell.yaml resolution: CELL_CONFIG_PATH, else walk up from this file to the
directory holding config/cell.yaml and ros2_ws/. A missing file yields an
empty mapping; callers apply their own explicit defaults.

Platform names (topics, frames, joints, actions, services, cameras) come from
cell_platform_interfaces.platform_config through the helpers below. They are
strict: a missing name raises, it never falls back to a literal (rule 10 of
the build prompt). The platform is cached per process.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

ENV_VAR = "CELL_CONFIG_PATH"
RELATIVE_PATH = Path("config") / "cell.yaml"

_PLATFORM: Any = None


def find_cell_config() -> Path | None:
    env = os.environ.get(ENV_VAR, "").strip()
    if env:
        return Path(os.path.expanduser(env))
    origin = Path(__file__).resolve()
    for parent in [origin, *origin.parents]:
        if (parent / RELATIVE_PATH).is_file() and (parent / "ros2_ws").is_dir():
            return parent / RELATIVE_PATH
    return None


def load_cell_config() -> tuple[dict[str, Any], Path | None]:
    import yaml

    path = find_cell_config()
    if path is None or not path.is_file():
        return {}, path
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a mapping at the top level")
    return data, path


def section(config: dict[str, Any], *keys: str) -> dict[str, Any]:
    node: Any = config
    for key in keys:
        node = (node or {}).get(key, {}) if isinstance(node, dict) else {}
    return node if isinstance(node, dict) else {}


# -- platform ------------------------------------------------------------------


def platform(reload: bool = False):
    """The PlatformConfig for this process (cached). Raises when unresolvable."""
    global _PLATFORM
    if _PLATFORM is None or reload:
        from cell_platform_interfaces.platform_config import load_platform_config

        cfg_path = find_cell_config()
        repo_root = cfg_path.parent.parent if cfg_path is not None else None
        _PLATFORM = load_platform_config(repo_root=repo_root)
    return _PLATFORM


def topic(key: str) -> str:
    return platform().topic(key)


def frame(key: str) -> str:
    return platform().frame(key)


def service(key: str) -> str:
    return platform().service(key)


def platform_use_sim_time() -> bool:
    return platform().use_sim_time


def cell_block() -> dict[str, Any]:
    """The registry-shaped `cell:` block, built from the platform config.

    Replaces model_registry.yaml's cell section (gripper.effort_joint_name,
    gripper.tcp_frame, base_frame) so the frames and the effort joint are
    platform values. On a platform with no gripper effort joint the key is
    absent, and the effort monitors refuse to arm instead of guessing.
    """
    p = platform()
    gripper: dict[str, Any] = {"tcp_frame": p.frame("tcp")}
    effort_joint = p.get_optional("joints", "gripper_effort", default="")
    if effort_joint:
        gripper["effort_joint_name"] = str(effort_joint)
    return {"gripper": gripper, "base_frame": p.frame("world")}


def watchdog_camera() -> str:
    """The camera whose frames and poses are WatchDog's own channel."""
    return "camera2"


def watchdog_camera_topics() -> dict[str, Any]:
    """Topics of WatchDog's camera for the image buffer: rgb candidates, depth,
    camera_info candidates, pointcloud. Absent entries are empty."""
    cam = platform().camera(watchdog_camera())
    topics = cam.get("topics", {}) if isinstance(cam.get("topics"), dict) else {}
    rgb = [t for t in (topics.get("rgb"), topics.get("rgb_compressed")) if t]
    info = [t for t in (topics.get("camera_info"), topics.get("camera_info_legacy")) if t]
    return {
        "rgb": tuple(str(t) for t in rgb),
        "depth": str(topics.get("depth", "") or ""),
        "camera_info": tuple(str(t) for t in info),
        "pointcloud": str(topics.get("pointcloud", "") or ""),
        "optical_frame": str(cam.get("optical_frame", "") or ""),
    }
