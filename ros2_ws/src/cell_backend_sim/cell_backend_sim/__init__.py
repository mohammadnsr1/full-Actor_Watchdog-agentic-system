"""Isaac Sim backends. `build_backends(node, platform)` returns the Backends container."""

from __future__ import annotations

from typing import Any

from cell_platform_interfaces.interfaces import Backends
from cell_platform_interfaces.platform_config import PlatformConfig


def build_backends(node: Any, platform: PlatformConfig, callback_group: Any = None, cell_config: dict | None = None) -> Backends:
    if platform.name != "sim":
        raise ValueError(f"cell_backend_sim serves platform 'sim', not {platform.name!r}")
    from cell_backend_sim.gripper import SimRG2Gripper
    from cell_backend_sim.motion import SimMotion
    from cell_backend_sim.poses import make_pose_sources
    from cell_backend_sim.scene import SimScene

    executor_poses, watchdog_poses = make_pose_sources(node, platform, callback_group)
    return Backends(
        motion=SimMotion(node, platform, callback_group),
        gripper=SimRG2Gripper(node, platform, callback_group),
        executor_poses=executor_poses,
        watchdog_poses=watchdog_poses,
        scene=SimScene(node, platform, callback_group, cell_config or {}),
    )
