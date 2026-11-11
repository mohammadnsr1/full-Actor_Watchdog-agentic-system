"""Hardware backends (scaffold). `build_backends(node, platform)` returns the Backends container."""

from __future__ import annotations

from typing import Any

from cell_platform_interfaces.interfaces import Backends
from cell_platform_interfaces.platform_config import PlatformConfig


def build_backends(node: Any, platform: PlatformConfig, callback_group: Any = None, cell_config: dict | None = None) -> Backends:
    if platform.name != "hardware":
        raise ValueError(f"cell_backend_hw serves platform 'hardware', not {platform.name!r}")
    from cell_backend_hw.gripper import URScriptRG6Gripper
    from cell_backend_hw.motion import HardwareMotion
    from cell_backend_hw.poses import make_pose_sources
    from cell_backend_hw.scene import OperatorScene

    executor_poses, watchdog_poses = make_pose_sources(node, platform, callback_group)
    return Backends(
        motion=HardwareMotion(node, platform, callback_group),
        gripper=URScriptRG6Gripper(node, platform, callback_group),
        executor_poses=executor_poses,
        watchdog_poses=watchdog_poses,
        scene=OperatorScene(node, platform, watchdog_poses, callback_group, cell_config or {}),
    )
