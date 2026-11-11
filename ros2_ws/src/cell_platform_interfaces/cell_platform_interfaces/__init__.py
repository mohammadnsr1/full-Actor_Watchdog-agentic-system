"""Platform-neutral contracts for the assembly cell.

Modules:
    platform_config   the one reader for config/platforms/<platform>.yaml and
                      config/cameras/<platform>.yaml (rule: no name literals in code)
    compat            the Humble/Jazzy differences, isolated here
    signals           signal-availability vocabulary (available | proxy | missing)
    types             result and reading dataclasses, each stamped with wall_time_iso
    interfaces        abstract MotionBackend, GripperBackend, PoseSource, SceneBackend
    fjt_motion        FollowJointTrajectory-based MotionBackend shared by sim and hardware
    topic_poses       Detection3DArray-based PoseSource shared by sim and hardware
    timebase          wall_time_iso()
"""

from cell_platform_interfaces.platform_config import (  # noqa: F401
    MissingPlatformValue,
    PlatformConfig,
    find_repo_root,
    load_cell_yaml,
    load_platform_config,
)
