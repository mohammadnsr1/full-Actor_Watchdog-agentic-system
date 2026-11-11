"""Humble / Jazzy differences, isolated in one place.

Everything the shared packages do differently per distro goes through a
function here, and docs/PLATFORMS.md lists the same items. Keep this module
free of ROS imports at module level so launch files and tests can use it.

Known differences handled:
    bag_record_command   rosbag2's default storage is sqlite3 on Humble and
                         mcap on Jazzy; the cell always records mcap.
    set_if_present       message fields that exist on one distro only
                         (moveit_msgs MotionPlanRequest.pipeline_id on Humble
                         needs verifying; see BUILD_PLAN section 5.3).
    python_baseline_ok   the shared code must run on 3.10 (Humble) and 3.12 (Jazzy).

APIs the shared packages must not use, because they do not exist on Humble:
    rclpy.wait_for_message (Iron+), Node.create_timer(autostart=...) (Jazzy),
    rclpy.executors.Executor.spin_once_until_future_complete changes (Jazzy),
    moveit_configs_utils features newer than MoveIt 2.5, and any Python
    feature newer than 3.10 (see docs/PLATFORMS.md).
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any, Sequence

HUMBLE = "humble"
JAZZY = "jazzy"
SUPPORTED_DISTROS = (HUMBLE, JAZZY)

PYTHON_BASELINE = (3, 10)


def distro() -> str:
    """The sourced ROS distro, from ROS_DISTRO; '' when not sourced."""
    return os.environ.get("ROS_DISTRO", "").strip().lower()


def is_humble() -> bool:
    return distro() == HUMBLE


def python_baseline_ok() -> bool:
    return sys.version_info[:2] >= PYTHON_BASELINE


def bag_record_command(
    out_dir: str | os.PathLike[str],
    topics: Sequence[str],
    storage: str = "mcap",
    extra_args: Sequence[str] = (),
) -> list[str]:
    """`ros2 bag record` with the storage plugin spelled out.

    Humble defaults to sqlite3 and needs ros-humble-rosbag2-storage-mcap for
    mcap; Jazzy defaults to mcap. Passing `-s` on both keeps the record
    identical. `--storage` is accepted by both distros; `-s` is its short form.
    """
    cmd = ["ros2", "bag", "record", "-s", storage, "-o", str(Path(out_dir)), *extra_args]
    cmd.extend(str(t) for t in topics)
    return cmd


def set_if_present(msg: Any, field: str, value: Any) -> bool:
    """Set a message field only when this distro's message has it.

    Returns True when set. Use for fields documented as distro-dependent
    (e.g. MotionPlanRequest.pipeline_id); anything else should be set directly
    so a missing field fails loudly.
    """
    fields = getattr(msg, "get_fields_and_field_types", None)
    names = fields() if callable(fields) else {}
    if field in names:
        setattr(msg, field, value)
        return True
    return False


def message_has_field(msg_type: Any, field: str) -> bool:
    fields = getattr(msg_type, "get_fields_and_field_types", None)
    return callable(fields) and field in fields()
