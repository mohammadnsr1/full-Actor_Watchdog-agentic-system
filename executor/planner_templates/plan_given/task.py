"""Task-file interpretation and step geometry: pure Python, no ROS, Python 3.10.

The task file (config/tasks/<assembly>.yaml) lists steps. This template knows
three step shapes, told apart by the step_id prefix (an authored planner may
do this differently):

    pick_<part>             grasp the part where it lies, then lift
    place_approach_<part>   carry the held part to its placement frame plus
                            target_offset (a standoff, in the frame's own axes)
    place_<part>            move the held part onto placement frame (x)
                            target_offset, release, retreat

Geometry is the executor's convention (geometry.py over robot_backend.utils, the helpers that
v1's motion_mcp_server used (removed in Milestone 7), validated live by
tools/pick_check.py on 2026-11-17):

    grasp centre  = object pose (x) grasp_center_offset   (both object-relative)
    pre-grasp     = grasp centre (x) [0, 0, approach_distance_m]  (its own Z)
    object target = placement frame (x) target_offset     (frame's own axes)
    TCP target    = tcp_target_from_grasp_center(centre, width)  (RG2 table)
    lift          = TCP + lift_height_m along world Z
    retreat       = TCP (x) [0, 0, approach_distance_m]   (back along its own Z)

Poses are robot_backend PoseDicts: {"position": {x,y,z}, "orientation": {x,y,z,w}, "frame_id"}.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from geometry import (  # noqa: F401  (re-exported for planner.py)
    DEFAULT_FORCE_THRESHOLD, GRASP_CLEARANCE_M, GRASP_MIN_PLAUSIBLE_WIDTH_M, IDENTITY,
    along_own_z, grasp_force, guarded_target_width, obs_pose, raised, spec_pose, tcp_for,
)

DEFAULT_APPROACH_M = -0.10
DEFAULT_LIFT_M = 0.10


def load_task(path: str | Path) -> dict[str, Any]:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(data.get("steps"), list) or not data["steps"]:
        raise ValueError(f"{path}: no steps")
    return data


def step_kind(step: dict[str, Any]) -> str:
    sid = str(step.get("step_id", ""))
    for prefix, kind in (("place_approach_", "place_approach"), ("pick_", "pick"), ("place_", "place")):
        if sid.startswith(prefix):
            return kind
    raise ValueError(f"step {sid!r}: unknown step shape (expected pick_, place_approach_ or place_)")


def pick_geometry(step: dict[str, Any], object_pose: dict[str, Any]) -> dict[str, Any]:
    from robot_backend.utils.geometry import grasp_center_from_object_and_offset

    width = float(step["grasp_width_m"])
    center = grasp_center_from_object_and_offset(object_pose, spec_pose(step.get("grasp_center_offset")))
    approach = float(step.get("approach_distance_m", DEFAULT_APPROACH_M))
    grasp = tcp_for(center, width)
    return {"width_m": width, "grasp_center": center, "pre_grasp_tcp": tcp_for(along_own_z(center, approach), width),
            "grasp_tcp": grasp, "lift_tcp": raised(grasp, float(step.get("lift_height_m", DEFAULT_LIFT_M)))}


def place_geometry(step: dict[str, Any], frame_pose: dict[str, Any]) -> dict[str, Any]:
    from robot_backend.utils.geometry import compose_poses, grasp_center_from_object_and_offset

    width = float(step["grasp_width_m"])
    object_target = frame_pose
    if step.get("target_offset"):
        object_target = compose_poses(frame_pose, spec_pose(step["target_offset"]))
    center = grasp_center_from_object_and_offset(object_target, spec_pose(step.get("grasp_center_offset")))
    return {"width_m": width, "object_target": object_target, "grasp_center": center, "tcp": tcp_for(center, width)}


def retreat_pose(tcp: dict[str, Any], approach_distance_m: float = DEFAULT_APPROACH_M) -> dict[str, Any]:
    return along_own_z(tcp, approach_distance_m)


def pick_step_for(task: dict[str, Any], object_id: str) -> dict[str, Any] | None:
    for step in task.get("steps") or []:
        if step_kind(step) == "pick" and step.get("object_id") == object_id:
            return step
    return None


def motion_scaling(step: dict[str, Any], default: float) -> tuple[float, float]:
    m = step.get("motion") or {}
    return (float(m.get("max_velocity_scaling_factor", default)), float(m.get("max_acceleration_scaling_factor", default)))
