"""Task-file interpretation and step geometry: pure Python, no ROS, Python 3.10.

The task file (config/tasks/<assembly>.yaml) lists steps. This template knows
three step shapes, told apart by the step_id prefix (an authored planner may
do this differently):

    pick_<part>             grasp the part where it lies, then lift
    place_approach_<part>   carry the held part to its placement frame plus
                            target_offset (a standoff, in the frame's own axes)
    place_<part>            move the held part onto placement frame (x)
                            target_offset, release, retreat

Geometry is the executor's convention (robot_backend.utils, the helpers that
motion_mcp_server and test/plan_assembly_step.py share, validated live by
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

IDENTITY = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
DEFAULT_APPROACH_M = -0.10
DEFAULT_LIFT_M = 0.10
# The executor's grasp (robot_backend/skills/gripper_skills.py): guarded close to
# grasp_width_m - GRASP_CLEARANCE_M, stopping on the part's force threshold; a
# final width at or below GRASP_MIN_PLAUSIBLE_WIDTH_M means it closed on nothing.
GRASP_CLEARANCE_M = 0.005
GRASP_MIN_PLAUSIBLE_WIDTH_M = 0.003
DEFAULT_FORCE_THRESHOLD = 0.5


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


def spec_pose(spec: dict[str, Any] | None) -> dict[str, Any]:
    spec = spec or {}
    pos, ori = spec.get("position") or {}, spec.get("orientation") or {}
    return {"position": {k: float(pos.get(k, 0.0)) for k in "xyz"},
            "orientation": {k: float(ori.get(k, IDENTITY[k])) for k in "xyzw"}}


def obs_pose(position: Any, orientation_xyzw: Any, frame_id: str = "world") -> dict[str, Any]:
    x, y, z = (float(v) for v in position)
    qx, qy, qz, qw = (float(v) for v in orientation_xyzw)
    return {"position": {"x": x, "y": y, "z": z}, "orientation": {"x": qx, "y": qy, "z": qz, "w": qw}, "frame_id": frame_id}


def _along_own_z(pose: dict[str, Any], distance_m: float) -> dict[str, Any]:
    from robot_backend.utils.geometry import compose_poses, make_pose

    return compose_poses(pose, make_pose({"x": 0.0, "y": 0.0, "z": float(distance_m)}, IDENTITY))


def _tcp(center: dict[str, Any], width_m: float) -> dict[str, Any]:
    from robot_backend.utils.rg2_geometry import tcp_target_from_grasp_center

    return tcp_target_from_grasp_center(center, float(width_m))


def pick_geometry(step: dict[str, Any], object_pose: dict[str, Any]) -> dict[str, Any]:
    from robot_backend.utils.geometry import grasp_center_from_object_and_offset

    width = float(step["grasp_width_m"])
    center = grasp_center_from_object_and_offset(object_pose, spec_pose(step.get("grasp_center_offset")))
    approach = float(step.get("approach_distance_m", DEFAULT_APPROACH_M))
    grasp = _tcp(center, width)
    lift = {"position": dict(grasp["position"]), "orientation": dict(grasp["orientation"]), "frame_id": grasp.get("frame_id", "world")}
    lift["position"]["z"] += float(step.get("lift_height_m", DEFAULT_LIFT_M))
    return {"width_m": width, "grasp_center": center, "pre_grasp_tcp": _tcp(_along_own_z(center, approach), width),
            "grasp_tcp": grasp, "lift_tcp": lift}


def place_geometry(step: dict[str, Any], frame_pose: dict[str, Any]) -> dict[str, Any]:
    from robot_backend.utils.geometry import compose_poses, grasp_center_from_object_and_offset

    width = float(step["grasp_width_m"])
    object_target = frame_pose
    if step.get("target_offset"):
        object_target = compose_poses(frame_pose, spec_pose(step["target_offset"]))
    center = grasp_center_from_object_and_offset(object_target, spec_pose(step.get("grasp_center_offset")))
    return {"width_m": width, "object_target": object_target, "grasp_center": center, "tcp": _tcp(center, width)}


def retreat_pose(tcp: dict[str, Any], approach_distance_m: float = DEFAULT_APPROACH_M) -> dict[str, Any]:
    return _along_own_z(tcp, approach_distance_m)


def pick_step_for(task: dict[str, Any], object_id: str) -> dict[str, Any] | None:
    for step in task.get("steps") or []:
        if step_kind(step) == "pick" and step.get("object_id") == object_id:
            return step
    return None


def guarded_target_width(grasp_width_m: float) -> float:
    return max(0.0, float(grasp_width_m) - GRASP_CLEARANCE_M)


def grasp_force(parts: dict[str, Any], object_id: str) -> float:
    entry = (parts.get(object_id) or {}).get("grasp") or {}
    return float(entry.get("force_threshold_n", DEFAULT_FORCE_THRESHOLD))


def motion_scaling(step: dict[str, Any], default: float) -> tuple[float, float]:
    m = step.get("motion") or {}
    return (float(m.get("max_velocity_scaling_factor", default)), float(m.get("max_acceleration_scaling_factor", default)))
