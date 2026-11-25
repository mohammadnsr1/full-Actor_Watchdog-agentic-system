"""Generic executor geometry and grasp helpers: pure Python, no ROS, Python 3.10.

Shared by both planner templates. Nothing here knows a task, a step or a part
name; it is the arm/gripper convention any planner on this cell uses:

    pose         robot_backend PoseDict {"position": {x,y,z}, "orientation": {x,y,z,w}, "frame_id"}
    compose(a,b) a (x) b, b expressed in a's own axes
    along_own_z  pose (x) [0, 0, d]: move d along the pose's own Z
    tcp_for      the RG2 TCP pose that puts the fingers' grasp centre at
                 `center` with the jaws `width_m` apart (robot_backend.utils.rg2_geometry)
    joint_path   a smooth (quintic) joint-space path for direct motion, no planning
    guarded grasp: close to (expected width - GRASP_CLEARANCE_M) under the
                 part's force_threshold_n from config/parts/executor_parts.yaml;
                 a final width at or below GRASP_MIN_PLAUSIBLE_WIDTH_M means
                 the gripper closed on nothing.
"""

from __future__ import annotations

from typing import Any

IDENTITY = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
GRASP_CLEARANCE_M = 0.005
GRASP_MIN_PLAUSIBLE_WIDTH_M = 0.003
DEFAULT_FORCE_THRESHOLD = 0.5


def spec_pose(spec: dict[str, Any] | None) -> dict[str, Any]:
    """A {position, orientation} mapping with missing fields filled (zero / identity)."""
    spec = spec or {}
    pos, ori = spec.get("position") or {}, spec.get("orientation") or {}
    return {"position": {k: float(pos.get(k, 0.0)) for k in "xyz"},
            "orientation": {k: float(ori.get(k, IDENTITY[k])) for k in "xyzw"}}


def obs_pose(position: Any, orientation_xyzw: Any, frame_id: str = "world") -> dict[str, Any]:
    x, y, z = (float(v) for v in position)
    qx, qy, qz, qw = (float(v) for v in orientation_xyzw)
    return {"position": {"x": x, "y": y, "z": z}, "orientation": {"x": qx, "y": qy, "z": qz, "w": qw}, "frame_id": frame_id}


def compose(a: dict[str, Any], b: dict[str, Any]) -> dict[str, Any]:
    from robot_backend.utils.geometry import compose_poses

    return compose_poses(a, spec_pose(b))


def along_own_z(pose: dict[str, Any], distance_m: float) -> dict[str, Any]:
    from robot_backend.utils.geometry import compose_poses, make_pose

    return compose_poses(pose, make_pose({"x": 0.0, "y": 0.0, "z": float(distance_m)}, IDENTITY))


def tcp_for(center: dict[str, Any], width_m: float) -> dict[str, Any]:
    from robot_backend.utils.rg2_geometry import load_rg2_grasp_offset_lookup, tcp_target_from_grasp_center

    # Without the calibration table (isaac_trajectory_bridge/config/rg2_calibration_table.csv, found
    # through the ament index or the source tree) rg2_geometry falls back to a straight line between
    # its open and closed offsets, which is 6.5 mm short of the table at 21 mm jaw width and 12.4 mm
    # at 43 mm. A TCP target from that would put every grasp and insert that far off, silently.
    if load_rg2_grasp_offset_lookup() is None:
        raise RuntimeError("tcp_for: the RG2 calibration table was not found (source ros2_ws/install/setup.bash); "
                           "refusing the linear fallback")
    return tcp_target_from_grasp_center(center, float(width_m))


def raised(pose: dict[str, Any], dz_m: float) -> dict[str, Any]:
    """The same pose moved dz_m along world Z."""
    out = {"position": dict(pose["position"]), "orientation": dict(pose["orientation"]), "frame_id": pose.get("frame_id", "world")}
    out["position"]["z"] += float(dz_m)
    return out


def guarded_target_width(expected_width_m: float) -> float:
    return max(0.0, float(expected_width_m) - GRASP_CLEARANCE_M)


def grasp_force(parts: dict[str, Any], object_id: str) -> float:
    entry = (parts.get(object_id) or {}).get("grasp") or {}
    return float(entry.get("force_threshold_n", DEFAULT_FORCE_THRESHOLD))


DIRECT_MAX_JOINT_SPEED = 0.5      # rad/s: default cap for direct joint moves (no collision checking)


def joint_path(start: list[float], goal: list[float], duration_s: float | None = None,
               max_speed: float = DIRECT_MAX_JOINT_SPEED, rate_hz: float = 50.0) -> list[tuple[float, list[float], list[float]]]:
    """[(t, positions, velocities)] from start to goal along a quintic (zero velocity and
    acceleration at both ends). Without a duration, the slowest one that keeps every joint
    under max_speed (a quintic peaks at 1.875 x the mean speed)."""
    if len(start) != len(goal):
        raise ValueError("start and goal need the same joints")
    span = max((abs(g - s0) for s0, g in zip(start, goal)), default=0.0)
    need = 1.875 * span / max_speed if max_speed > 0 else 0.0
    T = max(float(duration_s or 0.0), need, 0.1)
    if duration_s is not None and duration_s < need:
        raise ValueError(f"duration {duration_s:.2f}s would exceed {max_speed} rad/s; at least {need:.2f}s")
    n = max(2, int(round(T * rate_hz)))
    out = []
    for i in range(1, n + 1):
        t = T * i / n
        u = t / T
        s_, ds = 10 * u**3 - 15 * u**4 + 6 * u**5, (30 * u**2 - 60 * u**3 + 30 * u**4) / T
        out.append((t, [a + (b - a) * s_ for a, b in zip(start, goal)], [(b - a) * ds for a, b in zip(start, goal)]))
    return out
