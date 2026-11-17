#!/usr/bin/env python3
"""Pick check on the sim cell, driven by an executor instruction file.

For every `pick_*` step of the instruction (or the ones named with --step)
this runs the step's grasp exactly as the executor geometry defines it and
checks the pick-side clauses of the instruction against simulator truth:

    width     gripper width at plateau within grasp_width_tolerance_m of the
              step's grasp_width_m, and above 3 mm (not closed on nothing);
              the grasp is the executor's guarded move (see Cell.grasp)
    pads      a `found` contact on BOTH finger pads with the object, on the pad
              face (scene.contact_pads[].surface_frame, scene.pad_face box)
    only      no pad touched anything but the object
    lift      the object rose with the TCP (at least lift_height_min_m)
    slip      the object's pose in the TCP frame moved less than
              grasp_slip_tolerance_m during the lift
    release   a `lost` contact on both pads after opening
    others    every other layout part stayed within disturbance_tolerance_m

Geometry (the executor's, from robot_backend.utils, not re-derived here):
    grasp centre  = object pose (x) grasp_center_offset        (object frame)
    pre-grasp     = grasp centre (x) [0, 0, approach_distance_m] (its own Z)
    TCP target    = tcp_target_from_grasp_center(centre, grasp_width_m) (RG2 table)
    lift          = lift_height_m along world +Z

The object pose is the command server's ground truth (the same prim the TF
publisher sends). Tolerances come from config/parts/watchdog_parts.yaml.

    source ros2_ws/install/setup.bash
    python3 tools/pick_check.py --dry-run               # poses and plans only, nothing moves
    python3 tools/pick_check.py --step pick_u_brown
    python3 tools/pick_check.py                         # every pick step, layout reset between

Needs Isaac playing with the cell command server and the cell up (trajectory
bridge, rg2_controller, move_group, cell planning service). Every topic,
service, action and frame name comes from config/platforms/sim.yaml; the
planning group has no platform key yet (--planning-group, SRDF group).
A JSON report is written to records/pick_check/<UTC stamp>.json.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

REPO = Path(__file__).resolve().parents[1]

# Copied from robot_backend/skills/gripper_skills.py (not importable outside the
# robot_backend venv: it pulls pydantic). Keep in step with that file.
GRASP_CLEARANCE_M = 0.005            # guarded target = grasp_width_m - this
GRASP_MIN_PLAUSIBLE_WIDTH_M = 0.003  # at or below: closed on air / part lost
DEFAULT_FORCE_THRESHOLD = 0.5        # when the part has no grasp.force_threshold_n


# -- pose helpers ------------------------------------------------------------------------

def pose_dict(p: dict[str, Any]) -> dict[str, Any]:
    """command-server pose {position: [..], orientation_xyzw: [..]} -> robot_backend PoseDict."""
    x, y, z = p["position"]
    qx, qy, qz, qw = p["orientation_xyzw"]
    return {"position": {"x": x, "y": y, "z": z}, "orientation": {"x": qx, "y": qy, "z": qz, "w": qw}, "frame_id": "world"}


def spec_pose(spec: dict[str, Any] | None) -> dict[str, Any]:
    spec = spec or {}
    pos, ori = spec.get("position") or {}, spec.get("orientation") or {}
    return {"position": {k: float(pos.get(k, 0.0)) for k in "xyz"},
            "orientation": {k: float(ori.get(k, 1.0 if k == "w" else 0.0)) for k in "xyzw"}}


def quat_to_mat(q: Any) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def in_tcp_frame(tcp: dict[str, Any], part: dict[str, Any]) -> np.ndarray:
    return quat_to_mat(tcp["orientation_xyzw"]).T @ (np.array(part["position"]) - np.array(tcp["position"]))


def xyz(p: dict[str, Any]) -> list[float]:
    return [round(p["position"][k], 4) for k in "xyz"]


def short(path: Any) -> str:
    return str(path).rsplit("/", 1)[-1]


# -- ROS side --------------------------------------------------------------------------

class Cell:
    def __init__(self, node: Node, platform: Any, group: str, vel: float) -> None:
        from cell_planner_msgs.srv import PlanCartesian, PlanToPose
        from control_msgs.action import FollowJointTrajectory
        from std_msgs.msg import Float64
        from std_srvs.srv import Trigger

        self.node, self.group, self.vel = node, group, vel
        self.world = str(platform.get("frames", "world"))
        self.tip = str(platform.get("frames", "tcp"))
        ns = str(platform.get("services", "planning_namespace")).rstrip("/")
        self.Trigger, self.PlanToPose, self.PlanCartesian, self.FJT = Trigger, PlanToPose, PlanCartesian, FollowJointTrajectory
        self.plan_pose = node.create_client(PlanToPose, f"{ns}/plan_to_pose")
        self.plan_cart = node.create_client(PlanCartesian, f"{ns}/plan_cartesian")
        self.g_open = node.create_client(Trigger, str(platform.get("gripper", "services", "open")))
        self.g_close = node.create_client(Trigger, str(platform.get("gripper", "services", "close")))
        from isaac_trajectory_bridge_interfaces.action import GripperGuardedMove

        action = str(platform.get("motion", "action"))
        self.fjt = ActionClient(node, FollowJointTrajectory, action)
        self.Guarded = GripperGuardedMove
        guarded = str(platform.get("gripper", "action", "guarded_move"))
        self.guarded = ActionClient(node, GripperGuardedMove, guarded)
        self.width: float | None = None
        node.create_subscription(Float64, platform.topic("gripper_width_true"), self._on_width, 10)
        for client in (self.plan_pose, self.plan_cart, self.g_open, self.g_close):
            if not client.wait_for_service(timeout_sec=10.0):
                raise SystemExit(f"service {client.srv_name} not available: is the cell up?")
        for ac, name in ((self.fjt, action), (self.guarded, guarded)):
            if not ac.wait_for_server(timeout_sec=10.0):
                raise SystemExit(f"action {name} not available: is the cell up?")

    def _on_width(self, msg: Any) -> None:
        self.width = float(msg.data)

    def wait(self, future: Any, timeout_s: float) -> Any:
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_s)
        if not future.done():
            raise RuntimeError(f"timed out after {timeout_s:.0f}s")
        return future.result()

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def width_plateau(self, seconds: float = 2.5, window: int = 10, still_m: float = 0.0005) -> float | None:
        """Sample the true width until the last `window` samples span < still_m."""
        samples: list[float] = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.width is not None:
                samples.append(self.width)
                if len(samples) >= window and max(samples[-window:]) - min(samples[-window:]) < still_m:
                    return samples[-1]
        return samples[-1] if samples else None

    def _stamped(self, pose: dict[str, Any]) -> Any:
        from geometry_msgs.msg import PoseStamped

        ps = PoseStamped()
        ps.header.frame_id = self.world
        for k in "xyz":
            setattr(ps.pose.position, k, float(pose["position"][k]))
        for k in "xyzw":
            setattr(ps.pose.orientation, k, float(pose["orientation"][k]))
        return ps

    def plan_to(self, pose: dict[str, Any]) -> Any:
        req = self.PlanToPose.Request()
        req.planning_group, req.tip_link = self.group, self.tip
        req.target = self._stamped(pose)
        req.position_tolerance_m, req.orientation_tolerance_rad = 0.001, 0.01
        req.max_velocity_scaling = req.max_acceleration_scaling = self.vel
        # Sampling planners fail now and then on a reachable pose (IK checked):
        # a few attempts before calling the pose unplannable.
        for attempt in range(1, 4):
            res = self.wait(self.plan_pose.call_async(req), 90.0)
            if res.ok:
                if attempt > 1:
                    print(f"       (planned on attempt {attempt})")
                return res.trajectory
        raise RuntimeError(f"plan_to_pose failed 3 times: {res.report.failure_code} {res.report.failure_reason}")

    def straight(self, pose: dict[str, Any]) -> Any:
        req = self.PlanCartesian.Request()
        req.planning_group, req.tip_link = self.group, self.tip
        req.waypoints = [self._stamped(pose)]
        req.eef_step_m, req.min_fraction, req.avoid_collisions = 0.005, 0.95, False
        req.max_velocity_scaling = req.max_acceleration_scaling = self.vel
        res = self.wait(self.plan_cart.call_async(req), 30.0)
        if not res.ok:
            raise RuntimeError(f"plan_cartesian failed (fraction {res.fraction:.2f}): {res.report.failure_code} {res.report.failure_reason}")
        return res.trajectory

    def execute(self, traj: Any, label: str) -> None:
        goal = self.FJT.Goal()
        goal.trajectory = traj
        handle = self.wait(self.fjt.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise RuntimeError(f"{label}: trajectory rejected by the bridge")
        last = traj.points[-1].time_from_start if traj.points else None
        duration = (last.sec + last.nanosec * 1e-9) if last else 0.0
        result = self.wait(handle.get_result_async(), duration * 4 + 30.0).result
        if result.error_code != 0:
            raise RuntimeError(f"{label}: error_code {result.error_code} {result.error_string}")

    def move(self, pose: dict[str, Any], label: str, straight: bool = False) -> None:
        self.execute(self.straight(pose) if straight else self.plan_to(pose), label)

    def grasp(self, grasp_width_m: float, force_threshold: float) -> Any:
        """The executor's grasp (robot_backend/skills/gripper_skills.py grasp_skill):
        GripperGuardedMove, target width grasp_width_m - GRASP_CLEARANCE_M, the part's
        force_threshold_n, mode 'torque', no width-tolerance override."""
        goal = self.Guarded.Goal()
        goal.target_width_m = max(0.0, grasp_width_m - GRASP_CLEARANCE_M)
        goal.force_threshold = float(force_threshold)
        goal.mode = "torque"
        goal.width_tolerance_m = 0.0
        handle = self.wait(self.guarded.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise RuntimeError("guarded grasp rejected")
        return self.wait(handle.get_result_async(), 30.0).result

    def gripper(self, which: str) -> None:
        res = self.wait((self.g_open if which == "open" else self.g_close).call_async(self.Trigger.Request()), 30.0)
        if not res.success:
            raise RuntimeError(f"gripper {which}: {res.message}")


# -- the check ------------------------------------------------------------------------

def step_geometry(step: dict[str, Any], obj: dict[str, Any]) -> dict[str, Any]:
    from robot_backend.utils.geometry import compose_poses, grasp_center_from_object_and_offset, make_pose
    from robot_backend.utils.rg2_geometry import tcp_target_from_grasp_center

    width = float(step["grasp_width_m"])
    gc = grasp_center_from_object_and_offset(pose_dict(obj), spec_pose(step.get("grasp_center_offset")))
    pre_gc = compose_poses(gc, make_pose({"x": 0.0, "y": 0.0, "z": float(step.get("approach_distance_m", -0.10))}, {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}))
    grasp = tcp_target_from_grasp_center(gc, width)
    pre = tcp_target_from_grasp_center(pre_gc, width)
    lift = {"position": dict(grasp["position"]), "orientation": dict(grasp["orientation"])}
    lift["position"]["z"] += float(step.get("lift_height_m", 0.1))
    return {"grasp_center": gc, "pre_grasp_tcp": pre, "grasp_tcp": grasp, "lift_tcp": lift, "width_m": width}


def run_step(step: dict[str, Any], cell: Cell | None, call: Any, platform: Any, tol: dict[str, Any],
             layout_parts: list[str], args: argparse.Namespace, exec_parts: dict[str, Any]) -> dict[str, Any]:
    obj_id = str(step["object_id"])
    out: dict[str, Any] = {"step_id": step["step_id"], "object_id": obj_id, "checks": {}}
    checks = out["checks"]

    def check(name: str, ok: bool, detail: str) -> None:
        checks[name] = {"pass": bool(ok), "detail": detail}
        print(f"    {'PASS' if ok else 'FAIL'}  {name:8s} {detail}")

    call("load_layout", assembly=args.assembly, layout=args.layout, seed=0)
    poses = call("report_poses")["poses"]
    geo = step_geometry(step, poses[obj_id])
    out["geometry"] = {k: (v if not isinstance(v, dict) else {"position": xyz(v), "orientation": v["orientation"]}) for k, v in geo.items()}
    print(f"    object {xyz(pose_dict(poses[obj_id]))}  grasp centre {xyz(geo['grasp_center'])}  width {geo['width_m'] * 1000:.0f} mm")
    print(f"    TCP pre-grasp {xyz(geo['pre_grasp_tcp'])}  grasp {xyz(geo['grasp_tcp'])}  lift {xyz(geo['lift_tcp'])}")
    if cell is None:
        return out

    if args.dry_run:
        try:
            cell.plan_to(geo["pre_grasp_tcp"])
            check("reach", True, "pre-grasp planned (nothing moved)")
        except RuntimeError as exc:
            check("reach", False, str(exc))
        return out

    call("contacts_enable", enable=True)
    cell.gripper("open")
    cell.spin_for(1.5)
    before = call("report_poses")["poses"]

    print("    -> pre-grasp")
    cell.move(geo["pre_grasp_tcp"], "pre-grasp")
    print("    -> grasp (straight)")
    cell.move(geo["grasp_tcp"], "descend", straight=True)
    seq = call("contacts_poll", since_seq=0)["last_seq"]

    force = float(((exec_parts.get(obj_id) or {}).get("grasp") or {}).get("force_threshold_n", DEFAULT_FORCE_THRESHOLD))
    print(f"    -> guarded grasp (target {max(0.0, geo['width_m'] - GRASP_CLEARANCE_M) * 1000:.0f} mm, force_threshold {force})")
    res = cell.grasp(geo["width_m"], force)
    out["guarded_grasp"] = {"success": res.success, "stop_reason": res.stop_reason, "final_width_m": res.final_width_m,
                            "final_effort": res.final_effort, "asymmetry_detected": res.asymmetry_detected, "message": res.message}
    print(f"       stop_reason={res.stop_reason} final_width={res.final_width_m * 1000:.1f} mm effort={res.final_effort:.3f} asym={res.asymmetry_detected}")
    width = cell.width_plateau()
    poll = call("contacts_poll", since_seq=seq)
    seq = poll["last_seq"]
    close_events = [e for e in poll["events"] if e.get("event") != "error"]
    out["close_events"] = close_events
    for e in close_events:
        d = e.get("distance_m")
        print(f"       {e['event']:5s} {short(e['pad']):18s} {short(e['body']):20s} on_pad={e.get('on_pad_surface')} "
              f"dist={'-' if d is None else f'{d * 1000:.1f}mm'} in_pad_frame_mm={None if e.get('point_in_pad_frame') is None else [round(v * 1000, 1) for v in e['point_in_pad_frame']]}")

    wtol = float(tol.get("grasp_width_tolerance_m", 0.005))
    check("width", width is not None and width > GRASP_MIN_PLAUSIBLE_WIDTH_M and abs(width - geo["width_m"]) <= wtol,
          f"plateau {'-' if width is None else f'{width * 1000:.1f}'} mm vs grasp_width_m {geo['width_m'] * 1000:.0f} +/- {wtol * 1000:.0f} mm"
          f" (and > {GRASP_MIN_PLAUSIBLE_WIDTH_M * 1000:.0f} mm, else closed on nothing)")
    pads = [str(p["body"]) for p in platform.get("scene", "contact_pads")]
    on_obj = {short(e["pad"]): e for e in close_events if e["event"] == "found" and short(e["body"]) == obj_id}
    check("pads", all(short(p) in on_obj for p in pads) and all(e.get("on_pad_surface") for e in on_obj.values()),
          ", ".join(f"{short(p)}: {'no contact' if short(p) not in on_obj else ('on pad' if on_obj[short(p)].get('on_pad_surface') else 'off pad')}" for p in pads))

    held_tcp = call("report_poses")["poses"]
    rel0 = in_tcp_frame(held_tcp["tcp"], held_tcp[obj_id])
    print("    -> lift")
    cell.move(geo["lift_tcp"], "lift", straight=True)
    cell.spin_for(1.0)
    lifted = call("report_poses")["poses"]
    rise = lifted[obj_id]["position"][2] - held_tcp[obj_id]["position"][2]
    need = float(tol.get("lift_height_min_m", 0.05))
    check("lift", rise >= need, f"object rose {rise * 1000:.1f} mm (need >= {need * 1000:.0f})")
    slip = float(np.linalg.norm(in_tcp_frame(lifted["tcp"], lifted[obj_id]) - rel0))
    stol = float(tol.get("grasp_slip_tolerance_m", 0.01))
    check("slip", slip <= stol, f"object moved {slip * 1000:.1f} mm in the TCP frame (<= {stol * 1000:.0f})")

    print("    -> lower, open, retreat")
    cell.move(geo["grasp_tcp"], "lower", straight=True)
    cell.gripper("open")
    cell.spin_for(2.0)
    poll = call("contacts_poll", since_seq=seq)
    open_events = [e for e in poll["events"] if e.get("event") != "error"]
    out["open_events"] = open_events
    cell.move(geo["pre_grasp_tcp"], "retreat", straight=True)

    lost = {short(e["pad"]) for e in open_events if e["event"] == "lost" and short(e["body"]) == obj_id}
    check("release", all(short(p) in lost for p in pads), f"lost on {sorted(lost) or 'no pad'}")
    others = sorted({f"{short(e['pad'])}->{short(e['body'])}" for e in close_events + open_events if e["event"] == "found" and short(e["body"]) != obj_id})
    check("only", not others, "pads touched only the object" if not others else f"also touched: {others}")

    after = call("report_poses")["poses"]
    dtol = float(tol.get("disturbance_tolerance_m", 0.005))
    moved = {}
    for pid in layout_parts:
        if pid == obj_id or pid not in before or pid not in after:
            continue
        d = float(np.linalg.norm(np.array(after[pid]["position"]) - np.array(before[pid]["position"])))
        if d > dtol:
            moved[pid] = round(d * 1000, 1)
    check("others", not moved, f"all other parts within {dtol * 1000:.0f} mm" if not moved else f"moved (mm): {moved}")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--instruction", default=str(REPO / "config" / "tasks" / "fmb_assembly_2.yaml"))
    ap.add_argument("--step", action="append", help="step_id to run (repeatable); default every pick_* step")
    ap.add_argument("--assembly", default="fmb_assembly_2")
    ap.add_argument("--layout", default="nominal")
    ap.add_argument("--planning-group", default="ur_manipulator")
    ap.add_argument("--vel", type=float, default=0.2, help="velocity/acceleration scaling")
    ap.add_argument("--dry-run", action="store_true", help="compute poses and plan the pre-grasp only; nothing moves")
    args = ap.parse_args()

    from cell_backend_sim.scene import CommandServerClient
    from cell_platform_interfaces.platform_config import load_platform_config

    instruction = yaml.safe_load(Path(args.instruction).read_text())
    steps = [s for s in instruction.get("steps") or [] if (s["step_id"] in args.step if args.step else str(s["step_id"]).startswith("pick_"))]
    if not steps:
        raise SystemExit(f"no matching steps in {args.instruction}")
    parts_tol = (yaml.safe_load((REPO / "config" / "parts" / "watchdog_parts.yaml").read_text()) or {}).get("parts") or {}
    exec_parts = (yaml.safe_load((REPO / "config" / "parts" / "executor_parts.yaml").read_text()) or {}).get("parts") or {}
    layout = yaml.safe_load((REPO / "config" / "layouts" / args.assembly / f"{args.layout}.yaml").read_text())
    layout_parts = list((layout.get("parts") or {}).keys())

    platform = load_platform_config("sim")
    srv = CommandServerClient(str(platform.get_optional("scene", "host", default="127.0.0.1")), int(platform.get("scene", "port")), timeout_s=90.0)

    def call(cmd: str, **params: Any) -> dict[str, Any]:
        r = srv.call(cmd, **params)
        if not r.get("ok"):
            raise RuntimeError(f"{cmd}: {r.get('error') or r.get('errors') or r}")
        return r

    rclpy.init()
    node = Node("pick_check")
    results: list[dict[str, Any]] = []
    try:
        cell = Cell(node, platform, args.planning_group, args.vel)
        for step in steps:
            print(f"\n== {step['step_id']} ({step['object_id']}){' [dry run]' if args.dry_run else ''}")
            try:
                results.append(run_step(step, cell, call, platform, (parts_tol.get(step["object_id"]) or {}).get("tolerances") or {}, layout_parts, args, exec_parts))
            except (RuntimeError, KeyError) as exc:
                print(f"    ABORT {exc}")
                results.append({"step_id": step["step_id"], "object_id": step["object_id"], "error": str(exc), "checks": {}})
    finally:
        try:
            call("load_layout", assembly=args.assembly, layout=args.layout, seed=0)
        except RuntimeError as exc:
            print(f"final reset: {exc}")
        node.destroy_node()
        rclpy.shutdown()

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = REPO / "records" / "pick_check"
    out_dir.mkdir(parents=True, exist_ok=True)
    report = out_dir / f"{stamp}.json"
    report.write_text(json.dumps({"instruction": args.instruction, "dry_run": args.dry_run, "results": results}, indent=2, default=str))

    print("\nSUMMARY")
    all_ok = True
    for r in results:
        failed = [k for k, v in r["checks"].items() if not v["pass"]]
        ok = not failed and "error" not in r
        all_ok = all_ok and ok
        print(f"  {r['step_id']:28s} {'PASS' if ok else 'FAIL'}  {r.get('error') or (', '.join(failed) if failed else '')}")
    print(f"report: {report}")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
