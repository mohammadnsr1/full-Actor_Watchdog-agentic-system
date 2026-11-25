"""Plan-given planner template: execute the task file's steps in order.

    safe pose -> for each step: pick | place_approach | place -> claim_done

place_approach: free-space move to a hover pose above the approach pose, then a
straight descent; place: straight insert. Before each descent the part's pose in
the hand is measured (TF for the TCP, the pose channel for the part, arm at
rest), and the TCP target is computed so that the part, not the planned grasp,
lands on the target (task.py, "Carrying and inserting").

execute_task is called only after the episode runner has the start permission
(the WatchDog is ready, or there is no WatchDog), so the planner starts at once.
WatchDog consultations (consult on) are logged as they arrive; this template
does not act on them.

Every motion goes through the planning service and the platform motion
backend; every part and frame pose comes from the executor pose channel
(camera1). A cancelled trajectory is a halt: with watchdog.consult off the
planner stops; with it on, it waits for the ruling and re-plans the same
motion when allowed. See docs/PLANNER_CONTRACT.md.
"""

from __future__ import annotations

import logging
import os
import time
from pathlib import Path
from typing import Any

import yaml

import task as T
from worker import Halted, StepFailed, Worker

log = logging.getLogger("planner")

DEFAULT_SCALING = 0.2
RULING_WAIT_S = 120.0
AT_REST_S = 0.3   # after a motion, before reading TF and the pose channel together

_WORKER: Worker | None = None


def worker() -> Worker:
    global _WORKER
    if _WORKER is None:
        _WORKER = Worker(os.environ.get("CELL_PLATFORM", "sim"), os.environ["CELL_REPO_ROOT"])
    return _WORKER


class Stop(Exception):
    pass


class Planner:
    def __init__(self, w: Worker, task: dict[str, Any], parts: dict[str, Any]) -> None:
        self.w, self.task, self.parts = w, task, parts
        self.held: str | None = None
        self.last_tcp: dict[str, Any] | None = None
        self.denied: set[str] = set()
        self.as_grasped: dict[str, dict[str, Any]] = {}   # part -> its pose in the TCP frame, measured after the pick

    # -- motion with halt handling -------------------------------------------------

    def move(self, label: str, plan: Any) -> None:
        """plan() -> trajectory. Re-planned from the current state after an allowed ruling."""
        while True:
            seen = self.w.notice_count()
            try:
                self.w.execute(plan(), label)
                return
            except Halted as exc:
                if not self.w.consult:
                    raise Stop(f"halted during {label} (watchdog.consult is off): {exc}")
                ruling = self.w.wait_for_ruling(seen, RULING_WAIT_S)
                if ruling is None:
                    raise Stop(f"halted during {label}; no ruling within {RULING_WAIT_S:.0f}s")
                log.info("ruling %s for %s: %s", ruling.ruling, ruling.part or "episode", ruling.reason)
                if ruling.ruling == "denied_for_part":
                    self.denied.add(ruling.part)
                    raise Stop(f"halted during {label}; denied for part {ruling.part}: {ruling.reason}")
                if ruling.ruling not in ("allowed", "allowed_with_limits"):
                    raise Stop(f"halted during {label}; ruling {ruling.ruling}: {ruling.reason}")
                # allowed / allowed_with_limits: re-plan the same motion from where
                # the arm stopped. Limits (max_effort, max_tcp_speed_m_s, ...) are
                # physical quantities the WatchDog enforces with its own monitors;
                # an authored planner may also honour them here.

    def to_pose(self, label: str, pose: dict[str, Any], scaling: tuple[float, float], straight: bool = False) -> None:
        vel, acc = scaling
        self.move(label, (lambda: self.w.plan_straight(pose, vel, acc)) if straight else (lambda: self.w.plan_to_pose(pose, vel, acc)))
        self.last_tcp = pose

    # -- steps ---------------------------------------------------------------------

    def pick(self, step: dict[str, Any]) -> None:
        part = str(step["object_id"])
        if part in self.denied:
            raise Stop(f"{step['step_id']}: WatchDog denied further work on {part}")
        if self.held:
            raise Stop(f"{step['step_id']}: still holding {self.held}")
        geo = T.pick_geometry(step, self.w.pose(part))
        scaling = T.motion_scaling(step, DEFAULT_SCALING)
        self.w.fact("intent", verb="grasp", part=part)
        self.w.open_gripper(part)
        self.to_pose(f"{step['step_id']}:pre_grasp", geo["pre_grasp_tcp"], scaling)
        self.to_pose(f"{step['step_id']}:descend", geo["grasp_tcp"], scaling, straight=True)
        res = self.w.grasp(part, T.guarded_target_width(geo["width_m"]), T.grasp_force(self.parts, part))
        width = res.final_width_m
        if not res.ok and res.stop_reason not in ("force_threshold", "effort_plateau"):
            raise Stop(f"{step['step_id']}: grasp {res.stop_reason}: {res.detail}")
        if width is None or width <= T.GRASP_MIN_PLAUSIBLE_WIDTH_M:
            raise Stop(f"{step['step_id']}: gripper closed on nothing (width {width})")
        self.held = part
        self.as_grasped.pop(part, None)
        self.w.fact("hold", part=part, detail=f"width={width:.4f}")
        self.to_pose(f"{step['step_id']}:lift", geo["lift_tcp"], scaling, straight=True)
        self.w.fact("carry", part=part)

    def tcp_for_held(self, step: dict[str, Any], geo: dict[str, Any]) -> dict[str, Any]:
        """TCP target that puts the held part, as it sits in the hand now, on geo's object target.

        The first measurement after a pick is the part's pose as grasped; a later one
        that differs from it by more than SLIP_LIMIT_* means the part slipped or was
        knocked in the hand, and the episode stops rather than inserting it crooked."""
        part = str(step["object_id"])
        time.sleep(AT_REST_S)
        try:
            measured = T.in_hand(self.w.tcp_pose(), self.w.pose(part, max_age_s=0.5))
        except StepFailed as exc:
            log.warning("%s: in-hand pose not measured (%s); planned grasp used", step["step_id"], exc)
            self.w.fact("carry", part=part, detail=f"in hand: not measured ({exc}); planned grasp used")
            return geo["tcp"]
        planned = T.in_hand(geo["tcp"], geo["object_target"])
        grasped = self.as_grasped.setdefault(part, measured)
        d_plan, a_plan = T.pose_difference(planned, measured)
        d_slip, a_slip = T.pose_difference(grasped, measured)
        detail = (f"in hand: {d_plan * 1000:.1f} mm, {a_plan:.1f} deg from the planned grasp; "
                  f"{d_slip * 1000:.1f} mm, {a_slip:.1f} deg since the grasp")
        log.info("%s: %s %s", step["step_id"], part, detail)
        self.w.fact("carry", part=part, detail=detail)
        if d_slip > T.SLIP_LIMIT_M or a_slip > T.SLIP_LIMIT_DEG:
            raise Stop(f"{step['step_id']}: {part} moved in the hand since the grasp ({d_slip * 1000:.1f} mm, "
                       f"{a_slip:.1f} deg; limit {T.SLIP_LIMIT_M * 1000:.0f} mm, {T.SLIP_LIMIT_DEG:.0f} deg)")
        return T.tcp_for_object(geo["object_target"], measured)

    def place_approach(self, step: dict[str, Any]) -> None:
        part, frame = str(step["object_id"]), str(step["target_frame"])
        if self.held != part:
            raise Stop(f"{step['step_id']}: not holding {part} (holding {self.held})")
        scaling = T.motion_scaling(step, DEFAULT_SCALING)
        self.w.fact("intent", verb="carry", part=part, feature=frame)
        geo = T.place_geometry(step, self.w.pose(frame))
        self.to_pose(f"{step['step_id']}:hover", T.hover_pose(self.tcp_for_held(step, geo), step), scaling)
        geo = T.place_geometry(step, self.w.pose(frame))       # re-read: the frame may have moved
        self.to_pose(f"{step['step_id']}:approach", self.tcp_for_held(step, geo), scaling, straight=True)

    def place(self, step: dict[str, Any]) -> None:
        part, frame = str(step["object_id"]), str(step["target_frame"])
        if self.held != part:
            raise Stop(f"{step['step_id']}: not holding {part} (holding {self.held})")
        geo = T.place_geometry(step, self.w.pose(frame))
        scaling = T.motion_scaling(step, DEFAULT_SCALING)
        self.w.fact("intent", verb="insert", part=part, feature=frame)
        tcp = self.tcp_for_held(step, geo)
        self.to_pose(f"{step['step_id']}:insert", tcp, scaling, straight=True)
        self.w.fact("intent", verb="release", part=part, feature=frame)
        self.w.open_gripper(part)
        self.w.fact("release", part=part, feature=frame)
        self.held = None
        pick = T.pick_step_for(self.task, part) or {}
        retreat = T.retreat_pose(tcp, float(pick.get("approach_distance_m", T.DEFAULT_APPROACH_M)))
        self.to_pose(f"{step['step_id']}:retreat", retreat, scaling, straight=True)

    def run(self) -> dict[str, Any]:
        self.w.fact("intent", verb="move_to_safe_pose")
        self.move("safe_pose", lambda: self.w.plan_home(DEFAULT_SCALING, DEFAULT_SCALING))
        done = []
        for step in self.task["steps"]:
            kind = T.step_kind(step)
            log.info("step %s (%s)", step["step_id"], kind)
            getattr(self, kind)(step)
            done.append(step["step_id"])
        claim = f"all {len(done)} steps executed: {', '.join(done)}"
        self.w.fact("claim_done", detail=claim)
        return {"state": "done", "reason": claim}


def run_task(task_ref: str) -> dict[str, Any]:
    repo = Path(os.environ["CELL_REPO_ROOT"])
    path = Path(task_ref) if Path(task_ref).is_absolute() else repo / task_ref
    task = T.load_task(path)
    parts = (yaml.safe_load((repo / "config" / "parts" / "executor_parts.yaml").read_text(encoding="utf-8")) or {}).get("parts") or {}
    p = Planner(worker(), task, parts)
    try:
        return p.run()
    except (Stop, StepFailed) as exc:
        log.warning("stopped: %s", exc)
        return {"state": "stopped", "reason": str(exc)}
