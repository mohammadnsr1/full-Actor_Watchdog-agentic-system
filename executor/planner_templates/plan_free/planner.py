"""Plan-free planner skeleton: the contract plumbing, no assembly logic.

execute_task(task_ref) lands in run_task() below. The task file names the
goal and constraints (instruction_spec) and the objects (object_ids); it gives
no steps, no sequence of motions, no grasp poses and no insertion offsets.
Working out how to do the assembly is the planner's job.

What is already here (docs/PLANNER_CONTRACT.md):

    Planner.run()      the safe joint pose; then nothing: it returns
                       "stopped" until assembly logic is written
                       (execute_task is called only once the episode runner
                       has the start permission, so there is no gate here)
    Planner.move()     run a planned motion; a WatchDog halt (a cancelled
                       trajectory) is handled per watchdog.consult: stop, or
                       wait for the ruling and re-plan when allowed
    Planner.to_pose()  plan (free-space or straight-line) and move to a TCP pose

Primitives (worker.py, the ROS side; every name from the platform config):

    w.pose(id)                 latest world pose of a part or a placement
                               frame from the executor camera (camera1)
    w.plan_to_pose(p, v, a)    free-space plan to a TCP pose (planning service)
    w.plan_straight(p, v, a)   straight-line Cartesian plan to a TCP pose
    w.plan_home(v, a)          plan to the platform's safe joint pose
    w.execute(traj, label)     execute on the arm; raises Halted when cancelled
    w.open_gripper(part)       open fully
    w.grasp(part, width, f)    guarded close to `width` under force `f`;
                               result .ok .stop_reason .final_width_m .detail
    w.fact(kind, ...)          executor fact journal (intent, hold, carry,
                               release, claim_done), recorded only
    w.consultations(since)     WatchDog consultations so far (watchdog.consult
                               on): free-form text about what it observed
                               (repeated jams, a part off its frame, a risk)

Geometry (geometry.py): compose, along_own_z, raised, tcp_for (RG2 grasp
centre + jaw width -> TCP pose), guarded_target_width, grasp_force.

Object data: config/parts/executor_parts.yaml (collision shape, dimensions,
pose offset, grasp force threshold per part), loaded into self.parts.

Every file in this directory is yours to change, worker.py and geometry.py
included.
"""

from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Any

import yaml

import geometry as G  # noqa: F401  (for the assembly logic you write)
from worker import Halted, StepFailed, Worker

log = logging.getLogger("planner")

DEFAULT_SCALING = 0.2
RULING_WAIT_S = 120.0

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
        self.denied: set[str] = set()

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

    def to_pose(self, label: str, pose: dict[str, Any], vel: float = DEFAULT_SCALING, acc: float = DEFAULT_SCALING,
                straight: bool = False) -> None:
        self.move(label, (lambda: self.w.plan_straight(pose, vel, acc)) if straight else (lambda: self.w.plan_to_pose(pose, vel, acc)))

    def run(self) -> dict[str, Any]:
        self.w.fact("intent", verb="move_to_safe_pose")
        self.move("safe_pose", lambda: self.w.plan_home(DEFAULT_SCALING, DEFAULT_SCALING))
        # The assembly goes here.
        return {"state": "stopped", "reason": "plan-free skeleton: no assembly logic written yet"}


def run_task(task_ref: str) -> dict[str, Any]:
    repo = Path(os.environ["CELL_REPO_ROOT"])
    path = Path(task_ref) if Path(task_ref).is_absolute() else repo / task_ref
    task = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    parts = (yaml.safe_load((repo / "config" / "parts" / "executor_parts.yaml").read_text(encoding="utf-8")) or {}).get("parts") or {}
    p = Planner(worker(), task, parts)
    try:
        return p.run()
    except (Stop, StepFailed) as exc:
        log.warning("stopped: %s", exc)
        return {"state": "stopped", "reason": str(exc)}
