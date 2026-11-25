"""Executor-baseline planner scaffold (EB6): the delivery plumbing, no assembly logic.

execute_task(task_ref) lands in run_task(). task_ref is the task id: its record
is $CELL_TASK_DATA/tasks/<task_ref>/task.yaml, next to the environment
declaration, the robot composite and the assembly (`assemblies/<name>/`) it
names. This scaffold reads the task and stops with `inadmissible_state`: it
does not move the robot. Everything in this directory is yours to change.

What the cell gives a planner (docs/PLANNER_CONTRACT.md):

    worker.Worker(platform, repo)   the ROS side, every name from the platform config
      w.pose(id) / w.tcp_pose() / w.camera1()      sensing (camera1 poses, TF, wrist images)
      w.plan_to_pose / plan_straight / plan_home    the planning service (plan only)
      w.execute(traj, label)                        the arm (FollowJointTrajectory)
      w.move_joints(goal) / w.jog(delta, frame)     direct motion, no planning, no collision checks
      w.open_gripper() / w.grasp(part, width, f)    the RG2 controller
      w.fact(kind, ...)                             the fact journal
    geometry.py   compose, along_own_z, raised, tcp_for (RG2 calibration), joint_path
"""

from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Any

import yaml

log = logging.getLogger("planner")


def load_task(task_ref: str) -> dict[str, Any]:
    root = Path(os.environ.get("CELL_TASK_DATA", ""))
    path = Path(task_ref) if task_ref.endswith(".yaml") else root / "tasks" / task_ref / "task.yaml"
    if not path.is_file():
        raise FileNotFoundError(f"no task record at {path} (CELL_TASK_DATA={root})")
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def run_task(task_ref: str) -> dict[str, Any]:
    task = load_task(task_ref)
    log.info("task %s: assembly %s, environment %s", task.get("task_id"), task.get("assembly"), task.get("environment"))
    return {"state": "stopped", "reason": "inadmissible_state: the baseline scaffold has no assembly logic yet"}
