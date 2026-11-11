"""Seeded layouts: pure Python, no Kit, Python 3.10 safe (the episode runner
imports it too).

Layout file (config/layouts/<assembly>/<name>.yaml):

    assembly: fmb_assembly_2
    name: nominal
    frame: world
    verify_tolerances: {position_m: <TODO>, orientation_deg: <TODO>}
    parts:
      u_brown:
        prim: /World/u_brown
        position: [x, y, z]
        orientation_xyzw: [x, y, z, w]
        jitter: {xy_m: 0.0, yaw_deg: 0.0}     # per-part seeded variation ranges (uniform, +/-)
    fixtures: [...]            # placed but never jittered
    robot:
      home_joint_positions: {joint: rad, ...}
      gripper_open_position: <rad>

A seed turns the jitter ranges into deterministic offsets; seed 0 or a
missing jitter block means the nominal placement. Values come from
tools/capture_layout.py (report_poses) or from Mohammad; nothing here
invents a pose.
"""

from __future__ import annotations

import math
import random
from typing import Any


def _yaw_quat_xyzw(yaw_rad: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def _quat_mul_xyzw(a: Any, b: Any) -> list[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def seeded_placements(layout: dict[str, Any], seed: int) -> dict[str, dict[str, Any]]:
    """part id -> {prim, position, orientation_xyzw, jittered: bool}."""
    rng = random.Random(int(seed))
    out: dict[str, dict[str, Any]] = {}
    for pid, entry in (layout.get("parts") or {}).items():
        pos = [float(v) for v in entry["position"]]
        quat = [float(v) for v in entry.get("orientation_xyzw", [0, 0, 0, 1])]
        jitter = entry.get("jitter") or {}
        xy = float(jitter.get("xy_m", 0.0) or 0.0)
        yaw = float(jitter.get("yaw_deg", 0.0) or 0.0)
        jittered = False
        if seed and (xy > 0 or yaw > 0):
            dx = rng.uniform(-xy, xy) if xy > 0 else 0.0
            dy = rng.uniform(-xy, xy) if xy > 0 else 0.0
            dyaw = math.radians(rng.uniform(-yaw, yaw)) if yaw > 0 else 0.0
            pos = [pos[0] + dx, pos[1] + dy, pos[2]]
            quat = _quat_mul_xyzw(_yaw_quat_xyzw(dyaw), quat)
            jittered = True
        out[pid] = {"prim": str(entry["prim"]), "position": pos, "orientation_xyzw": quat, "jittered": jittered}
    return out


def validate_layout(layout: dict[str, Any]) -> list[str]:
    problems = []
    parts = layout.get("parts") or {}
    if not parts:
        problems.append("no parts")
    for pid, entry in parts.items():
        if not isinstance(entry, dict) or "prim" not in entry or "position" not in entry:
            problems.append(f"{pid}: needs prim and position")
            continue
        if len(entry["position"]) != 3:
            problems.append(f"{pid}: position must have 3 values")
        if "orientation_xyzw" in entry and len(entry["orientation_xyzw"]) != 4:
            problems.append(f"{pid}: orientation_xyzw must have 4 values")
    tol = layout.get("verify_tolerances") or {}
    if not tol.get("position_m"):
        problems.append("verify_tolerances.position_m is unset (TODO(mohammad))")
    return problems


def pose_error(a: dict[str, Any], b: dict[str, Any]) -> tuple[float, float]:
    """(position error m, orientation error deg) between two {position, orientation_xyzw}."""
    pa, pb = a["position"], b["position"]
    dp = math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(pa, pb)))
    qa, qb = a.get("orientation_xyzw", [0, 0, 0, 1]), b.get("orientation_xyzw", [0, 0, 0, 1])
    dot = abs(sum(float(x) * float(y) for x, y in zip(qa, qb)))
    return dp, math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))
