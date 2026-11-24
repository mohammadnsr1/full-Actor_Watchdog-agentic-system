"""Ground-truth acceptance checker (sim): pure Python, Python 3.10.

Judges the end state of an episode from simulator truth (the command server's
report_poses), independently of both the planner's claim and the WatchDog:

    parts       each acceptance part (config/requirements/<assembly>.yaml
                acceptance.parts) is at its placement frame within the part's
                registry tolerances (position_m, orientation_deg); the runner
                derives the placement frames from the fixture's reported pose
                (config/placements/, cell_platform_interfaces.placements)
    fixtures    acceptance.fixtures did not move (disturbance tolerance)
    bystanders  bystanders did not move (their disturbance_tolerance_m)

Tolerances come from config/parts/watchdog_parts.yaml, which is generated
from the registry (the same numbers both roles' part files carry). A pose the
scene did not report is `missing`, which fails the check.
"""

from __future__ import annotations

import math
from typing import Any

DEFAULT_DISTURBANCE_M = 0.005


def pose_error(a: dict[str, Any], b: dict[str, Any]) -> tuple[float, float]:
    """(position error m, orientation error deg) between two report_poses entries."""
    dp = math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a["position"], b["position"])))
    qa, qb = a.get("orientation_xyzw", [0, 0, 0, 1]), b.get("orientation_xyzw", [0, 0, 0, 1])
    dot = abs(sum(float(x) * float(y) for x, y in zip(qa, qb)))
    return dp, math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def _tol(parts: dict[str, Any], pid: str) -> dict[str, Any]:
    return ((parts.get(pid) or {}).get("tolerances") or {})


def _unmoved(ids: list[str], parts: dict[str, Any], before: dict[str, Any], after: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for pid in ids:
        if pid not in before or pid not in after:
            out[pid] = {"status": "missing"}
            continue
        dp, dq = pose_error(before[pid], after[pid])
        tol = float(_tol(parts, pid).get("disturbance_tolerance_m", DEFAULT_DISTURBANCE_M) or DEFAULT_DISTURBANCE_M)
        out[pid] = {"status": "pass" if dp <= tol else "fail", "moved_m": round(dp, 5), "rotated_deg": round(dq, 3), "tolerance_m": tol}
    return out


def check(requirements: dict[str, Any], parts: dict[str, Any], before: dict[str, Any], after: dict[str, Any]) -> dict[str, Any]:
    acc = requirements.get("acceptance") or {}
    placed: dict[str, Any] = {}
    for pid, spec in (acc.get("parts") or {}).items():
        frame = str((spec or {}).get("placement_frame", ""))
        if pid not in after or frame not in after:
            placed[pid] = {"status": "missing", "placement_frame": frame}
            continue
        dp, dq = pose_error(after[pid], after[frame])
        tol = _tol(parts, pid)
        pos_tol, ori_tol = tol.get("position_m"), tol.get("orientation_deg")
        if pos_tol is None:
            placed[pid] = {"status": "no_tolerance", "placement_frame": frame, "position_error_m": round(dp, 5), "orientation_error_deg": round(dq, 3)}
            continue
        ok = dp <= float(pos_tol) and (ori_tol is None or dq <= float(ori_tol))
        placed[pid] = {"status": "pass" if ok else "fail", "placement_frame": frame, "position_error_m": round(dp, 5),
                       "orientation_error_deg": round(dq, 3), "tolerance": {"position_m": pos_tol, "orientation_deg": ori_tol}}
    fixtures = _unmoved(list(acc.get("fixtures") or []), parts, before, after)
    bystanders = _unmoved(list(requirements.get("bystanders") or []), parts, before, after)
    groups = (placed, fixtures, bystanders)
    passed = all(v["status"] == "pass" for g in groups for v in g.values())
    return {
        "pass": passed,
        "parts_placed": sum(1 for v in placed.values() if v["status"] == "pass"),
        "parts_required": len(placed),
        "parts": placed,
        "fixtures": fixtures,
        "bystanders": bystanders,
    }
