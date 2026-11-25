"""Ground-truth acceptance checker (sim): pure Python, Python 3.10.

Judges the end state of an episode from simulator truth (the command server's
report_poses), independently of both the planner's claim and the WatchDog:

    parts       each acceptance part (config/requirements/<assembly>.yaml
                acceptance.parts) is at its placement frame within the part's
                registry tolerances (position_m, orientation_deg); the runner
                derives the placement frames from the fixture's reported pose
                (config/placements/, cell_platform_interfaces.placements).
                A part counts as at a frame up to its own proper symmetries
                (config/placements/ `symmetries`, from fmb1's proved
                certificates): at frame (x) S it occupies what it occupies at
                the frame. Parts listed together in acceptance.interchangeable
                (identical parts) may fill each other's frames, one part per
                frame; the assignment that seats the most parts is judged.
    fixtures    acceptance.fixtures did not move (disturbance tolerance)
    bystanders  bystanders did not move (their disturbance_tolerance_m)

Tolerances come from config/parts/watchdog_parts.yaml, which is generated
from the registry (the same numbers both roles' part files carry). A pose the
scene did not report is `missing`, which fails the check.
"""

from __future__ import annotations

import itertools
import math
from typing import Any

DEFAULT_DISTURBANCE_M = 0.005


def pose_error(a: dict[str, Any], b: dict[str, Any]) -> tuple[float, float]:
    """(position error m, orientation error deg) between two report_poses entries."""
    dp = math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a["position"], b["position"])))
    qa, qb = a.get("orientation_xyzw", [0, 0, 0, 1]), b.get("orientation_xyzw", [0, 0, 0, 1])
    dot = abs(sum(float(x) * float(y) for x, y in zip(qa, qb)))
    return dp, math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def _qmul(a: Any, b: Any) -> list[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz]


def _compose(frame: dict[str, Any], t: Any, q: Any) -> dict[str, Any]:
    """frame (x) (t, q), t and q in the frame's own axes."""
    fq = [float(v) for v in frame.get("orientation_xyzw", [0, 0, 0, 1])]
    x, y, z, _ = _qmul(_qmul(fq, [float(t[0]), float(t[1]), float(t[2]), 0.0]), [-fq[0], -fq[1], -fq[2], fq[3]])
    p = [float(v) for v in frame["position"]]
    return {"position": [p[0] + x, p[1] + y, p[2] + z], "orientation_xyzw": _qmul(fq, [float(v) for v in q])}


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


def _judge(pid: str, frame: str, parts: dict[str, Any], after: dict[str, Any],
           symmetries: dict[str, Any]) -> dict[str, Any]:
    """The part against one frame, at the symmetry that fits it best."""
    if pid not in after or frame not in after:
        return {"status": "missing", "placement_frame": frame}
    tol = _tol(parts, pid)
    pos_tol, ori_tol = tol.get("position_m"), tol.get("orientation_deg")
    best = None
    for i, (t, q) in enumerate([((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))] + list(symmetries.get(pid) or [])):
        dp, dq = pose_error(after[pid], _compose(after[frame], t, q))
        score = max(dp / float(pos_tol) if pos_tol else dp, dq / float(ori_tol) if ori_tol else 0.0)
        if best is None or score < best[0]:
            best = (score, dp, dq, i)
    _, dp, dq, sym = best
    out = {"placement_frame": frame, "position_error_m": round(dp, 5), "orientation_error_deg": round(dq, 3), "symmetry": sym}
    if pos_tol is None:
        return {"status": "no_tolerance", **out}
    ok = dp <= float(pos_tol) and (ori_tol is None or dq <= float(ori_tol))
    return {"status": "pass" if ok else "fail", **out, "tolerance": {"position_m": pos_tol, "orientation_deg": ori_tol}}


def _score(results: list[dict[str, Any]]) -> tuple[int, float]:
    return (sum(r["status"] == "pass" for r in results),
            -sum(r.get("position_error_m", 1e9) for r in results))


def check(requirements: dict[str, Any], parts: dict[str, Any], before: dict[str, Any], after: dict[str, Any],
          symmetries: dict[str, Any] | None = None) -> dict[str, Any]:
    acc = requirements.get("acceptance") or {}
    symmetries = symmetries or {}
    own = {pid: str((spec or {}).get("placement_frame", "")) for pid, spec in (acc.get("parts") or {}).items()}
    groups = [[pid for pid in g if pid in own] for g in (acc.get("interchangeable") or [])]
    grouped = {pid for g in groups for pid in g}
    groups += [[pid] for pid in own if pid not in grouped]
    placed: dict[str, Any] = {}
    for group in groups:
        frames = [own[pid] for pid in group]
        best = None
        for perm in itertools.permutations(frames):
            results = [_judge(pid, f, parts, after, symmetries) for pid, f in zip(group, perm)]
            if best is None or _score(results) > _score(best):
                best = results
        for pid, r in zip(group, best):
            if r["placement_frame"] != own[pid]:
                r["swapped_from"] = own[pid]
            placed[pid] = r
    placed = {pid: placed[pid] for pid in own}
    fixtures = _unmoved(list(acc.get("fixtures") or []), parts, before, after)
    bystanders = _unmoved(list(requirements.get("bystanders") or []), parts, before, after)
    all_groups = (placed, fixtures, bystanders)
    passed = all(v["status"] == "pass" for g in all_groups for v in g.values())
    return {
        "pass": passed,
        "parts_placed": sum(1 for v in placed.values() if v["status"] == "pass"),
        "parts_required": len(placed),
        "parts": placed,
        "fixtures": fixtures,
        "bystanders": bystanders,
    }
