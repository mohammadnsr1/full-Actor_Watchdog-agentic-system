"""The WatchDog's requirements file and part data (plan-blind, Milestone 5).

WATCHDOG_TASK_PATH names a requirements file (config/requirements/<assembly>.yaml):
acceptance contracts (per-part placement frame and grasp width) plus the
invariants in prose. It carries no steps. Part tolerances and limits come
from config/parts/watchdog_parts.yaml (the WatchDog-only split of the
registry), with limits chosen per platform (limits.sim / limits.hardware).

The acceptance inspection compares each part's latest WatchDog pose
observation (camera2 channel) with its placement frame's pose, within the
part's tolerances. A part or frame with no observation is MISSING EVIDENCE,
reported as such, never judged.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any

import yaml

from robot_backend.cell_config import find_cell_config, platform as _platform

TASK_PATH_ENV = "WATCHDOG_TASK_PATH"


def repo_root() -> Path:
    cfg = find_cell_config()
    if cfg is None:
        raise RuntimeError("config/cell.yaml not found; cannot locate the repo root")
    return cfg.parent.parent


def requirements_path() -> Path:
    raw = os.environ.get(TASK_PATH_ENV, "").strip()
    if not raw:
        raise RuntimeError(f"{TASK_PATH_ENV} is not set; it must name config/requirements/<assembly>.yaml")
    path = Path(os.path.expanduser(raw))
    if not path.is_file():
        raise RuntimeError(f"{TASK_PATH_ENV}={raw!r} does not name a file")
    return path


def load_requirements(path: Path | None = None) -> dict[str, Any]:
    path = path or requirements_path()
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must be a mapping")
    if "steps" in data or "step_plan" in data:
        raise ValueError(
            f"{path} carries a steps/step_plan block; the WatchDog requirements file must not "
            "(plan knowledge is not allowed to reach this process)"
        )
    acceptance = data.get("acceptance") or {}
    parts = acceptance.get("parts") if isinstance(acceptance, dict) else None
    if not isinstance(parts, dict) or not parts:
        raise ValueError(f"{path} has no acceptance.parts block")
    for pid, entry in parts.items():
        if not isinstance(entry, dict) or "grasp_width_m" not in entry:
            raise ValueError(f"{path}: acceptance.parts.{pid} needs grasp_width_m")
    data["_path"] = str(path)
    return data


def acceptance_parts(requirements: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return dict(requirements["acceptance"]["parts"])


def part_spec_for_contract(requirements: dict[str, Any], object_id: str) -> dict[str, Any]:
    """The step_spec-shaped dict load_pick_contract expects, from the requirements file."""
    parts = acceptance_parts(requirements)
    if object_id not in parts:
        raise ValueError(
            f"object_id {object_id!r} has no acceptance contract in the requirements file. "
            f"Parts with a contract: {sorted(parts)}"
        )
    entry = parts[object_id]
    return {
        "object_id": object_id,
        "grasp_width_m": float(entry["grasp_width_m"]),
        "contract_clause_groups": list(entry.get("contract_clause_groups", ["effects", "invariants"])),
        "placement_frame": entry.get("placement_frame"),
    }


def load_watchdog_parts(path: Path | None = None) -> dict[str, Any]:
    path = path or (repo_root() / "config" / "parts" / "watchdog_parts.yaml")
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    data["_path"] = str(path)
    return data


def part_limits(parts: dict[str, Any], object_id: str) -> dict[str, Any]:
    """limits for this platform (limits.sim or limits.hardware); None values are TODO."""
    entry = (parts.get("parts") or {}).get(object_id) or {}
    limits = entry.get("limits") or {}
    return dict(limits.get(_platform().name) or {})


def part_tolerances(parts: dict[str, Any], object_id: str) -> dict[str, Any]:
    entry = (parts.get("parts") or {}).get(object_id) or {}
    return dict(entry.get("tolerances") or {})


# -- acceptance inspection -----------------------------------------------------


def _position(sample: dict[str, Any]) -> tuple[float, float, float]:
    p = sample["pose"]["position"]
    return (float(p["x"]), float(p["y"]), float(p["z"]))


def _quat_angle_deg(a: dict[str, Any], b: dict[str, Any]) -> float:
    qa = a["pose"]["orientation"]
    qb = b["pose"]["orientation"]
    dot = abs(qa["x"] * qb["x"] + qa["y"] * qb["y"] + qa["z"] * qb["z"] + qa["w"] * qb["w"])
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def _xyzw(sample: dict[str, Any]) -> list[float]:
    q = sample["pose"]["orientation"]
    return [float(q["x"]), float(q["y"]), float(q["z"]), float(q["w"])]


def _qmul(a: list[float], b: list[float]) -> list[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz]


def _seat_error(part_pose: dict[str, Any], frame_pose: dict[str, Any], symmetries: list[Any],
                pos_tol: float, ori_tol: float) -> tuple[float, float, int]:
    """(position m, orientation deg, symmetry index) of the part against the frame, at the part's
    best-fitting proper symmetry S (frame (x) S; index 0 is the identity)."""
    fp, fq = _position(frame_pose), _xyzw(frame_pose)
    pp, pq = _position(part_pose), _xyzw(part_pose)
    best = None
    for i, (t, q) in enumerate([((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))] + list(symmetries)):
        x, y, z, _ = _qmul(_qmul(fq, [float(t[0]), float(t[1]), float(t[2]), 0.0]), [-fq[0], -fq[1], -fq[2], fq[3]])
        tq = _qmul(fq, [float(v) for v in q])
        dp = math.dist(pp, (fp[0] + x, fp[1] + y, fp[2] + z))
        dot = min(1.0, abs(sum(a * b for a, b in zip(pq, tq))) / (math.sqrt(sum(a * a for a in pq)) * math.sqrt(sum(b * b for b in tq))))
        dq = math.degrees(2.0 * math.acos(dot))
        score = max(dp / pos_tol, dq / ori_tol)
        if best is None or score < best[0]:
            best = (score, dp, dq, i)
    return best[1], best[2], best[3]


def _symmetries() -> dict[str, Any]:
    try:
        from cell_platform_interfaces.placements import load_symmetries

        return load_symmetries(repo_root())
    except Exception:  # noqa: BLE001 - no symmetries file means identity only
        return {}


def inspect_acceptance(
    recorder: Any,
    requirements: dict[str, Any],
    parts: dict[str, Any],
    max_age_s: float = 2.0,
    symmetries: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Compare each acceptance part's latest observed pose to its placement frame.

    Uses the recorder's buffered WatchDog poses (camera2 channel) for both the
    part and the frame. A part counts as at a frame up to its own proper
    symmetries (config/placements/ `symmetries`), and parts listed together in
    acceptance.interchangeable may fill each other's frames, one per frame (the
    assignment seating the most parts is reported; the same rule as the
    runner's ground-truth checker). Returns a table with one row per part:
        status: pass | fail | missing_evidence
        position_error_m, orientation_error_deg, symmetry, placement_frame (the
        frame judged; swapped_from when it is another part's), tolerances,
        evidence (source, sample ages), and a reason when evidence is missing.
    """
    import itertools

    from robot_backend.watchdog.timebase import wall_ns

    now = wall_ns()
    symmetries = _symmetries() if symmetries is None else symmetries
    rows: dict[str, dict[str, Any]] = {}
    missing: list[str] = []
    ready: dict[str, tuple[dict[str, Any], float, float]] = {}
    own: dict[str, str] = {}
    for pid, entry in acceptance_parts(requirements).items():
        frame_id = entry.get("placement_frame")
        tol = part_tolerances(parts, pid)
        pos_tol = tol.get("position_m")
        ori_tol = tol.get("orientation_deg")
        row: dict[str, Any] = {"part": pid, "placement_frame": frame_id, "tolerances": {"position_m": pos_tol, "orientation_deg": ori_tol}}
        part_pose = recorder.latest_object_pose(pid)
        frame_pose = recorder.latest_object_pose(frame_id) if frame_id else None
        reasons = []
        if not frame_id:
            reasons.append("no placement_frame in the requirements file")
        if part_pose is None:
            reasons.append(f"no observed pose for {pid}")
        elif now - int(part_pose["wall_ns"]) > max_age_s * 1e9:
            reasons.append(f"latest pose of {pid} is older than {max_age_s}s")
        if frame_id and frame_pose is None:
            reasons.append(f"no observed pose for frame {frame_id}")
        if pos_tol is None or ori_tol is None:
            reasons.append(f"tolerances for {pid} are missing in watchdog_parts.yaml")
        rows[pid] = row
        if reasons:
            row["status"] = "missing_evidence"
            row["reason"] = "; ".join(reasons)
            missing.append(pid)
            continue
        ready[pid] = (part_pose, float(pos_tol), float(ori_tol))
        own[pid] = str(frame_id)
    groups = [[pid for pid in g if pid in ready] for g in ((requirements.get("acceptance") or {}).get("interchangeable") or [])]
    grouped = {pid for g in groups for pid in g}
    groups = [g for g in groups if g] + [[pid] for pid in ready if pid not in grouped]
    for group in groups:
        best = None
        for perm in itertools.permutations([own[pid] for pid in group]):
            judged = []
            for pid, frame_id in zip(group, perm):
                part_pose, pos_tol, ori_tol = ready[pid]
                frame_pose = recorder.latest_object_pose(frame_id)
                dp, dq, sym = _seat_error(part_pose, frame_pose, symmetries.get(pid) or [], pos_tol, ori_tol)
                judged.append((pid, frame_id, frame_pose, dp, dq, sym, dp <= pos_tol and dq <= ori_tol))
            score = (sum(j[6] for j in judged), -sum(j[3] for j in judged))
            if best is None or score > best[0]:
                best = (score, judged)
        for pid, frame_id, frame_pose, dp, dq, sym, ok in best[1]:
            row = rows[pid]
            part_pose = ready[pid][0]
            row["placement_frame"] = frame_id
            if frame_id != own[pid]:
                row["swapped_from"] = own[pid]
            row["position_error_m"] = dp
            row["orientation_error_deg"] = dq
            row["symmetry"] = sym
            row["status"] = "pass" if ok else "fail"
            row["evidence"] = {
                "source": recorder.pose_source_name(),
                "camera": "camera2",
                "part_sample_wall_ns": int(part_pose["wall_ns"]),
                "frame_sample_wall_ns": int(frame_pose["wall_ns"]),
            }
    statuses = [r["status"] for r in rows.values()]
    overall = "pass" if statuses and all(s == "pass" for s in statuses) else ("fail" if "fail" in statuses else "missing_evidence")
    return {
        "overall": overall,
        "parts": rows,
        "missing_evidence": missing,
        "evidence_sources": [recorder.pose_source_name()],
        "shared_source": False,
        "wall_ns": now,
    }
