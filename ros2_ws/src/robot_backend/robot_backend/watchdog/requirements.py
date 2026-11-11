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


def inspect_acceptance(
    recorder: Any,
    requirements: dict[str, Any],
    parts: dict[str, Any],
    max_age_s: float = 2.0,
) -> dict[str, Any]:
    """Compare each acceptance part's latest observed pose to its placement frame.

    Uses the recorder's buffered WatchDog poses (camera2 channel) for both the
    part and the frame. Returns a table with one row per part:
        status: pass | fail | missing_evidence
        position_error_m, orientation_error_deg, tolerances, evidence (source,
        sample ages), and a reason when evidence is missing.
    """
    from robot_backend.watchdog.timebase import wall_ns

    now = wall_ns()
    rows: dict[str, dict[str, Any]] = {}
    missing: list[str] = []
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
        if reasons:
            row["status"] = "missing_evidence"
            row["reason"] = "; ".join(reasons)
            missing.append(pid)
            rows[pid] = row
            continue
        dp = math.dist(_position(part_pose), _position(frame_pose))
        dq = _quat_angle_deg(part_pose, frame_pose)
        row["position_error_m"] = dp
        row["orientation_error_deg"] = dq
        row["status"] = "pass" if (dp <= float(pos_tol) and dq <= float(ori_tol)) else "fail"
        row["evidence"] = {
            "source": recorder.pose_source_name(),
            "camera": "camera2",
            "part_sample_wall_ns": int(part_pose["wall_ns"]),
            "frame_sample_wall_ns": int(frame_pose["wall_ns"]),
        }
        rows[pid] = row
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
