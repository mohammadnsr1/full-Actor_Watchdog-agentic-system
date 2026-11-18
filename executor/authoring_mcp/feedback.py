"""What the authoring agent may see of an episode: pure Python, Python 3.10.

config/cell.yaml `executor_feedback` decides which judge's result reaches the
agent that writes the planner:

    watchdog   the WatchDog's run verdict (the ground-truth checker stays the
               hidden evaluation of the study)
    checker    the simulator ground-truth checker
    both       both

Always visible: the planner's own completion (state, reason, outcome), the
budget use, and whether the episode was valid. Never visible: the manifest
(input hashes, versions), the WatchDog transcript, other sessions' episodes.
"""

from __future__ import annotations

from typing import Any

MODES = ("watchdog", "checker", "both")


def needs_watchdog(mode: str) -> bool:
    return mode in ("watchdog", "both")


def filter_verdict(verdict: dict[str, Any], mode: str) -> dict[str, Any]:
    if mode not in MODES:
        raise ValueError(f"executor_feedback must be one of {MODES}, not {mode!r}")
    p = verdict.get("planner") or {}
    out: dict[str, Any] = {
        "episode_id": verdict.get("episode_id"),
        "valid": verdict.get("outcome") != "invalid",
        "stage_reached": verdict.get("stage"),
        "planner": {k: p.get(k) for k in ("state", "reason", "outcome", "wall_s", "sim_s") if k in p},
        "feedback_mode": mode,
    }
    if verdict.get("outcome") == "invalid":
        out["invalid_reason"] = verdict.get("reason") or p.get("reason")
    if mode in ("checker", "both"):
        c = verdict.get("checker")
        out["checker"] = c if c is not None else "not run"
        if c is not None:
            out["checker_pass"] = bool(c.get("pass"))
    if needs_watchdog(mode):
        w = verdict.get("watchdog") or {}
        if not w.get("ran"):
            out["watchdog"] = "no WatchDog session ran for this episode"
        else:
            env = w.get("run_verdict")
            payload = (env or {}).get("payload") if isinstance(env, dict) else None
            out["watchdog"] = payload if payload is not None else "the WatchDog recorded no run verdict"
    return out
