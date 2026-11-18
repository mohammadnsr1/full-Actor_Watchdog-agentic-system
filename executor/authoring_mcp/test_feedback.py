import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from feedback import filter_verdict, needs_watchdog  # noqa: E402

VERDICT = {
    "episode_id": "e1", "outcome": "failure", "stage": "done",
    "planner": {"state": "done", "reason": "all steps", "outcome": "completed", "wall_s": 200.0, "sim_s": 80.0, "tools": ["execute_task"]},
    "checker": {"pass": False, "parts_placed": 1, "parts_required": 4},
    "watchdog": {"ran": True, "run_verdict": {"event_type": "run_verdict", "payload": {"verdict": "nonconformant", "explanation": "u_orange knocked"}}},
    "reset": {"verify_ok": True},
}


def test_watchdog_mode_hides_the_checker():
    out = filter_verdict(VERDICT, "watchdog")
    assert "checker" not in out and "checker_pass" not in out
    assert out["watchdog"]["verdict"] == "nonconformant"
    assert out["planner"]["state"] == "done" and "tools" not in out["planner"]
    assert "reset" not in out


def test_checker_mode_hides_the_watchdog():
    out = filter_verdict(VERDICT, "checker")
    assert "watchdog" not in out and out["checker_pass"] is False


def test_both():
    out = filter_verdict(VERDICT, "both")
    assert out["checker_pass"] is False and out["watchdog"]["verdict"] == "nonconformant"


def test_no_session_and_no_verdict_are_said_plainly():
    v = dict(VERDICT, watchdog={"ran": False})
    assert "no WatchDog session" in filter_verdict(v, "watchdog")["watchdog"]
    v = dict(VERDICT, watchdog={"ran": True, "run_verdict": None})
    assert "no run verdict" in filter_verdict(v, "watchdog")["watchdog"]


def test_invalid_episode():
    v = {"episode_id": "e2", "outcome": "invalid", "stage": "reset", "reason": "reset not verified"}
    out = filter_verdict(v, "checker")
    assert out["valid"] is False and out["invalid_reason"] == "reset not verified" and out["checker"] == "not run"


def test_unknown_mode():
    with pytest.raises(ValueError):
        filter_verdict(VERDICT, "everything")
    assert needs_watchdog("both") and not needs_watchdog("checker")
