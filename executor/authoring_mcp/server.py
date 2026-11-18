"""Authoring MCP server: the tools an agent uses to write a planner.

    get_task()                         the task, the planner contract, where to edit
    evaluate_planner(episodes, seed)   freeze a snapshot of the editable planner,
                                       run fresh episodes with it, return feedback
    read_verdict(episode_id)           the feedback for one of this session's episodes
    submit_work(state, report)         end the session with the final planner

One authoring session = one directory under executor/workspace/<session>/
(AUTHORING_SESSION_DIR, created by runner/run_authoring.sh):

    planner/            the editable planner (a copy of executor/planner_template)
    snapshots/NNN/      frozen copy evaluated by evaluate_planner call NNN
    evals/              runner logs per episode
    session.json        every call, every episode, the submission

What the agent learns about an episode is filtered by config/cell.yaml
`executor_feedback` (feedback.py). Episodes run through
runner/episode_runner/run.sh, one at a time on the cell. The idea of an
authoring loop with frozen snapshots and submit_work is credited to
DROIDSTACK in docs/PROVENANCE.md; nothing is copied from it.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

HERE = Path(__file__).resolve().parent
REPO = Path(os.environ.get("CELL_REPO_ROOT") or HERE.parents[1])
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO))

import anyio  # noqa: E402
from mcp.server.fastmcp import FastMCP  # noqa: E402

from feedback import MODES, filter_verdict, needs_watchdog  # noqa: E402
from runner.episode_runner.manifest import tree_hash  # noqa: E402

MAX_EPISODES_PER_CALL = 3

mcp = FastMCP("executor-authoring")


def _utc() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


def _cell() -> dict[str, Any]:
    return yaml.safe_load((REPO / "config" / "cell.yaml").read_text()) or {}


def _feedback_mode() -> str:
    mode = os.environ.get("AUTHORING_FEEDBACK") or str(_cell().get("executor_feedback", "watchdog"))
    if mode not in MODES:
        raise ValueError(f"executor_feedback must be one of {MODES}, not {mode!r}")
    return mode


class Session:
    def __init__(self) -> None:
        d = os.environ.get("AUTHORING_SESSION_DIR")
        if not d:
            raise RuntimeError("AUTHORING_SESSION_DIR is not set (start the session with runner/run_authoring.sh)")
        self.dir = Path(d).resolve()
        self.planner = self.dir / "planner"
        self.task = Path(os.environ.get("AUTHORING_TASK", "config/tasks/fmb_assembly_2.yaml"))
        self.task = self.task if self.task.is_absolute() else REPO / self.task
        self.assembly = os.environ.get("AUTHORING_ASSEMBLY", "fmb_assembly_2")
        self.platform = os.environ.get("CELL_PLATFORM", "sim")
        self.state_path = self.dir / "session.json"
        if not self.planner.is_dir():
            raise RuntimeError(f"{self.planner} missing: run_authoring.sh copies the template there")
        self.state = json.loads(self.state_path.read_text()) if self.state_path.is_file() else {
            "session": self.dir.name, "started_wall_time_iso": _utc(), "task": str(self.task), "platform": self.platform,
            "evaluations": [], "submitted": None}

    def save(self) -> None:
        self.state_path.write_text(json.dumps(self.state, indent=2))

    def episode_ids(self) -> list[str]:
        return [e["episode_id"] for ev in self.state["evaluations"] for e in ev["episodes"]]


_S: Session | None = None


def session() -> Session:
    global _S
    if _S is None:
        _S = Session()
    return _S


@mcp.tool()
def get_task() -> dict[str, Any]:
    """The task to write a planner for, the planner contract, and where to edit.
    Call first."""
    s = session()
    return {
        "task_ref": str(s.task.relative_to(REPO)) if s.task.is_relative_to(REPO) else str(s.task),
        "task": s.task.read_text(),
        "planner_contract": (REPO / "docs" / "PLANNER_CONTRACT.md").read_text(),
        "editable_planner_dir": str(s.planner),
        "how_to": (
            "Edit the planner in editable_planner_dir (it starts as a copy of executor/planner_template; "
            "run.sh, server.py, planner.py, worker.py, task.py). evaluate_planner freezes a snapshot of that "
            "directory and runs fresh episodes with it on the cell; read the returned feedback (or read_verdict) "
            "and iterate. Executor-side data you may read: config/tasks/, config/parts/executor_parts.yaml, "
            "config/platforms/, the cell_platform_interfaces and cell_backend_* sources. When the planner is "
            "as good as you can make it, call submit_work."
        ),
        "feedback_mode": _feedback_mode(),
        "budgets": _cell().get("budgets"),
        "platform": s.platform,
        "evaluations_so_far": len(s.state["evaluations"]),
        "submitted": s.state["submitted"] is not None,
    }


def _run_episode(s: Session, snapshot: Path, episode_id: str, seed: int, watchdog: str) -> dict[str, Any]:
    log_path = s.dir / "evals" / f"{episode_id}.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = [str(REPO / "runner" / "episode_runner" / "run.sh"), "--planner", str(snapshot), "--task", str(s.task),
           "--assembly", s.assembly, "--seed", str(seed), "--platform", s.platform, "--episode-id", episode_id,
           "--watchdog", watchdog]
    env = {k: v for k, v in os.environ.items() if not k.startswith(("AUTHORING_", "VIRTUAL_ENV"))}
    started = time.monotonic()
    with log_path.open("w") as log:
        rc = subprocess.run(cmd, cwd=str(REPO), env=env, stdout=log, stderr=subprocess.STDOUT).returncode
    verdict_path = REPO / "records" / episode_id / "verdict.json"
    verdict = json.loads(verdict_path.read_text()) if verdict_path.is_file() else {
        "episode_id": episode_id, "outcome": "invalid", "stage": "runner", "reason": f"runner exited {rc} without a verdict (see its log)"}
    return {"episode_id": episode_id, "runner_exit": rc, "wall_s": round(time.monotonic() - started, 1), "verdict": verdict}


@mcp.tool()
async def evaluate_planner(episodes: int = 1, seed: int = 0) -> dict[str, Any]:
    """Freeze a snapshot of the editable planner and run `episodes` fresh episodes
    with it (seeds seed, seed+1, ...). Each episode resets the scene, runs
    execute_task once and is judged. Takes minutes per episode. Returns the
    feedback for each episode."""
    s = session()
    if s.state["submitted"] is not None:
        return {"error": "work already submitted; this session is closed"}
    if not 1 <= int(episodes) <= MAX_EPISODES_PER_CALL:
        return {"error": f"episodes must be 1..{MAX_EPISODES_PER_CALL}"}
    mode = _feedback_mode()
    n = len(s.state["evaluations"]) + 1
    snapshot = s.dir / "snapshots" / f"{n:03d}"
    shutil.copytree(s.planner, snapshot, ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".pytest_cache"))
    run_sh = snapshot / "run.sh"
    if run_sh.is_file():
        run_sh.chmod(run_sh.stat().st_mode | 0o111)
    entry: dict[str, Any] = {"n": n, "snapshot": str(snapshot), "planner_sha256": tree_hash(snapshot)["sha256"],
                             "started_wall_time_iso": _utc(), "seeds": [], "episodes": []}
    watchdog = "plan_blind" if needs_watchdog(mode) else "none"
    feedback = []
    for i in range(int(episodes)):
        sd = int(seed) + i
        episode_id = f"{s.dir.name}-e{n:03d}-{i + 1}-s{sd}"
        result = await anyio.to_thread.run_sync(_run_episode, s, snapshot, episode_id, sd, watchdog)
        entry["seeds"].append(sd)
        entry["episodes"].append({"episode_id": episode_id, "outcome": result["verdict"].get("outcome"), "runner_exit": result["runner_exit"],
                                  "wall_s": result["wall_s"]})
        feedback.append(filter_verdict(result["verdict"], mode))
        s.state["evaluations"] = [e for e in s.state["evaluations"] if e["n"] != n] + [entry]
        s.save()
    entry["ended_wall_time_iso"] = _utc()
    s.save()
    return {"evaluation": n, "snapshot": str(snapshot), "feedback_mode": mode, "episodes": feedback}


@mcp.tool()
def read_verdict(episode_id: str) -> dict[str, Any]:
    """The feedback for one episode of this session (filtered by the cell's
    executor_feedback setting), plus the tail of your planner's own log."""
    s = session()
    if episode_id not in s.episode_ids():
        return {"error": f"{episode_id!r} is not an episode of this session", "episodes": s.episode_ids()}
    ep = REPO / "records" / episode_id
    verdict_path = ep / "verdict.json"
    if not verdict_path.is_file():
        return {"error": "no verdict recorded for this episode"}
    out = filter_verdict(json.loads(verdict_path.read_text()), _feedback_mode())
    log = ep / "executor" / "planner.log"
    if log.is_file():
        out["planner_log_tail"] = log.read_text(errors="replace").splitlines()[-60:]
    return out


@mcp.tool()
def submit_work(state: str, report: str) -> dict[str, Any]:
    """End the session. state: 'done' (the planner is finished) or 'gave_up'.
    report: what the planner does, what you changed and why, what is known to
    fail. The editable planner at this moment is frozen as the submission."""
    s = session()
    if s.state["submitted"] is not None:
        return {"error": "already submitted"}
    if state not in ("done", "gave_up"):
        return {"error": "state must be 'done' or 'gave_up'"}
    final = s.dir / "snapshots" / "final"
    if final.exists():
        shutil.rmtree(final)
    shutil.copytree(s.planner, final, ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".pytest_cache"))
    s.state["submitted"] = {"state": state, "report": report[:100_000], "planner_sha256": tree_hash(final)["sha256"],
                            "snapshot": str(final), "wall_time_iso": _utc()}
    s.save()
    return {"ok": True, "submission": s.state["submitted"]}


if __name__ == "__main__":
    mcp.run()
