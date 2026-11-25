"""Authoring MCP server: the tools an agent uses to write a planner.

    get_task()                         the task, the planner contract, where to edit
    get_assembly_views()               pictures of the finished assembly (no poses)
    evaluate_planner(episodes, seed)   freeze a snapshot of the editable planner,
                                       run fresh episodes with it, return feedback
    read_verdict(episode_id)           the feedback for one of this session's episodes
    submit_work(state, report)         end the session with the final planner

One authoring session (created by runner/run_authoring.sh) has two directories:

    executor/workspace/<condition>/<session>/   AUTHORING_SESSION_DIR, the agent's
        planner/            the editable planner (a copy of the condition's template)
        snapshots/NNN/      frozen copy evaluated by evaluate_planner call NNN
    records/authoring/<session>/                AUTHORING_RECORD_DIR, never the agent's
        agent_config.json   run parameters
        session.json        every call, every episode, the submission
        evals.jsonl         one record per episode run: id, snapshot, runner
                            exit, wall time, outcome (the runner's own events
                            are in records/<episode>/runner.jsonl)

AUTHORING_CONDITION is the study condition: plan_given (the full task file)
or plan_free (goal, constraints and object descriptions only).

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
sys.path.insert(0, str(HERE.parent))  # executor/planner_lint.py
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "ros2_ws" / "src" / "cell_platform_interfaces"))  # pure modules only (assembly_views)


def _append_system_dist_packages() -> None:
    """Pillow (to downscale pictures) lives in the system Python, as for the planner."""
    import site

    version = f"{sys.version_info.major}.{sys.version_info.minor}"
    for candidate in (f"/usr/local/lib/python{version}/dist-packages", "/usr/lib/python3/dist-packages", f"/usr/lib/python{version}/dist-packages"):
        if Path(candidate).is_dir() and candidate not in sys.path:
            site.addsitedir(candidate)


_append_system_dist_packages()

import anyio  # noqa: E402
from mcp.server.fastmcp import FastMCP, Image  # noqa: E402

try:  # results carrying pictures exceed the default per-tool result cap
    from mcp.types import ToolAnnotations  # noqa: E402

    _IMAGES = ToolAnnotations(**{"anthropic/maxResultSizeChars": 4_000_000})
except Exception:  # noqa: BLE001
    _IMAGES = None

from feedback import MODES, filter_verdict, needs_watchdog  # noqa: E402
from runner.episode_runner.manifest import tree_hash  # noqa: E402
from runner.jsonl import JsonlLog, read_jsonl  # noqa: E402

MAX_EPISODES_PER_CALL = 3
IMAGES_PER_EPISODE_EVAL = 2
IMAGES_PER_EPISODE_READ = 6

from planner_lint import FORBIDDEN_IN_PLANNER  # noqa: E402,F401  (executor/planner_lint.py)
CONDITIONS = ("plan_given", "plan_free")

HOW_TO = {
    "plan_given": (
        "The task file lists the steps to execute (sequence, grasp and insertion geometry). Edit the planner in "
        "editable_planner_dir (it starts as a copy of executor/planner_templates/plan_given, which executes those "
        "steps; run.sh, server.py, planner.py, worker.py, task.py, geometry.py). Executor-side data you may read: "
        "config/tasks/, config/parts/executor_parts.yaml, config/platforms/, the cell_platform_interfaces and "
        "cell_backend_* sources."
    ),
    "plan_free": (
        "The task file gives the goal, the constraints and the objects, not a plan, and no placement poses: where "
        "each part goes comes from get_assembly_views (pictures of the finished assembly) and your sensors; deciding "
        "the sequence, the grasps, the approach and insertion motions is your job. Edit the planner in editable_planner_dir (it starts "
        "as executor/planner_templates/plan_free, a skeleton that passes the gate, moves to the safe pose and stops; "
        "planner.py lists the primitives in worker.py and geometry.py). Executor-side data you may read: "
        "config/parts/executor_parts.yaml (collision shape, dimensions and grasp force per part), config/platforms/, "
        "the cell_platform_interfaces and cell_backend_* sources, robot_backend.utils (geometry, rg2_geometry)."
    ),
}

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
        self.records = Path(os.environ.get("AUTHORING_RECORD_DIR") or REPO / "records" / "authoring" / self.dir.name).resolve()
        self.records.mkdir(parents=True, exist_ok=True)
        self.condition = os.environ.get("AUTHORING_CONDITION", "plan_given")
        if self.condition not in CONDITIONS:
            raise RuntimeError(f"AUTHORING_CONDITION must be one of {CONDITIONS}, not {self.condition!r}")
        self.planner = self.dir / "planner"
        self.task = Path(os.environ.get("AUTHORING_TASK", "config/tasks/fmb_assembly_2.yaml"))
        self.task = self.task if self.task.is_absolute() else REPO / self.task
        self.assembly = os.environ.get("AUTHORING_ASSEMBLY", "fmb_assembly_2")
        self.platform = os.environ.get("CELL_PLATFORM", "sim")
        self.state_path = self.records / "session.json"
        if not self.planner.is_dir():
            raise RuntimeError(f"{self.planner} missing: run_authoring.sh copies the template there")
        self.state = json.loads(self.state_path.read_text()) if self.state_path.is_file() else {
            "session": self.dir.name, "started_wall_time_iso": _utc(), "task": str(self.task), "platform": self.platform,
            "condition": self.condition,
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
        "how_to": HOW_TO[s.condition] + (
            " evaluate_planner freezes a snapshot of that directory and runs fresh episodes with it on the cell; "
            "read the returned feedback (or read_verdict) and iterate. When the planner is as good as you can make "
            "it, call submit_work."
        ),
        "condition": s.condition,
        "feedback_mode": _feedback_mode(),
        "budgets": _cell().get("budgets"),
        "platform": s.platform,
        "evaluations_so_far": len(s.state["evaluations"]),
        "submitted": s.state["submitted"] is not None,
    }


@mcp.tool(structured_output=False, annotations=_IMAGES)
def get_assembly_views():
    """Pictures of the FINISHED assembly from several views (with captions
    when provided): what the parts look like once seated. They are all you
    get about where parts go: no placement poses are published or given.
    Call once and keep what you need."""
    from assembly_views import load_views  # cell_platform_interfaces/assembly_views.py, pure

    s = session()
    views, info = load_views(REPO, s.assembly)
    text = {"assembly": s.assembly, "views": [{"name": v.name, "caption": v.caption, "size": v.note} for v in views],
            "notes": info.get("notes"), "skipped": info.get("skipped")}
    return [json.dumps(text, indent=2)] + [Image(data=v.data, format=v.format) for v in views]


def lint_planner(folder: Path) -> list[str]:
    """Forbidden references the agent added to the planner (executor/planner_lint.py)."""
    import planner_lint

    return planner_lint.lint_planner(folder, REPO / "executor" / "planner_templates")


def _episode_images(episode_id: str, limit: int) -> list[Any]:
    folder = REPO / "records" / episode_id / "executor" / "images"
    paths = sorted(folder.glob("*.png"))[-limit:] if folder.is_dir() else []
    out = []
    for path in paths:
        data, fmt = path.read_bytes(), "png"
        try:
            import io

            from PIL import Image as PILImage

            img = PILImage.open(io.BytesIO(data))
            img.thumbnail((800, 800))
            buf = io.BytesIO()
            img.convert("RGB").save(buf, format="JPEG", quality=85)
            data, fmt = buf.getvalue(), "jpeg"
        except Exception:  # noqa: BLE001
            pass
        out.append((path.name, Image(data=data, format=fmt)))
    return out


def _run_episode(s: Session, snapshot: Path, episode_id: str, seed: int, watchdog: str) -> dict[str, Any]:
    cmd = [str(REPO / "runner" / "episode_runner" / "run.sh"), "--planner", str(snapshot), "--task", str(s.task),
           "--assembly", s.assembly, "--seed", str(seed), "--platform", s.platform, "--episode-id", episode_id,
           "--watchdog", watchdog]
    env = {k: v for k, v in os.environ.items() if not k.startswith(("AUTHORING_", "VIRTUAL_ENV"))}
    started = time.monotonic()
    proc = subprocess.run(cmd, cwd=str(REPO), env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors="replace")
    rc = proc.returncode
    verdict_path = REPO / "records" / episode_id / "verdict.json"
    verdict = json.loads(verdict_path.read_text()) if verdict_path.is_file() else {
        "episode_id": episode_id, "outcome": "invalid", "stage": "runner", "reason": f"runner exited {rc} without a verdict"}
    entry = {"episode_id": episode_id, "snapshot": str(snapshot), "seed": seed, "watchdog": watchdog, "runner_exit": rc,
             "wall_s": round(time.monotonic() - started, 1), "outcome": verdict.get("outcome"), "stage": verdict.get("stage"),
             "record_dir": str(verdict_path.parent)}
    if not verdict_path.is_file():
        entry["runner_output_tail"] = proc.stdout.splitlines()[-80:]
    JsonlLog(s.records / "evals.jsonl").write("episode", **entry)
    return {"episode_id": episode_id, "runner_exit": rc, "wall_s": entry["wall_s"], "verdict": verdict}


@mcp.tool(structured_output=False, annotations=_IMAGES)
async def evaluate_planner(episodes: int = 1, seed: int = 0):
    """Freeze a snapshot of the editable planner and run `episodes` fresh episodes
    with it (seeds seed, seed+1, ...). Each episode resets the scene, runs
    execute_task once and is judged. Takes minutes per episode. Returns the
    feedback for each episode, followed by the last camera1 frames your
    planner saved in each (Worker.save_frame), newest last."""
    result = await _evaluate(episodes, seed)
    pictures: list[Any] = []
    names: dict[str, list[str]] = {}
    for ep in result.get("episodes", []) if isinstance(result, dict) else []:
        eid = ep.get("episode_id") if isinstance(ep, dict) else None
        for name, img in _episode_images(eid, IMAGES_PER_EPISODE_EVAL) if eid else []:
            names.setdefault(eid, []).append(name)
            pictures.append(img)
    if names:
        result["images_attached"] = names
    return [json.dumps(result, indent=2, default=str)] + pictures


async def _evaluate(episodes: int, seed: int) -> dict[str, Any]:
    s = session()
    if s.state["submitted"] is not None:
        return {"error": "work already submitted; this session is closed"}
    if not 1 <= int(episodes) <= MAX_EPISODES_PER_CALL:
        return {"error": f"episodes must be 1..{MAX_EPISODES_PER_CALL}"}
    hits = lint_planner(s.planner)
    if hits:
        return {"error": "the planner references what it may not use (docs/PLANNER_CONTRACT.md section 4); no episode was run",
                "references": hits[:40]}
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


@mcp.tool(structured_output=False, annotations=_IMAGES)
def read_verdict(episode_id: str):
    """The feedback for one episode of this session (filtered by the cell's
    executor_feedback setting), the tail of your planner's own log, and the
    last camera1 frames your planner saved in it."""
    out = _read_verdict(episode_id)
    pictures = _episode_images(episode_id, IMAGES_PER_EPISODE_READ) if "error" not in out else []
    if pictures:
        out["images_attached"] = [n for n, _ in pictures]
    return [json.dumps(out, indent=2, default=str)] + [img for _, img in pictures]


def _read_verdict(episode_id: str) -> dict[str, Any]:
    s = session()
    if episode_id not in s.episode_ids():
        return {"error": f"{episode_id!r} is not an episode of this session", "episodes": s.episode_ids()}
    ep = REPO / "records" / episode_id
    verdict_path = ep / "verdict.json"
    if not verdict_path.is_file():
        return {"error": "no verdict recorded for this episode"}
    out = filter_verdict(json.loads(verdict_path.read_text()), _feedback_mode())
    tail = read_jsonl(ep / "executor" / "planner.jsonl", tail=60)
    if tail:
        out["planner_log_tail"] = tail
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
