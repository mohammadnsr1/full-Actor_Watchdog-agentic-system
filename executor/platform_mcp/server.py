"""Platform MCP server (EB6): the executor baseline's `platform` tools, modelled on the
labmate's (docs/EXECUTOR_BASELINE.md section 1).

    check_delivery()                    is artifact/ a planner the runner can run? (delivery.py)
    evaluate_planner(episodes, seed?)   freeze artifact/, run 1-3 fresh episodes, write the
                                        evidence to <app>/evaluations/NNNN/; charged per
                                        completed episode against the budget (18)
    submit_work(state, report)          end the session; 'done' only for an evaluated artifact
    move_robot(joints | tool_delta)     direct arm motion between episodes (no planning)
    robot_health()                      controllers, joint states, planning service, simulator

Runs on the cell host, where ROS, the runner and the WatchDog are; the authoring agent
reaches it over loopback HTTP from its container (EB4) or over stdio:

    python3 executor/platform_mcp/server.py [--http 127.0.0.1:9020]

Environment (runner/run_baseline.sh sets it):
    PLATFORM_SESSION_DIR   host path of the agent's /app (artifact/, evaluations/, report.md)
    PLATFORM_AGENT_ROOT    the same directory as the agent sees it (default /app)
    PLATFORM_RECORD_DIR    host bookkeeping, never the agent's (default records/authoring/<session>)
    PLATFORM_TASK          the task record, <task data>/tasks/<id>/task.yaml
    PLATFORM_CONDITION     baseline | consult
    PLATFORM_BUDGET        evaluation episodes (default 18)
"""

from __future__ import annotations

import argparse
import fcntl
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
for p in (HERE, HERE.parent, REPO, REPO / "executor" / "authoring_mcp", REPO / "executor" / "planner_templates" / "common"):
    sys.path.insert(0, str(p))

import anyio  # noqa: E402
from mcp.server.fastmcp import FastMCP  # noqa: E402

import delivery  # noqa: E402
from feedback import filter_verdict  # noqa: E402
from runner.episode_runner.manifest import tree_hash  # noqa: E402
from runner.jsonl import JsonlLog  # noqa: E402

MAX_EPISODES_PER_CALL = 3
DEFAULT_BUDGET = 18
CONDITIONS = {"baseline": ("checker", "none"), "consult": ("both", "plan_blind")}   # feedback mode, runner --watchdog
FRAMES_KEPT_PER_S = 1.0          # recorder frames copied into the evidence (about 1 per sim second)
mcp = FastMCP("platform")


def _utc() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


def _cell() -> dict[str, Any]:
    return yaml.safe_load((REPO / "config" / "cell.yaml").read_text()) or {}


class Session:
    def __init__(self) -> None:
        d = os.environ.get("PLATFORM_SESSION_DIR")
        if not d:
            raise RuntimeError("PLATFORM_SESSION_DIR is not set (start the session with runner/run_baseline.sh)")
        self.app = Path(d).resolve()
        self.agent_root = os.environ.get("PLATFORM_AGENT_ROOT", "/app")
        self.records = Path(os.environ.get("PLATFORM_RECORD_DIR") or REPO / "records" / "authoring" / self.app.name).resolve()
        self.records.mkdir(parents=True, exist_ok=True)
        self.task = Path(os.environ.get("PLATFORM_TASK") or REPO / "task_data/tasks/fmb_assembly_2/task.yaml").resolve()
        self.condition = os.environ.get("PLATFORM_CONDITION", "baseline")
        if self.condition not in CONDITIONS:
            raise RuntimeError(f"PLATFORM_CONDITION must be one of {sorted(CONDITIONS)}")
        self.budget = int(os.environ.get("PLATFORM_BUDGET") or DEFAULT_BUDGET)
        self.artifact = self.app / "artifact"
        self.evaluations = self.app / "evaluations"
        self.evaluations.mkdir(parents=True, exist_ok=True)
        self.state_path = self.records / "session.json"
        self.state = json.loads(self.state_path.read_text()) if self.state_path.is_file() else {
            "session": self.app.name, "started_wall_time_iso": _utc(), "task": str(self.task), "condition": self.condition,
            "budget": self.budget, "charged": 0, "next_seed": 1, "evaluations": [], "submitted": None, "lab": []}

    def save(self) -> None:
        self.state_path.write_text(json.dumps(self.state, indent=2))

    def shown(self, path: Path) -> str:
        """A host path inside the session dir as the agent sees it."""
        return self.agent_root + "/" + path.resolve().relative_to(self.app).as_posix()


_S: Session | None = None


def session() -> Session:
    global _S
    if _S is None:
        _S = Session()
    return _S


def _planner_env() -> dict[str, str]:
    env = {k: v for k, v in os.environ.items() if not k.startswith(("PLATFORM_", "VIRTUAL_ENV"))}
    return {**env, "CELL_REPO_ROOT": str(REPO)}


async def _check(s: Session) -> dict[str, Any]:
    return await delivery.check_delivery(s.artifact, REPO / "executor" / "planner_templates", _planner_env())


@mcp.tool()
async def check_delivery() -> dict[str, Any]:
    """Check artifact/ as evaluate_planner will before any episode: manifest.yaml, the
    contract lint, and that the entrypoint serves exactly execute_task(task_ref)."""
    return await _check(session())


def _cell_busy() -> bool:
    lock = open(REPO / "records" / ".cell.lock", "a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.flock(lock, fcntl.LOCK_UN)
        return False
    except OSError:
        return True
    finally:
        lock.close()


def _copy_evidence(episode_id: str, dst: Path) -> dict[str, Any]:
    src = REPO / "records" / episode_id
    dst.mkdir(parents=True, exist_ok=True)
    copied = []
    for rel in ("runner.jsonl", "recording.json", "executor/planner.jsonl", "executor/planner.stderr.jsonl"):
        if (src / rel).is_file():
            shutil.copy2(src / rel, dst / Path(rel).name)
            copied.append(Path(rel).name)
    for name, sub in (("bag", "bag"), ("images", "executor/images")):
        if (src / sub).is_dir():
            shutil.copytree(src / sub, dst / name, dirs_exist_ok=True)
            copied.append(name + "/")
    frames = sorted((src / "frames").glob("*.jpg")) if (src / "frames").is_dir() else []
    rate = float((_cell().get("recording") or {}).get("frames", {}).get("rate_hz", 10.0) or 10.0)
    step = max(1, int(round(rate / FRAMES_KEPT_PER_S)))
    if frames:
        (dst / "frames").mkdir(exist_ok=True)
        for f in frames[::step]:
            shutil.copy2(f, dst / "frames" / f.name)
        copied.append(f"frames/ ({len(frames[::step])} of {len(frames)})")
    return {"copied": copied}


def _run_episode(s: Session, snapshot: Path, episode_id: str, seed: int, watchdog: str, sim_s: float) -> dict[str, Any]:
    cmd = [str(REPO / "runner" / "episode_runner" / "run.sh"), "--planner", str(snapshot), "--task", str(s.task),
           "--assembly", "fmb_assembly_2", "--seed", str(seed), "--episode-id", episode_id, "--watchdog", watchdog]
    cap = float((_cell().get("budgets") or {}).get("episode_sim_s") or 0)
    if sim_s:
        cmd += ["--budget-sim-s", str(min(float(sim_s), cap) if cap else float(sim_s))]
    started = time.monotonic()
    proc = subprocess.run(cmd, cwd=str(REPO), env=_planner_env(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True, errors="replace")
    verdict_path = REPO / "records" / episode_id / "verdict.json"
    verdict = json.loads(verdict_path.read_text()) if verdict_path.is_file() else {
        "episode_id": episode_id, "outcome": "invalid", "stage": "runner", "reason": f"runner exited {proc.returncode} without a verdict"}
    entry = {"episode_id": episode_id, "seed": seed, "runner_exit": proc.returncode, "wall_s": round(time.monotonic() - started, 1),
             "outcome": verdict.get("outcome"), "stage": verdict.get("stage")}
    if not verdict_path.is_file():
        entry["runner_output_tail"] = proc.stdout.splitlines()[-60:]
    JsonlLog(s.records / "evals.jsonl").write("episode", **entry)
    return {"entry": entry, "verdict": verdict}


def _verdict_text(fb: dict[str, Any]) -> str:
    c = fb.get("checker") or {}
    lines = [f"outcome: {fb.get('outcome')}", f"planner: {(fb.get('planner') or {}).get('state')} - {(fb.get('planner') or {}).get('reason', '')}"]
    if c:
        lines.append(f"parts placed: {c.get('parts_placed')}/{c.get('parts_required')}")
        for pid, r in (c.get("parts") or {}).items():
            lines.append(f"  {pid}: {r.get('status')} {r.get('position_error_m', '')} m {r.get('orientation_error_deg', '')} deg")
    return "\n".join(lines) + "\n"


@mcp.tool()
async def evaluate_planner(episodes: int = 1, seed: int | None = None) -> dict[str, Any]:
    """Freeze artifact/ and run 1-3 fresh episodes with it on the cell (each resets the
    scene with its seed, calls execute_task once, and is judged). Evidence goes to
    evaluations/NNNN/: evaluation.json, and per episode verdict.json, verdict.txt, the
    planner log, the runner report, the bag, saved images and scene frames. Each completed
    episode is charged against the evaluation budget; an invalid one (the cell, not your
    planner) is not. Takes minutes per episode."""
    s = session()
    if not 1 <= int(episodes) <= MAX_EPISODES_PER_CALL:
        return {"error": f"episodes must be 1..{MAX_EPISODES_PER_CALL}"}
    left = s.budget - s.state["charged"]
    if int(episodes) > left:
        return {"error": f"evaluation budget: {left} episode(s) left of {s.budget}"}
    check = await _check(s)
    if not check["ok"]:
        return {"error": "the artifact does not pass check_delivery; no episode was run", **check}
    n = len(s.state["evaluations"]) + 1
    snapshot = s.records / "snapshots" / f"{n:04d}"
    shutil.copytree(s.artifact, snapshot, symlinks=False, ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".pytest_cache"))
    out_dir = s.evaluations / f"{n:04d}"
    mode, watchdog = CONDITIONS[s.condition]
    first = int(seed) if seed is not None else int(s.state["next_seed"])
    ev: dict[str, Any] = {"n": n, "artifact_sha256": tree_hash(s.artifact)["sha256"], "snapshot": str(snapshot),
                          "started_wall_time_iso": _utc(), "episodes": []}
    for i in range(int(episodes)):
        sd = first + i
        episode_id = f"{s.app.name}-e{n:04d}-{i + 1}-s{sd}"
        r = await anyio.to_thread.run_sync(_run_episode, s, snapshot, episode_id, sd, watchdog, float(check.get("budget_sim_s") or 0))
        fb = {"outcome": r["verdict"].get("outcome"), **filter_verdict(r["verdict"], mode)}
        ep_dir = out_dir / f"episode-{i + 1}"
        ep_dir.mkdir(parents=True, exist_ok=True)
        (ep_dir / "verdict.json").write_text(json.dumps(fb, indent=2, default=str))
        (ep_dir / "verdict.txt").write_text(_verdict_text(fb))
        evidence = await anyio.to_thread.run_sync(_copy_evidence, episode_id, ep_dir)
        charged = r["entry"]["outcome"] in ("success", "failure")
        s.state["charged"] += int(charged)
        ev["episodes"].append({**r["entry"], "charged": charged, "evidence": s.shown(ep_dir), **evidence})
    s.state["next_seed"] = max(int(s.state["next_seed"]), first + int(episodes))
    ev["ended_wall_time_iso"] = _utc()
    ev["completed"] = any(e["charged"] for e in ev["episodes"])
    s.state["evaluations"].append(ev)
    s.save()
    summary = {"evaluation": n, "episodes": [{k: e[k] for k in ("episode_id", "seed", "outcome", "charged", "evidence")} for e in ev["episodes"]],
               "budget": {"charged": s.state["charged"], "left": s.budget - s.state["charged"], "of": s.budget}}
    (out_dir / "evaluation.json").write_text(json.dumps(summary, indent=2))
    return {**summary, "evidence_dir": s.shown(out_dir)}


@mcp.tool()
def submit_work(state: str, report: str = "report.md") -> dict[str, Any]:
    """Record your submission: state 'done' (you delivered a planner; refused unless
    evaluate_planner completed on artifact/ exactly as it stands now) or 'stopped' (you did
    not; always accepted). report: the path of your report file, absolute or relative to
    /app. Calling again replaces the earlier submission; the session ends when you end it,
    and artifact/ is collected as it stands then."""
    s = session()
    if state not in ("done", "stopped"):
        return {"error": "state must be 'done' or 'stopped'"}
    rel = report[len(s.agent_root) + 1:] if report.startswith(s.agent_root + "/") else report
    path = (s.app / rel).resolve()
    if not path.is_relative_to(s.app) or not path.is_file():
        return {"error": f"report file {report!r} not found under {s.agent_root}"}
    sha = tree_hash(s.artifact)["sha256"]
    if state == "done" and sha not in {e["artifact_sha256"] for e in s.state["evaluations"] if e.get("completed")}:
        return {"error": "SUBMIT_WORK_UNEVALUATED: no evaluate_planner call completed on artifact/ as it stands now"}
    final = s.records / "snapshots" / "submitted"
    if final.exists():
        shutil.rmtree(final)
    shutil.copytree(s.artifact, final, symlinks=False, ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".pytest_cache"))
    s.state["submitted"] = {"state": state, "artifact_sha256": sha, "report": path.read_text()[:100_000],
                            "wall_time_iso": _utc(), "charged": s.state["charged"],
                            "replaces": (s.state["submitted"] or {}).get("wall_time_iso")}
    s.save()
    return {"ok": True, "state": state, "artifact_sha256": sha}


# -- host-side lab verbs ------------------------------------------------------------

_W: Any = None


def _worker() -> Any:
    global _W
    if _W is None:
        from worker import Worker

        _W = Worker(os.environ.get("CELL_PLATFORM", "sim"), str(REPO), node_name="platform_lab")
        from std_msgs.msg import String

        _W._lab_pub = _W.node.create_publisher(String, _W.platform.topic("lab_events"), 10)
    return _W


def _lab_event(w: Any, kind: str, detail: dict[str, Any]) -> None:
    from std_msgs.msg import String

    w._lab_pub.publish(String(data=json.dumps({"kind": kind, "objects": ["robot"], "detail": detail,
                                               "source": "platform_mcp", "wall_time_iso": _utc()}, sort_keys=True)))


@mcp.tool()
async def move_robot(joints: dict[str, float] | None = None, tool_delta: dict[str, Any] | None = None, frame: str = "tool",
                     speed: float = 0.3) -> dict[str, Any]:
    """Move the arm directly, between episodes: `joints` {name: rad} along a smooth joint path
    (speed: max rad/s, default 0.3), or `tool_delta` {position: {x,y,z} m, orientation:
    {x,y,z,w}} as a straight TCP move in the tool's axes (frame='tool') or the world's
    (frame='world'; speed: velocity scaling 0-1). No planning and no collision checking.
    Refused while an episode runs. Publishes a lab event."""
    if _cell_busy():
        return {"error": "an episode is running on the cell (records/.cell.lock is held)"}
    if (joints is None) == (tool_delta is None):
        return {"error": "give exactly one of joints or tool_delta"}

    def _go() -> dict[str, Any]:
        w = _worker()
        if joints is not None:
            w.move_joints({k: float(v) for k, v in joints.items()}, max_speed=float(speed), label="lab:move_robot")
        else:
            v = max(0.01, min(1.0, float(speed)))
            w.jog(tool_delta, frame=frame, vel=v, acc=v, label="lab:move_robot")
        _lab_event(w, "move_robot", {"joints": joints, "tool_delta": tool_delta, "frame": frame})
        return {"ok": True, "tcp": w.tcp_pose()}

    try:
        result = await anyio.to_thread.run_sync(_go)
    except Exception as exc:  # noqa: BLE001
        return {"error": f"{type(exc).__name__}: {exc}"}
    s = session()
    s.state["lab"].append({"verb": "move_robot", "wall_time_iso": _utc(), "joints": joints, "tool_delta": tool_delta})
    s.save()
    return result


@mcp.tool()
async def robot_health() -> dict[str, Any]:
    """The controllers' side of robot health (arm action server, fresh joint states,
    planning service) and the simulator's (the lab verb robot_health)."""
    def _probe() -> dict[str, Any]:
        out: dict[str, Any] = {}
        try:
            w = _worker()
            snap = w.backends.motion.joint_state(max_age_s=1.0)
            out["joint_states_fresh"] = snap is not None
            out["arm_action_ready"] = bool(w.backends.motion._client.server_is_ready())
            out["planning_service_ready"] = bool(w.plan_pose.service_is_ready())
        except Exception as exc:  # noqa: BLE001
            out["error"] = f"{type(exc).__name__}: {exc}"
        try:
            from cell_backend_sim.scene import CommandServerClient

            plat = yaml.safe_load((REPO / "config" / "platforms" / "sim.yaml").read_text())["scene"]
            out["simulator"] = CommandServerClient(str(plat.get("host", "127.0.0.1")), int(plat["port"]), timeout_s=3.0).call("lab_robot_health")
        except Exception as exc:  # noqa: BLE001
            out["simulator"] = {"ok": False, "error": str(exc)}
        out["ok"] = bool(out.get("joint_states_fresh") and out.get("arm_action_ready") and (out["simulator"] or {}).get("ok"))
        out["episode_running"] = _cell_busy()
        return out

    return await anyio.to_thread.run_sync(_probe)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--http", default="", help="host:port for streamable HTTP (default: stdio)")
    args = ap.parse_args()
    if args.http:
        host, port = args.http.rsplit(":", 1)
        mcp.settings.host, mcp.settings.port = host, int(port)
        mcp.run("streamable-http")
    else:
        mcp.run()


if __name__ == "__main__":
    main()
