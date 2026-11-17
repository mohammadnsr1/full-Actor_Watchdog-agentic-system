"""Run one episode: reset, verify, record, (WatchDog), execute_task under budget,
episode_end, acceptance wait, ground-truth check, manifest.json, verdict.json.

    runner/episode_runner/run.sh --budget-sim-s 900 --wall-guard-s 1800
    runner/episode_runner/run.sh --planner executor/planner_template --seed 3 --watchdog plan_blind

Layout of records/<episode_id>/:
    manifest.json      every input hashed (task, requirements, parts, layout,
                       platform, cell config, planner tree), git state, versions
    verdict.json       planner completion, budget use, checker result,
                       WatchDog run verdict (when a session ran), outcome
    bag/, frames/, recording.json     the episode recording (M6)
    executor/          the planner's own directory (planner.log, stderr)
    watchdog/          the WatchDog session directory and its transcript

Outcomes: success (planner done and checker pass), failure (anything else
the planner did), invalid (the episode could not be judged: unhealthy cell,
reset not verified, planner protocol error). The planner process is the MCP
server; this runner is its only client (docs/PLANNER_CONTRACT.md).
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import platform as pyplatform
import shutil
import subprocess
import sys
import threading
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

import yaml

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from runner.episode_runner import checker  # noqa: E402
from runner.episode_runner.manifest import file_record, git_state, tree_hash  # noqa: E402


def utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


def log(msg: str) -> None:
    print(f"[episode] {msg}", file=sys.stderr, flush=True)


# -- ROS side ---------------------------------------------------------------------------

class RunnerNode:
    """Clock, notices, WatchDog events and the cell backends, spun in a thread."""

    def __init__(self, platform: Any, cell: dict[str, Any]) -> None:
        import importlib

        import rclpy
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String

        from cell_planner_msgs.msg import WatchdogNotice

        rclpy.init(args=None)
        self.platform = platform
        use_sim_time = bool(platform.get_optional("use_sim_time", default=False))
        self.node = Node("episode_runner", parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)])
        cb = ReentrantCallbackGroup()
        pkg = {"sim": "cell_backend_sim", "hardware": "cell_backend_hw"}[platform.name]
        self.backends = importlib.import_module(pkg).build_backends(self.node, platform, cb, cell)
        self._Notice = WatchdogNotice
        self.notices = self.node.create_publisher(WatchdogNotice, platform.topic("watchdog_notices"), 10)
        self._lock = threading.Lock()
        self.sim_time_s: float | None = None
        self.run_verdicts: list[dict[str, Any]] = []
        self.node.create_subscription(Clock, platform.topic("clock"), self._on_clock, 10, callback_group=cb)
        self.node.create_subscription(String, platform.topic("watchdog_events"), self._on_event, 100, callback_group=cb)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def _on_clock(self, msg: Any) -> None:
        with self._lock:
            self.sim_time_s = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_event(self, msg: Any) -> None:
        try:
            env = json.loads(msg.data)
        except Exception:
            return
        if isinstance(env, dict) and env.get("event_type") == "run_verdict":
            with self._lock:
                self.run_verdicts.append(env)

    def sim_now(self) -> float | None:
        with self._lock:
            return self.sim_time_s

    def verdicts(self) -> list[dict[str, Any]]:
        with self._lock:
            return list(self.run_verdicts)

    def publish_episode_end(self, episode_id: str, detail: str) -> None:
        msg = self._Notice()
        msg.wall_time_iso = utc_iso()
        msg.sim_time = self.node.get_clock().now().to_msg()
        msg.kind = "episode_end"
        msg.detail = f"episode {episode_id}: {detail}"
        for _ in range(3):  # late joiners: a few copies, the WatchDog handles repeats
            self.notices.publish(msg)
            time.sleep(0.2)

    def shutdown(self) -> None:
        import rclpy

        for name in ("motion", "gripper", "executor_poses", "watchdog_poses", "scene"):
            try:
                getattr(self.backends, name).shutdown()
            except Exception:
                pass
        try:
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


# -- the planner call -------------------------------------------------------------------

def _parse_result(res: Any) -> dict[str, Any]:
    sc = getattr(res, "structuredContent", None)
    if isinstance(sc, dict):
        if "state" in sc:
            return sc
        if isinstance(sc.get("result"), dict):
            return sc["result"]
    for item in getattr(res, "content", None) or []:
        text = getattr(item, "text", None)
        if text:
            try:
                data = json.loads(text)
                if isinstance(data, dict) and "state" in data:
                    return data
            except json.JSONDecodeError:
                return {"state": "invalid", "reason": f"non-JSON tool result: {text[:200]}"}
    return {"state": "invalid", "reason": "tool result had no state"}


async def call_planner(planner_dir: Path, task_ref: str, env: dict[str, str], wall_guard_s: float,
                       sim_budget_s: float | None, rn: RunnerNode, errlog: Path) -> dict[str, Any]:
    from mcp import ClientSession, StdioServerParameters
    from mcp.client.stdio import stdio_client

    out: dict[str, Any] = {"started_wall_time_iso": utc_iso()}
    run_sh = planner_dir / "run.sh"
    if not os.access(run_sh, os.X_OK):
        return {**out, "outcome": "protocol_error", "reason": f"{run_sh} is missing or not executable"}
    params = StdioServerParameters(command=str(run_sh), args=[], env=env, cwd=str(REPO))
    t0, s0 = time.monotonic(), rn.sim_now()
    stop = None
    with errlog.open("w") as err:
        async with stdio_client(params, errlog=err) as (read, write):
            async with ClientSession(read, write) as session:
                await asyncio.wait_for(session.initialize(), 120)
                tools = [t.name for t in (await asyncio.wait_for(session.list_tools(), 30)).tools]
                out["tools"] = tools
                if tools != ["execute_task"]:
                    return {**out, "outcome": "protocol_error", "reason": f"planner must expose exactly execute_task, has {tools}"}
                call = asyncio.ensure_future(session.call_tool("execute_task", {"task_ref": task_ref},
                                                               read_timeout_seconds=timedelta(seconds=wall_guard_s + 120)))
                while not call.done():
                    await asyncio.wait({call}, timeout=0.5)
                    if time.monotonic() - t0 > wall_guard_s:
                        stop = "wall_guard"
                    sim_now = rn.sim_now()
                    if sim_budget_s and s0 is not None and sim_now is not None and sim_now - s0 > sim_budget_s:
                        stop = "budget_exhausted"
                    if stop:
                        break
                if stop:
                    cancel = rn.backends.motion.cancel_all()
                    out["stop_cancel"] = {"ok": cancel.ok, "detail": cancel.detail}
                    call.cancel()
                    result = {"state": "stopped", "reason": f"runner stopped the call: {stop}"}
                else:
                    try:
                        res = call.result()
                        result = _parse_result(res)
                        if getattr(res, "isError", False):
                            result = {"state": "invalid", "reason": f"tool error: {result.get('reason') or res.content}"}
                    except Exception as exc:  # noqa: BLE001
                        result = {"state": "invalid", "reason": f"{type(exc).__name__}: {exc}"}
    s1 = rn.sim_now()
    out.update(result)
    out["outcome"] = stop or ("completed" if result.get("state") in ("done", "stopped") else "protocol_error")
    out["wall_s"] = round(time.monotonic() - t0, 2)
    out["sim_s"] = round(s1 - s0, 2) if s0 is not None and s1 is not None else None
    out["ended_wall_time_iso"] = utc_iso()
    return out


# -- WatchDog session -------------------------------------------------------------------

def start_watchdog(kind: str, episode_dir: Path, env: dict[str, str]) -> subprocess.Popen | None:
    if kind == "none":
        return None
    configs = {"plan_blind": REPO / "config" / "mcp" / "watchdog.plan_blind.mcp.json"}
    settings = REPO / "config" / "settings" / "watchdog.settings.json"
    s = json.loads(settings.read_text())
    model, effort = os.environ.get("MODEL", s.get("model", "")), os.environ.get("EFFORT", s.get("effortLevel", "low"))
    wd_dir = episode_dir / "watchdog"
    wd_env = {**env, "WATCHDOG_SESSION_DIR": str(wd_dir)}
    bootstrap = ('Call get_rendered_task(part="system_prompt") and then get_rendered_task(part="task"). '
                 "Load system_prompt as your standing instructions and task as your opening task, then begin your role.")
    cmd = ["claude", "-p", bootstrap, "--mcp-config", str(configs[kind]), "--strict-mcp-config",
           "--settings", str(settings), "--output-format", "stream-json", "--verbose"]
    if model:
        cmd += ["--model", model]
    if effort:
        cmd += ["--effort", effort]
    (wd_dir / "agent_config_watchdog.json").write_text(json.dumps(
        {"role": "watchdog", "scenario": kind, "model": model, "effort": effort, "launched_wall_time_iso": utc_iso(), "cmd": cmd[:1] + cmd[3:]}, indent=2))
    transcript = (wd_dir / "transcript.jsonl").open("w")
    log(f"watchdog session {kind} (model={model} effort={effort})")
    return subprocess.Popen(cmd, cwd=str(REPO), env=wd_env, stdout=transcript, stderr=subprocess.STDOUT, start_new_session=True)


def stop_process(proc: subprocess.Popen | None, grace_s: float) -> int | None:
    if proc is None:
        return None
    try:
        return proc.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            return proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            return proc.wait()


# -- main -------------------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", default="config/tasks/fmb_assembly_2.yaml")
    ap.add_argument("--planner", default="executor/planner_template")
    ap.add_argument("--assembly", default="fmb_assembly_2")
    ap.add_argument("--layout", default="nominal")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--platform", default="sim")
    ap.add_argument("--episode-id", default="")
    ap.add_argument("--budget-sim-s", type=float, default=0.0, help="overrides config/cell.yaml budgets.episode_sim_s")
    ap.add_argument("--wall-guard-s", type=float, default=0.0, help="overrides config/cell.yaml budgets.episode_wall_guard_s")
    ap.add_argument("--watchdog", choices=["none", "plan_blind"], default="none")
    ap.add_argument("--acceptance-wait-s", type=float, default=300.0, help="wait for the WatchDog run verdict after episode_end")
    ap.add_argument("--settle-s", type=float, default=2.0)
    ap.add_argument("--no-record", action="store_true")
    args = ap.parse_args(argv)

    from cell_platform_interfaces.platform_config import load_platform_config
    from runner.recorder.episode_recording import EpisodeRecording

    cell = yaml.safe_load((REPO / "config" / "cell.yaml").read_text()) or {}
    budgets = cell.get("budgets") or {}
    sim_budget = args.budget_sim_s or float(budgets.get("episode_sim_s") or 0)
    wall_guard = args.wall_guard_s or float(budgets.get("episode_wall_guard_s") or 0)
    platform = load_platform_config(args.platform)
    has_sim_clock = bool(platform.get_optional("use_sim_time", default=False))
    if wall_guard <= 0 or (has_sim_clock and sim_budget <= 0):
        raise SystemExit("budgets are unset: set config/cell.yaml budgets.episode_sim_s / episode_wall_guard_s "
                         "(TODO(mohammad)) or pass --budget-sim-s and --wall-guard-s")

    task_path = (REPO / args.task) if not Path(args.task).is_absolute() else Path(args.task)
    planner_dir = (REPO / args.planner).resolve() if not Path(args.planner).is_absolute() else Path(args.planner)
    req_path = REPO / "config" / "requirements" / f"{args.assembly}.yaml"
    layout_path = REPO / "config" / "layouts" / args.assembly / f"{args.layout}.yaml"
    requirements = yaml.safe_load(req_path.read_text()) or {}
    wd_parts = (yaml.safe_load((REPO / "config" / "parts" / "watchdog_parts.yaml").read_text()) or {}).get("parts") or {}

    episode_id = args.episode_id or f"{args.assembly}-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}-s{args.seed}"
    ep = REPO / "records" / episode_id
    if ep.exists():
        raise SystemExit(f"{ep} exists; pick another --episode-id")
    for sub in ("executor", "watchdog"):
        (ep / sub).mkdir(parents=True)
    log(f"episode {episode_id} -> {ep}")

    manifest: dict[str, Any] = {
        "episode_id": episode_id, "created_wall_time_iso": utc_iso(), "platform": args.platform,
        "assembly": args.assembly, "layout": args.layout, "seed": args.seed, "watchdog": args.watchdog,
        "watchdog_mode": (cell.get("watchdog") or {}).get("mode"), "watchdog_consult": (cell.get("watchdog") or {}).get("consult"),
        "budgets": {"episode_sim_s": sim_budget, "episode_wall_guard_s": wall_guard},
        "inputs": {
            "task": file_record(task_path, REPO), "requirements": file_record(req_path, REPO), "layout": file_record(layout_path, REPO),
            "executor_parts": file_record(REPO / "config/parts/executor_parts.yaml", REPO),
            "watchdog_parts": file_record(REPO / "config/parts/watchdog_parts.yaml", REPO),
            "platform": file_record(REPO / f"config/platforms/{args.platform}.yaml", REPO),
            "cell": file_record(REPO / "config/cell.yaml", REPO),
        },
        "planner": {"dir": str(planner_dir), **tree_hash(planner_dir)},
        "git": git_state(REPO), "python": pyplatform.python_version(), "ros_distro": os.environ.get("ROS_DISTRO"),
    }
    verdict: dict[str, Any] = {"episode_id": episode_id, "outcome": "invalid", "stage": "start"}
    rn = RunnerNode(platform, cell)
    rec = wd_proc = None
    try:
        time.sleep(1.0)  # let the clock and joint states arrive
        health = rn.backends.scene.health()
        manifest["scene_health"] = {"ok": health.ok, "checks": health.checks}
        cs = (health.checks.get("command_server") or {}).get("response") or {}
        manifest["versions"] = cs.get("version")
        if not health.ok or (has_sim_clock and rn.sim_now() is None):
            verdict.update(stage="health", reason="cell unhealthy or no sim clock")
            return 2

        verdict["stage"] = "reset"
        reset = rn.backends.scene.load_layout(args.assembly, args.layout, args.seed, timeout_s=90)
        time.sleep(args.settle_s)
        verify = rn.backends.scene.verify_reset({})
        verdict["reset"] = {"load_layout_ok": reset.ok, "load_layout_detail": reset.detail, "verify_ok": verify.ok,
                            "verify": {k: verify.data.get(k) for k in ("parts", "tolerances", "intruders", "episode_joints")}}
        if not (reset.ok and verify.ok):
            verdict.update(reason=f"reset not verified: {reset.detail or verify.detail}")
            return 2
        before = rn.backends.scene.report_poses().data.get("poses") or {}

        if not args.no_record:
            rec = EpisodeRecording(str(ep), platform, cell).start()
        base_env = {**os.environ, "CELL_PLATFORM": args.platform, "CELL_REPO_ROOT": str(REPO),
                    "CELL_EPISODE_ID": episode_id, "CELL_EPISODE_DIR": str(ep)}
        base_env.pop("VIRTUAL_ENV", None)
        wd_proc = start_watchdog(args.watchdog, ep, base_env)

        verdict["stage"] = "planner"
        log(f"execute_task({task_path.relative_to(REPO)}) with planner {planner_dir} "
            f"(sim budget {sim_budget}s, wall guard {wall_guard}s)")
        planner_env = {k: v for k, v in base_env.items() if not k.startswith("WATCHDOG_")}
        verdict["planner"] = asyncio.run(call_planner(planner_dir, str(task_path), planner_env, wall_guard,
                                                      sim_budget if has_sim_clock else None, rn, ep / "executor" / "planner.stderr.log"))
        log(f"planner: {verdict['planner'].get('state')} ({verdict['planner'].get('outcome')}) {verdict['planner'].get('reason', '')}")

        verdict["stage"] = "episode_end"
        time.sleep(args.settle_s)
        rn.publish_episode_end(episode_id, f"planner {verdict['planner'].get('state')}")
        if wd_proc is not None:
            deadline = time.monotonic() + args.acceptance_wait_s
            while time.monotonic() < deadline and not rn.verdicts() and wd_proc.poll() is None:
                time.sleep(1.0)
        wv = rn.verdicts()
        verdict["watchdog"] = ({"ran": False} if wd_proc is None else
                               {"ran": True, "run_verdict": wv[-1] if wv else None, "run_verdicts_seen": len(wv)})

        verdict["stage"] = "check"
        after = rn.backends.scene.report_poses().data.get("poses") or {}
        verdict["checker"] = checker.check(requirements, wd_parts, before, after)
        p = verdict["planner"]
        if p.get("outcome") == "protocol_error":
            verdict["outcome"] = "invalid"
        else:
            verdict["outcome"] = "success" if (p.get("state") == "done" and verdict["checker"]["pass"]) else "failure"
        verdict["stage"] = "done"
        return 0 if verdict["outcome"] == "success" else 1
    except KeyboardInterrupt:
        verdict.update(reason="interrupted")
        try:
            rn.backends.motion.cancel_all()
        except Exception:
            pass
        return 130
    finally:
        if rec is not None:
            try:
                manifest["recording"] = rec.stop()
            except Exception as exc:  # noqa: BLE001
                manifest["recording"] = {"error": str(exc)}
        if wd_proc is not None:
            manifest["watchdog_exit_code"] = stop_process(wd_proc, grace_s=30)
        verdict["ended_wall_time_iso"] = utc_iso()
        (ep / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str))
        (ep / "verdict.json").write_text(json.dumps(verdict, indent=2, default=str))
        c = verdict.get("checker") or {}
        log(f"outcome {verdict['outcome']}: planner {(verdict.get('planner') or {}).get('state')}, "
            f"checker {c.get('parts_placed', '-')}/{c.get('parts_required', '-')} placed, pass={c.get('pass')}; {ep}")
        rn.shutdown()


if __name__ == "__main__":
    sys.exit(main())
