"""Run one episode: reset, verify, record, (WatchDog), execute_task under budget,
episode_end, acceptance wait, ground-truth check, manifest.json, verdict.json.

    runner/episode_runner/run.sh --budget-sim-s 900 --wall-guard-s 1800
    runner/episode_runner/run.sh --planner executor/planner_templates/plan_given --seed 3 --watchdog plan_blind

Layout of records/<episode_id>/:
    manifest.json      every input hashed (task, requirements, parts, layout,
                       platform, cell config, planner tree), git state, versions
    verdict.json       planner completion, budget use, checker result,
                       WatchDog run verdict (when a session ran), outcome
    runner.jsonl       the runner's own events (stage changes, planner result,
                       outcome), one JSON object per line
    bag/, frames/, recording.json     the episode recording (M6);
                       bag.stderr.jsonl, frames.stderr.jsonl: the recorder
                       processes' stderr, one line per record
    executor/          the planner's own directory: planner.jsonl (its log),
                       planner.stderr.jsonl (raw stderr of the planner process)
    watchdog/          the WatchDog session directory and its transcript.jsonl

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
from runner.jsonl import JsonlLog, StderrPump, utc_iso  # noqa: E402

_EVENTS: JsonlLog | None = None


def log(msg: str, event: str = "log", **fields: Any) -> None:
    """One line to the terminal; one JSON record to records/<episode>/runner.jsonl once it exists."""
    print(f"[episode] {msg}", file=sys.stderr, flush=True)
    if _EVENTS is not None:
        _EVENTS.write(event, msg=msg, **fields)


def stage(verdict: dict[str, Any], name: str) -> None:
    verdict["stage"] = name
    if _EVENTS is not None:
        _EVENTS.write("stage", stage=name)


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
        # What the WatchDog said during the episode: halts, rulings, consultations.
        self.watchdog_said: list[dict[str, Any]] = []
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
        if not isinstance(env, dict):
            return
        kind = env.get("event_type")
        if kind == "run_verdict":
            with self._lock:
                self.run_verdicts.append(env)
        elif kind in ("halt", "ruling", "consultation"):
            with self._lock:
                self.watchdog_said.append({"kind": kind, "wall_time_iso": env.get("wall_time_iso"), **(env.get("payload") or {})})

    def sim_now(self) -> float | None:
        with self._lock:
            return self.sim_time_s

    def verdicts(self) -> list[dict[str, Any]]:
        with self._lock:
            return list(self.run_verdicts)

    def said(self) -> list[dict[str, Any]]:
        with self._lock:
            return list(self.watchdog_said)

    def escalation(self) -> dict[str, Any] | None:
        """The first `escalate` ruling of this episode, if any."""
        with self._lock:
            return next((x for x in self.watchdog_said if x["kind"] == "ruling" and x.get("ruling") == "escalate"), None)

    def declare_episode(self, episode_id: str, mode: str, timeout_s: float = 5.0) -> dict[str, Any]:
        """Declare the episode to cell_orchestrator: episode_id (a new id clears
        the previous episode's readiness, abort and latch) and guard_mode
        (guarded with a WatchDog session; unguarded without, so the start
        permission is immediate and recorded as unguarded_by_config)."""
        from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters

        name = self.platform.service("orchestrator_parameters")
        client = self.node.create_client(SetParameters, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout_s):
                return {"ok": False, "mode": mode, "detail": f"{name} not available"}
            req = SetParameters.Request()
            req.parameters = [ParamMsg(name=k, value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=v))
                              for k, v in (("episode_id", episode_id), ("guard_mode", mode))]
            fut = client.call_async(req)
            deadline = time.monotonic() + timeout_s
            while not fut.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            if not fut.done() or fut.result() is None:
                return {"ok": False, "mode": mode, "detail": "no response"}
            bad = [r.reason for r in fut.result().results if not r.successful]
            return {"ok": not bad, "mode": mode, "episode_id": episode_id, "detail": "; ".join(bad)}
        finally:
            self.node.destroy_client(client)

    def request_start(self, episode_id: str, timeout_s: float) -> dict[str, Any]:
        """The episode's start permission from cell_orchestrator (request_go):
        returns once the WatchDog has signalled ready, at once when guard_mode
        is unguarded, or after timeout_s (the gate never refuses)."""
        from cell_sync_interfaces.srv import RequestGo

        name = self.platform.service("request_go")
        client = self.node.create_client(RequestGo, name)
        t0 = time.monotonic()
        try:
            if not client.wait_for_service(timeout_sec=10.0):
                return {"ok": False, "reason": f"{name} not available"}
            req = RequestGo.Request()
            req.step_id, req.phase, req.timeout_sec = episode_id, "run", float(timeout_s)
            fut = client.call_async(req)
            deadline = t0 + timeout_s + 15.0
            while not fut.done() and time.monotonic() < deadline:
                time.sleep(0.1)
            res = fut.result() if fut.done() else None
            if res is None:
                return {"ok": False, "reason": "no response", "waited_s": round(time.monotonic() - t0, 2)}
            return {"ok": True, "proceed": bool(res.proceed), "timed_out": bool(res.timed_out), "reason": str(res.reason),
                    "waited_s": round(time.monotonic() - t0, 2)}
        finally:
            self.node.destroy_client(client)

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
        # Stop spinning and join the thread before the context goes away, or the
        # executor thread aborts the process ("terminate called without an active exception").
        try:
            self.executor.shutdown(timeout_sec=2.0)
        except Exception:
            pass
        self._thread.join(timeout=3.0)
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.try_shutdown()
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
                       sim_budget_s: float | None, rn: RunnerNode, errlog: Any, honour_escalation: bool = False) -> dict[str, Any]:
    from mcp import ClientSession, StdioServerParameters
    from mcp.client.stdio import stdio_client

    out: dict[str, Any] = {"started_wall_time_iso": utc_iso()}
    run_sh = planner_dir / "run.sh"
    if not os.access(run_sh, os.X_OK):
        return {**out, "outcome": "protocol_error", "reason": f"{run_sh} is missing or not executable"}
    params = StdioServerParameters(command=str(run_sh), args=[], env=env, cwd=str(REPO))
    t0, s0 = time.monotonic(), rn.sim_now()
    stop = None
    async with stdio_client(params, errlog=errlog) as (read, write):
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
                if honour_escalation and rn.escalation() is not None:
                    stop = "watchdog_escalate"
                    out["escalation"] = rn.escalation()
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
    log(f"watchdog session {kind} (model={model} effort={effort})", event="watchdog_start", model=model, effort=effort)
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
    ap.add_argument("--planner", default="executor/planner_templates/plan_given")
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

    # One episode at a time on the cell (an authoring session and a manual run
    # must not drive the same arm). Held until the process exits.
    import fcntl

    (REPO / "records").mkdir(exist_ok=True)
    lock = open(REPO / "records" / ".cell.lock", "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        raise SystemExit("another episode is running on the cell (records/.cell.lock is held)")

    episode_id = args.episode_id or f"{args.assembly}-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}-s{args.seed}"
    ep = REPO / "records" / episode_id
    if ep.exists():
        raise SystemExit(f"{ep} exists; pick another --episode-id")
    for sub in ("executor", "watchdog"):
        (ep / sub).mkdir(parents=True)
    global _EVENTS
    _EVENTS = JsonlLog(ep / "runner.jsonl")
    log(f"episode {episode_id} -> {ep}", event="episode_start", episode_id=episode_id, planner=args.planner,
        task=args.task, seed=args.seed, platform=args.platform, watchdog=args.watchdog)

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
        # DDS discovery takes a few seconds before the first /clock arrives.
        deadline = time.monotonic() + 20.0
        while has_sim_clock and rn.sim_now() is None and time.monotonic() < deadline:
            time.sleep(0.2)
        health = rn.backends.scene.health()
        manifest["scene_health"] = {"ok": health.ok, "checks": health.checks}
        cs = (health.checks.get("command_server") or {}).get("response") or {}
        manifest["versions"] = cs.get("version")
        if not health.ok or (has_sim_clock and rn.sim_now() is None):
            failing = {name: (c.get("response") or {}).get("error") or {k: v for k, v in c.items() if k != "response"}
                       for name, c in health.checks.items() if isinstance(c, dict) and c.get("ok") is False}
            if has_sim_clock and rn.sim_now() is None:
                failing["sim_clock"] = "no /clock received within 20 s"
            hint = (" (is the Isaac cell command server running? isaac/cell_command_server/start_in_script_editor.py "
                    "in the Script Editor, or isaac/headless/run_headless.sh)") if "command_server" in failing else ""
            verdict.update(stage="health", reason=f"cell unhealthy: {failing}{hint}", failing_checks=failing)
            log(f"health check failed: {failing}{hint}", event="health_failed", failing=failing)
            return 2

        stage(verdict, "reset")
        reset = rn.backends.scene.load_layout(args.assembly, args.layout, args.seed, timeout_s=90)
        time.sleep(args.settle_s)
        verify = rn.backends.scene.verify_reset({})
        verdict["reset"] = {"load_layout_ok": reset.ok, "load_layout_detail": reset.detail, "verify_ok": verify.ok,
                            "verify": {k: verify.data.get(k) for k in ("parts", "tolerances", "intruders", "episode_joints")}}
        if not (reset.ok and verify.ok):
            verdict.update(reason=f"reset not verified: {reset.detail or verify.detail}")
            return 2
        before = rn.backends.scene.report_poses().data.get("poses") or {}

        # Placements come from config/placements/<assembly>.yaml relative to the
        # fixture (cell_platform_interfaces.placements), not from published frames.
        from cell_platform_interfaces.placements import derive_into, load_placements, load_symmetries

        placements = load_placements(REPO, args.assembly)
        symmetries = load_symmetries(REPO, args.assembly)
        manifest["inputs"]["placements"] = file_record(REPO / "config" / "placements" / f"{args.assembly}.yaml", REPO)
        derive_into(before, placements)

        # A plan-free task must not see where parts go: no placement frame may
        # be visible on the executor's pose topic (camera1).
        task_doc = yaml.safe_load(task_path.read_text()) or {}
        # Executor-baseline tasks (task_data/, the labmate's droidstack.task schema) carry the
        # assembly graph's seats by design; only the M7 plan-free task hides where parts go.
        baseline_task = str(task_doc.get("schema", "")).startswith("droidstack.task")
        task_ref = str(task_doc["task_id"]) if baseline_task else str(task_path)
        task_data_env = {"CELL_TASK_DATA": str(task_path.parents[2])} if baseline_task else {}
        if not task_doc.get("steps") and not baseline_task:
            stage(verdict, "placement_leak_check")
            frames = [str(spec.get("placement_frame")) for spec in ((requirements.get("acceptance") or {}).get("parts") or {}).values() if spec]
            deadline = time.monotonic() + 3.0
            seen: list[str] = []
            while time.monotonic() < deadline and not seen:
                seen = [f for f in frames if rn.backends.executor_poses.latest(f, max_age_s=2.0) is not None]
                time.sleep(0.2)
            verdict["placement_leak_check"] = {"frames": frames, "visible_to_executor": seen}
            if seen:
                verdict.update(reason=f"plan-free task, but placement frames are published on the executor's pose topic: {seen} "
                                      "(stop publishing them from Isaac)")
                return 2

        if not args.no_record:
            rec = EpisodeRecording(str(ep), platform, cell).start()
        base_env = {**os.environ, **task_data_env, "CELL_PLATFORM": args.platform, "CELL_REPO_ROOT": str(REPO),
                    "CELL_EPISODE_ID": episode_id, "CELL_EPISODE_DIR": str(ep)}
        base_env.pop("VIRTUAL_ENV", None)
        guard = rn.declare_episode(episode_id, "unguarded" if args.watchdog == "none" else "guarded")
        manifest["guard_mode"] = guard
        log(f"guard_mode {guard['mode']} ({'ok' if guard['ok'] else guard['detail']})", event="guard_mode", **guard)
        if not guard["ok"]:
            verdict.update(stage="guard_mode", reason=f"could not declare guard_mode {guard['mode']} to the orchestrator: {guard['detail']}")
            return 2
        wd_proc = start_watchdog(args.watchdog, ep, {k: v for k, v in base_env.items() if k != "CELL_TASK_DATA"})

        # Start permission: the WatchDog prepares while nothing moves, then
        # signals ready; physics wakes it from then on. The planner is not
        # started before this, and the budgets start after it.
        stage(verdict, "start_permission")
        ready_timeout = float((cell.get("watchdog") or {}).get("ready_timeout_s") or 300.0)
        permission = rn.request_start(episode_id, ready_timeout if wd_proc is not None else 5.0)
        verdict["start_permission"] = permission
        log(f"start permission: {permission.get('reason')} after {permission.get('waited_s')} s",
            event="start_permission", **permission)
        if not permission.get("ok"):
            verdict.update(reason=f"no start permission: {permission.get('reason')}")
            return 2

        stage(verdict, "planner")
        log(f"execute_task({task_ref}) with planner {planner_dir} "
            f"(sim budget {sim_budget}s, wall guard {wall_guard}s)")
        planner_env = {k: v for k, v in base_env.items() if not k.startswith("WATCHDOG_")}
        pump = StderrPump(ep / "executor" / "planner.stderr.jsonl", "planner")
        try:
            verdict["planner"] = asyncio.run(call_planner(planner_dir, task_ref, planner_env, wall_guard,
                                                          sim_budget if has_sim_clock else None, rn, pump.file(),
                                                          honour_escalation=(wd_proc is not None and (cell.get("watchdog") or {}).get("mode") == "halt_enabled")))
        finally:
            pump.close()
        p = verdict["planner"]
        log(f"planner: {p.get('state')} ({p.get('outcome')}) {p.get('reason', '')}", event="planner_result",
            **{k: p.get(k) for k in ("state", "outcome", "reason", "wall_s", "sim_s")})

        stage(verdict, "episode_end")
        time.sleep(args.settle_s)
        rn.publish_episode_end(episode_id, f"planner {verdict['planner'].get('state')}")
        if wd_proc is not None:
            deadline = time.monotonic() + args.acceptance_wait_s
            while time.monotonic() < deadline and not rn.verdicts() and wd_proc.poll() is None:
                time.sleep(1.0)
        wv = rn.verdicts()
        verdict["watchdog"] = ({"ran": False} if wd_proc is None else
                               {"ran": True, "run_verdict": wv[-1] if wv else None, "run_verdicts_seen": len(wv),
                                "during_episode": rn.said()})

        stage(verdict, "check")
        after = rn.backends.scene.report_poses().data.get("poses") or {}
        verdict["placements_derived"] = derive_into(after, placements)
        verdict["checker"] = checker.check(requirements, wd_parts, before, after, symmetries)
        p = verdict["planner"]
        if p.get("outcome") == "protocol_error":
            verdict["outcome"] = "invalid"
        else:
            verdict["outcome"] = "success" if (p.get("state") == "done" and verdict["checker"]["pass"]) else "failure"
        stage(verdict, "done")
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
            f"checker {c.get('parts_placed', '-')}/{c.get('parts_required', '-')} placed, pass={c.get('pass')}"
            + (f"; stopped at {verdict.get('stage')}: {verdict.get('reason')}" if verdict["outcome"] == "invalid" else "") + f"; {ep}",
            event="episode_end", outcome=verdict["outcome"], stage=verdict.get("stage"), reason=verdict.get("reason"))
        rn.shutdown()


if __name__ == "__main__":
    sys.exit(main())
