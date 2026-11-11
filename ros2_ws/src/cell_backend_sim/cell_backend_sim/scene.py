"""Sim scene backend: the Kit-side cell command server (Milestone 6).

Transport: loopback TCP, one newline-terminated JSON request per connection,
one JSON line back (`scene.host`, `scene.port`). Until the command server
exists (port 0 in platforms/sim.yaml), reset and verify_reset return
ok=False with a clear detail instead of pretending.

health() checks what can be checked from ROS alone: the sim clock advancing
and joint states fresh, against config/cell.yaml `health.*` limits. A limit
left at 0 (TODO(mohammad)) marks that check `configured: false` and it does
not count against `ok`.
"""

from __future__ import annotations

import json
import socket
import threading
from typing import Any

from cell_platform_interfaces.interfaces import SceneBackend
from cell_platform_interfaces.platform_config import PlatformConfig
from cell_platform_interfaces.timebase import wall_ns, wall_time_iso
from cell_platform_interfaces.types import HealthReport, SceneResult


class CommandServerClient:
    """Tiny client for the newline-JSON command server. Written from scratch;
    the loopback-command-server idea is credited to DROIDSTACK in PROVENANCE.md."""

    def __init__(self, host: str, port: int, timeout_s: float = 10.0) -> None:
        self.host = host
        self.port = int(port)
        self.timeout_s = timeout_s

    @property
    def configured(self) -> bool:
        return self.port > 0

    def call(self, command: str, timeout_s: float | None = None, **params: Any) -> dict[str, Any]:
        if not self.configured:
            return {"ok": False, "error": "command server port not configured (scene.port, Milestone 6)"}
        request = {"command": command, "params": params, "wall_time_iso": wall_time_iso()}
        payload = (json.dumps(request) + "\n").encode("utf-8")
        try:
            with socket.create_connection((self.host, self.port), timeout=timeout_s or self.timeout_s) as sock:
                sock.sendall(payload)
                sock.shutdown(socket.SHUT_WR)
                chunks: list[bytes] = []
                while True:
                    chunk = sock.recv(65536)
                    if not chunk:
                        break
                    chunks.append(chunk)
                    if b"\n" in chunk:
                        break
            line = b"".join(chunks).split(b"\n", 1)[0]
            response = json.loads(line.decode("utf-8")) if line else {}
            if not isinstance(response, dict):
                return {"ok": False, "error": "command server returned a non-object"}
            return response
        except (OSError, ValueError) as exc:
            return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}


class SimScene(SceneBackend):
    def __init__(self, node: Any, platform: PlatformConfig, callback_group: Any = None, cell_config: dict | None = None) -> None:
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState

        self.node = node
        self.platform = platform
        self.health_cfg = dict((cell_config or {}).get("health", {}) or {})
        self.client = CommandServerClient(str(platform.get_optional("scene", "host", default="127.0.0.1")), int(platform.get_optional("scene", "port", default=0)))
        self._lock = threading.Lock()
        self._clock_ns = 0
        self._clock_recv_ns = 0
        self._clock_prev_ns = 0
        self._js_recv_ns = 0
        self._clock_sub = node.create_subscription(Clock, platform.topic("clock"), self._on_clock, 10, callback_group=callback_group)
        self._js_sub = node.create_subscription(JointState, platform.topic("joint_states"), self._on_js, 10, callback_group=callback_group)

    def _on_clock(self, msg: Any) -> None:
        with self._lock:
            self._clock_prev_ns = self._clock_ns
            self._clock_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)
            self._clock_recv_ns = wall_ns()

    def _on_js(self, _msg: Any) -> None:
        with self._lock:
            self._js_recv_ns = wall_ns()

    # -- commands ---------------------------------------------------------------

    def reset(self, layout: dict[str, Any] | None = None, timeout_s: float | None = None) -> SceneResult:
        params: dict[str, Any] = {}
        if layout:
            params["layout"] = layout
            command = "load_layout"
        else:
            command = "reset"
        response = self.client.call(command, timeout_s=timeout_s, **params)
        return SceneResult(bool(response.get("ok", False)), detail=str(response.get("error", response.get("detail", ""))), data=response)

    def verify_reset(self, tolerances: dict[str, Any], layout: dict[str, Any] | None = None) -> SceneResult:
        response = self.client.call("verify_reset", tolerances=tolerances, layout=layout or {})
        return SceneResult(bool(response.get("ok", False)), detail=str(response.get("error", response.get("detail", ""))), data=response)

    def load_layout(self, assembly: str, layout: str, seed: int, timeout_s: float | None = None) -> SceneResult:
        response = self.client.call("load_layout", timeout_s=timeout_s, assembly=assembly, layout=layout, seed=int(seed))
        return SceneResult(bool(response.get("ok", False)), detail=str(response.get("error", response.get("detail", ""))), data=response)

    def reload_stage(self, timeout_s: float | None = None) -> SceneResult:
        response = self.client.call("reload_stage", timeout_s=timeout_s)
        return SceneResult(bool(response.get("ok", False)), detail=str(response.get("error", response.get("detail", ""))), data=response)

    def report_poses(self) -> SceneResult:
        response = self.client.call("report_poses")
        return SceneResult(bool(response.get("ok", False)), detail=str(response.get("error", "")), data=response)

    # -- health -----------------------------------------------------------------

    def health(self) -> HealthReport:
        now = wall_ns()
        with self._lock:
            clock_ns, clock_recv, clock_prev, js_recv = self._clock_ns, self._clock_recv_ns, self._clock_prev_ns, self._js_recv_ns
        checks: dict[str, dict[str, Any]] = {}
        ok = True

        stall_limit = float(self.health_cfg.get("sim_clock_max_stall_s", 0.0) or 0.0)
        clock_age = (now - clock_recv) / 1e9 if clock_recv else None
        advancing = clock_ns > clock_prev
        checks["sim_clock"] = {"configured": stall_limit > 0, "last_message_age_s": clock_age, "advancing": advancing, "sim_time_s": clock_ns / 1e9}
        if stall_limit > 0:
            passed = clock_age is not None and clock_age <= stall_limit and advancing
            checks["sim_clock"]["ok"] = passed
            ok = ok and passed

        js_limit = float(self.health_cfg.get("joint_states_max_age_s", 0.0) or 0.0)
        js_age = (now - js_recv) / 1e9 if js_recv else None
        checks["joint_states"] = {"configured": js_limit > 0, "last_message_age_s": js_age}
        if js_limit > 0:
            passed = js_age is not None and js_age <= js_limit
            checks["joint_states"]["ok"] = passed
            ok = ok and passed

        if self.client.configured:
            response = self.client.call("health", timeout_s=2.0)
            passed = bool(response.get("ok", False))
            checks["command_server"] = {"configured": True, "ok": passed, "response": response}
            ok = ok and passed
        else:
            checks["command_server"] = {"configured": False, "detail": "scene.port not set (Milestone 6)"}
        return HealthReport(ok=ok, checks=checks)

    def shutdown(self) -> None:
        for sub in (self._clock_sub, self._js_sub):
            try:
                self.node.destroy_subscription(sub)
            except Exception:
                pass
