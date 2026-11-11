"""Hardware scene backend: the operator resets the scene.

reset()        prints the layout to reset to and waits for the operator's
               confirmation (terminal prompt; `scene.confirm` in
               platforms/hardware.yaml names the mechanism, terminal is the
               only one implemented). Never moves the robot.
verify_reset() compares WatchDog's camera2 pose observations with the layout
               within the given tolerances. With no poses published it
               reports the gap as missing evidence (ok=False, reason given).
health()       joint states fresh (config/cell.yaml health.joint_states_max_age_s)
               and, when readable, the UR driver status through the motion
               backend's driver_status().
"""

from __future__ import annotations

import math
import sys
import threading
from typing import Any

from cell_platform_interfaces.interfaces import PoseSource, SceneBackend
from cell_platform_interfaces.platform_config import PlatformConfig
from cell_platform_interfaces.timebase import wall_ns, wall_time_iso
from cell_platform_interfaces.types import HealthReport, SceneResult


class OperatorScene(SceneBackend):
    def __init__(self, node: Any, platform: PlatformConfig, watchdog_poses: PoseSource, callback_group: Any = None, cell_config: dict | None = None, prompt_in=None, prompt_out=None) -> None:
        from sensor_msgs.msg import JointState

        self.node = node
        self.platform = platform
        self.poses = watchdog_poses
        self.health_cfg = dict((cell_config or {}).get("health", {}) or {})
        self.confirm_mode = str(platform.get_optional("scene", "confirm", default="terminal"))
        self._in = prompt_in or sys.stdin
        self._out = prompt_out or sys.stdout
        self._lock = threading.Lock()
        self._js_recv_ns = 0
        self.motion: Any = None  # set by the runner when it wants driver status in health()
        self._js_sub = node.create_subscription(JointState, platform.topic("joint_states"), self._on_js, 10, callback_group=callback_group)

    def _on_js(self, _msg: Any) -> None:
        with self._lock:
            self._js_recv_ns = wall_ns()

    def reset(self, layout: dict[str, Any] | None = None, timeout_s: float | None = None) -> SceneResult:
        if self.confirm_mode != "terminal":
            return SceneResult(False, detail=f"scene.confirm={self.confirm_mode!r} is not implemented (TODO(mohammad))")
        self._out.write("\n=== OPERATOR RESET REQUIRED ===\n")
        if layout:
            for part, pose in layout.items():
                self._out.write(f"  {part}: {pose}\n")
        else:
            self._out.write("  (no layout given: restore the nominal scene)\n")
        self._out.write("Place the parts, clear the workspace, then type 'done' and Enter (or 'abort'): ")
        self._out.flush()
        answer = self._in.readline().strip().lower()
        stamp = wall_time_iso()
        if answer == "done":
            return SceneResult(True, detail="operator confirmed reset", data={"confirmed_wall_time_iso": stamp})
        return SceneResult(False, detail=f"operator answered {answer!r}", data={"answer": answer, "wall_time_iso": stamp})

    def verify_reset(self, tolerances: dict[str, Any], layout: dict[str, Any] | None = None) -> SceneResult:
        if not layout:
            return SceneResult(False, detail="no layout to verify against")
        max_age = float(tolerances.get("max_pose_age_s", 1.0))
        pos_tol = tolerances.get("position_m")
        if pos_tol is None:
            return SceneResult(False, detail="tolerances.position_m is required (TODO(mohammad) in the layout file)")
        observed = self.poses.latest_all(max_age_s=max_age)
        missing: list[str] = []
        off: dict[str, float] = {}
        for part, target in layout.items():
            obs = observed.get(part)
            if obs is None:
                missing.append(part)
                continue
            want = target.get("position") if isinstance(target, dict) else None
            if not want or len(want) != 3:
                missing.append(part)
                continue
            dist = math.dist(obs.position, (float(want[0]), float(want[1]), float(want[2])))
            if dist > float(pos_tol):
                off[part] = dist
        ok = not missing and not off
        detail = "verified" if ok else f"missing evidence for {missing}; out of tolerance {off}"
        return SceneResult(ok, detail=detail, data={"missing_evidence": missing, "out_of_tolerance": off, "source": self.poses.source, "camera": self.poses.camera})

    def health(self) -> HealthReport:
        now = wall_ns()
        with self._lock:
            js_recv = self._js_recv_ns
        checks: dict[str, dict[str, Any]] = {}
        ok = True
        js_limit = float(self.health_cfg.get("joint_states_max_age_s", 0.0) or 0.0)
        js_age = (now - js_recv) / 1e9 if js_recv else None
        checks["joint_states"] = {"configured": js_limit > 0, "last_message_age_s": js_age}
        if js_limit > 0:
            passed = js_age is not None and js_age <= js_limit
            checks["joint_states"]["ok"] = passed
            ok = ok and passed
        if self.motion is not None and hasattr(self.motion, "driver_status"):
            status = self.motion.driver_status()
            checks["ur_driver"] = status
            if status.get("available") and status.get("program_running") is False:
                checks["ur_driver"]["ok"] = False
                ok = False
        else:
            checks["ur_driver"] = {"configured": False, "detail": "motion backend not attached"}
        return HealthReport(ok=ok, checks=checks)

    def shutdown(self) -> None:
        try:
            self.node.destroy_subscription(self._js_sub)
        except Exception:
            pass
