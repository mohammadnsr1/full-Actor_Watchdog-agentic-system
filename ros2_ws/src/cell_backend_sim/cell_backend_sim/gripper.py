"""Sim gripper backend over the RG2 controller (isaac_trajectory_bridge).

Commands
    open / close      std_srvs/Trigger services `gripper.services.open|close`
    set_width, close(force)
                      GripperGuardedMove action `gripper.action.guarded_move`
                      (mode "width" without a force, "guarded" with one)
Readings (declared in `gripper.readings`)
    width             Float64 on `topics.gripper_width_true`  -> ground_truth
    effort            `joints.gripper_effort` effort on joint_states -> joint_state
    grip_detected     stop_reason of the last guarded move in the kinematics
                      file's contact.stop_reasons -> proxy (no live signal)

The RG2 constants are not used here: the controller owns them. The
kinematics file is read only for the stop-reason set.
"""

from __future__ import annotations

import threading
import time
from typing import Any

import yaml

from cell_platform_interfaces.interfaces import GripperBackend
from cell_platform_interfaces.platform_config import PlatformConfig
from cell_platform_interfaces.timebase import wall_ns
from cell_platform_interfaces.types import GripperReading, GripperResult


class SimRG2Gripper(GripperBackend):
    def __init__(self, node: Any, platform: PlatformConfig, callback_group: Any = None, server_wait_s: float = 2.0) -> None:
        from isaac_trajectory_bridge_interfaces.action import GripperGuardedMove
        from rclpy.action import ActionClient
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64
        from std_srvs.srv import Trigger

        self.node = node
        self.platform = platform
        self.server_wait_s = server_wait_s
        self._cb = callback_group
        self._Trigger = Trigger
        self._Guarded = GripperGuardedMove
        self._open_client = node.create_client(Trigger, platform.get("gripper", "services", "open"), callback_group=callback_group)
        self._close_client = node.create_client(Trigger, platform.get("gripper", "services", "close"), callback_group=callback_group)
        self._guarded = ActionClient(node, GripperGuardedMove, platform.get("gripper", "action", "guarded_move"), callback_group=callback_group)
        self._readings = {k: str(v) for k, v in platform.section("gripper", "readings").items()}
        kin_path = platform.resolve_path(str(platform.get("gripper", "kinematics")))
        with kin_path.open("r", encoding="utf-8") as handle:
            kin = yaml.safe_load(handle) or {}
        self._contact_stop_reasons = frozenset(str(r) for r in (kin.get("contact", {}) or {}).get("stop_reasons", []))
        self._effort_joint = platform.joints("gripper_effort")[0]
        self._lock = threading.Lock()
        self._width: float | None = None
        self._width_ns = 0
        self._effort: float | None = None
        self._effort_ns = 0
        self._last_stop_reason = ""
        self._width_sub = node.create_subscription(Float64, platform.topic("gripper_width_true"), self._on_width, 10, callback_group=callback_group)
        self._js_sub = node.create_subscription(JointState, platform.topic("joint_states"), self._on_js, 50, callback_group=callback_group)

    # -- readings -------------------------------------------------------------

    def _on_width(self, msg: Any) -> None:
        with self._lock:
            self._width = float(msg.data)
            self._width_ns = wall_ns()

    def _on_js(self, msg: Any) -> None:
        try:
            idx = list(msg.name).index(self._effort_joint)
        except ValueError:
            return
        if idx < len(msg.effort):
            with self._lock:
                self._effort = float(msg.effort[idx])
                self._effort_ns = wall_ns()

    def read(self) -> GripperReading:
        with self._lock:
            width, effort, reason = self._width, self._effort, self._last_stop_reason
        grip = None
        grip_source = "missing"
        if self._readings.get("grip_detected", "missing") != "missing" and reason:
            grip = reason in self._contact_stop_reasons
            grip_source = "proxy:guarded_move_stop_reason"
        return GripperReading(
            width_m=width if self._readings.get("width", "missing") != "missing" else None,
            effort=effort if self._readings.get("effort", "missing") != "missing" else None,
            grip_detected=grip,
            width_source=self._readings.get("width", "missing") if width is not None else "missing",
            effort_source=self._readings.get("effort", "missing") if effort is not None else "missing",
            grip_detected_source=grip_source,
        )

    @property
    def provided_readings(self) -> dict[str, str]:
        return dict(self._readings)

    # -- commands ---------------------------------------------------------------

    def open(self, timeout_s: float | None = None) -> GripperResult:
        return self._trigger(self._open_client, "open", timeout_s)

    def close(self, force: float | None = None, timeout_s: float | None = None) -> GripperResult:
        if force is None:
            return self._trigger(self._close_client, "close", timeout_s)
        return self._guarded_move(0.0, force, "guarded", timeout_s)

    def set_width(self, width_m: float, force: float | None = None, timeout_s: float | None = None) -> GripperResult:
        mode = "width" if force is None else "guarded"
        return self._guarded_move(float(width_m), force, mode, timeout_s)

    def _trigger(self, client: Any, label: str, timeout_s: float | None) -> GripperResult:
        if not client.wait_for_service(timeout_sec=self.server_wait_s):
            return GripperResult(False, stop_reason="unavailable", detail=f"{label} service not available")
        response = _wait(client.call_async(self._Trigger.Request()), timeout_s or 30.0)
        if response is None:
            return GripperResult(False, stop_reason="timeout", detail=f"{label} did not answer")
        reading = self.read()
        return GripperResult(bool(response.success), stop_reason=label, final_width_m=reading.width_m, final_effort=reading.effort, detail=str(response.message))

    def _guarded_move(self, width_m: float, force: float | None, mode: str, timeout_s: float | None) -> GripperResult:
        if not self._guarded.wait_for_server(timeout_sec=self.server_wait_s):
            return GripperResult(False, stop_reason="unavailable", detail="guarded_move action not available")
        goal = self._Guarded.Goal()
        goal.target_width_m = float(width_m)
        goal.mode = mode
        if force is not None:
            goal.force_threshold = float(force)
        handle = _wait(self._guarded.send_goal_async(goal), self.server_wait_s + 5.0)
        if handle is None or not handle.accepted:
            return GripperResult(False, stop_reason="rejected", detail="guarded_move goal rejected (controller busy?)")
        wrapped = _wait(handle.get_result_async(), timeout_s or 60.0)
        if wrapped is None:
            return GripperResult(False, stop_reason="timeout", detail="guarded_move gave no result")
        result = wrapped.result
        with self._lock:
            self._last_stop_reason = str(result.stop_reason)
        return GripperResult(
            bool(result.success),
            stop_reason=str(result.stop_reason),
            final_width_m=float(result.final_width_m),
            final_effort=float(result.final_effort),
            detail=str(result.message),
        )

    def shutdown(self) -> None:
        for sub in (self._width_sub, self._js_sub):
            try:
                self.node.destroy_subscription(sub)
            except Exception:
                pass


def _wait(future: Any, timeout_s: float) -> Any:
    start = time.monotonic()
    while not future.done():
        if time.monotonic() - start > timeout_s:
            return None
        time.sleep(0.005)
    return future.result()
