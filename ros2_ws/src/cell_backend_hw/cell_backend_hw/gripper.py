"""Hardware gripper backend: OnRobot RG6 through the URCap, over URScript.

Mohammad's answer (BUILD_PLAN q3): the RG6 is driven today with URScript
calls into the OnRobot URCap; no ROS 2 gripper driver exists. This backend
therefore has one transport, `urscript`: it publishes a script line on the
UR driver's script topic (`topics.urscript_command`, std_msgs/String). The
URCap function names and argument order are TODO(mohammad) in
platforms/hardware.yaml `gripper.urscript.*`; until they are filled every
command returns ok=False with that reason. Nothing is guessed.

Readings: none are available over ROS 2 on this path, so read() returns
None for width, effort and grip_detected with source 'missing', exactly as
`gripper.readings` declares. If a telemetry path appears later (URCap
registers through the driver's I/O, or a ROS 2 RG6 driver), add a second
transport and update the readings block; never fake a value here.

Script lines are built from the configured function name and the numeric
arguments only; no free-form script text is accepted from callers.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from cell_platform_interfaces.interfaces import GripperBackend
from cell_platform_interfaces.platform_config import MissingPlatformValue, PlatformConfig
from cell_platform_interfaces.types import GripperReading, GripperResult


class URScriptRG6Gripper(GripperBackend):
    def __init__(self, node: Any, platform: PlatformConfig, callback_group: Any = None) -> None:
        from std_msgs.msg import String

        self.node = node
        self.platform = platform
        self._String = String
        self._lock = threading.Lock()
        self._readings = {k: str(v) for k, v in platform.section("gripper", "readings").items()}
        self.transport = str(platform.get_optional("gripper", "transport", default=""))
        self._pub: Any = None
        if self.transport == "urscript":
            try:
                self._pub = node.create_publisher(String, platform.topic("urscript_command"), 10)
            except MissingPlatformValue as exc:
                node.get_logger().warning(f"RG6 URScript transport not configured: {exc}")
        else:
            node.get_logger().warning(f"RG6 transport {self.transport!r} is not implemented; commands will fail")
        self._width_range = [float(v) for v in platform.get_optional("gripper", "width_range_m", default=[0.0, 0.0])]
        self._force_range = [float(v) for v in platform.get_optional("gripper", "force_range_n", default=[0.0, 0.0])]

    # -- readings -------------------------------------------------------------

    def read(self) -> GripperReading:
        return GripperReading(width_m=None, effort=None, grip_detected=None)

    @property
    def provided_readings(self) -> dict[str, str]:
        return dict(self._readings)

    # -- commands ---------------------------------------------------------------

    def open(self, timeout_s: float | None = None) -> GripperResult:
        return self._send("release_function", [], "open")

    def close(self, force: float | None = None, timeout_s: float | None = None) -> GripperResult:
        return self._grip(0.0, force, "close")

    def set_width(self, width_m: float, force: float | None = None, timeout_s: float | None = None) -> GripperResult:
        return self._grip(float(width_m), force, "set_width")

    def _grip(self, width_m: float, force: float | None, label: str) -> GripperResult:
        if self._width_range[1] <= 0.0:
            return GripperResult(False, stop_reason="unconfigured", detail="gripper.width_range_m is TODO(mohammad)")
        if not (self._width_range[0] <= width_m <= self._width_range[1]):
            return GripperResult(False, stop_reason="rejected", detail=f"width {width_m} outside {self._width_range}")
        args: list[float] = [width_m]
        if force is not None:
            if self._force_range[1] <= 0.0:
                return GripperResult(False, stop_reason="unconfigured", detail="gripper.force_range_n is TODO(mohammad)")
            if not (self._force_range[0] <= force <= self._force_range[1]):
                return GripperResult(False, stop_reason="rejected", detail=f"force {force} outside {self._force_range}")
            args.append(float(force))
        return self._send("grip_function", args, label)

    def _send(self, function_key: str, args: list[float], label: str) -> GripperResult:
        if self._pub is None:
            return GripperResult(False, stop_reason="unavailable", detail="URScript transport not configured")
        function = str(self.platform.get_optional("gripper", "urscript", function_key, default=""))
        if not function:
            return GripperResult(False, stop_reason="unconfigured", detail=f"gripper.urscript.{function_key} is TODO(mohammad)")
        tool_index = int(self.platform.get_optional("gripper", "urscript", "tool_index", default=0))
        rendered = ", ".join(f"{a:.4f}" for a in args)
        if rendered:
            rendered = f"{rendered}, {tool_index}"
        else:
            rendered = f"{tool_index}"
        line = f"{function}({rendered})"
        msg = self._String()
        msg.data = line
        with self._lock:
            self._pub.publish(msg)
        # The URCap gives no completion signal over ROS 2: report that the
        # script was sent, not that the grip finished.
        return GripperResult(True, stop_reason=f"{label}_sent", detail=f"sent URScript: {line}; no completion feedback available")

    def shutdown(self) -> None:
        if self._pub is not None:
            try:
                self.node.destroy_publisher(self._pub)
            except Exception:
                pass


def wait_s(seconds: float) -> None:
    time.sleep(seconds)
