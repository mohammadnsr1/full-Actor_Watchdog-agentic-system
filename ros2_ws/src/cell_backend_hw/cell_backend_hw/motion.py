"""Hardware motion backend: the UR ROS 2 driver's trajectory controller.

Same FollowJointTrajectory client as sim; the action and its cancel service
come from platforms/hardware.yaml (`motion.action`, `motion.cancel_service`),
which still carry TODO(mohammad) confirmations against the installed driver
branch (BUILD_PLAN q4). A cancel on the UR driver's controller stops the
trajectory; whether the controller holds position afterwards is the
driver's behaviour, not ours, and must be checked on the real cell.

driver_status() reads the driver's robot/safety mode topics when the
ur_dashboard_msgs package is importable; otherwise it reports them missing.
Nothing here starts, stops, or re-enables the robot program.
"""

from __future__ import annotations

import threading
from typing import Any

from cell_platform_interfaces.fjt_motion import FollowJointTrajectoryMotion
from cell_platform_interfaces.platform_config import PlatformConfig
from cell_platform_interfaces.timebase import wall_ns, wall_time_iso


class HardwareMotion(FollowJointTrajectoryMotion):
    def __init__(self, node: Any, platform: PlatformConfig, callback_group: Any = None, server_wait_s: float = 2.0) -> None:
        super().__init__(node, platform, callback_group, server_wait_s)
        self._status_lock = threading.Lock()
        self._robot_mode: int | None = None
        self._safety_mode: int | None = None
        self._program_running: bool | None = None
        self._status_recv_ns = 0
        self._status_subs: list[Any] = []
        self._status_available = False
        try:
            from std_msgs.msg import Bool
            from ur_dashboard_msgs.msg import RobotMode, SafetyMode  # type: ignore[import]
        except Exception:
            node.get_logger().warning("ur_dashboard_msgs not importable: driver status topics are not read")
            return
        try:
            self._status_subs.append(node.create_subscription(RobotMode, platform.get("motion", "robot_mode_topic"), self._on_robot_mode, 10, callback_group=callback_group))
            self._status_subs.append(node.create_subscription(SafetyMode, platform.get("motion", "safety_mode_topic"), self._on_safety_mode, 10, callback_group=callback_group))
            self._status_subs.append(node.create_subscription(Bool, platform.get("motion", "robot_program_running_topic"), self._on_program_running, 10, callback_group=callback_group))
            self._status_available = True
        except Exception as exc:  # a TODO(mohammad) name left empty raises MissingPlatformValue
            node.get_logger().warning(f"driver status topics not subscribed: {exc}")

    def _on_robot_mode(self, msg: Any) -> None:
        with self._status_lock:
            self._robot_mode = int(msg.mode)
            self._status_recv_ns = wall_ns()

    def _on_safety_mode(self, msg: Any) -> None:
        with self._status_lock:
            self._safety_mode = int(msg.mode)
            self._status_recv_ns = wall_ns()

    def _on_program_running(self, msg: Any) -> None:
        with self._status_lock:
            self._program_running = bool(msg.data)
            self._status_recv_ns = wall_ns()

    def driver_status(self) -> dict[str, Any]:
        with self._status_lock:
            return {
                "available": self._status_available,
                "robot_mode": self._robot_mode,
                "safety_mode": self._safety_mode,
                "program_running": self._program_running,
                "age_s": (wall_ns() - self._status_recv_ns) / 1e9 if self._status_recv_ns else None,
                "wall_time_iso": wall_time_iso(),
            }

    def shutdown(self) -> None:
        super().shutdown()
        for sub in self._status_subs:
            try:
                self.node.destroy_subscription(sub)
            except Exception:
                pass
