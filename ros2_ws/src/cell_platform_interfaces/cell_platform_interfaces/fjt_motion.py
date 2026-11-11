"""FollowJointTrajectory-based MotionBackend, shared by sim and hardware.

Sim: the Isaac trajectory bridge serves the action; hardware: the UR driver's
trajectory controller does. Only the names differ, and they come from the
platform config (`motion.action`, `motion.cancel_service`,
`topics.joint_states`, `joints.arm`).

Cancel semantics: `cancel` cancels this backend's own goal through its goal
handle; `cancel_all` sends an all-zero goal id to the cancel service, which
rclpy action servers interpret as "cancel every goal". A cancelled trajectory
is reported as MOTION_CANCELED, which the planner treats as an ordinary
failed action.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from cell_platform_interfaces.interfaces import MotionBackend
from cell_platform_interfaces.platform_config import PlatformConfig
from cell_platform_interfaces.timebase import wall_ns
from cell_platform_interfaces.types import (
    MOTION_ABORTED,
    MOTION_ACTIVE,
    MOTION_CANCELED,
    MOTION_IDLE,
    MOTION_REJECTED,
    MOTION_SUCCEEDED,
    MOTION_TIMEOUT,
    MOTION_UNAVAILABLE,
    CancelResult,
    JointSnapshot,
    MotionResult,
    MotionStatus,
)


class FollowJointTrajectoryMotion(MotionBackend):
    def __init__(
        self,
        node: Any,
        platform: PlatformConfig,
        callback_group: Any = None,
        server_wait_s: float = 2.0,
    ) -> None:
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from sensor_msgs.msg import JointState

        self.node = node
        self.platform = platform
        self.action_name = str(platform.get("motion", "action"))
        self._cancel_service = str(platform.get("motion", "cancel_service"))
        self.joint_state_topic = platform.topic("joint_states")
        self.arm_joints = platform.joints("arm")
        self.server_wait_s = server_wait_s
        self._FJT = FollowJointTrajectory
        self._client = ActionClient(node, FollowJointTrajectory, self.action_name, callback_group=callback_group)
        self._cancel_client: Any = None
        self._cb_group = callback_group
        self._lock = threading.Lock()
        self._goal_handle: Any = None
        self._state = MOTION_IDLE
        self._goal_id = ""
        self._detail = ""
        self._latest_js: JointSnapshot | None = None
        self._js_sub = node.create_subscription(
            JointState, self.joint_state_topic, self._on_joint_state, 50, callback_group=callback_group
        )

    # -- joint state ---------------------------------------------------------

    def _on_joint_state(self, msg: Any) -> None:
        stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        snap = JointSnapshot(
            names=list(msg.name),
            positions=[float(v) for v in msg.position],
            velocities=[float(v) for v in msg.velocity],
            efforts=[float(v) for v in msg.effort],
            header_stamp_ns=stamp,
        )
        snap._recv_ns = wall_ns()  # type: ignore[attr-defined]
        with self._lock:
            self._latest_js = snap

    def joint_state(self, max_age_s: float | None = None) -> JointSnapshot | None:
        with self._lock:
            snap = self._latest_js
        if snap is None:
            return None
        if max_age_s is not None:
            age = (wall_ns() - getattr(snap, "_recv_ns", 0)) / 1e9
            if age > max_age_s:
                return None
        return snap

    # -- execute -----------------------------------------------------------------

    def execute(self, trajectory: Any, timeout_s: float | None = None) -> MotionResult:
        started = time.monotonic()
        if not self._client.wait_for_server(timeout_sec=self.server_wait_s):
            return MotionResult(MOTION_UNAVAILABLE, f"action server {self.action_name} not available")
        if list(trajectory.joint_names) != self.arm_joints:
            return MotionResult(
                MOTION_REJECTED,
                f"trajectory joint_names {list(trajectory.joint_names)} != platform arm joints {self.arm_joints}",
            )
        goal = self._FJT.Goal()
        goal.trajectory = trajectory
        with self._lock:
            if self._state == MOTION_ACTIVE:
                return MotionResult(MOTION_REJECTED, "a goal is already active on this backend")
            self._state = MOTION_ACTIVE
            self._detail = ""
        send_future = self._client.send_goal_async(goal)
        handle = self._wait_future(send_future, timeout_s=self.server_wait_s + 5.0)
        if handle is None or not handle.accepted:
            with self._lock:
                self._state = MOTION_REJECTED
            return MotionResult(MOTION_REJECTED, "goal rejected by the action server")
        with self._lock:
            self._goal_handle = handle
            self._goal_id = _uuid_hex(handle.goal_id)
        result_future = handle.get_result_async()
        deadline = None
        if timeout_s is not None:
            deadline = timeout_s
        else:
            try:
                last = trajectory.points[-1].time_from_start
                deadline = float(last.sec) + float(last.nanosec) / 1e9 + 30.0
            except Exception:
                deadline = None
        wrapped = self._wait_future(result_future, timeout_s=deadline)
        elapsed = time.monotonic() - started
        if wrapped is None:
            self.cancel()
            with self._lock:
                self._state = MOTION_TIMEOUT
            return MotionResult(MOTION_TIMEOUT, f"no result after {deadline}s; cancel sent", goal_id=self._goal_id, elapsed_s=elapsed)
        from action_msgs.msg import GoalStatus

        status = int(wrapped.status)
        result = wrapped.result
        code = int(getattr(result, "error_code", 0))
        error_string = str(getattr(result, "error_string", ""))
        if status == GoalStatus.STATUS_CANCELED:
            state = MOTION_CANCELED
        elif status == GoalStatus.STATUS_SUCCEEDED and code == 0:
            state = MOTION_SUCCEEDED
        else:
            state = MOTION_ABORTED
        with self._lock:
            self._state = state
            self._goal_handle = None
            self._detail = error_string
        return MotionResult(state, error_string, error_code=code, goal_id=self._goal_id, elapsed_s=elapsed)

    def _wait_future(self, future: Any, timeout_s: float | None) -> Any:
        """Block on a future without spinning: the owning executor spins the node."""
        start = time.monotonic()
        while not future.done():
            if timeout_s is not None and (time.monotonic() - start) > timeout_s:
                return None
            time.sleep(0.005)
        return future.result()

    # -- cancel ------------------------------------------------------------------

    def cancel(self) -> CancelResult:
        with self._lock:
            handle = self._goal_handle
        if handle is None:
            return CancelResult(ok=True, goals_cancelled=0, detail="no active goal", target=self.action_name)
        future = handle.cancel_goal_async()
        response = self._wait_future(future, timeout_s=self.server_wait_s)
        if response is None:
            return CancelResult(ok=False, goals_cancelled=None, detail="cancel request timed out", target=self.action_name)
        count = len(response.goals_canceling)
        return CancelResult(ok=int(response.return_code) == 0, goals_cancelled=count, detail=f"return_code={response.return_code}", target=self.action_name)

    def cancel_all(self) -> CancelResult:
        from action_msgs.msg import GoalInfo
        from action_msgs.srv import CancelGoal

        if self._cancel_client is None:
            self._cancel_client = self.node.create_client(CancelGoal, self._cancel_service, callback_group=self._cb_group)
        if not self._cancel_client.wait_for_service(timeout_sec=self.server_wait_s):
            return CancelResult(ok=False, goals_cancelled=None, detail="cancel service not available", target=self._cancel_service)
        req = CancelGoal.Request()
        req.goal_info = GoalInfo()  # all-zero uuid and stamp: cancel everything
        response = self._wait_future(self._cancel_client.call_async(req), timeout_s=self.server_wait_s)
        if response is None:
            return CancelResult(ok=False, goals_cancelled=None, detail="cancel-all timed out", target=self._cancel_service)
        return CancelResult(
            ok=int(response.return_code) == 0,
            goals_cancelled=len(response.goals_canceling),
            detail=f"return_code={response.return_code}",
            target=self._cancel_service,
        )

    # -- status ------------------------------------------------------------------

    def status(self) -> MotionStatus:
        with self._lock:
            return MotionStatus(self._state, goal_id=self._goal_id, detail=self._detail)

    @property
    def cancel_service(self) -> str:
        return self._cancel_service

    def shutdown(self) -> None:
        try:
            self.node.destroy_subscription(self._js_sub)
        except Exception:
            pass


def _uuid_hex(goal_id: Any) -> str:
    try:
        return bytes(goal_id.uuid).hex()
    except Exception:
        return ""
