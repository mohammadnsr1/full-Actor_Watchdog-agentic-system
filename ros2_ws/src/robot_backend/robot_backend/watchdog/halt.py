"""Narrow halt authority for WatchDog.

A halt is a cancel-all request to one or more action cancel services. The
first target is the one that decides whether the halt succeeded; later
targets are extras and only add to the detail. Nothing is published to
replace the cancelled motion: the bridge's FollowJointTrajectory server
returns `canceled` and leaves the arm at the last setpoint it already sent.

Targets come from config/cell.yaml `watchdog.halt.cancel_targets` through
the caller; the default here is the bridge alone.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Sequence

from robot_backend.events.schema import HaltPayload, make_event

BRIDGE_CANCEL_SERVICE = "/arm_controller/follow_joint_trajectory/_action/cancel_goal"
MOVE_GROUP_CANCEL_SERVICE = "/move_action/_action/cancel_goal"
DEFAULT_CANCEL_TARGETS: tuple[str, ...] = (BRIDGE_CANCEL_SERVICE,)


class HaltClient:
    """Cancel active goals on the configured action servers; never move."""

    def __init__(
        self,
        node: Any,
        event_bus: Any,
        callback_group: Any,
        timeout_sec: float = 1.0,
        cancel_targets: Sequence[str] | None = None,
    ) -> None:
        self.node = node
        self.event_bus = event_bus
        self._callback_group = callback_group
        self.timeout_sec = timeout_sec
        targets = [t.strip() for t in (cancel_targets or DEFAULT_CANCEL_TARGETS) if t and t.strip()]
        if not targets:
            raise ValueError("HaltClient needs at least one cancel target")
        self.cancel_targets: tuple[str, ...] = tuple(targets)
        self._cancel_clients: dict[str, Any] = {}
        self._CancelGoal: Any | None = None
        self._GoalInfo: Any | None = None
        self._lock = threading.Lock()

    @property
    def primary_target(self) -> str:
        return self.cancel_targets[0]

    def halt(
        self, monitor_id: str, detail: str, step_id: str | None = None
    ) -> dict[str, Any]:
        """Send cancel-all to every target; `ok` reflects the primary target.

        step_id is optional provenance for the published halt event. Nothing
        in the cancel itself reads it: the request cancels all active goals.
        """
        ok = False
        per_target: list[dict[str, Any]] = []
        goals_cancelled: int | None = None
        elapsed_sec: float | None = None
        started = time.monotonic()
        try:
            self._ensure_clients()
            for index, target in enumerate(self.cancel_targets):
                outcome = self._cancel_one(target)
                per_target.append(outcome)
                if outcome["goals_cancelled"] is not None:
                    goals_cancelled = (goals_cancelled or 0) + outcome["goals_cancelled"]
                if index == 0:
                    ok = bool(outcome["ok"])
        except Exception as exc:  # noqa: BLE001  a halt must always return and publish
            per_target.append({"target": "(setup)", "ok": False, "detail": str(exc), "goals_cancelled": None})
        finally:
            elapsed_sec = time.monotonic() - started
            self._publish_halt(monitor_id, detail, step_id)
        cancel_detail = "; ".join(f"{o['target']}: {o['detail']}" for o in per_target) or "cancel not attempted"
        return {
            "ok": bool(ok),
            "detail": cancel_detail,
            "goals_cancelled": goals_cancelled,
            "elapsed_sec": elapsed_sec,
            "targets": per_target,
        }

    def prewarm(self) -> None:
        """Construct ROS clients before a monitor callback needs them."""
        self._ensure_clients()

    # -- internals ---------------------------------------------------------

    def _cancel_one(self, target: str) -> dict[str, Any]:
        client = self._cancel_clients.get(target)
        if client is None or self._CancelGoal is None or self._GoalInfo is None:
            return {"target": target, "ok": False, "detail": "client unavailable", "goals_cancelled": None}
        if not client.wait_for_service(timeout_sec=0.2):
            return {"target": target, "ok": False, "detail": "service unavailable", "goals_cancelled": None}
        request = self._CancelGoal.Request()
        request.goal_info = self._GoalInfo()
        # All-zero goal id = cancel every active goal (action_msgs/srv/CancelGoal).
        request.goal_info.goal_id.uuid = [0] * 16
        done = threading.Event()
        holder: dict[str, Any] = {}
        call_started = time.monotonic()
        future = client.call_async(request)

        def _done(fut: Any) -> None:
            holder["future"] = fut
            holder["elapsed_sec"] = time.monotonic() - call_started
            done.set()

        future.add_done_callback(_done)
        if not done.wait(self.timeout_sec):
            return {
                "target": target, "ok": False,
                "detail": f"cancel-all timed out after {self.timeout_sec:.1f}s", "goals_cancelled": None,
            }
        result = future.result()
        return_code = getattr(result, "return_code", None)
        canceling = list(getattr(result, "goals_canceling", []) or [])
        return {
            "target": target,
            "ok": return_code in (None, 0),
            "detail": f"cancel-all requested; return_code={return_code}; goals_canceling={len(canceling)}",
            "goals_cancelled": len(canceling),
            "elapsed_sec": holder.get("elapsed_sec"),
        }

    def _ensure_clients(self) -> None:
        with self._lock:
            if self._cancel_clients:
                return
            from action_msgs.msg import GoalInfo
            from action_msgs.srv import CancelGoal

            # rclpy ActionClient has per-goal cancel APIs, not cancel-all, so the
            # action's cancel service is called directly with an all-zero goal id.
            for target in self.cancel_targets:
                self._cancel_clients[target] = self.node.create_client(
                    CancelGoal, target, callback_group=self._callback_group
                )
            self._CancelGoal = CancelGoal
            self._GoalInfo = GoalInfo

    def _publish_halt(self, monitor_id: str, detail: str, step_id: str | None) -> None:
        try:
            if self.event_bus is not None:
                self.event_bus.publish(make_event(
                    "halt",
                    HaltPayload(step_id=step_id, monitor_id=monitor_id, detail=detail),
                ))
        except Exception:
            pass
