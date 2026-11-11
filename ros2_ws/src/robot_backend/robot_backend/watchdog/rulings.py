"""Rulings after a halt (Milestone 5).

issue_ruling(halt_id, ruling, limits, reason) answers one halt. The ruling is
written to the WatchDog record as a `ruling` event, and published on the
platform's `topics.watchdog_notices` as a cell_planner_msgs/WatchdogNotice
when config/cell.yaml `watchdog.consult` is true (the planner subscribes
then). Limits are physical quantities only; the WatchDog enforces them with
its own monitors (an effort limit arms an effort_event monitor on the
gripper effort joint). Limits with no library monitor are recorded as
`enforcement: not_available` rather than silently accepted.
"""

from __future__ import annotations

from typing import Any

from cell_platform_interfaces.timebase import wall_time_iso

RULINGS = ("allowed", "allowed_with_limits", "denied_for_part", "escalate")

# Physical limit vocabulary. Values are numbers in the unit named.
LIMIT_NAMES: dict[str, str] = {
    "max_clamp_effort": "effort units of the platform's gripper effort joint",
    "max_tcp_speed_m_s": "metres per second",
    "max_joint_speed_rad_s": "radians per second",
    "min_approach_clearance_m": "metres",
}


def validate_ruling(ruling: str, limits: dict[str, Any] | None, part: str | None) -> dict[str, float]:
    if ruling not in RULINGS:
        raise ValueError(f"ruling must be one of {list(RULINGS)}")
    clean: dict[str, float] = {}
    for name, value in (limits or {}).items():
        if name not in LIMIT_NAMES:
            raise ValueError(f"unknown limit {name!r}; physical limits only: {sorted(LIMIT_NAMES)}")
        try:
            clean[name] = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"limit {name} must be a number") from exc
        if clean[name] <= 0:
            raise ValueError(f"limit {name} must be positive")
    if ruling == "allowed_with_limits" and not clean:
        raise ValueError("allowed_with_limits needs at least one limit")
    if ruling == "denied_for_part" and not part:
        raise ValueError("denied_for_part needs the part")
    return clean


class NoticePublisher:
    """Publishes WatchdogNotice messages on the notices topic."""

    def __init__(self, node: Any, topic: str) -> None:
        from cell_planner_msgs.msg import WatchdogNotice

        self._Notice = WatchdogNotice
        self._pub = node.create_publisher(WatchdogNotice, topic, 10)
        self.topic = topic

    def publish_ruling(self, halt_id: str, ruling: str, part: str | None, limits: dict[str, float], reason: str) -> None:
        from cell_planner_msgs.msg import Ruling

        msg = self._Notice()
        msg.wall_time_iso = wall_time_iso()
        msg.kind = "ruling"
        msg.halt_id = halt_id
        r = Ruling()
        r.ruling = ruling
        r.part = part or ""
        r.limit_names = list(limits.keys())
        r.limit_values = [float(v) for v in limits.values()]
        r.reason = reason
        msg.ruling = r
        msg.detail = reason
        self._pub.publish(msg)

    def publish_halt(self, halt_id: str, detail: str) -> None:
        msg = self._Notice()
        msg.wall_time_iso = wall_time_iso()
        msg.kind = "halt"
        msg.halt_id = halt_id
        msg.detail = detail
        self._pub.publish(msg)


def enforce_limits(engine: Any, recorder: Any, limits: dict[str, float], effort_joint: str | None, halt_on_fire: bool, clause: dict[str, Any]) -> list[dict[str, Any]]:
    """Arm the WatchDog's own monitors for the limits it can enforce.

    max_clamp_effort -> effort_event on the gripper effort joint. The others
    have no library monitor yet and are reported as not enforceable.
    """
    from robot_backend.watchdog.monitors import MONITOR_TYPES

    out: list[dict[str, Any]] = []
    for name, value in limits.items():
        if name == "max_clamp_effort":
            if not effort_joint:
                out.append({"limit": name, "enforcement": "not_available", "reason": "no gripper effort joint on this platform"})
                continue
            cls = MONITOR_TYPES["effort_event"]
            monitor = cls(
                {"joint_name": effort_joint, "comparison": "le", "threshold": float(value)},
                clause=clause,
                halt_on_fire=halt_on_fire,
            )
            monitor.provenance = {"threshold_source": "ruling", "threshold_verified": True}
            armed = engine.arm(monitor)
            out.append({"limit": name, "enforcement": "effort_event", "monitor_id": monitor.monitor_id, "armed": armed})
        else:
            out.append({"limit": name, "enforcement": "not_available", "reason": "no library monitor for this limit yet; recorded only"})
    return out
