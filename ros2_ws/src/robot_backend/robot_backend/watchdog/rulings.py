"""Rulings after a halt (Milestone 5).

issue_ruling(halt_id, ruling, limits, reason) answers one halt. The ruling is
written to the WatchDog record as a `ruling` event, and published on the
platform's `topics.watchdog_notices` as a cell_planner_msgs/WatchdogNotice
when config/cell.yaml `watchdog.consult` is true (the planner subscribes
then). Limits are physical quantities only; the WatchDog enforces them with
its own monitors (an effort limit arms an effort_event monitor on the
gripper effort joint). Limits with no library monitor are recorded as
`enforcement: not_available` rather than silently accepted.

issue_consultation(text, part, measurements) is the WatchDog's unsolicited
consultation (Milestone 7): free-form text for the executor, grounded in what
the WatchDog observed physically, optionally with measured quantities.
Published as a WatchdogNotice of kind `consultation` when `watchdog.consult`
is on, always recorded, and passed to the executor's authoring agent in the
episode feedback.
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


# Unit suffixes an optional consultation measurement name must end with.
MEASUREMENT_UNIT_SUFFIXES = ("_m", "_rad", "_n", "_s", "_m_s", "_rad_s", "_effort", "_count")
CONSULTATION_TEXT_MAX = 2000
CONSULTATION_MAX_MEASUREMENTS = 16


def validate_consultation(text: str, measurements: dict[str, Any] | None = None) -> tuple[str, dict[str, Any]]:
    """Free-form text (required, at most CONSULTATION_TEXT_MAX characters) plus
    optional measurements: each name ends in its unit, each value is a number or
    a list of numbers, and vectors need "frame" (the frame they are in)."""
    if not isinstance(text, str) or not text.strip():
        raise ValueError("text must be a non-empty string")
    text = text.strip()
    if len(text) > CONSULTATION_TEXT_MAX:
        raise ValueError(f"text is limited to {CONSULTATION_TEXT_MAX} characters")
    clean: dict[str, Any] = {}
    items = dict(measurements or {})
    if len(items) > CONSULTATION_MAX_MEASUREMENTS:
        raise ValueError(f"at most {CONSULTATION_MAX_MEASUREMENTS} measurements")
    for name, value in items.items():
        if name == "frame":
            if not isinstance(value, str) or not value:
                raise ValueError("frame must be a frame name")
            clean[name] = value
            continue
        if not isinstance(name, str) or not name.endswith(MEASUREMENT_UNIT_SUFFIXES):
            raise ValueError(f"measurement {name!r} must end in its unit, one of {list(MEASUREMENT_UNIT_SUFFIXES)}")
        if isinstance(value, bool):
            raise ValueError(f"measurement {name} must be a number or a list of numbers")
        if isinstance(value, (int, float)):
            clean[name] = float(value)
        elif isinstance(value, (list, tuple)) and value and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in value):
            clean[name] = [float(v) for v in value]
        else:
            raise ValueError(f"measurement {name} must be a number or a list of numbers")
    if any(isinstance(v, list) for v in clean.values()) and "frame" not in clean:
        raise ValueError("vector measurements need a frame")
    return text, clean


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
        msg.part = part or ""
        msg.detail = reason
        self._pub.publish(msg)

    def publish_consultation(self, part: str | None, text: str, measurements: dict[str, Any]) -> None:
        import json

        msg = self._Notice()
        msg.wall_time_iso = wall_time_iso()
        msg.kind = "consultation"
        msg.part = part or ""
        msg.detail = text
        msg.measurements_json = json.dumps(measurements, sort_keys=True) if measurements else ""
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
