"""Physical events and episode notices as WatchDog sees them (Milestone 5).

The recorder no longer hears the executor. Its wake sources are:
    /cell/physical_events   (cell_planner_msgs/PhysicalEvent, from the
                             cell-owned cell_physical_events node)
    /cell/watchdog_notices  (cell_planner_msgs/WatchdogNotice; only kind ==
                             episode_end is consumed here, published by the
                             episode runner)
    monitor fires            (the engine)
    wake conditions          (arm_wake, evaluated by the engine on physical events)

This module turns the two topics into recorder records with the same shape
every other event record has (wall_ns, event_type, payload, envelope), so
get_events, get_evidence and wait_for_event need no special cases. Topic
names come from the platform config.
"""

from __future__ import annotations

from typing import Any

from cell_platform_interfaces.timebase import parse_wall_time_iso

PHYSICAL_EVENT_KINDS: tuple[str, ...] = (
    "contact_made",
    "contact_lost",
    "part_lifted",
    "part_released",
    "part_settled",
    "part_approaching_feature",
    "part_engaged_feature",
    "effort_rising",
    "motion_started",
    "motion_stopped",
    "workspace_object_appeared",
)

EPISODE_END_EVENT_TYPE = "episode_end"
NOTICE_EVENT_TYPES: tuple[str, ...] = (EPISODE_END_EVENT_TYPE,)


def physical_record(msg: Any, received_wall_ns: int) -> dict[str, Any]:
    """Recorder record for one PhysicalEvent message."""
    try:
        stamped = parse_wall_time_iso(str(msg.wall_time_iso))
    except Exception:
        stamped = None
    payload = {
        "kind": str(msg.kind),
        "scene_ids": [str(s) for s in msg.scene_ids],
        "source": str(msg.source),
        "confidence": float(msg.confidence),
        "value": float(msg.value),
        "detail": str(msg.detail),
        "sim_time": float(msg.sim_time),
    }
    return {
        "wall_ns_received": received_wall_ns,
        "envelope_wall_ns": stamped,
        "wall_ns": stamped if stamped is not None else received_wall_ns,
        "wall_ns_source": "envelope" if stamped is not None else "received",
        "event_type": str(msg.kind),
        "origin": "physical",
        "payload": payload,
        "envelope": {
            "event_type": str(msg.kind),
            "origin": "physical",
            "wall_time_iso": str(msg.wall_time_iso),
            "payload": payload,
        },
    }


def notice_record(msg: Any, received_wall_ns: int) -> dict[str, Any] | None:
    """Recorder record for a WatchdogNotice the WatchDog consumes (episode_end only).

    Rulings and halts on the notices topic are WatchDog's own output and are
    not re-ingested; the planner's view of them is not WatchDog's evidence.
    """
    kind = str(msg.kind)
    if kind != EPISODE_END_EVENT_TYPE:
        return None
    try:
        stamped = parse_wall_time_iso(str(msg.wall_time_iso))
    except Exception:
        stamped = None
    payload = {"kind": kind, "detail": str(msg.detail), "halt_id": str(msg.halt_id)}
    return {
        "wall_ns_received": received_wall_ns,
        "envelope_wall_ns": stamped,
        "wall_ns": stamped if stamped is not None else received_wall_ns,
        "wall_ns_source": "envelope" if stamped is not None else "received",
        "event_type": kind,
        "origin": "episode_runner",
        "payload": payload,
        "envelope": {"event_type": kind, "origin": "episode_runner", "wall_time_iso": str(msg.wall_time_iso), "payload": payload},
    }


class WakeCondition:
    """A condition the agent armed with arm_wake, matched against physical events.

    condition keys (all optional, all ANDed):
        kinds          list of physical event kinds
        scene_ids      every listed id must appear in the event's scene_ids
        source         exact source string
        min_confidence float
        once           default True: the condition is removed after it matches
    """

    VALID_KEYS = ("kinds", "scene_ids", "source", "min_confidence", "once", "note")

    def __init__(self, wake_id: str, condition: dict[str, Any], armed_wall_ns: int) -> None:
        if not isinstance(condition, dict) or not condition:
            raise ValueError("condition must be a non-empty object")
        unknown = sorted(set(condition) - set(self.VALID_KEYS))
        if unknown:
            raise ValueError(f"unknown condition keys {unknown}; valid: {list(self.VALID_KEYS)}")
        kinds = condition.get("kinds")
        if kinds is not None:
            if not isinstance(kinds, list) or not all(isinstance(k, str) for k in kinds):
                raise ValueError("kinds must be a list of strings")
            bad = sorted(set(kinds) - set(PHYSICAL_EVENT_KINDS))
            if bad:
                raise ValueError(f"unknown physical event kinds {bad}; valid: {list(PHYSICAL_EVENT_KINDS)}")
        scene_ids = condition.get("scene_ids")
        if scene_ids is not None and (not isinstance(scene_ids, list) or not all(isinstance(s, str) for s in scene_ids)):
            raise ValueError("scene_ids must be a list of strings")
        if not kinds and not scene_ids and condition.get("source") is None:
            raise ValueError("condition needs at least one of kinds, scene_ids, source")
        self.wake_id = wake_id
        self.kinds = set(kinds or [])
        self.scene_ids = list(scene_ids or [])
        self.source = condition.get("source")
        self.min_confidence = float(condition.get("min_confidence", 0.0) or 0.0)
        self.once = bool(condition.get("once", True))
        self.note = str(condition.get("note", "") or "")
        self.armed_wall_ns = armed_wall_ns
        self.matches_count = 0

    def matches(self, kind: str, scene_ids: list[str], source: str, confidence: float) -> bool:
        if self.kinds and kind not in self.kinds:
            return False
        if any(s not in scene_ids for s in self.scene_ids):
            return False
        if self.source is not None and source != self.source:
            return False
        if confidence < self.min_confidence:
            return False
        return True

    def as_dict(self) -> dict[str, Any]:
        return {
            "wake_id": self.wake_id,
            "kinds": sorted(self.kinds),
            "scene_ids": list(self.scene_ids),
            "source": self.source,
            "min_confidence": self.min_confidence,
            "once": self.once,
            "note": self.note,
            "armed_wall_ns": self.armed_wall_ns,
            "matches": self.matches_count,
        }
