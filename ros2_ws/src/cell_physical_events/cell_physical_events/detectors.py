"""Pure detector logic for the physical event vocabulary (no ROS).

The node feeds samples in; detectors return events. Every threshold comes
from the `physical_events` block of config/cell.yaml. A detector whose
threshold is zero or missing is DISABLED and reports that once through
`disabled_reasons()`; it never guesses a value.

Vocabulary (kind -> scene_ids):
    contact_made / contact_lost            []            effort proxy or contact report
    part_lifted / part_released            [part]        camera2 pose + robot TF (tcp)
    part_settled                           [part]        camera2 pose
    part_approaching_feature               [part, feature]
    part_engaged_feature                   [part, feature]
    effort_rising                          [joint]       joint_state
    motion_started / motion_stopped        []            joint_state
    workspace_object_appeared              [id]          camera2 pose

Sources: ground_truth | camera2_pose (perception) for poses, joint_state for
joint-derived events, effort_proxy for contact by effort, contact_report
for pad contacts (Milestone 6, sim only).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

EVENT_KINDS = (
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


@dataclass
class Event:
    kind: str
    scene_ids: list[str]
    source: str
    confidence: float
    value: float
    detail: str
    t_ns: int


@dataclass
class PoseSample:
    object_id: str
    position: tuple[float, float, float]
    confidence: float
    source: str
    t_ns: int


def _positive(cfg: dict[str, Any], key: str) -> float:
    try:
        value = float(cfg.get(key, 0.0) or 0.0)
    except (TypeError, ValueError):
        return 0.0
    return value if value > 0.0 else 0.0


def _dist(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


class MotionDetector:
    """motion_started / motion_stopped from the arm joint-speed norm."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        self.threshold = _positive(cfg, "motion_speed_threshold_rad_s")
        self.debounce_ns = int(_positive(cfg, "motion_debounce_s") * 1e9)
        self.enabled = self.threshold > 0.0
        self.moving = False
        self._candidate_since: int | None = None
        self._last_positions: list[float] | None = None
        self._last_t: int | None = None

    def disabled_reason(self) -> str | None:
        return None if self.enabled else "physical_events.motion_speed_threshold_rad_s is 0 (TODO(mohammad))"

    def feed(self, positions: list[float], velocities: list[float], t_ns: int) -> list[Event]:
        if not self.enabled:
            return []
        if velocities and len(velocities) == len(positions):
            speed = math.sqrt(sum(v * v for v in velocities))
        elif self._last_positions is not None and self._last_t is not None and t_ns > self._last_t:
            dt = (t_ns - self._last_t) / 1e9
            speed = math.sqrt(sum((p - q) ** 2 for p, q in zip(positions, self._last_positions))) / dt
        else:
            speed = None
        self._last_positions = list(positions)
        self._last_t = t_ns
        if speed is None:
            return []
        want_moving = speed > self.threshold
        if want_moving == self.moving:
            self._candidate_since = None
            return []
        if self._candidate_since is None:
            self._candidate_since = t_ns
        if t_ns - self._candidate_since < self.debounce_ns:
            return []
        self.moving = want_moving
        self._candidate_since = None
        kind = "motion_started" if want_moving else "motion_stopped"
        return [Event(kind, [], "joint_state", 1.0, speed, f"joint speed norm vs motion_speed_threshold_rad_s={self.threshold}", t_ns)]


class EffortDetector:
    """effort_rising (slope) and contact_made/lost by effort proxy on one joint."""

    def __init__(self, cfg: dict[str, Any], joint: str, contact_proxy_allowed: bool) -> None:
        self.joint = joint
        self.rise_rate = _positive(cfg, "effort_rise_rate_per_s")
        self.rise_window_ns = int(_positive(cfg, "effort_rise_window_s") * 1e9)
        self.contact_threshold = _positive(cfg, "effort_contact_threshold")
        self.contact_ticks = int(_positive(cfg, "effort_contact_ticks"))
        self.proxy_confidence = _positive(cfg, "effort_proxy_confidence")
        self.rise_enabled = bool(joint) and self.rise_rate > 0.0 and self.rise_window_ns > 0
        self.contact_enabled = (
            bool(joint) and contact_proxy_allowed and self.contact_threshold > 0.0
            and self.contact_ticks > 0 and self.proxy_confidence > 0.0
        )
        self.in_contact = False
        self._above = 0
        self._below = 0
        self._history: list[tuple[int, float]] = []
        self._rising = False

    def disabled_reasons(self) -> list[str]:
        out = []
        if not self.joint:
            out.append("no gripper effort joint on this platform: effort events off")
            return out
        if not self.rise_enabled:
            out.append("physical_events.effort_rise_rate_per_s / effort_rise_window_s are 0 (TODO(mohammad)): effort_rising off")
        if not self.contact_enabled:
            out.append("effort contact proxy off (needs effort_contact_threshold, effort_contact_ticks, effort_proxy_confidence, and no contact report)")
        return out

    def feed(self, effort: float, t_ns: int) -> list[Event]:
        events: list[Event] = []
        value = abs(float(effort))
        if self.rise_enabled:
            self._history.append((t_ns, value))
            cutoff = t_ns - self.rise_window_ns
            self._history = [(t, v) for t, v in self._history if t >= cutoff]
            if len(self._history) >= 2:
                t0, v0 = self._history[0]
                span = (t_ns - t0) / 1e9
                slope = (value - v0) / span if span > 0 else 0.0
                if slope > self.rise_rate and not self._rising:
                    self._rising = True
                    events.append(Event("effort_rising", [self.joint], "joint_state", 1.0, slope, f"effort slope vs effort_rise_rate_per_s={self.rise_rate}", t_ns))
                elif slope <= 0.0:
                    self._rising = False
        if self.contact_enabled:
            if value >= self.contact_threshold:
                self._above += 1
                self._below = 0
            else:
                self._below += 1
                self._above = 0
            if not self.in_contact and self._above >= self.contact_ticks:
                self.in_contact = True
                events.append(Event("contact_made", [], "effort_proxy", self.proxy_confidence, value, f"|effort| >= effort_contact_threshold={self.contact_threshold} for {self.contact_ticks} ticks", t_ns))
            elif self.in_contact and self._below >= self.contact_ticks:
                self.in_contact = False
                events.append(Event("contact_lost", [], "effort_proxy", self.proxy_confidence, value, f"|effort| < effort_contact_threshold={self.contact_threshold} for {self.contact_ticks} ticks", t_ns))
        return events


@dataclass
class _PartState:
    rest_z: float | None = None
    lifted: bool = False
    held: bool = False
    last_pos: tuple[float, float, float] | None = None
    last_t: int | None = None
    settled: bool = True
    still_since: int | None = None
    approaching: set = field(default_factory=set)
    engaged: set = field(default_factory=set)


class PartDetector:
    """Lift / hold / release / settle / approach / engage for every known part.

    Inputs per tick: the latest pose of every known object (parts and feature
    frames) and the TCP position. Unknown ids produce workspace_object_appeared.
    """

    def __init__(self, cfg: dict[str, Any], parts: dict[str, dict[str, str]], frames: set[str]) -> None:
        self.parts = parts          # part_id -> {feature_kind: frame_id}
        self.frames = set(frames)   # known feature frame ids
        self.lift = _positive(cfg, "lift_threshold_m")
        self.hold = _positive(cfg, "hold_distance_m")
        self.release = _positive(cfg, "release_distance_m")
        self.settle_speed = _positive(cfg, "settle_speed_m_s")
        self.settle_window_ns = int(_positive(cfg, "settle_window_s") * 1e9)
        self.approach = _positive(cfg, "approach_distance_m")
        self.engage = _positive(cfg, "engage_distance_m")
        self.unknown_min_conf = _positive(cfg, "unknown_object_min_confidence")
        self.lift_enabled = self.lift > 0.0 and self.hold > 0.0 and self.release > 0.0
        self.settle_enabled = self.settle_speed > 0.0 and self.settle_window_ns > 0
        self.feature_enabled = self.approach > 0.0 and self.engage > 0.0
        self.unknown_enabled = self.unknown_min_conf > 0.0
        self.state: dict[str, _PartState] = {p: _PartState() for p in parts}
        self._seen_unknown: set[str] = set()

    def disabled_reasons(self) -> list[str]:
        out = []
        if not self.lift_enabled:
            out.append("physical_events.lift_threshold_m / hold_distance_m / release_distance_m are 0 (TODO(mohammad)): part_lifted/part_released off")
        if not self.settle_enabled:
            out.append("physical_events.settle_speed_m_s / settle_window_s are 0 (TODO(mohammad)): part_settled off")
        if not self.feature_enabled:
            out.append("physical_events.approach_distance_m / engage_distance_m are 0 (TODO(mohammad)): feature events off")
        if not self.unknown_enabled:
            out.append("physical_events.unknown_object_min_confidence is 0 (TODO(mohammad)): workspace_object_appeared off")
        return out

    def held_part(self) -> str | None:
        for part, st in self.state.items():
            if st.held:
                return part
        return None

    def feed(self, poses: dict[str, PoseSample], tcp: tuple[float, float, float] | None, t_ns: int) -> list[Event]:
        events: list[Event] = []
        for object_id, sample in poses.items():
            if object_id in self.parts or object_id in self.frames:
                continue
            if self.unknown_enabled and sample.confidence >= self.unknown_min_conf and object_id not in self._seen_unknown:
                self._seen_unknown.add(object_id)
                events.append(Event("workspace_object_appeared", [object_id], sample.source, sample.confidence, 0.0, "detection id not in watchdog_parts.yaml", t_ns))
        for part, st in self.state.items():
            sample = poses.get(part)
            if sample is None:
                continue
            pos = sample.position
            src, conf = sample.source, sample.confidence
            if st.rest_z is None:
                st.rest_z = pos[2]
            # speed for settle
            speed = None
            if st.last_pos is not None and st.last_t is not None and t_ns > st.last_t:
                speed = _dist(pos, st.last_pos) / ((t_ns - st.last_t) / 1e9)
            st.last_pos, st.last_t = pos, t_ns
            # lift / hold / release
            if self.lift_enabled:
                rise = pos[2] - st.rest_z
                near_tcp = tcp is not None and _dist(pos, tcp) <= self.hold
                far_tcp = tcp is None or _dist(pos, tcp) >= self.release
                if not st.lifted and rise > self.lift and (tcp is None or near_tcp):
                    st.lifted, st.held, st.settled = True, True, False
                    events.append(Event("part_lifted", [part], src, conf, rise, f"rise above rest vs lift_threshold_m={self.lift}; within hold_distance_m={self.hold} of tcp", t_ns))
                elif st.lifted and st.held and far_tcp:
                    st.held = False
                    st.lifted = False
                    st.still_since = None
                    events.append(Event("part_released", [part], src, conf, _dist(pos, tcp) if tcp else 0.0, f"part beyond release_distance_m={self.release} from tcp", t_ns))
                    for f in list(st.approaching | st.engaged):
                        pass
                    st.approaching.clear()
                    st.engaged.clear()
            # settle
            if self.settle_enabled and not st.held and not st.settled and speed is not None:
                if speed <= self.settle_speed:
                    if st.still_since is None:
                        st.still_since = t_ns
                    elif t_ns - st.still_since >= self.settle_window_ns:
                        st.settled = True
                        st.rest_z = pos[2]
                        events.append(Event("part_settled", [part], src, conf, speed, f"speed <= settle_speed_m_s={self.settle_speed} for settle_window_s", t_ns))
                else:
                    st.still_since = None
            # approach / engage against the part's own features
            if self.feature_enabled and st.held:
                for feature in self.parts[part].values():
                    fs = poses.get(feature)
                    if fs is None:
                        continue
                    d = _dist(pos, fs.position)
                    if d <= self.engage and feature not in st.engaged:
                        st.engaged.add(feature)
                        events.append(Event("part_engaged_feature", [part, feature], src, min(conf, fs.confidence), d, f"distance <= engage_distance_m={self.engage}", t_ns))
                    elif d <= self.approach and feature not in st.approaching and feature not in st.engaged:
                        st.approaching.add(feature)
                        events.append(Event("part_approaching_feature", [part, feature], src, min(conf, fs.confidence), d, f"distance <= approach_distance_m={self.approach}", t_ns))
                    elif d > self.approach:
                        st.approaching.discard(feature)
                        st.engaged.discard(feature)
        return events
