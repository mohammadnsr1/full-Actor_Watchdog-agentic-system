"""Result and reading types shared by every backend. Each carries wall_time_iso."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from cell_platform_interfaces.timebase import wall_time_iso

# Motion states
MOTION_IDLE = "idle"
MOTION_ACTIVE = "active"
MOTION_SUCCEEDED = "succeeded"
MOTION_CANCELED = "canceled"
MOTION_ABORTED = "aborted"
MOTION_REJECTED = "rejected"
MOTION_TIMEOUT = "timeout"
MOTION_UNAVAILABLE = "unavailable"


def _stamp() -> str:
    return wall_time_iso()


@dataclass
class MotionResult:
    state: str
    detail: str = ""
    error_code: int | None = None
    goal_id: str = ""
    elapsed_s: float = 0.0
    wall_time_iso: str = field(default_factory=_stamp)

    @property
    def ok(self) -> bool:
        return self.state == MOTION_SUCCEEDED


@dataclass
class CancelResult:
    ok: bool
    goals_cancelled: int | None
    detail: str = ""
    target: str = ""
    wall_time_iso: str = field(default_factory=_stamp)


@dataclass
class MotionStatus:
    state: str
    goal_id: str = ""
    detail: str = ""
    wall_time_iso: str = field(default_factory=_stamp)


@dataclass
class JointSnapshot:
    names: list[str]
    positions: list[float]
    velocities: list[float]
    efforts: list[float]
    header_stamp_ns: int
    wall_time_iso: str = field(default_factory=_stamp)

    def position_of(self, joint: str) -> float | None:
        try:
            return self.positions[self.names.index(joint)]
        except (ValueError, IndexError):
            return None

    def effort_of(self, joint: str) -> float | None:
        try:
            return self.efforts[self.names.index(joint)]
        except (ValueError, IndexError):
            return None


@dataclass
class GripperReading:
    """One gripper observation. A reading the platform does not provide is None
    and its *_source is 'missing'; it is never estimated."""

    width_m: float | None
    effort: float | None
    grip_detected: bool | None
    width_source: str = "missing"
    effort_source: str = "missing"
    grip_detected_source: str = "missing"
    wall_time_iso: str = field(default_factory=_stamp)


@dataclass
class GripperResult:
    ok: bool
    stop_reason: str = ""
    final_width_m: float | None = None
    final_effort: float | None = None
    detail: str = ""
    wall_time_iso: str = field(default_factory=_stamp)


@dataclass
class PoseObservation:
    object_id: str
    frame_id: str
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    confidence: float
    source: str          # ground_truth | foundationpose | <other pipeline name>
    camera: str          # camera1 | camera2
    header_stamp_ns: int
    wall_time_iso: str = field(default_factory=_stamp)

    def as_dict(self) -> dict[str, Any]:
        return {
            "object_id": self.object_id,
            "frame_id": self.frame_id,
            "position": list(self.position),
            "orientation_xyzw": list(self.orientation_xyzw),
            "confidence": self.confidence,
            "source": self.source,
            "camera": self.camera,
            "header_stamp_ns": self.header_stamp_ns,
            "wall_time_iso": self.wall_time_iso,
        }


@dataclass
class SceneResult:
    ok: bool
    detail: str = ""
    data: dict[str, Any] = field(default_factory=dict)
    wall_time_iso: str = field(default_factory=_stamp)


@dataclass
class HealthReport:
    ok: bool
    checks: dict[str, dict[str, Any]] = field(default_factory=dict)
    wall_time_iso: str = field(default_factory=_stamp)
