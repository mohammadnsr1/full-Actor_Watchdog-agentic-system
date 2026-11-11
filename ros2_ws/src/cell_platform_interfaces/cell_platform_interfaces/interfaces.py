"""Abstract backends. Sim and hardware implement these; nothing else does.

Registry-free: callers pass the parameters each call needs. No backend
attaches or detaches anything, and none of them knows the task.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Sequence

from cell_platform_interfaces.types import (
    CancelResult,
    GripperReading,
    GripperResult,
    HealthReport,
    JointSnapshot,
    MotionResult,
    MotionStatus,
    PoseObservation,
    SceneResult,
)


class MotionBackend(ABC):
    """Execute and cancel joint trajectories; report joint state and status."""

    @abstractmethod
    def execute(self, trajectory: Any, timeout_s: float | None = None) -> MotionResult:
        """Run a trajectory_msgs/JointTrajectory to completion, cancel, or abort."""

    @abstractmethod
    def cancel(self) -> CancelResult:
        """Cancel the current goal (the backend's own goal, if any)."""

    @abstractmethod
    def cancel_all(self) -> CancelResult:
        """Cancel every active goal on the motion server, whoever sent it."""

    @abstractmethod
    def joint_state(self, max_age_s: float | None = None) -> JointSnapshot | None:
        """Latest joint state, or None when nothing fresh enough arrived."""

    @abstractmethod
    def status(self) -> MotionStatus:
        """What the backend is doing right now."""

    @property
    @abstractmethod
    def cancel_service(self) -> str:
        """The action cancel service name, for anyone that cancels from outside."""


class GripperBackend(ABC):
    """Open, close, set width; read what the platform actually provides."""

    @abstractmethod
    def open(self, timeout_s: float | None = None) -> GripperResult: ...

    @abstractmethod
    def close(self, force: float | None = None, timeout_s: float | None = None) -> GripperResult:
        """Close; `force` is honoured where the platform supports it, else ignored and reported."""

    @abstractmethod
    def set_width(self, width_m: float, force: float | None = None, timeout_s: float | None = None) -> GripperResult: ...

    @abstractmethod
    def read(self) -> GripperReading: ...

    @property
    @abstractmethod
    def provided_readings(self) -> dict[str, str]:
        """{'width': 'ground_truth'|'driver'|'missing', 'effort': ..., 'grip_detected': ...}"""


class PoseSource(ABC):
    """Object poses with confidence, source and stamp. One instance per camera role."""

    @abstractmethod
    def latest(self, object_id: str, max_age_s: float | None = None) -> PoseObservation | None: ...

    @abstractmethod
    def latest_all(self, max_age_s: float | None = None) -> dict[str, PoseObservation]: ...

    @abstractmethod
    def object_ids(self) -> list[str]:
        """Object ids seen so far."""

    @property
    @abstractmethod
    def camera(self) -> str:
        """camera1 | camera2"""

    @property
    @abstractmethod
    def source(self) -> str:
        """ground_truth | foundationpose | ..."""


class SceneBackend(ABC):
    """Reset, verify, health. On hardware reset means asking the operator."""

    @abstractmethod
    def reset(self, layout: dict[str, Any] | None = None, timeout_s: float | None = None) -> SceneResult: ...

    @abstractmethod
    def verify_reset(self, tolerances: dict[str, Any], layout: dict[str, Any] | None = None) -> SceneResult: ...

    @abstractmethod
    def health(self) -> HealthReport: ...


class Backends:
    """Container the planner and the runner receive. Built per platform."""

    def __init__(
        self,
        motion: MotionBackend,
        gripper: GripperBackend,
        executor_poses: PoseSource,
        watchdog_poses: PoseSource,
        scene: SceneBackend,
    ) -> None:
        self.motion = motion
        self.gripper = gripper
        self.executor_poses = executor_poses
        self.watchdog_poses = watchdog_poses
        self.scene = scene

    def shutdown(self) -> None:
        for backend in (self.motion, self.gripper, self.executor_poses, self.watchdog_poses, self.scene):
            close = getattr(backend, "shutdown", None)
            if callable(close):
                try:
                    close()
                except Exception:
                    pass


def joint_names_match(expected: Sequence[str], actual: Sequence[str]) -> bool:
    return list(expected) == list(actual)
