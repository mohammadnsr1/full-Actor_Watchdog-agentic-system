"""The planning backend contract.

A backend plans and checks; it never executes. Every method either returns
a result dataclass or raises PlanningFailure with one of the shared failure
codes. The service adapter turns both into ROS responses and stamps the
PlanningReport. Backends are registry-free: every parameter arrives in the
request.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from cell_planning_service import failure_codes


class PlanningFailure(Exception):
    """Raised by a backend when a request cannot be satisfied."""

    def __init__(self, code: str, reason: str) -> None:
        if code not in failure_codes.ALL:
            raise ValueError(f"unknown failure code {code!r}")
        super().__init__(f"{code}: {reason}")
        self.code = code
        self.reason = reason


@dataclass
class PlanResult:
    trajectory: JointTrajectory
    fraction: float = 1.0           # Cartesian only; 1.0 otherwise
    extra: dict[str, Any] = field(default_factory=dict)


@dataclass
class CheckResult:
    valid: bool
    first_invalid_index: int = -1
    contacts: list[str] = field(default_factory=list)


@dataclass
class WorldResult:
    ok: bool
    message: str = ""
    synced: list[str] = field(default_factory=list)
    failed: list[str] = field(default_factory=list)
    skipped: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class PlanToPoseRequest:
    planning_group: str
    tip_link: str
    target: PoseStamped
    start_state: JointState
    position_tolerance_m: float
    orientation_tolerance_rad: float
    allowed_planning_time_s: float
    num_planning_attempts: int
    max_velocity_scaling: float
    max_acceleration_scaling: float
    planner_id: str


@dataclass(frozen=True)
class PlanToJointsRequest:
    planning_group: str
    joint_names: list[str]
    joint_positions: list[float]
    start_state: JointState
    joint_tolerance_rad: float
    allowed_planning_time_s: float
    num_planning_attempts: int
    max_velocity_scaling: float
    max_acceleration_scaling: float
    planner_id: str


@dataclass(frozen=True)
class PlanCartesianRequest:
    planning_group: str
    tip_link: str
    waypoints: list[PoseStamped]
    start_state: JointState
    eef_step_m: float
    min_fraction: float
    avoid_collisions: bool
    max_velocity_scaling: float
    max_acceleration_scaling: float


@dataclass(frozen=True)
class CheckTrajectoryRequest:
    planning_group: str
    trajectory: JointTrajectory
    check_resolution_rad: float


class PlanningBackend(ABC):
    """What every backend implements. See the module docstring."""

    #: "moveit" or "curobo"; copied into every PlanningReport.
    name: str = "unnamed"

    @abstractmethod
    def plan_to_pose(self, request: PlanToPoseRequest) -> PlanResult: ...

    @abstractmethod
    def plan_to_joints(self, request: PlanToJointsRequest) -> PlanResult: ...

    @abstractmethod
    def plan_cartesian(self, request: PlanCartesianRequest) -> PlanResult: ...

    @abstractmethod
    def check_trajectory(self, request: CheckTrajectoryRequest) -> CheckResult: ...

    @abstractmethod
    def add_collision_object(self, obj: CollisionObject) -> WorldResult: ...

    @abstractmethod
    def remove_collision_object(self, object_id: str) -> WorldResult: ...

    @abstractmethod
    def sync_collision_objects(self, object_ids: list[str]) -> WorldResult: ...

    def shutdown(self) -> None:  # optional
        return None
