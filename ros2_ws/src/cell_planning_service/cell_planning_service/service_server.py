"""ROS 2 service adapter: cell_planner_msgs services -> PlanningBackend.

One instance per node. It owns no planning logic; it unpacks requests,
calls the backend, catches PlanningFailure, and stamps every response with
a PlanningReport (backend, planning_time_s, failure_code, wall_time_iso).

Service names are <namespace>/<name>; the namespace comes from
config/cell.yaml planner.service_namespace (default /cell/planning).
"""

from __future__ import annotations

import logging
import time
from datetime import datetime, timezone
from typing import Any, Callable

from cell_planner_msgs.msg import PlanningReport
from cell_planner_msgs.srv import (
    AddCollisionObject,
    CheckTrajectory,
    PlanCartesian,
    PlanToJoints,
    PlanToPose,
    RemoveCollisionObject,
    SyncCollisionObjects,
)

from cell_planning_service import failure_codes
from cell_planning_service.backend import (
    CheckTrajectoryRequest,
    PlanCartesianRequest,
    PlanningBackend,
    PlanningFailure,
    PlanToJointsRequest,
    PlanToPoseRequest,
)

DEFAULT_SERVICE_NAMESPACE = "/cell/planning"

SERVICE_NAMES = (
    "plan_to_pose",
    "plan_to_joints",
    "plan_cartesian",
    "check_trajectory",
    "add_collision_object",
    "remove_collision_object",
    "sync_collision_objects",
)


def wall_time_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def make_report(backend: str, started: float, code: str = failure_codes.NONE, reason: str = "") -> PlanningReport:
    report = PlanningReport()
    report.backend = backend
    report.planning_time_s = float(time.monotonic() - started)
    report.failure_code = code
    report.failure_reason = reason
    report.wall_time_iso = wall_time_iso()
    return report


class PlanningServiceServer:
    def __init__(
        self,
        node: Any,
        backend: PlanningBackend,
        *,
        namespace: str = DEFAULT_SERVICE_NAMESPACE,
        callback_group: Any = None,
        logger: logging.Logger | None = None,
    ) -> None:
        self.node = node
        self.backend = backend
        self.namespace = namespace.rstrip("/") or DEFAULT_SERVICE_NAMESPACE
        self.log = logger or logging.getLogger(__name__)
        self._services: list[Any] = []
        kw = {"callback_group": callback_group} if callback_group is not None else {}
        table: list[tuple[str, Any, Callable[[Any, Any], Any]]] = [
            ("plan_to_pose", PlanToPose, self._on_plan_to_pose),
            ("plan_to_joints", PlanToJoints, self._on_plan_to_joints),
            ("plan_cartesian", PlanCartesian, self._on_plan_cartesian),
            ("check_trajectory", CheckTrajectory, self._on_check_trajectory),
            ("add_collision_object", AddCollisionObject, self._on_add_collision_object),
            ("remove_collision_object", RemoveCollisionObject, self._on_remove_collision_object),
            ("sync_collision_objects", SyncCollisionObjects, self._on_sync_collision_objects),
        ]
        for name, srv_type, handler in table:
            self._services.append(
                node.create_service(srv_type, f"{self.namespace}/{name}", handler, **kw)
            )

    # -- helpers -----------------------------------------------------------

    def _run(self, response: Any, started: float, fn: Callable[[], Any]) -> Any | None:
        """Call fn; on PlanningFailure fill the report and return None."""
        try:
            return fn()
        except PlanningFailure as exc:
            response.report = make_report(self.backend.name, started, exc.code, exc.reason)
            self.log.warning("%s: %s", exc.code, exc.reason)
        except Exception as exc:  # noqa: BLE001  a backend bug must not kill the node
            response.report = make_report(
                self.backend.name, started, failure_codes.BACKEND_ERROR,
                f"{type(exc).__name__}: {exc}",
            )
            self.log.exception("backend raised")
        return None

    # -- handlers ----------------------------------------------------------

    def _on_plan_to_pose(self, request: Any, response: Any) -> Any:
        started = time.monotonic()
        response.ok = False
        req = PlanToPoseRequest(
            planning_group=request.planning_group,
            tip_link=request.tip_link,
            target=request.target,
            start_state=request.start_state,
            position_tolerance_m=float(request.position_tolerance_m),
            orientation_tolerance_rad=float(request.orientation_tolerance_rad),
            allowed_planning_time_s=float(request.allowed_planning_time_s),
            num_planning_attempts=int(request.num_planning_attempts),
            max_velocity_scaling=float(request.max_velocity_scaling),
            max_acceleration_scaling=float(request.max_acceleration_scaling),
            planner_id=request.planner_id,
        )
        result = self._run(response, started, lambda: self.backend.plan_to_pose(req))
        if result is not None:
            response.ok = True
            response.trajectory = result.trajectory
            response.report = make_report(self.backend.name, started)
        return response

    def _on_plan_to_joints(self, request: Any, response: Any) -> Any:
        started = time.monotonic()
        response.ok = False
        req = PlanToJointsRequest(
            planning_group=request.planning_group,
            joint_names=list(request.joint_names),
            joint_positions=[float(v) for v in request.joint_positions],
            start_state=request.start_state,
            joint_tolerance_rad=float(request.joint_tolerance_rad),
            allowed_planning_time_s=float(request.allowed_planning_time_s),
            num_planning_attempts=int(request.num_planning_attempts),
            max_velocity_scaling=float(request.max_velocity_scaling),
            max_acceleration_scaling=float(request.max_acceleration_scaling),
            planner_id=request.planner_id,
        )
        result = self._run(response, started, lambda: self.backend.plan_to_joints(req))
        if result is not None:
            response.ok = True
            response.trajectory = result.trajectory
            response.report = make_report(self.backend.name, started)
        return response

    def _on_plan_cartesian(self, request: Any, response: Any) -> Any:
        started = time.monotonic()
        response.ok = False
        response.fraction = 0.0
        req = PlanCartesianRequest(
            planning_group=request.planning_group,
            tip_link=request.tip_link,
            waypoints=list(request.waypoints),
            start_state=request.start_state,
            eef_step_m=float(request.eef_step_m),
            min_fraction=float(request.min_fraction),
            avoid_collisions=bool(request.avoid_collisions),
            max_velocity_scaling=float(request.max_velocity_scaling),
            max_acceleration_scaling=float(request.max_acceleration_scaling),
        )
        result = self._run(response, started, lambda: self.backend.plan_cartesian(req))
        if result is not None:
            response.ok = True
            response.trajectory = result.trajectory
            response.fraction = float(result.fraction)
            response.report = make_report(self.backend.name, started)
        return response

    def _on_check_trajectory(self, request: Any, response: Any) -> Any:
        started = time.monotonic()
        response.valid = False
        response.first_invalid_index = -1
        req = CheckTrajectoryRequest(
            planning_group=request.planning_group,
            trajectory=request.trajectory,
            check_resolution_rad=float(request.check_resolution_rad),
        )
        result = self._run(response, started, lambda: self.backend.check_trajectory(req))
        if result is not None:
            response.valid = bool(result.valid)
            response.first_invalid_index = int(result.first_invalid_index)
            response.contacts = list(result.contacts)
            code = failure_codes.NONE if result.valid else failure_codes.TRAJECTORY_IN_COLLISION
            response.report = make_report(self.backend.name, started, code)
        return response

    def _on_add_collision_object(self, request: Any, response: Any) -> Any:
        started = time.monotonic()
        response.ok = False
        result = self._run(response, started, lambda: self.backend.add_collision_object(request.object))
        if result is not None:
            response.ok = bool(result.ok)
            response.message = result.message
            code = failure_codes.NONE if result.ok else failure_codes.WORLD_ERROR
            response.report = make_report(self.backend.name, started, code, "" if result.ok else result.message)
        return response

    def _on_remove_collision_object(self, request: Any, response: Any) -> Any:
        started = time.monotonic()
        response.ok = False
        result = self._run(response, started, lambda: self.backend.remove_collision_object(request.object_id))
        if result is not None:
            response.ok = bool(result.ok)
            response.message = result.message
            code = failure_codes.NONE if result.ok else failure_codes.WORLD_ERROR
            response.report = make_report(self.backend.name, started, code, "" if result.ok else result.message)
        return response

    def _on_sync_collision_objects(self, request: Any, response: Any) -> Any:
        started = time.monotonic()
        response.ok = False
        result = self._run(
            response, started, lambda: self.backend.sync_collision_objects(list(request.object_ids))
        )
        if result is not None:
            response.ok = bool(result.ok)
            response.message = result.message
            response.synced_object_ids = list(result.synced)
            response.failed_object_ids = list(result.failed)
            response.skipped_object_ids = list(result.skipped)
            code = failure_codes.NONE if result.ok else failure_codes.WORLD_ERROR
            response.report = make_report(self.backend.name, started, code, "" if result.ok else result.message)
        return response
