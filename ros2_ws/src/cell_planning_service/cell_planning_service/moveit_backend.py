"""MoveIt plan-only backend.

Everything goes through move_group's planning services, none of which can
execute:

    /plan_kinematic_path   (GetMotionPlan)     plan_to_pose, plan_to_joints
    /compute_cartesian_path (GetCartesianPath)  plan_cartesian
    /check_state_validity  (GetStateValidity)   check_trajectory
    /apply_planning_scene  (ApplyPlanningScene) add/remove collision objects
    /sync_scene_objects    (SyncSceneObjects)   sync, delegated to the cell's
                                                 scene_tf_sync_node, which owns
                                                 the registry

The MoveGroup action is never used, so no MoveIt controller is ever
engaged from here. The world comes only from the explicit collision
object calls (plus whatever the sync node pushes on request).

Defaults the request leaves at zero are filled from the `planner.moveit`
section of config/cell.yaml; nothing numeric is hardcoded here except
the message constants.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetMotionPlan,
    GetStateValidity,
)
from robot_backend_interfaces.srv import SyncSceneObjects
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cell_planning_service import failure_codes
from cell_planning_service.backend import (
    CheckResult,
    CheckTrajectoryRequest,
    PlanCartesianRequest,
    PlanningBackend,
    PlanningFailure,
    PlanResult,
    PlanToJointsRequest,
    PlanToPoseRequest,
    WorldResult,
)
from cell_planning_service.timing import (
    JointLimit,
    interpolate_joint_path,
    load_joint_limits_yaml,
    time_parameterize,
)

SERVICE_PLAN = "/plan_kinematic_path"
SERVICE_CARTESIAN = "/compute_cartesian_path"
SERVICE_VALIDITY = "/check_state_validity"
SERVICE_APPLY_SCENE = "/apply_planning_scene"
SERVICE_SYNC = "/sync_scene_objects"


class MoveItBackend(PlanningBackend):
    name = "moveit"

    def __init__(
        self,
        node: Any,
        *,
        defaults: dict[str, Any],
        joint_limits_text: str | None,
        callback_group: Any = None,
        joint_state_topic: str = "/joint_states",
    ) -> None:
        """
        defaults: the `planner.moveit` mapping from config/cell.yaml. Keys used:
            allowed_planning_time_s, num_planning_attempts, position_tolerance_m,
            orientation_tolerance_rad, joint_tolerance_rad, eef_step_m,
            jump_threshold, check_resolution_rad, service_timeout_s,
            joint_state_timeout_s, planner_id, pipeline_id
        joint_limits_text: contents of a MoveIt joint_limits.yaml, used only
            when the Cartesian service returns an untimed path.
        """
        self.node = node
        self.defaults = dict(defaults or {})
        self.joint_limits_text = joint_limits_text
        self._cb = callback_group
        self._lock = threading.Lock()
        self._joint_state: JointState | None = None
        self._joint_state_wall: float = 0.0
        kw = {"callback_group": callback_group} if callback_group is not None else {}
        self._plan = node.create_client(GetMotionPlan, SERVICE_PLAN, **kw)
        self._cartesian = node.create_client(GetCartesianPath, SERVICE_CARTESIAN, **kw)
        self._validity = node.create_client(GetStateValidity, SERVICE_VALIDITY, **kw)
        self._apply = node.create_client(ApplyPlanningScene, SERVICE_APPLY_SCENE, **kw)
        self._sync = node.create_client(SyncSceneObjects, SERVICE_SYNC, **kw)
        self._joint_sub = node.create_subscription(
            JointState, joint_state_topic, self._on_joint_state, 10, **kw
        )

    # -- defaults ----------------------------------------------------------

    def _d(self, key: str) -> Any:
        if key not in self.defaults:
            raise PlanningFailure(
                failure_codes.INVALID_REQUEST,
                f"request left {key} unset and config/cell.yaml planner.moveit.{key} is missing",
            )
        return self.defaults[key]

    def _pick(self, value: float, key: str) -> float:
        return float(value) if value > 0 else float(self._d(key))

    def _pick_int(self, value: int, key: str) -> int:
        return int(value) if value > 0 else int(self._d(key))

    def _timeout(self) -> float:
        return float(self.defaults.get("service_timeout_s", 30.0))

    # -- joint state -------------------------------------------------------

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            self._joint_state = msg
            self._joint_state_wall = time.monotonic()

    def _start_state(self, given: JointState) -> RobotState:
        state = RobotState()
        if list(given.name):
            state.joint_state = given
            state.is_diff = False
            return state
        deadline = time.monotonic() + float(self.defaults.get("joint_state_timeout_s", 1.0))
        while True:
            with self._lock:
                js = self._joint_state
            if js is not None:
                state.joint_state = js
                state.is_diff = False
                return state
            if time.monotonic() > deadline:
                raise PlanningFailure(
                    failure_codes.START_STATE_INVALID,
                    "no start_state given and no /joint_states received",
                )
            time.sleep(0.01)

    # -- service call helper ----------------------------------------------

    def _call(self, client: Any, request: Any, what: str) -> Any:
        if not client.wait_for_service(timeout_sec=float(self.defaults.get("service_wait_s", 2.0))):
            raise PlanningFailure(failure_codes.BACKEND_UNAVAILABLE, f"{what} not available")
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(self._timeout()):
            raise PlanningFailure(failure_codes.TIMEOUT, f"{what} did not answer in {self._timeout():.1f}s")
        if future.exception() is not None:
            raise PlanningFailure(failure_codes.BACKEND_ERROR, f"{what}: {future.exception()}")
        return future.result()

    # -- planning ----------------------------------------------------------

    def _base_request(
        self,
        group: str,
        start: RobotState,
        planning_time: float,
        attempts: int,
        vel: float,
        acc: float,
        planner_id: str,
    ) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name = group
        req.start_state = start
        req.allowed_planning_time = planning_time
        req.num_planning_attempts = attempts
        req.max_velocity_scaling_factor = vel if 0 < vel <= 1 else 1.0
        req.max_acceleration_scaling_factor = acc if 0 < acc <= 1 else 1.0
        req.planner_id = planner_id or str(self.defaults.get("planner_id", ""))
        req.pipeline_id = str(self.defaults.get("pipeline_id", ""))
        return req

    def _submit(self, req: MotionPlanRequest) -> JointTrajectory:
        srv = GetMotionPlan.Request()
        srv.motion_plan_request = req
        res = self._call(self._plan, srv, SERVICE_PLAN).motion_plan_response
        code = int(res.error_code.val)
        if code != 1:
            raise PlanningFailure(failure_codes.from_moveit(code), f"MoveItErrorCodes {code}")
        traj = res.trajectory.joint_trajectory
        if not traj.points:
            raise PlanningFailure(failure_codes.PLANNING_FAILED, "planner returned an empty trajectory")
        return traj

    def plan_to_pose(self, r: PlanToPoseRequest) -> PlanResult:
        if not r.planning_group or not r.tip_link or not r.target.header.frame_id:
            raise PlanningFailure(
                failure_codes.INVALID_REQUEST, "planning_group, tip_link and target.header.frame_id are required"
            )
        start = self._start_state(r.start_state)
        req = self._base_request(
            r.planning_group,
            start,
            self._pick(r.allowed_planning_time_s, "allowed_planning_time_s"),
            self._pick_int(r.num_planning_attempts, "num_planning_attempts"),
            r.max_velocity_scaling,
            r.max_acceleration_scaling,
            r.planner_id,
        )
        req.goal_constraints = [
            _pose_constraints(
                r.target,
                r.tip_link,
                self._pick(r.position_tolerance_m, "position_tolerance_m"),
                self._pick(r.orientation_tolerance_rad, "orientation_tolerance_rad"),
            )
        ]
        return PlanResult(trajectory=self._submit(req))

    def plan_to_joints(self, r: PlanToJointsRequest) -> PlanResult:
        if not r.planning_group or not r.joint_names or len(r.joint_names) != len(r.joint_positions):
            raise PlanningFailure(
                failure_codes.INVALID_REQUEST, "planning_group and matching joint_names/joint_positions are required"
            )
        start = self._start_state(r.start_state)
        req = self._base_request(
            r.planning_group,
            start,
            self._pick(r.allowed_planning_time_s, "allowed_planning_time_s"),
            self._pick_int(r.num_planning_attempts, "num_planning_attempts"),
            r.max_velocity_scaling,
            r.max_acceleration_scaling,
            r.planner_id,
        )
        tol = self._pick(r.joint_tolerance_rad, "joint_tolerance_rad")
        c = Constraints()
        c.name = "joint_goal"
        for name, pos in zip(r.joint_names, r.joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]
        return PlanResult(trajectory=self._submit(req))

    def plan_cartesian(self, r: PlanCartesianRequest) -> PlanResult:
        if not r.planning_group or not r.tip_link or not r.waypoints:
            raise PlanningFailure(
                failure_codes.INVALID_REQUEST, "planning_group, tip_link and at least one waypoint are required"
            )
        frames = {w.header.frame_id for w in r.waypoints}
        if len(frames) != 1 or "" in frames:
            raise PlanningFailure(failure_codes.INVALID_REQUEST, "all waypoints must share one non-empty frame_id")
        start = self._start_state(r.start_state)
        srv = GetCartesianPath.Request()
        srv.header.frame_id = r.waypoints[0].header.frame_id
        srv.start_state = start
        srv.group_name = r.planning_group
        srv.link_name = r.tip_link
        srv.waypoints = [w.pose for w in r.waypoints]
        srv.max_step = self._pick(r.eef_step_m, "eef_step_m")
        srv.jump_threshold = float(self.defaults.get("jump_threshold", 0.0))
        srv.avoid_collisions = bool(r.avoid_collisions)
        srv.max_velocity_scaling_factor = r.max_velocity_scaling if 0 < r.max_velocity_scaling <= 1 else 1.0
        srv.max_acceleration_scaling_factor = (
            r.max_acceleration_scaling if 0 < r.max_acceleration_scaling <= 1 else 1.0
        )
        res = self._call(self._cartesian, srv, SERVICE_CARTESIAN)
        code = int(res.error_code.val)
        if code != 1:
            raise PlanningFailure(failure_codes.from_moveit(code), f"MoveItErrorCodes {code}")
        fraction = float(res.fraction)
        min_fraction = r.min_fraction if 0 < r.min_fraction <= 1 else 1.0
        traj = res.solution.joint_trajectory
        if not traj.points:
            raise PlanningFailure(failure_codes.PLANNING_FAILED, "Cartesian path is empty")
        if fraction + 1e-9 < min_fraction:
            raise PlanningFailure(
                failure_codes.PARTIAL_PATH, f"fraction {fraction:.3f} below min_fraction {min_fraction:.3f}"
            )
        if _is_untimed(traj):
            traj = self._retime(traj, r.max_velocity_scaling, r.max_acceleration_scaling)
        return PlanResult(trajectory=traj, fraction=fraction)

    def _retime(self, traj: JointTrajectory, vel: float, acc: float) -> JointTrajectory:
        if not self.joint_limits_text:
            raise PlanningFailure(
                failure_codes.BACKEND_ERROR,
                "Cartesian path came back without timing and no joint_limits file is configured",
            )
        limits = load_joint_limits_yaml(self.joint_limits_text, list(traj.joint_names))
        ordered = [limits[n] for n in traj.joint_names]
        timed = time_parameterize(
            [list(p.positions) for p in traj.points], ordered,
            velocity_scaling=vel, acceleration_scaling=acc,
        )
        out = JointTrajectory()
        out.header = traj.header
        out.joint_names = list(traj.joint_names)
        for t, q, v in zip(timed.times, timed.positions, timed.velocities):
            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.velocities = v
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            out.points.append(pt)
        return out

    # -- checking ----------------------------------------------------------

    def check_trajectory(self, r: CheckTrajectoryRequest) -> CheckResult:
        traj = r.trajectory
        if not r.planning_group or not traj.joint_names or not traj.points:
            raise PlanningFailure(failure_codes.INVALID_REQUEST, "planning_group and a non-empty trajectory are required")
        resolution = self._pick(r.check_resolution_rad, "check_resolution_rad")
        states = interpolate_joint_path([list(p.positions) for p in traj.points], resolution)
        for index, q in states:
            srv = GetStateValidity.Request()
            srv.group_name = r.planning_group
            srv.robot_state.joint_state.name = list(traj.joint_names)
            srv.robot_state.joint_state.position = q
            res = self._call(self._validity, srv, SERVICE_VALIDITY)
            if not res.valid:
                contacts = sorted({f"{c.contact_body_1}|{c.contact_body_2}" for c in res.contacts})
                return CheckResult(valid=False, first_invalid_index=index, contacts=contacts)
        return CheckResult(valid=True)

    # -- world -------------------------------------------------------------

    def _apply_object(self, obj: CollisionObject) -> WorldResult:
        srv = ApplyPlanningScene.Request()
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        srv.scene = scene
        res = self._call(self._apply, srv, SERVICE_APPLY_SCENE)
        return WorldResult(ok=bool(res.success), message="" if res.success else "apply_planning_scene refused")

    def add_collision_object(self, obj: CollisionObject) -> WorldResult:
        if not obj.id or not obj.header.frame_id:
            raise PlanningFailure(failure_codes.INVALID_REQUEST, "object.id and object.header.frame_id are required")
        if not (obj.primitives or obj.meshes or obj.planes):
            raise PlanningFailure(failure_codes.INVALID_REQUEST, "object carries no geometry")
        obj.operation = CollisionObject.ADD
        return self._apply_object(obj)

    def remove_collision_object(self, object_id: str) -> WorldResult:
        if not object_id:
            raise PlanningFailure(failure_codes.INVALID_REQUEST, "object_id is required")
        obj = CollisionObject()
        obj.id = object_id
        obj.operation = CollisionObject.REMOVE
        return self._apply_object(obj)

    def sync_collision_objects(self, object_ids: list[str]) -> WorldResult:
        srv = SyncSceneObjects.Request()
        srv.object_ids = list(object_ids)
        res = self._call(self._sync, srv, SERVICE_SYNC)
        return WorldResult(
            ok=bool(res.success),
            message=str(res.message),
            synced=list(res.synced_object_ids),
            failed=list(res.failed_object_ids),
            skipped=list(res.skipped_object_ids),
        )


def _pose_constraints(target: PoseStamped, link: str, pos_tol: float, ori_tol: float) -> Constraints:
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [max(pos_tol, 1e-6)]
    region_pose = Pose()
    region_pose.position = target.pose.position
    region_pose.orientation.w = 1.0
    volume = BoundingVolume()
    volume.primitives = [sphere]
    volume.primitive_poses = [region_pose]

    pc = PositionConstraint()
    pc.header = target.header
    pc.link_name = link
    pc.constraint_region = volume
    pc.weight = 1.0

    oc = OrientationConstraint()
    oc.header = target.header
    oc.link_name = link
    oc.orientation = target.pose.orientation
    oc.absolute_x_axis_tolerance = ori_tol
    oc.absolute_y_axis_tolerance = ori_tol
    oc.absolute_z_axis_tolerance = ori_tol
    oc.weight = 1.0

    c = Constraints()
    c.name = "pose_goal"
    c.position_constraints = [pc]
    c.orientation_constraints = [oc]
    return c


def _is_untimed(traj: JointTrajectory) -> bool:
    if len(traj.points) < 2:
        return False
    last = traj.points[-1].time_from_start
    return last.sec == 0 and last.nanosec == 0


__all__ = ["MoveItBackend", "JointLimit"]
