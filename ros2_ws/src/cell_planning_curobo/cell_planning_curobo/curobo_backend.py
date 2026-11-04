"""cuRobo v0.7.8 backend.

API surface used (cuRobo v1 API, tag v0.7.8):
    curobo.wrap.reacher.motion_gen   MotionGen, MotionGenConfig, MotionGenPlanConfig
    curobo.wrap.reacher.ik_solver    IKSolver, IKSolverConfig
    curobo.wrap.model.robot_world    RobotWorld, RobotWorldConfig
    curobo.types.math                Pose
    curobo.types.state               JointState
    curobo.geom.types                WorldConfig and shapes (see world.py)
    curobo.util.xrdf_utils           convert_xrdf_to_curobo (see robot_config.py)

Mapping of the service onto cuRobo:
    plan_to_pose      MotionGen.plan_single(start, goal_pose)
    plan_to_joints    MotionGen.plan_single_js(start, goal_js)
    plan_cartesian    IK chain: interpolate the tip pose along the segment at
                      eef_step_m, solve each step with IKSolver seeded by the
                      previous solution, reject steps whose joint jump exceeds
                      the collision-check resolution, check every step with
                      RobotWorld, then time-parameterize with
                      cell_planning_service.timing. This is a straight line
                      in Cartesian space by construction. (MotionGen's
                      constrained planning could replace it later; the IK
                      chain is simpler to reason about for insertion.)
    check_trajectory  RobotWorld world + self collision distance over the
                      interpolated joint path
    world             world.World -> WorldConfig; MotionGen.update_world

Every default the request leaves at zero comes from planner.curobo in
config/cell.yaml, and a missing one is INVALID_REQUEST, never invented.

The library is imported lazily in __init__ so the package stays importable
and testable without cuRobo. The node re-execs into the cuRobo Python
before constructing this backend (venv.py).
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from robot_backend_interfaces.srv import SyncSceneObjects
from sensor_msgs.msg import JointState
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
from cell_planning_service.timing import interpolate_joint_path, load_joint_limits_yaml, time_parameterize

from cell_planning_curobo.robot_config import load_robot_config
from cell_planning_curobo.world import World

SERVICE_SYNC = "/sync_scene_objects"
SERVICE_GET_SCENE = "/get_planning_scene"


class CuRoboBackend(PlanningBackend):
    name = "curobo"

    def __init__(
        self,
        node: Any,
        *,
        defaults: dict[str, Any],
        repo_root: Path,
        joint_limits_text: str | None,
        planning_frame: str,
        callback_group: Any = None,
        joint_state_topic: str = "/joint_states",
        depth_layer: Any | None = None,
    ) -> None:
        self.node = node
        self.defaults = dict(defaults or {})
        self.repo_root = repo_root
        self.joint_limits_text = joint_limits_text
        self.planning_frame = planning_frame
        self.depth_layer = depth_layer
        self._lock = threading.Lock()
        self._joint_state: JointState | None = None
        self.world = World(planning_frame)
        self._world_version_loaded = -1
        kw = {"callback_group": callback_group} if callback_group is not None else {}
        self._sync = node.create_client(SyncSceneObjects, SERVICE_SYNC, **kw)
        self._get_scene = node.create_client(GetPlanningScene, SERVICE_GET_SCENE, **kw)
        node.create_subscription(JointState, joint_state_topic, self._on_joint_state, 10, **kw)
        self._init_curobo()

    # -- setup -------------------------------------------------------------

    def _resolve(self, spec: str) -> Path:
        if ":" in spec and not spec.startswith("/"):
            package, rel = spec.split(":", 1)
            from ament_index_python.packages import get_package_share_directory

            return Path(get_package_share_directory(package)) / rel
        p = Path(spec).expanduser()
        return p if p.is_absolute() else self.repo_root / p

    def _init_curobo(self) -> None:
        try:
            import torch  # type: ignore
            from curobo.types.base import TensorDeviceType  # type: ignore
            from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig  # type: ignore
            from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig  # type: ignore
            from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig  # type: ignore
        except ImportError as exc:
            raise RuntimeError(f"cuRobo environment incomplete: {exc} (see cell_planning_curobo/INSTALL.md)") from exc
        if not torch.cuda.is_available():
            raise RuntimeError("cuRobo backend needs a CUDA device; torch.cuda.is_available() is False")
        self._tensor_args = TensorDeviceType()
        robot_cfg = load_robot_config(
            self._resolve(str(self.defaults["urdf"])),
            self._resolve(str(self.defaults["xrdf"])),
            self._resolve(str(self.defaults["rg2_spheres"])),
            str(self.defaults.get("base_link", "base_link")),
            str(self.defaults.get("ee_link", "tcp")),
        )
        self._robot_cfg = robot_cfg
        overrides = dict(self.defaults.get("motion_gen_overrides", {}) or {})
        world_cfg = self.world.to_curobo_world_config()
        self._motion_gen = MotionGen(
            MotionGenConfig.load_from_robot_config(
                robot_cfg, world_cfg, tensor_args=self._tensor_args, **overrides
            )
        )
        self._motion_gen.warmup()
        self._ik = IKSolver(
            IKSolverConfig.load_from_robot_config(robot_cfg, world_cfg, tensor_args=self._tensor_args)
        )
        self._robot_world = RobotWorld(
            RobotWorldConfig.load_from_config(robot_cfg, world_cfg, tensor_args=self._tensor_args)
        )
        self._joint_names: list[str] = list(self._motion_gen.kinematics.joint_names)
        self._world_version_loaded = self.world.version

    def _refresh_world(self) -> None:
        if self._world_version_loaded == self.world.version and self.depth_layer is None:
            return
        world_cfg = self.world.to_curobo_world_config()
        if self.depth_layer is not None:
            world_cfg = self.depth_layer.augment(world_cfg)
        self._motion_gen.update_world(world_cfg)
        self._ik.update_world(world_cfg)
        self._robot_world.update_world(world_cfg)
        self._world_version_loaded = self.world.version

    # -- defaults and state -----------------------------------------------

    def _d(self, key: str) -> Any:
        if key not in self.defaults or self.defaults[key] in (None, 0, 0.0, ""):
            raise PlanningFailure(
                failure_codes.INVALID_REQUEST,
                f"request left {key} unset and config/cell.yaml planner.curobo.{key} is missing or zero",
            )
        return self.defaults[key]

    def _pick(self, value: float, key: str) -> float:
        return float(value) if value > 0 else float(self._d(key))

    def _pick_int(self, value: int, key: str) -> int:
        return int(value) if value > 0 else int(self._d(key))

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            self._joint_state = msg

    def _start_positions(self, given: JointState) -> list[float]:
        source = given if list(given.name) else None
        if source is None:
            deadline = time.monotonic() + float(self.defaults.get("joint_state_timeout_s", 1.0))
            while source is None:
                with self._lock:
                    source = self._joint_state
                if source is None and time.monotonic() > deadline:
                    raise PlanningFailure(failure_codes.START_STATE_INVALID, "no start_state and no /joint_states")
                if source is None:
                    time.sleep(0.01)
        table = dict(zip(source.name, source.position))
        missing = [n for n in self._joint_names if n not in table]
        if missing:
            raise PlanningFailure(failure_codes.START_STATE_INVALID, f"start state lacks joints {missing}")
        return [float(table[n]) for n in self._joint_names]

    def _js(self, positions: list[float]) -> Any:
        from curobo.types.state import JointState as CuJointState  # type: ignore

        return CuJointState.from_position(
            self._tensor_args.to_device([positions]), joint_names=list(self._joint_names)
        )

    def _pose(self, ps: PoseStamped) -> Any:
        from curobo.types.math import Pose as CuPose  # type: ignore

        if ps.header.frame_id and ps.header.frame_id != self.planning_frame:
            raise PlanningFailure(
                failure_codes.INVALID_REQUEST,
                f"target frame {ps.header.frame_id!r} differs from planning frame {self.planning_frame!r}; "
                "the cuRobo backend does no TF",
            )
        p, q = ps.pose.position, ps.pose.orientation
        return CuPose.from_list([p.x, p.y, p.z, q.w, q.x, q.y, q.z], tensor_args=self._tensor_args)

    def _to_trajectory(self, positions: list[list[float]], times: list[float], velocities: list[list[float]] | None) -> JointTrajectory:
        out = JointTrajectory()
        out.header.frame_id = self.planning_frame
        out.joint_names = list(self._joint_names)
        for i, (q, t) in enumerate(zip(positions, times)):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            if velocities is not None:
                pt.velocities = [float(v) for v in velocities[i]]
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            out.points.append(pt)
        return out

    def _plan_config(self, attempts: int, planning_time: float, vel: float, acc: float) -> Any:
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig  # type: ignore

        # v0.7.x: max_attempts, timeout, and time_dilation_factor (slows the
        # result; 1.0 = full speed). Velocity scaling maps onto dilation; the
        # acceleration scaling has no direct knob and is reported as unused.
        dilation = vel if 0 < vel <= 1 else 1.0
        return MotionGenPlanConfig(
            enable_graph=False,
            max_attempts=max(1, attempts),
            timeout=planning_time,
            time_dilation_factor=dilation,
        )

    def _result_to_trajectory(self, result: Any) -> JointTrajectory:
        traj = result.get_interpolated_plan()
        dt = float(result.interpolation_dt)
        positions = traj.position.cpu().numpy().tolist()
        velocities = traj.velocity.cpu().numpy().tolist() if traj.velocity is not None else None
        times = [i * dt for i in range(len(positions))]
        return self._to_trajectory(positions, times, velocities)

    def _check_result(self, result: Any) -> None:
        if result.success is not None and bool(result.success.item()):
            return
        status = str(result.status) if result.status is not None else "unknown"
        code = failure_codes.PLANNING_FAILED
        upper = status.upper()
        if "IK" in upper:
            code = failure_codes.NO_IK_SOLUTION
        elif "START" in upper and "COLLISION" in upper:
            code = failure_codes.START_STATE_IN_COLLISION
        elif "GOAL" in upper and "COLLISION" in upper:
            code = failure_codes.GOAL_IN_COLLISION
        elif "LIMIT" in upper:
            code = failure_codes.GOAL_VIOLATES_LIMITS
        raise PlanningFailure(code, f"cuRobo status: {status}")

    # -- planning ----------------------------------------------------------

    def plan_to_pose(self, r: PlanToPoseRequest) -> PlanResult:
        if not r.tip_link or r.tip_link != str(self.defaults.get("ee_link", "tcp")):
            raise PlanningFailure(
                failure_codes.INVALID_REQUEST,
                f"tip_link must be the configured ee_link {self.defaults.get('ee_link', 'tcp')!r}",
            )
        self._refresh_world()
        start = self._js(self._start_positions(r.start_state))
        cfg = self._plan_config(
            self._pick_int(r.num_planning_attempts, "num_planning_attempts"),
            self._pick(r.allowed_planning_time_s, "allowed_planning_time_s"),
            r.max_velocity_scaling,
            r.max_acceleration_scaling,
        )
        result = self._motion_gen.plan_single(start, self._pose(r.target), cfg)
        self._check_result(result)
        return PlanResult(trajectory=self._result_to_trajectory(result))

    def plan_to_joints(self, r: PlanToJointsRequest) -> PlanResult:
        if len(r.joint_names) != len(r.joint_positions):
            raise PlanningFailure(failure_codes.INVALID_REQUEST, "joint_names and joint_positions differ in length")
        table = dict(zip(r.joint_names, r.joint_positions))
        missing = [n for n in self._joint_names if n not in table]
        if missing:
            raise PlanningFailure(failure_codes.INVALID_REQUEST, f"goal lacks joints {missing}")
        self._refresh_world()
        start = self._js(self._start_positions(r.start_state))
        goal = self._js([float(table[n]) for n in self._joint_names])
        cfg = self._plan_config(
            self._pick_int(r.num_planning_attempts, "num_planning_attempts"),
            self._pick(r.allowed_planning_time_s, "allowed_planning_time_s"),
            r.max_velocity_scaling,
            r.max_acceleration_scaling,
        )
        result = self._motion_gen.plan_single_js(start, goal, cfg)
        self._check_result(result)
        return PlanResult(trajectory=self._result_to_trajectory(result))

    def plan_cartesian(self, r: PlanCartesianRequest) -> PlanResult:
        if not r.waypoints:
            raise PlanningFailure(failure_codes.INVALID_REQUEST, "at least one waypoint is required")
        self._refresh_world()
        step = self._pick(r.eef_step_m, "eef_step_m")
        resolution = self._pick(0.0, "check_resolution_rad")
        start_q = self._start_positions(r.start_state)
        # Start pose from forward kinematics of the start state.
        fk = self._motion_gen.kinematics.get_state(self._tensor_args.to_device([start_q]))
        start_pose = fk.ee_pose
        poses = [start_pose] + [self._pose(w) for w in r.waypoints]
        path: list[list[float]] = [start_q]
        total_steps = 0
        achieved_steps = 0
        for a, b in zip(poses, poses[1:]):
            n = max(1, int(_distance(a, b) / step + 0.999999))
            total_steps += n
            for s in range(1, n + 1):
                target = a.interpolate(b, s / n) if hasattr(a, "interpolate") else _lerp_pose(a, b, s / n, self._tensor_args)
                seed = self._tensor_args.to_device([path[-1]]).unsqueeze(0)
                ik = self._ik.solve_single(target, seed_config=seed, retract_config=self._tensor_args.to_device(path[-1]))
                if not bool(ik.success.item()):
                    return self._partial(path, achieved_steps, total_steps, r, failure_codes.NO_IK_SOLUTION)
                q = ik.solution[0, 0].cpu().numpy().tolist()
                if max(abs(x - y) for x, y in zip(q, path[-1])) > resolution * 10:
                    return self._partial(path, achieved_steps, total_steps, r, failure_codes.PLANNING_FAILED)
                if r.avoid_collisions and not self._state_free(q):
                    return self._partial(path, achieved_steps, total_steps, r, failure_codes.TRAJECTORY_IN_COLLISION)
                path.append(q)
                achieved_steps += 1
        return self._timed(path, 1.0, r)

    def _partial(self, path: list[list[float]], done: int, total: int, r: PlanCartesianRequest, code: str) -> PlanResult:
        fraction = done / total if total else 0.0
        min_fraction = r.min_fraction if 0 < r.min_fraction <= 1 else 1.0
        if fraction + 1e-9 < min_fraction or len(path) < 2:
            raise PlanningFailure(failure_codes.PARTIAL_PATH, f"{code} at fraction {fraction:.3f} (min {min_fraction:.3f})")
        return self._timed(path, fraction, r)

    def _timed(self, path: list[list[float]], fraction: float, r: PlanCartesianRequest) -> PlanResult:
        if not self.joint_limits_text:
            raise PlanningFailure(failure_codes.BACKEND_ERROR, "no joint_limits_file configured for Cartesian retiming")
        limits = load_joint_limits_yaml(self.joint_limits_text, self._joint_names)
        timed = time_parameterize(
            path, [limits[n] for n in self._joint_names],
            velocity_scaling=r.max_velocity_scaling, acceleration_scaling=r.max_acceleration_scaling,
        )
        return PlanResult(trajectory=self._to_trajectory(timed.positions, timed.times, timed.velocities), fraction=fraction)

    # -- checking ----------------------------------------------------------

    def _state_free(self, q: list[float]) -> bool:
        d_world, d_self = self._robot_world.get_world_self_collision_distance_from_joints(
            self._tensor_args.to_device([q])
        )
        return float(d_world.max().item()) <= 0.0 and float(d_self.max().item()) <= 0.0

    def check_trajectory(self, r: CheckTrajectoryRequest) -> CheckResult:
        traj = r.trajectory
        if not traj.joint_names or not traj.points:
            raise PlanningFailure(failure_codes.INVALID_REQUEST, "a non-empty trajectory is required")
        order = [list(traj.joint_names).index(n) for n in self._joint_names if n in traj.joint_names]
        if len(order) != len(self._joint_names):
            raise PlanningFailure(failure_codes.INVALID_REQUEST, "trajectory lacks some robot joints")
        self._refresh_world()
        resolution = self._pick(r.check_resolution_rad, "check_resolution_rad")
        states = interpolate_joint_path([[p.positions[i] for i in order] for p in traj.points], resolution)
        for index, q in states:
            if not self._state_free(q):
                return CheckResult(valid=False, first_invalid_index=index, contacts=["(cuRobo reports distance only)"])
        return CheckResult(valid=True)

    # -- world -------------------------------------------------------------

    def add_collision_object(self, obj: CollisionObject) -> WorldResult:
        try:
            ids = self.world.add(obj)
        except ValueError as exc:
            raise PlanningFailure(failure_codes.INVALID_REQUEST, str(exc)) from exc
        return WorldResult(ok=True, message=f"added {ids}")

    def remove_collision_object(self, object_id: str) -> WorldResult:
        n = self.world.remove(object_id)
        return WorldResult(ok=n > 0, message=f"removed {n} record(s)")

    def sync_collision_objects(self, object_ids: list[str]) -> WorldResult:
        """Delegate to the registry owner, then mirror MoveIt's world here.

        /sync_scene_objects (scene_tf_sync_node) applies registry geometry at
        TF poses to MoveIt's planning scene. This backend then reads that
        scene back and replaces its matching records, so the cuRobo world
        stays registry-free while agreeing with MoveIt on what was synced.
        Requires move_group to be running for /get_planning_scene.
        """
        res = self._call(self._sync, _sync_request(object_ids), SERVICE_SYNC)
        synced = list(res.synced_object_ids)
        if synced:
            scene = self._call(self._get_scene, _scene_request(), SERVICE_GET_SCENE).scene
            by_id = {o.id: o for o in scene.world.collision_objects}
            for oid in synced:
                if oid in by_id:
                    self.world.add(by_id[oid])
        return WorldResult(
            ok=bool(res.success), message=str(res.message), synced=synced,
            failed=list(res.failed_object_ids), skipped=list(res.skipped_object_ids),
        )

    def _call(self, client: Any, request: Any, what: str) -> Any:
        if not client.wait_for_service(timeout_sec=float(self.defaults.get("service_wait_s", 2.0))):
            raise PlanningFailure(failure_codes.BACKEND_UNAVAILABLE, f"{what} not available")
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(float(self.defaults.get("service_timeout_s", 30.0))):
            raise PlanningFailure(failure_codes.TIMEOUT, f"{what} did not answer")
        if future.exception() is not None:
            raise PlanningFailure(failure_codes.BACKEND_ERROR, f"{what}: {future.exception()}")
        return future.result()


def _sync_request(object_ids: list[str]) -> Any:
    req = SyncSceneObjects.Request()
    req.object_ids = list(object_ids)
    return req


def _scene_request() -> Any:
    req = GetPlanningScene.Request()
    req.components.components = (
        PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    )
    return req


def _distance(a: Any, b: Any) -> float:
    pa = a.position.view(-1)[:3]
    pb = b.position.view(-1)[:3]
    return float((pa - pb).norm().item())


def _lerp_pose(a: Any, b: Any, t: float, tensor_args: Any) -> Any:
    """Linear position, slerp orientation, for cuRobo Pose objects."""
    import torch  # type: ignore
    from curobo.types.math import Pose as CuPose  # type: ignore

    pa, pb = a.position.view(-1)[:3], b.position.view(-1)[:3]
    qa, qb = a.quaternion.view(-1)[:4], b.quaternion.view(-1)[:4]
    pos = pa + (pb - pa) * t
    dot = torch.dot(qa, qb)
    if dot < 0:
        qb, dot = -qb, -dot
    if dot > 0.9995:
        quat = qa + (qb - qa) * t
    else:
        theta = torch.acos(dot)
        quat = (torch.sin((1 - t) * theta) * qa + torch.sin(t * theta) * qb) / torch.sin(theta)
    quat = quat / quat.norm()
    return CuPose(position=pos.unsqueeze(0), quaternion=quat.unsqueeze(0))
