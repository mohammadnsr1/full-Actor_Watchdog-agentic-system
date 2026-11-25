"""ROS side of the planner template: one node, the platform backends, the
planning service, the fact journal and WatchDog notices (halts, rulings,
consultations). The start permission is the episode runner's: execute_task is only
called once the WatchDog (if any) is ready.

A MultiThreadedExecutor spins the node in a background thread; every call
here blocks the caller (the MCP tool thread) by polling a future, which is the
way the platform backends wait too. Every name comes from the platform config.
"""

from __future__ import annotations

import importlib
import logging
import threading
import time
from typing import Any

import yaml

log = logging.getLogger("planner")


class Halted(Exception):
    """The active trajectory was cancelled (a WatchDog halt or a runner stop)."""


class StepFailed(Exception):
    pass


def _wait(future: Any, timeout_s: float) -> Any:
    start = time.monotonic()
    while not future.done():
        if time.monotonic() - start > timeout_s:
            return None
        time.sleep(0.005)
    return future.result()


class Worker:
    def __init__(self, platform_name: str, repo_root: str, node_name: str = "planner") -> None:
        import rclpy
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter

        from cell_platform_interfaces.platform_config import load_platform_config

        if not rclpy.ok():
            rclpy.init(args=None)
        self.platform = load_platform_config(platform_name)
        with open(f"{repo_root}/config/cell.yaml", "r", encoding="utf-8") as handle:
            self.cell = yaml.safe_load(handle) or {}
        use_sim_time = bool(self.platform.get_optional("use_sim_time", default=False))
        self.node = Node(node_name, parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)])
        self.cb = ReentrantCallbackGroup()
        backend_pkg = {"sim": "cell_backend_sim", "hardware": "cell_backend_hw"}[self.platform.name]
        self.backends = importlib.import_module(backend_pkg).build_backends(self.node, self.platform, self.cb, self.cell)
        # Not the planner's: the scene backend is the harness's (reset, ground
        # truth) and camera2 poses are the WatchDog's channel.
        for name in ("scene", "watchdog_poses"):
            try:
                getattr(self.backends, name).shutdown()
            except Exception:
                pass
            setattr(self.backends, name, None)
        self._cam: dict[str, Any] = {}
        self._cam_lock = threading.Lock()
        self._tf = None
        self._cam_subscribed = False
        self._saved = 0

        from cell_planner_msgs.msg import ExecutorFact, WatchdogNotice
        from cell_planner_msgs.srv import PlanCartesian, PlanToJoints, PlanToPose

        self._Fact, self._PlanToPose, self._PlanCartesian, self._PlanToJoints = (
            ExecutorFact, PlanToPose, PlanCartesian, PlanToJoints)
        ns = str(self.platform.get("services", "planning_namespace")).rstrip("/")
        self.plan_pose = self.node.create_client(PlanToPose, f"{ns}/plan_to_pose", callback_group=self.cb)
        self.plan_cart = self.node.create_client(PlanCartesian, f"{ns}/plan_cartesian", callback_group=self.cb)
        self.plan_joints = self.node.create_client(PlanToJoints, f"{ns}/plan_to_joints", callback_group=self.cb)
        self.facts = self.node.create_publisher(ExecutorFact, self.platform.topic("executor_facts"), 50)
        self.consult = bool((self.cell.get("watchdog") or {}).get("consult", False))
        self._notices: list[Any] = []
        self._notice_lock = threading.Lock()
        if self.consult:
            self.node.create_subscription(WatchdogNotice, self.platform.topic("watchdog_notices"), self._on_notice, 20, callback_group=self.cb)
        self.world = str(self.platform.get("frames", "world"))
        self.tip = str(self.platform.get("frames", "tcp"))
        self.arm_joints = list(self.platform.joints("arm"))
        self.home = dict(self.platform.get_optional("scene", "home_joint_positions", default={}) or {})
        self.group = str(self.platform.get("motion", "planning_group"))

        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self._spin = threading.Thread(target=self.executor.spin, name="planner_ros", daemon=True)
        self._spin.start()
        for client, label in ((self.plan_pose, "plan_to_pose"), (self.plan_cart, "plan_cartesian"), (self.plan_joints, "plan_to_joints")):
            if not client.wait_for_service(timeout_sec=20.0):
                raise StepFailed(f"planning service {label} ({client.srv_name}) not available")

    # -- journal, notices --------------------------------------------------------

    def fact(self, kind: str, verb: str = "", part: str = "", feature: str = "", detail: str = "") -> None:
        from datetime import datetime, timezone

        msg = self._Fact()
        msg.wall_time_iso = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"
        msg.sim_time = self.node.get_clock().now().to_msg()
        msg.kind, msg.verb, msg.part, msg.feature, msg.detail = kind, verb, part, feature, detail
        self.facts.publish(msg)
        log.info("fact %s verb=%s part=%s feature=%s %s", kind, verb, part, feature, detail)

    def _on_notice(self, msg: Any) -> None:
        with self._notice_lock:
            self._notices.append(msg)
        if msg.kind == "consultation":
            log.info("WatchDog consultation (%s): %s %s", msg.part or "-", msg.detail, msg.measurements_json)

    def consultations(self, since_index: int = 0) -> list[dict[str, Any]]:
        """WatchDog consultations received since notice index since_index (only
        when watchdog.consult is on): [{index, part, text, measurements,
        wall_time_iso}]. Free-form text from the WatchDog about what it
        observed; what to do with it is the planner's call."""
        import json

        with self._notice_lock:
            notices = list(enumerate(self._notices))[since_index:]
        out = []
        for i, n in notices:
            if n.kind != "consultation":
                continue
            try:
                measurements = json.loads(n.measurements_json) if n.measurements_json else {}
            except ValueError:
                measurements = {}
            out.append({"index": i, "part": n.part, "text": n.detail, "measurements": measurements, "wall_time_iso": n.wall_time_iso})
        return out

    def wait_for_ruling(self, since_index: int, timeout_s: float) -> Any:
        """The first halt notice after since_index and the ruling with its halt_id."""
        deadline = time.monotonic() + timeout_s
        halt_id = None
        while time.monotonic() < deadline:
            with self._notice_lock:
                notices = list(self._notices[since_index:])
            for n in notices:
                if n.kind == "halt" and halt_id is None:
                    halt_id = n.halt_id
                if n.kind == "ruling" and halt_id is not None and n.halt_id == halt_id:
                    return n.ruling
            time.sleep(0.1)
        return None

    def notice_count(self) -> int:
        with self._notice_lock:
            return len(self._notices)

    # -- poses ---------------------------------------------------------------------

    def pose(self, object_id: str, max_age_s: float = 2.0, wait_s: float = 5.0) -> dict[str, Any]:
        from geometry import obs_pose

        deadline = time.monotonic() + wait_s
        while True:
            obs = self.backends.executor_poses.latest(object_id, max_age_s=max_age_s)
            if obs is not None:
                return obs_pose(obs.position, obs.orientation_xyzw, obs.frame_id or self.world)
            if time.monotonic() > deadline:
                raise StepFailed(f"no pose for {object_id!r} on the executor pose channel")
            time.sleep(0.1)

    # -- camera1 (the executor's wrist camera) -------------------------------------

    def _camera_subscribe(self) -> None:
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image

        def keep(key: str):
            def cb(msg: Any) -> None:
                with self._cam_lock:
                    self._cam[key] = msg
            return cb

        for key, typ in (("rgb", Image), ("depth", Image), ("camera_info", CameraInfo)):
            self.node.create_subscription(typ, self.platform.camera_topic("camera1", key), keep(key), qos_profile_sensor_data,
                                          callback_group=self.cb)
        self._ensure_tf()
        self._cam_subscribed = True

    def _ensure_tf(self) -> None:
        if self._tf is None:
            import tf2_ros

            self._tf = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf, self.node)

    def tcp_pose(self, wait_s: float = 2.0) -> dict[str, Any]:
        """The TCP link's current pose in the world frame, from TF (robot_state_publisher
        on joint_states; world -> base_link is static). Read it with the arm at rest:
        TF and the pose channel are not sampled at the same instant."""
        from rclpy.duration import Duration
        from rclpy.time import Time

        from geometry import obs_pose

        self._ensure_tf()
        try:
            tf = self._tf.lookup_transform(self.world, self.tip, Time(), timeout=Duration(seconds=wait_s))
        except Exception as exc:  # noqa: BLE001  (tf2 raises several lookup exception types)
            raise StepFailed(f"no TF {self.world} -> {self.tip}: {exc}")
        t, q = tf.transform.translation, tf.transform.rotation
        return obs_pose([t.x, t.y, t.z], [q.x, q.y, q.z, q.w], self.world)

    def camera1(self, wait_s: float = 3.0) -> dict[str, Any]:
        """The latest camera1 frame: {"rgb": HxWx3 uint8, "depth": HxW float32
        metres (NaN where none), "K": 3x3 intrinsics, "width", "height",
        "frame_id" (the optical frame), "stamp_s", "world_T_camera": pose dict
        of the optical frame in the world (z forward, x right, y down)}."""
        import numpy as np

        if not self._cam_subscribed:
            self._camera_subscribe()
        deadline = time.monotonic() + wait_s
        while True:
            with self._cam_lock:
                rgb, depth, info = self._cam.get("rgb"), self._cam.get("depth"), self._cam.get("camera_info")
            if rgb is not None and info is not None:
                break
            if time.monotonic() > deadline:
                raise StepFailed("no camera1 image (is camera1 enabled and publishing?)")
            time.sleep(0.05)
        enc = rgb.encoding.lower()
        ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(enc)
        if ch is None:
            raise StepFailed(f"camera1 rgb encoding {rgb.encoding!r} not handled")
        img = np.frombuffer(bytes(rgb.data), dtype=np.uint8).reshape(rgb.height, rgb.step // ch if rgb.step else rgb.width, ch)[:, : rgb.width, :3]
        if enc.startswith("bgr"):
            img = img[:, :, ::-1]
        out: dict[str, Any] = {"rgb": np.ascontiguousarray(img), "K": np.array(info.k, dtype=float).reshape(3, 3),
                               "width": int(rgb.width), "height": int(rgb.height), "frame_id": rgb.header.frame_id,
                               "stamp_s": rgb.header.stamp.sec + rgb.header.stamp.nanosec * 1e-9, "depth": None, "world_T_camera": None}
        if depth is not None:
            denc = depth.encoding.lower()
            if denc == "32fc1":
                d = np.frombuffer(bytes(depth.data), dtype=np.float32).reshape(depth.height, depth.width).astype(np.float32)
            elif denc in ("16uc1", "mono16"):
                d = np.frombuffer(bytes(depth.data), dtype=np.uint16).reshape(depth.height, depth.width).astype(np.float32) / 1000.0
                d[d == 0] = np.nan
            else:
                d = None
            out["depth"] = d
        try:
            from rclpy.time import Time

            tf = self._tf.lookup_transform(self.world, rgb.header.frame_id, Time())
            t, r = tf.transform.translation, tf.transform.rotation
            out["world_T_camera"] = {"position": {"x": t.x, "y": t.y, "z": t.z}, "orientation": {"x": r.x, "y": r.y, "z": r.z, "w": r.w},
                                     "frame_id": self.world}
        except Exception as exc:  # noqa: BLE001
            log.warning("camera1 pose not available: %s", exc)
        return out

    def save_frame(self, label: str, frame: dict[str, Any] | None = None) -> str | None:
        """Save a camera1 frame (default: a fresh one) into the episode's
        executor/images/: <n>_<label>.png (RGB) and .npy (depth). The authoring
        agent sees the last saved PNGs of each episode in its feedback."""
        import os
        import re

        import numpy as np
        from robot_backend.utils.png_encode import encode_png

        episode = os.environ.get("CELL_EPISODE_DIR")
        if not episode:
            return None
        frame = frame or self.camera1()
        folder = os.path.join(episode, "executor", "images")
        os.makedirs(folder, exist_ok=True)
        self._saved += 1
        stem = os.path.join(folder, f"{self._saved:03d}_{re.sub(r'[^A-Za-z0-9_.-]+', '_', label)[:60]}")
        rgb = frame["rgb"]
        with open(stem + ".png", "wb") as fh:
            fh.write(encode_png(np.ascontiguousarray(rgb).tobytes(), rgb.shape[1], rgb.shape[0], "rgb8"))
        if frame.get("depth") is not None:
            np.save(stem + ".npy", frame["depth"])
        log.info("saved camera1 frame %s.png", stem)
        return stem + ".png"

    # -- planning and motion ---------------------------------------------------------

    def _stamped(self, pose: dict[str, Any]) -> Any:
        from geometry_msgs.msg import PoseStamped

        ps = PoseStamped()
        ps.header.frame_id = pose.get("frame_id") or self.world
        for k in "xyz":
            setattr(ps.pose.position, k, float(pose["position"][k]))
        for k in "xyzw":
            setattr(ps.pose.orientation, k, float(pose["orientation"][k]))
        return ps

    def _call(self, client: Any, req: Any, timeout_s: float, label: str) -> Any:
        res = _wait(client.call_async(req), timeout_s)
        if res is None:
            raise StepFailed(f"{label}: no response in {timeout_s:.0f}s")
        return res

    def plan_to_pose(self, pose: dict[str, Any], vel: float, acc: float, attempts: int = 3) -> Any:
        req = self._PlanToPose.Request()
        req.planning_group, req.tip_link = self.group, self.tip
        req.target = self._stamped(pose)
        req.position_tolerance_m, req.orientation_tolerance_rad = 0.001, 0.01
        req.max_velocity_scaling, req.max_acceleration_scaling = vel, acc
        res = None
        for _ in range(attempts):  # sampling planners fail now and then on reachable poses
            res = self._call(self.plan_pose, req, 90.0, "plan_to_pose")
            if res.ok:
                return res.trajectory
        raise StepFailed(f"plan_to_pose failed: {res.report.failure_code} {res.report.failure_reason}")

    def plan_straight(self, pose: dict[str, Any], vel: float, acc: float) -> Any:
        req = self._PlanCartesian.Request()
        req.planning_group, req.tip_link = self.group, self.tip
        req.waypoints = [self._stamped(pose)]
        req.eef_step_m, req.min_fraction, req.avoid_collisions = 0.005, 0.95, False
        req.max_velocity_scaling, req.max_acceleration_scaling = vel, acc
        res = self._call(self.plan_cart, req, 30.0, "plan_cartesian")
        if not res.ok:
            raise StepFailed(f"plan_cartesian failed (fraction {res.fraction:.2f}): {res.report.failure_code} {res.report.failure_reason}")
        return res.trajectory

    def plan_home(self, vel: float, acc: float) -> Any:
        if not self.home:
            raise StepFailed("scene.home_joint_positions is empty in the platform config")
        req = self._PlanToJoints.Request()
        req.planning_group = self.group
        req.joint_names = list(self.home)
        req.joint_positions = [float(v) for v in self.home.values()]
        req.max_velocity_scaling, req.max_acceleration_scaling = vel, acc
        res = self._call(self.plan_joints, req, 90.0, "plan_to_joints")
        if not res.ok:
            raise StepFailed(f"plan_to_joints failed: {res.report.failure_code} {res.report.failure_reason}")
        return res.trajectory

    def execute(self, trajectory: Any, label: str) -> None:
        self.fact("arm_goal", detail=label)
        result = self.backends.motion.execute(trajectory)
        if result.state == "canceled":
            raise Halted(f"{label}: trajectory cancelled ({result.detail})")
        if result.state != "succeeded":
            raise StepFailed(f"{label}: motion {result.state} {result.detail}")

    # -- direct motion (no planning, no collision checking) ------------------------

    def move_joints(self, goal: dict[str, float], duration_s: float | None = None,
                    max_speed: float | None = None, label: str = "move_joints") -> None:
        """Drive the arm straight to joint positions (rad, by name; joints not named stay) along a
        quintic, through the motion backend. Nothing checks collisions: keep moves small and known."""
        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        from geometry import DIRECT_MAX_JOINT_SPEED, joint_path

        unknown = [j for j in goal if j not in self.arm_joints]
        if unknown:
            raise StepFailed(f"{label}: not arm joints: {unknown} (arm: {self.arm_joints})")
        snap = self.backends.motion.joint_state(max_age_s=1.0)
        if snap is None:
            raise StepFailed(f"{label}: no fresh joint state")
        start = [snap.position_of(j) for j in self.arm_joints]
        if any(q is None for q in start):
            raise StepFailed(f"{label}: joint state lacks some arm joints")
        target = [float(goal.get(j, q)) for j, q in zip(self.arm_joints, start)]
        try:
            path = joint_path(start, target, duration_s, max_speed or DIRECT_MAX_JOINT_SPEED)
        except ValueError as exc:
            raise StepFailed(f"{label}: {exc}") from exc
        traj = JointTrajectory()
        traj.joint_names = list(self.arm_joints)
        for t, q, v in path:
            pt = JointTrajectoryPoint()
            pt.positions, pt.velocities = q, v
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            traj.points.append(pt)
        self.execute(traj, label)

    def jog(self, delta: dict[str, Any], frame: str = "tool", vel: float = 0.05, acc: float = 0.05,
            label: str = "jog") -> None:
        """Move the TCP by delta ({position: {x,y,z}, orientation: {x,y,z,w}}; missing = zero/identity)
        along a straight line, in the TCP's own axes (frame="tool") or world axes (frame="world").
        Cartesian interpolation only: no collision checking."""
        from geometry import compose, spec_pose

        tcp = self.tcp_pose()
        d = spec_pose(delta)
        if frame == "tool":
            target = compose(tcp, d)
        elif frame == "world":
            a, b = d["orientation"], tcp["orientation"]          # world rotation applied about the TCP
            q = {"x": a["w"] * b["x"] + a["x"] * b["w"] + a["y"] * b["z"] - a["z"] * b["y"],
                 "y": a["w"] * b["y"] - a["x"] * b["z"] + a["y"] * b["w"] + a["z"] * b["x"],
                 "z": a["w"] * b["z"] + a["x"] * b["y"] - a["y"] * b["x"] + a["z"] * b["w"],
                 "w": a["w"] * b["w"] - a["x"] * b["x"] - a["y"] * b["y"] - a["z"] * b["z"]}
            target = {"position": {k: tcp["position"][k] + d["position"][k] for k in "xyz"}, "orientation": q,
                      "frame_id": tcp.get("frame_id", self.world)}
        else:
            raise StepFailed(f"{label}: frame must be 'tool' or 'world', not {frame!r}")
        self.execute(self.plan_straight(target, vel, acc), label)

    # -- gripper ---------------------------------------------------------------------

    def open_gripper(self, part: str = "") -> None:
        res = self.backends.gripper.open()
        self.fact("grip_open", part=part, detail=f"ok={res.ok} {res.stop_reason} {res.detail}".strip())
        if not res.ok:
            raise StepFailed(f"gripper open failed: {res.stop_reason} {res.detail}")

    def grasp(self, part: str, target_width_m: float, force: float) -> Any:
        res = self.backends.gripper.set_width(target_width_m, force=force)
        self.fact("grip_close", part=part, detail=f"stop={res.stop_reason} width={res.final_width_m} effort={res.final_effort}")
        return res

    def shutdown(self) -> None:
        try:
            self.executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
