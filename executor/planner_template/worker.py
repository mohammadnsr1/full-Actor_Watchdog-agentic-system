"""ROS side of the planner template: one node, the platform backends, the
planning service, the fact journal, WatchDog notices and the gate.

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
    def __init__(self, platform_name: str, repo_root: str) -> None:
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
        self.node = Node("planner_template", parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)])
        self.cb = ReentrantCallbackGroup()
        backend_pkg = {"sim": "cell_backend_sim", "hardware": "cell_backend_hw"}[self.platform.name]
        self.backends = importlib.import_module(backend_pkg).build_backends(self.node, self.platform, self.cb, self.cell)

        from cell_planner_msgs.msg import ExecutorFact, WatchdogNotice
        from cell_planner_msgs.srv import PlanCartesian, PlanToJoints, PlanToPose
        from cell_sync_interfaces.srv import RequestGo

        self._Fact, self._PlanToPose, self._PlanCartesian, self._PlanToJoints, self._RequestGo = (
            ExecutorFact, PlanToPose, PlanCartesian, PlanToJoints, RequestGo)
        ns = str(self.platform.get("services", "planning_namespace")).rstrip("/")
        self.plan_pose = self.node.create_client(PlanToPose, f"{ns}/plan_to_pose", callback_group=self.cb)
        self.plan_cart = self.node.create_client(PlanCartesian, f"{ns}/plan_cartesian", callback_group=self.cb)
        self.plan_joints = self.node.create_client(PlanToJoints, f"{ns}/plan_to_joints", callback_group=self.cb)
        self.request_go = self.node.create_client(RequestGo, str(self.platform.get("services", "request_go")), callback_group=self.cb)
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
        # The SRDF planning group; the platform config has no key for it yet.
        self.group = "ur_manipulator"

        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self._spin = threading.Thread(target=self.executor.spin, name="planner_ros", daemon=True)
        self._spin.start()
        for client, label in ((self.plan_pose, "plan_to_pose"), (self.plan_cart, "plan_cartesian"), (self.plan_joints, "plan_to_joints")):
            if not client.wait_for_service(timeout_sec=20.0):
                raise StepFailed(f"planning service {label} ({client.srv_name}) not available")

    # -- journal, gate, notices -------------------------------------------------

    def fact(self, kind: str, verb: str = "", part: str = "", feature: str = "", detail: str = "") -> None:
        from datetime import datetime, timezone

        msg = self._Fact()
        msg.wall_time_iso = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"
        msg.sim_time = self.node.get_clock().now().to_msg()
        msg.kind, msg.verb, msg.part, msg.feature, msg.detail = kind, verb, part, feature, detail
        self.facts.publish(msg)
        log.info("fact %s verb=%s part=%s feature=%s %s", kind, verb, part, feature, detail)

    def gate(self, timeout_s: float = 60.0) -> str:
        if not self.request_go.wait_for_service(timeout_sec=5.0):
            log.warning("request_go not available; running ungated")
            return "no_gate"
        req = self._RequestGo.Request()
        req.step_id, req.phase, req.timeout_sec = "episode", "", float(timeout_s)
        res = _wait(self.request_go.call_async(req), timeout_s + 10.0)
        reason = "no_response" if res is None else str(res.reason)
        log.info("gate: %s", reason)
        return reason

    def _on_notice(self, msg: Any) -> None:
        with self._notice_lock:
            self._notices.append(msg)

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
        from task import obs_pose

        deadline = time.monotonic() + wait_s
        while True:
            obs = self.backends.executor_poses.latest(object_id, max_age_s=max_age_s)
            if obs is not None:
                return obs_pose(obs.position, obs.orientation_xyzw, obs.frame_id or self.world)
            if time.monotonic() > deadline:
                raise StepFailed(f"no pose for {object_id!r} on the executor pose channel")
            time.sleep(0.1)

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
