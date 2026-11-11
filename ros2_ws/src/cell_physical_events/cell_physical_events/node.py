"""`physical_events_node`: publish the physical event vocabulary on the
platform's `topics.physical_events` (cell_planner_msgs/PhysicalEvent).

Inputs, each used only when the platform's `signals` block says it exists:
    joint_states            motion_started/stopped (arm joints), effort_rising
                            and the effort contact proxy (gripper effort joint)
    WatchDog pose topic     part_lifted/released/settled, approach/engage,
    (poses.watchdog_topic)  workspace_object_appeared; source = ground_truth
                            or camera2_pose per poses.source
    robot TF (frames.tcp)   the TCP position for hold/release
    pad contacts            TODO(mohammad), Milestone 6: contact_report source;
                            when it exists the effort proxy is not used
Thresholds: config/cell.yaml `physical_events`. Parts and feature frames:
config/parts/watchdog_parts.yaml. Both files are the node's only inputs
besides the platform config; nothing about the executor or the plan.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.node import Node

from cell_platform_interfaces.platform_config import load_cell_yaml, load_platform_config
from cell_platform_interfaces.signals import status_of
from cell_platform_interfaces.timebase import wall_ns, wall_time_iso
from cell_platform_interfaces.topic_poses import TopicPoseSource
from cell_physical_events.detectors import EffortDetector, MotionDetector, PartDetector, PoseSample


def _load_parts(path: Path) -> tuple[dict[str, dict[str, str]], set[str]]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    parts: dict[str, dict[str, str]] = {}
    for pid, entry in (data.get("parts") or {}).items():
        if isinstance(entry, dict) and bool(entry.get("enabled", True)):
            parts[str(pid)] = {str(k): str(v) for k, v in (entry.get("features") or {}).items()}
    frames = {str(fid) for fid, e in (data.get("frames") or {}).items() if isinstance(e, dict) and bool(e.get("enabled", True))}
    return parts, frames


class PhysicalEventsNode(Node):
    def __init__(self) -> None:
        super().__init__("cell_physical_events")
        from cell_planner_msgs.msg import PhysicalEvent
        from sensor_msgs.msg import JointState

        self.declare_parameter("parts_file", "")
        self.platform = load_platform_config()
        cell, cell_path = load_cell_yaml(self.platform.repo_root)
        self.cfg = dict(cell.get("physical_events", {}) or {})
        parts_param = str(self.get_parameter("parts_file").value).strip()
        parts_path = Path(parts_param).expanduser() if parts_param else self.platform.repo_root / "config" / "parts" / "watchdog_parts.yaml"
        parts, frames = _load_parts(parts_path)
        if not parts:
            raise RuntimeError(f"no enabled parts in {parts_path}")
        self._PhysicalEvent = PhysicalEvent
        self.pub = self.create_publisher(PhysicalEvent, self.platform.topic("physical_events"), 50)
        self.arm_joints = self.platform.joints("arm")
        effort_joint = str(self.platform.get_optional("joints", "gripper_effort", default="") or "")
        effort_available = status_of(self.platform, "finger_effort") in ("available", "proxy")
        contacts_available = status_of(self.platform, "pad_contacts") == "available"
        pose_source_name = str(self.platform.get("poses", "source"))
        self.pose_source_label = "ground_truth" if pose_source_name == "ground_truth" else "camera2_pose"

        self.motion = MotionDetector(self.cfg)
        self.effort = EffortDetector(self.cfg, effort_joint if effort_available else "", contact_proxy_allowed=not contacts_available)
        self.parts = PartDetector(self.cfg, parts, frames)
        for reason in [self.motion.disabled_reason()] + self.effort.disabled_reasons() + self.parts.disabled_reasons():
            if reason:
                self.get_logger().warning(f"detector disabled: {reason}")
        if contacts_available:
            self.get_logger().warning("pad_contacts is declared available but the contact_report subscriber is TODO(mohammad) (Milestone 6)")

        self.poses = TopicPoseSource(self, self.platform.get("poses", "watchdog_topic"), camera="camera2", source=self.pose_source_label)
        self._js_sub = self.create_subscription(JointState, self.platform.topic("joint_states"), self._on_joint_state, 50)
        self._sim_time_s = 0.0
        if self.platform.has("topics", "clock"):
            from rosgraph_msgs.msg import Clock

            self._clock_sub = self.create_subscription(Clock, self.platform.topic("clock"), self._on_clock, 10)
        import tf2_ros

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.world = self.platform.frame("world")
        self.tcp = self.platform.frame("tcp")
        rate = float(self.cfg.get("publish_rate_hz", 0.0) or 0.0)
        if rate <= 0.0:
            raise RuntimeError("physical_events.publish_rate_hz must be > 0")
        from rclpy.clock import Clock as RclClock, ClockType

        self.timer = self.create_timer(1.0 / rate, self._tick, clock=RclClock(clock_type=ClockType.SYSTEM_TIME))
        self.get_logger().info(
            f"physical events on {self.pub.topic_name}: {len(parts)} parts, {len(frames)} frames, pose source {self.pose_source_label}, "
            f"effort joint {effort_joint or 'none'}, platform {self.platform.name}"
        )

    def _on_clock(self, msg: Any) -> None:
        self._sim_time_s = float(msg.clock.sec) + float(msg.clock.nanosec) / 1e9

    def _on_joint_state(self, msg: Any) -> None:
        t = wall_ns()
        names = list(msg.name)
        try:
            idx = [names.index(j) for j in self.arm_joints]
        except ValueError:
            idx = []
        events = []
        if idx:
            pos = [float(msg.position[i]) for i in idx if i < len(msg.position)]
            vel = [float(msg.velocity[i]) for i in idx if i < len(msg.velocity)] if len(msg.velocity) == len(names) else []
            if len(pos) == len(idx):
                events += self.motion.feed(pos, vel, t)
        if self.effort.joint and self.effort.joint in names:
            j = names.index(self.effort.joint)
            if j < len(msg.effort):
                events += self.effort.feed(float(msg.effort[j]), t)
        for ev in events:
            self._publish(ev)

    def _tcp_position(self) -> tuple[float, float, float] | None:
        from rclpy.time import Time

        try:
            tf = self.tf_buffer.lookup_transform(self.world, self.tcp, Time())
        except Exception:
            return None
        tr = tf.transform.translation
        return (float(tr.x), float(tr.y), float(tr.z))

    def _tick(self) -> None:
        t = wall_ns()
        latest = self.poses.latest_all(max_age_s=1.0)
        poses = {
            oid: PoseSample(oid, obs.position, obs.confidence, obs.source, obs.header_stamp_ns)
            for oid, obs in latest.items()
        }
        for ev in self.parts.feed(poses, self._tcp_position(), t):
            self._publish(ev)

    def _publish(self, ev: Any) -> None:
        msg = self._PhysicalEvent()
        msg.wall_time_iso = wall_time_iso(ev.t_ns)
        msg.sim_time = float(self._sim_time_s)
        msg.kind = ev.kind
        msg.scene_ids = list(ev.scene_ids)
        msg.source = ev.source
        msg.confidence = float(ev.confidence)
        msg.value = float(ev.value)
        msg.detail = ev.detail
        self.pub.publish(msg)


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = PhysicalEventsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f"cell_physical_events: {exc}", file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
