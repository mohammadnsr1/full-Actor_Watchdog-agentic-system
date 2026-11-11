"""Publish simulator-truth object poses on the two camera pose topics.

Sim-only node for `poses.source: ground_truth`. It reads each registry
object's `tf_frame` / `lookup_frame` (the frames Isaac publishes into TF),
looks the transforms up at the latest common time, and publishes one
vision_msgs/Detection3DArray per camera topic:

    poses.executor_topic  (camera1, the executor's pose-query channel)
    poses.watchdog_topic  (camera2, WatchDog's observation channel)

Both carry the same truth today, so the sim and hardware topic contracts
match from day one. Visibility per camera is NOT computed (it is not cheap
without a render pass): every enabled registry object is published on both
topics with score 1.0. A verdict that rests on these poses is ground truth,
and the record says so through `poses.source`.

Parameters
    registry_path   model_registry.yaml (default: robot_backend's installed copy)
    rate_hz         publish rate (default: platforms/sim.yaml poses.publish_rate_hz)

Why TF -> topic rather than letting consumers read TF: WatchDog's pose
interface must look the same on hardware, where there is no object TF.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.node import Node

from cell_platform_interfaces.platform_config import load_platform_config


def _registry_object_frames(path: Path) -> dict[str, dict[str, str]]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    objects = data.get("objects", {}) if isinstance(data, dict) else {}
    frames: dict[str, dict[str, str]] = {}
    for object_id, entry in (objects or {}).items():
        if not isinstance(entry, dict) or not bool(entry.get("enabled", True)):
            continue
        tf_frame = entry.get("tf_frame")
        lookup = entry.get("lookup_frame")
        if isinstance(tf_frame, str) and tf_frame and isinstance(lookup, str) and lookup:
            frames[str(object_id)] = {"tf_frame": tf_frame, "lookup_frame": lookup}
    return frames


def _default_registry_path() -> Path:
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("robot_backend")) / "config" / "model_registry.yaml"


class GroundTruthPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("ground_truth_pose_publisher")
        import tf2_ros
        from vision_msgs.msg import Detection3DArray

        self.declare_parameter("registry_path", "")
        self.declare_parameter("rate_hz", 0.0)
        self.platform = load_platform_config()
        if self.platform.name != "sim":
            raise RuntimeError("ground_truth_pose_publisher runs on the sim platform only")
        if str(self.platform.get("poses", "source")) != "ground_truth":
            raise RuntimeError("poses.source is not ground_truth; this node has nothing to publish")
        reg_param = str(self.get_parameter("registry_path").value).strip()
        self.registry_path = Path(reg_param).expanduser() if reg_param else _default_registry_path()
        self.frames = _registry_object_frames(self.registry_path)
        if not self.frames:
            raise RuntimeError(f"no enabled objects with tf_frame/lookup_frame in {self.registry_path}")
        rate = float(self.get_parameter("rate_hz").value) or float(self.platform.get("poses", "publish_rate_hz"))
        self.world_frame = self.platform.frame("world")
        self._Detection3DArray = Detection3DArray
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub_executor = self.create_publisher(Detection3DArray, self.platform.get("poses", "executor_topic"), 10)
        self.pub_watchdog = self.create_publisher(Detection3DArray, self.platform.get("poses", "watchdog_topic"), 10)
        # Wall-clock timer on purpose: the bringup passes use_sim_time, and a
        # sim-time timer publishes at 20 Hz of SIM time, which Mohammad measured
        # at ~9 Hz wall on 2026-11-10 (Isaac below real time). WatchDog samples
        # on wall_ns, so the stream is paced on the wall clock; header stamps
        # stay on the node clock like every other message in the cell.
        from rclpy.clock import Clock, ClockType

        self._wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.timer = self.create_timer(1.0 / rate, self._tick, clock=self._wall_clock)
        self._missing_logged: set[str] = set()
        self.get_logger().info(
            f"ground-truth poses for {len(self.frames)} objects at {rate:.1f} Hz on "
            f"{self.pub_executor.topic_name} and {self.pub_watchdog.topic_name} (frame {self.world_frame})"
        )

    def _tick(self) -> None:
        from rclpy.time import Time
        from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose

        msg = self._Detection3DArray()
        msg.header.frame_id = self.world_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for object_id, frame in self.frames.items():
            try:
                tf = self.tf_buffer.lookup_transform(frame["lookup_frame"], frame["tf_frame"], Time())
            except Exception as exc:  # noqa: BLE001  a missing object is reported once, not every tick
                if object_id not in self._missing_logged:
                    self._missing_logged.add(object_id)
                    self.get_logger().warning(f"no TF for {object_id} ({frame['lookup_frame']} -> {frame['tf_frame']}): {exc}")
                continue
            self._missing_logged.discard(object_id)
            det = Detection3D()
            det.header.frame_id = frame["lookup_frame"]
            det.header.stamp = tf.header.stamp
            det.id = object_id
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = object_id
            hyp.hypothesis.score = 1.0
            hyp.pose.pose.position.x = tf.transform.translation.x
            hyp.pose.pose.position.y = tf.transform.translation.y
            hyp.pose.pose.position.z = tf.transform.translation.z
            hyp.pose.pose.orientation = tf.transform.rotation
            det.results.append(hyp)
            msg.detections.append(det)
        self.pub_executor.publish(msg)
        self.pub_watchdog.publish(msg)


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = GroundTruthPosePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f"ground_truth_pose_publisher: {exc}", file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
