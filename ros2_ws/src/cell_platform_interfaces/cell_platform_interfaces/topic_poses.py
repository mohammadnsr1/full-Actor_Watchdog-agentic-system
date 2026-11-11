"""Detection3DArray-based PoseSource, shared by sim and hardware.

One instance per camera role: the executor's (camera1 topic) and WatchDog's
(camera2 topic). The two never share an instance, a topic, or a frame
prefix. On sim in ground_truth mode both topics are fed by
cell_backend_sim's ground_truth_pose_publisher; on hardware by the two
FoundationPose pipelines (not available yet).

Message convention (vision_msgs/Detection3DArray, identical on Humble and
Jazzy):
    detections[i].id                       registry object id
    detections[i].results[0].hypothesis.class_id   same id (kept for consumers that read only results)
    detections[i].results[0].hypothesis.score      confidence in [0, 1]
    detections[i].results[0].pose.pose             pose in header.frame_id
    header.frame_id                        the frame the poses are expressed in

The pipeline name (ground_truth, foundationpose, ...) is not in the message;
it comes from the platform config `poses.source` and is stamped on every
observation so a verdict can say where its evidence came from.
"""

from __future__ import annotations

import threading
from typing import Any

from cell_platform_interfaces.interfaces import PoseSource
from cell_platform_interfaces.timebase import wall_ns, wall_time_iso
from cell_platform_interfaces.types import PoseObservation


class TopicPoseSource(PoseSource):
    def __init__(
        self,
        node: Any,
        topic: str,
        camera: str,
        source: str,
        callback_group: Any = None,
        qos_depth: int = 10,
    ) -> None:
        from vision_msgs.msg import Detection3DArray

        self.node = node
        self.topic = topic
        self._camera = camera
        self._source = source
        self._lock = threading.Lock()
        self._latest: dict[str, PoseObservation] = {}
        self._recv_ns: dict[str, int] = {}
        self._message_count = 0
        self._sub = node.create_subscription(
            Detection3DArray, topic, self._on_detections, qos_depth, callback_group=callback_group
        )

    def _on_detections(self, msg: Any) -> None:
        now = wall_ns()
        stamp_iso = wall_time_iso(now)
        frame = str(msg.header.frame_id)
        header_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        fresh: dict[str, PoseObservation] = {}
        for det in msg.detections:
            object_id = str(det.id)
            if not det.results:
                continue
            best = max(det.results, key=lambda r: float(r.hypothesis.score))
            if not object_id:
                object_id = str(best.hypothesis.class_id)
            if not object_id:
                continue
            pose = best.pose.pose
            fresh[object_id] = PoseObservation(
                object_id=object_id,
                frame_id=frame,
                position=(float(pose.position.x), float(pose.position.y), float(pose.position.z)),
                orientation_xyzw=(
                    float(pose.orientation.x), float(pose.orientation.y),
                    float(pose.orientation.z), float(pose.orientation.w),
                ),
                confidence=float(best.hypothesis.score),
                source=self._source,
                camera=self._camera,
                header_stamp_ns=header_ns,
                wall_time_iso=stamp_iso,
            )
        with self._lock:
            self._message_count += 1
            for object_id, obs in fresh.items():
                self._latest[object_id] = obs
                self._recv_ns[object_id] = now

    def latest(self, object_id: str, max_age_s: float | None = None) -> PoseObservation | None:
        with self._lock:
            obs = self._latest.get(object_id)
            recv = self._recv_ns.get(object_id, 0)
        if obs is None:
            return None
        if max_age_s is not None and (wall_ns() - recv) / 1e9 > max_age_s:
            return None
        return obs

    def latest_all(self, max_age_s: float | None = None) -> dict[str, PoseObservation]:
        now = wall_ns()
        with self._lock:
            items = list(self._latest.items())
            recv = dict(self._recv_ns)
        if max_age_s is None:
            return dict(items)
        return {k: v for k, v in items if (now - recv.get(k, 0)) / 1e9 <= max_age_s}

    def object_ids(self) -> list[str]:
        with self._lock:
            return sorted(self._latest)

    @property
    def message_count(self) -> int:
        with self._lock:
            return self._message_count

    @property
    def camera(self) -> str:
        return self._camera

    @property
    def source(self) -> str:
        return self._source

    def shutdown(self) -> None:
        try:
            self.node.destroy_subscription(self._sub)
        except Exception:
            pass
