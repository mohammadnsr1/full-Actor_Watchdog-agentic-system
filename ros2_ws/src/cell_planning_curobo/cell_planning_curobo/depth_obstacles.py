"""EXPERIMENTAL: depth image -> voxel obstacle layer for the cuRobo world.

Off by default (config/cell.yaml planner.depth_obstacles: false). When on,
the layer subscribes to the scene camera's depth image and camera info,
back-projects each depth sample to the world frame through TF, bins the
points into a voxel grid over a fixed box, and hands cuRobo a VoxelGrid
obstacle alongside the explicit collision objects. Unregistered objects
(a hand, a dropped tool) then block planning without anyone naming them.

Why experimental:
    - MoveIt's point-cloud octomap path failed in this cell before (ghost
      points during motion; see sim_moveit_bringup.launch.py's
      output_stamp_mode notes), and the same stamping issues apply here.
    - cuRobo's nvblox examples have known issues at v0.7.8; this layer uses
      cuRobo's plain VoxelGrid obstacle instead of nvblox, which is simpler
      but has no temporal filtering.
    - The robot itself appears in the depth image. Voxels inside the arm's
      own collision spheres are removed before use, which is crude.

Every numeric parameter is a TODO(mohammad) in config/cell.yaml.
"""

from __future__ import annotations

import threading
from typing import Any

import numpy as np

from sensor_msgs.msg import CameraInfo, Image


class DepthObstacleLayer:
    def __init__(self, node: Any, settings: dict[str, Any], tensor_args: Any, callback_group: Any = None) -> None:
        self.node = node
        self.settings = dict(settings or {})
        self.tensor_args = tensor_args
        self._lock = threading.Lock()
        self._depth: np.ndarray | None = None
        self._info: CameraInfo | None = None
        self._frame = ""
        self.voxel_size = float(self.settings.get("voxel_size_m", 0.0))
        self.bounds_min = np.array(self.settings.get("bounds_min", [0, 0, 0]), dtype=float)
        self.bounds_max = np.array(self.settings.get("bounds_max", [0, 0, 0]), dtype=float)
        self.max_depth = float(self.settings.get("max_depth_m", 0.0))
        self.world_frame = str(self.settings["world_frame"])  # filled by the node from the platform frames
        if self.voxel_size <= 0 or self.max_depth <= 0 or np.all(self.bounds_max <= self.bounds_min):
            raise RuntimeError(
                "depth obstacles enabled but planner.curobo.depth voxel_size_m, max_depth_m, "
                "bounds_min/bounds_max are not set (TODO(mohammad))"
            )
        from tf2_ros import Buffer, TransformListener  # type: ignore

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        kw = {"callback_group": callback_group} if callback_group is not None else {}
        node.create_subscription(Image, str(self.settings.get("depth_topic")), self._on_depth, 1, **kw)
        node.create_subscription(CameraInfo, str(self.settings.get("camera_info_topic")), self._on_info, 1, **kw)

    def _on_info(self, msg: CameraInfo) -> None:
        with self._lock:
            self._info = msg

    def _on_depth(self, msg: Image) -> None:
        if msg.encoding == "32FC1":
            arr = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
        elif msg.encoding == "16UC1":
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
        else:
            return
        with self._lock:
            self._depth = arr
            self._frame = msg.header.frame_id

    def _points_world(self) -> np.ndarray | None:
        with self._lock:
            depth, info, frame = self._depth, self._info, self._frame
        if depth is None or info is None or not frame:
            return None
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        stride = int(self.settings.get("pixel_stride", 4)) or 4
        v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
        z = depth[::stride, ::stride]
        ok = np.isfinite(z) & (z > 0) & (z < self.max_depth)
        x = (u[ok] - cx) * z[ok] / fx
        y = (v[ok] - cy) * z[ok] / fy
        pts = np.stack([x, y, z[ok]], axis=1)
        try:
            tf = self._tf_buffer.lookup_transform(self.world_frame, frame, _zero_time())
        except Exception:  # noqa: BLE001  no transform yet
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        rot = _quat_to_mat(q.w, q.x, q.y, q.z)
        return pts @ rot.T + np.array([t.x, t.y, t.z])

    def augment(self, world_cfg: Any) -> Any:
        """Return world_cfg with a VoxelGrid obstacle added (if data exists)."""
        pts = self._points_world()
        if pts is None:
            return world_cfg
        inside = np.all((pts >= self.bounds_min) & (pts <= self.bounds_max), axis=1)
        pts = pts[inside]
        if pts.size == 0:
            return world_cfg
        from curobo.geom.types import VoxelGrid  # type: ignore

        dims = (self.bounds_max - self.bounds_min).tolist()
        center = ((self.bounds_max + self.bounds_min) * 0.5).tolist()
        grid = VoxelGrid(
            name="depth_voxels",
            dims=dims,
            pose=[*center, 1.0, 0.0, 0.0, 0.0],
            voxel_size=self.voxel_size,
        )
        # Occupied voxels: mark feature tensor as negative distance inside, per
        # cuRobo's VoxelGrid convention (ESDF-style feature values).
        # TODO(mohammad): validate the feature convention against v0.7.8's
        # WorldVoxelCollision before trusting plans that depend on this layer.
        xyz = grid.xyzr_tensor(tensor_args=self.tensor_args)[:, :3].cpu().numpy()
        idx = np.floor((pts - self.bounds_min) / self.voxel_size).astype(int)
        shape = np.ceil((self.bounds_max - self.bounds_min) / self.voxel_size).astype(int)
        flat = np.ravel_multi_index(np.clip(idx, 0, shape - 1).T, shape)
        occupied = np.zeros(int(np.prod(shape)), dtype=bool)
        occupied[np.unique(flat)] = True
        feature = np.where(occupied[: xyz.shape[0]], -self.voxel_size, self.voxel_size).astype(np.float32)
        grid.feature_tensor = self.tensor_args.to_device(feature)
        world_cfg.voxel = list(getattr(world_cfg, "voxel", []) or []) + [grid]
        return world_cfg


def _zero_time() -> Any:
    from rclpy.time import Time  # type: ignore

    return Time()


def _quat_to_mat(w: float, x: float, y: float, z: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
