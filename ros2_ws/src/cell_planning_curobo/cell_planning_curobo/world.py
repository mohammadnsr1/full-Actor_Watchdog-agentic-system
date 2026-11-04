"""The explicit collision-object world for the cuRobo backend.

Objects arrive as moveit_msgs/CollisionObject (the same calls the MoveIt
backend takes) and are kept as plain records here; the backend turns the
set into a cuRobo WorldConfig when it changes. Boxes become cuboids;
meshes keep their vertices and triangles. Other primitives (sphere,
cylinder) map to cuRobo's own types. Frames are passed through: every
object must be expressed in the planning frame the backend was configured
with, or carry a pose in that frame; the service does no TF here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


@dataclass
class WorldObject:
    object_id: str
    frame_id: str
    kind: str                      # cuboid | sphere | cylinder | mesh
    pose: list[float]              # x y z qw qx qy qz
    dims: list[float] = field(default_factory=list)          # cuboid: x y z; sphere: r; cylinder: r h
    vertices: list[list[float]] = field(default_factory=list)  # mesh
    faces: list[list[int]] = field(default_factory=list)       # mesh


def _pose_list(pose: Pose) -> list[float]:
    return [
        float(pose.position.x), float(pose.position.y), float(pose.position.z),
        float(pose.orientation.w), float(pose.orientation.x),
        float(pose.orientation.y), float(pose.orientation.z),
    ]


def from_collision_object(obj: CollisionObject, planning_frame: str) -> list[WorldObject]:
    """Split one CollisionObject into world records (one per primitive/mesh)."""
    if obj.header.frame_id and obj.header.frame_id != planning_frame:
        raise ValueError(
            f"collision object {obj.id!r} is in frame {obj.header.frame_id!r}; "
            f"the cuRobo backend plans in {planning_frame!r} and does no TF"
        )
    base = _pose_list(obj.pose) if _has_pose(obj.pose) else [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    out: list[WorldObject] = []
    for index, (prim, pose) in enumerate(zip(obj.primitives, obj.primitive_poses)):
        pose_list = _compose(base, _pose_list(pose))
        name = obj.id if len(obj.primitives) + len(obj.meshes) == 1 else f"{obj.id}#{index}"
        dims = [float(d) for d in prim.dimensions]
        if prim.type == SolidPrimitive.BOX:
            out.append(WorldObject(name, planning_frame, "cuboid", pose_list, dims))
        elif prim.type == SolidPrimitive.SPHERE:
            out.append(WorldObject(name, planning_frame, "sphere", pose_list, dims[:1]))
        elif prim.type == SolidPrimitive.CYLINDER:
            # SolidPrimitive: [height, radius]; cuRobo Cylinder: radius, height
            out.append(WorldObject(name, planning_frame, "cylinder", pose_list, [dims[1], dims[0]]))
        else:
            raise ValueError(f"collision object {obj.id!r}: unsupported primitive type {prim.type}")
    for index, (mesh, pose) in enumerate(zip(obj.meshes, obj.mesh_poses)):
        pose_list = _compose(base, _pose_list(pose))
        name = obj.id if len(obj.primitives) + len(obj.meshes) == 1 else f"{obj.id}#m{index}"
        vertices = [[float(v.x), float(v.y), float(v.z)] for v in mesh.vertices]
        faces = [[int(i) for i in t.vertex_indices] for t in mesh.triangles]
        out.append(WorldObject(name, planning_frame, "mesh", pose_list, [], vertices, faces))
    if not out:
        raise ValueError(f"collision object {obj.id!r} carries no supported geometry")
    return out


def _has_pose(pose: Pose) -> bool:
    return any(
        abs(v) > 0.0
        for v in (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z,
        )
    ) or pose.orientation.w not in (0.0, 1.0)


def _compose(a: list[float], b: list[float]) -> list[float]:
    """Pose a (parent) composed with pose b (child), both as x y z qw qx qy qz."""
    ax, ay, az, aw, aqx, aqy, aqz = a
    bx, by, bz, bw, bqx, bqy, bqz = b
    # rotate b's position by a's quaternion
    rx, ry, rz = _rotate((aw, aqx, aqy, aqz), (bx, by, bz))
    qw, qx, qy, qz = _qmul((aw, aqx, aqy, aqz), (bw, bqx, bqy, bqz))
    return [ax + rx, ay + ry, az + rz, qw, qx, qy, qz]


def _qmul(p, q):
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return (
        pw * qw - px * qx - py * qy - pz * qz,
        pw * qx + px * qw + py * qz - pz * qy,
        pw * qy - px * qz + py * qw + pz * qx,
        pw * qz + px * qy - py * qx + pz * qw,
    )


def _rotate(q, v):
    qw, qx, qy, qz = q
    vx, vy, vz = v
    # v' = q * (0, v) * q^-1
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


class World:
    """Mutable set of WorldObject keyed by object id (prefix match on removal)."""

    def __init__(self, planning_frame: str) -> None:
        self.planning_frame = planning_frame
        self._objects: dict[str, WorldObject] = {}
        self.version = 0

    def add(self, obj: CollisionObject) -> list[str]:
        records = from_collision_object(obj, self.planning_frame)
        self.remove(obj.id)
        for rec in records:
            self._objects[rec.object_id] = rec
        self.version += 1
        return [r.object_id for r in records]

    def remove(self, object_id: str) -> int:
        keys = [k for k in self._objects if k == object_id or k.startswith(object_id + "#")]
        for k in keys:
            del self._objects[k]
        if keys:
            self.version += 1
        return len(keys)

    def objects(self) -> list[WorldObject]:
        return list(self._objects.values())

    def to_curobo_world_config(self) -> Any:
        """Build a curobo WorldConfig (imports cuRobo lazily)."""
        from curobo.geom.types import Cuboid, Cylinder, Mesh, Sphere, WorldConfig  # type: ignore

        cuboids, spheres, cylinders, meshes = [], [], [], []
        for o in self._objects.values():
            if o.kind == "cuboid":
                cuboids.append(Cuboid(name=o.object_id, pose=list(o.pose), dims=list(o.dims)))
            elif o.kind == "sphere":
                spheres.append(Sphere(name=o.object_id, pose=list(o.pose), radius=float(o.dims[0])))
            elif o.kind == "cylinder":
                cylinders.append(
                    Cylinder(name=o.object_id, pose=list(o.pose), radius=float(o.dims[0]), height=float(o.dims[1]))
                )
            elif o.kind == "mesh":
                meshes.append(
                    Mesh(name=o.object_id, pose=list(o.pose), vertices=[list(v) for v in o.vertices],
                         faces=[list(f) for f in o.faces])
                )
        return WorldConfig(cuboid=cuboids, sphere=spheres, cylinder=cylinders, mesh=meshes)
