"""Pure-Python tests for the cuRobo world model and the RG2 sphere loader."""

from __future__ import annotations

import math
from pathlib import Path

import pytest
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from geometry_msgs.msg import Point

from cell_planning_curobo.robot_config import RG2_LINKS, load_rg2_spheres, merge_spheres
from cell_planning_curobo.world import World, _compose, from_collision_object


def _box(object_id: str, frame: str = "base_link", x: float = 0.0) -> CollisionObject:
    obj = CollisionObject()
    obj.id = object_id
    obj.header.frame_id = frame
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [0.1, 0.2, 0.3]
    pose = Pose()
    pose.position.x = x
    pose.orientation.w = 1.0
    obj.primitives = [prim]
    obj.primitive_poses = [pose]
    return obj


def test_box_becomes_cuboid():
    recs = from_collision_object(_box("a", x=0.5), "base_link")
    assert len(recs) == 1
    assert recs[0].kind == "cuboid" and recs[0].dims == [0.1, 0.2, 0.3]
    assert recs[0].pose[:3] == [0.5, 0.0, 0.0]


def test_wrong_frame_rejected():
    with pytest.raises(ValueError):
        from_collision_object(_box("a", frame="world"), "base_link")


def test_mesh_kept():
    obj = CollisionObject()
    obj.id = "m"
    obj.header.frame_id = "base_link"
    mesh = Mesh()
    mesh.vertices = [Point(x=0.0, y=0.0, z=0.0), Point(x=1.0, y=0.0, z=0.0), Point(x=0.0, y=1.0, z=0.0)]
    tri = MeshTriangle()
    tri.vertex_indices = [0, 1, 2]
    mesh.triangles = [tri]
    pose = Pose()
    pose.orientation.w = 1.0
    obj.meshes = [mesh]
    obj.mesh_poses = [pose]
    recs = from_collision_object(obj, "base_link")
    assert recs[0].kind == "mesh" and len(recs[0].vertices) == 3 and recs[0].faces == [[0, 1, 2]]


def test_world_add_remove_versions():
    world = World("base_link")
    assert world.add(_box("a")) == ["a"]
    v = world.version
    assert world.remove("a") == 1 and world.version == v + 1
    assert world.remove("a") == 0 and world.version == v + 1


def test_compose_rotation():
    # parent rotated 90 deg about z, child at +x -> world +y
    s = math.sqrt(0.5)
    out = _compose([0, 0, 0, s, 0, 0, s], [1, 0, 0, 1, 0, 0, 0])
    assert abs(out[0]) < 1e-9 and abs(out[1] - 1.0) < 1e-9


def test_rg2_spheres_template_is_refused(tmp_path: Path):
    repo = Path(__file__).resolve().parents[4]
    template = repo / "config" / "robot" / "rg2_collision_spheres.yaml"
    if not template.is_file():
        pytest.skip("template not found")
    with pytest.raises(RuntimeError):
        load_rg2_spheres(template)


def test_rg2_spheres_filled_ok(tmp_path: Path):
    body = "spheres:\n" + "".join(f"  {link}:\n    - {{center: [0, 0, 0], radius: 0.01}}\n" for link in RG2_LINKS)
    p = tmp_path / "rg2.yaml"
    p.write_text(body, encoding="utf-8")
    spheres = load_rg2_spheres(p)
    assert set(spheres) == set(RG2_LINKS)
    cfg = {"robot_cfg": {"kinematics": {"collision_spheres": {"base_link": []}, "collision_link_names": ["base_link"]}}}
    merged = merge_spheres(cfg, spheres)
    kin = merged["robot_cfg"]["kinematics"]
    assert RG2_LINKS[0] in kin["collision_spheres"] and RG2_LINKS[0] in kin["collision_link_names"]
