"""Grasp-centre offset composition -- pure geometry (robot_backend.utils.geometry).

grasp centre = object pose (x) grasp_center_offset, both object-relative: the
convention the planner templates (executor/planner_templates/common/geometry.py)
build on. Runs with no cell.
"""

from __future__ import annotations

import pytest

from robot_backend.utils.geometry import compose_poses, make_pose


TOP_DOWN = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}  # 180 deg about x: z -> -z
IDENTITY = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}


def _object_pose(pos, orient):
    return make_pose(position=pos, orientation=orient, frame_id="world")


def test_identity_offset_leaves_grasp_centre_at_object_position():
    obj = _object_pose({"x": 0.5, "y": 0.2, "z": 0.1}, TOP_DOWN)
    offset = make_pose(position={"x": 0.0, "y": 0.0, "z": 0.0}, orientation=IDENTITY)
    gc = compose_poses(obj, offset)
    assert gc["position"]["x"] == pytest.approx(0.5)
    assert gc["position"]["y"] == pytest.approx(0.2)
    assert gc["position"]["z"] == pytest.approx(0.1)
    # Orientation unchanged (composed with identity).
    for axis in ("x", "y", "z", "w"):
        assert gc["orientation"][axis] == pytest.approx(TOP_DOWN[axis])


def test_translation_only_offset_with_identity_orientation():
    # Object at identity orientation: the offset applies as a pure translation.
    obj = _object_pose({"x": 0.5, "y": 0.2, "z": 0.1}, IDENTITY)
    offset = make_pose(
        position={"x": 0.01, "y": -0.02, "z": 0.03}, orientation=IDENTITY
    )
    gc = compose_poses(obj, offset)
    assert gc["position"]["x"] == pytest.approx(0.51)
    assert gc["position"]["y"] == pytest.approx(0.18)
    assert gc["position"]["z"] == pytest.approx(0.13)


def test_offset_is_expressed_in_object_frame_not_world():
    # With a top-down object orientation (180 deg about x), an object-frame +z
    # offset points along world -z: the grasp centre sits BELOW the object,
    # which is exactly why the offset must be composed in the object frame
    # rather than added in world coordinates.
    obj = _object_pose({"x": 0.5, "y": 0.2, "z": 0.1}, TOP_DOWN)
    offset = make_pose(position={"x": 0.0, "y": 0.0, "z": 0.03}, orientation=IDENTITY)
    gc = compose_poses(obj, offset)
    assert gc["position"]["x"] == pytest.approx(0.5)
    assert gc["position"]["y"] == pytest.approx(0.2)
    assert gc["position"]["z"] == pytest.approx(0.07)


def test_frame_id_preserved_through_composition():
    obj = _object_pose({"x": 0.0, "y": 0.0, "z": 0.0}, IDENTITY)
    offset = make_pose(position={"x": 0.1, "y": 0.0, "z": 0.0}, orientation=IDENTITY)
    gc = compose_poses(obj, offset)
    assert gc["frame_id"] == "world"
