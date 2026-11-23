"""Pure tests of the template's task interpretation and geometry.

Run with the workspace sourced (robot_backend.utils):
    source ros2_ws/install/setup.bash
    python3 -m pytest -q executor/planner_templates/plan_given
"""

import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
REPO = next(p for p in HERE.parents if (p / "config" / "cell.yaml").is_file())
sys.path.insert(0, str(HERE))

import task as T  # noqa: E402

TASK = REPO / "config" / "tasks" / "fmb_assembly_2.yaml"


def steps():
    return {s["step_id"]: s for s in T.load_task(TASK)["steps"]}


def test_step_kinds_cover_the_task():
    kinds = [T.step_kind(s) for s in T.load_task(TASK)["steps"]]
    assert kinds == ["pick", "place_approach", "place"] * 4


def test_unknown_step_shape_is_refused():
    with pytest.raises(ValueError):
        T.step_kind({"step_id": "wiggle_u_brown"})


def test_pick_geometry_matches_pick_check():
    # u_brown at its nominal pose; pick_check computed (2026-11-17 run):
    # pre-grasp [0.2404, -0.3167, 0.2272], grasp [0.2404, -0.3167, 0.1272], lift z 0.3272
    obj = T.obs_pose([0.21602563560009003, -0.31123197078704834, 0.03987877443432808],
                     [-0.7027253749154712, 0.07853735507196224, -0.07853929222298375, 0.7027378678321837])
    g = T.pick_geometry(steps()["pick_u_brown"], obj)
    p = lambda pose: [round(pose["position"][k], 3) for k in "xyz"]  # noqa: E731
    assert p(g["pre_grasp_tcp"]) == pytest.approx([0.240, -0.317, 0.227], abs=0.002)
    assert p(g["grasp_tcp"]) == pytest.approx([0.240, -0.317, 0.127], abs=0.002)
    assert g["lift_tcp"]["position"]["z"] - g["grasp_tcp"]["position"]["z"] == pytest.approx(0.2)


def test_place_with_identity_frame_and_no_offset_targets_the_frame():
    frame = T.obs_pose([0.4, -0.2, 0.05], [0, 0, 0, 1])
    step = {"step_id": "place_x", "grasp_width_m": 0.02}
    g = T.place_geometry(step, frame)
    assert g["object_target"] is frame
    assert g["grasp_center"]["position"] == pytest.approx({"x": 0.4, "y": -0.2, "z": 0.05})


def test_target_offset_is_in_the_frame_axes():
    # Frame rotated 90 deg about x: its local +y points along world +z.
    s = 2 ** -0.5
    frame = T.obs_pose([0.4, -0.2, 0.05], [s, 0, 0, s])
    step = {"step_id": "place_approach_x", "grasp_width_m": 0.02, "target_offset": {"position": {"y": -0.05}}}
    g = T.place_geometry(step, frame)
    assert g["object_target"]["position"]["z"] == pytest.approx(0.0)
    assert g["object_target"]["position"]["y"] == pytest.approx(-0.2)


def test_retreat_backs_off_along_the_tool_axis():
    # TCP pointing down (180 deg about x): own -z backing off is world +z.
    tcp = T.obs_pose([0.3, 0.0, 0.1], [1, 0, 0, 0])
    r = T.retreat_pose(tcp, -0.10)
    assert r["position"]["z"] == pytest.approx(0.2)


def test_grasp_parameters():
    assert T.guarded_target_width(0.02) == pytest.approx(0.015)
    parts = {"u_brown": {"grasp": {"force_threshold_n": 10.0}}}
    assert T.grasp_force(parts, "u_brown") == 10.0
    assert T.grasp_force(parts, "unknown") == T.DEFAULT_FORCE_THRESHOLD
    assert T.motion_scaling({"motion": {"max_velocity_scaling_factor": 0.05, "max_acceleration_scaling_factor": 0.05}}, 0.2) == (0.05, 0.05)
    assert T.motion_scaling({}, 0.2) == (0.2, 0.2)
