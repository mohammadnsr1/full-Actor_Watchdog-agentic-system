from runner.episode_runner.checker import check, pose_error
from runner.episode_runner.manifest import tree_hash

Q0 = [0.0, 0.0, 0.0, 1.0]
REQ = {"acceptance": {"parts": {"p": {"placement_frame": "p_place"}}, "fixtures": ["base"]}, "bystanders": ["b"]}
PARTS = {"p": {"tolerances": {"position_m": 0.01, "orientation_deg": 10.0}}, "b": {"tolerances": {"disturbance_tolerance_m": 0.02}}}


def pose(x, y=0.0, z=0.0, q=Q0):
    return {"position": [x, y, z], "orientation_xyzw": q}


def scene(p_x, base_x=1.0, b_x=2.0):
    return {"p": pose(p_x), "p_place": pose(0.5), "base": pose(base_x), "b": pose(b_x)}


def test_pose_error():
    assert pose_error(pose(0), pose(0.003, 0.004)) == (0.005, 0.0)


def test_placed_part_passes():
    r = check(REQ, PARTS, scene(0.0), scene(0.505))
    assert r["pass"] and r["parts"]["p"]["status"] == "pass" and r["parts_placed"] == 1


def test_part_off_its_frame_fails():
    r = check(REQ, PARTS, scene(0.0), scene(0.53))
    assert not r["pass"] and r["parts"]["p"]["status"] == "fail"


def test_wrong_orientation_fails():
    after = scene(0.5)
    after["p"]["orientation_xyzw"] = [0.0, 0.0, 0.2588, 0.9659]  # 30 deg
    assert check(REQ, PARTS, scene(0.0), after)["parts"]["p"]["status"] == "fail"


def test_moved_fixture_and_bystander_fail():
    r = check(REQ, PARTS, scene(0.0), scene(0.5, base_x=1.01, b_x=2.03))
    assert not r["pass"]
    assert r["fixtures"]["base"]["status"] == "fail"       # default 5 mm
    assert r["bystanders"]["b"]["status"] == "fail"        # its own 20 mm


def test_missing_pose_fails():
    after = scene(0.5)
    del after["p_place"]
    r = check(REQ, PARTS, scene(0.0), after)
    assert not r["pass"] and r["parts"]["p"]["status"] == "missing"


Z180 = [0.0, 0.0, 1.0, 0.0]
PAIR_REQ = {"acceptance": {"parts": {"a": {"placement_frame": "a_place"}, "c": {"placement_frame": "c_place"}},
                           "interchangeable": [["a", "c"]], "fixtures": []}}
PAIR_PARTS = {k: {"tolerances": {"position_m": 0.005, "orientation_deg": 5.0}} for k in ("a", "c")}


def test_swapped_identical_parts_pass_only_when_interchangeable():
    after = {"a": pose(0.3), "c": pose(0.1), "a_place": pose(0.1), "c_place": pose(0.3)}
    r = check(PAIR_REQ, PAIR_PARTS, {}, after)
    assert r["pass"] and r["parts"]["a"]["placement_frame"] == "c_place" and r["parts"]["a"]["swapped_from"] == "a_place"
    strict = {"acceptance": {**PAIR_REQ["acceptance"], "interchangeable": []}}
    assert not check(strict, PAIR_PARTS, {}, after)["pass"]


def test_interchangeable_parts_fill_one_frame_each():
    after = {"a": pose(0.1), "c": pose(0.1, 0.001), "a_place": pose(0.1), "c_place": pose(0.3)}
    r = check(PAIR_REQ, PAIR_PARTS, {}, after)
    assert not r["pass"] and r["parts_placed"] == 1


def test_symmetric_turn_is_seated_with_its_offset():
    # a part whose symmetry is 180 deg about its own z through (0.01, 0, 0): S = (t, Rz180) with t = c - R c
    sym = {"p": [((0.02, 0.0, 0.0), tuple(Z180))]}
    after = scene(0.5)
    after["p"] = pose(0.52, q=Z180)                      # the turned part's origin sits 20 mm over
    assert check(REQ, PARTS, scene(0.0), after)["parts"]["p"]["status"] == "fail"
    r = check(REQ, PARTS, scene(0.0), after, sym)
    assert r["pass"] and r["parts"]["p"]["symmetry"] == 1 and r["parts"]["p"]["position_error_m"] == 0.0


def test_turn_that_is_not_a_symmetry_still_fails():
    after = scene(0.5)
    after["p"]["orientation_xyzw"] = [0.0, 0.0, 0.7071068, 0.7071068]   # 90 deg about z
    assert not check(REQ, PARTS, scene(0.0), after, {"p": [((0.0, 0.0, 0.0), tuple(Z180))]})["pass"]


def test_tree_hash_ignores_caches(tmp_path):
    (tmp_path / "a.py").write_text("x")
    (tmp_path / "__pycache__").mkdir()
    (tmp_path / "__pycache__" / "a.pyc").write_text("y")
    h1 = tree_hash(tmp_path)
    (tmp_path / "planner.log").write_text("log")
    assert tree_hash(tmp_path)["sha256"] == h1["sha256"] and h1["files"] == 1
    (tmp_path / "a.py").write_text("changed")
    assert tree_hash(tmp_path)["sha256"] != h1["sha256"]
