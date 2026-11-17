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


def test_tree_hash_ignores_caches(tmp_path):
    (tmp_path / "a.py").write_text("x")
    (tmp_path / "__pycache__").mkdir()
    (tmp_path / "__pycache__" / "a.pyc").write_text("y")
    h1 = tree_hash(tmp_path)
    (tmp_path / "planner.log").write_text("log")
    assert tree_hash(tmp_path)["sha256"] == h1["sha256"] and h1["files"] == 1
    (tmp_path / "a.py").write_text("changed")
    assert tree_hash(tmp_path)["sha256"] != h1["sha256"]
