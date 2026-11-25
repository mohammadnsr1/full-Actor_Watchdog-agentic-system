import math

import pytest
import yaml

from cell_platform_interfaces.placements import compose, derive_into, load_placements, parse_placements, relative

YAW90 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))


def close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def test_relative_then_compose_round_trips():
    fx_p, fx_q = (0.5, -0.2, 0.01), YAW90
    pl_p, pl_q = (0.53, -0.17, 0.04), (0.0, 0.0, 0.0, 1.0)
    rp, rq = relative(fx_p, fx_q, pl_p, pl_q)
    p, q = compose(fx_p, fx_q, rp, rq)
    assert close(p, pl_p) and (close(q, pl_q) or close(q, tuple(-c for c in pl_q)))


def test_offset_follows_the_fixture():
    pl = parse_placements({"fixture": "base1", "placements": {"a_placement": {"position": [0.1, 0, 0], "orientation_xyzw": [0, 0, 0, 1]}}})
    poses = {"base1": {"position": [1.0, 2.0, 0.0], "orientation_xyzw": list(YAW90)}}
    assert derive_into(poses, pl) == ["a_placement"]
    assert close(poses["a_placement"]["position"], (1.0, 2.1, 0.0))
    assert poses["a_placement"]["derived_from"] == "base1"


def test_derived_replaces_a_published_frame_and_drops_without_fixture():
    pl = parse_placements({"fixture": "base1", "placements": {"a_placement": {"position": [0, 0, 0.05], "orientation_xyzw": [0, 0, 0, 1]}}})
    poses = {"base1": {"position": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 1]}, "a_placement": {"position": [9, 9, 9]}}
    derive_into(poses, pl)
    assert close(poses["a_placement"]["position"], (0, 0, 0.05))
    poses = {"a_placement": {"position": [9, 9, 9]}}
    assert derive_into(poses, pl) == [] and "a_placement" not in poses


def test_bad_entry_rejected():
    with pytest.raises(ValueError):
        parse_placements({"placements": {"x": {"position": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 1]}}})


def test_load_from_folder(tmp_path):
    (tmp_path / "config" / "placements").mkdir(parents=True)
    (tmp_path / "config" / "placements" / "a.yaml").write_text(yaml.safe_dump(
        {"fixture": "base1", "placements": {"p1": {"position": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 2]}}}))
    pl = load_placements(tmp_path)
    assert list(pl) == ["p1"] and close(pl["p1"].orientation_xyzw, (0, 0, 0, 1))
    assert load_placements(tmp_path, "missing") == {}


def test_symmetries_parse_and_load(tmp_path):
    from cell_platform_interfaces.placements import load_symmetries, parse_symmetries
    import pytest
    data = {"symmetries": {"u": [{"position": [0, 0, 0], "orientation_xyzw": [0, 2, 0, 0]}]}}
    assert parse_symmetries(data) == {"u": [((0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0))]}
    with pytest.raises(ValueError):
        parse_symmetries({"symmetries": {"u": [{"position": [0, 0]}]}})
    assert parse_symmetries({}) == {}
    d = tmp_path / "config" / "placements"
    d.mkdir(parents=True)
    (d / "asm.yaml").write_text("symmetries:\n  u:\n  - {position: [0, 0, 0], orientation_xyzw: [0, 1, 0, 0]}\n")
    assert list(load_symmetries(tmp_path, "asm")) == ["u"]
