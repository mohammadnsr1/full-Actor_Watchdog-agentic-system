import pytest

from isaac.cell_command_server.contacts import Pad, PadContactTracker, normalize_event_type, parse_face, parse_pads

LEFT = "/W/left_inner_finger"
FRAME = "/W/left_inner_finger/left_contact_frame"
PART = "/W/u_green"
# The RG2 pad face measured from the finger mesh (config/platforms/sim.yaml scene.pad_face).
FACE = parse_face({"x_m": [-0.0105, 0.0097], "y_m": [-0.002, 0.004], "z_m": [-0.0148, 0.0150]})


def identity_frame(_frame, points):
    return points


def rep(event, c0="a", c1="x", points=((0.0, 0.0013, 0.0),), impulse=0.0, swap=False):
    b0, b1 = (PART, LEFT) if swap else (LEFT, PART)
    return {"body0": b0, "body1": b1, "collider0": c0, "collider1": c1, "event": event, "points": list(points), "impulse": impulse}


def tracker(face=FACE):
    return PadContactTracker([Pad(LEFT, FRAME)], face, identity_frame)


def test_normalize_event_type():
    assert normalize_event_type("ContactEventType.CONTACT_FOUND") == "found"
    assert normalize_event_type("ContactEventType.CONTACT_PERSIST") == "persist"
    assert normalize_event_type("ContactEventType.CONTACT_LOST") == "lost"


def test_parse_pads_accepts_strings_and_dicts():
    pads = parse_pads(["/a", {"body": "/b", "surface_frame": "/b/f"}])
    assert pads == [Pad("/a"), Pad("/b", "/b/f")]


def test_parse_face_rejects_reversed_and_missing():
    assert parse_face(None) is None
    with pytest.raises(ValueError):
        parse_face({"x_m": [0.01, -0.01], "y_m": [0, 1], "z_m": [0, 1]})
    with pytest.raises(ValueError):
        parse_face({"x_m": [0, 1]})


def test_shape_pairs_fold_into_one_found_and_one_lost():
    t = tracker()
    ev = t.feed([rep("CONTACT_FOUND", "a", "x"), rep("CONTACT_FOUND", "a", "y")])
    assert [e["event"] for e in ev] == ["found"]
    assert t.feed([rep("CONTACT_PERSIST", "a", "x"), rep("CONTACT_PERSIST", "a", "y")]) == []
    assert t.feed([rep("CONTACT_LOST", "a", "x", points=())]) == []
    ev = t.feed([rep("CONTACT_LOST", "a", "y", points=())])
    assert [e["event"] for e in ev] == ["lost"] and t.active() == []


def test_pad_is_identified_in_either_body_order():
    ev = tracker().feed([rep("CONTACT_FOUND", swap=True)])
    assert ev[0]["pad"] == LEFT and ev[0]["body"] == PART


def test_observed_grasp_points_are_on_the_face():
    # point_in_pad_frame of the eight pick_check contacts, 2026-11-17 (metres);
    # the u_orange one (15.1 mm from the origin) failed the old 15 mm radius.
    observed = [(-0.0056, 0.0013, 0.0135), (0.0087, 0.0014, 0.0093), (0.0096, 0.0015, -0.0115), (-0.0100, 0.0014, 0.0029),
                (0.0094, 0.0013, -0.0032), (-0.0085, 0.0013, -0.0032), (0.0039, 0.0013, 0.0126), (-0.0099, 0.0013, 0.0084)]
    for p in observed:
        t = tracker()
        assert t.feed([rep("CONTACT_FOUND", points=[p])])[0]["on_pad_surface"] is True, p


def test_back_side_and_beyond_the_edge_are_not_on_the_face():
    for p in [(0.0, -0.024, 0.0),      # back of the finger
              (0.0, 0.0013, -0.030),   # below the pad, on the finger tip side
              (0.015, 0.0013, 0.0)]:   # past the side edge
        assert tracker().feed([rep("CONTACT_FOUND", points=[p])])[0]["on_pad_surface"] is False, p


def test_point_on_face_is_reported_over_a_closer_off_face_point():
    ev = tracker().feed([rep("CONTACT_FOUND", points=[(0.0, -0.003, 0.0), (0.009, 0.0013, 0.012)])])
    assert ev[0]["on_pad_surface"] is True and ev[0]["point_in_pad_frame"] == [0.009, 0.0013, 0.012]


def test_lost_reports_whether_any_point_was_on_the_face():
    t = tracker()
    t.feed([rep("CONTACT_FOUND", points=[(0.0, -0.024, 0.0)], impulse=0.2)])
    t.feed([rep("CONTACT_PERSIST", points=[(0.0, 0.0013, 0.0)], impulse=0.9)])
    ev = t.feed([rep("CONTACT_LOST", points=())])
    assert ev[0]["event"] == "lost" and ev[0]["on_pad_surface"] is True and ev[0]["impulse"] == 0.9


def test_no_face_never_claims_on_pad():
    ev = tracker(None).feed([rep("CONTACT_FOUND")])
    assert ev[0]["on_pad_surface"] is False and ev[0]["distance_m"] is not None


def test_other_bodies_are_ignored():
    assert tracker().feed([{"body0": "/W/table", "body1": PART, "event": "CONTACT_FOUND"}]) == []
