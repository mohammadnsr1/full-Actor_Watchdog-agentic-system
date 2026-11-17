from isaac.cell_command_server.contacts import Pad, PadContactTracker, normalize_event_type, parse_pads

LEFT = "/W/left_inner_finger"
FRAME = "/W/left_inner_finger/left_contact_frame"
PART = "/W/u_green"


def identity_frame(_frame, points):
    return points


def rep(event, c0="a", c1="x", points=((0.0, 0.0, 0.0),), impulse=0.0, swap=False):
    b0, b1 = (PART, LEFT) if swap else (LEFT, PART)
    return {"body0": b0, "body1": b1, "collider0": c0, "collider1": c1, "event": event, "points": list(points), "impulse": impulse}


def tracker(radius=0.015):
    return PadContactTracker([Pad(LEFT, FRAME)], radius, identity_frame)


def test_normalize_event_type():
    assert normalize_event_type("ContactEventType.CONTACT_FOUND") == "found"
    assert normalize_event_type("ContactEventType.CONTACT_PERSIST") == "persist"
    assert normalize_event_type("ContactEventType.CONTACT_LOST") == "lost"


def test_parse_pads_accepts_strings_and_dicts():
    pads = parse_pads(["/a", {"body": "/b", "surface_frame": "/b/f"}])
    assert pads == [Pad("/a"), Pad("/b", "/b/f")]


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


def test_on_pad_surface_uses_closest_point_to_frame():
    t = tracker(0.015)
    ev = t.feed([rep("CONTACT_FOUND", points=[(0.0, 0.05, 0.0), (0.0, 0.0, 0.01)])])
    assert ev[0]["on_pad_surface"] is True
    assert abs(ev[0]["distance_m"] - 0.01) < 1e-9
    assert ev[0]["point_in_pad_frame"] == [0.0, 0.0, 0.01]


def test_back_of_finger_is_not_on_pad_and_lost_reports_whole_contact():
    t = tracker(0.015)
    ev = t.feed([rep("CONTACT_FOUND", points=[(0.0, 0.03, 0.0)], impulse=0.2)])
    assert ev[0]["on_pad_surface"] is False
    t.feed([rep("CONTACT_PERSIST", points=[(0.0, 0.005, 0.0)], impulse=0.9)])
    ev = t.feed([rep("CONTACT_LOST", points=())])
    assert ev[0]["event"] == "lost" and ev[0]["on_pad_surface"] is True and ev[0]["impulse"] == 0.9


def test_no_radius_never_claims_on_pad():
    ev = tracker(None).feed([rep("CONTACT_FOUND")])
    assert ev[0]["on_pad_surface"] is False and ev[0]["distance_m"] == 0.0


def test_other_bodies_are_ignored():
    t = tracker()
    assert t.feed([{"body0": "/W/table", "body1": PART, "event": "CONTACT_FOUND"}]) == []
