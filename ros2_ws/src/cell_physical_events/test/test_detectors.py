from __future__ import annotations

from cell_physical_events.detectors import EffortDetector, MotionDetector, PartDetector, PoseSample

S = 1_000_000_000


def test_everything_disabled_with_zero_thresholds():
    cfg = {}
    assert MotionDetector(cfg).disabled_reason()
    assert len(EffortDetector(cfg, "finger_joint", True).disabled_reasons()) == 2
    p = PartDetector(cfg, {"a": {}}, set())
    assert len(p.disabled_reasons()) == 4
    assert MotionDetector(cfg).feed([0, 0], [1, 1], 0) == []
    assert p.feed({"a": PoseSample("a", (0, 0, 1), 1.0, "ground_truth", 0)}, (0, 0, 1), 0) == []


def test_motion_start_stop_with_debounce():
    m = MotionDetector({"motion_speed_threshold_rad_s": 0.1, "motion_debounce_s": 0.2})
    assert m.feed([0.0], [0.5], 0) == []            # candidate starts
    assert m.feed([0.0], [0.5], int(0.1 * S)) == []  # inside debounce
    ev = m.feed([0.0], [0.5], int(0.3 * S))
    assert [e.kind for e in ev] == ["motion_started"] and ev[0].source == "joint_state"
    assert m.feed([0.0], [0.0], int(0.4 * S)) == []
    ev = m.feed([0.0], [0.0], int(0.7 * S))
    assert [e.kind for e in ev] == ["motion_stopped"]


def test_effort_contact_proxy_and_rise():
    cfg = {"effort_contact_threshold": 1.0, "effort_contact_ticks": 2, "effort_proxy_confidence": 0.6,
           "effort_rise_rate_per_s": 5.0, "effort_rise_window_s": 1.0}
    e = EffortDetector(cfg, "finger_joint", True)
    assert e.disabled_reasons() == []
    assert e.feed(0.0, 0) == []
    first = e.feed(2.0, int(0.1 * S))  # one tick above; the 20/s slope fires effort_rising
    assert [x.kind for x in first] == ["effort_rising"] and first[0].source == "joint_state"
    out = e.feed(2.0, int(0.2 * S))
    kinds = [x.kind for x in out]
    assert "contact_made" in kinds and "effort_rising" not in kinds
    made = [x for x in out if x.kind == "contact_made"][0]
    assert made.source == "effort_proxy" and made.confidence == 0.6
    assert e.feed(0.0, int(0.3 * S)) == []
    assert [x.kind for x in e.feed(0.0, int(0.4 * S))] == ["contact_lost"]


def test_effort_proxy_off_when_contact_report_exists():
    cfg = {"effort_contact_threshold": 1.0, "effort_contact_ticks": 1, "effort_proxy_confidence": 0.6}
    e = EffortDetector(cfg, "finger_joint", contact_proxy_allowed=False)
    assert not e.contact_enabled
    assert e.feed(5.0, 0) == [] and e.feed(5.0, 1) == []


def test_part_lift_approach_engage_release_settle():
    cfg = {"lift_threshold_m": 0.02, "hold_distance_m": 0.05, "release_distance_m": 0.10,
           "settle_speed_m_s": 0.01, "settle_window_s": 0.5,
           "approach_distance_m": 0.05, "engage_distance_m": 0.01, "unknown_object_min_confidence": 0.5}
    p = PartDetector(cfg, {"u": {"placement": "u_placement"}}, {"u_placement"})
    assert p.disabled_reasons() == []
    frame = PoseSample("u_placement", (1.0, 0.0, 0.0), 1.0, "ground_truth", 0)

    def tick(pos, tcp, t, extra=None):
        poses = {"u": PoseSample("u", pos, 1.0, "ground_truth", t), "u_placement": frame}
        if extra:
            poses.update(extra)
        return [e.kind for e in p.feed(poses, tcp, t)]

    assert tick((0, 0, 0.0), (0, 0, 0.3), 0) == []
    assert tick((0, 0, 0.05), (0, 0, 0.06), 1 * S) == ["part_lifted"]
    assert p.held_part() == "u"
    assert tick((0.96, 0, 0.0), (0.96, 0, 0.01), 2 * S) == ["part_approaching_feature"]
    assert tick((1.0, 0, 0.0), (1.0, 0, 0.01), 3 * S) == ["part_engaged_feature"]
    assert tick((1.0, 0, 0.0), (1.0, 0, 0.5), 4 * S) == ["part_released"]
    assert p.held_part() is None
    # still since the release tick; the 0.5 s window has passed by 5 s
    assert tick((1.0, 0, 0.0), (1.0, 0, 0.5), 5 * S) == ["part_settled"]
    assert tick((1.0, 0, 0.0), (1.0, 0, 0.5), int(5.6 * S)) == []
    # an unknown detection appears once
    extra = {"mystery": PoseSample("mystery", (0.5, 0.5, 0), 0.9, "ground_truth", 6 * S)}
    assert tick((1.0, 0, 0.0), (1.0, 0, 0.5), 6 * S, extra) == ["workspace_object_appeared"]
    assert tick((1.0, 0, 0.0), (1.0, 0, 0.5), 7 * S, extra) == []
