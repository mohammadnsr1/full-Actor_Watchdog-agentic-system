"""WatchDog consultation (Milestone 7): free-form text, optional measurements with units."""

import pytest

from robot_backend.events.schema import EVENT_TYPE_PAYLOAD_CLASSES, KNOWN_EVENT_TYPES, ConsultationPayload, make_event
from robot_backend.watchdog.rulings import validate_consultation


def test_free_text_alone_is_enough():
    text, clean = validate_consultation(
        "  u_orange jammed against the left wall of its slot on the last two tries;\n"
        "the clamp effort reached 90% of its limit each time. A third forced try risks the part.  ")
    assert text.startswith("u_orange jammed") and text.endswith("risks the part.") and clean == {}


def test_measurements_are_checked_when_given():
    _, clean = validate_consultation("seated off", {"frame": "u_brown_placement", "position_offset_m": [0.0018, 0, 0], "attempts_count": 2})
    assert clean == {"frame": "u_brown_placement", "position_offset_m": [0.0018, 0.0, 0.0], "attempts_count": 2.0}


@pytest.mark.parametrize("measurements, match", [
    ({"offset": 0.002}, "unit"),
    ({"offset_m": "2 mm"}, "number"),
    ({"offset_m": True}, "number"),
    ({"position_offset_m": [0.001, 0, 0]}, "frame"),
])
def test_bad_measurements_rejected(measurements, match):
    with pytest.raises(ValueError, match=match):
        validate_consultation("text", measurements)


def test_text_required_and_bounded():
    with pytest.raises(ValueError):
        validate_consultation("   ")
    with pytest.raises(ValueError):
        validate_consultation("x" * 2001)


def test_consultation_is_a_known_event_type():
    assert "consultation" in KNOWN_EVENT_TYPES and EVENT_TYPE_PAYLOAD_CLASSES["consultation"] is ConsultationPayload
    ev = make_event("consultation", ConsultationPayload(part=None, text="t", published_to_planner=True))
    assert ev.event_type == "consultation" and ev.payload["text"] == "t" and ev.payload["measurements"] == {}
