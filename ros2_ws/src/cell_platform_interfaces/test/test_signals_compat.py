from __future__ import annotations

from pathlib import Path

import pytest

from cell_platform_interfaces.compat import bag_record_command, python_baseline_ok, set_if_present
from cell_platform_interfaces.platform_config import PlatformConfig, load_platform_config
from cell_platform_interfaces.signals import (
    REQUIRED_SIGNALS,
    SignalConfigError,
    evidence_sources_report,
    is_available,
    signal_availability,
    status_of,
)
from cell_platform_interfaces.timebase import parse_wall_time_iso, wall_time_iso

REPO = Path(__file__).resolve().parents[4]


def test_both_platform_files_declare_every_signal():
    for name in ("sim", "hardware"):
        p = load_platform_config(name, REPO)
        signals = signal_availability(p)
        assert set(REQUIRED_SIGNALS) <= set(signals)


def test_sim_vs_hardware_statuses():
    sim = load_platform_config("sim", REPO)
    hw = load_platform_config("hardware", REPO)
    assert status_of(sim, "finger_effort") == "available"
    assert status_of(hw, "finger_effort") == "missing"
    assert is_available(sim, "grip_detected", allow_proxy=True)
    assert not is_available(sim, "grip_detected")
    assert status_of(hw, "sim_time") == "missing"


def test_bad_signal_block_rejected(tmp_path: Path):
    p = PlatformConfig("sim", {"signals": {"finger_effort": {"status": "maybe"}}}, tmp_path / "s.yaml", REPO)
    with pytest.raises(SignalConfigError):
        signal_availability(p)


def test_evidence_report_shape():
    report = evidence_sources_report(load_platform_config("sim", REPO))
    assert report["platform"] == "sim" and report["pose_source"] == "ground_truth"
    assert report["cameras"]["camera2"]["role"] == "table"
    assert "signals" in report


def test_bag_command_spells_storage():
    cmd = bag_record_command("/tmp/x", ["/a", "/b"])
    assert cmd[:5] == ["ros2", "bag", "record", "-s", "mcap"] and cmd[-2:] == ["/a", "/b"]


def test_set_if_present_on_plain_object():
    class Msg:
        def __init__(self):
            self.a = 0

        @staticmethod
        def get_fields_and_field_types():
            return {"a": "int32"}

    m = Msg()
    assert set_if_present(m, "a", 3) and m.a == 3
    assert not set_if_present(m, "b", 1)


def test_wall_time_iso_roundtrip():
    assert python_baseline_ok()
    text = wall_time_iso(1_700_000_000_123_456_789)
    assert text.endswith("Z") and "T" in text
    # microsecond text, so the round trip is exact to within one microsecond
    assert abs(parse_wall_time_iso(text) - 1_700_000_000_123_456_789) <= 1_000
