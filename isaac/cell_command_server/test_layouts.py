"""Pure tests for the seeded-layout logic (no Kit)."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from isaac.cell_command_server import layouts as L  # noqa: E402

LAYOUT = {
    "parts": {
        "a": {"prim": "/World/a", "position": [0.1, 0.2, 0.0], "orientation_xyzw": [0, 0, 0, 1], "jitter": {"xy_m": 0.01, "yaw_deg": 10}},
        "b": {"prim": "/World/b", "position": [1.0, 0.0, 0.0]},
    },
    "verify_tolerances": {"position_m": 0.005},
}


def test_seed_zero_is_nominal_and_deterministic():
    p0 = L.seeded_placements(LAYOUT, 0)
    assert p0["a"]["position"] == [0.1, 0.2, 0.0] and not p0["a"]["jittered"]
    p3 = L.seeded_placements(LAYOUT, 3)
    assert p3 == L.seeded_placements(LAYOUT, 3)
    assert p3["a"]["jittered"] and abs(p3["a"]["position"][0] - 0.1) <= 0.01
    assert not p3["b"]["jittered"]  # no jitter block: never moved


def test_validate_reports_todo():
    assert L.validate_layout(LAYOUT) == []
    bad = {"parts": {"a": {"prim": "/World/a"}}, "verify_tolerances": {}}
    problems = L.validate_layout(bad)
    assert any("position" in p for p in problems) and any("TODO" in p for p in problems)


def test_pose_error():
    dp, dq = L.pose_error({"position": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 1]}, {"position": [0.003, 0.004, 0], "orientation_xyzw": [0, 0, 0.0871557, 0.9961947]})
    assert abs(dp - 0.005) < 1e-9 and abs(dq - 10.0) < 1e-3
