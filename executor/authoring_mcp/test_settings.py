"""The two authoring conditions' permission files stay in step.

plan_free = plan_given + read denies on everything that carries the plan."""

import json
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SETTINGS = REPO / "config" / "settings"

PLAN_LEAKS = ["./config/tasks/**", "./executor/planner_templates/plan_given/**", "./executor/workspace/plan_given/**",
              "./tools/pick_check.py", "./ros2_ws/src/robot_backend/test/**"]


def load(name):
    return json.loads((SETTINGS / name).read_text())


def test_plan_free_is_plan_given_plus_denies():
    for plat in ("sim", "hardware"):
        given, free = load(f"executor.{plat}.settings.json"), load(f"executor.{plat}.plan_free.settings.json")
        for key in ("model", "effortLevel", "autoMemoryEnabled"):
            assert given[key] == free[key]
        assert given["permissions"]["allow"] == free["permissions"]["allow"]
        assert set(given["permissions"]["deny"]) <= set(free["permissions"]["deny"])
        for path in PLAN_LEAKS:
            for tool in ("Read", "Grep", "Glob"):
                assert f"{tool}({path})" in free["permissions"]["deny"], (plat, tool, path)


def test_both_conditions_hide_records_and_shell():
    for name in ("executor.sim.settings.json", "executor.hardware.settings.json",
                 "executor.sim.plan_free.settings.json", "executor.hardware.plan_free.settings.json"):
        deny = load(name)["permissions"]["deny"]
        assert "Read(./records/**)" in deny and "Bash" in deny
        assert "Edit(./executor/planner_templates/**)" in deny
