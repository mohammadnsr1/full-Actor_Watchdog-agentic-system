import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
sys.path.insert(0, str(HERE))

from derive_plan_free_task import derive, out_path  # noqa: E402

FULL = sorted(p for p in (REPO / "config" / "tasks").glob("*.yaml") if not p.name.endswith(".plan_free.yaml"))


def test_every_full_task_has_an_up_to_date_plan_free_file():
    assert FULL
    for full in FULL:
        derived = out_path(full)
        assert derived.is_file(), f"run: python3 tools/derive_plan_free_task.py {full.relative_to(REPO)}"
        assert derived.read_text(encoding="utf-8") == derive(full), f"{derived.name} is stale: re-derive it"


def test_plan_free_carries_no_plan():
    for full in FULL:
        free = yaml.safe_load(out_path(full).read_text(encoding="utf-8"))
        assert "steps" not in free and "step_plan" not in free
        text = out_path(full).read_text(encoding="utf-8")
        for leak in ("grasp_center_offset", "target_offset", "approach_distance_m", "lift_height_m"):
            assert leak not in text
        orig = yaml.safe_load(full.read_text(encoding="utf-8"))
        assert free["object_ids"].splitlines()[0] == orig["object_ids"].splitlines()[0]
        for ref in ("grasp_width_m", "the step's", "TF frame", "_placement"):
            assert ref not in free["instruction_spec"], ref
        assert "_placement" not in free["object_ids"] and "TF frame" not in free["object_ids"]
