"""task_data/ must be exactly what tools/make_task_data.py derives from the config."""
import hashlib
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_task_data as M  # noqa: E402


def test_task_data_is_current():
    stale = [str(p) for p, text in M.build().items() if not p.is_file() or p.read_text() != text]
    assert not stale, f"run python3 tools/make_task_data.py: {stale}"


def test_records_hold_together():
    files = M.build()
    task = yaml.safe_load(files[M.OUT / "tasks" / M.TASK_ID / "task.yaml"])
    decl = yaml.safe_load(files[M.OUT / "environments" / M.ENVIRONMENT / "declaration.yaml"])
    comp_text = files[M.OUT / "robots" / M.ROBOT / "composite.yaml"]
    assert task["environment"] == decl["name"] and task["roles"][0]["robot"] == decl["robots"][0]["name"]
    assert decl["robots"][0]["composite"]["sha256"] == hashlib.sha256(comp_text.encode()).hexdigest()
    objs = {o["object"]: o for o in task["binding"]["objects"]}
    assert set(objs) == {"base1", "u_brown", "u_orange", "line_green", "inverted_u_yellow"}
    assert task["goal"]["equivalence"]["interchangeable"] == [[objs["u_brown"]["occurrence"], objs["u_orange"]["occurrence"]]]
    assert "*id" not in files[M.OUT / "tasks" / M.TASK_ID / "task.yaml"]       # no YAML aliases
    text = "".join(files.values())
    for leak in ("placements/", "requirements/", "max_clamp_effort", "_placement"):
        assert leak not in text, leak                                          # nothing WatchDog-only
