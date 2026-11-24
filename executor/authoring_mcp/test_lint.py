"""evaluate_planner refuses planners that reach around the contract."""

import os
import shutil
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent


@pytest.fixture()
def server(tmp_path, monkeypatch):
    pytest.importorskip("mcp")
    monkeypatch.setenv("AUTHORING_SESSION_DIR", str(tmp_path))
    sys.path.insert(0, str(HERE))
    import server as srv  # noqa: E402

    return srv


def test_shipped_templates_pass(server, tmp_path):
    for cond in ("plan_given", "plan_free"):
        dst = tmp_path / cond
        shutil.copytree(server.REPO / "executor" / "planner_templates" / cond, dst, symlinks=False)
        assert server.lint_planner(dst) == [], cond


def test_added_backdoor_is_caught(server, tmp_path):
    dst = tmp_path / "p"
    shutil.copytree(server.REPO / "executor" / "planner_templates" / "plan_free", dst, symlinks=False)
    with open(dst / "planner.py", "a") as f:
        f.write("\nfrom cell_backend_sim.scene import CommandServerClient  # cheat\npl = open('config/placements/fmb_assembly_2.yaml')\n")
    hits = server.lint_planner(dst)
    assert any("CommandServerClient" in h for h in hits) and any("config/placements" in h for h in hits)
