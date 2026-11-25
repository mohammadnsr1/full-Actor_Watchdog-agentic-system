"""Platform MCP off the cell: delivery checks on the shipped scaffold, and the evaluation
budget / submission rules with the episode runner stubbed out."""
import asyncio
import json
import os
import shutil
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SCAFFOLD = REPO / "executor" / "planner_templates" / "baseline"
sys.path.insert(0, str(HERE))
pytest.importorskip("mcp")
import delivery  # noqa: E402


def _artifact(tmp_path) -> Path:
    dst = tmp_path / "app" / "artifact"
    shutil.copytree(SCAFFOLD, dst, symlinks=False)
    return dst


def test_manifest_problems(tmp_path):
    art = _artifact(tmp_path)
    assert delivery.check_manifest(art)[1] == []
    (art / "manifest.yaml").write_text("schema: cell.planner-delivery/1\nentrypoint: ../x.sh\ntools: []\nbudget: {sim_s: 0}\n")
    problems = delivery.check_manifest(art)[1]
    assert len(problems) == 4                      # entrypoint outside, tools, budget, states


def test_surface_rules():
    good = [{"name": "execute_task", "input_schema": {"properties": {"task_ref": {"type": "string"}}, "required": ["task_ref"]}}]
    assert delivery.surface_problems(good) == []
    assert delivery.surface_problems(good + [{"name": "x", "input_schema": {}}])
    assert delivery.surface_problems([{"name": "execute_task", "input_schema": {"properties": {"task_ref": {"type": "integer"}}}}])


@pytest.mark.skipif(not os.environ.get("AMENT_PREFIX_PATH"), reason="needs the ROS environment (run.sh sources it)")
def test_scaffold_passes_the_full_check(tmp_path):
    art = _artifact(tmp_path)
    r = asyncio.run(delivery.check_delivery(art, REPO / "executor" / "planner_templates", {**os.environ, "CELL_REPO_ROOT": str(REPO)}))
    assert r["ok"], r
    assert r["tools"] == ["execute_task"] and r["budget_sim_s"] == 300
    (art / "planner.py").write_text((art / "planner.py").read_text() + "\nPORT = 9010\n")
    assert not asyncio.run(delivery.check_delivery(art, REPO / "executor" / "planner_templates", {**os.environ, "CELL_REPO_ROOT": str(REPO)}))["ok"]


@pytest.fixture
def platform(tmp_path, monkeypatch):
    _artifact(tmp_path)
    monkeypatch.setenv("PLATFORM_SESSION_DIR", str(tmp_path / "app"))
    monkeypatch.setenv("PLATFORM_RECORD_DIR", str(tmp_path / "records"))
    monkeypatch.setenv("PLATFORM_BUDGET", "4")
    import importlib.util

    spec = importlib.util.spec_from_file_location("platform_mcp_server", HERE / "server.py")
    S = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(S)
    S._S = None

    async def ok_check(s):
        return {"ok": True, "problems": [], "budget_sim_s": 300}

    outcomes = iter(["failure", "invalid", "success", "success", "success"])

    def fake_run(s, snapshot, episode_id, seed, watchdog, sim_s):
        out = next(outcomes)
        ep = tmp_path / "records_eps" / episode_id
        return {"entry": {"episode_id": episode_id, "seed": seed, "runner_exit": 0, "wall_s": 1.0, "outcome": out, "stage": "done"},
                "verdict": {"outcome": out, "planner": {"state": "stopped", "reason": "r"}, "checker": {"parts_placed": 0, "parts_required": 4, "parts": {}}}}

    monkeypatch.setattr(S, "_check", ok_check)
    monkeypatch.setattr(S, "_run_episode", fake_run)
    monkeypatch.setattr(S, "_copy_evidence", lambda eid, dst: {"copied": []})
    return S


def test_budget_charges_completed_episodes_only(platform):
    S = platform
    r = asyncio.run(S.evaluate_planner(episodes=2))
    assert r["budget"] == {"charged": 1, "left": 3, "of": 4}                 # failure charged, invalid not
    assert [e["seed"] for e in r["episodes"]] == [1, 2] and r["evidence_dir"] == "/app/evaluations/0001"
    ev = S.session().app / "evaluations" / "0001"
    assert json.loads((ev / "evaluation.json").read_text())["evaluation"] == 1
    assert "outcome: failure" in (ev / "episode-1" / "verdict.txt").read_text()
    assert "error" in asyncio.run(S.evaluate_planner(episodes=4))           # more than left / per call
    assert asyncio.run(S.evaluate_planner(episodes=3))["budget"]["left"] == 0
    assert "budget" in asyncio.run(S.evaluate_planner(episodes=1))["error"]


def test_done_needs_the_artifact_evaluated(platform):
    S = platform
    app = S.session().app
    assert "not found" in S.submit_work("stopped", "report.md")["error"]
    (app / "report.md").write_text("notes")
    assert "UNEVALUATED" in S.submit_work("done", "/app/report.md")["error"]
    asyncio.run(S.evaluate_planner(episodes=1))                       # failure: completed and charged
    assert S.submit_work("done", "/app/report.md")["ok"]
    (S.session().artifact / "planner.py").write_text("# changed after the evaluation\n")
    assert "UNEVALUATED" in S.submit_work("done", "report.md")["error"]
    r = S.submit_work("stopped", "report.md")                         # replaces the earlier one
    assert r["ok"] and S.session().state["submitted"]["replaces"]
