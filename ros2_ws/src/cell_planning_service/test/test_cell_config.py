from __future__ import annotations

import os
from pathlib import Path

from cell_planning_service.cell_config import find_cell_config, load_cell_config, planner_section


def test_env_override_wins(tmp_path, monkeypatch):
    target = tmp_path / "cell.yaml"
    target.write_text("planner:\n  backend: moveit\n", encoding="utf-8")
    monkeypatch.setenv("CELL_CONFIG_PATH", str(target))
    assert find_cell_config("/nope.yaml") == target
    config, path = load_cell_config()
    assert path == target
    assert planner_section(config)["backend"] == "moveit"


def test_explicit_path_used_without_env(tmp_path, monkeypatch):
    monkeypatch.delenv("CELL_CONFIG_PATH", raising=False)
    target = tmp_path / "cell.yaml"
    target.write_text("planner: {backend: curobo}\n", encoding="utf-8")
    config, path = load_cell_config(str(target))
    assert path == target and planner_section(config)["backend"] == "curobo"


def test_missing_file_returns_empty(tmp_path, monkeypatch):
    monkeypatch.delenv("CELL_CONFIG_PATH", raising=False)
    config, path = load_cell_config(str(tmp_path / "absent.yaml"))
    assert config == {} and path == Path(tmp_path / "absent.yaml")


def test_repo_walkup_finds_integration_config(monkeypatch):
    monkeypatch.delenv("CELL_CONFIG_PATH", raising=False)
    config, path = load_cell_config()
    # Runs from the source tree or the install tree; both are inside the repo.
    assert path is not None and path.name == "cell.yaml"
    assert os.path.isfile(path)
    assert planner_section(config).get("backend") in {"moveit", "curobo"}
