"""Locate and read config/cell.yaml.

Resolution order:
    1. CELL_CONFIG_PATH environment variable, if set.
    2. The `cell_config_path` value handed in by the caller (a ROS parameter).
    3. Walk up from this file looking for a directory that contains both
       `config/cell.yaml` and `ros2_ws/`, which is the integration repo root.
       This also works from the installed copy under ros2_ws/install because
       the walk passes through ros2_ws on its way up.

No value is invented here: when the file is missing, the caller gets an
empty mapping and must apply its own defaults explicitly.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml

ENV_VAR = "CELL_CONFIG_PATH"
RELATIVE_PATH = Path("config") / "cell.yaml"


def repo_root(start: Path | None = None) -> Path | None:
    """Return the integration repo root (contains config/cell.yaml and ros2_ws/)."""
    origin = (start or Path(__file__)).resolve()
    for parent in [origin, *origin.parents]:
        if (parent / RELATIVE_PATH).is_file() and (parent / "ros2_ws").is_dir():
            return parent
    return None


def find_cell_config(explicit: str | None = None) -> Path | None:
    env = os.environ.get(ENV_VAR, "").strip()
    if env:
        return Path(os.path.expanduser(env))
    if explicit and explicit.strip():
        return Path(os.path.expanduser(explicit.strip()))
    root = repo_root()
    return (root / RELATIVE_PATH) if root is not None else None


def load_cell_config(explicit: str | None = None) -> tuple[dict[str, Any], Path | None]:
    """Return (config mapping, path used). Missing file -> ({}, path_or_None)."""
    path = find_cell_config(explicit)
    if path is None or not path.is_file():
        return {}, path
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a mapping at the top level")
    return data, path


def planner_section(config: dict[str, Any]) -> dict[str, Any]:
    section = config.get("planner", {}) or {}
    if not isinstance(section, dict):
        raise ValueError("config/cell.yaml: `planner` must be a mapping")
    return section
