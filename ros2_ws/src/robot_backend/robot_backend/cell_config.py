"""Read config/cell.yaml for robot_backend processes.

Mirrors cell_planning_service.cell_config without importing it (robot_backend
must not depend on the planning service). Resolution: CELL_CONFIG_PATH, else
walk up from this file to the directory holding config/cell.yaml and
ros2_ws/. A missing file yields an empty mapping; callers apply their own
explicit defaults.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

ENV_VAR = "CELL_CONFIG_PATH"
RELATIVE_PATH = Path("config") / "cell.yaml"


def find_cell_config() -> Path | None:
    env = os.environ.get(ENV_VAR, "").strip()
    if env:
        return Path(os.path.expanduser(env))
    origin = Path(__file__).resolve()
    for parent in [origin, *origin.parents]:
        if (parent / RELATIVE_PATH).is_file() and (parent / "ros2_ws").is_dir():
            return parent / RELATIVE_PATH
    return None


def load_cell_config() -> tuple[dict[str, Any], Path | None]:
    import yaml

    path = find_cell_config()
    if path is None or not path.is_file():
        return {}, path
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a mapping at the top level")
    return data, path


def section(config: dict[str, Any], *keys: str) -> dict[str, Any]:
    node: Any = config
    for key in keys:
        node = (node or {}).get(key, {}) if isinstance(node, dict) else {}
    return node if isinstance(node, dict) else {}
