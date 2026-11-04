"""Re-exec into the cuRobo Python environment.

ROS launches executables with the system interpreter, which has rclpy but
not cuRobo. The node re-execs into `planner.curobo.python` from
config/cell.yaml (or CELL_CUROBO_PYTHON), then appends the system
dist-packages so rclpy and the message packages stay importable. Same
pattern robot_backend uses for its MCP servers.
"""

from __future__ import annotations

import os
import sys
import sysconfig
from pathlib import Path

_MARK = "_CELL_CUROBO_ENV_READY"
ENV_PYTHON = "CELL_CUROBO_PYTHON"


def ensure_curobo_environment(configured_python: str) -> None:
    """Exec into the cuRobo interpreter once; no-op if already there."""
    if os.environ.get(_MARK) == "1":
        _append_system_site_packages()
        return
    target = os.environ.get(ENV_PYTHON, "").strip() or (configured_python or "").strip()
    if not target:
        raise RuntimeError(
            "no cuRobo interpreter configured: set planner.curobo.python in config/cell.yaml "
            "or CELL_CUROBO_PYTHON (see cell_planning_curobo/INSTALL.md)"
        )
    python = Path(os.path.expanduser(target))
    if not python.is_file():
        raise RuntimeError(f"cuRobo interpreter not found: {python}")
    if Path(sys.executable).resolve() == python.resolve():
        os.environ[_MARK] = "1"
        _append_system_site_packages()
        return
    env = dict(os.environ)
    env[_MARK] = "1"
    env["_CELL_CUROBO_SYSTEM_SITE"] = sysconfig.get_paths()["purelib"]
    os.execve(str(python), [str(python), *sys.argv], env)


def _append_system_site_packages() -> None:
    site = os.environ.get("_CELL_CUROBO_SYSTEM_SITE", "").strip()
    if site and site not in sys.path:
        sys.path.append(site)
    # Also the ROS overlay paths already present in PYTHONPATH are kept by execve.
