"""Planner template: a stdio MCP server with exactly one tool, execute_task.

Started by run.sh. stdout is the MCP channel: logging goes to
$CELL_EPISODE_DIR/executor/planner.jsonl (stderr outside an episode). See docs/PLANNER_CONTRACT.md.
"""

from __future__ import annotations

import logging
import os
import site
import sys
import traceback
from pathlib import Path
from typing import Any

# Not resolved: in a template directory server.py is a symlink to ../common, and
# planner.py must be imported from the planner's own directory.
HERE = Path(os.path.abspath(__file__)).parent


def _append_system_dist_packages() -> None:
    """rclpy and the workspace live in the system Python's dist-packages; the venv
    (which has mcp) comes first. Same approach as the robot_backend MCP servers."""
    version = f"{sys.version_info.major}.{sys.version_info.minor}"
    for candidate in (f"/usr/local/lib/python{version}/dist-packages", "/usr/lib/python3/dist-packages", f"/usr/lib/python{version}/dist-packages"):
        if Path(candidate).is_dir() and candidate not in sys.path:
            site.addsitedir(candidate)


class _JsonLines(logging.Formatter):
    """One JSON object per record: wall_time_iso, level, logger, msg (+ exc)."""

    def format(self, record: logging.LogRecord) -> str:
        import json
        from datetime import datetime, timezone

        out = {"wall_time_iso": datetime.fromtimestamp(record.created, timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z",
               "level": record.levelname, "logger": record.name, "msg": record.getMessage()}
        if record.exc_info:
            out["exc"] = self.formatException(record.exc_info)
        return json.dumps(out, default=str)


def _setup_logging() -> None:
    """In an episode: $CELL_EPISODE_DIR/executor/planner.jsonl (the runner keeps
    the process's raw stderr separately). Outside one: plain stderr."""
    episode = os.environ.get("CELL_EPISODE_DIR")
    if episode:
        out = Path(episode) / "executor"
        out.mkdir(parents=True, exist_ok=True)
        handler: logging.Handler = logging.FileHandler(out / "planner.jsonl")
        handler.setFormatter(_JsonLines())
    else:
        handler = logging.StreamHandler(sys.stderr)
        handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logging.basicConfig(level=logging.INFO, handlers=[handler])


_append_system_dist_packages()
sys.path.insert(0, str(HERE))
_setup_logging()

import anyio  # noqa: E402
from mcp.server.fastmcp import FastMCP  # noqa: E402

mcp = FastMCP("planner")


@mcp.tool()
async def execute_task(task_ref: str) -> dict[str, Any]:
    """Execute the task in task_ref once. Returns {state: done|stopped, reason}."""

    def _run() -> dict[str, Any]:
        from planner import run_task

        try:
            return run_task(task_ref)
        except Exception as exc:  # noqa: BLE001
            logging.getLogger("planner").error("planner error:\n%s", traceback.format_exc())
            return {"state": "stopped", "reason": f"planner error: {type(exc).__name__}: {exc}"}

    return await anyio.to_thread.run_sync(_run)


if __name__ == "__main__":
    mcp.run()  # stdio
