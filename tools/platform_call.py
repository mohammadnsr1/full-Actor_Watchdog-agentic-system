#!/usr/bin/env python3
"""Call one platform MCP tool from the shell (EB6 checks without the container or an agent).

    source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash && source ros2_ws/.venv/bin/activate
    python tools/platform_call.py check_delivery
    python tools/platform_call.py robot_health
    python tools/platform_call.py move_robot '{"tool_delta": {"position": {"z": -0.02}}}'
    python tools/platform_call.py evaluate_planner '{"episodes": 1}'

Uses a scratch session in executor/workspace/baseline/_check/ (artifact = the baseline
scaffold, task data staged fresh); bookkeeping in records/authoring/_check/.
"""
import asyncio
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


async def main(tool: str, args: dict) -> None:
    from mcp import ClientSession, StdioServerParameters
    from mcp.client.stdio import stdio_client

    root = REPO / "executor" / "workspace" / "baseline" / "_check"
    if not (root / "app" / "artifact").is_dir():
        (root / "app").mkdir(parents=True, exist_ok=True)
        shutil.copytree(REPO / "executor/planner_templates/baseline", root / "app" / "artifact", symlinks=False)
    subprocess.run([sys.executable, str(REPO / "tools/stage_task_data.py"), str(root / "data")], check=True, stdout=subprocess.DEVNULL)
    env = {**os.environ, "PLATFORM_SESSION_DIR": str(root / "app"), "PLATFORM_RECORD_DIR": str(REPO / "records/authoring/_check"),
           "PLATFORM_TASK": str(root / "data/tasks/fmb_assembly_2/task.yaml"), "CELL_REPO_ROOT": str(REPO)}
    params = StdioServerParameters(command=sys.executable, args=[str(REPO / "executor/platform_mcp/server.py")], env=env)
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as client:
            await client.initialize()
            result = await client.call_tool(tool, args)
    for c in result.content:
        print(getattr(c, "text", c))


if __name__ == "__main__":
    asyncio.run(main(sys.argv[1], json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}))
