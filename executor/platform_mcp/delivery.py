"""check_delivery (EB6): is `artifact/` a planner the runner can run?

    manifest.yaml   schema cell.planner-delivery/1, entrypoint (executable, inside the
                    artifact), tools [{name: execute_task, input: {task_ref: string}}],
                    budget.sim_s > 0, admissible_states within {done, stopped}
    lint            no reference around the contract (executor/planner_lint.py)
    tool surface    the entrypoint starts a stdio MCP server whose tools are exactly
                    the manifest's, execute_task taking one required string task_ref

The server is only started and asked for its tools; execute_task is never called,
so the check needs no cell.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import planner_lint  # noqa: E402

SCHEMA = "cell.planner-delivery/1"
TOOLS = {"execute_task": {"task_ref": "string"}}


def check_manifest(artifact: Path) -> tuple[dict[str, Any], list[str]]:
    path = artifact / "manifest.yaml"
    if not path.is_file():
        return {}, ["artifact/manifest.yaml is missing"]
    try:
        m = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError as exc:
        return {}, [f"manifest.yaml does not parse: {exc}"]
    problems = []
    if m.get("schema") != SCHEMA:
        problems.append(f"manifest schema must be {SCHEMA!r}")
    entry = artifact / str(m.get("entrypoint", ""))
    if not m.get("entrypoint") or not entry.resolve().is_relative_to(artifact.resolve()):
        problems.append("manifest entrypoint must name a file inside the artifact")
    elif not entry.is_file() or not os.access(entry, os.X_OK):
        problems.append(f"entrypoint {m.get('entrypoint')} is missing or not executable")
    tools = {str(t.get("name")): dict(t.get("input") or {}) for t in (m.get("tools") or []) if isinstance(t, dict)}
    if tools != TOOLS:
        problems.append(f"manifest tools must be exactly {TOOLS}, got {tools}")
    sim_s = (m.get("budget") or {}).get("sim_s")
    if not isinstance(sim_s, (int, float)) or sim_s <= 0:
        problems.append("manifest budget.sim_s must be a positive number")
    states = set(m.get("admissible_states") or [])
    if not states or not states <= {"done", "stopped"}:
        problems.append("manifest admissible_states must be a non-empty subset of [done, stopped]")
    return m, problems


async def tool_surface(artifact: Path, entrypoint: str, env: dict[str, str], timeout_s: float = 60.0) -> list[dict[str, Any]]:
    import anyio
    from mcp import ClientSession, StdioServerParameters
    from mcp.client.stdio import stdio_client

    params = StdioServerParameters(command=str(artifact / entrypoint), args=[], env=env, cwd=str(artifact))
    with anyio.fail_after(timeout_s):
        async with stdio_client(params) as (read, write):
            async with ClientSession(read, write) as client:
                await client.initialize()
                listed = await client.list_tools()
    return [{"name": t.name, "input_schema": t.inputSchema} for t in listed.tools]


def surface_problems(tools: list[dict[str, Any]]) -> list[str]:
    names = sorted(t["name"] for t in tools)
    if names != ["execute_task"]:
        return [f"the server must expose exactly execute_task, it exposes {names}"]
    schema = tools[0]["input_schema"] or {}
    props = schema.get("properties") or {}
    if (props.get("task_ref") or {}).get("type") != "string" or set(props) != {"task_ref"} or "task_ref" not in (schema.get("required") or []):
        return [f"execute_task must take exactly one required string task_ref, schema is {schema}"]
    return []


async def check_delivery(artifact: Path, templates: Path, env: dict[str, str]) -> dict[str, Any]:
    manifest, problems = check_manifest(artifact)
    lint = planner_lint.lint_planner(artifact, templates)
    if lint:
        problems.append("the planner references what it may not use (docs/PLANNER_CONTRACT.md section 4)")
    tools: list[dict[str, Any]] = []
    if not problems:
        try:
            tools = await tool_surface(artifact, str(manifest["entrypoint"]), env)
            problems += surface_problems(tools)
        except BaseException as exc:  # noqa: BLE001 - a broken server is a delivery problem, never a crash here
            while isinstance(exc, BaseExceptionGroup) and exc.exceptions:
                exc = exc.exceptions[0]
            problems.append(f"the server did not start and list its tools: {type(exc).__name__}: {exc}")
    return {"ok": not problems, "problems": problems, "lint": lint[:40], "tools": [t["name"] for t in tools],
            "budget_sim_s": (manifest.get("budget") or {}).get("sim_s")}
