"""What a planner may not reference (docs/PLANNER_CONTRACT.md section 4), shared by the
M7 authoring MCP (executor/authoring_mcp) and the baseline platform MCP (executor/platform_mcp).

A planner reaches the cell only through the contract's channels. Text that would reach
around them (the simulator's command server, the Kit bridge and its lab verbs, the stage
file, the WatchDog's camera2 and part data, the placements, the requirements, the
registry) makes evaluation refuse the planner before any episode. Lines identical to a
line of the shipped templates are not judged: the lint judges what the agent wrote.
"""

from __future__ import annotations

from pathlib import Path

FORBIDDEN_IN_PLANNER = (
    "cell_command_server", "CommandServerClient", "report_poses", "contacts_poll", "9010",
    "9009", "kit_mcp", "omni_kit_mcp", "lab_tools",
    "backends.scene", "watchdog_poses", "camera2", "assembly_room", ".usd", "pxr",
    "config/placements", "placements.yaml", "watchdog_parts", "config/requirements", "model_registry",
)
SUFFIXES = (".py", ".sh", ".yaml", ".yml", ".json", ".txt", ".cfg", ".toml")


def template_lines(templates: Path) -> set[str]:
    lines: set[str] = set()
    for path in templates.rglob("*"):
        if path.is_file() and not path.is_symlink() and "__pycache__" not in path.parts:
            lines.update(line.strip() for line in path.read_text(errors="replace").splitlines())
    return lines


def lint_planner(folder: Path, templates: Path) -> list[str]:
    """Forbidden references added to the planner: ["<file>:<line>: ..."]."""
    shipped = template_lines(templates)
    hits = []
    for path in sorted(Path(folder).rglob("*")):
        if not path.is_file() or "__pycache__" in path.parts or path.suffix not in SUFFIXES:
            continue
        for n, line in enumerate(path.read_text(errors="replace").splitlines(), 1):
            if line.strip() in shipped:
                continue
            for word in FORBIDDEN_IN_PLANNER:
                if word in line:
                    hits.append(f"{path.relative_to(folder)}:{n}: {word!r} in {line.strip()[:120]}")
    return hits
