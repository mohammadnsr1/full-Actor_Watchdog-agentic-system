#!/usr/bin/env bash
# Container entrypoint (EB4): ROS Jazzy + the mounted message package, a fresh Claude
# config, then claude --print with the prompt on stdin (the labmate's launch line).
set -eo pipefail
source /opt/ros/jazzy/setup.bash
# Each mounted install prefix (runner/run_baseline.sh): ament index, Python, libraries.
for d in /opt/cell/*/; do
    d="${d%/}"
    [ -d "$d/share/ament_index" ] || continue
    export AMENT_PREFIX_PATH="$d${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    for sp in "$d"/lib/python3*/site-packages; do [ -d "$sp" ] && export PYTHONPATH="$sp${PYTHONPATH:+:$PYTHONPATH}"; done
    [ -d "$d/lib" ] && export LD_LIBRARY_PATH="$d/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
done
export PYTHONPATH="/opt/cell/py${PYTHONPATH:+:$PYTHONPATH}"
export CELL_REPO_ROOT=/opt/cell/repo CELL_PLATFORM=sim
export CLAUDE_CONFIG_DIR=/home/agent/.claude-session     # empty: no CLAUDE.md, skills or memory
mkdir -p "$CLAUDE_CONFIG_DIR"
cd /app
if [ "${1:-}" = "shell" ]; then exec bash; fi
exec claude --print --model "${MODEL:?MODEL is required}" \
    --mcp-config /session/mcp.json --strict-mcp-config \
    --output-format stream-json --verbose \
    --dangerously-skip-permissions < /session/prompt.md
