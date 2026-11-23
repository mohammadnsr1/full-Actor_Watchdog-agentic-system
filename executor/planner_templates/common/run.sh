#!/bin/bash
# Start the planner's stdio MCP server (docs/PLANNER_CONTRACT.md).
# stdout is the MCP channel: everything else goes to stderr.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# HERE is the planner directory as invoked (not symlink-resolved). The repo root
# is the first ancestor holding config/cell.yaml unless the runner set it.
if [ -z "${CELL_REPO_ROOT:-}" ]; then
    d="$HERE"; while [ "$d" != "/" ] && [ ! -f "$d/config/cell.yaml" ]; do d="$(dirname "$d")"; done
    CELL_REPO_ROOT="$d"
fi
REPO="$CELL_REPO_ROOT"
export CELL_REPO_ROOT="$REPO"
DISTRO="${ROS_DISTRO:-}"
if [ -z "$DISTRO" ]; then
    for d in jazzy humble; do [ -f "/opt/ros/$d/setup.bash" ] && DISTRO=$d && break; done
fi
set +u
source "/opt/ros/$DISTRO/setup.bash" >&2
source "$REPO/ros2_ws/install/setup.bash" >&2
unset VIRTUAL_ENV
exec "$REPO/ros2_ws/.venv/bin/python" "$HERE/server.py"
