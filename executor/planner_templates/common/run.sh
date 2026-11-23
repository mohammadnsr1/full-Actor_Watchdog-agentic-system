#!/bin/bash
# Start the planner's stdio MCP server (docs/PLANNER_CONTRACT.md).
# stdout is the MCP channel: everything else goes to stderr.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${CELL_REPO_ROOT:-$(cd "$HERE/../.." && pwd)}"
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
