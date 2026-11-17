#!/bin/bash
# Run one episode (runner/episode_runner/run_episode.py; --help for options).
# Sources ROS and the workspace, then runs the runner with the workspace venv
# (it has the mcp client) plus the system dist-packages for rclpy.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
DISTRO="${ROS_DISTRO:-}"
if [ -z "$DISTRO" ]; then
    for d in jazzy humble; do [ -f "/opt/ros/$d/setup.bash" ] && DISTRO=$d && break; done
fi
set +u
source "/opt/ros/$DISTRO/setup.bash"
source "$REPO/ros2_ws/install/setup.bash"
unset VIRTUAL_ENV
cd "$REPO"
exec "$REPO/ros2_ws/.venv/bin/python" -c "
import site, sys
v = f'{sys.version_info.major}.{sys.version_info.minor}'
for d in (f'/usr/local/lib/python{v}/dist-packages', '/usr/lib/python3/dist-packages', f'/usr/lib/python{v}/dist-packages'):
    site.addsitedir(d)
sys.argv = ['run_episode'] + sys.argv[1:]
from runner.episode_runner.run_episode import main
sys.exit(main())
" "$@"
