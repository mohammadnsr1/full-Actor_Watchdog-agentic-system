#!/usr/bin/env bash
# Source Humble and the cell workspace, then run the given command.
set -e
source /opt/ros/humble/setup.bash
if [ -f /cell/ros2_ws/install/setup.bash ]; then
  source /cell/ros2_ws/install/setup.bash
fi
exec "$@"
