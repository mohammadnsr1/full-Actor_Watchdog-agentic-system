#!/bin/bash
# Headless Isaac with the same ROS 2 environment the GUI gets.
#
#   isaac/headless/run_headless.sh [--gui] [--no-play] [--stage PATH]
#
# isaac-sim.sh (the GUI launcher) sources $ISAACSIM_ROOT/setup_ros_env.sh
# before starting Kit; python.sh does not, so a bare
# `python.sh run_headless.py` loads the bridge's internal Jazzy without its
# libraries on LD_LIBRARY_PATH and the bridge fails ("libament_index_cpp.so:
# cannot open shared object file"). LD_LIBRARY_PATH must be set before the
# process starts, hence this wrapper rather than a fix inside Python.
#
# Run it from a shell WITHOUT /opt/ros/*/setup.bash sourced, like the GUI:
# setup_ros_env.sh only adds the internal libraries when ROS_DISTRO is unset.
set -e
ISAACSIM_ROOT="${ISAACSIM_ROOT:-$HOME/isaacsim}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -n "$ROS_DISTRO" ]; then
    echo "[run_headless.sh] note: ROS_DISTRO=$ROS_DISTRO is set (system ROS sourced?);" \
         "Isaac will use that ROS instead of its internal Jazzy, unlike the GUI." >&2
fi
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo "[run_headless.sh] warning: ROS_DOMAIN_ID is unset; the cell uses the domain from your shell (42 in ~/.bashrc)." >&2
fi

source "$ISAACSIM_ROOT/setup_ros_env.sh"
echo "[run_headless.sh] ROS_DISTRO=$ROS_DISTRO RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
exec "$ISAACSIM_ROOT/python.sh" "$HERE/run_headless.py" "$@"
