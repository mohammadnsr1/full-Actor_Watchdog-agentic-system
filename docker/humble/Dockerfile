# Humble build of the shared cell packages plus the hardware backend.
#
# Runs on the workstation with host networking (compose.yaml) so WatchDog, the
# camera2 pipeline and whatever else Mohammad places on the workstation share
# one Humble graph with the UR AI Accelerator. Nothing from the sim platform
# is built here (see SKIP_PACKAGES). Python is the image's 3.10.
#
# Build from the repo root:
#   docker build -f docker/humble/Dockerfile -t cell-humble .
# or with compose: docker compose -f docker/humble/compose.yaml build

FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 ROS_DISTRO=humble

# ROS packages the shared code imports. moveit_msgs is needed for the message
# definitions only; move_group itself is not installed here (MoveIt on
# hardware is backup only, BUILD_PLAN q8).
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-venv python3-colcon-common-extensions python3-yaml python3-numpy \
        ros-humble-action-msgs \
        ros-humble-control-msgs \
        ros-humble-trajectory-msgs \
        ros-humble-sensor-msgs \
        ros-humble-geometry-msgs \
        ros-humble-shape-msgs \
        ros-humble-std-srvs \
        ros-humble-vision-msgs \
        ros-humble-moveit-msgs \
        ros-humble-tf2-ros \
        ros-humble-tf2-ros-py \
        ros-humble-rosgraph-msgs \
        ros-humble-rosbag2 \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-launch ros-humble-launch-ros \
    && rm -rf /var/lib/apt/lists/*

# TODO(mohammad): add the UR driver message packages once the branch is known
# (ros-humble-ur-dashboard-msgs, ros-humble-ur-msgs) and the Orbbec driver if
# camera2 runs inside this container.

WORKDIR /cell

# Packages that exist only on the sim platform. The tree is copied whole so
# the workspace layout matches the native Jazzy build; these are skipped.
ENV SKIP_PACKAGES="isaac_trajectory_bridge sim_moveit_bringup ur5e_moveit_config cell_backend_sim"

COPY ros2_ws/src /cell/ros2_ws/src
COPY ros2_ws/requirements.txt /cell/ros2_ws/requirements.txt
COPY config /cell/config
COPY agents /cell/agents

# The MCP servers re-exec into ros2_ws/.venv (robot_backend/mcp_servers/__init__.py).
# Same pins as the Jazzy workspace, built with this image's Python 3.10.
RUN python3 -m venv /cell/ros2_ws/.venv \
    && /cell/ros2_ws/.venv/bin/pip install --no-cache-dir --upgrade pip \
    && /cell/ros2_ws/.venv/bin/pip install --no-cache-dir -r /cell/ros2_ws/requirements.txt

RUN . /opt/ros/humble/setup.sh \
    && cd /cell/ros2_ws \
    && colcon build --symlink-install --packages-skip ${SKIP_PACKAGES} \
       --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# Platform selection for every process in the container.
ENV CELL_PLATFORM=hardware
ENV CELL_REPO_ROOT=/cell
ENV CELL_CONFIG_PATH=/cell/config/cell.yaml

COPY docker/humble/entrypoint.sh /cell/entrypoint.sh
RUN chmod +x /cell/entrypoint.sh
ENTRYPOINT ["/cell/entrypoint.sh"]
CMD ["bash"]
