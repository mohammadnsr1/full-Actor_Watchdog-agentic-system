# The executor-baseline authoring agent's container (EB4), like the labmate's:
# Claude Code with all its tools, a fresh config, /app read-write, /app/evaluations
# and /data read-only, nothing else of the repo.
#
# Sim is one Jazzy graph (docs/ARCHITECTURE.md section 9), so the image is Jazzy on
# Ubuntu 24.04 like the host: the agent may echo the cell's topics (decision 11) with
# the same distro. The repo's message package comes in as a read-only mount of
# ros2_ws/install/cell_planner_msgs (runner/run_baseline.sh), not as a copy.
#
# Build (BuildKit; the kit-mcp gateway comes from the omni-kit-mcp checkout):
#   docker build -f docker/authoring/Dockerfile \
#       --build-context kitmcp=$HOME/omni-kit-mcp -t cell-authoring .

FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ARG CLAUDE_CODE_VERSION=2.1.331
ENV LANG=C.UTF-8 ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
        curl ca-certificates git ripgrep less jq \
        python3-pip python3-venv python3-yaml python3-numpy python3-scipy python3-pil python3-opencv \
        ros-jazzy-control-msgs ros-jazzy-trajectory-msgs ros-jazzy-vision-msgs \
        ros-jazzy-moveit-msgs ros-jazzy-tf2-ros-py ros-jazzy-tf2-msgs \
    && rm -rf /var/lib/apt/lists/*

# The planner-side cell code is mounted read-only under /opt/cell (runner/run_baseline.sh):
# executor-side ROS packages, robot_backend.utils alone (the RG2 geometry; not the WatchDog's
# code), and a config subset. robot_backend gets an empty package root here.
RUN mkdir -p /opt/cell/py/robot_backend /opt/cell/repo && touch /opt/cell/py/robot_backend/__init__.py

# The agent runs as an unprivileged user with the same uid as the host user, so files
# it writes into /app belong to the host user.
ARG UID=1000
RUN (id -u ubuntu >/dev/null 2>&1 && userdel -r ubuntu || true) && useradd -m -u ${UID} -s /bin/bash agent
USER agent
WORKDIR /home/agent

# Claude Code, pinned (native installer, as on the host).
RUN curl -fsSL https://claude.ai/install.sh | bash -s ${CLAUDE_CODE_VERSION}
ENV PATH=/home/agent/.local/bin:$PATH

# The kit-mcp gateway (MCP over stdio -> the Kit bridge on 127.0.0.1:9009) in its own venv.
COPY --from=kitmcp --chown=agent pyproject.toml README.md /home/agent/omni-kit-mcp/
COPY --from=kitmcp --chown=agent kit_mcp /home/agent/omni-kit-mcp/kit_mcp
RUN python3 -m venv /home/agent/kitmcp-venv \
    && /home/agent/kitmcp-venv/bin/pip install --no-cache-dir /home/agent/omni-kit-mcp \
    && ln -s /home/agent/kitmcp-venv/bin/kit-mcp /home/agent/.local/bin/kit-mcp

# Python for the agent's own scripts and planner tests: the system's (rclpy) plus mcp.
RUN python3 -m venv --system-site-packages /home/agent/cell-venv \
    && /home/agent/cell-venv/bin/pip install --no-cache-dir mcp==1.30.0 trimesh==5.2.0
ENV PATH=/home/agent/cell-venv/bin:$PATH

COPY --chown=agent docker/authoring/entrypoint.sh /home/agent/entrypoint.sh
ENTRYPOINT ["/home/agent/entrypoint.sh"]
