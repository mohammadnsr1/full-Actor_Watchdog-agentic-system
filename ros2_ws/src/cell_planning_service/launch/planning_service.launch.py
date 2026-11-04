"""Start the planning service with the backend named in config/cell.yaml.

planner.backend: moveit  -> cell_planning_service planning_service_node
planner.backend: curobo  -> cell_planning_curobo curobo_planning_node

Arguments:
    cell_config_path  path to cell.yaml (default: CELL_CONFIG_PATH or repo walk-up)
    use_sim_time      forwarded to the node
"""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from cell_planning_service.cell_config import load_cell_config, planner_section


def _setup(context):
    explicit = LaunchConfiguration("cell_config_path").perform(context).strip()
    use_sim_time = LaunchConfiguration("use_sim_time")
    config, path = load_cell_config(explicit or None)
    backend = str(planner_section(config).get("backend", "")).strip().lower() if config else ""
    params = [{"use_sim_time": use_sim_time, "cell_config_path": explicit}]
    env = dict(os.environ)
    if explicit:
        env["CELL_CONFIG_PATH"] = explicit
    if backend == "moveit":
        node = Node(
            package="cell_planning_service",
            executable="planning_service_node",
            name="cell_planning_service",
            output="screen",
            parameters=params,
            additional_env=env,
        )
    elif backend == "curobo":
        node = Node(
            package="cell_planning_curobo",
            executable="curobo_planning_node",
            name="cell_planning_service",
            output="screen",
            parameters=params,
            additional_env=env,
        )
    else:
        return [LogInfo(msg=f"planning service NOT started: planner.backend={backend!r} in {path}")]
    return [LogInfo(msg=f"planning service backend={backend} (from {path})"), node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("cell_config_path", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
