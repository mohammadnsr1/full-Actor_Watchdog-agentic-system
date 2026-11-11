# Platforms

Written 2026-11-10 (Milestone 3, v2). One codebase, two platforms, one ROS 2
distro per graph.

## What runs where

| Process | Sim platform | Hardware platform |
|---|---|---|
| Simulator / robot | Isaac Sim 5.1 (Kit Python 3.11), GUI or headless (M6), on the workstation | UR7e through the UR ROS 2 driver on the UR AI Accelerator (Jetson Orin, Ubuntu 22, Humble) |
| Motion server | `isaac_trajectory_bridge` FollowJointTrajectory, Jazzy native | UR driver trajectory controller on the accelerator |
| Gripper | RG2 controller node (sim) | OnRobot RG6 through the URCap, URScript over the driver's script topic (no ROS 2 readings yet) |
| camera1 (wrist) pose pipeline | none yet (camera1 disabled in `config/cameras/sim.yaml`) | FoundationPose on the Orin (not built yet), `/camera1/poses` |
| camera2 (table) pose pipeline | `cell_backend_sim ground_truth_pose_publisher` (Isaac TF to `/camera2/poses`) | FoundationPose on the workstation (not built yet), `/camera2/poses` |
| Planning service | Jazzy native, MoveIt backend default | accelerator's own planner first (UR AI Accelerator SDK), cuRobo node second, MoveIt backup; placement `TODO(mohammad)` |
| WatchDog MCP server | Jazzy native, `config/mcp/watchdog.mcp.json` | Humble container on the workstation, `config/mcp/watchdog.hardware.mcp.json` |
| Physical events (M5), recorder, episode runner (M7) | Jazzy native | container or accelerator, decided in M7 (Mohammad's answer 7) |

Mohammad's answer 7 (2026-11-10): the Humble container holds WatchDog and the
camera2 pipeline; everything else runs on the accelerator.

## The single-distro rule

Every ROS graph runs one distro. Sim is Jazzy end to end on the workstation.
Hardware is Humble end to end: the accelerator is Humble, and the
workstation-side hardware nodes run inside `docker/humble/` with host
networking so they join the accelerator's DDS domain as Humble nodes. A Jazzy
node is never started in that domain, and the container is never started
while the sim graph is up on the same host with the same `ROS_DOMAIN_ID`.

Platform selection is `config/cell.yaml: platform` or `CELL_PLATFORM`. The
container sets `CELL_PLATFORM=hardware`, `CELL_REPO_ROOT=/cell`,
`CELL_CONFIG_PATH=/cell/config/cell.yaml`.

## Baseline

Shared packages (everything except `isaac_trajectory_bridge`,
`sim_moveit_bringup`, `ur5e_moveit_config`, `cell_backend_sim`, and the
`isaac/` tree) run on Python 3.10 with Humble and on Python 3.12 with Jazzy.
Not allowed in shared code: the `type` statement, PEP 695 generics,
`ExceptionGroup`/`except*`, `typing.Self`, `tomllib`, `StrEnum`,
`datetime.UTC`, `asyncio.TaskGroup`/`asyncio.timeout`, `NotRequired`,
`hashlib.file_digest`, `itertools.batched`, `typing.override`, 3.12 f-string
quoting, and `datetime.fromisoformat` on strings ending in `Z` (strip or
replace the `Z` first; `cell_platform_interfaces.timebase.parse_wall_time_iso`
does). The audit in `docs/BUILD_PLAN.md` section 5.2 found none of these in
the copied code; new code is written to the same rule.

## Humble and Jazzy differences found

All isolated in `cell_platform_interfaces/compat.py` unless noted.

| Difference | Where it bites | Handling |
|---|---|---|
| rosbag2 default storage: sqlite3 on Humble, mcap on Jazzy | the session bag (`sim_moveit_bringup.launch.py`), the per-episode bag (M6) | `compat.bag_record_command` always passes `-s mcap`; the Dockerfile installs `ros-humble-rosbag2-storage-mcap`. |
| `moveit_msgs/MotionPlanRequest.pipeline_id` | `cell_planning_service/moveit_backend.py` | Set directly today; if the Humble build reports the field missing, switch that one line to `compat.set_if_present`. Verify with `ros2 interface show moveit_msgs/msg/MotionPlanRequest` in the container. |
| `MoveItErrorCodes` values added after Humble | `cell_planning_service/failure_codes.py` | Integer map; unknown codes fall back to `PLANNING_FAILED` with the integer in the reason. |
| MoveIt config format (`planning_plugins` list and `default_planning_request_adapters/*` names are MoveIt 2.10) | `ur5e_moveit_config/config/ompl_planning.yaml` | Sim-only package. A hardware MoveIt config, if ever wanted, is a separate Humble-format package built from the driver's `ur_moveit_config`. |
| `rclpy.wait_for_message` (Iron+), `Node.create_timer(autostart=)` (Jazzy) | nowhere | Forbidden in shared code; listed in `compat.py`. |
| Python of the workspace venv | `ros2_ws/.venv` re-exec in `robot_backend/mcp_servers/__init__.py` | The Dockerfile creates the venv with the image's 3.10 from the same `requirements.txt`. |
| `vision_msgs/Detection3DArray` | pose topics | vision_msgs 4.1.x on both distros; only `id`, `results[].hypothesis.class_id/score`, `results[].pose.pose`, `header` are used. |
| `use_sim_time` | session manifest, every node | A platform value (`use_sim_time` in `config/platforms/<platform>.yaml`), not a literal. |

## Where names live

| Name kind | File |
|---|---|
| topics, frames, joints, motion action and cancel, gripper services, cell services, pose topics, signals | `config/platforms/{sim,hardware}.yaml` |
| camera roles, topics, extrinsics | `config/cameras/{sim,hardware}.yaml` |
| gripper models (WatchDog-only) | `config/gripper/{rg2,rg6}_kinematics.yaml` |
| planner defaults, modes, budgets, recording | `config/cell.yaml` |

Code reads them through `cell_platform_interfaces.platform_config` (strict:
a missing value raises naming the key) or, inside `robot_backend`, through
`robot_backend.cell_config`. The launch files read the same files at
launch-description time. `ros2 run cell_platform_interfaces show_platform
sim` prints what a process will see.

## Building

Sim, native Jazzy:

```bash
cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash
```

Hardware, Humble container (workstation):

```bash
docker compose -f docker/humble/compose.yaml build
docker compose -f docker/humble/compose.yaml run --rm cell bash
# inside: ros2 run cell_platform_interfaces show_platform hardware
```

Accelerator (native Humble): clone the repo, build the same package set with
`colcon build --packages-skip isaac_trajectory_bridge sim_moveit_bringup
ur5e_moveit_config cell_backend_sim`, and set `CELL_PLATFORM=hardware`.
`TODO(mohammad)`: confirm the accelerator's Python is 3.10 and which packages
from the UR AI Accelerator SDK provide the planner.
