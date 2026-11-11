# Provenance

## WATCHDOG copy (Milestone 2, 2026-11-03)

Source repository: `/home/mohammad/watchdog/WatchDog_V1`, branch `main`, commit `595ede3354dc40c916b4bcd464ebb2db1b014b32` (read as text from `.git/HEAD` and `.git/refs/heads/main`; approved by Mohammad as "the latest commit"). The survey's `cb973bbf…` is three commits older; the difference is paper-media runs and a re-saved `assets/usd/assembly_room.usd`.

Copy method: `rsync -a` / `cp`, excluding `__pycache__`, `*.pyc`, `.pytest_cache`, `*.egg-info`. No file in the source repository was modified.

### Files edited after copying (the only edits in Milestone 2)

| INTEGRATION path | Source path | Edit |
|---|---|---|
| `config/mcp/watchdog.mcp.json` | `watchdog_2_s3.mcp.json` | Three absolute paths rewritten to this repo: `install/setup.bash`, `WATCHDOG_TASK_PATH`, `WATCHDOG_AGENT_CONFIG`. |
| `agents/config/watchdog_agent_s3.yaml` | `agents/config/watchdog_agent_s3.yaml` | `mcp_servers[].cwd` rewritten to this repo (used only by the API path). |
| `runner/run_watchdog.sh` | `run_watchdog.sh` | Runs from the repo root; config and settings paths point at `config/mcp/` and `config/settings/`; only `s3 2` accepted. Launch line otherwise unchanged. |
| `.gitignore` | `.gitignore` (INTEGRATION's own) | Appended `ros2_ws/{build,install,log}/`, `executor/workspace/`, `records/`, `*.mcap`, `*.jpg`, `*.mp4`. |

### Relocated without edits

| INTEGRATION path | Source path |
|---|---|
| `config/settings/watchdog.settings.json` | `watchdog.settings.json` |
| `config/robot/*` | `config/*` |
| `runner/recorder/frame_recorder.py` | `paper_media/recorder/frame_recorder.py` |
| `runner/supervisor/preflight_check.sh` | `scripts/preflight_check.sh` |

### Copied at the same relative path, byte for byte

`ros2_ws/src/**` (10 packages), `ros2_ws/BUILD_NOTES.md`, `ros2_ws/colcon_defaults.yaml`, `ros2_ws/requirements.txt`, `agents/agent_harness/**`, `agents/prompts/watchdog_system_s3.txt`, `agents/prompts/watchdog_task.txt`, `agents/tasks/fmb_assembly_2.watchdog.yaml`, `assets/**`, `generated_urdf/**`, `.mcp.json`.

The full file list follows (generated from the INTEGRATION tree after the copy; every path below exists at the same relative path in the source repository except the relocated and edited files listed above).

## Milestone 3 (2026-11-03): no new copies

Everything in `cell_planner_msgs`, `cell_planning_service`, `cell_planning_curobo`, `config/cell.yaml`, `config/robot/rg2_collision_spheres.yaml`, and `robot_backend/cell_config.py` was written fresh. Values carried over from WATCHDOG code, with their source:

| Value | Where it is now | Source in WATCHDOG |
|---|---|---|
| Pose-goal planning budget 15 s, 30 attempts | `config/cell.yaml` `planner.moveit` | `robot_backend/ros_moveit_backend/motion.py:265-266` |
| Position 1e-4 m, orientation 1e-3 rad, joint 1e-4 rad tolerances | same | `motion.py:56-65` |
| Joint velocity/acceleration limits | read at run time from `ur5e_moveit_config/config/joint_limits.yaml` | same file |
| UR5e arm collision spheres | read at run time from `config/robot/ur5e.xrdf` | `config/ur5e.xrdf` |

Edited WATCHDOG-derived files in M3: `robot_backend/robot_backend/watchdog/halt.py` (rewritten, configurable targets), `robot_backend/robot_backend/mcp_servers/watchdog_mcp_server.py` (halt construction only), `robot_backend/package.xml` (missing dependencies), `robot_backend/test/test_watchdog_halt_callback_group.py` (stub service name follows the client's primary target), `sim_moveit_bringup/launch/sim_moveit_bringup.launch.py` and `package.xml` (planning-service include and two launch arguments).

## DROIDSTACK idea credits

DROIDSTACK (`/home/mohammad/droidstack`, branch `dev`, commit `17192dbca327771cb63eb22688b7ff1d62921ea5`) is unlicensed and belongs to a labmate. No code was copied or paraphrased from it. Designs credited to it as ideas, to be implemented from scratch in the milestone shown:

| Idea | Where it appears in this repo | Milestone |
|---|---|---|
| A planner is a stdio MCP server exposing exactly one tool, `execute_task(task_ref) -> {state: done or stopped, reason}`, called once per episode by a harness that is the MCP client. | `docs/PLANNER_CONTRACT.md`, `executor/planner_template/`, `runner/episode_runner/` | M6 |
| A structured "robot facts" journal published by the executor side (arm goal accepted, grip close/open, hold, carry, release) as untrusted hints for scoring. | `/cell/executor_facts` in `cell_planner_msgs`, `executor/planner_template/` | M6 |
| An authoring loop: freeze a snapshot of the editable planner, run a fresh episode, hand the agent the verdict paths, repeat; `submit_work` ends the session. | `executor/authoring_mcp/` | M6 |
| A loopback TCP newline-JSON command server running inside Kit, one request per connection, for layout reset, pose report, timeline control and health. | `isaac/cell_command_server/` | M5 |
| Headless Kit launch owned by the harness, with the ROS 2 bridge enabled by script. | `isaac/headless/` | M5 |
| Episode budget in simulated seconds plus a wall-clock guard around the single planner call. | `runner/episode_runner/` | M6 |
| An episode manifest that hashes every input (requirements, part data, planner snapshot) and records seed and versions. | `records/<episode_id>/manifest.json` | M6 |

## Full copied file list

```
agents/agent_harness/config.py
agents/agent_harness/__init__.py
agents/agent_harness/mcp_client.py
agents/agent_harness/providers.py
agents/agent_harness/runner.py
agents/agent_harness/task_render.py
agents/agent_harness/timebase.py
agents/agent_harness/transcript.py
agents/config/watchdog_agent_s3.yaml
agents/prompts/watchdog_system_s3.txt
agents/prompts/watchdog_task.txt
agents/tasks/fmb_assembly_2.watchdog.yaml
assets/ur5e/configuration/ur5e_base.usd
assets/ur5e/configuration/ur5e_physics.usd
assets/ur5e/configuration/ur5e_robot.usd
assets/ur5e/configuration/ur5e_sensor.usd
assets/ur5e/ur5e.usd
assets/ur_assets/ur_description/meshes/ur10/collision/base.stl
assets/ur_assets/ur_description/meshes/ur10/collision/forearm.stl
assets/ur_assets/ur_description/meshes/ur10/collision/shoulder.stl
assets/ur_assets/ur_description/meshes/ur10/collision/upperarm.stl
assets/ur_assets/ur_description/meshes/ur10/collision/wrist1.stl
assets/ur_assets/ur_description/meshes/ur10/collision/wrist2.stl
assets/ur_assets/ur_description/meshes/ur10/collision/wrist3.stl
assets/ur_assets/ur_description/meshes/ur10e/collision/base.stl
assets/ur_assets/ur_description/meshes/ur10e/collision/forearm.stl
assets/ur_assets/ur_description/meshes/ur10e/collision/shoulder.stl
assets/ur_assets/ur_description/meshes/ur10e/collision/upperarm.stl
assets/ur_assets/ur_description/meshes/ur10e/collision/wrist1.stl
assets/ur_assets/ur_description/meshes/ur10e/collision/wrist2.stl
assets/ur_assets/ur_description/meshes/ur10e/collision/wrist3.stl
assets/ur_assets/ur_description/meshes/ur10e/visual/base.dae
assets/ur_assets/ur_description/meshes/ur10e/visual/forearm.dae
assets/ur_assets/ur_description/meshes/ur10e/visual/shoulder.dae
assets/ur_assets/ur_description/meshes/ur10e/visual/upperarm.dae
assets/ur_assets/ur_description/meshes/ur10e/visual/wrist1.dae
assets/ur_assets/ur_description/meshes/ur10e/visual/wrist2.dae
assets/ur_assets/ur_description/meshes/ur10e/visual/wrist3.dae
assets/ur_assets/ur_description/meshes/ur10/visual/base.dae
assets/ur_assets/ur_description/meshes/ur10/visual/forearm.dae
assets/ur_assets/ur_description/meshes/ur10/visual/shoulder.dae
assets/ur_assets/ur_description/meshes/ur10/visual/upperarm.dae
assets/ur_assets/ur_description/meshes/ur10/visual/wrist1.dae
assets/ur_assets/ur_description/meshes/ur10/visual/wrist2.dae
assets/ur_assets/ur_description/meshes/ur10/visual/wrist3.dae
assets/ur_assets/ur_description/meshes/ur15/collision/base.stl
assets/ur_assets/ur_description/meshes/ur15/collision/forearm.stl
assets/ur_assets/ur_description/meshes/ur15/collision/shoulder.stl
assets/ur_assets/ur_description/meshes/ur15/collision/upperarm.stl
assets/ur_assets/ur_description/meshes/ur15/collision/wrist1.stl
assets/ur_assets/ur_description/meshes/ur15/collision/wrist2.stl
assets/ur_assets/ur_description/meshes/ur15/collision/wrist3.stl
assets/ur_assets/ur_description/meshes/ur15/LICENSE.txt
assets/ur_assets/ur_description/meshes/ur15/visual/base.dae
assets/ur_assets/ur_description/meshes/ur15/visual/forearm.dae
assets/ur_assets/ur_description/meshes/ur15/visual/shoulder.dae
assets/ur_assets/ur_description/meshes/ur15/visual/upperarm.dae
assets/ur_assets/ur_description/meshes/ur15/visual/UR15_DIFF_8bit_2K.jpg
assets/ur_assets/ur_description/meshes/ur15/visual/wrist1.dae
assets/ur_assets/ur_description/meshes/ur15/visual/wrist2.dae
assets/ur_assets/ur_description/meshes/ur15/visual/wrist3.dae
assets/ur_assets/ur_description/meshes/ur16e/collision/forearm.stl
assets/ur_assets/ur_description/meshes/ur16e/collision/upperarm.stl
assets/ur_assets/ur_description/meshes/ur16e/visual/forearm.dae
assets/ur_assets/ur_description/meshes/ur16e/visual/upperarm.dae
assets/ur_assets/ur_description/meshes/ur18/collision/forearm.stl
assets/ur_assets/ur_description/meshes/ur18/collision/upperarm.stl
assets/ur_assets/ur_description/meshes/ur18/LICENSE.txt
assets/ur_assets/ur_description/meshes/ur18/visual/forearm.dae
assets/ur_assets/ur_description/meshes/ur18/visual/upperarm.dae
assets/ur_assets/ur_description/meshes/ur18/visual/UR18_DIFF_8bit_2K.jpg
assets/ur_assets/ur_description/meshes/ur20/collision/base.stl
assets/ur_assets/ur_description/meshes/ur20/collision/forearm.stl
assets/ur_assets/ur_description/meshes/ur20/collision/shoulder.stl
assets/ur_assets/ur_description/meshes/ur20/collision/upperarm.stl
assets/ur_assets/ur_description/meshes/ur20/collision/wrist1.stl
assets/ur_assets/ur_description/meshes/ur20/collision/wrist2.stl
assets/ur_assets/ur_description/meshes/ur20/collision/wrist3.stl
assets/ur_assets/ur_description/meshes/ur20/LICENSE.txt
assets/ur_assets/ur_description/meshes/ur20/visual/base.dae
assets/ur_assets/ur_description/meshes/ur20/visual/forearm.dae
assets/ur_assets/ur_description/meshes/ur20/visual/shoulder.dae
assets/ur_assets/ur_description/meshes/ur20/visual/upperarm.dae
assets/ur_assets/ur_description/meshes/ur20/visual/UR20_DIFF_8bit_2K.png
assets/ur_assets/ur_description/meshes/ur20/visual/wrist1.dae
assets/ur_assets/ur_description/meshes/ur20/visual/wrist2.dae
assets/ur_assets/ur_description/meshes/ur20/visual/wrist3.dae
assets/ur_assets/ur_description/meshes/ur30/collision/forearm.stl
assets/ur_assets/ur_description/meshes/ur30/collision/upperarm.stl
assets/ur_assets/ur_description/meshes/ur30/LICENSE.txt
assets/ur_assets/ur_description/meshes/ur30/visual/forearm.dae
assets/ur_assets/ur_description/meshes/ur30/visual/upperarm.dae
assets/ur_assets/ur_description/meshes/ur30/visual/UR30_DIFF_8bit_2K.png
assets/ur_assets/ur_description/meshes/ur3/collision/base.stl
assets/ur_assets/ur_description/meshes/ur3/collision/forearm.stl
assets/ur_assets/ur_description/meshes/ur3/collision/shoulder.stl
assets/ur_assets/ur_description/meshes/ur3/collision/upperarm.stl
assets/ur_assets/ur_description/meshes/ur3/collision/wrist1.stl
assets/ur_assets/ur_description/meshes/ur3/collision/wrist2.stl
assets/ur_assets/ur_description/meshes/ur3/collision/wrist3.stl
assets/ur_assets/ur_description/meshes/ur3e/collision/base.stl
assets/ur_assets/ur_description/meshes/ur3e/collision/forearm.stl
assets/ur_assets/ur_description/meshes/ur3e/collision/shoulder.stl
assets/ur_assets/ur_description/meshes/ur3e/collision/upperarm.stl
assets/ur_assets/ur_description/meshes/ur3e/collision/wrist1.stl
assets/ur_assets/ur_description/meshes/ur3e/collision/wrist2.stl
assets/ur_assets/ur_description/meshes/ur3e/collision/wrist3.stl
assets/ur_assets/ur_description/meshes/ur3e/visual/base.dae
assets/ur_assets/ur_description/meshes/ur3e/visual/forearm.dae
assets/ur_assets/ur_description/meshes/ur3e/visual/shoulder.dae
assets/ur_assets/ur_description/meshes/ur3e/visual/upperarm.dae
assets/ur_assets/ur_description/meshes/ur3e/visual/wrist1.dae
assets/ur_assets/ur_description/meshes/ur3e/visual/wrist2.dae
assets/ur_assets/ur_description/meshes/ur3e/visual/wrist3.dae
assets/ur_assets/ur_description/meshes/ur3/visual/base.dae
assets/ur_assets/ur_description/meshes/ur3/visual/forearm.dae
assets/ur_assets/ur_description/meshes/ur3/visual/shoulder.dae
assets/ur_assets/ur_description/meshes/ur3/visual/upperarm.dae
assets/ur_assets/ur_description/meshes/ur3/visual/wrist1.dae
assets/ur_assets/ur_description/meshes/ur3/visual/wrist2.dae
assets/ur_assets/ur_description/meshes/ur3/visual/wrist3.dae
assets/ur_assets/ur_description/meshes/ur5/collision/base.stl
assets/ur_assets/ur_description/meshes/ur5/collision/forearm.stl
assets/ur_assets/ur_description/meshes/ur5/collision/shoulder.stl
assets/ur_assets/ur_description/meshes/ur5/collision/upperarm.stl
assets/ur_assets/ur_description/meshes/ur5/collision/wrist1.stl
assets/ur_assets/ur_description/meshes/ur5/collision/wrist2.stl
assets/ur_assets/ur_description/meshes/ur5/collision/wrist3.stl
assets/ur_assets/ur_description/meshes/ur5e/collision/base.stl
assets/ur_assets/ur_description/meshes/ur5e/collision/forearm.stl
assets/ur_assets/ur_description/meshes/ur5e/collision/shoulder.stl
assets/ur_assets/ur_description/meshes/ur5e/collision/upperarm.stl
assets/ur_assets/ur_description/meshes/ur5e/collision/wrist1.stl
assets/ur_assets/ur_description/meshes/ur5e/collision/wrist2.stl
assets/ur_assets/ur_description/meshes/ur5e/collision/wrist3.stl
assets/ur_assets/ur_description/meshes/ur5e/visual/base.dae
assets/ur_assets/ur_description/meshes/ur5e/visual/forearm.dae
assets/ur_assets/ur_description/meshes/ur5e/visual/shoulder.dae
assets/ur_assets/ur_description/meshes/ur5e/visual/upperarm.dae
assets/ur_assets/ur_description/meshes/ur5e/visual/wrist1.dae
assets/ur_assets/ur_description/meshes/ur5e/visual/wrist2.dae
assets/ur_assets/ur_description/meshes/ur5e/visual/wrist3.dae
assets/ur_assets/ur_description/meshes/ur5/visual/base.dae
assets/ur_assets/ur_description/meshes/ur5/visual/forearm.dae
assets/ur_assets/ur_description/meshes/ur5/visual/shoulder.dae
assets/ur_assets/ur_description/meshes/ur5/visual/upperarm.dae
assets/ur_assets/ur_description/meshes/ur5/visual/wrist1.dae
assets/ur_assets/ur_description/meshes/ur5/visual/wrist2.dae
assets/ur_assets/ur_description/meshes/ur5/visual/wrist3.dae
assets/ur_assets/ur_description/meshes/ur8long/collision/forearm.stl
assets/ur_assets/ur_description/meshes/ur8long/collision/upperarm.stl
assets/ur_assets/ur_description/meshes/ur8long/LICENSE.txt
assets/ur_assets/ur_description/meshes/ur8long/visual/forearm.dae
assets/ur_assets/ur_description/meshes/ur8long/visual/upperarm.dae
assets/ur_assets/ur_description/meshes/ur8long/visual/UR8Long_DIFF_8bit_2K.jpg
assets/usd/assembly_room.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb1/base1.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb1/.collect.mapping.json
assets/usd/Collected_fmb_assemblies/Collected_fmb1/home/aaugus11/Projects/aruco-grasp-annotator/data/usd/Collected_base1/NVIDIA/Materials/Base/Plastics/Plastic.mdl
assets/usd/Collected_fmb_assemblies/Collected_fmb1/home/aaugus11/Projects/aruco-grasp-annotator/data/usd/Collected_inverted_u_yellow/NVIDIA/Materials/Base/Plastics/Plastic.mdl
assets/usd/Collected_fmb_assemblies/Collected_fmb1/home/aaugus11/Projects/aruco-grasp-annotator/data/usd/Collected_u_brown/NVIDIA/Materials/Base/Plastics/Plastic.mdl
assets/usd/Collected_fmb_assemblies/Collected_fmb1/inverted_u_yellow.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb1/line_green.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb1_prepared/base1.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb1_prepared/inverted_u_yellow.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb1_prepared/line_green.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb1_prepared/u_brown.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb1_prepared/u_orange.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb1/u_brown.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb1/u_orange.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb2/base2_2x.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb2/base2.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb2/.collect.mapping.json
assets/usd/Collected_fmb_assemblies/Collected_fmb2/hex_blue_2x.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb2/hex_blue.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb2/hex_red_2x.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb2/hex_red.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb2/inverted_u_brown_2x.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb2/inverted_u_brown.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb2/u_green_2x.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb2/u_green.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb3/base3.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb3/.collect.mapping.json
assets/usd/Collected_fmb_assemblies/Collected_fmb3/fork_orange.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb3/fork_yellow.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb3/home/aaugus11/Projects/aruco-grasp-annotator/data/usd/Collected_base3/NVIDIA/Materials/Base/Plastics/Plastic.mdl
assets/usd/Collected_fmb_assemblies/Collected_fmb3/home/aaugus11/Projects/aruco-grasp-annotator/data/usd/Collected_fork_orange/NVIDIA/Materials/Base/Plastics/Plastic.mdl
assets/usd/Collected_fmb_assemblies/Collected_fmb3/home/aaugus11/Projects/aruco-grasp-annotator/data/usd/Collected_line_brown/NVIDIA/Materials/Base/Plastics/Plastic.mdl
assets/usd/Collected_fmb_assemblies/Collected_fmb3/home/aaugus11/Projects/aruco-grasp-annotator/data/usd/Collected_line_red/NVIDIA/Materials/Base/Plastics/Plastic.mdl
assets/usd/Collected_fmb_assemblies/Collected_fmb3/line_brown.usd
assets/usd/Collected_fmb_assemblies/Collected_fmb3/line_red.usd
assets/usd/Collected_workspace/.collect.mapping.json
assets/usd/Collected_workspace/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/UniversalRobots/ur5e/configuration/ur5e_base.usd
assets/usd/Collected_workspace/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/UniversalRobots/ur5e/configuration/ur5e_physics.usd
assets/usd/Collected_workspace/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/UniversalRobots/ur5e/configuration/ur5e_sensor.usd
assets/usd/Collected_workspace/RG2.usd
assets/usd/Collected_workspace/ur5e.usd
assets/usd/RG2_Good.usd
assets/usd/Rg2_mn.usd
assets/usd/RG2.usd
assets/usd/vention.usd
config/mcp/watchdog.mcp.json
config/robot/ur5e_rmpflow_common_v2.yaml
config/robot/ur5e.urdf
config/robot/ur5e.xrdf
config/robot/ur5e.yaml
config/settings/watchdog.settings.json
generated_urdf/rg2_only/README.md
generated_urdf/rg2_only/rg2_only/configuration/rg2_only_base.usd
generated_urdf/rg2_only/rg2_only/configuration/rg2_only_physics.usd
generated_urdf/rg2_only/rg2_only/configuration/rg2_only_robot.usd
generated_urdf/rg2_only/rg2_only/configuration/rg2_only_sensor.usd
generated_urdf/rg2_only/rg2_only_isaac/configuration/rg2_only_isaac_base.usd
generated_urdf/rg2_only/rg2_only_isaac/configuration/rg2_only_isaac_physics.usd
generated_urdf/rg2_only/rg2_only_isaac/configuration/rg2_only_isaac_robot.usd
generated_urdf/rg2_only/rg2_only_isaac/configuration/rg2_only_isaac_sensor.usd
generated_urdf/rg2_only/rg2_only_isaac.old_custom_backup.urdf
generated_urdf/rg2_only/rg2_only_isaac/rg2_only_isaac.usd
generated_urdf/rg2_only/rg2_only_isaac.urdf
generated_urdf/rg2_only/rg2_only/rg2_only.usd
generated_urdf/rg2_only/rg2_only.urdf
.mcp.json
ros2_ws/BUILD_NOTES.md
ros2_ws/colcon_defaults.yaml
ros2_ws/requirements.txt
ros2_ws/src/cell_orchestrator/cell_orchestrator/__init__.py
ros2_ws/src/cell_orchestrator/cell_orchestrator/node.py
ros2_ws/src/cell_orchestrator/cell_orchestrator/timebase.py
ros2_ws/src/cell_orchestrator/launch/orchestrator.launch.py
ros2_ws/src/cell_orchestrator/package.xml
ros2_ws/src/cell_orchestrator/resource/cell_orchestrator
ros2_ws/src/cell_orchestrator/setup.cfg
ros2_ws/src/cell_orchestrator/setup.py
ros2_ws/src/cell_orchestrator/test/test_guard_mode.py
ros2_ws/src/cell_orchestrator/test/test_step_signal_opens_gate.py
ros2_ws/src/cell_sync_interfaces/CMakeLists.txt
ros2_ws/src/cell_sync_interfaces/package.xml
ros2_ws/src/cell_sync_interfaces/srv/RequestGo.srv
ros2_ws/src/cell_sync_interfaces/srv/SignalReady.srv
ros2_ws/src/isaac_trajectory_bridge/config/rg2_calibration_table.csv
ros2_ws/src/isaac_trajectory_bridge_interfaces/action/GripperGuardedMove.action
ros2_ws/src/isaac_trajectory_bridge_interfaces/CMakeLists.txt
ros2_ws/src/isaac_trajectory_bridge_interfaces/package.xml
ros2_ws/src/isaac_trajectory_bridge_interfaces/srv/SetGripperPosition.srv
ros2_ws/src/isaac_trajectory_bridge/isaac_trajectory_bridge/calibrate_rg2_width.py
ros2_ws/src/isaac_trajectory_bridge/isaac_trajectory_bridge/follow_joint_trajectory_server.py
ros2_ws/src/isaac_trajectory_bridge/isaac_trajectory_bridge/__init__.py
ros2_ws/src/isaac_trajectory_bridge/isaac_trajectory_bridge/rg2_controller.py
ros2_ws/src/isaac_trajectory_bridge/launch/trajectory_bridge.launch.py
ros2_ws/src/isaac_trajectory_bridge/package.xml
ros2_ws/src/isaac_trajectory_bridge/README.md
ros2_ws/src/isaac_trajectory_bridge/README_RG2_CONTROLLER.md
ros2_ws/src/isaac_trajectory_bridge/resource/isaac_trajectory_bridge
ros2_ws/src/isaac_trajectory_bridge/setup.cfg
ros2_ws/src/isaac_trajectory_bridge/setup.py
ros2_ws/src/onrobot_rg_description/CMakeLists.txt
ros2_ws/src/onrobot_rg_description/config/rg2_visualize.rviz
ros2_ws/src/onrobot_rg_description/config/rg6_visualize.rviz
ros2_ws/src/onrobot_rg_description/meshes/rg2/collision/base_link.stl
ros2_ws/src/onrobot_rg_description/meshes/rg2/collision/inner_finger.stl
ros2_ws/src/onrobot_rg_description/meshes/rg2/collision/inner_knuckle.stl
ros2_ws/src/onrobot_rg_description/meshes/rg2/collision/outer_knuckle.stl
ros2_ws/src/onrobot_rg_description/meshes/rg2/visual/base_link.stl
ros2_ws/src/onrobot_rg_description/meshes/rg2/visual/inner_finger.stl
ros2_ws/src/onrobot_rg_description/meshes/rg2/visual/inner_knuckle.stl
ros2_ws/src/onrobot_rg_description/meshes/rg2/visual/outer_knuckle.stl
ros2_ws/src/onrobot_rg_description/meshes/rg6/collision/base_link.stl
ros2_ws/src/onrobot_rg_description/meshes/rg6/collision/base.stl
ros2_ws/src/onrobot_rg_description/meshes/rg6/collision/inner_finger.stl
ros2_ws/src/onrobot_rg_description/meshes/rg6/collision/inner_knuckle.stl
ros2_ws/src/onrobot_rg_description/meshes/rg6/collision/outer_knuckle.stl
ros2_ws/src/onrobot_rg_description/meshes/rg6/visual/base_link.stl
ros2_ws/src/onrobot_rg_description/meshes/rg6/visual/base.stl
ros2_ws/src/onrobot_rg_description/meshes/rg6/visual/inner_finger.stl
ros2_ws/src/onrobot_rg_description/meshes/rg6/visual/inner_knuckle.stl
ros2_ws/src/onrobot_rg_description/meshes/rg6/visual/outer_knuckle.stl
ros2_ws/src/onrobot_rg_description/package.xml
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg2_gazebo.xacro
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg2_model_macro.xacro
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg2_model.xacro
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg2_transmission.xacro
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg2.xacro
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg6/configuration/onrobot_rg6_base.usd
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg6/configuration/onrobot_rg6_physics.usd
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg6/configuration/onrobot_rg6_sensor.usd
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg6/configuration/.thumbs/256x256/onrobot_rg6_base.usd.png
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg6/configuration/.thumbs/256x256/onrobot_rg6_physics.usd.png
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg6_gazebo.xacro
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg6_model_macro.xacro
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg6_model.xacro
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg6/onrobot_rg6.usd
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg6_standalone.xacro
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg6/.thumbs/256x256/onrobot_rg6.usd.png
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg6_transmission.xacro
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg6.urdf
ros2_ws/src/onrobot_rg_description/urdf/onrobot_rg6.xacro
ros2_ws/src/robot_backend/config/assets/base1.obj
ros2_ws/src/robot_backend/config/assets/base2_2x.obj
ros2_ws/src/robot_backend/config/assets/fmb_u_green.obj
ros2_ws/src/robot_backend/config/assets/hex_blue_2x.obj
ros2_ws/src/robot_backend/config/assets/hex_red_2x.obj
ros2_ws/src/robot_backend/config/assets/inverted_u_brown_2x.obj
ros2_ws/src/robot_backend/config/assets/inverted_u_yellow.obj
ros2_ws/src/robot_backend/config/assets/line_green.obj
ros2_ws/src/robot_backend/config/assets/u_brown.obj
ros2_ws/src/robot_backend/config/assets/u_green_2x.obj
ros2_ws/src/robot_backend/config/assets/u_orange.obj
ros2_ws/src/robot_backend/config/model_registry.yaml
ros2_ws/src/robot_backend_interfaces/CMakeLists.txt
ros2_ws/src/robot_backend_interfaces/package.xml
ros2_ws/src/robot_backend_interfaces/srv/GetObjectPose.srv
ros2_ws/src/robot_backend_interfaces/srv/GetObjectPoses.srv
ros2_ws/src/robot_backend_interfaces/srv/SyncSceneObjects.srv
ros2_ws/src/robot_backend/LICENSE
ros2_ws/src/robot_backend/package.xml
ros2_ws/src/robot_backend/resource/robot_backend
ros2_ws/src/robot_backend/robot_backend/cell/gate.py
ros2_ws/src/robot_backend/robot_backend/cell/__init__.py
ros2_ws/src/robot_backend/robot_backend/contracts/__init__.py
ros2_ws/src/robot_backend/robot_backend/contracts/loader.py
ros2_ws/src/robot_backend/robot_backend/contracts/schema.py
ros2_ws/src/robot_backend/robot_backend/events/bus.py
ros2_ws/src/robot_backend/robot_backend/events/decorator.py
ros2_ws/src/robot_backend/robot_backend/events/__init__.py
ros2_ws/src/robot_backend/robot_backend/events/schema.py
ros2_ws/src/robot_backend/robot_backend/__init__.py
ros2_ws/src/robot_backend/robot_backend/interfaces/gripper_backend.py
ros2_ws/src/robot_backend/robot_backend/interfaces/__init__.py
ros2_ws/src/robot_backend/robot_backend/interfaces/motion_backend.py
ros2_ws/src/robot_backend/robot_backend/interfaces/robot_backend.py
ros2_ws/src/robot_backend/robot_backend/interfaces/scene_backend.py
ros2_ws/src/robot_backend/robot_backend/manual_tests/gripper_backend_smoke_test.py
ros2_ws/src/robot_backend/robot_backend/manual_tests/__init__.py
ros2_ws/src/robot_backend/robot_backend/manual_tests/motion_backend_smoke_test.py
ros2_ws/src/robot_backend/robot_backend/manual_tests/scene_backend_smoke_test.py
ros2_ws/src/robot_backend/robot_backend/mcp_servers/executor_monitor_mcp_server.py
ros2_ws/src/robot_backend/robot_backend/mcp_servers/gripper_mcp_server.py
ros2_ws/src/robot_backend/robot_backend/mcp_servers/__init__.py
ros2_ws/src/robot_backend/robot_backend/mcp_servers/motion_mcp_server.py
ros2_ws/src/robot_backend/robot_backend/mcp_servers/role_context.py
ros2_ws/src/robot_backend/robot_backend/mcp_servers/scene_mcp_server.py
ros2_ws/src/robot_backend/robot_backend/mcp_servers/watchdog_mcp_server.py
ros2_ws/src/robot_backend/robot_backend/nodes/__init__.py
ros2_ws/src/robot_backend/robot_backend/nodes/README.md
ros2_ws/src/robot_backend/robot_backend/nodes/scene_tf_sync_node.py
ros2_ws/src/robot_backend/robot_backend/registry/geometry.py
ros2_ws/src/robot_backend/robot_backend/registry/__init__.py
ros2_ws/src/robot_backend/robot_backend/registry/loader.py
ros2_ws/src/robot_backend/robot_backend/registry/model.py
ros2_ws/src/robot_backend/robot_backend/ros_moveit_backend/backend.py
ros2_ws/src/robot_backend/robot_backend/ros_moveit_backend/gripper.py
ros2_ws/src/robot_backend/robot_backend/ros_moveit_backend/__init__.py
ros2_ws/src/robot_backend/robot_backend/ros_moveit_backend/motion.py
ros2_ws/src/robot_backend/robot_backend/ros_moveit_backend/scene.py
ros2_ws/src/robot_backend/robot_backend/skills/gripper_skills.py
ros2_ws/src/robot_backend/robot_backend/skills/__init__.py
ros2_ws/src/robot_backend/robot_backend/skills/maintenance_skills.py
ros2_ws/src/robot_backend/robot_backend/skills/motion_skills.py
ros2_ws/src/robot_backend/robot_backend/skills/pose_query_skills.py
ros2_ws/src/robot_backend/robot_backend/utils/geometry.py
ros2_ws/src/robot_backend/robot_backend/utils/grasp_offsets.py
ros2_ws/src/robot_backend/robot_backend/utils/__init__.py
ros2_ws/src/robot_backend/robot_backend/utils/png_encode.py
ros2_ws/src/robot_backend/robot_backend/utils/rg2_geometry.py
ros2_ws/src/robot_backend/robot_backend/vision/__init__.py
ros2_ws/src/robot_backend/robot_backend/vision/look.py
ros2_ws/src/robot_backend/robot_backend/vision/observation.py
ros2_ws/src/robot_backend/robot_backend/watchdog/authored.py
ros2_ws/src/robot_backend/robot_backend/watchdog/engine.py
ros2_ws/src/robot_backend/robot_backend/watchdog/halt.py
ros2_ws/src/robot_backend/robot_backend/watchdog/image_buffer.py
ros2_ws/src/robot_backend/robot_backend/watchdog/__init__.py
ros2_ws/src/robot_backend/robot_backend/watchdog/monitors.py
ros2_ws/src/robot_backend/robot_backend/watchdog/README.md
ros2_ws/src/robot_backend/robot_backend/watchdog/ready.py
ros2_ws/src/robot_backend/robot_backend/watchdog/recorder.py
ros2_ws/src/robot_backend/robot_backend/watchdog/session.py
ros2_ws/src/robot_backend/robot_backend/watchdog/timebase.py
ros2_ws/src/robot_backend/scripts/bake_fmb_2x.py
ros2_ws/src/robot_backend/scripts/reconcile_event_seq.py
ros2_ws/src/robot_backend/setup.cfg
ros2_ws/src/robot_backend/setup.py
ros2_ws/src/robot_backend/test/capture_registry_box_golden.py
ros2_ws/src/robot_backend/test/executor_task_to_plan.py
ros2_ws/src/robot_backend/test/fixtures/registry_box_golden.json
ros2_ws/src/robot_backend/test/full_assembly_2.plan.yaml
ros2_ws/src/robot_backend/test/full_assembly.plan.yaml
ros2_ws/src/robot_backend/test/grasp_effort_test.py
ros2_ws/src/robot_backend/test/hex_blue_pick.plan.yaml
ros2_ws/src/robot_backend/test/hex_red_pick.plan.yaml
ros2_ws/src/robot_backend/test/inverted_u_brown_pick.plan.yaml
ros2_ws/src/robot_backend/test/inverted_u_yellow_pick.plan.yaml
ros2_ws/src/robot_backend/test/line_green_pick.plan.yaml
ros2_ws/src/robot_backend/test/log_grasp_effort.py
ros2_ws/src/robot_backend/test/pick_place_stress_test.py
ros2_ws/src/robot_backend/test/plan_assembly_step.py
ros2_ws/src/robot_backend/test/registry_golden_utils.py
ros2_ws/src/robot_backend/test/rg2_effort_sweep.py
ros2_ws/src/robot_backend/test/rg2_gripper_stiffness_sweep.py
ros2_ws/src/robot_backend/test/test_array_lane.py
ros2_ws/src/robot_backend/test/test_audit_intruder_leak.py
ros2_ws/src/robot_backend/test/test_authored_lane.py
ros2_ws/src/robot_backend/test/test_authored_monitor.py
ros2_ws/src/robot_backend/test/test_authored_parity.py
ros2_ws/src/robot_backend/test/test_authored_purpose_and_rate.py
ros2_ws/src/robot_backend/test/test_authored_validator.py
ros2_ws/src/robot_backend/test/test_camera_streams_live.py
ros2_ws/src/robot_backend/test/test_copyright.py
ros2_ws/src/robot_backend/test/test_engine.py
ros2_ws/src/robot_backend/test/test_event_bus.py
ros2_ws/src/robot_backend/test/test_executor_terminal_status_source.py
ros2_ws/src/robot_backend/test/test_final_run_verdict_cadence.py
ros2_ws/src/robot_backend/test/test_flake8.py
ros2_ws/src/robot_backend/test/test_grasp_skill.py
ros2_ws/src/robot_backend/test/test_grasp_skill_width_floor.py
ros2_ws/src/robot_backend/test/test_inject_intruder.py
ros2_ws/src/robot_backend/test/test_inject_scenarios.py
ros2_ws/src/robot_backend/test/test_maintenance_skills.py
ros2_ws/src/robot_backend/test/test_monitors.py
ros2_ws/src/robot_backend/test/test_motion_geometry.py
ros2_ws/src/robot_backend/test/test_observation.py
ros2_ws/src/robot_backend/test/test_pep257.py
ros2_ws/src/robot_backend/test/test_pick_contract.py
ros2_ws/src/robot_backend/test/test_pose_query_skills.py
ros2_ws/src/robot_backend/test/test_prompt_variants.py
ros2_ws/src/robot_backend/test/test_recorder_windows.py
ros2_ws/src/robot_backend/test/test_registry_kinds_and_acm.py
ros2_ws/src/robot_backend/test/test_registry_loader.py
ros2_ws/src/robot_backend/test/test_rendered_surface.py
ros2_ws/src/robot_backend/test/test_rg2_geometry.py
ros2_ws/src/robot_backend/test/test_role_context_provenance.py
ros2_ws/src/robot_backend/test/test_run_ended_terminal_event.py
ros2_ws/src/robot_backend/test/test_scenario_gating.py
ros2_ws/src/robot_backend/test/test_scorers.py
ros2_ws/src/robot_backend/test/test_session_and_image_buffer.py
ros2_ws/src/robot_backend/test/test_skill_event_coverage.py
ros2_ws/src/robot_backend/test/test_tf_time_alignment.py
ros2_ws/src/robot_backend/test/test_threshold_provenance.py
ros2_ws/src/robot_backend/test/test_watchdog_forced_termination.py
ros2_ws/src/robot_backend/test/test_watchdog_halt_callback_group.py
ros2_ws/src/robot_backend/test/test_watchdog_release_lifecycle.py
ros2_ws/src/robot_backend/test/test_watchdog_run_verdict.py
ros2_ws/src/robot_backend/test/u_brown_pick.plan.yaml
ros2_ws/src/robot_backend/test/u_green_pick.plan.yaml
ros2_ws/src/robot_backend/test/u_orange_pick.plan.yaml
ros2_ws/src/robot_backend/test/verify_plan_matches_task.py
ros2_ws/src/robot_backend/test/watchdog_rung2_effort_event_test.py
ros2_ws/src/robot_backend/test/watchdog_rung2_region_test.py
ros2_ws/src/robot_backend/test/watchdog_rung2_relative_frame_test.py
ros2_ws/src/robot_backend/test/watchdog_rung2_stationarity_test.py
ros2_ws/src/robot_backend/test/watchdog_rung3_halt_on_fire_test.py
ros2_ws/src/robot_backend/test/watchdog_smoke_test.py
ros2_ws/src/robot_description/CMakeLists.txt
ros2_ws/src/robot_description/launch/view_ur5e.launch.py
ros2_ws/src/robot_description/meshes/onrobot_rg2ft/collision/angle_bracket.stl
ros2_ws/src/robot_description/meshes/onrobot_rg2ft/README.md
ros2_ws/src/robot_description/meshes/onrobot_rg2ft/visual/angle_bracket.stl
ros2_ws/src/robot_description/meshes/ur5e/collision/base.stl
ros2_ws/src/robot_description/meshes/ur5e/collision/forearm.stl
ros2_ws/src/robot_description/meshes/ur5e/collision/shoulder.stl
ros2_ws/src/robot_description/meshes/ur5e/collision/upperarm.stl
ros2_ws/src/robot_description/meshes/ur5e/collision/wrist1.stl
ros2_ws/src/robot_description/meshes/ur5e/collision/wrist2.stl
ros2_ws/src/robot_description/meshes/ur5e/collision/wrist3.stl
ros2_ws/src/robot_description/meshes/ur5e/visual/base.dae
ros2_ws/src/robot_description/meshes/ur5e/visual/forearm.dae
ros2_ws/src/robot_description/meshes/ur5e/visual/shoulder.dae
ros2_ws/src/robot_description/meshes/ur5e/visual/upperarm.dae
ros2_ws/src/robot_description/meshes/ur5e/visual/wrist1.dae
ros2_ws/src/robot_description/meshes/ur5e/visual/wrist2.dae
ros2_ws/src/robot_description/meshes/ur5e/visual/wrist3.dae
ros2_ws/src/robot_description/package.xml
ros2_ws/src/robot_description/rviz/ur5e.rviz
ros2_ws/src/robot_description/urdf/ur5e_ros.urdf
ros2_ws/src/sim_moveit_bringup/CMakeLists.txt
ros2_ws/src/sim_moveit_bringup/config/cameras.yaml
ros2_ws/src/sim_moveit_bringup/launch/camera_tf.launch.py
ros2_ws/src/sim_moveit_bringup/launch/sim_moveit_bringup.launch.py
ros2_ws/src/sim_moveit_bringup/package.xml
ros2_ws/src/sim_moveit_bringup/README.md
ros2_ws/src/sim_moveit_bringup/sim_moveit_bringup/pointcloud_workspace_filter.py
ros2_ws/src/sim_moveit_bringup/sim_moveit_bringup/print_current_collisions.py
ros2_ws/src/sim_moveit_bringup/sim_moveit_bringup/static_camera_tf_broadcaster.py
ros2_ws/src/sim_moveit_bringup/sim_moveit_bringup/workspace_bounds_publisher.py
ros2_ws/src/ur5e_moveit_config/CMakeLists.txt
ros2_ws/src/ur5e_moveit_config/config/initial_positions.yaml
ros2_ws/src/ur5e_moveit_config/config/joint_limits.yaml
ros2_ws/src/ur5e_moveit_config/config/kinematics.yaml
ros2_ws/src/ur5e_moveit_config/config/moveit_controllers.yaml
ros2_ws/src/ur5e_moveit_config/config/moveit_cpp.yaml
ros2_ws/src/ur5e_moveit_config/config/ompl_planning.yaml
ros2_ws/src/ur5e_moveit_config/config/sensors_3d.yaml
ros2_ws/src/ur5e_moveit_config/config/ur5e.srdf
ros2_ws/src/ur5e_moveit_config/launch/demo.launch.py
ros2_ws/src/ur5e_moveit_config/launch/move_group.launch.py
ros2_ws/src/ur5e_moveit_config/launch/moveit_rviz.launch.py
ros2_ws/src/ur5e_moveit_config/launch/planning_context.launch.py
ros2_ws/src/ur5e_moveit_config/launch/rsp.launch.py
ros2_ws/src/ur5e_moveit_config/NOTES_RG2_MIGRATION.md
ros2_ws/src/ur5e_moveit_config/package.xml
ros2_ws/src/ur5e_moveit_config/rviz/moveit.rviz
runner/recorder/frame_recorder.py
runner/run_watchdog.sh
runner/supervisor/preflight_check.sh
```

## Milestone 3 (v2) addendum, 2026-11-10: platform backends, names in config, Humble container

No new files were copied from WATCHDOG. DROIDSTACK was not read for this milestone; the loopback command-server idea credited above is referenced by `cell_backend_sim/scene.py` (client side, written from scratch).

### New files (all written from scratch)

- `config/platforms/sim.yaml`, `config/platforms/hardware.yaml`, `config/cameras/sim.yaml`, `config/cameras/hardware.yaml`, `config/gripper/rg2_kinematics.yaml`, `config/gripper/rg6_kinematics.yaml`, `config/mcp/watchdog.hardware.mcp.json`, `config/settings/watchdog.hardware.settings.json` (byte copy of `config/settings/watchdog.settings.json`).
- `ros2_ws/src/cell_platform_interfaces/` (package: `platform_config.py`, `compat.py`, `signals.py`, `types.py`, `interfaces.py`, `fjt_motion.py`, `topic_poses.py`, `timebase.py`, `show_platform.py`, tests, README).
- `ros2_ws/src/cell_backend_sim/` (`motion.py`, `gripper.py`, `poses.py`, `scene.py`, `ground_truth_pose_publisher.py`, test, README).
- `ros2_ws/src/cell_backend_hw/` (`motion.py`, `gripper.py`, `poses.py`, `scene.py`, test, README).
- `docker/humble/Dockerfile`, `docker/humble/compose.yaml`, `docker/humble/entrypoint.sh`, `docs/PLATFORMS.md`.

### Value sources

| Value | Source |
|---|---|
| Every name in `config/platforms/sim.yaml` | the literals removed from the files in the next table (BUILD_PLAN v2 section 5.1) |
| `config/cameras/sim.yaml` camera2 extrinsics | `ros2_ws/src/sim_moveit_bringup/config/cameras.yaml` (copied) |
| `config/cameras/sim.yaml` camera2 topics | `robot_backend/watchdog/image_buffer.py` former `_DEFAULT_*` constants |
| `config/gripper/rg2_kinematics.yaml` | `isaac_trajectory_bridge/rg2_controller.py` constants and the launch override (copied, lines cited in the file) |
| `config/platforms/hardware.yaml` driver names | Universal_Robots_ROS2_Driver conventional names, every one marked `TODO(mohammad)` to confirm; camera models from Mohammad (BUILD_PLAN q2) |
| Everything else in `hardware.yaml`, `cameras/hardware.yaml`, `rg6_kinematics.yaml` | `TODO(mohammad)` |

### WATCHDOG-derived files edited in this milestone (all for rule 10, names from config)

| File | Edit |
|---|---|
| `robot_backend/robot_backend/cell_config.py` | platform helpers (`platform`, `topic`, `frame`, `service`, `cell_block`, `watchdog_camera_topics`, `platform_use_sim_time`) |
| `robot_backend/robot_backend/watchdog/recorder.py` | topic constants removed; subscriptions use the platform topics; `_cell_frame_ids` reads `cell_block()` instead of the registry `cell:` block |
| `robot_backend/robot_backend/events/bus.py` | event topic from the platform |
| `robot_backend/robot_backend/cell/gate.py`, `watchdog/ready.py` | gate service names from the platform |
| `robot_backend/robot_backend/watchdog/image_buffer.py` | the five camera topic constants replaced by `_camera_topics()` (cameras config, camera2); env overrides kept |
| `robot_backend/robot_backend/watchdog/halt.py` | name literals removed; primary target = platform `motion.cancel_service`; extras from `cell.yaml watchdog.halt.extra_cancel_targets` |
| `robot_backend/robot_backend/mcp_servers/watchdog_mcp_server.py` | halt construction; new read-only tool `available_evidence_sources`; `get_contract` reports the platform cell block |
| `robot_backend/robot_backend/vision/observation.py` | observation frame = platform `frames.world` |
| `robot_backend/robot_backend/contracts/loader.py` | effort joint from `cell_block()` |
| `robot_backend/robot_backend/watchdog/session.py` | manifest `cell_use_sim_time` from the platform |
| `robot_backend/package.xml`, `cell_orchestrator/package.xml`, `isaac_trajectory_bridge/package.xml`, `sim_moveit_bringup/package.xml` | dependency on `cell_platform_interfaces` (+ `cell_backend_sim` for the bringup) |
| `cell_orchestrator/cell_orchestrator/node.py` | service names from parameters, else the platform |
| `isaac_trajectory_bridge/launch/trajectory_bridge.launch.py` | all names and the RG2 launch targets read from `platforms/sim.yaml`; node files untouched |
| `sim_moveit_bringup/launch/sim_moveit_bringup.launch.py` | bag topics, world/World static TF, scene-sync service parameters from the platform; `-s mcap` through `compat.bag_record_command`; starts `ground_truth_pose_publisher` (`start_pose_publisher`, default true) |
| `agents/prompts/watchdog_system_s3.txt` | one tool entry added (`available_evidence_sources`), stating what it returns only |
| `runner/recorder/frame_recorder.py` | topic from the cameras config `recorder.topics.rgb` when the parameter is empty |
| `config/cell.yaml` | `platform:`; empty-means-platform for `planner.service_namespace`, `planner.joint_state_topic`, cuRobo `base_link`/`ee_link`, depth topics, recorder topic; `watchdog.halt.extra_cancel_targets` |

Milestone 4 (v1 M3) files edited for the same rule: `cell_planning_service/node.py`, `moveit_backend.py` (service names from `defaults["services"]`), `cell_planning_curobo/node.py`, `curobo_backend.py`, `depth_obstacles.py`.

## Milestone 5 addendum, 2026-11-11: physical events, plan-blind WatchDog

No new files were copied from WATCHDOG. DROIDSTACK was not read.

### New files (written from scratch)

- `ros2_ws/src/cell_planner_msgs/msg/PhysicalEvent.msg`.
- `ros2_ws/src/cell_physical_events/` (`detectors.py`, `node.py`, tests, README).
- `robot_backend/robot_backend/watchdog/physical.py`, `requirements.py`, `rulings.py`.
- `robot_backend/test/test_plan_blind_watchdog.py`.
- `config/requirements/fmb_assembly_2.yaml` (derived from `agents/tasks/fmb_assembly_2.watchdog.yaml`: `instruction_spec` rewritten without steps or executor events; grasp widths copied per part; bystanders and object_ids copied).
- `config/parts/watchdog_parts.yaml`, `config/parts/executor_parts.yaml` (generated by `tools/split_registry.py` from the registry; every number copied).
- `agents/prompts/watchdog_system_plan_blind.txt` (derived from `watchdog_system_s3.txt`, see `docs/INDEPENDENCE.md` row 10), `agents/prompts/watchdog_task_plan_blind.txt`, `agents/config/watchdog_agent_plan_blind.yaml`, `config/mcp/watchdog.plan_blind.mcp.json`.
- `docs/INDEPENDENCE.md`, `tools/split_registry.py`.

### WATCHDOG-derived files edited

| File | Edit |
|---|---|
| `robot_backend/events/schema.py` | `HaltPayload` + `halt_id`, `suppressed`, `mode`; `VerdictPayload.step_id` → `subject` + evidence fields; `RunVerdictPayload.steps_covered` → `parts_covered` + evidence fields and counts; new `RulingPayload`, `AcceptancePayload`; event types `ruling`, `acceptance`. |
| `robot_backend/watchdog/recorder.py` | executor-origin envelopes dropped at ingest; step tracking removed; subscriptions to physical events and notices; objects sampled from the WatchDog pose channel, TCP aligned to the observations' stamp; object frames answered from the channel in `lookup_transform*`; `physical_state()`. |
| `robot_backend/watchdog/engine.py` | `on_skill_event` → `on_physical_event` (release auto-disarm kept); wake conditions. |
| `robot_backend/watchdog/halt.py` | `mode` (record_only suppresses the cancel), `halt_id`, halt registry, notice on real halts. |
| `robot_backend/mcp_servers/watchdog_mcp_server.py` | requirements loader replaces step loading; episode-end termination; per-part rollup; evidence tagging; tools `arm_wake`, `disarm_wake`, `wrist_pose_cue`, `issue_ruling`, `inspect_acceptance`; `look(camera=)`; `signal_ready(scope="run")` only; `get_execution_state` reports physical state. |
| `sim_moveit_bringup/launch/sim_moveit_bringup.launch.py` | starts `cell_physical_events` (`start_physical_events`). |
| `runner/run_watchdog.sh` | scenario `plan_blind` (default) or `s3`. |
| `config/cell.yaml` | `physical_events:` thresholds (all TODO). |

## Milestone 6 addendum, 2026-11-11: command server, headless Isaac, sim cameras, recording

No files copied from WATCHDOG. DROIDSTACK was not read; the loopback command-server and headless-launch ideas were already credited above. Stage facts used (prim paths, graph node types, finger pad bodies, the existing camera graph's shape) were read from `assets/usd/assembly_room.usd` with usd-core on 2026-11-11, not from any repo's code. Isaac API names were checked against the installed 5.1 extensions (`isaacsim.core.experimental.prims`, `isaacsim.ros2.bridge` node types, `isaacsim.simulation_app`).

New files, all written from scratch: `isaac/common/kit_api.py` (the only Kit-version-specific module), `isaac/cell_command_server/{server.py, layouts.py, start_in_script_editor.py, test_layouts.py, README.md}`, `isaac/headless/{run_headless.py, README.md}`, `isaac/cameras/{setup_cameras.py, README.md}`, `config/layouts/fmb_assembly_2/nominal.yaml` (poses null, `TODO(mohammad)`; home pose copied from `ur5e_moveit_config/config/ur5e.srdf`; gripper open from `rg2_controller.py:66`), `ros2_ws/src/cell_planner_msgs/msg/PadContact.msg`, `ros2_ws/src/cell_backend_sim/cell_backend_sim/pad_contacts_relay.py`, `runner/recorder/{episode_recording.py, test_episode_recording.py}`.

Edited: `config/platforms/sim.yaml` (`scene` block: port 9010 as an arbitrary loopback choice, stage prim paths, SRDF home pose, contact pads; `pad_contacts` signal now `available`), `config/cell.yaml` (`recording.bag` topic keys, `recording.frames.enabled`), `cell_backend_sim/scene.py` (`load_layout`), `cell_backend_sim/setup.py`, `package.xml`.

### Removed tests (they tested the removed executor coupling)

`test_executor_terminal_status_source.py`, `test_final_run_verdict_cadence.py`, `test_run_ended_terminal_event.py`, `test_watchdog_forced_termination.py`, `test_watchdog_run_verdict.py`, `test_watchdog_release_lifecycle.py`, `test_tf_time_alignment.py`. Their surviving concerns (held span, release auto-disarm, per-part rollup, backstop finalize, TCP/object alignment) are covered by `test_plan_blind_watchdog.py`.
