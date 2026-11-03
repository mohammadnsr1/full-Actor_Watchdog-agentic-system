# Build plan (Milestone 1, read only)

Written 2026-11-03 from files only. Nothing was built, run, or changed in WATCHDOG or DROIDSTACK. Builds on `docs/INTEGRATION_SURVEY.md`, especially Part G.

Paths: `W/` = `/home/mohammad/watchdog/WatchDog_V1`, `I/` = `/home/mohammad/full-Actor_Watchdog-agentic-system`. Line numbers are from the files as they stand today.

**WATCHDOG commit note.** `W/.git/HEAD` → `refs/heads/main`, and `W/.git/refs/heads/main` now reads `595ede3354dc40c916b4bcd464ebb2db1b014b32`, three commits after the `cb973bbf…` recorded in the survey. The reflog subjects of those three commits (`W/.git/logs/HEAD`, last three lines) are paper-media runs and "Update assembly_room.usd with new binary data", so the stage file changed after the survey. `origin/main` is still at `443a3fa8…` (`W/.git/packed-refs`). PROVENANCE.md will record `595ede33…` unless Mohammad says otherwise. A second local branch `media` exists.

**Isaac install check outside the repo.** To settle the Isaac version, the file `~/isaacsim/VERSION` and the Kit user config under `~/.local/share/ov/data/Kit/Isaac-Sim Full/5.1/` were read. Those are outside both repositories; they were only read.

---

## 1. WATCHDOG inventory

Legend: **AS IS** = byte copy, same relative path. **ADAPT** = copy, then edit in the named milestone. **NO** = not copied (reason given). "M2" means copied in Milestone 2; later milestones only edit what M2 copied unless noted.

### 1.1 ROS 2 workspace `W/ros2_ws/` → `I/ros2_ws/`

| Source | Verdict | Notes |
|---|---|---|
| `src/robot_backend/` (whole package: `package.xml`, `setup.py`, `setup.cfg`, `resource/`, `robot_backend/**`, `config/model_registry.yaml`, `config/assets/*.obj`, `scripts/`, `test/`, `manual_tests/`) | **ADAPT** (M2 copy whole; prune in M6) | Must be copied whole in M2 so it builds and the unchanged WatchDog session runs: the WatchDog server imports `vision`, `role_context`, `events`, `registry`, `watchdog/*`, `contracts/*`, `session` from the same package. Executor-side modules to retire in M6 once the planner template replaces them: `mcp_servers/motion_mcp_server.py`, `gripper_mcp_server.py`, `scene_mcp_server.py`, `executor_monitor_mcp_server.py` (S4 only), `skills/*`, `ros_moveit_backend/*`, `interfaces/*`, `cell/gate.py` (the executor-side gate client; the planner template gets its own). Keep: `nodes/scene_tf_sync_node.py` (serves the executor pose-query channel `/get_object_pose(s)` and `/sync_scene_objects`, `nodes/scene_tf_sync_node.py:66-71, 196-225`), `utils/rg2_geometry.py` (the validated width table reader, `:63-76, 141-185`), everything WatchDog-side and shared. `package.xml` is missing `cell_sync_interfaces` and `action_msgs` although `cell/gate.py:60`, `watchdog/ready.py` and `watchdog/halt.py` import them; it builds today only because they are installed anyway. Fix in M4 when `halt.py` is edited. |
| `src/robot_backend/config/model_registry.yaml` | **AS IS** in M2 (inside the package) | 867 lines; `cell:` at `:112`, `acm_allowed_object_pairs:` at `:138`, `objects:` at `:227`. In M4/M6 it is split into `config/parts/watchdog_parts.yaml` and `config/parts/executor_parts.yaml`; the package copy stays as the loader's default until then. Loaders: `registry/loader.py:106`, `watchdog/recorder.py:144-162`, `contracts/loader.py:35-101`, `nodes/scene_tf_sync_node.py:144, 498-509`, `mcp_servers/role_context.py:191-206`, the bringup launch `:95-107, 122`. |
| `src/robot_backend_interfaces/` | **AS IS** | `GetObjectPose.srv`, `GetObjectPoses.srv`, `SyncSceneObjects.srv` (`CMakeLists.txt:8-11`). The executor pose-query channel. |
| `src/cell_orchestrator/` | **AS IS** in M2; **ADAPT** in M6 | The gate. `/cell/request_go` and `/cell/signal_ready` (`cell_orchestrator/node.py:216-221`), abort latch (`:337-358`, refusal `:405-406`). M6 adds the episode-end signal path. `RequestGo.srv:8` documents reason codes that the node no longer returns (node returns `released`, `timeout_gate`, `unguarded_by_config`, `unguarded_latched`, `aborted`); fix the comment when the srv is next touched. |
| `src/cell_sync_interfaces/` | **AS IS** | `RequestGo.srv`, `SignalReady.srv`. |
| `src/isaac_trajectory_bridge/` | **AS IS** | `follow_joint_trajectory_server.py`, `rg2_controller.py`, `calibrate_rg2_width.py`, `launch/trajectory_bridge.launch.py`, `config/rg2_calibration_table.csv` (537 rows, installed by `setup.py:22-25`), both READMEs. Never edited: the RG2 constants and the width table live here. |
| `src/isaac_trajectory_bridge_interfaces/` | **AS IS** | `SetGripperPosition.srv`, `GripperGuardedMove.action`. |
| `src/sim_moveit_bringup/` | **AS IS** in M2; **ADAPT** in M3, M5 | `launch/sim_moveit_bringup.launch.py` (whole-cell launch), `launch/camera_tf.launch.py`, `config/cameras.yaml`, `scripts/static_camera_tf_broadcaster.py`, `pointcloud_workspace_filter.py`, `print_current_collisions.py`, `workspace_bounds_publisher.py`. M3 adds a `planner.backend` switch and keeps `move_group` plan-only; M5 adds the three new bag topics (`:24`). The launch imports `robot_backend.watchdog.session` and `robot_backend.registry` directly (`:61, 95-96, 234, 244`). |
| `src/ur5e_moveit_config/` | **AS IS** | SRDF (`config/ur5e.srdf`: `world_joint` `:3`, group `ur_manipulator` base_link→tcp `:5-6`, `home` `:9-15`), `moveit_controllers.yaml` (`arm_controller`, `action_ns: follow_joint_trajectory`, `:13-18`), `kinematics.yaml`, `joint_limits.yaml`, `ompl_planning.yaml`, `sensors_3d.yaml`, `moveit_cpp.yaml`, `initial_positions.yaml`, five launch files, `rviz/moveit.rviz`, `NOTES_RG2_MIGRATION.md`. Needed by the MoveIt planning backend (M3). |
| `src/robot_description/` | **AS IS** | `urdf/ur5e_ros.urdf` (tcp link `:204`, RG2 links from `:212`), `launch/view_ur5e.launch.py`, `rviz/ur5e.rviz`, `meshes/onrobot_rg2ft/README.md`. |
| `src/onrobot_rg_description/` | **AS IS** | RG2 and RG6 xacros, meshes, rviz configs. Referenced by the URDF through `package://`. |
| `BUILD_NOTES.md`, `colcon_defaults.yaml`, `requirements.txt` | **AS IS** | `colcon_defaults.yaml` pins `-DPython3_EXECUTABLE=/usr/bin/python3`; `requirements.txt` pins `numpy<2`, `mcp<2`. The MCP servers re-exec into `ros2_ws/.venv` (`robot_backend/mcp_servers/__init__.py:47-80`), so Mohammad recreates that venv in `I/ros2_ws/.venv`. |
| `.venv/`, `build/`, `install/`, `log/` | **NO** | Build products; gitignored in both repos. |

### 1.2 Agents, prompts, configs, launchers

| Source | Verdict | Notes |
|---|---|---|
| `W/agents/agent_harness/` (`config.py`, `task_render.py`, `timebase.py`, `__init__.py`; also `mcp_client.py`, `providers.py`, `runner.py`, `transcript.py`) | **AS IS** (M2) → `I/agents/agent_harness/` | Required at run time: `role_context.build_rendered_task` inserts `<repo>/agents` on `sys.path` and imports `agent_harness.config.load_config` and `agent_harness.task_render.render_task` (`role_context.py:232-236`). The repo root is found by walking up for a directory holding both `agents/` and `ros2_ws/` (`role_context.py:97-110`, `session.py repo_root`). The API-path modules (`mcp_client`, `providers`, `runner`) are not needed but are copied so the package is intact; they can go in M6. |
| `W/agents/config/watchdog_agent_s3.yaml` | **ADAPT** (M2: paths; M4: prompt names) | Points at `prompts/watchdog_system_s3.txt` and `prompts/watchdog_task.txt` relative to `agents/` (`:17-18`); its `mcp_servers[].cwd` hardcodes `~/watchdog/WatchDog_V1/ros2_ws` (`:21-24`), used only by the API path. M2 copies and rewrites that cwd. |
| `W/agents/config/watchdog_agent.yaml`, `watchdog_agent_s2.yaml`, `executor_agent.yaml`, `executor_agent_s4.yaml` | **NO** | S1, S2, S4 and executor API-path configs. The executor role is replaced by the authoring agent. |
| `W/agents/prompts/watchdog_system_s3.txt` | **ADAPT** (M4) | Source of the plan-blind prompt. Keep: Authority `:9-19` (rewritten to allow halt in `halt_enabled` mode), evidence discipline `:49-75` with the agent-chosen-threshold rule `:57-59` and `:696-699`, verdict meanings `:648-692`. Remove: the control loop `:418-646` (tells it when to call tools), "What is expected to move" `:349-371` and "Steps on and off the plan" `:373-416` (plan knowledge), the `begin_step`/`end_step` flow. |
| `W/agents/prompts/watchdog_task.txt` | **ADAPT** (M4) | The `{{STEPS}}` block at `:9-18` ("Expected process") hands the WatchDog the step plan; `task_render.py:24-26` wrongly claims there is no such placeholder. Remove `{{STEPS}}` and `{{STEP_PLAN}}`. |
| `W/agents/prompts/watchdog_system.txt`, `watchdog_system_s2.txt`, `executor_system.txt`, `executor_system_s4.txt`, `executor_task.txt`, `prompt_variants.py`, `regenerate_variants.py` | **NO** | S1/S2/S4 and executor prompts. `executor_system.txt:152-203` ("MoveIt and the planning scene") is useful reading for the planner contract but is not copied. |
| `W/agents/tasks/fmb_assembly_2.watchdog.yaml` | **ADAPT** (M2 as is for the unchanged check; M4 becomes `config/requirements/fmb_assembly_2.yaml`) | Keys `name :10`, `instruction_spec :11`, `step_plan :64`, `steps :101-163`, `object_ids :167`, `run_notes :225`. M4 removes `step_plan` and `steps` and restructures into acceptance contracts and invariants. |
| `W/agents/tasks/fmb_assembly_2.executor.yaml` | **ADAPT** (M6 → `config/tasks/fmb_assembly_2.yaml`) | Its `instruction_spec` and per-step fields (`grasp_width_m`, `grasp_center_offset`, `target_frame`, `target_offset`, `approach_distance_m`, `lift_height_m`, `axis_note`) become the executor's work instruction and part data. Not copied into `agents/tasks/` (the WatchDog server must never find it). |
| `W/agents/tasks/fmb_assembly.*.yaml`, `*_s4.executor.yaml`, `runs/*` (r2–r6 variants), `single_step_pick.*`, `two_cube_stack.*`, `u_green_pick.*` | **NO** | Assembly 1, S4, Class 2/3 variants, dev tasks. |
| `W/agents/run_agent.py`, `requirements.txt`, `tests/`, `README.md` | **NO** | API-path runner and its tests. `README.md` is reference only. |
| `W/agents/transcripts/` (117 session dirs) | **NO** | Sessions. Excluded by instruction. |
| `W/watchdog_2_s3.mcp.json` | **ADAPT** (M2 → `I/config/mcp/watchdog.mcp.json`) | Hardcodes `~/watchdog/WatchDog_V1/ros2_ws/install/setup.bash` (`:28`), `WATCHDOG_TASK_PATH` and `WATCHDOG_AGENT_CONFIG` absolute paths (`:31-33`). M2 rewrites those three paths to `I/`; nothing else. Its `_venv_comment` (`:37`) about `unset VIRTUAL_ENV` stays. |
| `W/watchdog.mcp.json`, `watchdog_2.mcp.json`, `watchdog_s2.mcp.json`, `watchdog_2_s2.mcp.json`, `watchdog_s3.mcp.json` | **NO** | S1/S2 and assembly-1 variants. |
| `W/executor*.mcp.json` (13 files) | **NO** | Executor servers are replaced by `executor/authoring_mcp/`. `executor_2.mcp.json:19-55` is reference for the env-var pattern. |
| `W/watchdog.settings.json` | **ADAPT** (M2 → `I/config/settings/watchdog.settings.json`; M4 tool list) | Model/effort knobs (`:4-5`), `autoMemoryEnabled: false` (`:7`), allow `mcp__robot-backend-watchdog` only, deny list (`:13-37`). Content unchanged in M2; only the location moves. |
| `W/executor.settings.json` | **ADAPT** (M6 → `I/config/settings/executor.settings.json`) | Reference for the authoring role's restricted settings; the allow list changes completely. |
| `W/run_watchdog.sh` | **ADAPT** (M2 → `I/runner/run_watchdog.sh` for the unchanged check; M6 the episode runner starts the session) | Launch line `:75-78` and the two-part `get_rendered_task` bootstrap. M2 rewrites the config path only. |
| `W/run_executor.sh` | **NO** | Reference for the agent-config record pattern (`:65-111`). |
| `W/.mcp.json` | **AS IS** (M2) | Project-wide developer MCP config holding only the `kit` TCP bridge (`:34-42`, `OMNI_KIT_MCP_PORT=9009`). Scored sessions bypass it with `--strict-mcp-config`. Useful for Mohammad's own Kit sessions. |
| `W/.claude/settings.local.json` | **NO** | Local permissions. |

### 1.3 Assets, robot configs, URDFs

| Source | Verdict | Notes |
|---|---|---|
| `W/assets/usd/` (`assembly_room.usd`, `RG2_Good.usd`, `Collected_fmb_assemblies/`, `Collected_workspace/`) | **AS IS** | The stage. Note the stage was re-saved on the last WATCHDOG commit. The recorder camera `/recorder_rgb` (1280x720, frame `sim_camera`) was "added to the stage on 2026-10-29" (`paper_media/recorder/frame_recorder.py:31-33`); nothing in the repo creates it, so it lives inside the USD. |
| `W/assets/ur5e/` (`ur5e.usd`, `configuration/`), `W/assets/ur_assets/ur_description/` | **AS IS** | Robot USD and description. |
| `W/config/` (`ur5e.urdf`, `ur5e.xrdf`, `ur5e.yaml`, `ur5e_rmpflow_common_v2.yaml`) | **AS IS** → `I/config/robot/` | `ur5e.xrdf` and the RMPflow yaml are Isaac/Lula robot descriptions, directly useful to the cuRobo backend (M3). Moved under `config/robot/` so `config/` keeps the layout in section 5. |
| `W/generated_urdf/` (`rg2_only.urdf`, `rg2_only_isaac.urdf`, `rg2_only_isaac.old_custom_backup.urdf`, `rg2_only/`, `rg2_only_isaac/`, `README.md`) | **AS IS** | RG2 URDFs for Isaac import and for cuRobo collision spheres. |

### 1.4 Recording, scorers, labels, tools

| Source | Verdict | Notes |
|---|---|---|
| `W/paper_media/recorder/frame_recorder.py` | **ADAPT** (M2 copy as is → `I/runner/recorder/frame_recorder.py`; M5 edits) | **This is the high-resolution scene camera frame recorder.** Passive rclpy node `paper_media_frame_recorder` (`:96`), subscribes best-effort depth 1 (`:150-156`) to `/recorder_rgb` (`:62`), 1280x720 from a recording-only camera on the stage, JPEG quality 95, 10 Hz wall-clock throttle, `max_width` 1280 (`:97-102`). Writes `<session>/frames/<camera_name>/<index:06d>.jpg` and `frames.jsonl` with `index, file, wall_time_iso, header_stamp, topic, camera_name, width, height` (`:181-207`). Session dir from `out_dir`, else `$WATCHDOG_SESSION_DIR/frames`, else `~/.watchdog/current_session` (`:78-91, 112-124`). Restart-safe numbering (`:164-175`). Started by hand with the ROS Python (`:43-46`); needs Pillow in that Python. M5 adds `out_dir` = `records/<episode_id>/frames/` and start/stop under the episode runner. |
| `W/paper_media/isaac_replay/` (`render_replay.py`, `extract_bag.py`, `replay_config.yaml`, README) | **NO** (reference for M5) | Offline re-render inside Isaac 5.1. Not run time, but `render_replay.py` is the only in-repo code that opens a stage, finds the articulation `/World/ur5e`, and uses `isaacsim.*` and Replicator APIs against this exact Isaac version (`:417, 437, 507, 594, 604, 656, 661-665`). Milestone 5's command server and headless launcher will be written against the same API family; this file is the version-matched reference, written by Mohammad, so reading it is unproblematic. |
| `W/paper_media/replay/`, `stories/`, `audit/`, `outputs/`, `requirements.txt`, `README.md` | **NO** | Paper compositor and outputs. The clock rule in `paper_media/README.md:48-54` (wall clock only, `wall_time_iso`) is the rule this plan carries over. |
| `W/watchdog_scorer.py`, `W/observation_scorer.py` | **AS IS** → `I/scoring/` | Offline scorers. Not run time, but asked for. `watchdog_scorer.py` reads `events_watchdog.jsonl` verdicts plus a `gt_*.yaml` label; M6's ground-truth checker will produce a label-shaped file so the scorer keeps working. |
| `W/s4_scorer.py` | **NO** | S4 (executor self-monitoring) only. |
| `W/label_templates/` (`gt_fmb2_s1c1.yaml`, `gt_fmb2_s1c2.yaml`, `gt_fmb2_s1c4.yaml`, `gt_fmb1_s1c1.yaml`, README) | **AS IS** → `I/scoring/label_templates/` | The key list the scorer reads (`gt_fmb2_s1c1.yaml:4-18`) is the contract for M6's `verdict.json` ground-truth block. |
| `W/scripts/preflight_check.sh` | **AS IS** → `I/runner/supervisor/preflight_check.sh` | ROS-graph duplicate-node check with `--kill` (`:1-14`). A supervisor health check (M7). |
| `W/scripts/log_gripper_grasp.py`, `extract_rg2_urdf.py`, `inspect_rg2_urdf_joints.py`, `make_rg2_isaac_urdf.py`, `validate_rg2_urdf.py`, `sweep_graspcenter_offset.py` | **NO** | RG2 URDF and calibration tooling; done work. |
| `W/scripts/tune_physics.py`, `camera_reset.py` | **NO** (reference) | Kit-side scripts run through the kit-mcp bridge; M5 reference for talking to the running Kit. |
| `W/scripts/audit_intruder_leak.py`, `backfill_tp_mode.py`, `make_corrupt_registry.py`, `r4_allow_finger_contact.py`, `c2_drop_open_gripper.py`, `render_session_log.py` | **NO** | Study-specific. |
| `W/isaac/script_editor/*` (`inject_intruder.py`, `inject_scenarios.py`, `s1_c1_inject.py`, `s1_c2_nudge.py`, `spawn_random_intruders.py`, `script_editor_correct_pattern.py`, `Inject_intruder_Script_editor.py`) | **NO** now; candidate for a later fault-injection milestone | Operator injection tools pasted into the Script Editor. `script_editor_correct_pattern.py:1-38` (SimulationContext, `Articulation("/World/ur5e")`, `timeline.play()`) is the pattern M5's command server will follow. |
| `W/reference_watchdog/` | **NO** | Scripted oracle for the paper. |
| `W/analysis/`, `W/paper_results/`, `W/docs/dev/*` | **NO** | Paper analysis, scored batches, dated records. `docs/dev/RUN.md` and `BATCH_RUNBOOK.md` are the operator runbooks; OPERATIONS.md (M7) replaces them for the loop. |
| `W/README.md`, `W/docs/README.md` | **NO** | Study-specific. |

---

## 2. Isaac Sim version

**Isaac Sim 5.1, Kit Python 3.11, installed at `~/isaacsim`.** Determined from files:

- `W/paper_media/isaac_replay/render_replay.py:1` ("runs INSIDE Isaac Sim 5.1") and `W/paper_media/isaac_replay/README.md:3`.
- Install path `~/isaacsim/python.sh` (`render_replay.py:13`; `isaac_replay/README.md:54, 57`).
- Extension namespace is `isaacsim.*` everywhere (`W/isaac/script_editor/script_editor_correct_pattern.py:5-6`, `inject_scenarios.py:534`, `render_replay.py:417, 437, 507, 594, 656, 661-662`); the one `omni.isaac.*` string is a note at `W/docs/dev/VISION_DISCOVERY.md:110`.
- Bytecode: `W/isaac/script_editor/__pycache__/*.cpython-311.pyc` (Kit's Python; Isaac 4.x ships 3.10, 5.x ships 3.11). Everything else is cpython-312, the system ROS Python.
- Outside the repo, read only: `~/isaacsim/VERSION` = `5.1.0-rc.19+release.26219.9c81211b.glbin`; Kit user config `~/.local/share/ov/data/Kit/Isaac-Sim Full/5.1/user.config.json` lists `assembly_room.usd` among recent files (`:509`) and autoloads `omni.kit.mcp` on port 9009 (`:336-337, 524-530`).

Not determinable from files: the version string inside `assembly_room.usd` itself (binary, not read), and whether the stage was first authored in 5.1 or migrated. The stage was re-saved on 2026-11-02 (last WATCHDOG commit), so it has been opened and saved by the current install.

Consequence: Milestone 5 targets Isaac Sim 5.1 APIs (`isaacsim.core.*`, `isaacsim.ros2.bridge`, Replicator), not 6.1. DROIDSTACK's Isaac-6.1-only code is irrelevant to the implementation beyond idea credit.

---

## 3. Where things live today

| Item | File and line |
|---|---|
| Scene load and play | Manual GUI: open `assets/usd/assembly_room.usd`, reset sim time, press Play (`W/README.md:127-128`; `W/docs/dev/BATCH_RUNBOOK.md:60`; `W/docs/dev/RUN.md:156`). No script launches Isaac. The bringup assumes it is already playing (`W/ros2_ws/src/sim_moveit_bringup/launch/sim_moveit_bringup.launch.py:281-288`). Programmatic play exists only in `W/isaac/script_editor/script_editor_correct_pattern.py:20-21` (`timeline.play()`) and the offline replay (`render_replay.py:604, 662-665`). Between runs the stage is reopened discarding changes (`W/isaac/script_editor/s1_c1_inject.py:12-14`). |
| Existing kit MCP bridge | Client: `W/.mcp.json:34-42` (`kit`, stdio, `exec kit-mcp`, `OMNI_KIT_MCP_PORT=9009`); scripted use in `W/ros2_ws/src/robot_backend/test/rg2_gripper_stiffness_sweep.py:84-85, 107-117` (`python -m kit_mcp.client run_python … --port 9009`) and `W/scripts/tune_physics.py:4`. Server: **not in the repo**; an external Kit extension `omni.kit.mcp` under `/home/mohammad/omni-kit-mcp/exts/` (editable install, `~/.local/share/uv/tools/kit-mcp/`), autoloaded by the Kit user config. Built-ins: `run_python`, `put_file`, `get_file`, `status`, `stat_file`, `list_tools`, `list_bridges`, `reload_tools` (`omni-kit-mcp/exts/omni.kit.mcp/omni_kit_mcp/builtins.py:248-306`), NDJSON over TCP. `run_python` is effectively the whole interface. |
| Scene camera frame recorder | `W/paper_media/recorder/frame_recorder.py` (section 1.4). Resolution 1280x720 on `/recorder_rgb`; intrinsics on `/recorder_camera_info`. |
| Bridge `FollowJointTrajectory` server | `W/ros2_ws/src/isaac_trajectory_bridge/isaac_trajectory_bridge/follow_joint_trajectory_server.py`. Node `isaac_follow_joint_trajectory_bridge` (`:28`); action name parameter default `/arm_controller/follow_joint_trajectory` (`:33-35`; launch passes the same, `launch/trajectory_bridge.launch.py:106`); server at `:157-165`; publishes `JointState` on `/joint_command` (`:31, 148`), 6 arm joints only by default (`:73`); subscribes `/joint_states` (`:32, 149-154`); `goal_tolerance` 0.002 rad (`:64`), `goal_time_tolerance_sec` 2.0 (`:665-669`), `publish_rate_hz` 60 (`:40`), `clock_stall_timeout_sec` 5.0 (`:41`). Cancel: `_cancel_callback` returns `ACCEPT` (`:229-231`); the loop checks `is_cancel_requested` each tick, calls `goal_handle.canceled()`, returns `error_code=SUCCESSFUL` with `error_string="Trajectory goal canceled."` (`:301-307`). It publishes **no hold setpoint**; the arm keeps the last `/joint_command` already sent. |
| **Exact cancel service name** | `/arm_controller/follow_joint_trajectory/_action/cancel_goal`, type `action_msgs/srv/CancelGoal` (rclpy action naming; action name from the launch line above). |
| RG2 controller interface | `W/ros2_ws/src/isaac_trajectory_bridge/isaac_trajectory_bridge/rg2_controller.py`. Node `rg2_controller` (`:189`), namespace `/rg2_controller` (`:194`). Services `/rg2_controller/open`, `/close` (`std_srvs/Trigger`, `:348-353`), `/set_position` (`SetGripperPosition`, master joint angle in rad, `:354-359, 726-732`). Action `/rg2_controller/guarded_move` (`GripperGuardedMove`, `:362-368`; modes `width` and `torque`, `:915-959`; **no cancel callback**, so cancels are rejected and `:994-997` is unreachable; busy lock `:812-827`). Publishes `/joint_command` (gripper joints, `:317, 1207-1212`) and `/gripper_width_true` (`:318`). Constants: `DEFAULT_OPEN_POSITION = -0.558505` (`:66`), `DEFAULT_CLOSE_POSITION = 0.7731809` (`:67`; both launch files override with `0.785398`: `trajectory_bridge.launch.py:49`, `sim_moveit_bringup.launch.py:744`), `DEFAULT_FINGER_LINK_OFFSET_M = 0.021` (`:72`), `DEFAULT_HARD_FORCE_THRESHOLD = 0.5` (`:157`), `DEFAULT_EFFORT_NOISE_FLOOR = 0.03` (`:113`), `DEFAULT_EFFORT_CONTACT_TICKS = 3` (`:163`), `CONTACT_STOP_REASONS = {"force_threshold", "effort_plateau"}` (`:184`). Width table `config/rg2_calibration_table.csv` loaded at `:382-443` (`open_to_close` rows only). These are copied, never re-derived. |
| Current halt client target | `W/ros2_ws/src/robot_backend/robot_backend/watchdog/halt.py`: cancel-all `action_msgs/srv/CancelGoal` with an all-zero goal id (`:60-62`) to `/move_action/_action/cancel_goal` (`:117-121`); unused `ActionClient(MoveGroup, "/move_action")` at `:116`; hardcoded (`:59, 116, 119`), constructor takes no name (`:15-21`). Callers: `engine.py:498` on a fire with `halt_on_fire`, `watchdog_mcp_server.py:2689` (`halt_execution`). It never targets the bridge or the gripper. |
| Session directory | `W/ros2_ws/src/robot_backend/robot_backend/watchdog/session.py:129-142` (env, pointer `~/.watchdog/current_session`, fresh); manifest `:290-397`; bringup writes the pointer and manifest (`sim_moveit_bringup.launch.py:127, 245, 280`). |

---

## 4. Every place WatchDog receives executor activity or plan knowledge

All paths under `W/ros2_ws/src/robot_backend/robot_backend/` unless given in full. For each, how it is removed or gated in Milestone 4.

| # | Channel | Where | Removal or gate |
|---|---|---|---|
| 1 | **Task file `steps:` block** loaded by `_load_task_steps` (`mcp_servers/watchdog_mcp_server.py:103-139`), collapsed per object by `_load_task_objects` (`:142-192`), cached in `WatchDogContext.task_objects` (`:413-418`); `get_contract` (`:1483-1548`) returns per-object specs and, on an unknown object, lists the task's objects (`:1530-1535`). | **Remove.** The requirements file (`config/requirements/<assembly>.yaml`) carries acceptance contracts and invariants keyed by part, with no steps. `_load_task_steps` is replaced by a requirements loader; `get_contract` keeps serving per-part clauses from it. |
| 2 | **`{{STEPS}}` and `{{STEP_PLAN}}` in the task template** (`W/agents/prompts/watchdog_task.txt:9-18`; `task_render.py:23-59` `format_steps`; `role_context.py:257`). | **Remove** both placeholders from the template; the requirements file has no `steps`/`step_plan` keys, so `render_task` would fail loudly on a leftover placeholder (`task_render.py` hard failure, `role_context.py:218-220`). |
| 3 | **`/watchdog/events` subscription** in the recorder (`watchdog/recorder.py:19, 360-362`); `_on_event_msg` stores every type (`:499-561`), `begin_step`/`end_step` set `current_step_id` (`:535-538`); executor-origin types `begin_step, end_step, run_ended, skill_start, skill_end, phase` (`events/schema.py:610-612`); every `skill_start` carries the skill's keyword arguments (`events/decorator.py:32-61, 74-77`). Surfaced by `get_events` (`:2328-2345`), `get_evidence` events channel (`:2409-2413`), `get_execution_state` (`:1550-1574`), `wait_for_event` (`:2209-2296`), `available_event_types` (`:2307-2326`), and the `current_step` stamp on `look`, `capture_frames`, `halt_execution`. | **Gate at the subscription.** The recorder keeps a subscription only for WatchDog-origin types (`monitor_*`, `verdict`, `look` with `role=watchdog`, `watchdog_ready`, `halt`) and drops any envelope whose type is in the executor-origin set, even if something publishes one. `current_step_id` and `current_step()` are deleted; the step stamp on tools goes. `get_execution_state` is rewritten to report physical state only (motion active, gripper width, held part from `part_lifted`). `available_event_types` lists physical events plus WatchDog-origin types. |
| 4 | **Engine coupling to skill events**: recorder calls `engine.on_skill_event` (`recorder.py:539-556`); `MonitorEngine.on_skill_event` tracks the held object from `grasp_skill`/`release_skill` and auto-disarms `relative_frame` monitors on release (`watchdog/engine.py:371-447`); `held_object_id()` feeds fire attribution (`:554-568`) and arm provenance (`watchdog_mcp_server.py:849-890`). | **Replace the feed, keep the behaviour.** `on_skill_event` becomes `on_physical_event`, fed by `/cell/physical_events`: `part_lifted` sets the held object, `part_released` clears it and runs the same `_release_disarm_slip_monitors` (`:422-447`), so the release auto-disarm rule survives unchanged. |
| 5 | **`signal_ready(scope="step"|"phase")`** (`watchdog_mcp_server.py:2719-2799`; `ready.py:41-87`; orchestrator `node.py:364-380`). | **Remove** the `step` and `phase` scopes from the tool; keep `run` (warm-buffer check `:2754-2794`) and `abort`. The orchestrator keeps its scopes for compatibility but nothing on the WatchDog side can name a step. |
| 6 | **Server-side run termination** reads `run_ended` (`_resolve_observed_terminal_status` `:500-525`; `recorder.observed_terminal_status` `recorder.py:675-710`; `_observe_drained_events` `:528-555`) and finalizes after two silent 180 s waits (`_MAX_CONSECUTIVE_TIMEOUTS=2` `:459`, `_timeout_or_finalize` `:1298-1430`, default `timeout_sec=180` `:2221`). `steps_covered` only fills from `wait_for_event` drains (`:528-555, 1347`). | **Replace.** The episode runner publishes an `episode_end` notice on `/cell/watchdog_notices` (or a dedicated service) and the server finalizes on it; the two-silence finalize is removed, since an hour-long quiet episode must not self-terminate. `steps_covered` and the `clause_results[].step_id` check in `record_run_verdict` (`:2994-3010`) go; the acceptance inspection tool replaces `record_run_verdict`. |
| 7 | **Shared session directory**: executor files `events_motion/gripper/scene.jsonl` (`session.py:164-174`), `mcp_calls_executor.jsonl` (`role_context.py:296-326, 377-401`), `exec_look_*` frames (`vision/look.py:394`), `registry_provenance_executor.json`, `agent_config_executor.json`. No WatchDog tool reads them, but `Read` is only denied by settings. | **Separate directories.** The WatchDog session dir is `records/<episode_id>/watchdog/`; the planner and authoring logs go to `records/<episode_id>/executor/`. The WatchDog settings keep `Read`, `Bash`, `Glob`, `Grep` denied (`W/watchdog.settings.json:13-37`). |
| 8 | **The executor's `look` events** published with `exec_look_` prefix (`mcp_servers/scene_mcp_server.py:517-583`) reach the recorder as non-self `look`. | Covered by #3 (dropped at the subscription) and by the planner contract: the planner publishes nothing on `/watchdog/events`. |
| 9 | **Gate records** in the orchestrator log (`orchestrator/` under the session dir, `session.py:187-192`), including `step_id` and `phase` strings from `request_go`. | The planner template never calls `request_go` with a step id; the episode runner calls it once per episode with the episode id. The log moves to `records/<episode_id>/orchestrator/`. |
| 10 | **Prompt text** that describes the executor's tools and flow (`watchdog_system_s3.txt:333-347` "Reading the event stream", `:349-416`, `:418-646`). | **Rewrite** (section 1.2). |

Executor-side files that are deleted in M6 (motion/gripper/scene servers, skills) stop publishing `begin_step`, `end_step`, `run_ended`, `skill_*`, `phase` at the source as well.

---

## 5. Proposed INTEGRATION layout, with adjustments

The target layout from the prompt is kept, with these adjustments:

| Adjustment | Reason |
|---|---|
| **Keep `I/agents/` at the repo root** holding `agent_harness/`, `config/`, `prompts/`, `tasks/` (WatchDog task only). The prompt layout puts prompts under `config/`; this plan leaves them in `agents/prompts/` and puts only the per-role MCP and settings files under `config/mcp/` and `config/settings/`. | The WatchDog server locates its prompts by walking up from its own file to a directory containing both `agents/` and `ros2_ws/` (`role_context.py:97-110`) and imports `agent_harness` from there (`:232-236`). Moving prompts would require editing `role_context.py` in M2, which M2 forbids. Can be revisited in M4 if wanted. |
| **`config/robot/`** holds `ur5e.urdf`, `ur5e.xrdf`, `ur5e.yaml`, `ur5e_rmpflow_common_v2.yaml` (from `W/config/`). | Keeps `W/config/` content without colliding with the new `config/` tree; the xrdf is the cuRobo input. |
| **`config/mcp/watchdog.mcp.json`** and **`config/settings/watchdog.settings.json`** exist from M2 (copied, paths rewritten). The executor pair appears in M6. | The unchanged WatchDog check in M2 needs a launchable config in the new tree. |
| **`runner/run_watchdog.sh`** exists from M2 (copied, config path rewritten). | Same reason; M6 folds it into the episode runner. |
| **`runner/recorder/frame_recorder.py`** (from `W/paper_media/recorder/`) rather than a ROS package. | It is a standalone rclpy script run with the ROS Python; keeping it a script preserves the "invisible to both agents, in no MCP config" property its docstring relies on (`:2-7`). |
| **`scoring/`** holds `watchdog_scorer.py`, `observation_scorer.py`, `label_templates/`. | Not in the prompt layout; added so the scorers are available without living at the root. |
| **`ros2_ws/src/cell_planner_msgs/`** also defines the physical-event message, the notice message and the ruling message, not only planner-side types. | One interfaces package for all new cell messages keeps the dependency graph flat; the name from the prompt is kept. |
| **`executor/workspace/`** and **`records/`** are gitignored, as specified; `I/.gitignore` already ignores `.venv`, `build/`, `install/`, `log/`. | `records/` will also need `*.mcap`, `*.jpg` patterns; added in M2. |
| **`ros2_ws/.venv`** recreated by Mohammad from `ros2_ws/requirements.txt`. | The MCP servers re-exec into it (`mcp_servers/__init__.py:47-80`). |

Resulting tree after M2 (new packages appear in their milestones):

```
I/
  docs/                       INTEGRATION_SURVEY, BUILD_PLAN, PROVENANCE, NEXT_STEPS (+ later ARCHITECTURE, INDEPENDENCE, PLANNER_CONTRACT, OPERATIONS)
  agents/                     agent_harness/, config/watchdog_agent_s3.yaml, prompts/{watchdog_system_s3.txt, watchdog_task.txt}, tasks/fmb_assembly_2.watchdog.yaml
  config/
    mcp/watchdog.mcp.json     settings/watchdog.settings.json
    robot/                    ur5e.urdf, ur5e.xrdf, ur5e.yaml, ur5e_rmpflow_common_v2.yaml
    (M3+) cell.yaml  layouts/  requirements/  tasks/  gripper/  parts/
  assets/                     usd/, ur5e/, ur_assets/
  generated_urdf/
  ros2_ws/                    BUILD_NOTES.md, colcon_defaults.yaml, requirements.txt, src/<the 10 copied packages>
  runner/                     run_watchdog.sh, recorder/frame_recorder.py, supervisor/preflight_check.sh  (M6/M7: episode_runner/, supervisor/, index/)
  scoring/                    watchdog_scorer.py, observation_scorer.py, label_templates/
  .mcp.json                   developer kit bridge
  (M3+) ros2_ws/src/{cell_planner_msgs, cell_planning_service, cell_planning_curobo, cell_physical_events}
  (M5)  isaac/{cell_command_server, headless}
  (M6)  executor/{authoring_mcp, planner_template, workspace/}
  (M7)  records/
```

---

## 6. Open questions for Mohammad

**Answered 2026-11-03.** 1: copy from the latest commit (`595ede33…`). 2: yes, the path rewrites only. 3: copy whole, prune in M6. 4: cuRobo `v0.7.8`; the xrdf describes the same UR5e. 5: `planner.backend: moveit` default. 6: halt = bridge cancel, no hold published, bridge untouched. 7: gripper left alone on halt. 8: `/recorder_rgb` is present in the stage. 9: Isaac 5.1 now; Mohammad can move to 6.1 later, so M5 code should isolate version-specific calls. 10: the session location must change automatically per episode, so the runner sets `WATCHDOG_SESSION_DIR` explicitly in every process it launches and also writes the pointer file for anything started by hand. 11: scorers are **not copied**; new scorers come after the integration shows results, so the `scoring/` entry in section 5 is dropped. 12: `agents/` stays at the root; may be changed in M4 if it is better.

Original questions, kept for the record:

1. **Commit to record.** PROVENANCE.md will cite WATCHDOG `595ede33…` (current `main`). Confirm, or name the commit to copy from. The stage USD differs from `cb973bbf…`.
2. **Path rewrites in M2.** The unchanged WatchDog check needs `I/config/mcp/watchdog.mcp.json` to point at `I/ros2_ws/install/setup.bash`, `I/agents/tasks/…`, `I/agents/config/…`, and `agents/config/watchdog_agent_s3.yaml` `cwd` rewritten. Those are the only edits planned in M2 besides file placement. Agree?
3. **robot_backend pruning.** Copy the whole package in M2 and delete the executor-side modules in M6 (recommended, keeps M2 verifiable), or omit them already in M2?
4. **cuRobo tag.** The prompt says `v0.7.8` unless you say otherwise. `W/config/ur5e.xrdf` and the RMPflow yaml exist; confirm they describe the same UR5e as `ur5e_ros.urdf` (same base and tcp offsets).
5. **Planning backend default.** `planner.backend: moveit` as the default for M3, since MoveIt is installed and configured today?
6. **Halt semantics.** The bridge cancel leaves the arm at its last setpoint and publishes no hold. Is that acceptable as "halt", or should the halt path also publish the current `/joint_states` position as a hold on `/joint_command` (an edit to the bridge, which this plan otherwise never touches)?
7. **Gripper halt.** `guarded_move` cannot be cancelled (no cancel callback). Should halt also call `/rg2_controller/open` or leave the gripper alone as today (`halt.py` sends nothing to the gripper)?
8. **Recorder camera.** `/recorder_rgb` is created inside the stage. Confirm it is present in the current `assembly_room.usd` and whether the headless launch (M5) should rely on it or add a render product by script.
9. **Isaac paths.** Confirm `~/isaacsim` (5.1.0) is the install the headless launcher should use, and that `/home/mohammad/omni-kit-mcp` stays the developer bridge (it is not needed by the cell at run time).
10. **Session root.** Keep `~/.watchdog/current_session` as the pointer mechanism (the recorder, servers and bringup all read it), with the episode runner writing it per episode, or set `WATCHDOG_SESSION_DIR` explicitly in every launched process?
11. **Scorer retention.** Keep `watchdog_scorer.py` and `observation_scorer.py` under `scoring/` as asked, even though the plan-blind WatchDog will not produce step verdicts? The scorer's step-level columns will be empty.
12. **`agents/` location.** Accept the adjustment in section 5 (prompts stay under `agents/prompts/`), or should M4 edit `role_context.py` so prompts can live under `config/`?
