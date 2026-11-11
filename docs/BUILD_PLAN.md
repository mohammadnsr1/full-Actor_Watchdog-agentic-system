# Build plan (Milestone 1, read only)

Version 2, written 2026-11-10 against the v2 build prompt (hardware platform, Python 3.10 / ROS 2 Humble baseline, two cameras with separate roles, Humble container). Version 1 was written 2026-11-03; its content is kept below where it still holds, with milestone numbers renumbered to the v2 scheme. Nothing was built, run, or changed in WATCHDOG or DROIDSTACK for this version; the audit in section 5 was done with file reads and text searches only.

Paths: `W/` = `/home/mohammad/watchdog/WatchDog_V1`, `I/` = `/home/mohammad/full-Actor_Watchdog-agentic-system`. Line numbers for copied files are from the `I/` copies, which are byte-identical to `W/` except where PROVENANCE.md lists an edit.

## 0. Where the build stands against the v2 milestones

| v2 milestone | v1 name | Status on 2026-11-10 |
|---|---|---|
| M1 plan | M1 | **This document.** STOP after it. |
| M2 bring WatchDog in | M2 | Done (commit `9314d3e`), checked live by Mohammad with an unchanged S3 session. |
| M3 platform backends, names in config, Humble container | (new) | Built and offline-tested 2026-11-10 (see `docs/NEXT_STEPS.md`); awaiting Mohammad's live check and container build. |
| M4 planning service | v1 M3 | Built and offline-tested (commit `56133b4`): `cell_planner_msgs`, `cell_planning_service` (MoveIt plan-only), `cell_planning_curobo`, `config/cell.yaml`, halt retargeted to the bridge cancel. Mohammad's live check (`docs/NEXT_STEPS.md`, v1 text) has not been reported yet. What v2 adds on top: the halt client must go through the motion backend's cancel (section 5.4 item H1), the 3.10/Humble review in section 5 applies to the three new packages, and the UR-with-RG6 cuRobo robot config is missing. These are finished in M4 after M3 lands. |
| M5 physical events, plan-blind WatchDog | v1 M4 | Built and offline-tested 2026-11-11; awaiting Mohammad's live check (`docs/NEXT_STEPS.md`). |
| M6 command server, headless Isaac, sim cameras, recording | v1 M5 | Written and offline-tested 2026-11-11 (Kit code compiled, not yet run inside Kit); awaiting Mohammad's check. |
| M7 planner contract, authoring tools, episode runner | v1 M6 | Not started. |
| M8 supervisor, operations, index | v1 M7 | Not started. |

**WATCHDOG commit note (unchanged from v1).** `W/.git/refs/heads/main` reads `595ede3354dc40c916b4bcd464ebb2db1b014b32`, three commits after the `cb973bbf…` in the survey; PROVENANCE.md records `595ede33…` per Mohammad's answer to v1 question 1.

---

## 1. WATCHDOG inventory

Legend: **AS IS** = byte copy, same relative path. **ADAPT** = copy, then edit in the named milestone. **NO** = not copied (reason given). Everything marked M2 is already in `I/`.

### 1.1 ROS 2 workspace `W/ros2_ws/` → `I/ros2_ws/`

| Source | Verdict | Notes |
|---|---|---|
| `src/robot_backend/` (whole package) | **ADAPT** (M2 copied whole; prune in M7) | Copied whole so the unchanged WatchDog session runs: the WatchDog server imports `vision`, `role_context`, `events`, `registry`, `watchdog/*`, `contracts/*`, `session` from the same package. Executor-side modules to retire in M7 once the planner template replaces them: `mcp_servers/motion_mcp_server.py`, `gripper_mcp_server.py`, `scene_mcp_server.py`, `executor_monitor_mcp_server.py`, `skills/*`, `ros_moveit_backend/*`, `interfaces/*`, `cell/gate.py`. Keep: `nodes/scene_tf_sync_node.py` (executor pose-query channel and `/sync_scene_objects`), `utils/rg2_geometry.py` (validated width table reader), everything WatchDog-side and shared. In v2 the sim gripper backend (M3) wraps the RG2 controller directly, so `ros_moveit_backend/gripper.py` is reference only. `package.xml` was fixed in v1 M3 (`action_msgs`, `cell_sync_interfaces`, `python3-yaml`). |
| `src/robot_backend/config/model_registry.yaml` | **AS IS** in M2 (inside the package) | 867 lines; `cell:` at `:112` (`gripper.effort_joint_name: finger_joint` `:114`, `tcp_frame: tcp` `:119`, `base_frame: world` `:125`), `acm_allowed_object_pairs:` at `:138`, `objects:` at `:227` with per-object `tf_frame`/`lookup_frame` (`:232-233, 288-289, 347-348, 417-418, 444`). Split in M5 into `config/parts/watchdog_parts.yaml` and `config/parts/executor_parts.yaml`; the `cell:` block moves to the platform config in M3 (section 5.1). Loaders: `registry/loader.py:106`, `watchdog/recorder.py:144-162`, `contracts/loader.py:35-101`, `nodes/scene_tf_sync_node.py:144, 498-509`, `mcp_servers/role_context.py:191-206`, bringup launch `:95-107, 122`. |
| `src/robot_backend_interfaces/` | **AS IS** | `GetObjectPose.srv`, `GetObjectPoses.srv`, `SyncSceneObjects.srv`. The executor pose-query channel keeps these service types; in M3 the server behind them becomes the camera1-backed pose backend. |
| `src/cell_orchestrator/` | **AS IS** in M2; **ADAPT** in M3 (names) and M7 (episode end) | `/cell/request_go` and `/cell/signal_ready` are literals (`cell_orchestrator/node.py:217, 220`). |
| `src/cell_sync_interfaces/` | **AS IS** | `RequestGo.srv`, `SignalReady.srv`. |
| `src/isaac_trajectory_bridge/` | **AS IS** | Sim-only package. `follow_joint_trajectory_server.py`, `rg2_controller.py`, `calibrate_rg2_width.py`, `launch/trajectory_bridge.launch.py`, `config/rg2_calibration_table.csv`. Never edited: the RG2 constants and the width table live here. Its topic and action names are already node parameters (section 5.1); the sim platform config feeds them through the launch file. |
| `src/isaac_trajectory_bridge_interfaces/` | **AS IS** | `SetGripperPosition.srv`, `GripperGuardedMove.action`. |
| `src/sim_moveit_bringup/` | **AS IS** in M2; **ADAPT** M3 (names from config), M4 (done: planning-service include), M6 (bag topics) | Sim-only. The launch imports `robot_backend.watchdog.session` and `robot_backend.registry` directly (`:61, 95-96, 234, 244`). |
| `src/ur5e_moveit_config/` | **AS IS** | Jazzy-format MoveIt config (`ompl_planning.yaml:1` uses the `planning_plugins` list and `default_planning_request_adapters/*` names, which are MoveIt 2.10; Humble's MoveIt 2.5 wants `planning_plugin` and `default_planner_request_adapters/*`). Used on sim only. The hardware MoveIt config, if MoveIt is used on hardware at all, comes from the UR driver's own `ur_moveit_config` (question 14). |
| `src/robot_description/`, `src/onrobot_rg_description/` | **AS IS** | URDF (`tcp` link `urdf/ur5e_ros.urdf:204`, RG2 links from `:212`); RG2 and RG6 xacros and meshes. The RG6 xacro is the starting point for the hardware URDF. |
| `BUILD_NOTES.md`, `colcon_defaults.yaml`, `requirements.txt` | **AS IS** | `colcon_defaults.yaml` pins `-DPython3_EXECUTABLE=/usr/bin/python3` (valid on both distros). `requirements.txt` pins `numpy<2`, `mcp<2` (both install on 3.10). |

### 1.2 Agents, prompts, configs, launchers

Unchanged from v1 except milestone numbers: `agents/agent_harness/` (AS IS), `agents/config/watchdog_agent_s3.yaml` (ADAPT: paths in M2, prompt names in M5), `agents/prompts/watchdog_system_s3.txt` (ADAPT in M5: keep Authority `:9-19`, evidence discipline `:49-75` with the agent-chosen-threshold rule `:57-59, 696-699`, verdict meanings `:648-692`; remove the control loop `:418-646`, "What is expected to move" `:349-371`, "Steps on and off the plan" `:373-416`), `agents/prompts/watchdog_task.txt` (ADAPT in M5: drop `{{STEPS}}`/`{{STEP_PLAN}}` `:9-18`), `agents/tasks/fmb_assembly_2.watchdog.yaml` (ADAPT in M5 → `config/requirements/fmb_assembly_2.yaml`), `W/agents/tasks/fmb_assembly_2.executor.yaml` (ADAPT in M7 → `config/tasks/fmb_assembly_2.yaml`, never under `agents/`), `config/mcp/watchdog.mcp.json` and `config/settings/watchdog.settings.json` (M2, paths only; tool list in M5; a hardware variant in M3), `runner/run_watchdog.sh` (M2), `.mcp.json` (AS IS). Not copied: S1/S2/S4 and executor prompts, configs, MCP files, `run_executor.sh`, transcripts, `agents/tests`.

### 1.3 Assets, robot configs, URDFs

Unchanged from v1: `assets/` (stage, robot USD, UR description), `config/robot/` (`ur5e.urdf`, `ur5e.xrdf`, `ur5e.yaml`, `ur5e_rmpflow_common_v2.yaml`), `generated_urdf/`. v2 note: the stage has one depth camera and the recorder camera; there is no wrist camera prim in any file in the repo (question 5).

### 1.4 Recording, scorers, labels, tools

Unchanged from v1 except: scorers are **not copied** (Mohammad's v1 answer 11; new scorers after results exist). `runner/recorder/frame_recorder.py` (M2 copy of `W/paper_media/recorder/frame_recorder.py`) is the high-resolution scene camera frame recorder; `runner/supervisor/preflight_check.sh` is the duplicate-node check. `W/paper_media/isaac_replay/render_replay.py` and `W/isaac/script_editor/script_editor_correct_pattern.py` are the version-matched Isaac 5.1 API references for M6.

---

## 2. Isaac Sim version

**Isaac Sim 5.1, Kit Python 3.11, installed at `~/isaacsim`** (unchanged from v1; evidence: `W/paper_media/isaac_replay/render_replay.py:1`, `isaacsim.*` namespaces throughout, cpython-311 bytecode under `W/isaac/script_editor/__pycache__/`, and outside the repo `~/isaacsim/VERSION` = `5.1.0-rc.19`). Mohammad may move to 6.1 later (v1 answer 9), so M6 isolates version-specific calls in one module. Kit's Python is 3.11, so Isaac-side code (`isaac/`) is exempt from the 3.10 rule by the prompt; the 3.10 baseline applies to every shared ROS package and to `runner/`, `executor/`.

---

## 3. Where things live today

| Item | File and line |
|---|---|
| Scene load and play | Manual GUI: open `assets/usd/assembly_room.usd`, reset sim time, press Play (`W/README.md:127-128`, `W/docs/dev/RUN.md:156`). No script launches Isaac; the bringup assumes it is playing (`sim_moveit_bringup.launch.py:281-288`). Programmatic play pattern: `W/isaac/script_editor/script_editor_correct_pattern.py:20-21`. |
| Kit MCP bridge | Client: `.mcp.json` (`kit`, `OMNI_KIT_MCP_PORT=9009`). Server: external extension `omni.kit.mcp` under `/home/mohammad/omni-kit-mcp/exts/` (not in either repo), `run_python` is the whole interface. Developer tool only; the cell does not use it at run time. |
| Scene camera frame recorder | `runner/recorder/frame_recorder.py`: topic `/recorder_rgb` (`:62`), intrinsics `/recorder_camera_info`, 1280x720, JPEG q95, 10 Hz, `frames.jsonl` with `wall_time_iso`; session dir from `out_dir`, `$WATCHDOG_SESSION_DIR`, or `~/.watchdog/current_session` (`:78-91, 112-124`). |
| Bridge `FollowJointTrajectory` server | `isaac_trajectory_bridge/follow_joint_trajectory_server.py`: node `isaac_follow_joint_trajectory_bridge`; parameters `joint_command_topic` (`:31`), `joint_state_topic` (`:32`), `action_name` default `/arm_controller/follow_joint_trajectory` (`:33-35`); arm joint list literal `:17-22`; cancel accepted (`:229-231`), goal ends CANCELED with `error_code=SUCCESSFUL` and no hold setpoint (`:301-307`). |
| **Exact cancel service name (sim)** | `/arm_controller/follow_joint_trajectory/_action/cancel_goal`, `action_msgs/srv/CancelGoal`. |
| RG2 controller interface | `isaac_trajectory_bridge/rg2_controller.py`: node `rg2_controller` (`:189`); parameters `joint_state_topic`, `joint_command_topic`, `service_namespace` (`:192-194`); services `<ns>/open`, `<ns>/close`, `<ns>/set_position`; action `<ns>/guarded_move` (no cancel callback); publishes `/joint_command` and the literal `/gripper_width_true` (`:318`). Constants `DEFAULT_OPEN_POSITION = -0.558505` (`:66`), `DEFAULT_CLOSE_POSITION = 0.7731809` (`:67`; launches override with `0.785398`), `DEFAULT_FINGER_LINK_OFFSET_M = 0.021` (`:72`), `DEFAULT_HARD_FORCE_THRESHOLD = 0.5` (`:157`), noise floor `0.03` (`:113`), contact ticks `3` (`:163`); width table `config/rg2_calibration_table.csv`. Copied, never re-derived. |
| Current halt client target | `robot_backend/watchdog/halt.py:21-23` (after v1 M3): `BRIDGE_CANCEL_SERVICE`, `MOVE_GROUP_CANCEL_SERVICE` literals, default targets `(BRIDGE_CANCEL_SERVICE,)`; the server reads `watchdog.halt.cancel_targets` from `config/cell.yaml` (`watchdog_mcp_server.py` HaltClient construction). Callers: `engine.py:498`, `watchdog_mcp_server.py` `halt_execution`. |
| **Current camera topics and frames** | One Isaac depth camera. Extrinsics: `sim_moveit_bringup/config/cameras.yaml` (`camera_depth`, parent `world`, child `camera_depth_optical_frame`, `xyz [0.25, -0.2, 1.7]`, `quat_xyzw [0, 0.99863, -0.05234, 0]`), broadcast by `sim_moveit_bringup/static_camera_tf_broadcaster.py:76-110` through `launch/camera_tf.launch.py`. Topics read by `look()`: `/camera/depth/rgb` (or `/compressed`) `robot_backend/watchdog/image_buffer.py:56-57`, depth `/camera/depth/image_raw` `:66`, intrinsics `/camera/depth/camera_info` then `/camera_info` `:75-76`, dormant cloud `/camera/depth/points` `:91`; override env `WATCHDOG_IMAGE_TOPICS`. Bag topics: `sim_moveit_bringup.launch.py:40, 46`. Recorder camera: `/recorder_rgb`, frame `sim_camera` (stage-internal). No wrist camera exists. |
| Session directory | `robot_backend/watchdog/session.py:129-142` (env, pointer file, fresh), manifest `:290-397` (`cell_use_sim_time: True` literal `:363`). |

---

## 4. Every place WatchDog receives executor activity or plan knowledge

Unchanged from v1 (ten channels), now scheduled for **M5**. Paths under `robot_backend/robot_backend/`.

| # | Channel | Where | Removal or gate (M5) |
|---|---|---|---|
| 1 | Task file `steps:` | `mcp_servers/watchdog_mcp_server.py:103-139` (`_load_task_steps`), `:142-192`, `:413-418`, `get_contract :1483-1548` | Remove; requirements loader replaces it. |
| 2 | `{{STEPS}}`, `{{STEP_PLAN}}` | `agents/prompts/watchdog_task.txt:9-18`, `agents/agent_harness/task_render.py:23-59`, `role_context.py:257` | Remove the placeholders. |
| 3 | `/watchdog/events` subscription | `watchdog/recorder.py:19, 360-362, 499-561`; executor types `events/schema.py:610-612`; skill kwargs `events/decorator.py:32-61` | Gate at the subscription: keep WatchDog-origin types only; delete `current_step_id`. |
| 4 | Engine on skill events | `recorder.py:539-556`, `watchdog/engine.py:371-447`, `:554-568` | `on_physical_event` fed by `part_lifted`/`part_released`; release auto-disarm (`:422-447`) kept verbatim. |
| 5 | `signal_ready(scope=step|phase)` | `watchdog_mcp_server.py:2719-2799`, `watchdog/ready.py:41-87`, orchestrator `node.py:364-380` | Remove the `step`/`phase` scopes. |
| 6 | Termination on `run_ended` | `watchdog_mcp_server.py:454-459, 500-555, 1298-1430`, `recorder.py:675-710` | Episode-end notice from the runner; remove the two-silence finalize. |
| 7 | Shared session directory | `session.py:164-192`, `role_context.py:296-326` | `records/<episode_id>/watchdog/` vs `records/<episode_id>/executor/`. |
| 8 | Executor `look` events | `mcp_servers/scene_mcp_server.py:517-583` | Covered by 3; the planner publishes nothing on `/watchdog/events`. |
| 9 | Gate records with `step_id` | orchestrator log, `session.py:187-192` | Runner calls `request_go` once per episode. |
| 10 | Prompt text about executor tools | `watchdog_system_s3.txt:333-416, 418-646` | Rewrite. |

v2 adds an eleventh channel: **camera1 poses**. On sim today every pose comes from the same TF (Isaac truth); on hardware camera1 is the executor's pipeline. The WatchDog pose interface (M3) reads only `/camera2/poses`; camera1 poses reach WatchDog only through the cue tool (M5), and any verdict touching them is flagged `shared_source: true`.

---

## 5. Portability audit

Scope: the ten copied packages, the three v1-M3 packages, `runner/`, `agents/agent_harness/`. Searched with text patterns for literal ROS names, Python features newer than 3.10, APIs that differ between Humble and Jazzy, and simulator-truth assumptions. Lines are from the `I/` copies.

### 5.1 Hard-coded topic, frame, joint, action, and service names

Handling rule for all of them: one shared reader, `cell_platform_interfaces/platform_config.py` (M3), loads `config/platforms/<platform>.yaml` chosen by `config/cell.yaml: platform: sim | hardware` (or `CELL_PLATFORM`). Code asks it for names; launch files pass the same values as node parameters. Nodes whose parameter defaults are literals keep the parameter but get the value from the launch file, so no default survives in code.

| File:line | Literal | Handling |
|---|---|---|
| `robot_backend/watchdog/recorder.py:18-19` | `/joint_states`, `/watchdog/events` | Both from platform config (`topics.joint_states`, `topics.watchdog_events`); `recorder.py:358, 361` subscribe through the resolved names. M3. |
| `robot_backend/config/model_registry.yaml:114, 119, 125` | `effort_joint_name: finger_joint`, `tcp_frame: tcp`, `base_frame: world` | Move the `cell:` block into the platform config (`gripper.effort_joint`, `frames.tcp`, `frames.base`); `contracts/loader.py:172-173` and `recorder.py:219-244` read it from there. On hardware `gripper.effort_joint` is absent and the signal list says `finger_effort: missing` or `proxy`. M3. |
| `robot_backend/watchdog/image_buffer.py:56-57, 66, 75-76, 91` | `/camera/depth/rgb`, `/camera/depth/image_raw`, `/camera/depth/camera_info`, `/camera_info`, `/camera/depth/points` | From `config/cameras/<platform>.yaml`; `look()` takes a camera name (`camera1`, `camera2`) and resolves topics from it. `WATCHDOG_IMAGE_TOPICS` override kept. M3 (names), M5 (two-camera `look`). |
| `robot_backend/watchdog/halt.py:21-23` | bridge and `/move_action` cancel services | Primary target = `motion.cancel_service` from platform config; `/move_action` stays an optional extra under `watchdog.halt.extra_cancel_targets`. M4 (section 0). |
| `robot_backend/events/bus.py:96` | `/watchdog/events` | `topics.watchdog_events`. M3. |
| `robot_backend/cell/gate.py:45`, `robot_backend/watchdog/ready.py:101`, `cell_orchestrator/cell_orchestrator/node.py:217, 220` | `/cell/request_go`, `/cell/signal_ready` | `services.request_go`, `services.signal_ready`; the orchestrator gets them as parameters from its launch entry. M3. |
| `robot_backend/nodes/scene_tf_sync_node.py:66-70` | `/apply_planning_scene`, `/sync_scene_objects`, `/get_object_pose`, `/get_object_poses`, `/reset_tf` | Already parameters (`:155-160`); launch passes values from `services.*`. M3. |
| `robot_backend/ros_moveit_backend/gripper.py:61-77, 308-312`, `skills/gripper_skills.py:74`, `ros_moveit_backend/motion.py:38-49, 864`, `ros_moveit_backend/scene.py:28-31` | `/rg2_controller/*`, `/move_action`, `/compute_ik`, `/joint_states`, `/check_state_validity`, MoveIt scene services, arm joint list, `tcp` | Executor-side modules retired in M7. Not ported. The sim gripper backend (M3) takes `gripper.services.*` and `gripper.action` from the sim platform config instead. |
| `robot_backend/interfaces/motion_backend.py:28`, `vision/observation.py:188-200`, `skills/pose_query_skills.py:94` | `tcp`, `world` | `observation.py` rejects any frame but `world`; becomes `frames.base` from platform config (WatchDog side, kept). The other two are executor-side, retired. M3 / M7. |
| `isaac_trajectory_bridge/follow_joint_trajectory_server.py:17-22, 31-34`, `rg2_controller.py:55-65, 192-194, 318`, `launch/trajectory_bridge.launch.py:97-106, 127-129` | arm joint names, `/joint_command`, `/joint_states`, action name, `/rg2_controller`, `/gripper_width_true` | Sim-only package; parameters already exist except the joint list and `/gripper_width_true`. The sim platform config is the single source and the launch file reads it (M3). The literal joint list and width topic stay in the sim-only node and are recorded here as sim-only literals, since the file is never edited. |
| `sim_moveit_bringup/launch/sim_moveit_bringup.launch.py:24, 40, 46, 315, 556, 585, 590`, `config/cameras.yaml`, `pointcloud_workspace_filter.py:21-23`, `workspace_bounds_publisher.py:54, 61, 63-64`, `print_current_collisions.py:20, 24, 76-81` | bag topics, camera topics, `world`, `base_link`, joint lists | Sim-only launch and tools. Bag topic lists come from the platform config in M6 (recording); camera entries from `config/cameras/sim.yaml` in M6. The small tools keep their literals (dev tools). |
| `cell_planning_service/cell_planning_service/moveit_backend.py:71-75`, `cell_planning_curobo/cell_planning_curobo/curobo_backend.py:67-68` | MoveIt service names, `/sync_scene_objects`, `/get_planning_scene` | MoveIt's own service names are fixed by move_group, but a namespace is possible on hardware; make them `planner.moveit.services.*` in `cell.yaml` with the current values as defaults in config, not code. M4. |
| `cell_planning_service/.../node.py:78`, `cell_planning_curobo/.../node.py:77`, `moveit_backend.py:88`, `curobo_backend.py:83` | `/joint_states` fallback | Remove the code fallback; `planner.joint_state_topic` resolves to `topics.joint_states` from the platform config. M4. |
| `cell_planning_service/.../service_server.py:39` | `/cell/planning` | `services.planning_namespace`. M4. |
| `cell_planning_curobo/.../curobo_backend.py:129-130, 272-275`, `node.py:75`, `depth_obstacles.py:46` | `base_link`, `tcp`, `world` fallbacks | Remove the fallbacks; values from `frames.*` and the per-platform cuRobo block. M4. |
| `runner/recorder/frame_recorder.py:62` | `/recorder_rgb` | `recording.frames.topic` already exists in `config/cell.yaml` (v1 M3); M6 wires it and removes the literal default. |
| `agents/tasks/fmb_assembly_2.watchdog.yaml` | object frames via registry | Unchanged; object IDs are registry IDs, not frames. |

### 5.2 Python features newer than 3.10

Searched for: the `type` statement, PEP 695 generics, `ExceptionGroup`/`except*`, `typing.Self`, `tomllib`, `StrEnum`, `datetime.UTC`, `asyncio.TaskGroup`/`asyncio.timeout`, `NotRequired`, `hashlib.file_digest`, `itertools.batched`, `typing.override`, `warnings.deprecated`, same-quote nesting inside f-strings, and `datetime.fromisoformat` on strings with `Z`.

**None found** in the copied packages, the three v1-M3 packages, `runner/`, or `agents/agent_harness/`. The only near miss is `robot_backend/watchdog/recorder.py:252`, which already does `value.replace("Z", "+00:00")` before `fromisoformat`, so it parses on 3.10. All files use `from __future__ import annotations`, so `X | None` and `list[str]` annotations are fine on 3.10; no `isinstance(x, A | B)` at run time was found. `match` statements, `dataclass(slots=True, kw_only=True)` and `zip(strict=True)` are 3.10 features and allowed.

Residual risk that only a 3.10 interpreter can settle: `mcp<2` and `pydantic` behaviour under 3.10, and any 3.11-only standard-library default I did not search for. The Humble container build in M3 is the check; `docs/PLATFORMS.md` will list anything it turns up.

### 5.3 APIs that differ between Humble and Jazzy

| Where | Difference | Handling |
|---|---|---|
| `sim_moveit_bringup.launch.py:420` (`ros2 bag record -o …`) and the per-episode bag (M6) | Default storage is sqlite3 on Humble, mcap on Jazzy. | Always pass `-s mcap`; the Dockerfile installs `ros-humble-rosbag2-storage-mcap`. One helper in the compatibility module builds the record command. |
| `ur5e_moveit_config/config/ompl_planning.yaml:1-11` | MoveIt 2.10 (Jazzy) keys `planning_plugins:` (list) and `default_planning_request_adapters/*`; MoveIt 2.5 (Humble) keys `planning_plugin:` (string) and `default_planner_request_adapters/*` as one space-separated string. `launch/planning_context.launch.py:35-36` sets `planning_pipelines`/`default_planning_pipeline`, which both accept. | This package stays sim/Jazzy. A hardware MoveIt config, if wanted, is a separate Humble-format package (question 14); the planning service does not care which one runs. |
| `cell_planning_service/.../moveit_backend.py:197` (`MotionPlanRequest.pipeline_id`) | Field added to `moveit_msgs` in 2.2.x; Humble ships 2.2.1, so it should exist. | Verify at container build (`ros2 interface show moveit_msgs/msg/MotionPlanRequest`); if absent, the compatibility module sets it only when present. All other fields used (`:190-196, 275-283`, `GetCartesianPath` `max_step`, `jump_threshold`, `avoid_collisions`, `link_name`, `waypoints`) exist in Humble. |
| `moveit_msgs/MoveItErrorCodes` | Jazzy added codes (`START_STATE_INVALID = -26` and later); Humble's set stops earlier. `failure_codes.MOVEIT_ERROR_CODE_MAP` was written from the Jazzy `.msg`. | The map stays a dict by integer; unknown integers already fall back to `PLANNING_FAILED` with the integer in the reason. Listed in PLATFORMS.md. |
| `tf2_ros.TransformListener(..., spin_thread=False)` (`scene_tf_sync_node.py:191, 387`), `Buffer.lookup_transform(..., rclpy.time.Time())` (`recorder.py:590-601`) | Same signatures on both. | None. |
| `rclpy` timers, subscriptions, clients, action clients, `MultiThreadedExecutor`, `CancelGoal` service naming (`_action/cancel_goal`) | Same on both. `rclpy.wait_for_message` (Iron+) and `create_timer(autostart=)` (Jazzy) are **not used** anywhere. | Keep it that way; PLATFORMS.md lists them as forbidden. |
| `vision_msgs/Detection3DArray` (new pose topics) | `vision_msgs` 4.1.x on both distros: `detections[].id`, `results[].hypothesis.class_id`, `results[].hypothesis.score`, `results[].pose.pose`. | Use only those fields. |
| `control_msgs/FollowJointTrajectory`, `trajectory_msgs/JointTrajectory`, `sensor_msgs/JointState` | Identical. | None. |
| `ros2_control` on hardware (UR driver, Humble) | Controller names and the scaled trajectory controller differ by driver branch (question 4). | All names in `config/platforms/hardware.yaml`. |
| Python packaging (`ament_python` `setup.py` packages) | Humble's setuptools prints deprecation warnings; builds fine. | None. |
| `ros2_ws/.venv` re-exec (`robot_backend/mcp_servers/__init__.py:47-80`) | The venv must be created with the distro's Python (3.10 in the container, 3.12 on Jazzy). | Dockerfile creates it; PLATFORMS.md says so. |
| `session.py:363-364` (`cell_use_sim_time: True`) | Hardware has no sim time. | `platform.use_sim_time` from the platform config; manifest records the actual value. M3. |

### 5.4 Simulator ground-truth assumptions

| # | File:line | Assumption | Handling |
|---|---|---|---|
| G1 | `watchdog/recorder.py:355-356, 428-480, 585-602`; registry `tf_frame`/`lookup_frame` per object (`model_registry.yaml:232-233, 288-289, 347-348, 417-418, 444`); monitors `watchdog/monitors.py:231, 245` | Every part pose is a TF lookup of the Isaac-published object frame: exact, always visible, one shared source for both roles. | M3 pose interface: WatchDog samples poses from `/camera2/poses` (confidence, source, stamp per pose); the recorder's TF tick becomes a pose-topic tick with the same `wall_ns` discipline. Robot frames (`tcp`, `base`) still come from TF on both platforms. Sim `ground_truth` mode publishes `/camera2/poses` from TF (M6). Occluded part = missing evidence, never a verdict. |
| G2 | `nodes/scene_tf_sync_node.py:316-380`, `skills/pose_query_skills.py:43-230` | Executor pose query is the same TF. | Backed by `/camera1/poses` through the executor pose backend (M3); service types unchanged. |
| G3 | `watchdog/monitors.py:382-467` (`EffortEventMonitor`), `contracts/loader.py:172-182, 264-296`, registry `limits.max_clamp_effort` (`:279, 335, 400, 467, 497, 527, 607, 639, 671, 703`) and `grasp_effort_floor` | Clean `finger_joint` effort on `/joint_states` in Isaac units; thresholds are sim constants. | Signal availability `finger_effort: available | proxy | missing`; on hardware the gripper backend reports driver force/current as `gripper_driver` source, and the per-part limits for hardware are `TODO(mohammad)` in the requirements file. The monitor refuses to arm when the signal is `missing` and says so. |
| G4 | `isaac_trajectory_bridge/rg2_controller.py` contact detection (`:113, 157, 163`), `/gripper_width_true` (`:318`) | Sim gripper gives true width and effort-based contact. | Sim-only; the gripper interface's declared readings list (`width`, `effort`, `grip_detected`) is `available` on sim and whatever the RG6 driver provides on hardware (question 3). |
| G5 | (none in WATCHDOG) | Pad contact identity. | Sim only via `contacts_enable` (M6); `pad_contacts: missing` on hardware. |
| G6 | (none in WATCHDOG; stage reopened by hand) | Teleport reset. | M6 command server on sim; operator prompt plus verify on hardware. |
| G7 | `watchdog/image_buffer.py:56-91` | One overhead depth camera with perfect intrinsics. | Two cameras from `config/cameras/<platform>.yaml`; depth optional per camera. |
| G8 | `session.py:363`, bridge `clock_stall_timeout_sec` (`follow_joint_trajectory_server.py:41`) | Sim clock exists and can stall. | `use_sim_time` per platform; the bridge is sim-only. |
| G9 | `sim_moveit_bringup.launch.py:281-288` | Isaac is already playing. | Sim backend `scene.health` checks it; hardware checks the UR driver instead. |

---

## 6. Proposed INTEGRATION layout, with adjustments

The v2 target layout is adopted with these adjustments (v1 adjustments that still apply are repeated):

| Adjustment | Reason |
|---|---|
| **`agents/` stays at the repo root** (`agent_harness/`, `config/`, `prompts/`, `tasks/`). | `role_context.py:97-110` locates prompts by walking up to a directory holding both `agents/` and `ros2_ws/`; Mohammad accepted this in v1 (answer 12). |
| **`config/robot/`** keeps the four Isaac/Lula robot files and the RG2 sphere template; **`config/gripper/rg2_kinematics.yaml`** (v2) is the WatchDog-only width model, derived in M3 from the validated constants by copying them, with the width table referenced by path, not duplicated. | Keeps v1 placement and satisfies the v2 `gripper/` entry without re-deriving anything. |
| **`config/platforms/sim.yaml`** is the single source for every name in section 5.1, including the `cell:` block that today lives in the registry. **`config/platforms/hardware.yaml`** mirrors it with `TODO(mohammad)` values. | Rule 10. |
| **`config/cell.yaml`** (exists) gains `platform:`, `pose_source:`, and the `watchdog.halt` block changes to `extra_cancel_targets` (primary comes from the platform). | Section 5.1. |
| **`ros2_ws/src/cell_platform_interfaces/`** holds the Python ABCs, the platform-config reader, the Humble/Jazzy compatibility module, and the message/service/action definitions that the backends need beyond what `cell_planner_msgs` already has. | One home for everything both backends share. `cell_planner_msgs` keeps the planner, fact, ruling and notice types from v1 M3 so nothing built so far moves. |
| **`ros2_ws/src/cell_backend_sim/`** and **`cell_backend_hw/`** as in the prompt. `cell_backend_sim` depends on `isaac_trajectory_bridge_interfaces`; `cell_backend_hw` on `control_msgs` and the RG6 driver interfaces Mohammad names. | Prompt layout. |
| **`docker/humble/`** builds only the shared packages plus `cell_backend_hw`; the sim packages (`isaac_trajectory_bridge*`, `sim_moveit_bringup`, `ur5e_moveit_config`, `cell_backend_sim`) carry a `COLCON_IGNORE`-style skip list in the Dockerfile rather than files in the tree. | The tree must still build whole on Jazzy. |
| **`isaac/cameras/`** holds the camera prims setup script run inside Kit (5.1 API) and the `ground_truth` pose publisher node lives in `cell_backend_sim` (ROS side), not under `isaac/`. | The publisher is a plain rclpy node reading TF; only prim creation needs Kit. |
| **`records/`, `executor/workspace/`** gitignored (done in M2/M3). | As specified. |

Resulting tree (new items marked with their milestone):

```
I/
  docs/            INTEGRATION_SURVEY, BUILD_PLAN, PROVENANCE, NEXT_STEPS, (M3) PLATFORMS, (M5) INDEPENDENCE, (M7) PLANNER_CONTRACT, (M8) OPERATIONS, ARCHITECTURE
  agents/          agent_harness/, config/, prompts/, tasks/ (WatchDog task only)
  config/
    cell.yaml                         (M3: + platform, pose_source)
    platforms/{sim,hardware}.yaml     (M3)
    cameras/{sim,hardware}.yaml       (M3 names; M6 sim poses)
    gripper/{rg2,rg6}_kinematics.yaml (M3)
    robot/                            ur5e.urdf, ur5e.xrdf, ur5e.yaml, ur5e_rmpflow_common_v2.yaml, rg2_collision_spheres.yaml, (M4) rg6_collision_spheres.yaml
    requirements/, tasks/, parts/     (M5, M7)
    layouts/<assembly>/               (M6)
    mcp/, settings/                   watchdog.* (M2), (M3) watchdog.hardware.*, (M7) executor.{sim,hardware}.*
  docker/humble/   Dockerfile, compose.yaml (M3)
  ros2_ws/src/     10 copied packages; cell_planner_msgs, cell_planning_service, cell_planning_curobo (M4, exist);
                   (M3) cell_platform_interfaces, cell_backend_sim, cell_backend_hw; (M5) cell_physical_events
  isaac/           (M6) cell_command_server/, headless/, cameras/
  executor/        (M7) authoring_mcp/, planner_template/, workspace/ (ignored)
  runner/          run_watchdog.sh, recorder/frame_recorder.py, supervisor/preflight_check.sh; (M7) episode_runner/; (M8) supervisor/, index/
  records/         (ignored)
```

---

## 7. Questions for Mohammad (v2)

The v1 questions and answers are kept at the end of this section for the record. New questions:

1. **Wrist camera calibration and chessboard code.** Where do the camera1 hand-eye calibration result and the chessboard perception code live (repo, path, format of the calibration output), and which frame is the result expressed in (`tool0`, `tcp`, flange)? Nothing in WATCHDOG references either.
2. **Camera hardware models** for camera1 (wrist) and camera2 (table), with resolution, depth availability, and the ROS driver and topic names each publishes today.
3. **RG6 driver.** Which RG6 ROS 2 driver will run on the accelerator (OnRobot's own, a community package, or UR URCap via the UR driver's tool I/O), and which readings it exposes: width, force or motor current, grip detected, busy. The gripper interface declares exactly that list; missing readings are declared missing.
4. **UR driver branch** installed on the accelerator (`Universal_Robots_ROS2_Driver` Humble branch and version), the robot model (UR5e or other), and the trajectory controller in use (`scaled_joint_trajectory_controller` or `joint_trajectory_controller`), so `platforms/hardware.yaml` names the right action and cancel service.
5. **Sim cameras.** The stage has one overhead depth camera (`camera_depth_optical_frame`, section 3) and the recorder camera. Should that depth camera become camera2 (table camera), and is the wrist camera to be added to the USD by hand in the GUI or by the M6 prim script? Its pose must come from the hardware calibration (question 1).
6. **FoundationPose output.** Does your camera2 FoundationPose pipeline already publish `vision_msgs/Detection3DArray` with registry IDs, or should M3 include a small adapter node from its native output? Same for the Orin-side camera1 pipeline.
7. **Where each process runs on hardware.** Confirm: UR driver and camera1 FoundationPose on the Orin; WatchDog server, planner, planning service, physical events node, recorder, camera2 FoundationPose, and the episode runner in the Humble container on the workstation. This fixes the compose mounts and which packages the container builds.
8. **MoveIt on hardware.** Will the planning service's MoveIt backend run on hardware (needs a Humble-format MoveIt config for the UR with RG6, likely from the driver's `ur_moveit_config`), or is cuRobo the only hardware backend? Section 5.3 explains why `ur5e_moveit_config` cannot be reused there.
9. **Hardware effort limits.** The registry's `max_clamp_effort` and `grasp_effort_floor` are Isaac units. Should the requirements file carry a per-platform limits block (`limits.sim`, `limits.hardware`) with the hardware values left `TODO(mohammad)`, or a single block that the platform scales? I plan the per-platform block.
10. **Execution and git going forward.** The v2 prompt restates "do not execute anything" and "never commit or push". On 2026-11-03 you allowed builds, tests, commits and pushes inside INTEGRATION. Which rule applies from now on? Until you answer I run nothing and commit nothing.
11. **Pruning the executor-side modules.** v1 planned to delete `ros_moveit_backend/`, `skills/`, and the motion/gripper/scene MCP servers in M7. With backends arriving in M3 they become dead weight earlier. Keep M7 (recommended, keeps the M2 check reproducible) or prune in M3?
12. **ROS_DOMAIN_ID and RMW** on the accelerator, for the compose file (`TODO(mohammad)` until then).

**Answered 2026-11-10.** 1: calibration data is on the accelerator; `TODO(mohammad)` until fetched. 2: both cameras are Orbbec Gemini 335Lg (model G40055-270). 3: RG6 runs through the OnRobot URCap; today it is driven with URScript through the UR API; no ROS 2 gripper driver yet, so readings over ROS are `TODO(mohammad)` and declared missing until a path exists. 4: robot is a **UR7e**; driver branch and controller name `TODO(mohammad)`. 5: the existing Isaac camera is the scene observation camera and becomes **camera2**; there is no wrist camera; Mohammad will ask for one to be added through the Kit MCP bridge (M6). 6: no FoundationPose runs anywhere yet, sim or hardware; the pose topics are the contract and adapters come later. 7: workstation Humble container runs WatchDog and the camera2 pipeline; everything else (UR driver, camera1, planner, planning service, physical events) runs on the accelerator; the recorder and episode runner placement is decided in M7. 8: hardware motion and planning use the best option on the UR AI Accelerator SDK (`https://docs.universal-robots.com/AI_Accelerator_Documentation/`), the same on sim, MoveIt as backup only; so `planner.backend: curobo` is the intended hardware default and `moveit` the fallback, pending what the SDK ships (`TODO(mohammad)`). 9: per-platform limits blocks (`limits.sim`, `limits.hardware`). 10: builds, tests and installs inside INTEGRATION are allowed; **commits and pushes are Mohammad's**. 11: prune the executor-side modules later (M7). 12: domain id and RMW later (`TODO(mohammad)`).

**v1 questions, answered 2026-11-03** (kept for the record): 1 latest commit `595ede33…`; 2 path rewrites only; 3 copy whole, prune later; 4 cuRobo `v0.7.8`, xrdf is the same UR5e; 5 `planner.backend: moveit`; 6 halt = bridge cancel, no hold; 7 gripper left alone on halt; 8 `/recorder_rgb` is in the stage; 9 Isaac 5.1 now, maybe 6.1 later; 10 the session location changes automatically per episode (runner sets `WATCHDOG_SESSION_DIR` and writes the pointer file); 11 scorers not copied; 12 `agents/` stays at the root.
