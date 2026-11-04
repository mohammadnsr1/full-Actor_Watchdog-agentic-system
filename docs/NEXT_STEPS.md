# Next steps

Updated 2026-11-03 at the end of Milestone 3 (planning service with switchable backends).

## State

- M1 plan, M2 copy: done and checked by Mohammad (an S3 WatchDog session ran from this repo).
- M3: built and offline-tested here. Nothing run against Isaac or MoveIt yet.

## What Milestone 3 added

| Path | What |
|---|---|
| `ros2_ws/src/cell_planner_msgs/` | Services `PlanToPose`, `PlanToJoints`, `PlanCartesian`, `CheckTrajectory`, `AddCollisionObject`, `RemoveCollisionObject`, `SyncCollisionObjects`; messages `PlanningReport` (backend, planning time, failure code, `wall_time_iso`), `ExecutorFact`, `Ruling`, `WatchdogNotice` (the last three are for M4 and M6). |
| `ros2_ws/src/cell_planning_service/` | Backend contract, generic service adapter, failure-code vocabulary, time parameterization, `cell.yaml` reader, **MoveIt plan-only backend** (`/plan_kinematic_path`, `/compute_cartesian_path`, `/check_state_validity`, `/apply_planning_scene`; sync delegated to `/sync_scene_objects`), node, launch file that picks the backend from `config/cell.yaml`. |
| `ros2_ws/src/cell_planning_curobo/` | cuRobo v0.7.8 backend (own venv, re-exec), robot config from `config/robot/ur5e.xrdf` plus RG2 spheres, explicit world, IK-chain Cartesian planner, experimental depth-obstacle layer behind `planner.depth_obstacles`, `INSTALL.md`. |
| `config/cell.yaml` | New. `planner.backend: moveit`, request defaults per backend, `watchdog.halt.cancel_targets`, and the shape of later sections. |
| `config/robot/rg2_collision_spheres.yaml` | New, all `TODO(mohammad)`. |
| `robot_backend/watchdog/halt.py` | Rewritten: cancel-all to a configurable list of targets. Default and first target `/arm_controller/follow_joint_trajectory/_action/cancel_goal`; `/move_action/_action/cancel_goal` kept as an extra in `cell.yaml`. |
| `robot_backend/cell_config.py` | New, small reader for `config/cell.yaml`. |
| `robot_backend/mcp_servers/watchdog_mcp_server.py` | One edit: halt targets and timeout come from `cell.yaml`. |
| `robot_backend/package.xml` | Added the `action_msgs`, `cell_sync_interfaces`, `python3-yaml` dependencies it already used. |
| `sim_moveit_bringup` launch | New args `start_planning_service` (default true) and `cell_config_path`; includes the planning-service launch after the orchestrator. |

Checked here: `colcon build` (13 packages), 21 planning tests, the halt callback-group regression test (now against the bridge cancel service), the WatchDog server imports from the venv with the new halt config, the node starts from the launch file and registers the seven services, and a call with MoveIt down returns `BACKEND_UNAVAILABLE` with a stamped report.

## `TODO(mohammad)` values introduced in M3 (all in `config/cell.yaml` unless noted)

- `planner.moveit.eef_step_m`, `jump_threshold`: Cartesian step; required before `plan_cartesian` works (or callers pass `eef_step_m` in the request).
- `planner.moveit.check_resolution_rad`: required before `check_trajectory` works (or callers pass it).
- `planner.curobo.python`, `allowed_planning_time_s`, `num_planning_attempts`, `eef_step_m`, `check_resolution_rad`, `motion_gen_overrides`, and the whole `depth` block.
- `config/robot/rg2_collision_spheres.yaml`: every RG2 link. The cuRobo node refuses to start until filled.
- `budgets.*`, `health.*`: for M6 and M7.

## What Mohammad runs to check Milestone 3

1. Rebuild and source (already built here; rebuild after any edit):

   ```bash
   cd ~/full-Actor_Watchdog-agentic-system/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash
   ```

2. Isaac playing, then bring the cell up as before. The bringup now also starts the planning service; expect the line `planning service backend=moveit (from .../config/cell.yaml)` and the node's `planning service up` line.

3. Plan from the current pose to a joint goal (replace the six values with a nearby pose you trust; the request's start state is empty, so it plans from `/joint_states`):

   ```bash
   ros2 service call /cell/planning/plan_to_joints cell_planner_msgs/srv/PlanToJoints \
     "{planning_group: ur_manipulator, joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], joint_positions: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]}"
   ```

   Expect `ok: true`, a non-empty `trajectory` with increasing `time_from_start`, and `report.backend: moveit`, `report.failure_code: ''`. The arm must **not** move: this is the proof that the service is plan-only.

4. Prove the halt retarget with the bridge alone. Send the returned trajectory to the bridge (`ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: ...}"`, or any client you prefer), and while it runs call the cancel service:

   ```bash
   ros2 service call /arm_controller/follow_joint_trajectory/_action/cancel_goal action_msgs/srv/CancelGoal "{goal_info: {goal_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}}}"
   ```

   Expect `return_code: 0`, one entry in `goals_canceling`, the action ending with status CANCELED, and the arm stopping at its current setpoint (no hold is published; this was your choice in Q6).

5. Optional: `plan_cartesian` and `check_trajectory` need `eef_step_m` and `check_resolution_rad` either in `cell.yaml` or in the request; until then they return `INVALID_REQUEST` naming the missing default.

6. cuRobo: follow `ros2_ws/src/cell_planning_curobo/INSTALL.md` when you want it. Not needed for Milestone 4.

## What Milestone 4 will do once approved

- `ros2_ws/src/cell_physical_events/`: the event node (`contact_made/lost`, `part_lifted/released/settled`, `part_approaching_feature`, `part_engaged_feature`, `effort_rising`, `motion_started/stopped`, `workspace_object_appeared`), thresholds as `TODO(mohammad)` config, effort proxy for contact until M5.
- WatchDog server: strip `steps:` and step-scoped `signal_ready`, drop the executor-event subscription, held-object tracking from physical events, run termination from the episode runner, `arm_wake`, `issue_ruling`, the acceptance inspection tool, `record_only | halt_enabled` and `consult` modes from `cell.yaml`.
- Plan-blind prompt from the S3 prompt; `docs/INDEPENDENCE.md`.
