# Next steps

Updated 2026-11-10 at the end of Milestone 3 (v2): platform backends, names in config, Humble container.

## State

- M1 (v2 plan), M2 (copy): done. Your 2026-11-10 answers are recorded in `docs/BUILD_PLAN.md` section 7.
- M3 (this milestone): built and offline-tested here. Nothing run against Isaac yet. **Not committed**: commits and pushes are yours (answer 10).
- M4 (planning service, old M3): built earlier; its names now come from the platform config too. Your live check (below) covers both.

## What Milestone 3 added

| Path | What |
|---|---|
| `config/platforms/sim.yaml`, `hardware.yaml` | Every topic, frame, joint, action, service name; gripper readings; signal availability. Hardware values are `TODO(mohammad)` where unknown. |
| `config/cameras/sim.yaml`, `hardware.yaml` | camera1 (wrist, executor's pose query) and camera2 (table, WatchDog's channel) roles, topics, extrinsics. Sim camera2 = the existing stage camera; camera1 disabled until you add it (M6). Hardware: Orbbec Gemini 335Lg both, poses `TODO`. |
| `config/gripper/rg2_kinematics.yaml`, `rg6_kinematics.yaml` | WatchDog-only gripper models. RG2 constants copied from the controller; RG6 all `TODO`. |
| `ros2_ws/src/cell_platform_interfaces/` | Abstract motion/gripper/pose/scene backends, the strict platform-config reader, Humble/Jazzy `compat.py`, signal vocabulary, shared `FollowJointTrajectory` motion client and `Detection3DArray` pose source. `ros2 run cell_platform_interfaces show_platform sim`. |
| `ros2_ws/src/cell_backend_sim/` | Bridge motion, RG2 gripper, two pose sources, command-server scene client (M6), and `ground_truth_pose_publisher` (Isaac TF to `/camera1/poses` and `/camera2/poses`). |
| `ros2_ws/src/cell_backend_hw/` | Scaffold: UR driver motion, RG6 over URScript (refuses until the URCap function names are filled), pose topics, operator reset prompt. |
| WatchDog server | New read-only tool `available_evidence_sources`; halt primary target from the platform; recorder, event bus, gate, camera topics, observation frame, effort joint all from config. Registry `cell:` block superseded by the platform frames. |
| Launch files | Bridge and bringup read the platform config; bringup records with `-s mcap` and starts the pose publisher. |
| `docker/humble/` | Dockerfile (ros:humble, Python 3.10, builds the shared packages + `cell_backend_hw`, skips the sim packages), compose with host networking, `ROS_DOMAIN_ID`/RMW `TODO`. |
| `docs/PLATFORMS.md` | What runs where, the single-distro rule, the Humble/Jazzy differences, where names live. |

Checked here: `colcon build` (16 packages); 43 tests (platform config, signals, compat, sim backend, hardware scaffold, planning, halt regression); WatchDog server imports from the venv with the new helpers; `show_platform` for both platforms; both launch files load and resolve their names; the orchestrator, the pose publisher (21 registry objects) and the planning node start and report the configured names.

## What you run to check Milestone 3

1. Rebuild and source:

   ```bash
   cd ~/full-Actor_Watchdog-agentic-system/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash
   ros2 run cell_platform_interfaces show_platform sim      # the names every process will use
   ```

2. Rerun the Milestone 2 check: Isaac playing, bringup as before, one S3 WatchDog session. Expect the same behaviour as before plus: a `ground_truth_pose_publisher` line listing 21 objects, `ros2 topic hz /camera2/poses` at 20 Hz, and the `available_evidence_sources` tool listed by the session (call it once by hand if you like; it only reports).

3. Milestone 4 live check (still owed): plan to a joint goal through `/cell/planning/plan_to_joints` with no arm motion, then cancel a running bridge goal through `/arm_controller/follow_joint_trajectory/_action/cancel_goal` (full commands in the previous NEXT_STEPS, unchanged; the names now come from `config/platforms/sim.yaml`).

4. Humble container (workstation):

   ```bash
   docker compose -f docker/humble/compose.yaml build
   docker compose -f docker/humble/compose.yaml run --rm cell ros2 run cell_platform_interfaces show_platform hardware
   ```

   Expect the hardware signal list (everything gripper-related `missing`). Report any build error: the 3.10 and Humble differences this build turns up go into `docs/PLATFORMS.md`.

## `TODO(mohammad)` introduced in M3

- `config/platforms/hardware.yaml`: `frames.tcp`, `motion.action`/`cancel_service` (confirm), `motion.goal_*`, driver status topics, `topics.urscript_command` (confirm), `gripper.urscript.*`, `gripper.width_range_m`, `gripper.force_range_n`.
- `config/platforms/sim.yaml`: `scene.port` (M6).
- `config/cameras/sim.yaml`: camera1 pose and resolution (after you add the wrist camera), camera2 resolution. `config/cameras/hardware.yaml`: both poses, topic names to confirm.
- `config/gripper/rg6_kinematics.yaml`: everything.
- `docker/humble/compose.yaml`: `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`; Dockerfile: UR driver message packages.
- Still open from M4: `planner.moveit.eef_step_m`, `jump_threshold`, `check_resolution_rad`; all `planner.curobo.*` request defaults and `python`; `config/robot/rg2_collision_spheres.yaml`; `budgets.*`, `health.*`.

## What Milestone 4 finishing touches and Milestone 5 will do once approved

- M4 close-out: UR7e + RG6 cuRobo robot config placeholders (`config/robot/rg6_collision_spheres.yaml`, a UR7e XRDF `TODO`), `compat.set_if_present` for `pipeline_id` if the container build asks for it. Small.
- M5: `cell_physical_events` node on `/cell/physical_events` with `source` and `confidence` per event and thresholds as `TODO(mohammad)`; plan-blind WatchDog (strip `steps:`, step-scoped `signal_ready`, executor-event subscription; held object from `part_lifted/released`; termination from the runner; `arm_wake`, `issue_ruling`, cue tool for camera1 poses, two-camera `look()`, evidence tagging with `shared_source`, acceptance inspection, `record_only | halt_enabled`, `consult`); plan-blind prompt; `docs/INDEPENDENCE.md`.
