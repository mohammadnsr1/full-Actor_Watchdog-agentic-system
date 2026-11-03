# Next steps

Updated 2026-11-03 at the end of Milestone 2 (WatchDog brought in).

## State

- Milestone 1: `docs/BUILD_PLAN.md`, all twelve questions answered (recorded in its section 6).
- Milestone 2: approved files copied from WATCHDOG at commit `595ede33…`; `docs/PROVENANCE.md` lists every path. Four files edited, paths only (listed at the top of PROVENANCE.md). Nothing built or run. WATCHDOG and DROIDSTACK untouched.

## What Mohammad builds and runs to check Milestone 2

The goal is one WatchDog session that behaves exactly like `./run_watchdog.sh s3 2` in WATCHDOG, but launched from this repo.

1. Create the workspace venv and build, from this repo:

   ```bash
   cd ~/full-Actor_Watchdog-agentic-system/ros2_ws
   python3 -m venv .venv
   .venv/bin/pip install -r requirements.txt
   source /opt/ros/jazzy/setup.bash
   colcon build
   source install/setup.bash
   ```

   `colcon_defaults.yaml` pins the system Python as before.

2. Optional but recommended: run the backend unit tests the way WATCHDOG's README does, from `ros2_ws`:

   ```bash
   PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="$PWD/src/robot_backend:$PYTHONPATH" .venv/bin/python3 -m pytest -q src/robot_backend/test
   ```

3. Bring the cell up exactly as today, but source this repo's install:
   - Isaac Sim: open `assets/usd/assembly_room.usd` (this repo's copy, or WATCHDOG's; they are identical at `595ede33…`), reset simulation time, press Play.
   - In a fresh shell with nothing exported: `source /opt/ros/jazzy/setup.bash && source ~/full-Actor_Watchdog-agentic-system/ros2_ws/install/setup.bash && ros2 launch sim_moveit_bringup sim_moveit_bringup.launch.py`.
   - Check that `~/.watchdog/current_session` now names a directory under `~/full-Actor_Watchdog-agentic-system/agents/transcripts/sessions/` (the session root is derived from the repo root; this confirms the copied tree is the one running).

4. Launch the WatchDog session from this repo:

   ```bash
   cd ~/full-Actor_Watchdog-agentic-system
   ./runner/run_watchdog.sh s3 2
   ```

   Expected: the launcher echoes `model=claude-sonnet-5 effort=low`, the session's first turn calls `get_rendered_task` twice, and `sources.task_path` in the result points into this repo. The session should reach `signal_ready(scope="run")` as before. No executor is needed for this check; stop the session after readiness, then Ctrl-C the bringup.

5. If anything differs from a WATCHDOG session, the first suspects are the three rewritten paths in `config/mcp/watchdog.mcp.json` and the `agents/` lookup in `ros2_ws/src/robot_backend/robot_backend/watchdog/session.py:65-75`.

## What Milestone 3 will do once approved

- `ros2_ws/src/cell_planner_msgs/`: planning service and action types (plus the physical-event, notice and ruling messages used later).
- `ros2_ws/src/cell_planning_service/`: the backend-neutral planning node with `plan_to_pose`, `plan_to_joints`, `plan_cartesian`, `check_trajectory`, `add_collision_object`, `remove_collision_object`, `sync_collision_objects`; MoveIt backend, plan only.
- `ros2_ws/src/cell_planning_curobo/`: cuRobo `v0.7.8` backend node plus `INSTALL.md`; depth obstacles off by default.
- `config/cell.yaml` with `planner.backend: moveit`.
- Halt client retargeted to `/arm_controller/follow_joint_trajectory/_action/cancel_goal`, with `/move_action` kept as a configurable extra target.
