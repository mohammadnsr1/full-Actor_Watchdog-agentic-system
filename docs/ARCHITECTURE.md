# Architecture

Written 2026-11-23 (Milestone 7). How the code and files connect, from the
simulator and ROS 2 up to the two agents. Diagrams are plain text so they
show everywhere (editor, preview, terminal).

Two agents never talk to each other:

- the **executor** side: an authoring agent writes a *planner* (a program), and
  the episode runner runs that planner on the cell;
- the **WatchDog**: an agent that watches each episode through its own MCP
  server and judges it against the requirements.

Everything between them and the robot is ROS 2, and every ROS name comes from
`config/platforms/<platform>.yaml` (sim or hardware).

## 1. Layers

```text
 ┌─ AGENTS ───────────────────────────────────────────────┬──────────────────────────────────────────────────────┐
 │ EXECUTOR AUTHORING agent (Claude Code)                 │ WATCHDOG agent (claude -p)                           │
 │ runner/run_authoring.sh [sim|hardware]                 │ one session per episode, plan-blind                  │
 │                         [plan_given|plan_free]         │ agents/prompts/watchdog_*_plan_blind.txt             │
 │ writes a planner; never moves the arm itself           │ judges; may halt the arm                             │
 └────────────────────────────────────────────────────────┴──────────────────────────────────────────────────────┘
                             ▲ MCP (stdio)                                           ▲ MCP (stdio)
                             ▼                                                       ▼
 ┌─ MCP SERVERS ──────────────────────────────────────────┬──────────────────────────────────────────────────────┐
 │ executor/authoring_mcp/server.py                       │ robot_backend/mcp_servers/                           │
 │ get_task  evaluate_planner  read_verdict               │   watchdog_mcp_server.py (a ROS node too)            │
 │ submit_work                                            │ monitors, evidence, look, halt, rulings,             │
 │ feedback filtered by cell.yaml executor_feedback       │ record_run_verdict                                   │
 └────────────────────────────────────────────────────────┴──────────────────────────────────────────────────────┘
                             │ evaluate_planner -> one episode each                  ▲ the runner starts this session
                             ▼                                                       │
 ┌─ HARNESS ─────────────────────────────────────────────────────────────────────────────────────────────────────┐
 │ EPISODE RUNNER  runner/episode_runner/run_episode.py                                                          │
 │ health, reset + verify, recording, START PERMISSION (waits for the WatchDog's ready),                         │
 │ sim budget + wall guard (from the permission on), ground-truth checker,                                       │
 │ starts the WatchDog first, the planner only after the permission; writes records/<id>/                        │
 └───────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
                             │ MCP client: execute_task(task_ref), once              ▲ episode_end in, run_verdict out
                             ▼                                                       ▼
 ┌─ PLANNER | WATCHDOG ROS SIDE ──────────────────────────┬──────────────────────────────────────────────────────┐
 │ executor/planner_templates/{plan_given|plan_free}      │ wakes on : its monitors + wake conditions over       │
 │   or a session snapshot                                │   /camera2/poses, /cell/physical_events,             │
 │ planner.py -> worker.py (one ROS node) + geometry.py   │   /joint_states, /tf, images; episode_end notice     │
 │   plan   : /cell/planning/*  (plan only)               │ halt     : damage risk / unrecoverable only          │
 │   move   : backends.motion / backends.gripper          │ rulings, consultations: /cell/watchdog_notices       │
 │   poses  : backends.executor_poses (camera1)           │ ready    : /cell/signal_ready (start permission)     │
 │   consult: w.consultations() (WatchDog text)           │ verdict  : /watchdog/events (run_verdict)            │
 │   facts  : /cell/executor_facts (recorded only)        │                                                      │
 └────────────────────────────────────────────────────────┴──────────────────────────────────────────────────────┘
                             ▲ cell_platform_interfaces + cell_backend_sim|hw        ▲ plain ROS 2 topics and services
                             ▼                                                       ▼
 ┌─ CELL NODES (ROS 2, sim_moveit_bringup) ──────────────────────────────────────────────────────────────────────┐
 │ isaac_trajectory_bridge   FollowJointTrajectory server + rg2_controller (open, close, guarded_move)           │
 │ cell_planning_service     /cell/planning/*: MoveIt (move_group) now, cuRobo (cell_planning_curobo) later      │
 │ scene_tf_sync_node        /sync_scene_objects: registry geometry into the planning scene                      │
 │ cell_orchestrator         start permission: the runner's request_go waits for signal_ready                    │
 │ cell_physical_events      lifted / released / contact / stationary, from the cell's own signals               │
 │ ground_truth_pose_pub.    /camera1/poses (executor) and /camera2/poses (WatchDog), from Isaac TF              │
 │ pad_contacts_relay        /cell/pad_contacts, polled from the command server                                  │
 │ all names come from config/platforms/<platform>.yaml                                                          │
 └───────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
                             ▲ ROS 2 bridge + TCP 9010
                             ▼
 ┌─ ISAAC SIM 5.1  (assets/usd/assembly_room.usd) ───────────────────────────────────────────────────────────────┐
 │ ROS 2 bridge graphs: /clock /joint_states /joint_command /tf, camera1 + camera2 images                        │
 │ cell command server (isaac/cell_command_server, TCP 9010): reset, poses, contacts, timeline                   │
 │ on hardware this band is the UR driver, the RG6 and the Orbbec cameras (cell_backend_hw)                      │
 └───────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

Arrows show who calls or starts whom (▲▼ both ways where data flows back). Read it bottom up:

1. **Isaac Sim** runs the stage. Its OmniGraph ROS 2 bridge publishes
   `/clock`, `/joint_states`, `/tf` and camera images, and takes
   `/joint_command`. The **cell command server** is a loopback TCP server
   inside Kit: layout reset, pose reports, contact reports, timeline.
2. **Cell nodes** turn that into the cell's services:
   - `isaac_trajectory_bridge`: the arm's FollowJointTrajectory action, and the
     RG2 controller (open/close/guarded move);
   - `cell_planning_service`: planning only, it never moves the arm. MoveIt
     today; cuRobo via `cell_planning_curobo` once installed;
   - `cell_orchestrator`: the start permission (the runner asks, the
     WatchDog's signal_ready grants it; reset per episode);
   - `cell_physical_events`: lifted, released, contact and stationarity events
     from the cell's own signals;
   - `ground_truth_pose_publisher`: on sim, camera1 and camera2 poses from
     Isaac TF;
   - `scene_tf_sync_node`: pushes registry geometry into the MoveIt scene.
3. **Platform layer**: `cell_platform_interfaces` defines the backend
   interfaces and reads the platform config. `cell_backend_sim` and
   `cell_backend_hw` implement them. `build_backends(node, platform)` returns
   motion, gripper, executor_poses, watchdog_poses and scene. The planner and
   the runner use the same backends, so they run unchanged on sim and
   hardware.
4. **Harness**: the episode runner owns one episode. It starts the planner as
   an MCP server and calls `execute_task` once.
5. **Agents** only ever touch MCP tools. The authoring agent edits planner
   files and calls `evaluate_planner`. The WatchDog agent calls its monitoring
   and judging tools.

## 2. One episode

```text
 from               to                     what
 ──────────────────────────────────────────────────────────────────────────────────────────────────────

 ── BEFORE ANYTHING MOVES ──
 authoring MCP    → episode runner         run.sh --planner snapshots/NNN --task <task file> --seed N
 episode runner   → itself                 lock records/.cell.lock, wait for /clock, health check
 episode runner   → Isaac command srv      load_layout(assembly, layout, seed), verify_reset
 episode runner   → Isaac command srv      report_poses  -> 'before' snapshot
 episode runner   → recorder               start bag + frames
 episode runner   → orchestrator           declare episode_id + guard_mode (resets the previous episode)
 episode runner   → WatchDog               start claude -p (plan_blind), session dir records/<id>/watchdog
 WatchDog         → itself                 prepare: requirements, contracts, monitors + wake conditions
 WatchDog         → orchestrator           signal_ready
 episode runner   → orchestrator           request_go(phase=run) -> START PERMISSION (immediate if no WatchDog)
 episode runner   → planner                start run.sh, call execute_task(task_ref) once; budgets start

 ── DURING THE PLANNER CALL (no gating; physics wakes the WatchDog) ──
 planner          → planning service       plan_to_pose / plan_cartesian / plan_to_joints
 planner          → bridge                 execute trajectory (FollowJointTrajectory), gripper open/grasp
 planner          → /cell/executor_facts   intent, hold, carry, release, claim_done (recorded only)
 cell physics     → WatchDog               monitor fires / wake condition matches -> wait_for_event returns
 WatchDog         → bridge                 halt = cancel the arm's goal (or a halt_on_fire monitor does it)
 WatchDog         → planner                ruling after a halt; free-text consultation -- only if consult
 planner          → episode runner         {state: done | stopped, reason}

 ── AFTER THE PLANNER ──
 episode runner   → WatchDog               episode_end notice
 WatchDog         → episode runner         run_verdict on /watchdog/events
 episode runner   → Isaac command srv      report_poses  -> 'after' snapshot, ground-truth checker
 episode runner   → records/<id>/          manifest.json, verdict.json, runner.jsonl
 episode runner   → authoring MCP          verdict (filtered by executor_feedback) + WatchDog halts,
                                         rulings, consultations -> the agent
```

The WatchDog prepares while nothing moves; the start permission is the only
synchronisation, and the budgets start after it. The sim budget and the wall
guard are enforced by the runner: when either is
exceeded it cancels the arm's goal and stops the call.

## 3. The authoring loop and its two conditions

```text
  plan_given ─ config/tasks/fmb_assembly_2.yaml            plan_free ─ config/tasks/fmb_assembly_2.plan_free.yaml
               instruction + step plan + steps                          goal, constraints, objects
                         │                                              (tools/derive_plan_free_task.py)
                         └──────────────────┬───────────────────────────────────┘
                                            ▼ get_task
                                   ┌─────────────────┐
  planner_templates/<condition> ──►│ AUTHORING AGENT │◄──────────────────────────────┐
  copied into                      └───┬─────────┬───┘                               │
  executor/workspace/<condition>/      │ edits   │ evaluate_planner(episodes, seed)  │ feedback:
    <session>/planner/  ◄──────────────┘         ▼                                   │ verdict filtered by
                                      snapshots/NNN (frozen copy)                    │ executor_feedback
                                                 │ runs episodes (section 2)         │ watchdog | checker | both
                                                 └───────────────────────────────────┘
                                       submit_work ─► snapshots/final
```

| | `plan_given` | `plan_free` |
|---|---|---|
| Task file | full | goal, constraints, objects; plan references redacted |
| Template | runs the steps | safe pose, halt handling, primitives; stops |
| Settings | `config/settings/executor.<platform>.settings.json` | `...plan_free.settings.json` (also denies every place the plan is written) |

Both templates share `executor/planner_templates/common/`: `run.sh`, the MCP
`server.py`, `worker.py` (the ROS side) and `geometry.py`.

## 4. What each agent can reach

| | Authoring agent | WatchDog agent |
|---|---|---|
| Tools | `executor-authoring` MCP only (`--strict-mcp-config`) | `robot-backend-watchdog` MCP only |
| Files | Read: executor-side config and sources. Edit: `executor/workspace/` only. No shell. | No Read, Glob, Grep or Bash at all |
| Task | `config/tasks/...` via `get_task` | `config/requirements/<assembly>.yaml` (no steps; refused if it has any) |
| Part data | `config/parts/executor_parts.yaml` (geometry, grasp force) | `config/parts/watchdog_parts.yaml` (tolerances, limits) |
| Poses | camera1 (`/camera1/poses`) through the planner | camera2 (`/camera2/poses`); camera1 only as a flagged cue |
| Outcome of an episode | `read_verdict`, filtered by `executor_feedback` | its own run verdict |
| Never | `records/`, `agents/`, WatchDog code and config | the executor's task, planner, facts as evidence, `records/` |

See `docs/INDEPENDENCE.md` for each channel and where it is cut.

## 5. Where things live

| Path | What | Used by |
|---|---|---|
| `config/platforms/{sim,hardware}.yaml` | every topic, service, action, frame and joint name | all ROS code, via `cell_platform_interfaces.platform_config` |
| `config/cell.yaml` | planner backend, budgets, WatchDog mode/consult, `executor_feedback`, recording | runner, planning service, WatchDog server, authoring MCP |
| `config/tasks/` | executor task files (full and plan-free) | planner, authoring MCP |
| `config/requirements/` | what the WatchDog judges against | WatchDog server, runner's checker |
| `config/parts/` | executor and WatchDog part data, generated from `robot_backend/config/model_registry.yaml` by `tools/split_registry.py` | planner; WatchDog; checker |
| `config/placements/<assembly>.yaml` | each placement frame relative to the fixture (measured once); WatchDog side only | WatchDog pose source, `cell_physical_events`, runner checker |
| `config/assembly_views/<assembly>/` | pictures of the finished assembly | both agents (`get_assembly_views`, `assembly_reference`) |
| `config/layouts/<assembly>/` | reset poses per layout | Isaac command server |
| `config/mcp/`, `config/settings/` | per-role MCP configs and permissions | `run_authoring.sh`, `run_episode.py`, `run_watchdog.sh` |
| `agents/` | WatchDog prompt templates and agent config; `agent_harness/` renders them | `role_context.py` in the WatchDog server |
| `executor/` | `authoring_mcp/`, `planner_templates/`, `workspace/` (ignored) | authoring sessions |
| `runner/` | episode runner, recorder, launch scripts, `jsonl.py` | everything that writes `records/` |
| `isaac/` | Kit-side code: command server, headless launch, camera setup | Isaac Sim |
| `ros2_ws/src/` | the ROS packages in section 1, plus descriptions (`robot_description`, `onrobot_rg_description`, `ur5e_moveit_config`) and `sim_moveit_bringup` (the sim launch) | the cell |
| `tools/` | `pick_check.py`, `split_registry.py`, `derive_plan_free_task.py` | by hand |
| `records/` (ignored) | episode records | runner, authoring MCP |

## 6. Records

Everything is JSON or JSON Lines (one object per line).

```
records/<episode_id>/
  manifest.json            inputs hashed, planner tree hash, git state, versions
  verdict.json             planner result, budget use, checker, WatchDog verdict, outcome
  runner.jsonl             runner events: episode_start, stage, planner_result, episode_end
  recording.json           what was recorded
  bag/  frames/            rosbag (mcap) and camera frames
  bag.stderr.jsonl  frames.stderr.jsonl
  executor/planner.jsonl         the planner's log
  executor/planner.stderr.jsonl  the planner process's raw stderr
  watchdog/                WatchDog session dir, transcript.jsonl, agent_config_watchdog.json
records/authoring/<session>/
  agent_config.json  session.json  evals.jsonl
records/pick_check/<stamp>.json
```

## 7. Where the v1 robot_backend pieces went

| v1 (WatchDog_V1 `robot_backend`) | v2 |
|---|---|
| motion/gripper/scene MCP servers that an LLM executor called | the planner's `worker.py` calling backends directly; the LLM writes the planner instead of driving the arm |
| `ros_moveit_backend/motion.py`: plan and execute through MoveGroup | plan: `cell_planning_service` (MoveIt or cuRobo); execute: `cell_backend_sim` FollowJointTrajectory to the bridge |
| `ros_moveit_backend/gripper.py`, `skills/gripper_skills.py` | `cell_backend_sim/gripper.py` (same RG2 controller); guarded grasp rule in `planner_templates/common/geometry.py` |
| `interfaces/*` | `cell_platform_interfaces` |
| `cell/gate.py` (per-step gating) | one start permission per episode, requested by the episode runner from `cell_orchestrator` |
| poses through TF / `/get_object_pose` | `/camera1/poses` (executor) and `/camera2/poses` (WatchDog) |
| WatchDog: `watchdog/*`, `watchdog_mcp_server`, `events`, `registry`, `contracts`, `vision` | kept in `robot_backend`, made plan-blind in Milestone 5 |
| `nodes/scene_tf_sync_node.py`, `utils/geometry.py`, `utils/rg2_geometry.py` | kept |

The executor modules were deleted in Milestone 7 (`docs/PROVENANCE.md`).
