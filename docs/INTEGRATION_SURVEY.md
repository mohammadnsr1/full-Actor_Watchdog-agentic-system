# Integration survey: DROIDSTACK as executor, WATCHDOG as supervisor

Survey date: 2026-11-02. Reading only; nothing was built, run, or edited in either source repository.

Repositories surveyed:

| Name | Path | Branch / commit (read from `.git/HEAD` and the ref file) |
|---|---|---|
| DROIDSTACK | `/home/mohammad/droidstack` | branch `dev`, commit `17192dbca327771cb63eb22688b7ff1d62921ea5` |
| WATCHDOG | `/home/mohammad/watchdog/WatchDog_V1` | branch `main`, commit `cb973bbf13fa420f02556d7cf5459d9146f3d06d` (remote `origin/main` is at `443a3fa8…`, so local main is ahead of or diverged from the remote; unknown from code which) |

Confidence vocabulary used throughout:

- **read**: stated directly in code, config, or docs at the cited `path:line`.
- **inferred**: follows from the cited code plus standard ROS 2 / rclpy / MoveIt behaviour that is not itself in the repo.
- **unknown from code**: cannot be determined from either repository; listed again in Part E.

All DROIDSTACK paths are relative to `/home/mohammad/droidstack`. All WATCHDOG paths are relative to `/home/mohammad/watchdog/WatchDog_V1`.

---

## The one finding that reframes everything

**DROIDSTACK is not an LLM agent. It is an evaluation harness.** (read)

- There is no model, no API client, no prompt loop, and no `anthropic`/`openai`/`claude` string anywhere in the repository (grep over `*.py *.md *.yaml *.json *.txt` returned none).
- The thing DROIDSTACK runs is a **"planner"**: a stdio MCP *server* authored by someone else that must expose exactly one tool, `execute_task(task_ref) -> {state: done|stopped, reason}` (`src/droidstack/harbor/constants.py:4`, `src/droidstack/harbor/manifest.py:127-130`). DROIDSTACK is the MCP *client* that starts that server, calls the one tool once, and stops it (`src/droidstack/harbor/runner.py:303-353`).
- The planner's own code is expected to drive the robot through ROS 2 actions from a separate "ROS worker" process (`examples/my-lab/planners/skeleton/server.py:25-44`, `docs/getting-started.md:242-264`).
- The LLM lives *outside* DROIDSTACK. The `droidstack author` subcommand serves an "authoring session" over stdin/stdout newline-delimited JSON with three operations: `prompt`, `evaluate_planner`, `submit_work` (`src/droidstack/cli/main.py:337-371`, `src/droidstack/harbor/authoring.py:274-369`). An external agent harness (not in this repo; **unknown from code** which one) reads the instruction bytes, edits the planner source in an isolated copy, asks for an evaluation, and submits a report.

So "DROIDSTACK takes about an hour per FMB assembly and works by planning and trial and error" most plausibly describes the *authoring loop*: the external LLM edits `server.py`/`ros_worker.py`, DROIDSTACK runs a full fresh simulator episode per `evaluate_planner`, the LLM reads the verdict, edits again. Each episode is one `execute_task` call bounded by a simulated-time budget. **This is inferred**; the labmate must confirm (Part E, Q1-Q4).

Consequences for the intended design, developed in Part D:

- "Executor context" has two possible meanings now: (a) the context of the *external LLM* that authors the planner, and (b) the *planner process* that runs inside DROIDSTACK's episode. Neither is reachable through a DROIDSTACK config file for message injection.
- The motion command path is the planner author's ROS client talking to DROIDSTACK's own `FollowJointTrajectory` adapter, not MoveIt.
- Everything the planner process can know arrives through one environment variable, `DROIDSTACK_CONTEXT`, and the ROS graph.

---

## Part A: DROIDSTACK

### A1. Overview

**Purpose** (read): "Bring declared robots, parts, and tasks into Isaac Sim, run a local MCP planner against a simulator-neutral ROS 2 contract, and score preserved evidence offline" (`README.md:3-4`). M1 is a Franka FR3 pick-and-place qualification; an external "benchmark workspace" road adds assemblies, mates, seeded layouts, prompt layers, and a workspace-owned checker (`docs/workspaces.md:58-215`).

**Language and version** (read): Python, `requires-python = ">=3.10"` (`pyproject.toml:9`). Measured deployment: system Python 3.10 with ROS 2 Humble; Jazzy on Ubuntu 24.04 with Python 3.12 is also supported through `ROS_SETUP` (`docs/getting-started.md:14-17, 36-38`; `docs/troubleshooting.md:46-48`). Isaac Sim 6.1's Kit process runs its bundled Python 3.12 (`docs/getting-started.md:16`).

**Top-level layout** (read, from the file listing):

| Folder | One line |
|---|---|
| `src/droidstack/` | The Python package: CLI, cell compiler, simulator adapters, planner admission/runner ("harbor"), benchmark runner, offline referee, dispatch. |
| `src/droidstack/cli/` | `droidstack` subcommands: init, doctor, preflight, assets, cell, planner, evaluate, dry-walk, dispatch, benchmark, author, referee (`cli/main.py:449-598`). |
| `src/droidstack/cell/` | Workspace loading, schema validation, compile to `CompiledCell`, `CellRunner` lifecycle, `evaluate_local`. |
| `src/droidstack/sim/` | `SimulatorAdapter` protocol (`base.py`), `FakeAdapter`, `isaacsim/` (Kit process owner + in-Kit command server + ROS 2 OmniGraph builder), lab capabilities, mate gateway, evidence tape. |
| `src/droidstack/harbor/` | Planner manifest, artifact hashing, MCP stdio client, one-call runner, episode bracket, authoring session. |
| `src/droidstack/benchmark/` | External benchmark inputs, prompt composition, layout generator, episode executor, validity receipt, checker resolution. |
| `src/droidstack/referee/` | Offline pose scorer and benchmark checker runner. |
| `src/droidstack/dispatch/` | Local attempt binding, hermetic ROS runtime process, SSH dispatcher. |
| `ros2/` | Seven ROS 2 packages: `droidstack_gripper_msgs`, `droidstack_assembly_msgs`, `droidstack_robot_contract`, `droidstack_robot_conformance`, `droidstack_joint_trajectory_adapter`, `droidstack_parallel_jaw_adapter`, `droidstack_mate_adapter`, `droidstack_robot_bringup`, `droidstack_robot_test_utils`. |
| `schemas/` | JSON Schemas for every public declaration (`droidstack.*/1`). |
| `examples/my-lab/` | The copyable Franka FR3 tabletop workspace template (site, environment, sim-binding, task, layout, planner skeleton). |
| `docs/` | Getting started, acceptance, workspaces, bring-your-own-robot, dispatch, troubleshooting, and `docs/examples/new-lab/` declaration examples. |
| `tests/` | Hostless pytest suite plus `ros`/`isaac` marked live gates; `tests/fixtures/m1_reference_planner/server.py` is a complete working planner. |
| `scripts/build-ros-workspace` | Hermetic colcon build wrapper. |

**Main entry points** (read): console script `droidstack = droidstack.cli.main:main` (`pyproject.toml:13`). Live production entrances: `evaluate_local` (`src/droidstack/cell/evaluate.py:49-106`), `BenchmarkRunner.run_episode` (`src/droidstack/benchmark/runner.py:99-143`), `open_benchmark_authoring` + `serve_authoring` (`src/droidstack/benchmark/authoring.py:20-65`, `src/droidstack/harbor/authoring.py:356-369`).

**Dependency files** (read): `pyproject.toml` only. Runtime dependency `PyYAML>=6,<7`; extras `m1 = ["mcp==1.30.0", "pydantic==2.14.1"]`, `test = ["pytest>=7", "jsonschema>=4"]` (`pyproject.toml:10-17`). ROS packages declare deps in their `package.xml` files. No lock file.

**License file**: none at the repository root (read, from listing). Only asset licenses exist (`examples/my-lab/assets/part/LICENSE.txt`, `examples/my-lab/assets/table/LICENSE.txt`). The planner must therefore be treated as private, as the task statement says.

### A2. Agent framework

- **Framework running the agent**: none inside DROIDSTACK (read; see the reframing section). DROIDSTACK is an MCP 1.30.0 *client* implemented by hand over newline-framed JSON-RPC (`src/droidstack/harbor/runner.py:62-300`, protocol version `2025-06-18` at `harbor/constants.py:5`). The planner skeleton uses `mcp.server.fastmcp.FastMCP` (`examples/my-lab/planners/skeleton/server.py:7, 62`). The external authoring agent is **unknown from code**.
- **Model configuration**: none anywhere (read; grep). No API key variable is read by any DROIDSTACK code.
- **Prompt files and system-prompt assembly**: for the benchmark road, the agent-facing instruction is composed from four ordered layers `platform`, `domain`, `scene_facts`, `task_layer` (`src/droidstack/benchmark/inputs.py:13, 108-157`). `platform`/`domain`/`task_layer` come from `task.yaml`'s `prompt:` block, either inline text or a named file under `<home>/prompts/<name>{,.md,.txt}` with optional SHA-256 pin (`inputs.py:198-214`); `scene_facts` is built from `<home>/assets/<asset>/description.md` for each `starting_scene.extra_objects` fixture (`inputs.py:217-249`). One template token `{evaluation_budget_episodes}` is substituted from the protocol (`inputs.py:119-123, 257-279`). The composed bytes are returned base64 by the `prompt` operation (`src/droidstack/harbor/authoring.py:284-294`). For the M1 road, `tasks/pick-and-place/instruction.md` exists but is not read by any code path I found; it is listed only as installed data (`pyproject.toml:40`) — **read, with the absence inferred from grep**.
- **How the agent is started**: `droidstack author --workspace … --home … --benchmark … --task … --planner … --session … --max-episodes N [--dry-run]` then NDJSON on stdin (`src/droidstack/cli/main.py:560-576`, `docs/workspaces.md:179-215`). Who pipes the LLM into that stdin is **unknown from code**.
- **Several agents or roles**: one planner per episode, one authoring session per planner (`harbor/authoring.py:118-161`). No multi-agent structure.

### A3. Inputs

What a run takes (read):

| Input | Where loaded |
|---|---|
| `site.yaml` (simulator adapter/version, deployment paths `isaac_install`, `ros_setup`, endpoints) | `src/droidstack/cell/workspace.py` via `load_workspace`; resolution order in `docs/workspaces.md:14-18` |
| `environments/<env>/environment.yaml` (stage USD, assets, robots rows, `frame_bindings`, `dynamic_parts`, regions) | `src/droidstack/cell/compile.py:51-103`; example `examples/my-lab/environments/tabletop/environment.yaml` |
| `environments/<env>/sim-binding.yaml` (USD articulation prim, DOF map, frame prims, pad colliders, object prims) | `examples/my-lab/environments/tabletop/sim-binding.yaml`; translated in `src/droidstack/sim/isaacsim/articulation.py` |
| `robots/<r>/binding.yaml`, `composite.yaml`, `kinematics-description.json`, `vendor-assets.lock.json` | `src/droidstack/harbor/artifact.py:96-133`; `ros2/contract/droidstack_robot_contract/droidstack_robot_contract/binding.py:149-287` |
| `tasks/<t>/task.yaml` (target pose, tolerances, required events, optional `mate_action`) | `artifact.py:99-101, 138-142`; `schemas/task.schema.json:11` |
| `environments/<env>/layouts/<l>.yaml` (seed, placements) | `compile.py:55-57, 76-79` |
| Planner delivery dir with `manifest.yaml` (`droidstack.planner-manifest/1`: command, cwd, env, dependencies, `budget.sim_s`, tools) | `src/droidstack/harbor/manifest.py:51-140` |
| Benchmark home: `tasks/<task>/task.yaml`, `assemblies/<a>/declaration.json`, `environments/<e>/declaration.yaml`, `benchmarks/<b>/protocol.yaml`, `prompts/`, `assets/<asset>/description.md` | `src/droidstack/benchmark/inputs.py:68-105` |
| Benchmark workspace: `runtime/manifest.json` (`entrypoint: "<file>.py:<factory>"`), `checker/manifest.json`, `layout-generator/manifest.json` | `src/droidstack/benchmark/runtime.py:38-115`; `benchmark/checker.py:27-43`; `benchmark/runner.py:193-233` |
| CAD/part files: workspace-relative USD assets referenced from `environment.yaml` and `scene.usda`; FMB board IDs do not exist as a concept in this repo | `examples/my-lab/environments/tabletop/scene.usda:10-29` |

**FMB-specific inputs are not in this repository** (read, by absence). The FMB benchmark home and workspace (tasks, assemblies, `runtime/episode.py`, checker, prompts, USD) must live in a separate tree owned by the labmate. **Unknown from code** where.

**Single place where the work instruction could be supplied from outside**: yes, two.

1. For the external LLM: the benchmark home's `task.yaml` `prompt:` block plus `prompts/` files, composed by `compose_agent_prompt` (`inputs.py:108-157`). Replace the file, and the agent gets different bytes; the hash is recorded.
2. For the planner process: everything it is told comes through one environment variable, `DROIDSTACK_CONTEXT`, a JSON object with `workspace, task_ref, task, robots, ros, ros_runtime, mate_action` (`src/droidstack/harbor/artifact.py:40-61, 160-171`). `task` is the whole `task.yaml` document passed through as `dict(task)` (`artifact.py:145`), so extra keys in `task.yaml` reach the planner unchanged **if** the schema admits them (`schemas/task.schema.json` was not fully read; **unknown from code** whether `additionalProperties` is false).

### A4. Planning and discovery loop

- **How it plans the assembly sequence and grasps**: DROIDSTACK does not. The planner author's code does. The reference planner computes a fixed pick/place sequence with its own damped-least-squares IK over a DH description from the workspace (`tests/fixtures/m1_reference_planner/server.py:62-154, 289-335`). The FMB planner's strategy is **unknown from code**.
- **Trial and error structure** (read): one episode = exactly one `execute_task` call (`src/droidstack/harbor/episode.py:167-170`). Retries across attempts happen only in the authoring session: each `evaluate_planner` freezes a byte-identical snapshot of the editable planner under `records/evaluations/NNNN/artifact`, runs `run_episode` or `run_panel` on it, and writes `evaluation.json` (`src/droidstack/harbor/authoring.py:171-191`, `benchmark/authoring.py:41-56`). Every evaluation is a fresh simulator cell lifecycle (`src/droidstack/benchmark/executor.py:306-312`). Memory between attempts beyond those files is the external agent's business (**unknown from code**). Within one episode, retries are whatever the planner code does; the reference planner has none.
- **What ends a run**: the planner returns `CompletionReport{state: done|stopped, reason}` (`harbor/runner.py:337-345`); or a harness error (budget, wall guard, process exit, protocol). The authoring session ends with `submit_work` `state: done` (accepted only if the artifact is unchanged since the last evaluation) or `stopped` (`harbor/authoring.py:193-235`). Success is decided only by the offline referee or workspace checker, never by the planner's claim (`docs/getting-started.md:311-312`).
- **Timeouts and attempt limits** (read):
  - `budget.sim_s` in the manifest, 1 to 86400 simulated seconds (`harbor/constants.py:13-16`); checked against `/clock` every 20 ms (`harbor/runner.py:151-163`, `constants.py:25`).
  - Wall guard for the call: `max(30 s, 4 × budget_sim_s)` (`harbor/runner.py:323-327`, `constants.py:28-31`).
  - **M1 `evaluate` road only**: a hard `SIGALRM` wall bound of **600 s** on the planner call (`src/droidstack/cell/evaluate.py:188`). The benchmark road (`benchmark/executor.py`) does not use `_wall_bound`. An hour-long episode is therefore only possible on the benchmark road, and only with `budget.sim_s` sized for it (**inferred**).
  - `max_episodes` per authoring session (`harbor/authoring.py:134-136, 240-249`).
  - MCP startup 5 s, stop grace 1 s (`harbor/constants.py:19-22`); cell readiness 180 s (`cell/constants.py:63`).
  - The reference planner's own waits: 20 s per action/state wait (`tests/fixtures/m1_reference_planner/server.py:13-15`).

### A5. Simulator and robot interface

- **Isaac Sim version**: 6.1 only (`src/droidstack/cell/constants.py:19`; `src/droidstack/sim/isaacsim/constants.py:15`; `docs/getting-started.md:16`). The adapter refuses other versions (`src/droidstack/sim/isaacsim/__init__.py:425-434`).
- **How the scene is built** (read): DROIDSTACK **launches its own headless Kit process** per cell: `<isaac>/isaac-sim.sh isaacsim.exp.base --no-window … --enable omni.kit.stagerecorder.core --exec _command_server.py` (`sim/isaacsim/__init__.py:449-460`), with a sanitized environment that strips `/opt/ros` paths and sets `ROS_DISTRO`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID`, and `LD_LIBRARY_PATH` to the bundled bridge (`__init__.py:172-213`). It talks to Kit over a per-attempt loopback TCP JSON command channel (`__init__.py:396-416`; `_command_server.py:1783-1849`). The workspace stage USD is opened with `open_stage_async` (`_command_server.py:465-540`), physics is applied before initialization, the ROS contract is bound, and a layout reset teleports rigid bodies (`_command_server.py:600-638`). The timeline is played only inside bounded windows: readiness stepping (`_stepping_start`, `_command_server.py:1488-1540`) and the planner-call progress window (`_progress_start`, `:1444-1485`), then paused (`_stepping_stop` `:1546-1560`). Teardown stops the process group (`__init__.py:1071-…`).
- **Robot and gripper models** (read): the shipped workspace is **Franka FR3** with its parallel gripper (`examples/my-lab/robots/franka/binding.yaml:3-4`; `environments/tabletop/sim-binding.yaml:3-31`). **Not UR5e, not RG2.** The strings `ur5e` and `rg2` appear only as forbidden tokens in a skeleton-purity test (`tests/test_planner_manifest.py:157`) and as a prim path in a stage-traversal test (`tests/test_isaacsim_ros_contract.py:1246-1252`). A UR5e/RG2 workspace would have to be declared externally (binding, sim-binding, composite, locked USD). The gripper contract supports exactly one driven jaw joint with linear or polynomial width coupling; a mimic chain is seeded from that joint (`binding.py:198-243`; `sim/isaacsim/ros2.py:647-717`; `docs/bring-your-own-robot.md:114-125, 150-154`). **Unknown from code** whether the labmate has such a workspace.
- **ROS 2 distro** (read): whatever `ROS_SETUP` exports as `ROS_DISTRO`; Humble measured, Jazzy accepted (`docs/workspaces.md:20-24`; `docs/troubleshooting.md:14`). Isaac must bundle `exts/isaacsim.ros2.core/<ROS_DISTRO>/lib` (`sim/isaacsim/constants.py:149`).
- **MoveIt**: **not used** (read; grep for `moveit` returns nothing). Motion is DROIDSTACK's own trajectory adapter, see A6.
- **Controllers** (read): three generic ROS nodes launched by `ros2 launch droidstack_robot_bringup robot.launch.py binding:=…` inside a hermetic `env -i` system-Python process (`ros2/bringup/droidstack_robot_bringup/launch/robot.launch.py:11-22`; `src/droidstack/dispatch/_ros_runtime.py:69-84`; `dispatch/local.py:498-548`):
  - `droidstack_joint_trajectory_adapter/trajectory_adapter` (node `<instance>_arm_adapter`): `FollowJointTrajectory` server; publishes `sensor_msgs/JointState` position targets.
  - `droidstack_parallel_jaw_adapter/parallel_jaw_adapter` (node `<instance>_gripper_adapter`): `Grip` action server; publishes `JointState` with effort.
  - `droidstack_mate_adapter/mate_adapter`: `ExecuteMate` action server, started only when the task declares `mate_action` (`_ros_runtime.py:268-320`).
  Inside Kit, OmniGraph `ROS2SubscribeJointState → IsaacArticulationController` nodes consume those command topics (`sim/isaacsim/ros2.py:1134-1162, 1317-1341`).
- **Asset paths for FMB parts**: none in this repository (read, by absence). **Unknown from code.**
- **Coordinate frames and TF names** (read): parent frame is literally `world`; child frames are `<instance>/<suffix>` for every binding frame (`base`, `tool`, `grasp`, `left_pad`, `right_pad`) published by `ROS2PublishRawTransformTree` nodes on the topic named by `sim-binding.yaml` `bridge_graph.publishers.tf.topic` (`/tf` in the example) (`ros2.py:1119-1133, 157`; `binding.py:92-93, 136-137`; `sim-binding.yaml:32-34`). **Parts are not published to TF**; only robot frame prims are TF targets (`ros2.py:991-998`). Joint names are `<instance>_<joint>` (`binding.py:89-90`), e.g. `franka_0_fr3_joint1`. The clock graph publishes `/clock` with best-effort QoS (`ros2.py:1040-1055`; `docs/getting-started.md:55-56`).
- **Can the scene file be shared with WATCHDOG's scene?** DROIDSTACK loads *a* USD stage path from `environment.yaml` into *its own* Kit process and refuses nothing about the stage's extra content: `_create_graph` removes only the graph prims it owns (`ros2.py:1023-1029`). So WATCHDOG's `assembly_room.usd` could in principle be the declared stage **if** it also satisfies DROIDSTACK's requirements (explicit physics scene, articulation prim and DOF names in `sim-binding.yaml`, locked vendor asset digests, `/World/<env>` hierarchy) (`docs/bring-your-own-robot.md:97-104`). It **cannot** be shared as a running simulator: DROIDSTACK cannot attach to an already-running Isaac; it only launches and owns one (`__init__.py:396-521`). See Part C.

### A6. Motion command path (most important)

There is no MoveIt and no `/joint_command` in DROIDSTACK. Every physical effect goes through one of four roads.

**Road 1: arm trajectory** (read unless marked)

1. Planner author's ROS worker (system Python, separate process started by `ros_worker_command`, `examples/my-lab/planners/skeleton/server.py:25-44`) creates `ActionClient(FollowJointTrajectory, context["ros"][0]["arm_action"])`. The name is derived, never hard-coded: `/<instance>/arm/follow_joint_trajectory` (`src/droidstack/harbor/artifact.py:241-252`; `binding.py:115-117`). Reference: `tests/fixtures/m1_reference_planner/server.py:102, 256-273`.
2. `TrajectoryNode._execute` (`ros2/adapters/droidstack_joint_trajectory_adapter/droidstack_joint_trajectory_adapter/node.py:59-115`) validates the goal, journals `arm_goal accepted` on `/<instance>/robot_facts`, then loops: publish `JointState` on `/<instance>/arm/joint_commands` (`node.py:32, 88-101`; `binding.py:107-109`), interpolated in sim time against `time_from_start` (`core.py:133-151`), until measured positions are within `goal_tolerance_rad` after the final time (`core.py:153-163`).
3. Kit OmniGraph `SubscribeArm (ROS2SubscribeJointState) → ControlArm (IsaacArticulationController)` applies the position targets to the articulation on every physics step (`sim/isaacsim/ros2.py:1134-1146, 1317-1325`).

   Excerpt, the loop that owns motion (`node.py:88-106`):
   ```python
   self._commands.publish(message)
   with self._condition:
       while self._core.status(self._now_s()) != "succeeded":
           self._condition.wait(timeout=self._binding.arm["measurement_timeout_s"])
           if self._measurement_error is not None:
               raise self._measurement_error
           command = self._core.command(self._now_s())
           ...
           self._commands.publish(message)
       ...
   handle.succeed()
   ```

- **Synchronous or asynchronous**: the action is asynchronous at the ROS level; the reference planner blocks on the result with a 20 s wait and treats any status other than `SUCCEEDED (4)` as a refusal (`fixture server.py:239-253`). The FMB planner's style is **unknown from code**.
- **Choke point for an external abort**: the `FollowJointTrajectory` goal *looks* like one, but the `ActionServer` is constructed with no `cancel_callback` (`node.py:35`), so rclpy's default cancel callback applies, which **rejects** cancel requests (**inferred** from rclpy defaults). The execute loop never checks `is_cancel_requested`. The only exits are success or a `TrajectoryError` (`MeasurementStaleError` after `measurement_timeout_s = 0.5 s` without a fresh `/joint_states` sample, or a bad sample) which aborts the goal with `INVALID_GOAL` and `error_string = "<ErrorType>: …"` (`node.py:109-115`; `core.py:169-173`; `binding.yaml:21`). So the practical external choke points are, in decreasing cleanliness:
  1. **Starve the adapter of fresh joint states.** Pausing the Kit timeline stops `/clock` and `/joint_states`; within 0.5 s the adapter aborts the goal with a normal `INVALID_GOAL` result. The planner then sees an ordinary failed action. Pausing is a Kit command on DROIDSTACK's loopback channel (`_stepping_stop`/`progress_stop` call `timeline.pause()`, `_command_server.py:1546-1560`), but that channel is single-request-per-connection, port chosen per attempt, and DROIDSTACK's own progress state machine will object to foreign stop/start calls (`:1451-1457`). The port is written to `runtime/isaac-launch.json` as `DROIDSTACK_COMMAND_PORT` because the redaction only masks names containing sensitive fragments (`src/droidstack/process.py:66-88`) — **inferred** that `PORT` is not such a fragment; the fragment list in `process_constants.py` was not read. A cleaner variant of the same idea is a WATCHDOG-side node that pauses physics through the running Kit's own extension (for example the `kit-mcp` bridge WATCHDOG already uses, `.mcp.json:33-41`); whether that works against DROIDSTACK's headless Kit is **unknown from code**.
  2. **Publish a competing hold target on `/<instance>/arm/joint_commands`** (reliable, keep-last depth 1, `binding.yaml:38`). The adapter keeps republishing its own target at each measurement tick, so the two publishers fight; the adapter would then time out on tolerance never being reached? No: there is no time limit in the loop, so it would hang until a stale measurement. Hacky and unsafe.
  3. **Kill the adapter node.** The planner's result future never completes; the hermetic ROS runtime treats a lost child as a lifecycle error. Not viable.
  There is **no** clean goal-cancel choke point without editing the adapter.

**Road 2: gripper** (read)

1. Planner → `ActionClient(Grip, context["ros"][0]["grip_action"])` = `/<instance>/gripper/grip` (`artifact.py:249`; `ros2/interfaces/droidstack_gripper_msgs/action/Grip.action`).
2. `ParallelJawNode._execute` (`ros2/adapters/droidstack_parallel_jaw_adapter/droidstack_parallel_jaw_adapter/node.py:101-168`): maps width to joint via the declared coupling, publishes one `JointState` with position and signed effort on `/<instance>/gripper/joint_commands` (`node.py:39, 110-114`), waits up to `goal_timeout_s` (5 s in the example) for either the width to be reached or a two-pad contact witness of `expected_object` (`node.py:115-128`; `core.py:110-124`), journals `grip_close/grip_open accepted`, then `hold` or `release`, and later `carry` when the arm moves while holding (`node.py:58-86, 146-152`).
3. Kit `SubscribeGripper → ControlGripper` (`ros2.py:1147-1162, 1328-1341`). Contact identity comes from PhysX full contact reports reduced to `{left_object, right_object, identity_known}` and published as JSON on `/<instance>/gripper/contact_witness` (`ros2.py:278-287, 564-579`; `sim/isaacsim/contacts.py:30-59`).

   No cancel callback either (`node.py:43`); bounded by `goal_timeout_s`, so a gripper goal always returns within seconds (**read**).

**Road 3: mate** (read)

`ExecuteMate` action at `task.mate_action` (e.g. `/assembly/execute_mate`) → `droidstack_mate_adapter` → Unix socket → `MateGateway` in the DROIDSTACK process → `LabClient.execute_mate` → Kit `lab` operation that **only appends the mate name to a journal**; `verify_assembly` passes when every declared mate is in the journal (`_command_server.py:989-1014`; `sim/mate.py:104-127`; `ros2/adapters/droidstack_mate_adapter/*`). No physics is applied by a mate (**read**). What physically seats a part is arm motion; what the FMB checker verifies is **unknown from code**.

**Road 4: in-process workspace action** (read)

On the benchmark road, the workspace's `EpisodePreparer.prepare()` returns an `action(plan, runtime)` callable that DROIDSTACK runs *inside its own process* during recording with full access to the adapter: `LabClient` (`resolve, frame, tool_pose_for_grasp, contacts_arm/read, attach, detach, execute_mate, at_rest, until_at_rest, reset_scene, verify_assembly`, `sim/lab.py:234-295`), `runtime.adapter.advance(cell, sim_s)`, and the planner bracket (`benchmark/executor.py:226-280`; `tests/test_benchmark_isaac_live.py:44-87`). `attach` creates a USD `FixedJoint` between two bodies (`_command_server.py:882-904`). So the labmate's `runtime/episode.py` can weld, teleport (via `reset_scene`), and step physics without any ROS traffic. **Unknown from code** what the FMB runtime does here; this is the single biggest hole in the "all motion goes through ROS" assumption.

### A7. Failure handling

**Harness side** (read):

- `McpServer.call_tool` raises `PlannerCompletionError` if the tool returns `isError` or malformed structured content; `PlannerBudgetExceededError` when sim time exceeds `budget_sim_s`; `PlannerWallGuardError` on the wall ceiling; `PlannerExitedError` (with 16 KiB stderr tail) if the server dies or closes stdout (`src/droidstack/harbor/runner.py:151-230`; `harbor/errors.py`).
- `PlannerRecordingBracket.call` wraps the call; any `BaseException` becomes `PlannerBracketError` after the recorder, bag, mate adapter, and progress window are closed (`src/droidstack/harbor/episode.py:167-227`).
- `evaluate()` catches `BaseException` into `primary`, still stops recording, ROS, and the cell, writes `planner-report.json` with `primary_error {type, message}` and the cleanup steps, then raises `EvaluationError` (`src/droidstack/cell/evaluate.py:191-265, 298-323`).

**Planner side**: entirely the author's code. The reference planner catches every `Exception` and returns `{"state": "stopped", "reason": "<Type>: <msg>"}` (`tests/fixtures/m1_reference_planner/server.py:51-59`), and its `_action_result` raises `PlannerRefusal` when the action status is not 4 (`:239-253`). **For that planner an externally stopped goal ends the episode rather than continuing.** The FMB planner's reaction is **unknown from code**.

**Would an externally cancelled trajectory look like an ordinary failed action, or crash/hang?**

- A cancel request itself does nothing (rejected, **inferred**), so nothing changes and the goal continues.
- A stopped clock (timeline pause) produces an ordinary `aborted` result with `INVALID_GOAL` and a `MeasurementStaleError` string within 0.5 s (`node.py:109-115`); the adapter node survives and accepts the next goal once joint states resume. From the planner's seat it is an ordinary failed action. The harness keeps running because its budget check reads sim time, which simply stops advancing; only the wall guard keeps ticking (`runner.py:151-163, 193-201`).
- Killing the ROS runtime or Kit mid-call crashes the episode (`RosLifecycleError`, `EvaluationError`).

### A8. Perception

- **How the planner knows where parts are** (read): it does not get part poses from DROIDSTACK at run time. The reference planner reads the *declared start pose* from the layout file and the *declared target* from `task.yaml`, and uses TF only for robot frames (`fixture server.py:107-118, 156-179`). TF carries only robot frame prims (`ros2.py:991-998`). The contact witness topic tells it which declared object the pads touch (`/<instance>/gripper/contact_witness`). Simulator ground truth for object poses is available only to **in-process** workspace code through `LabClient.frame(name)` (`_command_server.py:964-970`) and to the offline referee through the evidence tape sampled at `fps` for `evidence_frames` (`compile.py:112-120`; `_command_server.py:243-281`). Whether the FMB `runtime/episode.py` relays ground-truth poses to the planner through some side channel is **unknown from code**.
- **Cameras**: none. The OmniGraph built by DROIDSTACK contains only clock, joint-state, raw-TF, and command-subscribe nodes (`ros2.py:880-957`). Whether the labmate's stage USD contains camera prims or camera graphs is **unknown from code**.
- **Own success checks** (read): the planner may do what it likes (the reference planner checks `holding` after a close, `fixture server.py:276-287`). DROIDSTACK's verdict is offline: `referee/pose.py` scores the sealed tape against position/orientation tolerance, at-rest window, region, released, and required journal events (`src/droidstack/referee/pose.py:25-236`; `examples/my-lab/tasks/pick-and-place/task.yaml:7-35`). On the benchmark road, the workspace checker command receives `referee-request.json` on stdin and returns a verdict with `refusal_codes`/`failure_codes` (`benchmark/runner.py:140-143`; `referee/benchmark.py`), plus a platform-owned `rollout-validity.json` (`benchmark/executor.py:381-459`).

### A9. Message injection points (text into the executor's context without editing code)

Because there are two "executors", two lists, each ranked cleanest to hackiest.

**Into the planner process (the thing that moves the robot during an episode):**

1. **A ROS topic or service the planner's `ros_worker.py` subscribes to.** Cleanest *if* the labmate's planner already reads something like a status topic, otherwise it requires the labmate to add a subscriber in *their* planner (not a DROIDSTACK edit). **Unknown from code** whether any such subscription exists.
2. **`DROIDSTACK_CONTEXT`** (`artifact.py:160-171`): static per episode, built from `task.yaml` (`dict(task)` passes extra keys through, `:145`) and the binding. Good for standing requirements or a "retry ruling" that applies to the *next* episode, useless mid-episode. Schema tolerance of extra keys is **unknown from code**.
3. **`manifest.yaml` `server.env` `PLANNER_*` variables** (`manifest.py:97-107`): static, set at admission; the manifest lives in the planner delivery, which is hashed, so changing it changes the artifact identity (`artifact.py:255-278`).
4. **Files under the workspace** the planner chooses to read (`context["workspace"]`): a notice file dropped by WATCHDOG would be read only if the planner polls it. Hacky.
5. **stdin/stdout**: owned by the MCP framing; not usable. **stderr** is captured (16 KiB tail) into `planner-report.json`, output only.

**Into the external authoring LLM (the thing that plans and retries across episodes):**

1. **Checker verdict** (`verdict.json` with `refusal_codes`/`failure_codes`, written by the workspace checker, `benchmark/runner.py:140-143`) and the `evaluate_planner` response that names those verdict paths (`harbor/authoring.py:90-115`). A WATCHDOG halt notice and retry ruling can be folded into the checker's verdict for that episode. Clean, but per episode.
2. **Prompt layers** (`inputs.py:108-157`): standing requirements, static for the session.
3. **`evaluation.json` / `records/`** files the agent may read. Hacky, depends on the harness.
4. Anything in the external harness itself: **unknown from code**.

There are **no** config hooks, callback registries, plugin loaders, or MCP-server lists on the DROIDSTACK side. The only pluggable code path is the benchmark workspace `runtime/manifest.json` entrypoint, which DROIDSTACK imports and executes in-process (`benchmark/runtime.py:77-109`); that is the labmate's file, so it is the natural place for an integration shim if they agree.

### A10. Tool surface

- **Tools the planner server exposes to DROIDSTACK**: exactly one, `execute_task(task_ref: str) -> CompletionReport{state ∈ {done, stopped}, reason}` (`harbor/constants.py:4`; `manifest.py:127-130`; skeleton `server.py:47-62`). Admission refuses any other tool: discovered tools must equal the manifest surface and the manifest must declare only `execute_task` (`artifact.py:202-232`).
- **Operations the planner process can invoke**: ROS actions `/<instance>/arm/follow_joint_trajectory` (`control_msgs/FollowJointTrajectory`), `/<instance>/gripper/grip` (`droidstack_gripper_msgs/Grip`), `<task.mate_action>` (`droidstack_assembly_msgs/ExecuteMate`); topics `/clock`, `/<instance>/joint_states` (with efforts), `/tf`, `/<instance>/gripper/contact_witness`, `/<instance>/robot_facts` (`binding.py:103-125`; `ros2.py:1084-1089, 1282-1283`).
- **Authoring entrance operations**: `prompt`, `evaluate_planner{episodes, seed}`, `submit_work{state, report}` (`harbor/authoring.py:274-328`).
- **Can we add a tool via config?** **No.** There is no MCP server list in DROIDSTACK; DROIDSTACK is the client, and the single-tool rule is enforced at admission. An extra *ROS* interface the planner calls is possible only if the labmate's planner code calls it.

### A11. Logging and timing

Per attempt (M1 `evaluate` road; `docs/getting-started.md:289-312`; `cell/evaluate.py`):

| File | Content | Timestamps |
|---|---|---|
| `attempt.json` | Attempt binding (workspace, task, layout, endpoints, domain id, planner digest) | none |
| `planner-report.json` (`droidstack.planner-report/1`) | `planner.completion{state,reason}`, `artifact_sha256`, `started_pid`, `returncode`, `stderr` tail, `elapsed_sim_s`; `simulation_progress{started_at_sim_s, stopped_at_sim_s, budget_sim_s}`; `primary_error`; `cleanup[]` (`evaluate.py:298-323`) | simulated seconds only |
| `recording-manifest.json`, `*.evidence.jsonl`, `*.usd` | Sealed evidence tape (pose samples at `fps`, journal events, `simulation_progress` phases) and Stage Recorder USD, SHA-256 sealed (`sim/evidence.py:177-215`; `sim/isaacsim/__init__.py:826-903`) | simulated seconds (`_sample_evidence(record, sim_time_s)`, `_command_server.py:271-281`) |
| `verdict.json` | Referee verdict | none |
| `episode.mcap/` | `ros2 bag record --all --include-hidden-topics --storage mcap` for the duration of the planner call (`_ros_runtime.py:343-392`) | rosbag2 `recv_timestamp` is system clock (**inferred** rosbag2 behaviour) |
| `runtime/isaacsim.log`, `runtime/isaac-launch.json`, `runtime/ros-observer-evidence.json`, `runtime/bindings/robot-N.json` | Kit log; launch argv/cwd/env (env values redacted for sensitive name fragments, `process.py:72-79`); ROS graph snapshot via `ros2 topic list/info` (`ros_evidence.py:31-86`) | Kit log lines as Kit prints them |

Per benchmark episode: `layout.json`, `referee-request.json`, `rollout-validity.json` (includes `termination.planner_completion`, `executor.py:454-457`), `verdict.json`, `panel-result.json`; per authoring session: `records/evaluations/NNNN/{artifact/, evaluation.json}`, `records/submissions/NNNN/{<report>, submission.json}` (`harbor/authoring.py:177-235`).

- **Wall-clock UTC vs process time**: the evidence tape and planner report carry **simulated seconds**; no ISO wall-clock field was found in any artifact (**read**, by absence). WATCHDOG correlates by `wall_ns`; aligning the two will need the rosbag `recv_timestamp` or Kit log.
- **Final self-report**: yes. `planner-report.json → planner.completion` and `rollout-validity.json → termination.planner_completion`; the authoring `submit_work` report file (≤ 1 MiB UTF-8, `harbor/authoring.py:15, 206-207`).

### A12. Runtime facts

- **Expected run duration**: not stated anywhere (**read**, by absence). The only bounds are those in A4.
- **Resources**: NVIDIA GPU and driver, Isaac Sim 6.1 install (`ISAAC_SIM_INSTALL` or `site.deployment.isaac_install`), a sourced ROS overlay (`ROS_SETUP` or `site.deployment.ros_setup`) (`docs/getting-started.md:12-30`). No API keys are read by DROIDSTACK. CPU PhysX is what the shipped stage declares (`docs/m1-acceptance.md:99-101`).
- **Seeds and determinism**: layout `seed` (47 in the example, `layouts/start.yaml:5`), benchmark seeds from the protocol's ordered panel (`benchmark/runner.py:258-279`); physics regime is declared and read back (`sim/lab.py:34-104`); re-scoring is byte-deterministic (`docs/getting-started.md:355-362`).
- **Anything that resists a second agent process alongside it** (read):
  - It **owns** the Kit process and kills the process group on stop (`sim/isaacsim/__init__.py:1071-…`; `process.py`).
  - `ROS_DOMAIN_ID` from the environment, default 0 (`dispatch/local.py:463-473`; `dispatch/constants.py:3`); the ROS runtime is `env -i` hermetic, passing through only `ROS_LOCALHOST_ONLY`, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI` (`local.py:509-534`).
  - The planner process inherits only a whitelist of env names (`harbor/constants.py:43-59`).
  - `ros2 bag record --all` captures every topic on the domain, including WATCHDOG's cameras if present.
  - Fixed node names `droidstack_evaluation_observer`, `droidstack_isaac_contract`, `<instance>_arm_adapter`, `<instance>_gripper_adapter`, `mate_adapter`.
  - Graph conformance is checked live before recording and refuses if the derived action names are not served (`_ros_runtime.py:127-176`); a proxy action server with the same name would be a duplicate.
  - The SSH dispatcher refuses to start if any foreign GPU process exists (`docs/dispatch.md:7-9`); the local roads do not check.

---

## Part B: WATCHDOG

### B1. Map

Branch `main`, commit `cb973bbf13fa420f02556d7cf5459d9146f3d06d` (read from `.git/HEAD` and `.git/refs/heads/main`).

| Path | One line (read from `README.md:68-85` and the listing) |
|---|---|
| `ros2_ws/src/robot_backend/` | Executor stack: skills, three executor MCP servers (`motion`, `gripper`, `scene`), the WatchDog MCP server, monitor engine, recorder, vision, registry, event bus, gate client. |
| `ros2_ws/src/robot_backend_interfaces/` | `GetObjectPose`, `GetObjectPoses`, `SyncSceneObjects` services. |
| `ros2_ws/src/cell_orchestrator/` + `cell_sync_interfaces/` | The gate: `/cell/request_go`, `/cell/signal_ready`. |
| `ros2_ws/src/isaac_trajectory_bridge/` + `_interfaces/` | `FollowJointTrajectory` bridge to Isaac `/joint_command`; RG2 controller (`/rg2_controller/{open,close,set_position}`, `guarded_move` action). |
| `ros2_ws/src/sim_moveit_bringup/` | One launch file for the whole cell, camera TF, workspace bounds, point-cloud filter. |
| `ros2_ws/src/ur5e_moveit_config/`, `robot_description/`, `onrobot_rg_description/` | MoveIt config, UR5e URDF with RG2, RG2 xacros. |
| `ros2_ws/src/robot_backend/config/model_registry.yaml` | Part registry. |
| `agents/` | Prompts, task files per role/scenario, API-path runner, transcripts/sessions (gitignored). |
| `*.mcp.json`, `*.settings.json`, `run_watchdog.sh`, `run_executor.sh` | Per-role Claude Code launch configs. |
| `isaac/script_editor/`, `scripts/`, `reference_watchdog/`, `label_templates/`, `paper_results/`, `*_scorer.py` | Injection tools, helpers, scripted oracle, labels, batches, scorers. |
| `assets/usd/assembly_room.usd`, `config/`, `generated_urdf/` | Stage, UR5e configs, RG2 URDFs. |
| `docs/dev/` | Runbooks and dated records. |

### B2. Runtime components WATCHDOG needs

All inside one process, `watchdog_mcp_server`, which creates ROS node `watchdog_node` spun by a 4-thread `MultiThreadedExecutor` (`ros2_ws/src/robot_backend/robot_backend/mcp_servers/watchdog_mcp_server.py:657-770`):

| Component | Where | Subscribes / calls |
|---|---|---|
| Evidence recorder | `robot_backend/watchdog/recorder.py:300-363` | `/joint_states` (`:18, 358`), `/watchdog/events` (`:19, 361`), TF via `tf2_ros.Buffer` + listener sampled at 20 Hz for every enabled registry entry and `cell.gripper.*_frame` (`:20, 165-245, 398-…`). Buffers ~1 h (`:141-142`). |
| Monitor engine | `robot_backend/watchdog/engine.py:104-609` | Fed by recorder callbacks `on_tf_tick`/`on_joint_state` (`:449-455`); authored lanes on their own threads (`:150-302`). |
| Halt client | `robot_backend/watchdog/halt.py` | `/move_action/_action/cancel_goal` (`:116-121`). |
| Ready client | `robot_backend/watchdog/ready.py:99` | `/cell/signal_ready`. |
| Look / camera | `robot_backend/watchdog/image_buffer.py` | RGB `/camera/depth/rgb` or `/compressed` (`:56-57`), depth `/camera/depth/image_raw` (`:66`), `camera_info` `/camera/depth/camera_info` or `/camera_info` (`:75-76`), point cloud `/camera/depth/points` (`:91`, not published any more). `look()` in `robot_backend/vision/look.py`. |
| Event bus | `robot_backend/events/bus.py:69-213` | Publishes `std_msgs/String` JSON envelopes on `/watchdog/events` (`:96`) and appends `events_watchdog.jsonl` (`session.py:177-184`). |
| Session manifest | `robot_backend/watchdog/session.py:290-397` | `manifest.json` embedding the registry; session dir from `WATCHDOG_SESSION_DIR`, else `~/.watchdog/current_session`, else fresh (`:129-142`). |
| `mcp_calls_<role>.jsonl` | `robot_backend/mcp_servers/role_context.py:292-378` | Every tool call and answer, role from `WATCHDOG_TASK_PATH`/`EXECUTOR_TASK_PATH` (`:65-98`). |
| Orchestrator | `ros2_ws/src/cell_orchestrator/cell_orchestrator/node.py:216-221` | Serves `/cell/request_go`, `/cell/signal_ready`; abort state (`:186-194, 271-290, 337-352`). Launched by bringup (`sim_moveit_bringup.launch.py:385-394`). |
| Rosbag | `sim_moveit_bringup.launch.py:25, 404` | `/joint_states /tf /tf_static /watchdog/events` (+ `/camera/depth/rgb`, `/camera/depth/camera_info` when `record_images`). |
| Scene TF sync node | `robot_backend/nodes/scene_tf_sync_node.py:196-224` | Serves `/sync_scene_objects`, `/get_object_pose`, `/get_object_poses`, `/reset_tf`; executor-side. |

MCP configs per role (read): WatchDog `watchdog.mcp.json`, `watchdog_2.mcp.json`, `_s2`, `_s3` variants, each with only `robot-backend-watchdog` and env `WATCHDOG_TASK_PATH`, `WATCHDOG_SCENARIO`, optionally `WATCHDOG_AGENT_CONFIG` (`watchdog.mcp.json:22-35`; `watchdog_2_s3.mcp.json:22-36`). Executor `executor.mcp.json`, `executor_2.mcp.json`, `_s4`, `_rN` variants with `robot-backend-motion`, `-gripper`, `-scene` and env `EXECUTOR_TASK_PATH`, `CELL_GATE_TIMEOUT_SEC` (`executor_2.mcp.json:19-55`). Project `.mcp.json` carries only the `kit` TCP bridge on `OMNI_KIT_MCP_PORT=9009` (`.mcp.json:33-42`). Launch via `claude --mcp-config … --strict-mcp-config --settings …` (`run_watchdog.sh:75-78`; `run_executor.sh:124-127`).

### B3. Halt path

- **Where a halt is issued** (read): `MonitorEngine._handle_fire` calls `halt_client.halt()` when the fired monitor has `halt_on_fire` (`engine.py:496-514`); the agent tool `halt_execution(detail)` calls the same client (`watchdog_mcp_server.py:2683-2694`).
- **What it actually stops** (read + inferred): `HaltClient.halt` sends `action_msgs/CancelGoal` with an all-zero goal id to `/move_action/_action/cancel_goal`, cancelling **every active MoveGroup goal** (`halt.py:32-99, 114-121`). MoveIt then cancels the controller execution, which reaches the bridge's `FollowJointTrajectory` server; that server accepts cancels and returns `canceled` (`isaac_trajectory_bridge/isaac_trajectory_bridge/follow_joint_trajectory_server.py:229-231, 301-307`). The chain from MoveGroup cancel to controller cancel is **inferred** from MoveIt behaviour. Nothing is sent to the gripper. `abort_run` is different: it flags the orchestrator so the **next** `request_go` is refused with `reason="aborted"`; it does not stop motion (`watchdog_mcp_server.py:2696-2717`; `node.py:271-290`).
- **Would it stop a DROIDSTACK motion?** **No** (read). DROIDSTACK never sends a MoveGroup goal; its arm goals go to `/<instance>/arm/follow_joint_trajectory` on its own adapter, which rejects cancels (A6). The gate is also irrelevant: the DROIDSTACK planner never calls `/cell/request_go`.
- **Halts disabled for observe-and-record mode** (read): not a code switch. Three things together:
  1. The WatchDog system prompts' Authority section: "You NEVER intervene … you do not stop, pause or abort execution. The arm runs to completion whatever fires." (`agents/prompts/watchdog_system.txt:9-16`; `watchdog_system_s3.txt:9-16`).
  2. `halt_on_fire` defaults to `False` on `arm_monitor` and `arm_custom_monitor` (`watchdog_mcp_server.py:1610, 1695`) and is refused for `purpose="observation"` (`:1895-1897`).
  3. The design record states the study is record-only in every scenario and halting is "a future arm" (`docs/dev/OBSERVATION_DUTY_AND_SCENARIOS.md:324-332`).
  The code path (`halt_execution`, engine halt) remains fully wired and prewarmed (`watchdog_mcp_server.py:746-749`).

### B4. Executor visibility (everything to cut off by default)

Read:

1. **`/watchdog/events` subscription** in the recorder (`recorder.py:361`) delivering executor-origin types `begin_step, end_step, run_ended, skill_start, skill_end, phase` and the shared `look`/`observation` (`events/schema.py:563-580, 590-592`). Exposed through `get_events` (`:2329`), `get_execution_state` (`:1551`), `wait_for_event` (`:2210-2296`), `get_evidence` events channel (`:2348`), and `recorder.current_step()`.
2. **Engine coupling to skill events**: `on_skill_event` tracks the held object from `grasp_skill`/`release_skill` events and auto-disarms slip monitors on release (`engine.py:371-447`). Cutting the topic also removes effort-fire attribution and the auto-disarm; both must be replaced by physical detection.
3. **The WatchDog task file's `steps:` block** (`agents/tasks/fmb_assembly_2.watchdog.yaml`, loaded by `_load_task_steps` and `get_contract`, `watchdog_mcp_server.py:103-142, 1484`): this is plan knowledge handed to the WatchDog on purpose today.
4. **Gate step scoping**: `signal_ready(scope="step", step_id=…)` presumes knowledge of step ids (`:2720-2799`).
5. **Server-side run termination** reads `run_ended` from the executor to decide the terminal status (`:500-555, 1298-1430`); without executor events the server finalizes after two 180 s silences (`:455-459`).
6. **Shared session directory**: `mcp_calls_executor.jsonl`, `events_motion.jsonl` etc. land beside the WatchDog's files (`session.py:164-184`); no WatchDog tool reads them, but the operator does.

DROIDSTACK publishes **none** of these (no `/watchdog/events` publisher exists there), so items 1, 2, 5 are cut automatically; items 3 and 4 must be removed from the WatchDog's own configuration.

### B5. Physical events

**Detected today** (read, `robot_backend/watchdog/monitors.py:177-468`):

| Primitive | Feed | Detects |
|---|---|---|
| `relative_frame` | TF | distance between two frames departs from its armed reference by `> max_delta_m` (slip). |
| `stationarity` | TF | object moved `> epsilon_m` from armed pose (disturbance). |
| `region` | TF | object leaves or enters an axis-aligned box. |
| `effort_event` | `/joint_states` effort | named joint's `abs(effort)` violates `comparison, threshold` for `sustained_ticks` samples (over-clamp). |
| authored scalar/array monitors | recorder buffers, camera arrays | arbitrary Python within budgets (`watchdog/authored.py`, `engine.py:179-302`). |

**Missing from the integration vocabulary** (read, by absence):

- **contact**: no contact topic; only the effort proxy. (DROIDSTACK's `/<instance>/gripper/contact_witness` would supply pad contact identity.)
- **lift / release / grasp confirmed**: currently taken from executor skill events (`engine.py:393-416`), not from physics.
- **part approaching a feature**: no proximity primitive; `region` around the feature frame is a workaround.
- **effort rising**: `effort_event` is absolute-threshold, not a rate or trend.
- **motion started / stopped**: no robot-velocity or joint-stationarity primitive; `stationarity` is object-TF only.
- **workspace intrusion**: `region` works only on registry objects; intruders are found by authored camera monitors (`reference_watchdog/README.md`).
- Wake is only on monitor fires, executor events, or timeout (`watchdog_mcp_server.py:2274-2294`); there is no "physical event stream" that wakes the agent without an armed monitor.

### B6. Scene and robot

Read:

- **Isaac scene**: `assets/usd/assembly_room.usd` (`README.md:90`), plus `assets/usd/RG2_Good.usd`, `Collected_fmb_assemblies/`, `Collected_workspace/`. Isaac Sim version is **unknown from code** (README says "Isaac Sim"; Script Editor and `omni.kit.mcp` are referenced; `isaac/script_editor/__pycache__` contains CPython 3.11 bytecode, which suggests a Kit older than 6.1's Python 3.12 — **inferred**, weak).
- **Robot description**: `ros2_ws/src/robot_description/urdf/ur5e_ros.urdf` — links `world, base_link, shoulder_link … wrist_3_link, tcp, onrobot_rg2_*`; joints `shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint`, fixed `wrist_3_link_to_TCP`, `finger_joint` plus mimic knuckle joints (`:7-485`). MoveIt group `ur_manipulator` chain `base_link → tcp`, virtual joint `world_joint` parents `base_link` to `world` (`ur5e_moveit_config/config/ur5e.srdf:3-6`); controller `arm_controller` type `FollowJointTrajectory` (`moveit_controllers.yaml`). Bridge parameters: action `/arm_controller/follow_joint_trajectory`, command topic `/joint_command`, state topic `/joint_states` (`isaac_trajectory_bridge/launch/trajectory_bridge.launch.py:104-106`); RG2 controller namespace `/rg2_controller` (`:129`). A static identity TF `world → World` bridges MoveIt and Isaac frames (`sim_moveit_bringup.launch.py:343-357`).
- **TF frame names**: `world`, `base_link`, `tcp` (`model_registry.yaml:112-127`), object frames equal to the prim name (`u_brown`, `base1`, …), pose-only frames `*_placement`, `*_reorient`, `*_hole` (`model_registry.yaml:706-863`). Camera frame `camera_depth_optical_frame` at `[0.25, -0.2, 1.7]` in `world` (`sim_moveit_bringup/config/cameras.yaml`).
- **Part registry fields** (`model_registry.yaml`): top-level `cell{gripper{effort_joint_name, tcp_frame}, base_frame}` (`:112-127`), `acm_allowed_object_pairs` (`:138-225`), `objects.<id>{enabled, kind ∈ {object, frame}, object_id, tf_frame, lookup_frame, class_name, source, graspable, collision{type ∈ {box, mesh}, dimensions | mesh_path, scale, pose_offset}, spec{grasp{force_threshold_n}, tolerances{position_m, orientation_deg, lift_height_min_m, disturbance_tolerance_m, grasp_width_tolerance_m, grasp_slip_tolerance_m}, limits{max_clamp_effort, grasp_effort_floor}}}` (`:227-704`).
- **FMB assets**: meshes under `config/assets/*.obj` (`mesh_path` relative to `asset_root`, `:354`); the `fmb1` set (`base1, u_brown, u_orange, inverted_u_yellow, line_green`) at native scale, the `fmb2` set (`base2, u_green, hex_blue, hex_red, inverted_u_brown`) baked at 2× (`:545-558`, `scripts/bake_fmb_2x.py`).

---

## Part C: Compatibility

| Item | DROIDSTACK | WATCHDOG | Conflict or match |
|---|---|---|---|
| Python version | ROS side: system Python of the sourced distro (3.10 Humble measured, 3.12 Jazzy supported); planner venv `>=3.10` with `mcp==1.30.0`, `pydantic==2.14.1`; Kit bundled 3.12 (`docs/getting-started.md:14-38`; `pyproject.toml:9-17`) | system Python 3.12; `ros2_ws/.venv` with `mcp<2` (1.27.1 noted), `numpy<2`; `agents/.venv` with `mcp` 2.x (`README.md:89-99`; `ros2_ws/requirements.txt`; `watchdog.mcp.json:36`) | **Match on Jazzy/3.12**. Separate venvs avoid the `mcp` pin difference; never activate one in the other's shell (`watchdog.mcp.json:36`). |
| ROS distro | from `ROS_SETUP`'s `ROS_DISTRO`; Humble measured, Jazzy accepted | Jazzy (`/opt/ros/jazzy`, `watchdog.mcp.json:28`) | **Match if DROIDSTACK is deployed on Jazzy** (untested in its docs, **unknown from code** whether the labmate runs Jazzy). |
| Isaac Sim version | 6.1, enforced (`cell/constants.py:19`) | **unknown from code** | **Unknown**; if WATCHDOG's stage was authored for Isaac 4.x/5.x, loading it in 6.1 is untested. |
| Robot | Franka FR3 template; UR5e needs a new workspace declaration and a locked USD | UR5e (`ur5e_ros.urdf`) | **Conflict** until a UR5e DROIDSTACK workspace exists. |
| Gripper | generic `parallel_jaw` single driven joint, linear/polynomial width coupling, PhysX mimic chain seeded | OnRobot RG2 with calibrated width table and effort-guarded close (`rg2_controller.py`) | **Partial**: RG2's `finger_joint` + mimics fits the mimic path; a polynomial width coupling must be measured (`docs/bring-your-own-robot.md:114-125`). |
| Scene ownership | DROIDSTACK launches and kills its own headless Kit; loads the workspace stage; plays the timeline only in bounded windows | operator runs Isaac GUI with `assembly_room.usd` playing continuously; bringup assumes Isaac is already publishing | **Conflict**. Two simulators cannot represent one cell; WATCHDOG cannot attach to DROIDSTACK's Kit except through ROS topics it does not publish today, and DROIDSTACK cannot attach to WATCHDOG's Isaac. |
| TF frame names | parent `world`, children `<instance>/base|tool|grasp|left_pad|right_pad`; **parts not on TF**; joints `<instance>_<joint>` | `world`, `base_link`, `tcp`, parts by prim name, `*_placement` frames; UR joint names | **Conflict**: WATCHDOG's recorder and registry expect part and `tcp` frames; DROIDSTACK publishes neither. |
| Part names and IDs | workspace `frame_bindings` portable names (`part`) mapped to prims; FMB names **unknown from code** | registry `object_id` = prim name (`u_brown` …) | **Unknown** until the FMB workspace is seen. |
| Topics / actions | `/clock` (best-effort), `/<instance>/joint_states`, `/tf`, `/<instance>/arm/joint_commands`, `/<instance>/gripper/joint_commands`, `/<instance>/gripper/contact_witness`, `/<instance>/robot_facts`, actions `/<instance>/arm/follow_joint_trajectory`, `/<instance>/gripper/grip`, `<mate_action>` | `/clock`, `/joint_states`, `/joint_command`, `/tf`, `/tf_static`, `/watchdog/events`, `/move_action`, `/arm_controller/follow_joint_trajectory`, `/rg2_controller/*`, `/cell/*`, `/camera/depth/*`, `/get_object_pose(s)`, `/sync_scene_objects` | **No name clash** except `/clock` and `/tf` if both simulators ran; the real problem is that WATCHDOG hard-codes `/joint_states` (`recorder.py:18`) while DROIDSTACK namespaces it. Both record bags: DROIDSTACK's `--all` would also capture WATCHDOG's topics. |
| Package names | `droidstack_*` | `robot_backend*`, `cell_*`, `isaac_trajectory_bridge*`, `sim_moveit_bringup`, `ur5e_moveit_config`, `robot_description`, `onrobot_rg_description` | **Match** (no clash); node names also distinct. |
| Dependency versions | `mcp==1.30.0`, `pydantic==2.14.1`, `PyYAML 6`; Kit numpy inside Kit | `mcp<2` (1.27.1), `numpy<2`, `trimesh`, `usd-core`, pydantic (version unpinned) | **Match if kept in separate venvs**; DROIDSTACK's `mcp==1.30.0` is still `<2`, so one venv could even serve both, but `numpy<2` vs Isaac 6.1 Python is irrelevant because DROIDSTACK never imports numpy outside Kit. |
| ROS domain / RMW | `ROS_DOMAIN_ID` env (default 0), `rmw_fastrtps_cpp` forced for Kit (`sim/isaacsim/constants.py:146`) | unspecified (Jazzy default is `rmw_fastrtps_cpp`) | **Match** if WATCHDOG uses the same domain id and RMW. |

**Can both run against one shared simulator instance and one ROS graph?**

**Not as-is.** The blocker is scene ownership: DROIDSTACK is the simulator's owner by construction (it starts Kit, builds the ROS bridge graphs inside it, steps the timeline, records, and kills it), while WATCHDOG is a pure ROS consumer that expects an operator-run Isaac already publishing `/joint_states`, `/tf` with part frames, and `/camera/depth/*`. The only consistent arrangement is **DROIDSTACK's Kit as the single simulator and WATCHDOG as a consumer of what that Kit publishes**, which requires:

1. A DROIDSTACK workspace for the FMB cell with UR5e/RG2 (**labmate's**, unknown from code whether it exists).
2. Part poses and the `tcp` frame on TF. DROIDSTACK does not publish parts; the stage USD itself would have to carry its own OmniGraph TF/camera publishers (DROIDSTACK leaves foreign graph prims alone, `ros2.py:1023-1029`) or WATCHDOG's recorder would have to read poses another way. Whether the FMB stage already has such graphs is **unknown from code**.
3. WATCHDOG's recorder topic (`/joint_states`, `recorder.py:18`) and effort joint name (`model_registry.yaml:114`) reconfigured to `/<instance>/joint_states` and `<instance>_finger_joint`; DROIDSTACK's `/<instance>/joint_states` does carry efforts (`ros2.py:1282-1283`).
4. Tolerance of a paused `/clock` outside planner windows (readiness stepping and progress only), and of `use_sim_time`.
5. Same `ROS_DOMAIN_ID` and RMW.

Running WATCHDOG's own Isaac + MoveIt cell and pointing DROIDSTACK at it is **not possible** without editing DROIDSTACK: the adapter has no "attach to existing simulator" mode.

---

## Part D: Feasibility verdict

| # | Need | Verdict | Mechanism and references |
|---|---|---|---|
| 1 | Supply the work instruction from outside | **Yes** | Benchmark home `task.yaml` `prompt:` block + `prompts/` files, composed and hashed by `compose_agent_prompt` (`src/droidstack/benchmark/inputs.py:108-157`), delivered to the authoring agent by the `prompt` operation (`harbor/authoring.py:284-294`). The planner process receives the task document via `DROIDSTACK_CONTEXT` (`harbor/artifact.py:160-171`). Caveat: this reaches the *external* agent; how that agent is driven is unknown from code (Q1). |
| 2 | Abort the current motion from outside, executor sees a normal failure | **No** for a clean cancel; **yes with workaround** (hacky) | The arm adapter rejects cancels (no `cancel_callback`, `droidstack_joint_trajectory_adapter/node.py:35`, rclpy default, inferred) and its loop has no cancel check (`:88-106`). Workaround: starve it of joint states by pausing the Kit timeline; after `measurement_timeout_s` (0.5 s, `binding.yaml:21`) the goal aborts with `INVALID_GOAL` and a `MeasurementStaleError` string (`node.py:109-115`), which is an ordinary failed action; the adapter stays alive. Reaching the timeline needs DROIDSTACK's per-attempt loopback command channel (`_command_server.py:1783-1849`) or a Kit-side pause from inside the stage; both collide with DROIDSTACK's progress state (`:1451-1457`). Whether the FMB planner continues after a failed action is unknown (the reference planner stops the episode, `fixture server.py:239-253`). Also note Road 4 (`LabClient.attach`, `advance`) bypasses ROS entirely (`benchmark/executor.py:226-280`). |
| 3 | Deliver a halt notice and retry ruling into the executor's context | **Yes with workaround**, two layers | Live, into the planner process: only via a ROS topic/service the labmate's `ros_worker.py` subscribes to (their code, not a DROIDSTACK edit); no DROIDSTACK hook exists (A9). Per episode, into the authoring agent: fold the ruling into the workspace checker's `verdict.json` `refusal_codes`/`failure_codes` and explanation (`benchmark/runner.py:140-143`; `referee/benchmark.py`), which `evaluate_planner` returns by path (`harbor/authoring.py:90-115`). Static rulings for the next episode: extra keys in `task.yaml` pass through `DROIDSTACK_CONTEXT` (`artifact.py:145`) if the schema allows (Q7). |
| 4 | Executor announces intent and "done" through an added tool | **No** as an MCP tool; **yes with workaround** as ROS | Admission enforces exactly one tool (`manifest.py:127-130`; `artifact.py:214-218`). But DROIDSTACK's adapters already publish a structured fact journal on `/<instance>/robot_facts` (`arm_goal accepted/result`, `grip_close/open accepted`, `hold`, `carry`, `release`; `binding.py:123-125`; `node.py:82, 107, 146-152`), and `ExecuteMate` is an explicit "mate done" claim (`ExecuteMate.action`). These are untrusted hints by construction. A dedicated intent topic needs the labmate's `ros_worker.py` to publish it. The authoring agent's "done" is `submit_work{state: done}` (`harbor/authoring.py:193-235`). |
| 5 | Read the executor's final self-report | **Yes** | `planner-report.json → planner.completion{state, reason}` (`cell/evaluate.py:303-311`); benchmark road `rollout-validity.json → termination.planner_completion` (`benchmark/executor.py:454-457`); `verdict.json`; authoring `records/submissions/NNNN/<report>` and `submission.json` (`harbor/authoring.py:219-235`). |
| 6 | Hide executor activity from WatchDog, still log it for scoring | **Yes** | DROIDSTACK publishes nothing on `/watchdog/events`, so recorder/engine/executor-event coupling (`recorder.py:361`; `engine.py:371-416`) is silent automatically. Remove the `steps:` block from the WatchDog task file (`watchdog_mcp_server.py:103-142`) and do not use step-scoped `signal_ready`. Do **not** relay `/<instance>/robot_facts` to the WatchDog. For scoring: DROIDSTACK's `episode.mcap` (`--all`, `_ros_runtime.py:349-360`), evidence tape, `planner-report.json`, stderr tail, and the authoring `records/`; add `/<instance>/robot_facts` to WATCHDOG's bag list (`sim_moveit_bringup.launch.py:25`) if desired. Caution: the server's termination guarantee finalizes after two silent 180 s waits (`:455-459, 1298-1430`); an hour-long episode with no events will trip it unless monitors fire or the timeout is raised. |
| 7 | Share one scene with WatchDog's observation channels, cameras, effort topics | **No** as-is; **yes with workaround** | Single simulator must be DROIDSTACK's Kit. Needs: FMB stage declared as the workspace stage with physics, articulation, locked assets (`docs/bring-your-own-robot.md:97-104`); part and `tcp` TF publishers plus camera publishers authored in that stage (foreign graphs survive, `ros2.py:1023-1029`; `isaacsim.ros2.nodes` is enabled, `ros2.py:258`); WATCHDOG recorder topic/joint renames (`recorder.py:18`; `model_registry.yaml:114, 119`); same `ROS_DOMAIN_ID`. Effort is already on `/<instance>/joint_states` (`ros2.py:1282-1283`). Unknown: Isaac version of the WATCHDOG stage (Q11), whether its graphs exist (Q12). |

---

## Part E: Questions for the labmate

1. Which program drives the `droidstack author` stdin/stdout session (Claude Code, a custom harness, something else), and where does its code live?
2. Which road do you use for FMB: `droidstack evaluate` (600 s wall bound on the planner call) or `droidstack benchmark`/`author` (no such bound)? What `budget.sim_s` do you declare?
3. Is "about an hour per assembly" the wall time of one `execute_task` episode, or of the whole authoring session including repeated `evaluate_planner` calls?
4. Where is the FMB benchmark home (tasks, assemblies, prompts) and the benchmark workspace (`runtime/episode.py`, checker, layout generator)? Can we read it?
5. Does your `runtime/episode.py` action use `LabClient.attach`/`detach`, `advance`, `reset_scene`, or `execute_mate` to seat parts, or is every physical effect produced by arm and gripper actions?
6. Inside one episode, does your planner's `ros_worker.py` subscribe to any topic or service it could receive a notice on, and does it continue after an aborted `FollowJointTrajectory` goal or stop the episode like the reference planner?
7. Does your `task.yaml` schema (or `schemas/task.schema.json` as deployed) accept extra keys, so a ruling placed there reaches `DROIDSTACK_CONTEXT`?
8. Do you have a UR5e/RG2 DROIDSTACK workspace (binding, sim-binding, composite, locked USD, polynomial width coupling), or is the FMB cell still Franka?
9. Which ROS distro and system Python does your DROIDSTACK deployment use (Humble/3.10 or Jazzy/3.12)?
10. Which `ROS_DOMAIN_ID` and RMW do you run with, and is `ROS_LOCALHOST_ONLY` set?
11. Which Isaac Sim version authored `assembly_room.usd` on the WatchDog side, and have you opened it in Isaac Sim 6.1?
12. Does your FMB stage USD contain its own OmniGraph publishers for part TF frames and cameras, or do you rely on DROIDSTACK's robot-only TF?
13. How does your planner learn part poses mid-episode: layout declaration only, `LabClient.frame` relayed through a side channel, or something else?
14. Where in your authoring loop does the agent read verdicts, and would it read extra fields (for example a WatchDog ruling) in `verdict.json` or `evaluation.json`?
15. Is it acceptable for a WatchDog-side process to pause the Kit timeline (or otherwise stop `/joint_states`) to abort a motion, given that your adapter then reports `MeasurementStaleError`?
16. Can a DROIDSTACK run share the GPU with a second Kit process if we ever need WATCHDOG's own Isaac, or is one Kit the hard limit on your host?
17. What is the `mate_action` name and the list of mates in your FMB task, and does the checker verify seating from pose evidence or only from the mate journal?
18. Is the DROIDSTACK `dev` branch at `17192db` the version you run, and is there a public remote we may cite?

---

## Part G: Adopted direction (added 2026-11-03)

This part records what was learned from the labmate after the survey and the design decision taken on it. It supersedes the "DROIDSTACK's Kit as the single simulator" framing in Parts C and D wherever the two differ.

### G1. What the labmate clarified

- **The agent writes the planner and the environment.** In the labmate's use, an LLM agent (Claude-class) authors the MCP planner server *and* the workspace that DROIDSTACK loads: the USD stage, and any extra OmniGraph action graphs for TF and part poses. This matches the authoring road in A4 (**read**, `harbor/authoring.py`), with one correction to the labmate's wording: the agent does not *call* DROIDSTACK's MCP server. DROIDSTACK is the MCP **client**; the agent-written planner is the MCP **server** exposing the single tool `execute_task` (`harbor/manifest.py:127-130`). Part TF, where it exists, is a graph the agent added to its own stage, not a DROIDSTACK feature (A8; `ros2.py:1023-1029` keeps foreign graphs).
- **The planner does its own IK, no sampling-based planning.** The labmate reports the agent plans and executes, computing inverse kinematics itself and sending joint-space goals; there is no sampling-based motion planner in the loop. This is consistent with the code: DROIDSTACK ships no planner and no IK. The lab client offers only pose arithmetic (`tool_pose_for_grasp`, `object_pose_for_grasp`; confirmed by the live contract test `tests/test_benchmark_isaac_live.py:53-56`), and the trajectory adapter interpolates whatever joint targets it is given (`core.py:133-151`). Source: labmate's statement, **not read from code**.

### G2. Decision: borrow the contract, keep our ROS graph

The DROIDSTACK **runtime** (its Kit launch, bridge graphs, adapters, episode executor) is **not** adopted. Our existing Isaac scene, trajectory bridge, cameras, part TF, and WatchDog stay as they are (B2, B6). Three DROIDSTACK ideas are adopted:

1. **The planner contract.** The executor is an agent-authored planner process that exposes exactly one MCP tool, `execute_task(task_ref) -> {state: done | stopped, reason}` (`harbor/manifest.py:127-130`; `harbor/artifact.py:214-218`). The agent edits the planner source between episodes; nothing edits it during an episode. The planner's ROS worker drives our robot directly.
2. **The intent journal.** The planner publishes a `robot_facts`-style topic announcing `arm_goal`, `grip_close`/`grip_open`, `hold`, `carry`, `release`, and a "mate done" claim, modelled on DROIDSTACK's fact journal (`droidstack_robot_contract/binding.py:123-125`; adapter `node.py:82, 107, 146-152`). This satisfies need 4 (Part D) without adding a second MCP tool. The facts are untrusted hints: they are logged for scoring and never relayed to the WatchDog session (B4).
3. **The episode loop.** Fresh scene, run planner, checker verdict, agent reads the verdict and rewrites the planner. In DROIDSTACK this is `evaluate_planner` returning the path to `verdict.json` (`harbor/authoring.py:90-115`; `benchmark/runner.py:140-143`). On our side, the orchestrator's `/cell/request_go` and `/cell/signal_ready` gate (`cell_orchestrator/node.py:216-221`) already marks episode boundaries; a verdict file written at `signal_ready` closes the loop.

### G3. Consequences for the integration

- **MoveIt is dropped from the executor's path but a planner stays in our system.** Given G1, the agent's planner will do IK and straight-line joint moves on its own. Collision avoidance around the fixture would then be entirely the agent's responsibility. Decision: keep a motion planner available on our side as a service the agent's ROS worker may call, so a collision-checked path is one call away rather than something each authored planner must reinvent. Which planner (MoveIt's `/move_action` kept as an optional service, or a lighter IK-plus-collision-check service) is open; see G4.
- **Halt becomes clean.** The agent's planner sends `FollowJointTrajectory` goals to our bridge server `/arm_controller/follow_joint_trajectory`, which accepts cancels and returns `canceled` (`isaac_trajectory_bridge/.../follow_joint_trajectory_server.py:229-231, 301-307`). `HaltClient` must therefore target that action's `cancel_goal` service instead of `/move_action/_action/cancel_goal` (`halt.py:114-121`), or both if MoveIt stays as an optional service. This replaces the timeline-pause hack of D2 and upgrades need 2 to **Yes**.
- **Observation surface is unchanged.** WatchDog keeps `/joint_states`, `tcp`, part frames by prim name, and the depth camera topics (B2, B6). None of the renames listed in C and D7 are needed, and the scene-ownership conflict in C disappears because there is one operator-run Isaac.
- **Executor visibility is unchanged.** The planner publishes nothing on `/watchdog/events`; strip the `steps:` block from the WatchDog task file as in D6. Add the intent-journal topic to the bringup bag list (`sim_moveit_bringup.launch.py:25`) so it is scored but not seen.
- **Message injection.** The halt notice and retry ruling (need 3) go in by two layers, as in D3: live, a ROS topic the planner's ROS worker subscribes to by contract; per episode, extra fields in the verdict file the agent reads before rewriting. Both are now ours to define, since the planner contract is ours, not the labmate's.
- **Part poses and CAD.** The planner reads part poses from `/tf` and the existing `/get_object_pose(s)` service (Part C table), and receives CAD and assembly description through the task document, mirroring `DROIDSTACK_CONTEXT` (`harbor/artifact.py:160-171`). No DROIDSTACK lab-client calls (`attach`, `advance`, `execute_mate`) are available; every physical effect comes from arm and gripper actions.

### G4. Open points (ours, not the labmate's)

1. Which planner to keep as a service: MoveIt `/move_action` retained but optional, or a smaller IK plus collision-check service.
2. The exact contract for the intent-journal topic and the halt-notice topic (message types, names), to be fixed before the first authored planner.
3. Where the verdict file lives and what the checker verifies for the FMB task (pose evidence vs. journal claims), the same question as E17 but now answered on our side.
4. Whether the authoring agent runs under Claude Code with a restricted settings file, as WatchDog's does (`watchdog.settings.json`), so the executor's and WatchDog's transcripts stay independent.

---

## Paste back summary

**What DROIDSTACK is and how it runs.** DROIDSTACK is not an LLM agent. It is a benchmark harness that launches its own headless Isaac Sim 6.1 process, builds ROS 2 bridge graphs inside it, starts three of its own ROS adapter nodes (arm trajectory, parallel-jaw gripper, mate journal), and then calls exactly one MCP tool, `execute_task`, on a "planner" server written by the labmate. The planner's own ROS code drives the arm. The LLM sits outside the repo: a separate harness talks to `droidstack author` over stdin/stdout, reads the composed instruction, edits the planner source, asks for a fresh simulator episode, reads the verdict, and repeats. "An hour of planning and trial and error" is most likely this authoring loop, each iteration a full episode. There is no MoveIt, no camera, no model config, and no API key anywhere in DROIDSTACK. Parts are not published on TF; only robot frames are.

**Exact motion choke points.** The only arm path is the planner's `FollowJointTrajectory` goal to `/<instance>/arm/follow_joint_trajectory`, served by DROIDSTACK's adapter, which republishes `JointState` targets on `/<instance>/arm/joint_commands` until the measured joints reach tolerance. That server has no cancel callback, so cancels are rejected and the loop never checks for them. The one external lever is starving it of `/joint_states`: pause the Kit timeline and within 0.5 s the goal aborts with an ordinary `INVALID_GOAL` result, which the planner sees as a normal failed action. Reaching the timeline means DROIDSTACK's per-attempt loopback command channel, which fights DROIDSTACK's own progress state machine, so it is a hack. The gripper action is self-bounded at 5 s. A fourth road exists on the benchmark track: the workspace's own Python runs inside DROIDSTACK with `attach`, `detach`, `advance`, and `reset_scene`, bypassing ROS entirely; we must learn whether the FMB runtime uses it.

**Best message injection point.** There is none inside DROIDSTACK for a live message. Into the planner process, only a ROS topic that the labmate's `ros_worker.py` subscribes to. Into the authoring agent, per episode, the workspace checker's `verdict.json` (`refusal_codes`, `failure_codes`, explanation), which the `evaluate_planner` response points the agent to. Standing requirements go in the prompt layers; next-episode rulings can ride as extra keys in `task.yaml`, which pass through to the planner's `DROIDSTACK_CONTEXT` environment variable.

**Verdicts.**

| Need | Verdict |
|---|---|
| 1 Instruction from outside | Yes (prompt layers, `DROIDSTACK_CONTEXT`) |
| 2 Abort current motion, seen as normal failure | No cleanly; yes with the timeline-pause hack |
| 3 Halt notice + retry ruling into context | Yes with workaround (ROS topic in labmate's planner; checker verdict per episode) |
| 4 Intent/"done" via added tool | No as MCP tool (exactly one allowed); yes via ROS, and `/<instance>/robot_facts` already exists |
| 5 Read final self-report | Yes (`planner-report.json`, `rollout-validity.json`, submission report) |
| 6 Hide executor activity, still log | Yes (DROIDSTACK emits no `/watchdog/events`; strip WatchDog task `steps:`; log via mcap and tape) |
| 7 One shared scene with cameras and efforts | No as-is; yes if DROIDSTACK's Kit is the single simulator and the stage carries its own TF/camera graphs |

**Top three compatibility conflicts.**

1. Scene ownership: DROIDSTACK owns and kills its simulator; WATCHDOG expects an operator-run Isaac with MoveIt. Only "DROIDSTACK's Kit as the single sim, WATCHDOG as consumer" is consistent.
2. Observation surface: WATCHDOG hard-codes `/joint_states`, `tcp`, and part frames by prim name; DROIDSTACK publishes `/<instance>/joint_states`, `<instance>/tool`, and no part frames or cameras.
3. Robot: DROIDSTACK ships Franka FR3; a UR5e/RG2 workspace with a measured RG2 width coupling must exist on the labmate's side, and the Isaac version of the WATCHDOG stage is unknown.

**Top five questions for the labmate.**

1. Which program drives the `droidstack author` session, and where is its code?
2. Where are the FMB benchmark home and workspace, and does `runtime/episode.py` seat parts through `attach`/`advance` or only through arm and gripper actions?
3. Does your planner continue after an aborted trajectory goal, and does it subscribe to any topic we could send a notice on?
4. Do you have a UR5e/RG2 DROIDSTACK workspace, on which ROS distro, and does its stage carry part-TF and camera publishers?
5. Is pausing the Kit timeline to stop a motion acceptable to you, and which road and `budget.sim_s` do you run with?

**Update, 2026-11-03 (Part G).** The labmate confirmed that their agent authors both the planner and the environment, and that the planner does its own IK with no sampling-based motion planning. Decision: do not adopt the DROIDSTACK runtime; keep our Isaac scene, bridge, cameras, part TF, and WatchDog. Adopt three DROIDSTACK ideas: the one-tool planner contract (`execute_task`, agent rewrites the planner between episodes), a `robot_facts`-style intent journal (answers need 4 without a second tool), and the episode loop closed by a verdict file at our existing `/cell/request_go` gate. Keep a motion planner available on our side as an optional service so authored planners are not forced to solve collision avoidance alone. Because the agent's goals now go to our bridge server, which accepts cancels, halt becomes a clean goal cancel and need 2 becomes **Yes**; needs 7 and the scene-ownership conflict disappear. Remaining open points are ours: which planner service to keep, the intent and halt-notice topic contracts, the verdict file, and the executor's Claude Code settings.
