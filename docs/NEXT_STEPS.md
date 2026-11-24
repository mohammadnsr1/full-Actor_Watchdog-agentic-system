# Next steps

Updated 2026-11-23 during Milestone 7 (v2), sim side. How everything
connects: `docs/ARCHITECTURE.md`.

## State

- M1 to M6: done and checked (M6 live checks below).
- M7 sim side: built and unit-tested here; the live checks below are next.
  Nothing since your last commit is committed. Hardware side of M7 follows.

## What Milestone 7 added

| Path | What |
|---|---|
| `docs/PLANNER_CONTRACT.md` | What a planner is (a stdio MCP server with one tool, `execute_task`), what the runner gives it, what it may touch, what it reports. |
| `executor/planner_templates/` | `common/` (`run.sh`, MCP `server.py`, ROS `worker.py`, `geometry.py`), `plan_given/` (executes the task file's steps), `plan_free/` (skeleton: gate, safe pose, halt handling, no assembly logic). |
| `config/tasks/fmb_assembly_2.yaml`, `.plan_free.yaml` | The executor task (moved from `agents/tasks/`), and its plan-free derivation (`tools/derive_plan_free_task.py`: drops `step_plan` and `steps`, redacts the step grasp-width clause). |
| `runner/episode_runner/` | One episode: lock, health, reset + verify, recording, WatchDog session, `execute_task` under the sim budget and wall guard, `episode_end`, acceptance wait, ground-truth checker, `manifest.json`, `verdict.json`, `runner.jsonl`. |
| `executor/authoring_mcp/` | `get_task`, `evaluate_planner`, `read_verdict`, `submit_work`; feedback filtered by `config/cell.yaml executor_feedback`. |
| `runner/run_authoring.sh` | `[platform] [plan_given\|plan_free] [task]`: workspace under `executor/workspace/<condition>/`, bookkeeping under `records/authoring/<session>/`, condition-specific settings. |
| `config/settings/executor.*.json`, `config/mcp/executor.*.json` | Authoring permissions per platform and condition (plan-free also denies every place the plan is written). |
| `runner/jsonl.py` | JSON Lines for every log in `records/` (runner events, process stderr, planner log, authoring evals). |
| Synchronisation | One start permission per episode, held by the runner: it starts the WatchDog, waits for its `signal_ready` (`cell_orchestrator`, `phase="run"`, up to `watchdog.ready_timeout_s`), then starts the planner; budgets start there. No planner gate. Episodes declared to the orchestrator (`episode_id` resets readiness and abort; `guard_mode` unguarded without a WatchDog, so no wait). WatchDog prompt: prepare from the requirements and the physics before ready, then wake on physics only. |
| WatchDog roles | Prompt rewritten around three roles: conformance and safety; risk analysis (halt only for damage risk, repetition included, or unrecoverability; failed attempts are exploration); free-form consultation (`issue_consultation`), which reaches the running planner when consult is on (`w.consultations()`) and the authoring agent in its feedback (`watchdog_to_executor`). An `escalate` ruling in `halt_enabled` mode ends the episode. |
| Executor prompt | `executor/prompts/authoring_system.md` + `condition_<condition>.md`, appended by `run_authoring.sh` (`--append-system-prompt-file`); copy in `records/authoring/<session>/system_prompt.md`. |
| Where parts go | Placement frames derived from the fixture (`config/placements/`, `tools/measure_placements.py`) for the WatchDog, `cell_physical_events` and the checker; `assembly_reference` (WatchDog) and `get_assembly_views` (executor) serve the pictures in `config/assembly_views/<assembly>/`; camera1 RGB-D in the worker (`camera1()`, `save_frame()`), saved frames in the authoring feedback; plan-free leak guards (runner check, settings, worker, lint). `cell_physical_events` turns pad contacts into `contact_made` / `contact_lost`. |
| Thresholds | Starting `physical_events` and `health` values in `config/cell.yaml`, measured from the r3 bag (all detectors now on). |
| Prune | The v1 executor stack, the S3 WatchDog session, unused assets and configs, and tests of v1 code that is not in this repo (list in `docs/PROVENANCE.md`, Milestone 7). |

Checked here: planner templates, authoring MCP, derive tool, settings, JSONL,
recorder, checker tests (30); `robot_backend` 350 passed after the prune
(424 before: the difference is the deleted tests; the 40 failures and 5
collection errors before the prune were all in deleted tests of v1 code
never copied); both templates start and expose exactly `execute_task`; the
WatchDog renders its plan-blind prompt and task; `get_task` in plan-free mode
returns no steps.

Episodes run before the prune and the JSONL change: `records/m7-first-s0-r3`
(template, all 12 steps executed, 79 s sim; checker: only inverted_u_yellow
placed, base1 moved 6.5 mm; the straight-line inserts of line_green and
inverted_u_yellow disturbed the seated parts).

## Before the next tests: placements and pictures

1. With the cell up and Isaac still publishing the placement frames:

   ```bash
   python3 tools/measure_placements.py fmb_assembly_2 --dry-run   # look at it
   python3 tools/measure_placements.py fmb_assembly_2
   ```

   Writes `config/placements/fmb_assembly_2.yaml` (fixture-relative; WatchDog side).
2. Put the pictures of the finished assembly in `config/assembly_views/fmb_assembly_2/` (see its README).
3. For plan-free runs, stop publishing the placement frames from Isaac. The runner refuses a plan-free episode if any is still visible to the executor.

## What you run to check Milestone 7 (sim)

Cell up (Isaac playing with the command server, `sim_moveit_bringup`), from
the repo root.

Steps 1 and 2 run the two *templates* directly, with no agent: they check the
harness (runner, records, budgets) with a known planner. Step 3 is the actual
experiment: an agent writes its own planner and the harness evaluates it.

1. **Harness check with the plan-given template** (no agent, no tokens). The template executes your task file's steps as written:

   ```bash
   runner/episode_runner/run.sh --planner executor/planner_templates/plan_given --episode-id m7-given-s0
   ```

   Expect `records/m7-given-s0/` with `manifest.json`, `verdict.json`,
   `runner.jsonl`, `executor/planner.jsonl`, `bag/`, and no `.log` files.
   `runner.jsonl` shows `start_permission: unguarded_by_config` with ~0 s waited.

2. **Harness check with the plan-free skeleton** (no agent, no tokens). The skeleton has no assembly logic; this only checks that the plan-free task and template run end to end:

   ```bash
   runner/episode_runner/run.sh --planner executor/planner_templates/plan_free \
     --task config/tasks/fmb_assembly_2.plan_free.yaml --episode-id m7-free-s0
   ```

   Expect the arm to go to the safe pose and the planner to return
   `stopped: plan-free skeleton: no assembly logic written yet`, outcome
   `failure`.

3. **Authoring session: the agent writes the planner** (authoring tokens only). It starts from the plan-free skeleton, decides the sequence, grasps and inserts itself, and every `evaluate_planner` call runs episodes like steps 1-2 with its current planner. `sim plan_given` instead starts it from the step-executing template with the full task file:

   ```bash
   AUTHORING_FEEDBACK=checker runner/run_authoring.sh sim plan_free
   ```

   First ask it to read `config/tasks/fmb_assembly_2.yaml` and
   `executor/planner_templates/plan_given/planner.py`: both must be denied.
   Then let it work. `records/authoring/<session>/evals.jsonl` gets one line
   per episode.

4. **WatchDog episode** (WatchDog tokens): step 1 with `--watchdog plan_blind`.
   Expect `runner.jsonl` to show `start_permission` with `released` after the
   WatchDog's preparation, and the planner starting only then. With
   `watchdog.consult: true` and `mode: halt_enabled` in `config/cell.yaml`, consultations
   appear in `executor/planner.jsonl` and in `verdict.json` under `watchdog.during_episode`.

### Open

- cuRobo: `cell_planning_curobo` is written but not installed
  (`planner.curobo.python` is `TODO(mohammad)`; separate venv per its
  `INSTALL.md`). Every episode so far planned with MoveIt.
- Plan-free sessions can read other plan-free sessions' planners (a deny
  would also cover their own); clear `executor/workspace/plan_free/` between
  sessions that must be independent.

## Milestone 6 record

### Milestone 6 live checks (2026-11-17)

| Step | Status |
|---|---|
| 1 command server (GUI) | PASS. Use `nc -N` (Ubuntu's nc quits before the reply otherwise). |
| 2 capture nominal layout | PASS. `verify_tolerances` 5 mm / 2 deg. |
| 3 reset + verify | PASS (all 13 prims within 0.05 mm; arm home, gripper open). |
| 4 contacts | PASS via `tools/pick_check.py`: all four pick steps of `config/tasks/fmb_assembly_2.yaml` grasp, lift 200-300 mm, slip < 1 mm, found/lost on both pads, nothing else touched. Empty close shows nothing because `enabledSelfCollisions` is off. |
| 5 headless | PASS with `isaac/headless/run_headless.sh` (wrapper sources `~/isaacsim/setup_ros_env.sh`); `/clock` and `/joint_states` ~38 Hz; `pick_line_green` passed on headless. |
| 6 recording | PASS (headless): `python3 -m runner.recorder.episode_recording records/manual_test --seconds 10` gave a 9.4 s MCAP (1008 msgs: joint_states, tf, tf_static, camera1/2 poses) and 74 JPEG frames with frames.jsonl (sim-time stamps). Frames ~8 Hz effective against the 10 Hz setting (source /recorder_rgb ~34 Hz; ~0.8 s recorder start-up). Topics without a publisher (watchdog, pad_contacts without the relay) are simply absent from the bag. |
| 7 camera1 | PASS (GUI): `/World/ur5e/wrist_3_link/camera1` + `/Graph/ROS_Camera1`, saved in the stage (timeline stopped). 640x400, H94 x V68 deg (fx 298.4), ~25 Hz rgb + depth + camera_info on `/camera1/...`, frame `camera1_optical_frame`. Mount (sim choice, not calibrated): optical frame at (0, -0.07, 0.03) in wrist_3_link, tilted 18 deg toward the tool axis. The static TF wrist_3_link -> camera1_optical_frame comes from `sim_moveit_bringup/config/cameras.yaml` (relaunch the cell). New server commands `setup_cameras`, `timeline(action)`, `save_stage` (refuses while playing). |

Fixed on 2026-11-17 (uncommitted): contacts folded per pad/body pair with the contact point in the pad surface frame (`isaac/cell_command_server/contacts.py`, `PadContact.msg` extended, relay updated, `scene.contact_pads` now `{body, surface_frame}`); one contact subscription only; impulse computed; `start_in_script_editor.py` restarts and reloads the server; `kit_api.open_stage` accepts the bool that 5.1 returns (broke headless and `reload_stage`); `run_headless.py` prints a traceback on failure; `tools/pick_check.py` (new).

Also 2026-11-17: on-pad test is now a box, `scene.pad_face` (pad face measured from the inner-finger collision mesh: plane y = 0 of the contact frames, 20.2 x 29.8 mm; all eight pick_check contacts inside, u_orange included); `grasp_width_tolerance_m` 0.02 -> 0.005 for the FMB parts in `model_registry.yaml` (regenerated `config/parts/*`, robot_backend rebuilt); stale intruder removed from the saved stage offline (backup kept outside the repo; git has the old file too); `python3 -m runner.recorder.episode_recording` CLI.

Open from M6:
- `line_green`: plateau 25.8 mm against `grasp_width_m: 0.03` leaves 0.8 mm inside the new 5 mm band (the part is ~26 mm wide). Consider `grasp_width_m: 0.026` in `config/tasks/fmb_assembly_2.yaml`. Re-run `tools/pick_check.py` after the tolerance change.
- `pick_inverted_u_yellow` pre-grasp: MoveIt failed 1 of 3 plans although IK is collision-free; pick_check retries 3 times, the executor may hit it too.
- camera1: Gemini 335L FOV/resolution are from memory (TODO(mohammad)); replace the mount with the hand-eye calibration when it exists (both camera files, keep in step). The RG2 housing fills the lower ~40% of the image; move the camera further out or reduce the tilt if that is too much.

### What Milestone 6 added

| Path | What |
|---|---|
| `isaac/common/kit_api.py` | The only Kit-version-specific module: stage, timeline, poses, teleport, articulation joints, episode-joint cleanup, contact reports, camera prims and ROS 2 camera graphs. Uses `isaacsim.core.experimental.prims` (present on 5.1, primary on 6.x) with the classic API as fallback, so the 6.1/6.2 move touches this file only. |
| `isaac/cell_command_server/` | `server.py`: loopback TCP (port `scene.port`, 9010), newline JSON, every command on Kit's main thread. `health`, `report_poses`, `load_layout(assembly, layout, seed)`, `reset`, `verify_reset`, `reload_stage`, `contacts_enable`/`contacts_poll`, `capture_layout`. `layouts.py`: seeded placements (deterministic per seed, jitter ranges per part, fixtures never moved). `start_in_script_editor.py` for the GUI workflow. |
| `isaac/headless/run_headless.py` | `isaac/headless/run_headless.sh`: headless app, ROS 2 bridge on, stage open, server up, Play. `--gui` for debugging. |
| `isaac/cameras/setup_cameras.py` | Verifies camera2 (`/World/Camera`, `/Graph/ROS_Camera`) against `config/cameras/sim.yaml`; creates camera1 under the wrist with a ROS 2 graph of the same shape once its pose and size are filled and `enabled: true`. |
| `config/layouts/fmb_assembly_2/nominal.yaml` | Layout schema with prim paths from the stage, home pose from the SRDF, gripper open from the controller; **positions null** until you capture them. |
| `config/platforms/sim.yaml: scene` | Port, robot and TCP prims, intruder root, finger pads, home pose. `pad_contacts` signal is now `available`. |
| `cell_planner_msgs/PadContact.msg`, `cell_backend_sim pad_contacts_relay` | Kit cannot publish for the system ROS, so the relay polls the server and publishes `/cell/pad_contacts`. |
| `runner/recorder/episode_recording.py` | Per-episode MCAP bag (`-s mcap`, topics from `cell.yaml recording.bag` keys into the platform config, pose topics included) plus the frame recorder into `records/<episode_id>/frames/`, `recording.json` manifest, optional ffmpeg video off by default. Works on hardware (no sim time assumed). |

Checked here: the Kit modules compile; 20 pure tests (layouts, recording commands, platform config); the relay refuses cleanly when no server is up; `cell_planner_msgs` and `cell_backend_sim` rebuild.

A stage fact worth knowing: `assets/usd/assembly_room.usd` still contains a stale injected prim `/World/WatchdogIntruders/foreign_a41b84` from a paper run. `reset` removes it each episode; you may want to delete it from the saved stage.

### `TODO(mohammad)` introduced in M6

- `config/layouts/fmb_assembly_2/nominal.yaml`: positions (capture), `verify_tolerances`, jitter.
- `config/cameras/sim.yaml: camera1` pose, size, `enabled`; `parent_prim` if not the wrist link.
- Headless ROS 2 bridge sourcing (see `isaac/headless/run_headless.py` docstring).
- `budgets.*` and `health.*` in `cell.yaml` remain open for M7/M8.
