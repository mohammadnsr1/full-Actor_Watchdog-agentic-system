# Next steps

Updated 2026-11-25. Current state and what to do next; kept current as we go.
How it fits together: `docs/ARCHITECTURE.md`. How we got here: `docs/PROGRESS.md`.
The executor-baseline plan and decisions: `docs/EXECUTOR_BASELINE.md`.

## Where we are

- Sim cell, fmb1 stage: the plan-given template places 4/4 (`fmb1-given-s1`,
  seeds 0 and 2). The nominal layout has no jitter, so seeds do not vary the start yet.
- EB1 done. EB2: stage, seats and checker done; task records next.
- Not committed since e4071bc: the checker and WatchDog acceptance modulo
  symmetry and the interchangeable U pair (`runner/episode_runner/checker.py`,
  `robot_backend/watchdog/requirements.py`, `cell_platform_interfaces/placements.py`,
  `config/requirements/fmb_assembly_2.yaml`, `config/placements/fmb_assembly_2.yaml`
  `symmetries`, `tools/build_fmb1_stage.py`), and this docs reorganisation
  (`PROGRESS.md` new; `BUILD_PLAN.md`, `INTEGRATION_SURVEY.md`, `PLATFORMS.md`
  removed, their live parts moved into `ARCHITECTURE.md`, `PROGRESS.md` and the
  hardware list below). `cell_platform_interfaces` and `robot_backend` were
  rebuilt (`colcon build --packages-select ...`) so the runner and WatchDog use them.

## Next

1. **EB2, task records** (in progress): `tasks/fmb_assembly_2/task.yaml`, an
   environment declaration and a robot composite in the labmate's schemas,
   generated from `config/platforms/sim.yaml` so names stay in one place; the
   agent reads them under `/data` (EB4/EB5).
2. **EB3, lab verbs**, then EB6 platform MCP, EB4 container, EB5 prompts, EB7
   WatchDog in the lab, EB8 protocol (`docs/EXECUTOR_BASELINE.md` section 4).
3. **M8**: supervisor, operations doc, records index; budgets and health thresholds.

## Run on the cell

From the repo root, Isaac playing with the command server
(`isaac/cell_command_server/start_in_script_editor.py`) and `sim_moveit_bringup` up.

```bash
# the stage changed on disk -> reopen it (never save the open copy over it)
printf '{"command":"reload_stage"}\n' | nc -N 127.0.0.1 9010
printf '{"command":"health"}\n' | nc -N 127.0.0.1 9010
# layout + verify (the runner does this too)
printf '{"command":"load_layout","params":{"assembly":"fmb_assembly_2","layout":"nominal","seed":0}}\n' | nc -N 127.0.0.1 9010
printf '{"command":"verify_reset","params":{}}\n' | nc -N 127.0.0.1 9010
# one episode with the plan-given template (no agent)
runner/episode_runner/run.sh --planner executor/planner_templates/plan_given --episode-id <id> [--seed N]
# plan-free skeleton (expect: safe pose, then stopped, outcome failure)
runner/episode_runner/run.sh --planner executor/planner_templates/plan_free \
  --task config/tasks/fmb_assembly_2.plan_free.yaml --episode-id <id>
# an authoring session (the agent writes the planner)
AUTHORING_FEEDBACK=checker runner/run_authoring.sh sim plan_free
# with the WatchDog: add --watchdog plan_blind to an episode
```

- Restart `cell_physical_events` after `config/placements/` changes (it reads the
  file at start); the runner and the WatchDog read it per episode.
- Do not save the stage from Isaac after Play: each save writes that session's
  render products into the file. `tools/clean_stage_render.py --check-only` tells.
- Stage and placements can be rebuilt any time:
  `build_fmb1_stage.py` (full, from cd4d6c7), `--placements`, `--cell-physics`,
  `--check-only` (Isaac's USD; see the tool's docstring).
- Next episode check for the uncommitted checker: `verdict.json` `checker.parts.*`
  now carries `symmetry` (0 = as placed) and `swapped_from` when the U pair swaps.

## Open items

Decisions waiting on Mohammad:
- Layout jitter ranges per part (`config/layouts/fmb_assembly_2/nominal.yaml`;
  base1 stays 0), needed for the 9-seed panel.
- `place_u_brown` / `place_u_orange` keep a -0.5 mm `x` target offset tuned on the
  old frames (aims 0.5 mm off the exact seat).

Loose ends:
- The table has no physics material (labmate's 0.8/0.6); part-on-table friction ~0.65.
- The labmate's resolver composes STEP-frame features with centroid-frame placements
  (16.7 mm for Body1); check before the prompts point the agent at `frame()`/`locate()`.
- `run_python` (EB3) runs inside Kit on the host and can read host files
  (`docs/EXECUTOR_BASELINE.md` section 5).
- `cell_physical_events` emits approach/engage only against a part's own frame, so
  a swapped U part gets no `part_engaged_feature` (acceptance itself handles the swap).
- `line_green` `grasp_width_m: 0.03` against a 25.8 mm plateau (consider 0.026);
  MoveIt occasionally fails the `pick_inverted_u_yellow` pre-grasp plan (1 of 3).
- cuRobo: `cell_planning_curobo` written, not installed (`planner.curobo.python`);
  every episode so far planned with MoveIt.
- Plan-free sessions can read other plan-free sessions' planners; clear
  `executor/workspace/plan_free/` between sessions that must be independent.
- camera1: mount is a sim choice, FOV/resolution from memory; replace with the
  hand-eye calibration when it exists (`config/cameras/`).

## Hardware (`TODO(mohammad)`)

Answered 2026-11-10: UR7e; RG6 via URCap and URScript, no ROS 2 readings yet;
two Orbbec Gemini 335Lg cameras; WatchDog + camera2 pipeline in the Humble
container on the workstation, the rest on the accelerator; planner from the UR
AI Accelerator SDK, cuRobo, MoveIt as backup; per-platform effort limits.
Still open:
- camera1 hand-eye calibration (on the accelerator) and its frame;
- UR driver branch and trajectory controller name (`config/platforms/hardware.yaml`);
- RG6 readings over ROS (width, force, grip detected) once a path exists;
- accelerator Python version and the SDK planner package;
- `ROS_DOMAIN_ID` and RMW for `docker/humble/compose.yaml`;
- hardware effort limits (`limits.hardware` in the registry);
- FoundationPose adapters to `vision_msgs/Detection3DArray` on both cameras;
- the M7 hardware side (runner placement, hardware episode).
