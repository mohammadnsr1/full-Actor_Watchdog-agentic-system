# Next steps

Updated 2026-11-25. Current state and what to do next; kept current as we go.
How it fits together: `docs/ARCHITECTURE.md`. How we got here: `docs/PROGRESS.md`.
The executor-baseline plan and decisions: `docs/EXECUTOR_BASELINE.md`.

## Where we are

- Sim cell, fmb1 stage: the plan-given template places 4/4 (`fmb1-given-s1`,
  seeds 0 and 2). The nominal layout has no jitter, so seeds do not vary the start yet.
- Built offline 2026-11-25, **waiting for the live check below**: EB2 (task records,
  checker modulo symmetry and the interchangeable U pair), EB3 (lab verbs in Kit and on
  the host, lab events), EB4 (authoring container and session launcher), EB5 (baseline
  prompt layers and docs), EB6 (platform MCP: delivery check, budgeted evaluation,
  submission). Nothing since caf08cc is committed.
- Rebuilt here (`colcon build --packages-select ...`): `cell_platform_interfaces`,
  `robot_backend`, `cell_backend_sim`, `sim_moveit_bringup`.

## Live check: EB3, EB4, EB6 (2026-11-25)

From the repo root. Expected results after each step.

**A. Restart the cell with the new pieces**

1. Isaac down: register the lab verbs with the Kit bridge (once):
   `python3 ~/omni-kit-mcp/scripts/install.py add-project --path $PWD/isaac --module lab_tools`
2. Start Isaac, open the stage, Play, run `isaac/cell_command_server/start_in_script_editor.py`.
   `printf '{"command":"list_commands"}\n' | nc -N 127.0.0.1 9010` lists twelve `lab_*` commands.
   `python3 -m kit_mcp.client --list-bridges` shows the bridge on 9009;
   `python3 -m kit_mcp.client list_tools` (from `~/omni-kit-mcp`) includes `lab.resolve` ... `lab.events`.
3. Relaunch `sim_moveit_bringup` (it now also starts `lab_events_relay`).
   `ros2 topic echo /cell/lab_events` stays quiet until step B3.

**B. Lab verbs over TCP** (`nc -N 127.0.0.1 9010`, one JSON line each)

1. `{"command":"lab_resolve","params":{"ref":"u_brown_placement","relative_to":"base1"}}`:
   position about (0, 0.0476, 0.021).
2. `{"command":"lab_checkpoint","params":{"name":"start"}}`: five objects, joints > 6.
3. `{"command":"lab_set","params":{"objects":{"u_brown":{"position":[0.3,-0.3,0.15]}}}}`:
   u_brown appears 15 cm up and falls; `/cell/lab_events` prints a `set` event.
4. `{"command":"lab_rest","params":{}}` right after: `at_rest: false`; a few seconds
   later: `true`.
5. `{"command":"lab_contacts_arm","params":{"bodies":["u_brown"]}}`, step 3 again, then
   `{"command":"lab_contacts_read","params":{}}`: `contact_made` with the table body.
6. `{"command":"lab_restore","params":{"checkpoint":"start"}}`: back where it was; a
   `restore` event.
7. `{"command":"lab_verify_assembly","params":{}}`: `pass: false`, 0 of 4 placed.
8. `{"command":"lab_camera_view","params":{"name":"top","eye":[0.4,-0.2,0.8],"target":[0.4,-0.2,0.0]}}`: ok.
9. Bridge-only verbs (from `~/omni-kit-mcp`): `python3 -m kit_mcp.client lab.camera_capture '{"camera":"top"}'`
   writes `records/lab/captures/*.png` (look at it: base1 from above);
   `python3 -m kit_mcp.client lab.until_at_rest '{"budget_s":5}'`: `at_rest: true`.
10. Start an episode (any template) and during it send step 3: refused, "an episode is running".
11. Stage hygiene: after Stop, `tools/clean_stage_render.py --check-only` still says 0;
    the lab camera is not in the stage file (session layer).

**C. Platform MCP from the shell** (ROS + `ros2_ws/.venv` sourced, cell up)

1. `python tools/platform_call.py check_delivery`: `ok: true`, `tools: ["execute_task"]`.
2. `python tools/platform_call.py robot_health`: `ok: true`, arm action, planning
   service and simulator all ready.
3. `python tools/platform_call.py move_robot '{"tool_delta": {"position": {"z": -0.03}}}'`:
   the TCP moves 3 cm along its own +Z (the approach axis); a `move_robot` lab event. Then `'{"joints": {"shoulder_pan_joint": 0.2}}'`: a slow pan.
4. `python tools/platform_call.py evaluate_planner '{"episodes": 1}'`: one episode with the
   baseline scaffold; outcome `failure` (it stops at once, `inadmissible_state`), budget
   1 of 18 charged; `executor/workspace/baseline/_check/app/evaluations/0001/episode-1/`
   holds `verdict.txt`, `planner.jsonl`, `runner.jsonl`, `bag/`, `frames/`.

**D. The container**

1. `docker build -f docker/authoring/Dockerfile --build-context kitmcp=$HOME/omni-kit-mcp -t cell-authoring .`
   (needs docker access: `sudo usermod -aG docker $USER`, or sudo).
2. `SHELL_ONLY=1 runner/run_baseline.sh baseline`, then inside:
   `ls /data /app/artifact`; `ros2 topic list` shows the cell's topics;
   `python3 -c "from robot_backend.utils import rg2_geometry as g; print(g.load_rg2_grasp_offset_lookup() is not None)"` prints True;
   `cd /app/artifact && python3 -c "import planner; print(planner.run_task('fmb_assembly_2'))"` prints the scaffold's stop;
   `touch /app/evaluations/x` fails (read only); `claude --version` prints 2.1.331.
3. Optional, costs tokens: `ANTHROPIC_API_KEY=... WALL_S=900 runner/run_baseline.sh baseline`
   for a 15-minute session; `records/authoring/<session>/transcript.jsonl` should show it
   reading `/data`, calling `lab__*` and `platform` tools.

Send the outputs that differ from these expectations.

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
- The referee's bands (`watchdog_parts.yaml` tolerances, also told to the agent in
  `task.yaml`): 10-20 mm and 10 deg, against fmb1's own 1 mm / 1 deg, and different for
  the two identical U parts (u_brown 10 mm, u_orange 20 mm). A part stuck halfway into its
  slot can be within 20 mm of its seat.
- camera1's pose channel carries the `<part>_placement` frames in the baseline condition
  too (the labmate's channel has object frames only; the agent has the seats in the graph
  either way). Keep, or stop publishing them for baseline sessions.
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
