# Next steps

Updated 2026-11-11 at the end of Milestone 5 (v2): physical events, evidence sources, plan-blind WatchDog.

## State

- M1, M2: done. M3 and M4: built, and checked live by Mohammad on 2026-11-10 (20 Hz poses, plan-only service, bridge cancel through the platform target, full motion through the sim backend).
- M5 (this milestone): built and offline-tested here. Not run against Isaac yet. **Not committed** (yours).
- Container build (M3 step 4) not reported yet; it does not block M5.

## What Milestone 5 added

| Path | What |
|---|---|
| `ros2_ws/src/cell_physical_events/` | Node `physical_events_node` on `/cell/physical_events` (`cell_planner_msgs/PhysicalEvent`: `wall_time_iso`, `sim_time`, `kind`, `scene_ids`, `source`, `confidence`, `value`, `detail`). Eleven kinds; detectors in `detectors.py` are ROS-free and tested. Every threshold lives in `config/cell.yaml: physical_events` and is `TODO(mohammad)`: a zero disables that detector with one warning at start. Started by the bringup (`start_physical_events`). |
| `config/requirements/fmb_assembly_2.yaml` | The WatchDog's only task input: acceptance parts (placement frame, grasp width), fixtures, bystanders, invariants in prose. No steps. The server refuses a file with `steps`. |
| `config/parts/watchdog_parts.yaml`, `executor_parts.yaml` | The registry split by role (`tools/split_registry.py`). WatchDog's copy carries tolerances and `limits.sim` / `limits.hardware` (hardware values `TODO`), and each part's feature frames. |
| WatchDog server | Plan-blind: executor events dropped at ingest; held part from `part_lifted`/`part_released`; object poses from the camera2 pose channel (never object TF), TCP aligned to the observations' stamp; termination on the episode-end notice only; per-part rollup; evidence tagging with `shared_source` and missing-evidence counts; modes `record_only`/`halt_enabled` and `consult` from `cell.yaml`; new tools `arm_wake`, `disarm_wake`, `wrist_pose_cue`, `issue_ruling`, `inspect_acceptance`, `look(camera=)`. Full map in `docs/INDEPENDENCE.md`. |
| Prompt and configs | `agents/prompts/watchdog_system_plan_blind.txt` (from the S3 prompt), `watchdog_task_plan_blind.txt`, `agents/config/watchdog_agent_plan_blind.yaml`, `config/mcp/watchdog.plan_blind.mcp.json`. `runner/run_watchdog.sh plan_blind` (default) launches it; `run_watchdog.sh s3` keeps the old session for liveness checks. |
| Tests | 450 pass across the workspace. Seven WATCHDOG tests that tested the removed coupling were deleted (listed in `docs/PROVENANCE.md`) and replaced by `test_plan_blind_watchdog.py`. |

Checked here: full `colcon build` (17 packages); the plan-blind WatchDog context comes up against a bare graph (mode `record_only`, pose source `ground_truth`, 21 objects, TCP frame), a halt in `record_only` is recorded as suppressed with a `halt_id`; the physical events node starts and reports every detector disabled until thresholds are set; the requirements file loads and renders without a `{{STEPS}}` placeholder; the bringup launch imports with the new node.

## What you run to check Milestone 5

1. Rebuild, then set a first set of physical-event thresholds in `config/cell.yaml` so the node has something to publish. Nothing is invented on my side; pick values you can defend from the cell (joint speeds, effort units you have measured, the RG2 geometry):

   ```yaml
   physical_events:
     motion_speed_threshold_rad_s: <rad/s>    # arm counts as moving above this
     motion_debounce_s: <s>
     lift_threshold_m: <m>                    # rise above rest height = lifted
     hold_distance_m: <m>                     # part within this of the TCP = held
     release_distance_m: <m>                  # beyond this from the TCP = released
     settle_speed_m_s: <m/s>
     settle_window_s: <s>
     approach_distance_m: <m>
     engage_distance_m: <m>
     unknown_object_min_confidence: <0..1>
   ```

   Leave the effort proxy at zero unless you want contact-by-effort; `effort_rise_*` likewise.

2. Bring the cell up as before. Expect the new line `physical events on /cell/physical_events: 10 parts, 11 frames, pose source ground_truth, effort joint finger_joint` and one warning per detector you left at zero. Then, in a second terminal:

   ```bash
   ros2 topic echo /cell/physical_events
   python3 runner/tools/plan_execute_cancel_check.py --goal 0.0 -1.57 1.57 -1.57 -1.57 0.0 --no-cancel
   ```

   Expect `motion_started` then `motion_stopped` with `source: joint_state`. If you move a part by hand in Isaac you should see `part_lifted` (and `part_released`, `part_settled` when you put it down) only if the TCP is near it; otherwise nothing, because held means near the TCP. Paste a few events.

3. Plan-blind WatchDog session, `record_only` (the default in `cell.yaml`):

   ```bash
   ./runner/run_watchdog.sh plan_blind
   ```

   Expect the session to load the plan-blind prompt and the requirements file, call `available_evidence_sources` and `get_contract` for the four parts, arm monitors and wake conditions, and then wait. It must never see a step name. While it waits, drive the arm with the check script (step 2) and watch whether it wakes on `motion_started`. Then end the episode by hand (the runner does this in M7):

   ```bash
   ros2 topic pub --once /cell/watchdog_notices cell_planner_msgs/msg/WatchdogNotice "{wall_time_iso: '$(date -u +%Y-%m-%dT%H:%M:%S.%6NZ)', kind: episode_end, detail: manual}"
   ```

   Expect the session to wake with `wake: episode_end`, call `inspect_acceptance` (every part should report `pass` or `fail` against its placement frame, or `missing_evidence` if a frame is not published), and record one `record_run_verdict` with `parts_covered` and `evidence_sources`. Paste the final verdict event from `events_watchdog.jsonl` in the session directory.

4. Optional: set `watchdog.mode: halt_enabled` in `cell.yaml`, restart the session, and call `halt_execution` by hand through the session while the check script moves the arm. Expect the arm to stop and a `halt_id`; then `issue_ruling` on it. In `record_only` the same call returns `suppressed: true`.

## `TODO(mohammad)` introduced in M5

- `config/cell.yaml: physical_events.*` (all thresholds).
- `config/parts/watchdog_parts.yaml: parts.*.limits.hardware.*` (newtons).
- Hardware placement frames: on hardware nothing publishes the `*_placement` frames yet, so `inspect_acceptance` reports missing evidence for every part until a fixture-relative frame source exists (camera2 FoundationPose of the fixture plus the known offsets, or a static TF).
- camera1 stays disabled in `config/cameras/sim.yaml` until the wrist camera is added (M6); until then `wrist_pose_cue` and `look(camera="camera1")` report it unavailable.

## Known limits to say out loud

- On sim both pose topics carry the same Isaac truth, so WatchDog's verdicts are independent of the executor's reasoning, not of its perception. Every sample and verdict says `ground_truth`.
- The ground-truth publisher does not model visibility; "camera2 cannot see the part" never happens on sim until FoundationPose runs there.
- The registry is still loaded by the WatchDog server for the contract loader and the ACM list (part data, no plan knowledge). Pruning is M7.

## What Milestone 6 will do once approved

- `isaac/cell_command_server/` inside Kit (Isaac 5.1 API, version-specific calls isolated): `load_layout`, `reset`, `verify_reset`, `reload_stage`, `report_poses`, `health`, `contacts_enable` publishing `/cell/pad_contacts`; `config/layouts/fmb_assembly_2/<name>.yaml` seeded placements.
- `isaac/headless/`: open `assembly_room.usd` headless, ROS 2 bridge on, command server loaded, play; the GUI workflow kept.
- `isaac/cameras/`: camera1 on the wrist and camera2 at the table from `config/cameras/sim.yaml` (poses `TODO`), publishing RGB, depth, camera info on the configured topics.
- Recording per episode: frame recorder start/stop into `records/<episode_id>/frames/`, MCAP bag with `/cell/physical_events`, `/cell/watchdog_notices`, `/cell/executor_facts`, both pose topics; works on hardware without sim time.
