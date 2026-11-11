# Independence

Written 2026-11-11 (Milestone 5). For every channel that used to carry
executor information or plan knowledge into the WatchDog, the file and line
that now cuts it off. Line numbers are from the files as of this milestone;
re-check with the greps at the end after later edits.

## 1. Channels into the WatchDog process and where each is cut

| # | Channel (v1 BUILD_PLAN section 4) | Cut at | How |
|---|---|---|---|
| 1 | Task file `steps:` block (`_load_task_steps`, `_load_task_objects`) | `robot_backend/mcp_servers/watchdog_mcp_server.py` `_load_requirements` (the only task loader left) and `robot_backend/watchdog/requirements.py:load_requirements` | `WATCHDOG_TASK_PATH` must name a requirements file; a file containing `steps` or `step_plan` is refused with an error. `get_contract` reads `acceptance.parts[object_id]` only. |
| 2 | `{{STEPS}}` / `{{STEP_PLAN}}` in the task template | `agents/prompts/watchdog_task_plan_blind.txt` | The plan-blind template has only `INSTRUCTION_SPEC`, `OBJECT_IDS`, `RUN_NOTES`. The S3 template is kept for the Milestone 2 liveness check only and is not wired to the plan-blind config. |
| 3 | `/watchdog/events` subscription delivering executor-origin envelopes | `robot_backend/watchdog/recorder.py` `_on_event_msg`: `if envelope.event_type in _EXECUTOR_EVENT_TYPES: return` and the role check for `look`/`observation` | Every `begin_step`, `end_step`, `run_ended`, `skill_start`, `skill_end`, `phase` envelope is dropped before it is recorded, waited on, or shown to the agent, whoever publishes it. Executor-role looks and observations are dropped the same way. |
| 4 | Engine coupling to skill events (held part, release auto-disarm) | `robot_backend/watchdog/engine.py` `on_physical_event` (replaces `on_skill_event`, which no longer exists) | The held span opens on `part_lifted` and closes on `part_released` from `/cell/physical_events`; the release auto-disarm of `relative_frame` monitors is unchanged in behaviour (`_release_disarm_slip_monitors`) and only its trigger moved. |
| 5 | `signal_ready(scope="step"|"phase")` | `watchdog_mcp_server.py` `signal_ready`: only `scope="run"` is accepted | The orchestrator keeps its scopes for the planner side; nothing on the WatchDog side can name a step or phase. |
| 6 | Run termination from `run_ended` and the two-silence finalize | `watchdog_mcp_server.py` `_timeout_or_finalize`, `_resolve_observed_terminal_status`, `_observe_drained_events`; `recorder.py` `_on_notice_msg` | The episode ends only on the episode runner's `episode_end` notice on the platform's `topics.watchdog_notices`. Before it, a timeout is a timeout. After it, two silent waits make the server record `incomplete`. `executor_terminal_status` now records `episode_end` or `no_episode_end`, never the executor's status. |
| 7 | Shared session directory | unchanged in M5; see M7 | The episode runner will give WatchDog `records/<episode_id>/watchdog/` and the planner `records/<episode_id>/executor/`. The WatchDog settings keep `Read`, `Bash`, `Glob`, `Grep` denied (`config/settings/watchdog.settings.json`). |
| 8 | Executor `look` events | `recorder.py` `_on_event_msg` role check (see 3) | Dropped at ingest. |
| 9 | Gate records with `step_id` | `cell_orchestrator` untouched; `signal_ready` (5) | WatchDog never names a step; the orchestrator log is the planner's business. |
| 10 | Prompt text describing the executor's tools and flow | `agents/prompts/watchdog_system_plan_blind.txt` | Built from the S3 prompt: Authority rewritten for the two halt modes and rulings; evidence discipline kept with the agent-chosen-threshold rule; the tools section rewritten for the plan-blind tool set; "Reading the event stream", "What is expected to move", "Steps on and off the plan" and the control loop removed. The prompt says what each tool does and what must be watched, never when to call a tool. |
| 11 | camera1 (wrist) poses, the executor's pose pipeline | `watchdog_mcp_server.py` `_ensure_watchdog` (`cue_poses` is a separate `TopicPoseSource` on `poses.executor_topic`), `wrist_pose_cue`, `_is_shared_source`; `recorder.py` `_attach_ros` (the recorder's `pose_source` subscribes to `poses.watchdog_topic` only) | camera1 poses never enter the recorder's buffers, monitors, evidence or snapshots. They are reachable only through `wrist_pose_cue`, whose result is tagged `camera1_pose_cue`; any verdict or ruling citing a camera1 source is flagged `shared_source: true` and counted. |
| 12 | Object TF (simulator truth, shared with the executor's TF path) | `recorder.py` `_on_tf_timer` (object chains removed; objects sampled from `pose_source`), `_observation_as_transform`, `lookup_transform`, `lookup_transform_at` | Object frames are never looked up in TF any more, not by the tick and not by `relative_frame` monitors: an object frame request is answered from the WatchDog pose channel or raises `LookupError`. Only cell frames (the TCP) come from robot TF. On sim in `ground_truth` mode the channel is fed by `cell_backend_sim ground_truth_pose_publisher` and every sample says `ground_truth`. |

## 2. Camera roles

| Camera | Role | Who reads its poses | Who reads its frames |
|---|---|---|---|
| camera1 (wrist) | executor's pose query (`/get_object_pose` on sim today; `/camera1/poses` contract) | planner only | planner; WatchDog through `look(camera="camera1")`, tagged `look:camera1` |
| camera2 (table) | WatchDog's observation channel (`/camera2/poses`) | WatchDog only (recorder `pose_source`, `cell_physical_events`) | WatchDog `look()` default; recorder's image buffer |

Both topics are `vision_msgs/Detection3DArray`. If poses are also put on TF, the prefixes `cam1/<id>` and `cam2/<id>` (platform config `poses.tf_prefix_*`) keep the two estimates in different frames. On sim today both topics carry the same simulator truth (the publisher does not model visibility), which is recorded as `pose_source: ground_truth` on every verdict; independence of the two channels is physical only on hardware.

## 3. The shared-source flag and missing evidence

- `record_verdict` and `record_run_verdict` require `evidence_sources` from a closed vocabulary (`_EVIDENCE_SOURCES` in the server). Any source starting with `camera1`, `executor` or `wrist`, or `look:camera1`, sets `shared_source: true` on the event.
- `issue_ruling` carries the same tagging.
- `inspect_acceptance` records a part whose pose or placement frame the WatchDog camera cannot see as `missing_evidence`, never as pass or fail.
- The run verdict carries `shared_source_count` and `missing_evidence_count` for the episode (server-populated from the calls it saw), plus the recorded acceptance inspection.

## 4. What the WatchDog still receives that is worth knowing

- The requirements file (`config/requirements/<assembly>.yaml`): acceptance contracts and invariants in prose, per-part placement frame and grasp width. No order, no steps.
- The WatchDog part data (`config/parts/watchdog_parts.yaml`, generated from the registry by `tools/split_registry.py`): frames, tolerances, per-platform limits, feature frames per part. The registry itself is still loaded by the server for the contract loader and the ACM list; it carries no plan knowledge. Pruning the executor-only registry fields out of that path is Milestone 7 work.
- Physical events from `cell_physical_events`: computed by the cell from its own signals (joint states, robot TF, the WatchDog pose topic). The node never subscribes to anything the executor publishes.

## 5. Remaining risks

- **Sim ground truth is one source.** Until FoundationPose runs on the sim cameras, `/camera1/poses` and `/camera2/poses` are identical copies of Isaac TF; a verdict is independent of the executor's reasoning but not of its perception. The record says so on every sample.
- **Hardware has no object poses yet.** With no camera2 pipeline, every pose-based clause is missing evidence and the acceptance inspection reports `missing_evidence` for every part. That is the honest outcome, not a pass.
- **Registry still loaded.** See section 4.
- **`arm_custom_monitor` code reads the recorder.** An authored check sees the same buffers the server does (object poses from camera2, TCP from TF); it has no path to the executor's data.
- **Session directory** separation lands in Milestone 7.

## 6. Greps to re-check after edits

```bash
grep -rnE 'begin_step|end_step|run_ended|skill_start|skill_end|on_skill_event|_load_task_steps|current_step_id' ros2_ws/src/robot_backend/robot_backend --include='*.py' | grep -v '/test/'
grep -nE 'executor_topic|camera1' ros2_ws/src/robot_backend/robot_backend/watchdog/recorder.py   # must be empty
```
