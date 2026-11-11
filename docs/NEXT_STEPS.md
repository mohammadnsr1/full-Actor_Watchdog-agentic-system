# Next steps

Updated 2026-11-11 at the end of Milestone 6 (v2): cell command server, headless Isaac, sim cameras, recording.

## State

- M1 to M4: done and checked. M5: built; the plan-blind session ran on your side once the config bug was fixed; the physical-event thresholds are yours to set (you said later).
- M6 (this milestone): written and offline-tested. **The Kit-side code has not run inside Isaac yet** (I cannot run Isaac from here); its first run is your check below. Not committed.

## What Milestone 6 added

| Path | What |
|---|---|
| `isaac/common/kit_api.py` | The only Kit-version-specific module: stage, timeline, poses, teleport, articulation joints, episode-joint cleanup, contact reports, camera prims and ROS 2 camera graphs. Uses `isaacsim.core.experimental.prims` (present on 5.1, primary on 6.x) with the classic API as fallback, so the 6.1/6.2 move touches this file only. |
| `isaac/cell_command_server/` | `server.py`: loopback TCP (port `scene.port`, 9010), newline JSON, every command on Kit's main thread. `health`, `report_poses`, `load_layout(assembly, layout, seed)`, `reset`, `verify_reset`, `reload_stage`, `contacts_enable`/`contacts_poll`, `capture_layout`. `layouts.py`: seeded placements (deterministic per seed, jitter ranges per part, fixtures never moved). `start_in_script_editor.py` for the GUI workflow. |
| `isaac/headless/run_headless.py` | `~/isaacsim/python.sh isaac/headless/run_headless.py`: headless app, ROS 2 bridge on, stage open, server up, Play. `--gui` for debugging. |
| `isaac/cameras/setup_cameras.py` | Verifies camera2 (`/World/Camera`, `/Graph/ROS_Camera`) against `config/cameras/sim.yaml`; creates camera1 under the wrist with a ROS 2 graph of the same shape once its pose and size are filled and `enabled: true`. |
| `config/layouts/fmb_assembly_2/nominal.yaml` | Layout schema with prim paths from the stage, home pose from the SRDF, gripper open from the controller; **positions null** until you capture them. |
| `config/platforms/sim.yaml: scene` | Port, robot and TCP prims, intruder root, finger pads, home pose. `pad_contacts` signal is now `available`. |
| `cell_planner_msgs/PadContact.msg`, `cell_backend_sim pad_contacts_relay` | Kit cannot publish for the system ROS, so the relay polls the server and publishes `/cell/pad_contacts`. |
| `runner/recorder/episode_recording.py` | Per-episode MCAP bag (`-s mcap`, topics from `cell.yaml recording.bag` keys into the platform config, pose topics included) plus the frame recorder into `records/<episode_id>/frames/`, `recording.json` manifest, optional ffmpeg video off by default. Works on hardware (no sim time assumed). |

Checked here: the Kit modules compile; 20 pure tests (layouts, recording commands, platform config); the relay refuses cleanly when no server is up; `cell_planner_msgs` and `cell_backend_sim` rebuild.

A stage fact worth knowing: `assets/usd/assembly_room.usd` still contains a stale injected prim `/World/WatchdogIntruders/foreign_a41b84` from a paper run. `reset` removes it each episode; you may want to delete it from the saved stage.

## What you run to check Milestone 6

1. **Command server in the GUI first** (lowest risk). Isaac open, stage playing. Paste `isaac/cell_command_server/start_in_script_editor.py` into the Script Editor and run it. Expect `cell command server: 127.0.0.1 9010 commands: [...]`. Then from a terminal:

   ```bash
   printf '{"command":"health"}\n' | nc 127.0.0.1 9010
   printf '{"command":"report_poses"}\n' | nc 127.0.0.1 9010 | python3 -m json.tool | head -40
   ```

   Expect `"ok": true`, `playing: true`, `sim_advancing: true`, 23 TF prims, and the stale intruder listed. Paste both outputs.

2. **Capture the nominal layout**, then look at the file:

   ```bash
   printf '{"command":"capture_layout","params":{"assembly":"fmb_assembly_2","name":"nominal"}}\n' | nc 127.0.0.1 9010
   ```

   It overwrites `config/layouts/fmb_assembly_2/nominal.yaml` with the measured poses. Fill `verify_tolerances` (and jitter ranges if you want variation).

3. **Reset and verify.** Move a part by hand in the viewport, then:

   ```bash
   printf '{"command":"load_layout","params":{"assembly":"fmb_assembly_2","layout":"nominal","seed":0}}\n' | nc 127.0.0.1 9010
   printf '{"command":"verify_reset"}\n' | nc 127.0.0.1 9010
   ```

   Expect the part back in place, the arm at the SRDF home pose, the gripper open, `removed_intruders` listing the stale prim once, and `verify_reset` passing within your tolerances. This is the first time the experimental prim API is exercised on this stage: if `teleport` or the articulation calls raise, paste the `traceback` field of the response. Then `seed: 3` to see jitter if you set ranges.

4. **Contacts.** With the server up:

   ```bash
   ros2 run cell_backend_sim pad_contacts_relay &
   ros2 topic echo /cell/pad_contacts
   ```

   Close the gripper on a part (the check script or the RG2 service). Expect `found`/`lost` events naming the pad and the part prim.

5. **Headless.** Close the GUI Isaac, then:

   ```bash
   ~/isaacsim/python.sh isaac/headless/run_headless.py
   ```

   Expect the `[headless] stage ... playing=True server 127.0.0.1:9010` line, then bring the cell up as usual and confirm `/joint_states` and `/clock` flow. Paste any error: the ROS 2 bridge in headless mode is the step most likely to need a tweak (sourcing).

6. **Recording.** With the cell up:

   ```bash
   python3 -c "
   import yaml, time, sys; sys.path.insert(0, '.')
   from cell_platform_interfaces.platform_config import load_platform_config
   from runner.recorder.episode_recording import EpisodeRecording
   rec = EpisodeRecording('records/manual_test', load_platform_config(), yaml.safe_load(open('config/cell.yaml'))).start()
   time.sleep(10); print(rec.stop())"
   ls records/manual_test records/manual_test/frames/recorder | head
   ```

   Expect `bag/` with an `.mcap`, about 100 JPEG frames with `frames.jsonl`, and `recording.json`.

7. **camera1** only when its pose and size are in `config/cameras/sim.yaml`: run `isaac/cameras/setup_cameras.py` in the Script Editor and save the stage.

## `TODO(mohammad)` introduced in M6

- `config/layouts/fmb_assembly_2/nominal.yaml`: positions (capture), `verify_tolerances`, jitter.
- `config/cameras/sim.yaml: camera1` pose, size, `enabled`; `parent_prim` if not the wrist link.
- Headless ROS 2 bridge sourcing (see `isaac/headless/run_headless.py` docstring).
- `budgets.*` and `health.*` in `cell.yaml` remain open for M7/M8.

## What Milestone 7 will do once approved

- `docs/PLANNER_CONTRACT.md`; `executor/planner_template/` (stdio MCP server with the single `execute_task` tool, ROS worker over the platform backends and the planning service, fact journal on `/cell/executor_facts`, notices subscription when `consult` is on, Python 3.10); `executor/authoring_mcp/` (`get_task`, `evaluate_planner`, `read_verdict`, `submit_work`); restricted settings and MCP configs per platform; `runner/episode_runner/` (reset, verify, WatchDog session, recording, `execute_task` under budget and wall guard, episode-end notice, acceptance wait, ground-truth checker on sim, `manifest.json`, `verdict.json`, cleanup).
