# cell_command_server (inside Kit)

Loopback TCP, newline JSON, one request per connection. Every command runs on
Kit's main thread. Config: `config/platforms/sim.yaml: scene`.

```bash
# from the ROS side, any Python:
printf '{"command":"health"}\n' | nc 127.0.0.1 9010
printf '{"command":"load_layout","params":{"assembly":"fmb_assembly_2","layout":"nominal","seed":3}}\n' | nc 127.0.0.1 9010
```

| Command | Params | Does |
|---|---|---|
| `health` | | playing, sim time advancing, stage path, Kit version, intruders present, episode joints present |
| `report_poses` | | world pose of every prim the stage's TF publisher targets, plus the TCP |
| `load_layout` | `assembly`, `layout`, `seed` | teleport parts to the seeded layout (`layouts.py`), then `reset` |
| `reset` | `keep_parts` | remove `/World/WatchdogIntruders/*`, remove joints marked `cell:episode_joint`, re-place parts from the last layout, zero velocities, home the arm (SRDF home pose from the platform config), open the gripper |
| `verify_reset` | `tolerances` | compare parts with the loaded layout within `verify_tolerances` |
| `reload_stage` | | reopen the stage file and play |
| `contacts_enable` | `enable` | PhysX contact reports on `scene.contact_pads`; `contacts_poll(since_seq)` returns them to the ROS-side relay (`cell_backend_sim pad_contacts_relay`) |
| `capture_layout` | `assembly`, `name` | write the current part poses as `config/layouts/<assembly>/<name>.yaml` |

Start it in the GUI with `start_in_script_editor.py`, or let `isaac/headless/run_headless.py` start it.

Known stage facts (read from `assets/usd/assembly_room.usd` on 2026-11-11): the
articulation root is `/World/ur5e/root_joint`, the TCP is `/World/ur5e/tcp`,
parts are `/World/<id>`, placement frames are children of `/World/base1` and
`/World/base2`, the finger pads are `/World/ur5e/Rg2_mn/onrobot_rg2/{left,right}_inner_finger`,
and the saved stage still contains a stale injected prim under
`/World/WatchdogIntruders`, which `reset` removes.

Kit-version-specific calls live only in `isaac/common/kit_api.py` (5.1 today,
6.x path through the experimental prim API).
