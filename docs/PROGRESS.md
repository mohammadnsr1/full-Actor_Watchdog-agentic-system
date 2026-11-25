# Progress

How the project has evolved, in summary. Updated 2026-11-25. Details live
elsewhere: how it fits together in `docs/ARCHITECTURE.md`, what to do next in
`docs/NEXT_STEPS.md`, the executor-baseline plan and decisions in
`docs/EXECUTOR_BASELINE.md`, where every file came from in `docs/PROVENANCE.md`.

## What the project is

A robot assembly cell (UR5e + OnRobot RG2 in Isaac Sim 5.1; UR7e + RG6 on
hardware later) with two agents that never talk to each other directly:

- the **executor**: an authoring agent writes a *planner* program, and a harness
  runs it on the cell, one episode at a time;
- the **WatchDog**: a plan-blind agent that watches each episode and judges it
  against the requirements; it may halt the arm and may send free-form
  consultations.

Research question: **do the WatchDog's consultations make the executor's
authoring faster?** The executor side is the baseline, set up like the
labmate's prompt-to-assembly workspace so that only the consultations differ.

## Status at a glance

| Track | Status |
|---|---|
| M1-M6: WatchDog brought in, platforms, planning service, plan-blind WatchDog, command server, recording | done, checked live |
| M7: planner contract, episode runner, authoring MCP | sim side done and live-tested; hardware side open |
| M8: supervisor, operations doc, records index | not started |
| EB1: labmate's assemblies, tasks and prompts in `benchmark/` | done |
| EB2: fmb1 on the stage, task bundle, checker | stage, placements, checker done; task records (`task.yaml`, environment, robot) next |
| EB3-EB8: lab verbs, container, prompts, platform MCP, WatchDog in the lab, protocol | not started |

Plan-given template on the fmb1 stage: **4/4 parts placed** (`fmb1-given-s1`,
seed 0; seed 2 also 4/4 — the nominal layout has no jitter yet, so seeds do
not vary the start).

## Timeline

| Date | What happened |
|---|---|
| 2026-10-20 | Repository started. |
| 2026-11-02 | Survey of WatchDog_V1 and the labmate's DroidStack: borrow DroidStack's ideas, not its runtime (it is unlicensed). |
| 2026-11-03 | M1 build plan; M2 WatchDog_V1 copied in (commit `595ede33`), checked live; M3 (v1 numbering) planning service with switchable backends. |
| 2026-11-10 | v2 plan: hardware platform too. Every ROS name moved to `config/platforms/{sim,hardware}.yaml`; backends `cell_backend_sim`/`hw`; Humble container; planning service (MoveIt now, cuRobo written). |
| 2026-11-11 | M5: physical events from the cell's own signals; the WatchDog made plan-blind (requirements file only, camera2 poses). M6: Kit-side command server (TCP 9010), headless Isaac, cameras, MCAP recording. |
| 2026-11-17 | M6 live checks all pass (contacts on pads, headless, recording, wrist camera1). M7 starts: planner contract, plan-given/plan-free templates, episode runner, authoring MCP. |
| 2026-11-23 | M7: v1 executor stack pruned; one start permission per episode; WatchDog roles (conformance, risk, consultation); placement frames derived from the fixture; first episodes (`m7-first-s0-r3`: 1/4). |
| 2026-11-24 | Decision: the executor is the research baseline and mirrors the labmate's setup (EB1-EB8). EB1: `benchmark/` pinned at prompt-to-assembly `f36678cb`. Found this cell's `fmb_assembly_2` *is* the labmate's fmb1. fmb1's parts put on the stage, Board 2 removed. Old stage `m7-given-s0`: 3/4; first fmb1 episode `fmb1-given-s0`: 0/4, diagnosed from the bag. Fixes: kinematic base, depenetration cap, speculative CCD, hover-then-descend, in-hand correction. |
| 2026-11-25 | Stage rebuilt and stripped of 176 saved render products; placement frames re-derived from fmb1's seats; RG2 model confirmed correct (an analysis error of mine had blamed it); `fmb1-given-s1` 4/4. Checker and WatchDog acceptance accept the swapped U pair and each part's proven symmetries. |

## Episodes so far

| Episode | Stage | Result | What it showed |
|---|---|---|---|
| `m7-first-s0-r3` | old (Board 1+2, SDF meshes) | 1/4 | harness works end to end; straight-line inserts disturbed seated parts |
| `m7-given-s0` | old | 3/4 | old mesh colliders forgave ~1.5 mm misalignment |
| `fmb1-given-s0` | fmb1 box colliders | 0/4 | 1.6 mm lateral error vs ±1.4 mm slot play; free-space approach swept into a seated part; overlap resolved by throwing a part 4.9 m |
| `fmb1-given-s1` (+ seed 2) | fmb1, fixes, re-derived seats | 4/4 | hover + in-hand correction + exact seats work |

## From the labmate's work, and what we added

The labmate's work is two repositories: **DroidStack** (`/home/mohammad/droidstack`,
unlicensed: ideas only, no code) and **prompt-to-assembly**
(`/home/mohammad/prompt-to-assembly`, Apache-2.0: data copied into the git-ignored
`benchmark/`, rebuilt by `tools/fetch_benchmark.sh`). Neither is edited.

### Taken

| From | What | Used here as |
|---|---|---|
| DroidStack (idea) | a planner is a stdio MCP server with one tool, `execute_task`, called once per episode | `docs/PLANNER_CONTRACT.md`, `executor/planner_templates/`, the episode runner |
| DroidStack (idea) | executor "robot facts" journal as untrusted hints | `/cell/executor_facts` |
| DroidStack (idea) | authoring loop: frozen snapshot, fresh episode, verdict back, `submit_work` | `executor/authoring_mcp/` |
| DroidStack (idea) | loopback newline-JSON command server inside Kit; harness-owned headless launch | `isaac/cell_command_server/`, `isaac/headless/` |
| DroidStack (idea) | sim-time budget plus wall guard; hashed episode manifest | `runner/episode_runner/` |
| DroidStack (setup) | how the authoring agent runs: Claude Code `--print`, isolated container, `kit` + `platform` MCP, evaluation budget 18, 8 h | the EB4/EB6 design in `docs/EXECUTOR_BASELINE.md` |
| prompt-to-assembly | seven assemblies with graphs, symmetry certificates and USD bundles; the fmb1 task; the prompts; ingest, resolver and checker sources | `benchmark/` (pinned, byte-identical) |
| prompt-to-assembly | fmb1's five part bundles (visual mesh, box colliders, `printed_plastic` 0.8/0.6, 0.35 mm contact offset, mass and inertia) | `assets/usd/fmb1/<part>.usda`, on the stage |
| prompt-to-assembly | fmb1's assembled poses (the seats) | the placement frames and `config/placements/fmb_assembly_2.yaml` |
| prompt-to-assembly | each part's proved symmetry group; Body2/Body5 as one interchangeable class | `symmetries` in the placements file; `acceptance.interchangeable` |
| prompt-to-assembly | physics practice: depenetration cap (PhysX SDF advice their launcher cites), speculative CCD, fixed base | `CELL_PHYSICS` in `tools/build_fmb1_stage.py` |

### Not taken

Their robots, environments, contracts, layout generator, physics profile,
reference planners, other tasks, and all DroidStack runtime. This cell keeps its
own robot, gripper (with Mohammad's RG2 calibration), controllers, ROS stack,
WatchDog and harness.

### Added by us around it

| What | Where |
|---|---|
| Matching fmb1's bodies to this cell's prims, keeping every prim frame so no config changed | `assets/usd/fmb1/binding.json`, `tools/build_fmb1_stage.py` |
| Stage build with Isaac's own USD, checks (geometry 4e-6 mm, mass, inertia, TF targets, physics, seats), `--cell-physics`, `--placements` | `tools/build_fmb1_stage.py` |
| Placement frames and the symmetry list derived from fmb1's graph (were measured on the old stage, 1-6.55 mm high) | `build_fmb1_stage.py --placements` |
| Cleaning render products Isaac saves into the stage | `tools/clean_stage_render.py` |
| Kinematic-safe teleports | `isaac/common/kit_api.py` |
| Plan-given template: hover then straight descent; in-hand correction from TF + pose channel; slip stop; `Worker.tcp_pose()` | `executor/planner_templates/` |
| Guard against the RG2 table's silent linear fallback | `planner_templates/common/geometry.py` `tcp_for` |
| Checker and WatchDog acceptance modulo symmetry and interchangeable parts | `runner/episode_runner/checker.py`, `robot_backend/watchdog/requirements.py` |
| FMB CAD split per part (all three boards) | `assets/cad/fmb/`, `tools/ingest/split_step.py` |

## Milestones (build track)

| | Scope | Status |
|---|---|---|
| M1 | build plan | done 2026-11-03 |
| M2 | bring WatchDog_V1 in | done 2026-11-03, checked live |
| M3 | platform backends, names in config, Humble container | done 2026-11-10; hardware container not yet run |
| M4 | planning service (MoveIt plan-only, cuRobo package) | done; cuRobo not installed |
| M5 | physical events, plan-blind WatchDog | done 2026-11-11 |
| M6 | command server, headless Isaac, cameras, recording | done, live checks 2026-11-17 |
| M7 | planner contract, authoring tools, episode runner | sim done 2026-11-24; hardware open |
| M8 | supervisor, operations doc, records index; budgets and health thresholds | not started |

The executor-baseline phases EB1-EB8 run alongside; `docs/EXECUTOR_BASELINE.md`
section 4.

## Removed documents

On 2026-11-25 three documents were folded in and removed; git keeps them
(`git show e4071bc:docs/<name>.md`):

- `BUILD_PLAN.md` (M1 plan, v2): its milestone table is above, its open hardware
  questions are in `docs/NEXT_STEPS.md`. Code and config comments that cite
  "BUILD_PLAN qN" refer to Mohammad's answers, kept below with the same numbers.
- `PLATFORMS.md`: now `docs/ARCHITECTURE.md` section 9.
- `INTEGRATION_SURVEY.md` (2026-11-02): the survey behind the "borrow DroidStack's
  ideas, not its runtime" decision; the credits it led to are in `docs/PROVENANCE.md`.

### Mohammad's answers to the build plan's questions (2026-11-10)

1. camera1 hand-eye calibration data is on the accelerator (`TODO(mohammad)` until fetched).
2. Both cameras are Orbbec Gemini 335Lg (G40055-270).
3. The RG6 runs through the OnRobot URCap, driven with URScript through the UR API; no ROS 2 gripper driver, so its readings are declared missing.
4. The robot is a UR7e; driver branch and controller name `TODO(mohammad)`.
5. The existing Isaac camera is the scene camera and becomes camera2; a wrist camera1 is added in the stage (done in M6).
6. No FoundationPose runs yet, sim or hardware; the pose topics are the contract, adapters come later.
7. Workstation Humble container: WatchDog and the camera2 pipeline; everything else on the accelerator; recorder and runner placement decided in M7.
8. Hardware planning: the UR AI Accelerator SDK's planner first, cuRobo, MoveIt as backup.
9. Per-platform effort limits (`limits.sim`, `limits.hardware`).
10. Builds, tests and installs are allowed here; commits and pushes are Mohammad's.
11. Prune the v1 executor modules in M7 (done).
12. `ROS_DOMAIN_ID` and RMW later (`TODO(mohammad)`).

v1 answers (2026-11-03), for the comments that cite them: WatchDog_V1 latest
commit `595ede33`; path rewrites only; copy whole, prune later; cuRobo `v0.7.8`,
same UR5e xrdf; `planner.backend: moveit`; halt = bridge cancel, no hold (v1 q6);
gripper left alone on halt; `/recorder_rgb` is in the stage; Isaac 5.1 now; the
runner sets the WatchDog session dir per episode; scorers not copied; `agents/`
stays at the root.
