# Executor baseline

Written 2026-11-24. The executor authoring agent is the research baseline. It is
set up like the labmate's prompt-to-assembly workspace
(`/home/mohammad/prompt-to-assembly`, run by DroidStack at
`/home/mohammad/droidstack` 9f410e04), adapted to this cell. One research
question it serves: **do free-form WatchDog consultations speed up authoring?**

This replaces the M7 authoring setup (`runner/run_authoring.sh`, the
`plan_given` / `plan_free` conditions, `executor/prompts/`) as the way the
executor is run for the study. M7's episode runner, planner contract, backends,
recorder and checker stay. Phases are numbered EB1-EB8 so they do not collide
with the build track's M8 (`docs/PROGRESS.md`).

## 1. What the labmate's setup is

| Piece | How it is done there |
|---|---|
| Launch | `claude --print --model <m> --mcp-config <kit+platform> --strict-mcp-config --output-format stream-json --dangerously-skip-permissions` (`droidstack/src/droidstack/harnesses/claude_code.py`), one call, 8 h wall, no nudges or turn limit. Ends when the agent calls `submit_work`. |
| Isolation | Docker: read-only root, `--cap-drop ALL`, network only to the model API, empty `CLAUDE_CONFIG_DIR` (no CLAUDE.md, skills, memory). `/app` read-write (workspace, `artifact/`, `report.md`), `/app/evaluations` and `/data` (task view) read-only. |
| Prompt | First user message on stdin: `prompts/platform.md` + `prompts/assembly.md` (+ scene facts, task layer, optional style layer), `{evaluation_budget_episodes}` substituted. The ~1,700 lines of `prompts/docs/*.md` are read from `/data/prompts/docs/`, not pasted. |
| Task data | `tasks/<id>/task.yaml`, `assemblies/<name>/` (STEP, assembly graph with mates: seated pose, acceptance band, closing screw legs, handoff window, required order; part graphs with symmetry certificates; collider policies; USD bundles), environment, robot composite, libraries `assembly_resolver`, `grasp_frame`, `robot_record`. Checker, reference planners and protocol withheld. |
| `kit` MCP | omni-kit-mcp: `run_python`, `put_file` / `get_file` / `stat_file`, `reload_tools`, and about 20 `lab__*` verbs (frames, grasp frames, contacts arm/read, checkpoint/restore, set, attach/detach, reset_scene(seed), rest, camera, robot health, execute_mate, verify_assembly). |
| `platform` MCP | `evaluate_planner(episodes 1-3, seed?)` against a budget of 18 episodes, evidence in `/app/evaluations/NNNN/` (verdict, bag, recording, planner log, runner report); `submit_work(state, report)`, `done` refused unless the current artifact was evaluated. |
| Delivery | `/app/artifact/manifest.yaml` (budget.sim_s, admissible state, exact tool schemas) + stdio MCP server with `execute_task(task_ref)`; raw ROS (FollowJointTrajectory, Grip with `expect`, ExecuteMate), own IK, no MoveIt. |
| Protocol | 3 authoring runs per task; each frozen planner then runs on a 9-seed panel it never saw. |

## 2. Decisions (Mohammad, 2026-11-24)

1. **Lab**: build our own lab verbs on omni-kit-mcp and the cell command
   server. Do not port DroidStack's Kit command server (Isaac 6.1, >7.7k lines).
   Keeps the 2026-11-03 decision: borrow ideas, not the runtime.
2. **Agent access**: Docker container, all Claude Code tools including Bash.
3. **Task data**: bring the assemblies from prompt-to-assembly; Mohammad supplies
   the FMB board 2 CAD (FMB dataset site), which goes through their ingest to
   the same graph form.
4. **Robot surface**: keep our backends and planning service (MoveIt, cuRobo
   later). Add a direct-motion tool so the agent, and its planner, can move the
   robot without the planner (for example to wiggle a part in hand that did not
   go fully in).
5. **Prompts**: port `platform.md`, `assembly.md` and `docs/`, rewritten for this
   cell, sent as the first user message. Baseline text has no WatchDog in it;
   the consultation condition adds one layer.
6. **Metric**: evaluation episodes to the first PASS, plus wall time and tokens,
   3 runs per condition, then a 9-seed panel of each frozen planner. The
   WatchDog also watches the lab experiments in the consultation condition:
   the question is whether a second agent's free-form perspective speeds up the
   planning and solving.
7. **Adopt**: `manifest.yaml` delivery, evaluation budget 18, one fixed model
   for both conditions, 8 h wall.

Answers to the first open questions (Mohammad, 2026-11-24):

8. **Tasks**: bring from prompt-to-assembly only what the baseline needs, never
   committed (EB1), all seven assemblies; FMB CAD from the FMB site (EB2).
9. **Halts** are off in the `consult` condition: the WatchDog only consults.
10. **Two-way**: the executor may ask for a consultation, or signal that it needs
    one on something. Sharing the plan with the WatchDog is acceptable: the
    WatchDog judges against specifications and requirements, not steps. This
    relaxes plan-blindness (`docs/INDEPENDENCE.md`) for the `consult` condition
    only, and only through the request channel.
11. **Shared ROS topics** are acceptable: they are raw data streams. Host
    networking; Isaac in a container may come later.
12. **The 9-seed panel runs with the WatchDog** in the `consult` condition.

## 3. Conditions

| | `baseline` | `consult` |
|---|---|---|
| Prompt | base layers | base layers + `consult` layer |
| WatchDog during lab work | none | standing session, consultation only, no halts |
| WatchDog during `evaluate_planner` episodes and the panel | none | per-episode session as in M7, consultation only, no halts |
| WatchDog to executor | - | free-form consultations to the authoring agent (EB7) and to the running planner (`w.consultations()`) |
| Executor to WatchDog | - | `request_consultation(topic, detail, part?)`: from the authoring agent (MCP tool) and from the running planner (`w.request_consultation(...)`); may include the plan |
| Everything else | identical | identical |

## 4. Phases

### EB1. Bring in the workspace pieces

Done 2026-11-24. Only what the baseline uses (Mohammad: this cell has its own
robot, RG2 and controllers, so none of their robot, environment or asset files):

- `benchmark/` is **git-ignored**, rebuilt by `tools/fetch_benchmark.sh` from
  the pinned commit (`git archive`, byte-identical; `benchmark/UPSTREAM.json`
  holds the file count and tree digest). 381 files, about 100 MB.
- `assemblies/`: all seven (Mohammad, 2026-11-24). `fmb1` and its task: the
  worked example of the graph and task format. **fmb1 is this cell's
  fmb_assembly_2**: FMB Board 1 at 0.7 scale, identical part geometry and
  placements (`assets/cad/fmb/README.md`).
- `prompts/`: the source text for EB5.
- `src/prompt_to_assembly_ingest` + `src/droidstack_assembly_checker` (which
  the ingest imports for its symmetry math): CAD to assembly graph, run in its
  own venv with the `[cad]` pins from `pyproject.toml`. Neither imports DroidStack.
- `src/assembly_resolver`: plain Python, the planner-side feature query.
- Our own derived data (the fmb_assembly_2 graph and task) lives outside
  `benchmark/` and is tracked.
- Ingest venv: `tools/ingest/setup_venv.sh` (Python 3.12, their `[cad]` pins).
  Checked by re-deriving fmb1 from its STEP sources: all verdicts PASS, the
  assembly graph, part graphs, colliders and declaration byte-identical to the
  shipped ones; one bundle differs by a ~1e-17 quaternion component (numpy
  build). `sim-checks.json` comes from their Isaac physics battery, not run here.

### EB2. Task bundles for this cell

- FMB CAD (CC BY 4.0) in `assets/cad/fmb/`: the downloads unchanged in
  `source/`, and each split into one STEP per part by `tools/ingest/split_step.py`
  (`tools/ingest/split_fmb.sh` regenerates them byte for byte); the three boards
  also as named assemblies the ingest's `target` step accepts.
- `fmb_assembly_2` is FMB Board 1 at 0.7, the same assembly as the labmate's
  fmb1: the Board 1 split at 0.7 equals fmb1's part STEPs at every vertex and its
  assembled placements to 1e-17 m. So its graph is fmb1's; what remains is the
  binding: fmb1 occurrence -> this cell's prim and frame (Body1 base1, Body2/Body5
  the U pair, Body3 line_green, Body4 inverted_u_yellow), with the transform
  between fmb1's part frames (footprint centre, bottom at z 0) and the prim frames
  of `assets/usd/assembly_room.usd`.
- Board 2 (this cell's `base2` family at 1.4) and Board 3: split and accepted by
  `target`; a full graph also needs a motion declaration (fmb1 vendors the
  labmate's drop-animation and parametric scripts for that), later if wanted.
- **Done 2026-11-24 (stage):** the stage carries fmb1. `tools/build_fmb1_stage.py`
  replaced the five Board 1 assets with fmb1's bundles (box colliders, 1250
  kg/m^3, `printed_plastic` 0.8/0.6 average, contact offset 0.35 mm), keeping
  every prim name and prim frame, so no pose, placement, grasp offset or mesh in
  the configs changed. fmb1's object frame is the child `fmb1_occurrence_NN` of
  each part, published on TF under that name. Matching and transforms:
  `assets/usd/fmb1/binding.json` (base1 = Body1 under Rz(-90); u_brown = Body2
  / occurrence:01, u_orange = Body5 / occurrence:04; line_green = Body3;
  inverted_u_yellow = Body4). Board 2 removed from the stage and the configs.
  Independent check: geometry, poses of all child frames, mass, inertia, CoM,
  materials and TF targets all as intended.
- **Found, not changed:**
  - Our seated placement frames sat above fmb1's CAD seats: U parts +2.0 mm,
    inverted U +1.0 mm, `line_green` +6.55 mm (Body1 frame), and `line_green` and
    `inverted_u_yellow` overlapped 5.55 mm at them. **Re-derived 2026-11-25**
    (Mohammad: "derive them again"): `tools/build_fmb1_stage.py --placements`
    writes fmb1's assembled poses, through `binding.json`, to
    `/World/base1/*_placement` and `config/placements/fmb_assembly_2.yaml`; its
    check holds them there. With the real colliders every part touches base1 at
    its frame and none overlaps another. Sideways moves: `inverted_u_yellow` 1.0 mm
    (to the slot centre), `line_green` 0.5 mm, the U pair 0.1-0.2 mm. The
    `place_u_brown` / `place_u_orange` steps keep their -0.5 mm `x` target offsets
    (task file, tuned on the old frames).
  - The labmate's resolver composes part-graph features (STEP frame) with
    occurrence placements (centroid frame), which reads as 16.7 mm off for Body1
    features; to verify before the prompts point the agent at `frame()`/`locate()`.
  - Physics: the table has no physics material here (0.8/0.6 in theirs), so
    part-on-table friction is about 0.65 (was 1.0 with our old 1.0/1.0 max
    material). Pads stay 1.0 (max wins). (base1 kinematic and speculative CCD:
    settled below.)
- **fmb1-given-s0 (first episode on the fmb1 stage, 2026-11-24): 0/4, then fixed
  offline.** The old stage's `m7-given-s0` placed 3/4 with the same plan. From the
  bag (`/tf`, `/camera1/poses`, the real collider boxes placed at the recorded poses):
  1. `u_brown` reached the slot 0.6 / 1.6 mm off in x / y (the slot gives about
     ±1.4 mm) and landed on base1's top face 31 mm above the seat, the fingers
     sliding down it; it fell in on release. The old SDF colliders (rounded at
     256 voxels, 1 mm contact offset) funnelled such errors in; the boxes do not.
  2. `place_approach_u_orange` has its `target_offset` commented out in
     `config/tasks/fmb_assembly_2.yaml`, so its "approach" is a free-space move to
     the seat itself; the hanging part swept into the seated `u_brown` (it did in
     `m7-given-s0` too, by 1.4 mm), twisted 13 deg in the hand and jammed at 37 deg,
     pushing base1 21 mm.
  3. `line_green` came to rest on the jammed `u_orange`; the `inverted_u_yellow`
     insert pressed into the pile and PhysX resolved the box overlap by throwing
     `line_green` 4.9 m (no depenetration velocity cap).
  4. ~~Every part hung 6-15 mm further along the TCP's Z than the RG2 model.~~
     **Wrong, corrected 2026-11-25.** That analysis imported `robot_backend` from
     the install tree without the ROS environment, where `rg2_geometry` cannot
     find `rg2_calibration_table.csv` and silently uses its linear open/closed
     fallback, which is 6.5 / 8.0 / 12.4 mm short of the table at 21 / 26 / 43 mm.
     The executor had the table: the TCP reached the table-based grasp target to
     0.01 mm, the sim's finger contact frames sat where the table puts the grasp
     centre (within 0.5 mm; 0.7 mm for the 43 mm grasp), and every part stayed
     within ~2 mm of the planned grasp until its insert touched something. The
     real in-hand error is along the TCP's X: the fingers' midpoint moves 1.1-2.3
     mm as they close (one finger touches first), against the slot's +-1.4 mm.
     The in-hand correction below removes that; `common/geometry.tcp_for` now
     refuses to run without the table.

  Fixed (Mohammad: "both"): stage `CELL_PHYSICS` in `tools/build_fmb1_stage.py`
  (`--cell-physics`: maxDepenetrationVelocity 0.5 m/s and speculative CCD on the five
  parts, base1 kinematic), `kit_api` writes no velocity to kinematic bodies; the
  plan-given template hovers `hover_height_m` (0.10) above the approach pose and
  descends straight, and before each descent measures the part in the hand
  (`Worker.tcp_pose()` + the pose channel, arm at rest) and targets the TCP so the
  part lands on the target, stopping if the part moved >10 mm / 15 deg since the
  grasp. Offline: a fake-worker run lands all four parts exactly; tests 483 pass.
  Settled 2026-11-25 with Mohammad: the commented `u_orange` approach offset stays
  (the hover covers it); placements re-derived (above); the RG2 model is right
  (item 4 corrected); base1 kinematic means "base moved" cannot fail, accepted.
  The stage was also rebuilt from cd4d6c7 and stripped of 176 render products
  Isaac had saved into it (`tools/clean_stage_render.py`).
- `tasks/fmb_assembly_2/task.yaml` in their schema, the subset this cell uses:
  roles, starting scene, goal target, `mate_realization` (all physical-only),
  tolerances, layouts (regions anchored on our table, base1 static, seeded
  draws from `isaac/cell_command_server/layouts.py`), streams, prompt layers,
  binding (occurrence -> prim, frame id).
- An environment declaration and robot composite for this cell (token, joint
  and frame suffixes, home configuration, gripper width domain), generated
  from `config/platforms/sim.yaml` so names stay in one place.
- Checker: our ground-truth checker judges against the graph's seated poses
  and acceptance bands, modulo the symmetry certificates.

### EB3. Lab verbs

Registered as `lab.*` tools in the Kit bridge (reaching the agent as
`lab__*`), backed by the cell command server and host-side services:

| Verb | Backed by |
|---|---|
| `resolve(ref, relative_to, source)` | TF (`ros`) or stage (`sim`) |
| `checkpoint`, `restore(cp)` | command server: part poses, arm and gripper state |
| `set(objects, dynamics, robot, gripper)` | command server teleport, joint write |
| `reset_scene(seed, home)` | `load_layout` + `verify_reset` |
| `is_at_rest`, `until_at_rest(budget_s)` | pose tail window over `/clock` |
| `contacts_arm(bodies)`, `contacts_read()` | `contacts_enable` / `contacts_poll` |
| `camera_capture(pose or view)`, `camera_view_create/move` | render product on a lab camera |
| `robot_health` | bridge and controller state |
| `verify_assembly` | the checker on the live poses |
| `move_robot(joints \| tool_delta, frame, speed)` | decision 4: direct motion through the backends, no planning |

Every verb that changes the world also publishes a measured lab event
(`/cell/lab_events`: reset, restore, set, teleport) so the WatchDog does not
read a teleport as physics. `run_python` and file tools come from omni-kit-mcp
as they are.

Direct motion in the planner: `Worker.move_joints(q, duration)`,
`Worker.jog(delta, frame="tool", speed)` and a small `wiggle` example in the docs,
over `backends.motion.execute` with IK from the planning service; never through
MoveIt's planner.

### EB4. Authoring container

- Image from `docker/humble/`: ROS 2 Humble, this repo's message and backend
  packages built, `mcp` pinned, Claude Code CLI pinned, omni-kit-mcp client.
- Mounts: `/app` (session workspace) read-write, `/app/evaluations` read-only,
  `/data` (staged task view: task, assembly, environment, robot, `prompts/`,
  executor-side config) read-only. Nothing else of the repo.
- `claude --print ... --dangerously-skip-permissions`, fresh `CLAUDE_CONFIG_DIR`,
  stream-json to `records/authoring/<session>/transcript.jsonl`, wall 8 h.
- Network: ROS 2 needs DDS to the cell. Host networking with web tools denied
  and an egress allowlist (model API only), or a topic-allowlisting domain
  bridge (open question 4).

### EB5. Prompts

`executor/prompts/platform.md`, `assembly.md`, `docs/*.md` (gateway, lab, robot,
delivery, task data, layouts, packages, assembly graph, resolver) rewritten for
this cell, `consult.md` for the consultation condition. Composition and the
`{evaluation_budget_episodes}` substitution as theirs.

### EB6. Platform MCP and delivery

`evaluate_planner(episodes 1-3, seed?)` with a budget of 18 (charged only for
completed episodes), evidence in `/app/evaluations/NNNN/` (`evaluation.json`,
per episode `verdict.txt`, bag, frames, `planner.jsonl`, runner report);
`submit_work`. Delivery is `manifest.yaml` + server, checked by a local
`check-delivery` (schema, server starts, tool surface matches, no forbidden
imports). Starting scaffold: a skeleton that returns
`stopped / inadmissible_state`.

### EB7. WatchDog in the lab (`consult` only)

- One standing plan-blind session for the authoring session, woken by physics
  and lab events, consultation only.
- Consultations reach the agent through both MCP servers: every `kit` and
  `platform` tool result carries any consultations issued since the last
  result (`watchdog_notes`), and a `read_consultations` tool returns them
  on demand. Harness-independent (works for Codex too).
- Requests: `request_consultation` from the agent or the planner publishes on
  a request topic that wakes the WatchDog; its answer is an ordinary
  consultation carrying the request id. The WatchDog prompt says a request may
  describe the executor's plan and that it answers against the requirements.
- Long sessions: rotate the WatchDog session on a token or wake count with a
  carried-over summary.

### EB8. Protocol and metrics

Per authoring run: episodes used up to the first PASS (or none), wall time,
tokens and cost from the stream-json usage, lab calls, consultations issued.
Then the 9-seed panel of the submitted planner. Three runs per condition,
same model, same opening seed set. Results under `records/study/`.

## 5. Independence holes to close

- `run_python` executes inside Kit on the host and can read host files
  (`records/`, `config/requirements/`). Log every call's code; refuse code that
  names repo paths outside the task view; containerising Isaac is the full fix.
- Shared ROS domain: accepted (decision 11).

## 6. Open questions

None at the moment. Answered: tasks (8), halts (9), two-way (10), ROS (11),
panel (12).
