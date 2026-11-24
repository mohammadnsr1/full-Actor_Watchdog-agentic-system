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
with BUILD_PLAN's M8.

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
   committed (EB1); `fmb_assembly_2` from Mohammad's FMB CAD (EB2).
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
  holds the file count and tree digest). 149 files, 4.3 MB.
- `assemblies/fmb1/` and its task: the worked example of the graph and task
  format, and a possible second task on this cell.
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

- `fmb_assembly_2`: ingest Mohammad's FMB Board 2 CAD into
  `assemblies/fmb_assembly_2/` (tracked). fmb1 is the labmate's FMB Board 1 by
  the same recipe: `sources/` holds one STEP per part, `_assembly.step`, a
  `manifest.json` (`assembly-ingest/corpus-manifest/1`: parts, occurrences, a
  fastening class per mate, `motion_source: anim_drop_script`, each part
  dropped straight down in assembly order), and the scripts. Without an
  assembled STEP, compose one from the per-part STEPs and the seated poses in
  `config/placements/fmb_assembly_2.yaml`. Reconcile part frames with the prims in
  `assets/usd/assembly_room.usd` (the graph's part frame must be the prim frame,
  or the bundle carries the transform).
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
