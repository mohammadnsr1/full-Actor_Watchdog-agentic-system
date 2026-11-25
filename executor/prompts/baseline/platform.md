# Platform brief

Deliver a planner that autonomously completes the entire task named by `task_ref`
(`{task_id}`).

All `docs/...` paths below resolve under `/data/prompts`. `/data` is the read-only task
data (`CELL_TASK_DATA`); `/app` is yours.

## Authorities

The task record and the declarations it references are the authority for the
plan-independent facts they state; the domain brief lists them under "What is given and
what you author".

The simulated cell is the authority for the physical state and outcomes it produces. The
verdict is computed from the target and tolerances over the cell's physical end state,
not from your planner's report.

The platform does not prescribe how you derive the planner or when you use the lab.

## What you deliver

Leave a runnable planner under `/app/artifact`: a stdio MCP server whose one tool,
`execute_task(task_ref)`, returns `{state: done | stopped, reason}`, and a root
`manifest.yaml` (schema `cell.planner-delivery/1`) declaring its entrypoint, its
simulated-time budget and its tools. `/app/artifact` starts as a scaffold that already
satisfies the delivery contract and stops without moving. `docs/delivery.md` is the
contract for every part of it, for what the planner may depend on, and for how it runs.

## Authoring and execution environments

During authoring, in this container:

- You have the simulator through the `kit` MCP server (`docs/lab.md`): `run_python`
  inside Isaac Sim, file transfer, and the `lab__*` verbs (poses, checkpoints and
  restore, teleport, reset with a seed, rest, contacts, cameras, verification).
- You command the robot through the same ROS 2 interfaces your delivered planner uses
  (`docs/robot.md`); this container shares the cell's ROS 2 graph. The `platform` server's
  `move_robot` moves the arm directly between episodes.
- The cell starts in the task's declared layout. What you arrange here is for testing;
  what execution starts from, and how the planner reads it, is `docs/layouts.md`.
- A second MCP server, `platform`, serves `check_delivery`, `evaluate_planner`,
  `submit_work`, `move_robot` and `robot_health`.

After delivery, the planner runs on the cell host from a fresh task-declared state, with
the same assets and physics, and no `kit` gateway, no lab verbs, no model calls and no
network; `docs/delivery.md` states what it does receive.

## Commanding the robot

Read `docs/robot.md` before commanding the robot. It is the contract for every robot
surface: names, the arm and gripper interfaces, the planning service, measured state,
frames and the simulated clock. Each name is read from the declarations, never assumed.

## Given task data

`/data` maps as `docs/task-data.md` describes. A reference that does not resolve is a
stop-and-report, never a guess.

## When to end and what to report

Work until one of these holds:

- An `evaluate_planner` episode of `/app/artifact` reads PASS.
- Your evaluation budget is spent.
- A concrete blocker in the inputs or tools prevents further progress. An unsuccessful
  approach is not a blocker while useful alternatives remain.

A session that ends without one completed `evaluate_planner` episode is recorded
UNPROVEN.

Start `/app/report.md` before your first planner edit and keep your working notes in it:
what you are building, the checks you ran and what they showed, unresolved issues, and
any platform limitations you met. Your final report is that file as it stands. If you
could not deliver a runnable planner, it states the concrete blocker and the evidence.

## Evaluate your own delivery

`evaluate_planner(episodes, seed?)` takes `/app/artifact` as it stands, runs
`check_delivery` on it, and runs your planner 1 to 3 times from a fresh task-declared
state (one seed per episode) through the same referee that judges it after delivery.
It returns when the episodes are done, with one verdict per episode and the directory
holding the evidence (`docs/delivery.md` describes it). It takes about as long as your
planner's runs; there is nothing to poll.

This session has an evaluation budget of {evaluation_budget_episodes} referee-scored
episodes across all calls; each result says how many remain. An episode the cell could
not run (`invalid`) is not charged. When the budget is spent, `evaluate_planner`
refuses, and the planner under `/app/artifact` at the end is the one administered. Your
own use of the lab is not counted.

A PASS from `evaluate_planner` is the submission bar. Your planner returning `done` in
the lab is not: only the referee's verdict on a fresh episode counts.

## Finish authoring

If you delivered a planner, before ending:

1. Leave the complete delivery under `/app/artifact`.
2. Call `check_delivery` and correct every problem it reports. `ok` means the delivery
   is well-formed, not that the planner works.
3. Call `evaluate_planner` on that same delivery and state the verdict of your last
   evaluation in your report.
4. Confirm that everything the planner needs is inside `/app/artifact` or listed as
   available at execution in `docs/delivery.md`.

Whether or not you delivered, end by calling `submit_work(state, report)`: `state` is
`done` when you delivered a planner and `stopped` when you did not; `report` is your
report file's path (`/app/report.md`). `done` is refused `SUBMIT_WORK_UNEVALUATED` unless
an `evaluate_planner` call completed on `/app/artifact` exactly as it stands; an edit
after the evaluation needs a new one. `stopped` is always accepted. Calling again
replaces the earlier submission. When you end the session, `/app/artifact` is collected
as it stands. If a concrete blocker prevented a runnable delivery, do not fabricate one:
submit `stopped` with the report and end.
