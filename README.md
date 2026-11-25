# Actor / WatchDog cell

A robot assembly cell (UR5e + RG2 in Isaac Sim; UR7e + RG6 on hardware) with
two independent agents: an executor side that writes and runs a planner, and
a WatchDog that judges each episode without seeing the plan.

| Read | For |
|---|---|
| `docs/ARCHITECTURE.md` | how the code and files connect, ROS 2 up to the agents (diagrams), platforms |
| `docs/NEXT_STEPS.md` | current state and what to run next |
| `docs/PROGRESS.md` | how the project has evolved; what came from the labmate's work |
| `docs/EXECUTOR_BASELINE.md` | the executor-baseline plan (EB1-EB8) and decisions |
| `docs/PLANNER_CONTRACT.md` | what a planner is and may do |
| `docs/INDEPENDENCE.md` | what each agent can and cannot reach |
| `docs/PROVENANCE.md` | where every file came from |

Entry points: `runner/episode_runner/run.sh` (one episode),
`runner/run_authoring.sh [sim|hardware] [plan_given|plan_free]` (an authoring
session), `runner/run_watchdog.sh` (a manual WatchDog session).
