# Actor / WatchDog cell

A robot assembly cell (UR5e + RG2 in Isaac Sim; UR7e + RG6 on hardware) with
two independent agents: an executor side that writes and runs a planner, and
a WatchDog that judges each episode without seeing the plan.

| Read | For |
|---|---|
| `docs/ARCHITECTURE.md` | how the code and files connect, ROS 2 up to the agents (diagrams) |
| `docs/NEXT_STEPS.md` | current state and what to run next |
| `docs/PLANNER_CONTRACT.md` | what a planner is and may do |
| `docs/INDEPENDENCE.md` | what each agent can and cannot reach |
| `docs/PLATFORMS.md` | sim vs hardware |
| `docs/BUILD_PLAN.md`, `docs/PROVENANCE.md` | milestones; where every file came from |

Entry points: `runner/episode_runner/run.sh` (one episode),
`runner/run_authoring.sh [sim|hardware] [plan_given|plan_free]` (an authoring
session), `runner/run_watchdog.sh` (a manual WatchDog session).
