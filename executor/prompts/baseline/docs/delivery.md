# The delivery contract

## 1. What `/app/artifact` holds

- `manifest.yaml`, schema `cell.planner-delivery/1`:

  ```yaml
  schema: cell.planner-delivery/1
  entrypoint: run.sh          # inside the artifact, executable; starts a stdio MCP server
  tools:
    - name: execute_task
      input: {task_ref: string}
  budget:
    sim_s: 300                # simulated seconds for one execute_task call (the cell caps it)
  admissible_states: [done, stopped]
  ```

- The server the entrypoint starts: exactly one tool, `execute_task(task_ref: str)`,
  returning `{"state": "done" | "stopped", "reason": str}`. stdout is the MCP channel:
  log through Python `logging` (logger `planner`), never `print`.
- Everything else the planner needs, as files inside the artifact.

The scaffold's `run.sh` sources ROS 2 Jazzy and the cell's workspace on the cell host and
starts `server.py` with the cell's Python; `server.py` calls `planner.run_task(task_ref)`.
Keep that chain unless you have a reason to change it.

## 2. What execution provides

The planner runs on the cell host, started by the episode runner, with:

| | |
|---|---|
| `CELL_TASK_DATA` | the task data (the same files as `/data`) |
| `CELL_PLATFORM`, `CELL_REPO_ROOT` | the platform name and the cell's root, for `cell_platform_interfaces.platform_config` |
| `CELL_EPISODE_DIR` | the episode's record; the scaffold logs to `executor/planner.jsonl` and saves images to `executor/images/` there |
| Python 3.12 | `rclpy` and the ROS 2 Jazzy Python packages, `cell_platform_interfaces`, `cell_backend_sim`, `cell_planner_msgs`, `robot_backend.utils`, numpy, scipy, PyYAML, Pillow, OpenCV, trimesh, mcp |

Nothing else: no network, no package installs, no `kit` gateway or lab verbs, no model
calls. A planner may not reference the simulator's command server or stage files, the
Kit bridge, the table camera (`camera2`) or the referee's data; `check_delivery` refuses
one that does.

## 3. An episode

1. The scene is reset to the episode's seed (`docs/layouts.md`) and verified.
2. The runner starts your entrypoint, lists its tools, and calls `execute_task` once,
   with the task id.
3. The call has the manifest's sim-time budget (capped by the cell) and a wall-clock
   guard; past either, the arm's goal is cancelled and the call is stopped.
4. After it returns, the parts settle and the referee judges the end state.

**PASS** means your planner returned `done` and the referee found every part seated
within the task's tolerances (modulo symmetry and interchangeability) and the fixture
undisturbed. Anything else is a failure, except `invalid`: the cell could not run the
episode, which is not charged.

## 4. The evidence

`evaluate_planner` writes `/app/evaluations/NNNN/`:

| | |
|---|---|
| `evaluation.json` | the episodes, their seeds and outcomes, and the budget left |
| `episode-K/verdict.json`, `verdict.txt` | the referee's verdict: outcome, your planner's return, each part's error to its seat |
| `episode-K/planner.jsonl`, `planner.stderr.jsonl` | your planner's log and raw stderr |
| `episode-K/runner.jsonl` | the runner's stages and timings |
| `episode-K/bag/` | the episode's MCAP bag: joint states, TF, the pose channel, your facts, the cell's physical events and pad contacts |
| `episode-K/images/`, `frames/` | images your planner saved; scene-camera frames, about one per second |

The bag reads with the `mcap` and `mcap-ros2-support` Python packages, which you may
install here for analysis.
