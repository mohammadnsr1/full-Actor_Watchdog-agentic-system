# cell_planning_service

One planning interface, two backends, no execution.

## Services

All under `planner.service_namespace` from `config/cell.yaml` (default `/cell/planning`). Types are in `cell_planner_msgs`.

| Service | Does |
|---|---|
| `plan_to_pose` | Returns a time-parameterized `JointTrajectory` that brings `tip_link` to `target`, collision-free against the current world. |
| `plan_to_joints` | Same, to a joint-space goal. |
| `plan_cartesian` | Straight-line segment(s) through `waypoints`; returns the trajectory and the `fraction` achieved. Fails with `PARTIAL_PATH` when below `min_fraction`. |
| `check_trajectory` | Collision-checks a given trajectory; returns the first invalid point and the contact pairs. |
| `add_collision_object` | Adds or replaces one collision object; the caller supplies geometry and pose. |
| `remove_collision_object` | Removes one object by id. |
| `sync_collision_objects` | Pulls the named registry objects' TF poses into the world by delegating to the cell's `/sync_scene_objects`. Separate from `add_collision_object` on purpose. |

Every response carries a `PlanningReport`: `backend`, `planning_time_s`, `failure_code` (empty on success; vocabulary in `failure_codes.py`), `failure_reason`, `wall_time_iso`.

Request fields left at zero or empty fall back to the `planner.<backend>` section of `config/cell.yaml`. A missing default is reported as `INVALID_REQUEST`, never invented.

The service never executes. The planner sends the returned trajectory to the bridge's `/arm_controller/follow_joint_trajectory` itself.

## Backends

- **moveit** (this package): `GetMotionPlan`, `GetCartesianPath`, `GetStateValidity`, `ApplyPlanningScene`. The MoveGroup action is never used, so MoveIt never executes or touches a controller. If `compute_cartesian_path` returns an untimed path, `timing.py` parameterizes it from the joint limits file named in `planner.moveit.joint_limits_file`.
- **curobo** (`cell_planning_curobo`): same services, GPU planner, own Python environment. See that package's `INSTALL.md`.

The launch file picks the executable from `planner.backend`:

```bash
ros2 launch cell_planning_service planning_service.launch.py
```

## Tests

Offline, no ROS graph needed beyond the message packages being built:

```bash
cd ros2_ws && source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python3 -m pytest -q src/cell_planning_service/test
```
