# cell_physical_events

Publishes the cell's physical event vocabulary on the platform's
`topics.physical_events` as `cell_planner_msgs/PhysicalEvent`. WatchDog wakes
on these events; the planner may read them. Nothing here comes from the
executor.

| Event | Scene ids | Signals used | Source |
|---|---|---|---|
| `motion_started`, `motion_stopped` | | arm joint speed | `joint_state` |
| `effort_rising` | [joint] | gripper effort joint slope | `joint_state` |
| `contact_made`, `contact_lost` | | gripper effort (proxy) until a pad contact report exists | `effort_proxy` / `contact_report` |
| `part_lifted`, `part_released` | [part] | camera2 pose + TCP from robot TF | `ground_truth` or `camera2_pose` |
| `part_settled` | [part] | camera2 pose speed | same |
| `part_approaching_feature`, `part_engaged_feature` | [part, feature] | camera2 pose of part and its feature frames | same |
| `workspace_object_appeared` | [id] | a camera2 detection id not in the parts file | same |

Thresholds: `config/cell.yaml: physical_events` (all `TODO(mohammad)`; a zero
disables that detector, logged once). Parts and features:
`config/parts/watchdog_parts.yaml`. A signal the platform declares `missing`
produces no events. Detector logic is ROS-free in `detectors.py` and tested in
`test/`.

```bash
ros2 run cell_physical_events physical_events_node
ros2 topic echo /cell/physical_events
```
