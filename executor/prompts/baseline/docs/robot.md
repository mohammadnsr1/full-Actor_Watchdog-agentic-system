# The robot contract

The robot reaches you as **ROS 2 topics, services and actions** on the cell's graph (ROS 2
Jazzy, simulated time on `/clock`). Your planner uses exactly these; so may you, from this
container. Names come from the robot composite (`robots/<name>/composite.yaml`) and the
environment declaration (`environments/<name>/declaration.yaml`): read them, never type
them in. This cell's names carry no robot token or namespace.

Code that already speaks this contract is in the scaffold (`/app/artifact`): `worker.py`
(one ROS node: poses, TF, camera, planning, arm, gripper, direct motion, facts) and
`geometry.py` (poses, RG2 grasp geometry, joint paths). Use it, change it or replace it.
It builds on Python packages available at execution and here: `cell_platform_interfaces`
(platform config, backends), `cell_backend_sim` (sim backends), `cell_planner_msgs`
(messages and services), `robot_backend.utils` (pose math, RG2 calibration).

## 1. Frames

`frames.world` is the planning and pose frame. `base` (`base_link`) is fixed in it.
`tcp` is a fixed frame on the gripper; TF publishes `world -> ... -> tcp` from the joint
states. The pose channel's poses are in `world`.

## 2. The arm

| surface | type | name |
|---|---|---|
| trajectories | `control_msgs/action/FollowJointTrajectory` | `controllers.arm.action` |
| cancel | the action's cancel service | `controllers.arm.cancel_service` |
| joint state | `sensor_msgs/JointState` | `measured_state.joint_states` |

Address joints by name (`joints.arm`). A trajectory's last point is reached within
`goal_tolerance_rad`. The arm executes exactly what it is sent: nothing checks
collisions on this path.

## 3. Planning (plan only, never moves the arm)

Services under `controllers.planning.service_namespace` (`cell_planner_msgs/srv`):
`plan_to_pose` (collision-free, to a `tip_link` pose), `plan_to_joints`,
`plan_cartesian` (straight-line waypoints; `avoid_collisions` selectable), and
`check_trajectory`; `add_collision_object` / `remove_collision_object` edit the planning
scene. The planning scene holds the cell and the parts as the registry describes them.
Each returns a `trajectory_msgs/JointTrajectory` to send to the arm, or `ok: false` with
a failure code and reason. Sampling planners occasionally fail on reachable goals: retry.

## 4. Observation

| stream | what |
|---|---|
| `streams.object_poses` (`/camera1/poses`, `vision_msgs/Detection3DArray`) | one detection per id: `results[0].hypothesis.class_id` is the object id (`task.yaml streams.object_frames`), `results[0].pose.pose` its prim-frame pose in `world`. On this cell the channel also carries `<part>_placement` frames, the poses the referee seats each part at. Simulator truth at 20 Hz |
| TF | the robot's links and `tcp` |
| `streams.camera1_images` | the wrist camera's RGB, depth (metres) and intrinsics; `optical_frame` on TF |
| `streams.gripper_width` | the RG2's true finger gap (m) |
| `streams.executor_facts` | yours to publish (§6) |

Read a part's pose with the arm at rest when precision matters: a pose sampled while
moving lags TF by up to a few millimetres.

## 5. The gripper (OnRobot RG2)

Services `controllers.gripper.services` (`open`, `close`, `set_position`) and the action
`guarded_move_action` (close to a width under a force threshold; it stops on contact and
reports `stop_reason`, the final width and effort). `gripper.width_domain_m` is the
physical range.

The RG2 is a four-bar linkage: its fingertips stay parallel and move along the TCP's +Z
as they close, so the **grasp centre** (midway between the fingertips) is 68.5 mm from
the TCP when closed and 25 mm when fully open, calibrated per width.
`robot_backend.utils.rg2_geometry.tcp_target_from_grasp_center(center, width)` gives the
TCP pose that puts the grasp centre at `center` with the jaws `width` apart. Use it, not
a straight line between the two ends (6-12 mm off). The fingers do not close exactly
symmetrically: measure a part's pose in the hand after grasping (TCP from TF, part from
the pose channel) rather than assuming the planned grasp.

## 6. The fact journal

Publish `cell_planner_msgs/ExecutorFact` on `streams.executor_facts` as you work (kinds
`arm_goal`, `grip_close`, `grip_open`, `hold`, `carry`, `release`, `intent`,
`claim_done`). It is recorded for analysis, never read by the referee.

## 7. Direct motion

`Worker.move_joints(goal)` sends a quintic joint path straight to the arm (default cap
0.5 rad/s) and `Worker.jog(delta, frame)` moves the TCP along a straight line; both skip
collision checking, for small known moves such as wiggling a part that did not go fully
in. Between episodes, the platform's `move_robot` does the same for you.
