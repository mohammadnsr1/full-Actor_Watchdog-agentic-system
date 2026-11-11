# cell_backend_sim

Isaac Sim implementation of `cell_platform_interfaces`. Sim only (Jazzy);
the Humble container skips it.

| Backend | Wraps | Names from `config/platforms/sim.yaml` |
|---|---|---|
| `SimMotion` | Isaac trajectory bridge `FollowJointTrajectory` | `motion.action`, `motion.cancel_service`, `topics.joint_states`, `joints.arm` |
| `SimRG2Gripper` | RG2 controller services and `guarded_move` action | `gripper.services.*`, `gripper.action.guarded_move`, `topics.gripper_width_true`, `joints.gripper_effort`, `gripper.readings` |
| pose sources | `vision_msgs/Detection3DArray` on the two camera topics | `poses.*` |
| `SimScene` | Kit-side command server over loopback TCP (Milestone 6) and ROS health | `scene.host`, `scene.port`, `topics.clock` |

`ground_truth_pose_publisher` turns the Isaac-published object TF into the
two pose topics so sim and hardware consumers read the same contract. It
publishes every enabled registry object on both topics with score 1.0 and
does not model per-camera visibility.

```bash
ros2 run cell_backend_sim ground_truth_pose_publisher
```

The bringup launch starts it when `poses.source` is `ground_truth`
(`start_pose_publisher:=true`, the default).
