# cell_platform_interfaces

Platform-neutral contracts for the cell. Python 3.10 / ROS 2 Humble and
Python 3.12 / ROS 2 Jazzy.

- `platform_config.py`: strict reader for `config/platforms/<platform>.yaml`
  and `config/cameras/<platform>.yaml`. Every topic, frame, joint, action and
  service name used by shared code is looked up here; a missing value raises
  `MissingPlatformValue` naming the key. Platform chosen by the argument,
  `CELL_PLATFORM`, or `config/cell.yaml: platform`.
- `compat.py`: the Humble/Jazzy differences (bag storage flag, optional
  message fields) and the list of APIs shared code must not use.
- `signals.py`: `available | proxy | missing` vocabulary and the report
  WatchDog's read-only `available_evidence_sources()` tool returns.
- `types.py`, `interfaces.py`: dataclasses and the four abstract backends
  (`MotionBackend`, `GripperBackend`, `PoseSource`, `SceneBackend`) plus the
  `Backends` container.
- `fjt_motion.py`: the FollowJointTrajectory motion backend both platforms use
  (bridge on sim, UR driver controller on hardware), names from config.
- `topic_poses.py`: the `vision_msgs/Detection3DArray` pose source both
  platforms use; one instance per camera role.
- `show_platform`: `ros2 run cell_platform_interfaces show_platform sim`
  prints the resolved config.

No new ROS interfaces are defined: the backends exchange standard messages
only, which keeps the package `ament_python` and identical on both distros.
