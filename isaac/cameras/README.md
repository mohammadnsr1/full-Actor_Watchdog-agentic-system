# Sim cameras

- **camera2 (table)** is the stage's existing `/World/Camera` with `/Graph/ROS_Camera`
  (rgb, depth, camera_info on the topics in `config/cameras/sim.yaml`). Its
  optical-frame TF comes from `sim_moveit_bringup/config/cameras.yaml`, which
  carries the same extrinsics as the cameras config.
- **camera1 (wrist)** does not exist yet. `setup_cameras.py` creates it under the
  wrist link with a ROS 2 graph of the same shape once `config/cameras/sim.yaml`
  has its pose (from the hardware hand-eye calibration), resolution and
  `enabled: true`. Run inside Kit, then save the stage.
- **Ground-truth poses** on `/camera1/poses` and `/camera2/poses` come from
  `cell_backend_sim ground_truth_pose_publisher` (Milestone 3), not from these
  prims; FoundationPose on the sim images is a later switch (`poses.source`).
