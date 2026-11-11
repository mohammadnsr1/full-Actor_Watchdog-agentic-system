# One physical event on /cell/physical_events, computed by the cell-owned
# cell_physical_events node from the platform's signals. WatchDog wakes on
# these; the planner may read them. Nothing here comes from the executor.

# UTC ISO-8601 wall-clock time at detection. The only cross-process ordering key.
string wall_time_iso

# Simulation time at detection, seconds; 0.0 on a platform without a sim clock.
float64 sim_time

# One of: contact_made, contact_lost, part_lifted, part_released, part_settled,
# part_approaching_feature, part_engaged_feature, effort_rising,
# motion_started, motion_stopped, workspace_object_appeared.
string kind

# Scene ids the event concerns, in order: [part], [part, feature], [joint], ...
string[] scene_ids

# Where the evidence came from: ground_truth, effort_proxy, gripper_driver,
# camera2_pose, camera1_pose_cue, contact_report, joint_state.
string source

# Confidence in [0, 1]. 1.0 for simulator truth; a detection score for a
# perception pipeline; the configured value for a proxy.
float64 confidence

# The measured quantity behind the event (metres, effort units, rad/s ...).
float64 value

# Free text: the threshold that was crossed and the name it has in config/cell.yaml.
string detail
