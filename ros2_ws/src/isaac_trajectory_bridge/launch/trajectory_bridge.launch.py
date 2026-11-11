from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Names (joints, topics, action, gripper services) come from the sim platform
# config, config/platforms/sim.yaml, never from this file. The RG2 targets
# below are validated controller constants, not names, and stay as arguments.
from cell_platform_interfaces.platform_config import load_platform_config

_PLATFORM = load_platform_config("sim")


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_full_joint_command = LaunchConfiguration("publish_full_joint_command")
    start_rg2_controller = LaunchConfiguration("start_rg2_controller")
    rg2_open_position = LaunchConfiguration("rg2_open_position")
    rg2_close_position = LaunchConfiguration("rg2_close_position")
    rg2_command_duration_sec = LaunchConfiguration("rg2_command_duration_sec")
    rg2_command_rate_hz = LaunchConfiguration("rg2_command_rate_hz")
    goal_tolerance = LaunchConfiguration("goal_tolerance")
    goal_time_tolerance_sec = LaunchConfiguration("goal_time_tolerance_sec")

    motion = _PLATFORM.section("motion")
    gripper = _PLATFORM.section("gripper")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if Isaac publishes /clock.",
            ),
            DeclareLaunchArgument(
                "publish_full_joint_command",
                default_value="false",
                description=(
                    "false = publish only the configured arm joints to "
                    "the joint command topic; true = publish all joints from the joint "
                    "state topic while overwriting the arm joints, which should only be used if "
                    "Isaac requires full-articulation commands and gripper "
                    "preservation is known safe."
                ),
            ),
            DeclareLaunchArgument(
                "start_rg2_controller",
                default_value="true",
                description="Start the gripper-only RG2 controller node.",
            ),
            DeclareLaunchArgument(
                "rg2_open_position",
                default_value=str(gripper.get("open_position", "")),
                description="Default RG2 open target in master joint radians (platforms/sim.yaml gripper.open_position).",
            ),
            DeclareLaunchArgument(
                "rg2_close_position",
                default_value=str(gripper.get("close_position", "")),
                description="Default RG2 close target in master joint radians (platforms/sim.yaml gripper.close_position).",
            ),
            DeclareLaunchArgument(
                "rg2_command_duration_sec",
                default_value=str(gripper.get("command_duration_s", "")),
                description="Duration in seconds for RG2 gripper interpolation commands.",
            ),
            DeclareLaunchArgument(
                "rg2_command_rate_hz",
                default_value=str(gripper.get("command_rate_hz", "")),
                description="Publish rate in Hz for RG2 gripper interpolation commands.",
            ),
            DeclareLaunchArgument(
                "goal_tolerance",
                default_value=str(motion.get("goal_tolerance_rad", "")),
                description=(
                    "Per-joint acceptance band in radians for the final "
                    "trajectory point -- a gross-failure detector, NOT the "
                    "precision spec. A per-joint band cannot express a "
                    "Cartesian requirement; do not re-tighten it to a TCP "
                    "figure. Measured floor is ~4.1e-4 rad on wrist_2 and is "
                    "stiffness-independent, and /joint_states is quantized to "
                    "1e-4 rad. See follow_joint_trajectory_server.py for the "
                    "measurements. Cartesian precision comes from MoveIt's "
                    "goal constraints instead."
                ),
            ),
            DeclareLaunchArgument(
                "goal_time_tolerance_sec",
                default_value=str(motion.get("goal_time_tolerance_s", "")),
                description=(
                    "Seconds past the trajectory's final timestamp that the "
                    "bridge keeps holding the last setpoint while waiting for "
                    "goal_tolerance. Used only when MoveIt does not set "
                    "goal_time_tolerance on the goal itself. Raising it "
                    "distinguishes a drive still ringing (a longer window "
                    "converges) from genuine steady-state droop (it does not)."
                ),
            ),
            Node(
                package="isaac_trajectory_bridge",
                executable="follow_joint_trajectory_server",
                name="isaac_follow_joint_trajectory_bridge",
                output="screen",
                parameters=[
                    {
                        "joint_names": _PLATFORM.joints("arm"),
                        "joint_command_topic": _PLATFORM.topic("joint_command"),
                        "joint_state_topic": _PLATFORM.topic("joint_states"),
                        "action_name": str(_PLATFORM.get("motion", "action")),
                        # Matches physxScene:timeStepsPerSecond in the Isaac
                        # stage; a mismatch beats against the physics loop.
                        "publish_rate_hz": float(_PLATFORM.get("motion", "publish_rate_hz")),
                        "goal_tolerance": goal_tolerance,
                        "goal_time_tolerance_sec": goal_time_tolerance_sec,
                        "require_joint_state_before_execute": True,
                        "publish_full_joint_command": publish_full_joint_command,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="isaac_trajectory_bridge",
                executable="rg2_controller",
                name="rg2_controller",
                output="screen",
                condition=IfCondition(start_rg2_controller),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "joint_state_topic": _PLATFORM.topic("joint_states"),
                        "joint_command_topic": _PLATFORM.topic("joint_command"),
                        "service_namespace": str(_PLATFORM.get("gripper", "namespace")),
                        "open_position": rg2_open_position,
                        "close_position": rg2_close_position,
                        "command_duration_sec": rg2_command_duration_sec,
                        "command_rate_hz": rg2_command_rate_hz,
                    }
                ],
            ),
        ]
    )
