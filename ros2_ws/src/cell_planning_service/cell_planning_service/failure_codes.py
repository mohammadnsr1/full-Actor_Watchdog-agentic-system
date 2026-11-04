"""Failure codes shared by every planning backend.

A backend maps its native error onto exactly one of these so the episode
record can compare backends on the same vocabulary. The native detail goes
into PlanningReport.failure_reason.
"""

from __future__ import annotations

NONE = ""                       # success
INVALID_REQUEST = "INVALID_REQUEST"         # missing or malformed request fields
NO_IK_SOLUTION = "NO_IK_SOLUTION"           # pose unreachable for the tip link
START_STATE_INVALID = "START_STATE_INVALID"  # start state missing, stale or out of limits
START_STATE_IN_COLLISION = "START_STATE_IN_COLLISION"
GOAL_IN_COLLISION = "GOAL_IN_COLLISION"
GOAL_VIOLATES_LIMITS = "GOAL_VIOLATES_LIMITS"
PLANNING_FAILED = "PLANNING_FAILED"         # search ran and found nothing
TIMEOUT = "TIMEOUT"                         # budget exhausted
PARTIAL_PATH = "PARTIAL_PATH"               # Cartesian fraction below min_fraction
TRAJECTORY_IN_COLLISION = "TRAJECTORY_IN_COLLISION"  # check_trajectory result
BACKEND_UNAVAILABLE = "BACKEND_UNAVAILABLE"  # service/action not up, library missing
BACKEND_ERROR = "BACKEND_ERROR"             # anything else the backend raised
WORLD_ERROR = "WORLD_ERROR"                 # collision object add/remove/sync failed

ALL = frozenset(
    {
        INVALID_REQUEST,
        NO_IK_SOLUTION,
        START_STATE_INVALID,
        START_STATE_IN_COLLISION,
        GOAL_IN_COLLISION,
        GOAL_VIOLATES_LIMITS,
        PLANNING_FAILED,
        TIMEOUT,
        PARTIAL_PATH,
        TRAJECTORY_IN_COLLISION,
        BACKEND_UNAVAILABLE,
        BACKEND_ERROR,
        WORLD_ERROR,
    }
)


# moveit_msgs/MoveItErrorCodes value -> shared code. Values are the message
# constants; names are kept as comments so the table can be checked against
# /opt/ros/<distro>/share/moveit_msgs/msg/MoveItErrorCodes.msg.
MOVEIT_ERROR_CODE_MAP: dict[int, str] = {
    1: NONE,                        # SUCCESS
    99999: PLANNING_FAILED,         # FAILURE
    -1: PLANNING_FAILED,            # PLANNING_FAILED
    -2: PLANNING_FAILED,            # INVALID_MOTION_PLAN
    -3: PLANNING_FAILED,            # MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE
    -4: BACKEND_ERROR,              # CONTROL_FAILED (never expected: plan only)
    -5: BACKEND_ERROR,              # UNABLE_TO_AQUIRE_SENSOR_DATA
    -6: TIMEOUT,                    # TIMED_OUT
    -7: BACKEND_ERROR,              # PREEMPTED
    -10: START_STATE_IN_COLLISION,  # START_STATE_IN_COLLISION
    -11: START_STATE_INVALID,       # START_STATE_VIOLATES_PATH_CONSTRAINTS
    -26: START_STATE_INVALID,       # START_STATE_INVALID
    -12: GOAL_IN_COLLISION,         # GOAL_IN_COLLISION
    -13: GOAL_VIOLATES_LIMITS,      # GOAL_VIOLATES_PATH_CONSTRAINTS
    -14: GOAL_VIOLATES_LIMITS,      # GOAL_CONSTRAINTS_VIOLATED
    -27: GOAL_VIOLATES_LIMITS,      # GOAL_STATE_INVALID
    -28: INVALID_REQUEST,           # UNRECOGNIZED_GOAL_TYPE
    -15: INVALID_REQUEST,           # INVALID_GROUP_NAME
    -16: INVALID_REQUEST,           # INVALID_GOAL_CONSTRAINTS
    -17: INVALID_REQUEST,           # INVALID_ROBOT_STATE
    -18: INVALID_REQUEST,           # INVALID_LINK_NAME
    -19: INVALID_REQUEST,           # INVALID_OBJECT_NAME
    -21: BACKEND_ERROR,             # FRAME_TRANSFORM_FAILURE
    -22: BACKEND_ERROR,             # COLLISION_CHECKING_UNAVAILABLE
    -23: BACKEND_ERROR,             # ROBOT_STATE_STALE
    -24: BACKEND_ERROR,             # SENSOR_INFO_STALE
    -25: BACKEND_ERROR,             # COMMUNICATION_FAILURE
    -29: BACKEND_ERROR,             # CRASH
    -30: BACKEND_ERROR,             # ABORT
    -31: NO_IK_SOLUTION,            # NO_IK_SOLUTION
}


def from_moveit(value: int) -> str:
    """Map a MoveItErrorCodes value onto the shared vocabulary."""
    return MOVEIT_ERROR_CODE_MAP.get(int(value), BACKEND_ERROR)
