# Attached to every planning-service response so the two backends can be
# compared from the episode record.

# Backend that answered: "moveit" or "curobo".
string backend

# Wall-clock seconds the backend spent on this request.
float64 planning_time_s

# Empty on success. Otherwise one of the codes in
# cell_planning_service/failure_codes.py (for example NO_IK_SOLUTION,
# PLANNING_FAILED, START_STATE_IN_COLLISION, GOAL_IN_COLLISION,
# INVALID_REQUEST, BACKEND_UNAVAILABLE, TIMEOUT, PARTIAL_PATH).
string failure_code

# Human-readable detail for the code above.
string failure_reason

# UTC ISO-8601 wall-clock time when the response was built. The only field
# used for cross-process ordering.
string wall_time_iso
