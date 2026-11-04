# A WatchDog ruling after a halt. Limits are physical quantities only
# (effort, speed, approach clearance), never motions; the WatchDog enforces
# them through its own monitors.

# One of: allowed, allowed_with_limits, denied_for_part, escalate.
string ruling

# The part the ruling applies to (empty = whole episode).
string part

# Parallel arrays: limit name (for example max_effort, max_tcp_speed_m_s,
# min_approach_clearance_m) and its value.
string[] limit_names
float64[] limit_values

# The WatchDog's stated reason.
string reason
