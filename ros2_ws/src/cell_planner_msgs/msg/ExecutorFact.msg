# One entry of the executor's fact journal, published by the planner on
# /cell/executor_facts. Untrusted by construction: it is logged for scoring
# and never relayed to the WatchDog session.

# UTC ISO-8601 wall-clock time at publication.
string wall_time_iso

# Simulation time at publication (reference only, never used for ordering).
builtin_interfaces/Time sim_time

# One of: arm_goal, grip_close, grip_open, hold, carry, release, intent,
# claim_done.
string kind

# For kind == intent: the verb (for example grasp, carry, insert, release).
string verb

# Part id from the executor's part data, when the fact concerns a part.
string part

# Feature id (hole, slot, placement frame) when the fact concerns a feature.
string feature

# Free text: goal id, action result, or the claim's own explanation.
string detail
