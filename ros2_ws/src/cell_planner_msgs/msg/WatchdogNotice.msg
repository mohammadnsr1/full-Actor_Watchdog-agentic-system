# Published on /cell/watchdog_notices. The planner subscribes when
# watchdog.consult is on; the episode runner uses kind == episode_end.

# UTC ISO-8601 wall-clock time at publication.
string wall_time_iso

# Simulation time at publication (reference only).
builtin_interfaces/Time sim_time

# One of: halt, ruling, episode_end.
string kind

# Identifier shared by a halt notice and the ruling that answers it.
string halt_id

# Filled when kind == ruling.
Ruling ruling

# Free text.
string detail
