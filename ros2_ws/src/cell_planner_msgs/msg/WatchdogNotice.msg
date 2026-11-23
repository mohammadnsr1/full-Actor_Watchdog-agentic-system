# Published on /cell/watchdog_notices. The planner subscribes when
# watchdog.consult is on; the episode runner publishes kind == episode_end.

# UTC ISO-8601 wall-clock time at publication.
string wall_time_iso

# Simulation time at publication (reference only).
builtin_interfaces/Time sim_time

# One of: halt, ruling, consultation, episode_end.
string kind

# Identifier shared by a halt notice and the ruling that answers it.
string halt_id

# Filled when kind == ruling.
Ruling ruling

# Free text. For a consultation: the WatchDog's free-form text to the executor.
string detail

# The part a ruling or a consultation concerns ("" when none).
string part

# consultation only, optional ("" when none): measured physical quantities
# as a JSON object of name -> number or list of numbers, every name ending in
# its unit (_m, _rad, _n, _s, _m_s, _rad_s, _effort, _count), plus "frame"
# naming the frame vectors are expressed in.
string measurements_json
