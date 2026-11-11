# One finger-pad contact report relayed from the simulator (Milestone 6),
# published on topics.pad_contacts by cell_backend_sim pad_contacts_relay.
string wall_time_iso
float64 sim_time
# Prim path of the pad and of the other body, as the simulator names them.
string pad
string body
# found | lost | persist (PhysX contact event type as a string)
string event
float64 impulse
