# One finger-pad contact event relayed from the simulator (Milestone 6),
# published on topics.pad_contacts by cell_backend_sim pad_contacts_relay.
# PhysX shape-pair reports are folded per (pad, body) pair in the command
# server (isaac/cell_command_server/contacts.py): one found, one lost.
string wall_time_iso
float64 sim_time
# Prim path of the pad rigid body (finger link) and of the other body.
string pad
string body
# found: the bodies started touching | lost: they stopped touching
string event
# found: summed impulse magnitude at first touch; lost: peak per-step impulse
# during the contact (N*s, PhysX units).
float64 impulse
# Shape pairs in contact at this event (0 on lost).
int32 shape_pairs
# Pad surface frame (scene.contact_pads[].surface_frame), empty if none.
string pad_frame
# Contact point closest to the pad frame origin, in world and in the pad frame.
# has_point false: PhysX gave no point (e.g. on lost without a previous point).
bool has_point
geometry_msgs/Point point_world
geometry_msgs/Point point_in_pad_frame
# Distance of that point from the pad frame origin; -1 when unknown.
float64 distance_to_pad_frame_m
# found: the point is within scene.pad_surface_radius_m of the pad frame.
# lost: any point during the whole contact was.
bool on_pad_surface
