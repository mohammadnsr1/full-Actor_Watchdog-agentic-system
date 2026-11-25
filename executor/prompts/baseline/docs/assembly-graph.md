# The assembly graph

`assemblies/<name>/artifacts/<name>.assembly-graph.json` is `{schema, nodes}`; nodes are
addressed by id (`occurrence:01`, `mate:00`, ...). What a planner uses:

## 1. Occurrences

`occurrence:NN.payload.assembled_pose` is the occurrence's pose in the assembly frame
(`rotation_row_major`, `translation_m`), with the parts assembled. `part_type.label`
names the part (`Body1` ...). The pose addresses the part's **object frame**: the
part's STEP frame translated to its centroid (`frame:object.payload.origin_step_m` in its
part graph), same axes.

The seated pose of a part relative to the fixture is
`inverse(assembled_pose(fixture)) (x) assembled_pose(part)`, object frame to object frame.
To compare with the pose channel, go through the task's binding: the channel's pose of
an object is its **prim frame**, and `prim_from_object` gives the object frame in it, so

    world_T_object = world_T_prim (x) prim_from_object

## 2. Mates

`mate:NN.payload`: `mover` and `reference` (occurrences), `mate_class` (`insertion`),
`mate_frame` (origin and axes in the reference occurrence's object frame; `z` points
from the seated pose toward the approach side), `legs` (the closing motion: for these
mates a straight screw along `-z` of `translate_m`, i.e. the mover comes down the mate
frame's `z` axis into the seat), `entry_plane` and `handoff_window` (where the approach
hands over to the insertion), and `seated.rel_pose` (the mover in the reference's
object frame when seated). `seated.acceptance` is the source assembly's own band
(1 mm, 1 deg); **this task's referee uses the task's `tolerances`**.

## 3. Classes and symmetry

`class:NN.payload.members` groups geometrically identical parts: members of one class
are interchangeable where the task allows (`goal.equivalence`). Each part graph's
`cert:00` is a proved symmetry certificate; its `gen:*` nodes with `kind: rotation` and
`proof.verdict: PROVED` generate the part's proper symmetries (axis `direction` through
`point_m`, in the STEP frame, angle `angle_rad`). A part turned by one of them occupies
exactly the same space, and the referee accepts it.

## 4. Part graphs

`<Part>.part-graph.json` holds faces, edges, vertices and their orbits under the
symmetry group, in the STEP frame of the part (`sources/<Part>.step`). Subtract
`frame:object.origin_step_m` to express a feature in the object frame before composing
it with an assembled pose or a channel pose.
