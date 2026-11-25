# Layouts: where things start

## 1. The regime

`task.yaml` `layouts.regime: fixed_with_seeded_jitter`. Every episode starts from the
fixed layout `layouts/<name>.json` (world poses of every object), with each free part
moved by a draw from its `layouts.jitter` range: x and y within `xy_m`, yaw within
`yaw_deg`, uniform, from the episode's seed. Seed 0 is the fixed layout exactly. A range
of 0 means the part always starts at its fixed pose. Ranges can change between task
versions: never rely on a part being exactly where the file says.

## 2. Static and free

`layouts.static` lists the fixture (`base1`): it stands at its fixed pose in every
episode, is kinematic (no contact moves it), and is the reference the target is judged
against. `layouts.free` lists the parts the planner moves; they start resting on the
table.

## 3. Resetting in the lab

`lab__reset_scene(seed)` puts the cell into the arrangement an episode with that seed
would start from: parts placed, arm at the home pose, gripper open. Use other seeds to
see the starts your planner will meet. `lab__checkpoint` / `lab__restore` keep and bring
back any arrangement you make yourself.

## 4. Reading the arrangement at run time

The planner reads where things are from the pose channel (`docs/robot.md` §4), each
time it needs them, with the arm at rest when precision matters. Never from the layout
file or a literal pose: the referee runs seeds you have not seen.
