# The lab

The `kit` MCP server connects you to the running simulator (Isaac Sim 5.1) for authoring
only; your delivered planner has none of it.

## 1. The verbs (`lab__*`)

| verb | what |
|---|---|
| `lab__resolve(ref, relative_to?)` | world pose of an object id, `tcp`, or a prim path; or its pose in another's frame |
| `lab__checkpoint(name?)` / `lab__restore(checkpoint)` | keep / bring back every object's pose and every robot joint |
| `lab__set(objects?, dynamics?, robot?, gripper?)` | teleport objects (zero velocity), switch an object kinematic or dynamic, set joints (rad) or the gripper master joint (rad) |
| `lab__reset_scene(seed, layout?)` | the starting arrangement of an episode with that seed (`docs/layouts.md`), arm home, gripper open |
| `lab__is_at_rest(objects?, ...)` / `lab__until_at_rest(budget_s, ...)` | speeds over the last second of sim time against 2 mm/s and 0.02 rad/s; the second waits |
| `lab__contacts_arm(bodies)` / `lab__contacts_read(since_seq)` | contact reports between any rigid bodies: made, lost, impulse, point, sim time; and who touches whom now |
| `lab__verify_assembly()` | the referee's rule on the live scene: each part's error to its seat, modulo symmetry and interchangeability |
| `lab__camera_view(name, eye, target)` / `lab__camera_capture(camera, width, height)` | place a lab camera; render one RGB image from `camera1`, `camera2` (the table camera), or a lab camera, saved as a PNG on the cell host (fetch it with `get_file`) |
| `lab__robot_health()` | the simulator's side: playing, sim time advancing, joint positions |
| `lab__events(since_seq)` | the lab's own journal of what you changed |

Teleports, restores and resets are published as lab events, so the scene's own record
tells them apart from physics. World-changing verbs are refused while an evaluation
episode runs.

## 2. `run_python` and files

`run_python(code)` executes Python inside the simulator on its main thread (`omni`,
`pxr`, `isaacsim` modules; the stage is `omni.usd.get_context().get_stage()`); objects
are at `/World/<object id>`. Use it to read physics properties, inspect colliders or
measure. Do not stop the timeline, open or save stages, or delete prims: the cell is
shared with the referee's episodes. `put_file` / `get_file` / `stat_file` move files
between this container and the cell host.

## 3. The robot from here

This container is on the cell's ROS 2 graph: you can echo topics, call the planning
service, and run your planner's code directly (`cd /app/artifact && python3 -c "import
planner; print(planner.run_task('<task id>'))"`), which drives the real simulated arm. The
platform's `move_robot` and `robot_health` cover the common cases. Do not drive the arm
while an `evaluate_planner` call is running.

## 4. Physics you can measure

Parts are rigid bodies with box colliders (`bundles/<Part>.usda`: sizes, mass, inertia,
`printed_plastic` friction 0.8 static / 0.6 dynamic, contact offset 0.35 mm); the
gripper pads use friction 1.0, and PhysX combines a pair with its `max` rule. The table
has no material of its own. The fixture is kinematic. Physics runs at 60 Hz. Measure
what your approach depends on (slot play, slip in the hand, settling) rather than
assuming it.
