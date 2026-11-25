# Assembly — domain brief

Complete the requested assembly by producing the physical target the task declares.

Read the assembly at `/data/assemblies/<name>/` with `docs/assembly-graph.md`, and the
task's binding from graph occurrences to the cell's objects and frames.

## What is given and what you author

A given is a fact that holds however you solve the task. The task record and the
assembly, environment and robot records it references hold every such fact; this brief
states none of them. Read them before planning.

You are given:

- The part geometry (STEP and meshes), the assembled poses, the mates with their
  seated poses and approach motions, the equivalence classes and each part's proved
  symmetries.
- The layout declaration, the target and its tolerances.
- The cell, the robot, the gripper (with its calibrated grasp geometry), the fixture, the
  frames and the observation streams.

You author:

- The order of the parts, grasps and regrasps, approach and insertion motions.
- Robot configurations and motions (the planning service plans collision-free paths
  against the cell's scene; direct motion is yours to make safe).
- How each insertion is physically carried out and checked.
- The planner's internal model and logic. Any library available at execution is fine;
  the delivery must not depend on authoring-only tools (`docs/delivery.md`).

## What completed means

The task target is `assembled_state`: every listed occurrence seated at its pose
relative to the fixture occurrence, within the task's per-part tolerances, with the
fixture undisturbed. `placement: pinned` on the fixture means the assembly is judged
where the fixture stands; the fixture does not move.

Apply the equivalences the task permits: identical parts in an `interchangeable` group
may fill each other's seats, one part per seat, and a part turned by one of its proved
symmetries is seated just the same.

Seated parts are held by contact only: nothing is welded or attached. Before returning
`done`, release every part, retreat, and make sure the parts stay where they are.

## Cell fixtures and task regions

The fixture (`base1`) is static at its layout pose and never moved by contact. Parts
start free on the table. The layout declaration says where each free part may start
(`docs/layouts.md`); `lab__reset_scene(seed)` gives you other starting arrangements the
task admits.

## A working method

This order is a method, not a requirement:

1. Read the records: the task, the assembly graph, the environment and the robot
   composite. Every number your planner needs is in them or is measured from the world.
2. Measure the physics you depend on rather than assuming it: how a part sits in the
   gripper after a grasp, how much play a slot has, how a part settles after release.
   The lab verbs (checkpoint, restore, set, contacts, rest) make such measurements cheap.
3. Make the planner read the current arrangement from the declared streams
   (`docs/layouts.md`), never from a layout file or a literal pose.
4. Exercise it from the starting states it will meet: the layout your session started in
   and other seeds.
5. Run the delivery the way it runs after the session with `evaluate_planner`, and read
   the evidence it returns.
