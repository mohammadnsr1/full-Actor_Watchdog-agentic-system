# The task data

Everything about the task is a file in one read-only tree, the **task data**: `/data` in
this container, `CELL_TASK_DATA` for your planner (the same files on the cell host).
`task_ref`, the argument of `execute_task`, is the task's id: its folder under `tasks/`.
`STAGED.json` at the root lists what was staged, with digests.

| path | what it holds |
|---|---|
| `tasks/<id>/task.yaml` | what is asked: target, equivalences, tolerances, layouts, binding |
| `tasks/<id>/layouts/*.json` | the fixed layout the task's starting arrangements are drawn around |
| `environments/<name>/declaration.yaml` | the cell: contents, frames, streams (topics), physics |
| `robots/<name>/composite.yaml` | the robot: joints, frames, home pose, controllers, planning service, gripper |
| `assemblies/<name>/` | the assembly: CAD, graphs, simulation bundles (`docs/assembly-graph.md`) |
| `prompts/docs/` | these documents |

Inside an assembly:

| path | what it is |
|---|---|
| `sources/*.step` | the CAD originals, per part, exact geometry |
| `artifacts/<name>.assembly-graph.json` | occurrences, assembled poses, mates, classes |
| `artifacts/<Part>.part-graph.json` | one part's faces, edges, frames and proved symmetry certificate |
| `artifacts/<Part>.collider-policy.json` | how the part's collision boxes were authored |
| `bundles/<Part>.usda` | the simulation asset each part on the stage is built from (mesh, box colliders, mass, material) |

The records are this cell's, in the shapes of the prompt-to-assembly records (each says
`subset_of`); fields the cell does not use are absent.

## Names

The task names its assembly and environment; the environment names its robot. Resolve a
name to its folder here. A name that does not resolve is a stop-and-report: there is no
second copy to fall back to.

## The task record

| block | what it declares |
|---|---|
| `assembly`, `environment`, `roles` | names; the manipulator role and its grasp mode (`friction`: parts are held by the fingers only) |
| `goal.target` | `assembled_state` over the listed occurrences, `placement: pinned` on the fixture occurrence |
| `goal.equivalence` | `interchangeable` groups of identical occurrences; symmetric orientations are accepted by default |
| `mate_realization` | every mate is physical only: nothing is ever welded |
| `tolerances` | the referee's band per part (position m, orientation deg) and for the fixture staying put |
| `layouts` | the fixed layout, the static fixture, the free parts and their seeded jitter (`docs/layouts.md`) |
| `streams.object_frames` | the object ids published on the pose channel |
| `binding.objects` | per occurrence: the part label, the cell's object id (`pose_id`), its prim, and `prim_from_object`, the transform from the graph's object frame to the object's pose-channel frame |
