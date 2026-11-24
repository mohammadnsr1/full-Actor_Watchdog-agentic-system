## Your condition: plan free

The task file gives you the goal, the constraints and the objects, and no
plan: no sequence of motions, no grasp poses, no approach or insertion
offsets, and no placement poses (none are published). Working those out is
your job: where each part goes on the fixture (from the pictures of the
finished assembly, the part shapes and what your camera sees), which part
when, where to grasp it and how wide, how to approach its place, how to
insert it, how to retreat. Your starting planner
(`executor/planner_templates/plan_free`) is a skeleton: it moves to the safe
pose and stops. Its docstring lists what you have: part and fixture poses and
the wrist camera's RGB-D images, free-space and straight-line planning, the
arm and the gripper, and geometry helpers for the gripper (`geometry.py`: the
TCP pose that puts the fingers' grasp centre at a point with the jaws a given
width apart). `config/parts/executor_parts.yaml` gives each part's collision
shape, dimensions and grasp force.
