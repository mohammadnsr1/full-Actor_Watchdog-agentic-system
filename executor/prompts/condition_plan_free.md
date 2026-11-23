## Your condition: plan free

The task file gives you the goal, the constraints and the objects, and no
plan: no sequence of motions, no grasp poses, no approach or insertion
offsets. Working those out is your job: which part when, where to grasp it
and how wide, how to approach its placement, how to insert it, how to
retreat. Your starting planner (`executor/planner_templates/plan_free`) is a
skeleton: it moves to the safe pose and stops. Its docstring lists what you
have: part and placement-frame poses from the executor camera, free-space and
straight-line planning, the arm and the gripper, and geometry helpers for the
gripper (`geometry.py`: the TCP pose that puts the fingers' grasp centre at a
point with the jaws a given width apart). `config/parts/executor_parts.yaml`
gives each part's collision shape, dimensions and grasp force.
