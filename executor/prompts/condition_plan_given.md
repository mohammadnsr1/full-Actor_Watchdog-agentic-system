## Your condition: plan given

The task file gives you the goal and constraints (`instruction_spec`), the
objects, and a plan: `steps`, each with its grasp centre offset, approach
distance, target offsets, grasp width and motion scaling. Your starting
planner (`executor/planner_templates/plan_given`) executes those steps as
written. The plan is a starting point, not a guarantee: it may be wrong or
imprecise, and the episodes will show where. You may change how the planner
uses it, correct its numbers in your planner, or depart from it entirely.
