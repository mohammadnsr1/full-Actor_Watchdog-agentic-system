"""Backend-neutral planning service for the cell.

Layout:
    failure_codes.py    the failure vocabulary every backend maps onto
    cell_config.py      locate and read config/cell.yaml (planner section)
    timing.py           conservative time parameterization for joint paths
    backend.py          PlanningBackend contract and result types
    service_server.py   ROS 2 services -> backend calls, report stamping
    moveit_backend.py   MoveIt plan-only backend (GetMotionPlan, GetCartesianPath,
                        GetStateValidity, ApplyPlanningScene)
    node.py             the MoveIt-backed node entry point

The cuRobo backend lives in the sibling package cell_planning_curobo and
reuses backend.py, service_server.py, timing.py and failure_codes.py.
"""
