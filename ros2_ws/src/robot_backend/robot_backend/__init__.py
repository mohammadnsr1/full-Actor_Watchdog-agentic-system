"""WatchDog side of the cell, plus a few shared pieces.

`watchdog`      the WatchDog session: recorder, monitors, engine, halt, rulings,
                requirements, session directory.
`mcp_servers`   watchdog_mcp_server (the WatchDog's only MCP server) and
                role_context (rendered task, call logging).
`contracts`, `registry`, `events`, `vision`   loaded by the WatchDog server.
`nodes`         scene_tf_sync_node: /sync_scene_objects for the planning service
                and the /get_object_pose(s) queries (skills/pose_query_skills).
`utils`         geometry and the RG2 width table (also used by the executor's
                planner templates and tools/pick_check.py).

The v1 executor stack (motion/gripper/scene/executor-monitor MCP servers,
skills, ros_moveit_backend, interfaces, cell/gate) was removed in Milestone 7:
the executor is now a planner (docs/PLANNER_CONTRACT.md) over
cell_platform_interfaces, cell_backend_<platform> and cell_planning_service.
"""
