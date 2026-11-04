"""cuRobo backend for the cell planning service.

    venv.py             re-exec into the cuRobo Python named in config/cell.yaml
    robot_config.py     build the cuRobo robot config from URDF + XRDF + RG2 spheres
    world.py            explicit collision-object world (cuboids and meshes)
    curobo_backend.py   PlanningBackend implementation on cuRobo v0.7.8
    depth_obstacles.py  optional depth -> voxel world layer (experimental)
    node.py             the executable

Everything that touches the cuRobo library is imported lazily inside the
backend, so this package builds and its pure-Python parts are testable
without cuRobo installed.
"""
