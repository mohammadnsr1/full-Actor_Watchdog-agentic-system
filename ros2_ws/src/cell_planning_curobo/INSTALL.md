# cell_planning_curobo: installation

Nothing here is installed by the build. The cuRobo backend runs in its own
Python environment with PyTorch and CUDA; the ROS node re-execs into that
interpreter at start (`venv.py`) and appends the system `dist-packages` so
rclpy and the message packages stay importable.

## 1. Environment

cuRobo `v0.7.8` is pinned (tag, not branch). It needs a CUDA toolkit that
matches the PyTorch build, and `ninja` for the extension build.

```bash
python3 -m venv ~/.venvs/curobo            # system Python 3.12 (same as ROS Jazzy)
. ~/.venvs/curobo/bin/activate
pip install --upgrade pip wheel ninja
# PyTorch with CUDA. Pick the index that matches your driver; check with nvidia-smi.
# TODO(mohammad): choose the torch/CUDA pair, e.g.
#   pip install torch --index-url https://download.pytorch.org/whl/cu121
pip install torch
# cuRobo from source at the pinned tag (the wheel builds CUDA kernels; takes minutes).
git clone --branch v0.7.8 --depth 1 https://github.com/NVlabs/curobo.git ~/src/curobo
pip install -e ~/src/curobo --no-build-isolation
# Also needed by this backend:
pip install pyyaml numpy
deactivate
```

Check:

```bash
~/.venvs/curobo/bin/python -c "import torch, curobo; print(torch.__version__, torch.cuda.is_available(), curobo.__version__)"
```

`torch.cuda.is_available()` must print `True`; the backend refuses to start
otherwise.

## 2. Tell the cell where the interpreter is

In `config/cell.yaml`:

```yaml
planner:
  backend: curobo
  curobo:
    python: ~/.venvs/curobo/bin/python
```

or export `CELL_CUROBO_PYTHON=~/.venvs/curobo/bin/python`.

## 3. Robot description

- Arm collision spheres come from `config/robot/ur5e.xrdf` (34 spheres over
  `base_link` through `tcp`), converted at start with cuRobo's
  `convert_xrdf_to_curobo`.
- RG2 spheres: fill `config/robot/rg2_collision_spheres.yaml`. The file
  explains two ways to generate them. The backend refuses to start while any
  RG2 link has no spheres.
- The URDF is `ros2_ws/src/robot_description/urdf/ur5e_ros.urdf`. Meshes are
  not loaded (collision is sphere-based), so `package://` URIs are fine.

## 4. Run

Set `planner.backend: curobo`, then bring the cell up as usual; the bringup
includes `cell_planning_service/launch/planning_service.launch.py`, which
starts `curobo_planning_node` instead of the MoveIt node. move_group still
runs, because `sync_collision_objects` reads MoveIt's planning scene back to
mirror synced registry objects (see `curobo_backend.py`).

Standalone:

```bash
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
ros2 run cell_planning_curobo curobo_planning_node
```

## 5. Depth obstacles (experimental, off by default)

`planner.depth_obstacles: true` turns on `depth_obstacles.py`: the scene
camera's depth image is back-projected through TF into a voxel grid over a
fixed box and handed to cuRobo as a `VoxelGrid` obstacle. Every parameter
under `planner.curobo.depth` is a `TODO(mohammad)`; the layer refuses to
start while they are zero. Known limitations are in the module docstring:
no temporal filtering, the robot is masked crudely, and the voxel feature
convention must be validated against v0.7.8 before plans that depend on it
are trusted. MoveIt's point-cloud path failed in this cell before and
cuRobo's nvblox examples have known issues, which is why this stays behind a
flag.

## 6. Comparing backends

Every service response carries `report.backend`, `report.planning_time_s`
and `report.failure_code`. The episode record (Milestone 6) logs them, so
MoveIt and cuRobo runs can be compared on the same requests.
