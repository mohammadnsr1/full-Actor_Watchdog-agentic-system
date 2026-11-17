# Headless Isaac

```bash
cd ~/full-Actor_Watchdog-agentic-system
isaac/headless/run_headless.sh            # headless, plays, command server up
isaac/headless/run_headless.sh --gui      # same with the window, for debugging
```

Run it from a shell without `/opt/ros/jazzy/setup.bash` sourced (like the GUI):
the wrapper sources `~/isaacsim/setup_ros_env.sh`, which adds the bridge's
internal Jazzy libraries; bare `python.sh run_headless.py` fails to load them.

Then the usual `ros2 launch sim_moveit_bringup sim_moveit_bringup.launch.py`.

The command server port is `config/platforms/sim.yaml: scene.port`. Stop with
Ctrl-C; the supervisor (Milestone 8) restarts it. Not yet run on this machine:
first run is Mohammad's check (see docs/NEXT_STEPS.md).
