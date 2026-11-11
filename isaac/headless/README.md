# Headless Isaac

```bash
cd ~/full-Actor_Watchdog-agentic-system
~/isaacsim/python.sh isaac/headless/run_headless.py            # headless, plays, command server up
~/isaacsim/python.sh isaac/headless/run_headless.py --gui      # same with the window, for debugging
```

Then the usual `ros2 launch sim_moveit_bringup sim_moveit_bringup.launch.py`.

The command server port is `config/platforms/sim.yaml: scene.port`. Stop with
Ctrl-C; the supervisor (Milestone 8) restarts it. Not yet run on this machine:
first run is Mohammad's check (see docs/NEXT_STEPS.md).
