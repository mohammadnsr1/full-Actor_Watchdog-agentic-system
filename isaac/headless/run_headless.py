"""Headless Isaac Sim for the 24/7 loop.

    isaac/headless/run_headless.sh [--stage PATH] [--gui]
    (the wrapper sources ~/isaacsim/setup_ros_env.sh like isaac-sim.sh does,
    then runs this file with ~/isaacsim/python.sh)

Opens assembly_room.usd, enables the ROS 2 bridge, starts the cell command
server, presses Play, and keeps the app updating until SIGINT. The GUI
workflow (open the stage by hand, press Play, paste
cell_command_server/start_in_script_editor.py) keeps working; this script is
the unattended alternative, not a replacement.

Notes for Mohammad:
  * ROS 2 bridge: launched bare through python.sh, the bridge's internal Jazzy
    fails (libament_index_cpp.so not found, 2026-11-17) because python.sh does
    not source setup_ros_env.sh. run_headless.sh does, matching the GUI.
  * Isaac 5.1: `from isaacsim.simulation_app import SimulationApp`. On 6.x the
    same import exists; if it moves, this is the only place to change.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", default=str(REPO / "assets" / "usd" / "assembly_room.usd"))
    parser.add_argument("--gui", action="store_true", help="run with the window (debugging)")
    parser.add_argument("--no-play", action="store_true")
    args = parser.parse_args()

    try:
        from isaacsim.simulation_app import SimulationApp
    except ImportError:  # older layout
        from isaacsim import SimulationApp  # type: ignore

    app = SimulationApp({"headless": not args.gui, "hide_ui": not args.gui})
    try:
        from isaac.common import kit_api as K

        if not K.enable_extension("isaacsim.ros2.bridge"):
            print("[headless] WARNING: isaacsim.ros2.bridge did not enable", file=sys.stderr)
        for _ in range(5):
            app.update()
        if not K.open_stage(args.stage):
            print(f"[headless] could not open {args.stage}", file=sys.stderr)
            return 2
        for _ in range(10):
            app.update()
        from isaac.cell_command_server.server import start_server

        server = start_server()
        if not args.no_play:
            K.play()
        print(f"[headless] stage {args.stage} playing={K.is_playing()} server {server.host}:{server.port} version {K.kit_version()}")

        stop = {"now": False}

        def _sig(*_a):
            stop["now"] = True

        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)
        while app.is_running() and not stop["now"]:
            app.update()
        server.stop()
        return 0
    except BaseException:
        # Kit routes Python stderr into its log as "[Error] [py stderr]" lines and
        # app.close() exits with status 0, so make a failure impossible to miss.
        import traceback

        print("[headless] FAILED:\n" + traceback.format_exc(), file=sys.stderr, flush=True)
        return 1
    finally:
        app.close()


if __name__ == "__main__":
    sys.exit(main())
