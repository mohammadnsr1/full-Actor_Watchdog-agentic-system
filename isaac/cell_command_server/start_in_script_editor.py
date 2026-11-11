# Paste into Isaac Sim's Script Editor (GUI workflow) with assembly_room.usd
# open. Starts the cell command server on the port in config/platforms/sim.yaml.
# Idempotent: running it twice keeps the first server.
import sys

REPO = "/home/mohammad/full-Actor_Watchdog-agentic-system"  # TODO(mohammad): adjust if the repo moves
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from isaac.cell_command_server.server import start_server  # noqa: E402

server = start_server()
print("cell command server:", server.host, server.port, "commands:", sorted(server.commands))
