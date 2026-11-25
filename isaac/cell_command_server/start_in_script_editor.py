# Paste into Isaac Sim's Script Editor (GUI workflow) with assembly_room.usd
# open. Starts the cell command server on the port in config/platforms/sim.yaml.
# Running it again stops the running server, reloads the server code from disk
# and starts a fresh one (a loaded layout and contact subscription are dropped:
# load_layout again and restart pad_contacts_relay).
import importlib
import sys

REPO = "/home/mohammad/full-Actor_Watchdog-agentic-system"  # TODO(mohammad): adjust if the repo moves
if REPO not in sys.path:
    sys.path.insert(0, REPO)

_old = sys.modules.get("isaac.cell_command_server.server")
if _old is not None and getattr(_old, "_SERVER", None) is not None:
    _old._SERVER.stop()
    _old._SERVER = None
for _name in ("isaac.common.kit_api", "isaac.cell_command_server.contacts", "isaac.cell_command_server.layouts", "isaac.cell_command_server.lab", "isaac.cell_command_server.server"):
    if _name in sys.modules:
        importlib.reload(sys.modules[_name])

from isaac.cell_command_server.server import start_server  # noqa: E402

server = start_server()
print("cell command server:", server.host, server.port, "commands:", sorted(server.commands))
