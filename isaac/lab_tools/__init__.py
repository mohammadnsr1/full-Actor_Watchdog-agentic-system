"""lab_tools: the lab verbs on the omni.kit.mcp bridge (EB3), namespace `lab`.

The agent sees them as `lab__<verb>` through the kit-mcp gateway. Each verb is
the cell command server's `lab_<verb>` command (isaac/cell_command_server/lab.py)
called in-process, so the bridge tool and the TCP 9010 command are one
implementation; two verbs that wait across app updates are written here:
`camera_capture` and `until_at_rest`.

Register once (Isaac down, see omni-kit-mcp README):

    python3 ~/omni-kit-mcp/scripts/install.py add-project \\
        --path /home/mohammad/full-Actor_Watchdog-agentic-system/isaac --module lab_tools

The cell command server must be running (start_in_script_editor.py or headless);
a verb called before it is up returns an error saying so.
"""

from __future__ import annotations

import sys
from pathlib import Path

MCP_NAMESPACE = "lab"
REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

OBJ = {"type": "string", "description": "object id as report_poses names it (u_brown, base1, ...), 'tcp', or a prim path"}
POSE = {"type": "object", "description": "{position: [x,y,z], orientation_xyzw: [x,y,z,w]} in world, metres"}


def _server():
    from isaac.cell_command_server import server as S

    if S._SERVER is None:
        raise RuntimeError("the cell command server is not running (isaac/cell_command_server/start_in_script_editor.py)")
    return S._SERVER


def _call(command: str, params: dict) -> dict:
    out = _server().commands[command]({k: v for k, v in params.items() if v is not None})
    if not out.get("ok", False) and "error" in out:
        raise RuntimeError(out["error"])
    return out


CAMERAS = {"camera1": "/World/ur5e/wrist_3_link/camera1", "camera2": "/World/Camera"}


def register(registrar):  # noqa: C901 - one block per verb
    @registrar.tool("World pose of an object, frame or prim; with relative_to, its pose in that frame.",
                    {"ref": {**OBJ, "required": True}, "relative_to": {**OBJ, "description": "optional: express the pose in this frame"}})
    def resolve(ref: str, relative_to: str | None = None):
        return _call("lab_resolve", {"ref": ref, "relative_to": relative_to})

    @registrar.tool("Snapshot every tracked object's pose (and kinematic flag) and all robot joint positions under a name.",
                    {"name": {"type": "string", "description": "checkpoint name (default cpN)"}})
    def checkpoint(name: str | None = None):
        return _call("lab_checkpoint", {"name": name})

    @registrar.tool("Put a checkpoint back: objects teleported with zero velocity, joints set. Publishes a lab event.",
                    {"checkpoint": {"type": "string", "required": True}})
    def restore(checkpoint: str):
        return _call("lab_restore", {"checkpoint": checkpoint})

    @registrar.tool("Teleport objects, switch objects kinematic/dynamic, set robot joints (rad) and/or the gripper "
                    "master joint (rad). Publishes a lab event.",
                    {"objects": {"type": "object", "description": "{object id: pose}", "additionalProperties": POSE},
                     "dynamics": {"type": "object", "description": "{object id: 'kinematic' | 'dynamic'}"},
                     "robot": {"type": "object", "description": "{joint name: radians}"},
                     "gripper": {"type": "number", "description": "gripper master joint, radians"}})
    def set(objects: dict | None = None, dynamics: dict | None = None, robot: dict | None = None, gripper: float | None = None):
        return _call("lab_set", {"objects": objects, "dynamics": dynamics, "robot": robot, "gripper": gripper})

    @registrar.tool("Reset the scene to the task's layout with a seed (seed 0 = nominal), arm home, gripper open, "
                    "then verify. Publishes a lab event.",
                    {"seed": {"type": "integer", "default": 0}, "layout": {"type": "string", "default": "nominal"},
                     "assembly": {"type": "string", "default": "fmb_assembly_2"}})
    def reset_scene(seed: int = 0, layout: str = "nominal", assembly: str = "fmb_assembly_2"):
        return _call("lab_reset_scene", {"seed": seed, "layout": layout, "assembly": assembly})

    @registrar.tool("Are the objects at rest now? Max speeds over the last window of sim time "
                    "(default: 1 s, 2 mm/s, 0.02 rad/s, 5 samples; all dynamic objects).",
                    {"objects": {"type": "array", "items": {"type": "string"}}, "window_s": {"type": "number"},
                     "linear_m_s": {"type": "number"}, "angular_rad_s": {"type": "number"}})
    def is_at_rest(objects: list | None = None, window_s: float | None = None, linear_m_s: float | None = None,
                   angular_rad_s: float | None = None):
        return _call("lab_rest", {"objects": objects, "window_s": window_s, "linear_m_s": linear_m_s, "angular_rad_s": angular_rad_s})

    @registrar.tool("Wait (sim time keeps running) until the objects are at rest or budget_s of sim time passes.",
                    {"budget_s": {"type": "number", "default": 5.0}, "objects": {"type": "array", "items": {"type": "string"}},
                     "window_s": {"type": "number"}, "linear_m_s": {"type": "number"}, "angular_rad_s": {"type": "number"}})
    async def until_at_rest(budget_s: float = 5.0, objects: list | None = None, window_s: float | None = None,
                            linear_m_s: float | None = None, angular_rad_s: float | None = None):
        import omni.kit.app

        from isaac.common import kit_api as K

        params = {"objects": objects, "window_s": window_s, "linear_m_s": linear_m_s, "angular_rad_s": angular_rad_s}
        start = K.sim_time_s()
        last = None
        for _ in range(100000):
            last = _call("lab_rest", params)
            waited = K.sim_time_s() - start
            if last["at_rest"] or waited >= budget_s:
                return {**last, "waited_sim_s": waited, "timed_out": not last["at_rest"]}
            if not K.is_playing():
                return {**last, "waited_sim_s": waited, "timed_out": True, "reason": "timeline is not playing"}
            await omni.kit.app.get_app().next_update_async()
        return {**(last or {}), "timed_out": True}

    @registrar.tool("Start contact reports on bodies (object ids or prim paths); read them with contacts_read.",
                    {"bodies": {"type": "array", "items": OBJ, "required": True}})
    def contacts_arm(bodies: list):
        return _call("lab_contacts_arm", {"bodies": bodies})

    @registrar.tool("Contact events (made/lost, bodies, impulse, point, sim time) since a sequence number, "
                    "and the pairs touching now.", {"since_seq": {"type": "integer", "default": 0}})
    def contacts_read(since_seq: int = 0):
        return _call("lab_contacts_read", {"since_seq": since_seq})

    @registrar.tool("The simulator's side of robot health: playing, sim time advancing, joint positions. "
                    "The controllers' side is the platform MCP's robot_health.", {})
    def robot_health():
        return _call("lab_robot_health", {})

    @registrar.tool("Judge the live scene with the episode checker's rule: each part at its seat within its band, "
                    "modulo symmetry, the U pair interchangeable.", {"assembly": {"type": "string", "default": "fmb_assembly_2"}})
    def verify_assembly(assembly: str = "fmb_assembly_2"):
        return _call("lab_verify_assembly", {"assembly": assembly})

    @registrar.tool("Create or move a lab camera named `name` at eye, looking at target (world, metres). "
                    "Never saved into the stage file.",
                    {"name": {"type": "string", "default": "view"}, "eye": {"type": "array", "items": {"type": "number"}, "required": True},
                     "target": {"type": "array", "items": {"type": "number"}, "required": True}, "focal_length_mm": {"type": "number", "default": 18.0}})
    def camera_view(eye: list, target: list, name: str = "view", focal_length_mm: float = 18.0):
        return _call("lab_camera_view", {"name": name, "eye": eye, "target": target, "focal_length_mm": focal_length_mm})

    @registrar.tool("Render one RGB image from camera1 (wrist), camera2 (table), a lab camera name or a camera prim; "
                    "saved as PNG on the cell host (fetch it with get_file). Does not step physics.",
                    {"camera": {"type": "string", "default": "camera2"}, "width": {"type": "integer", "default": 640},
                     "height": {"type": "integer", "default": 480}})
    async def camera_capture(camera: str = "camera2", width: int = 640, height: int = 480):
        import time

        import numpy as np
        import omni.kit.app
        import omni.replicator.core as rep
        from PIL import Image

        from isaac.cell_command_server.lab import LAB_CAMERA_ROOT
        from isaac.common import kit_api as K

        path = CAMERAS.get(camera) or (camera if camera.startswith("/") else f"{LAB_CAMERA_ROOT}/{camera}")
        if not K.prim_exists(path):
            raise RuntimeError(f"no camera {camera!r} ({path})")
        rp = rep.create.render_product(path, (int(width), int(height)))
        ann = rep.AnnotatorRegistry.get_annotator("rgb")
        ann.attach([rp])
        try:
            data = None
            for _ in range(60):
                await omni.kit.app.get_app().next_update_async()
                d = ann.get_data()
                if d is not None and getattr(d, "size", 0) and np.asarray(d).max() > 0:
                    data = np.asarray(d)
                    break
            if data is None:
                raise RuntimeError("the camera produced no image within 60 app updates")
        finally:
            ann.detach()
            rp.destroy()
        out = REPO / "records" / "lab" / "captures" / f"{time.strftime('%Y%m%dT%H%M%S')}_{camera.strip('/').replace('/', '_')}.png"
        out.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(data[..., :3].astype(np.uint8)).save(out)
        return {"path": str(out), "camera": path, "width": int(data.shape[1]), "height": int(data.shape[0]),
                "sim_time_s": K.sim_time_s()}

    @registrar.tool("Lab events (reset, restore, set) since a sequence number.", {"since_seq": {"type": "integer", "default": 0}})
    def events(since_seq: int = 0):
        return _call("lab_events_poll", {"since_seq": since_seq})
