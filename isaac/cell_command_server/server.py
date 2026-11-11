"""Cell command server: runs inside Kit, listens on loopback TCP.

Protocol: one connection per request; the client sends one JSON object
terminated by a newline, {"command": ..., "params": {...}}, and receives one
JSON object terminated by a newline, {"ok": bool, ...}. The socket thread only
queues requests; every command runs on Kit's main thread in the app update
callback, which is the only safe place for USD and physics calls.

Commands
    health                      playing, sim time advancing, stage, versions, intruders
    report_poses                world pose of every prim the stage publishes to TF (+ tcp)
    load_layout(assembly, layout, seed)   teleport parts to the seeded layout, then reset
    reset                       remove intruders and episode joints, home the arm, open
                                the gripper, zero part velocities (parts stay where they
                                are unless a layout was loaded)
    verify_reset(tolerances)    compare parts with the last loaded layout
    reload_stage                reopen the stage file from disk
    contacts_enable(enable)     contact reports on the finger pads; contacts_poll(since)
                                returns them for the ROS-side relay
    capture_layout(name)        write the current part poses as a layout file (helper)

Config: config/platforms/sim.yaml `scene` (port, prim paths, home joints,
gripper open position, contact pads, intruder root) read at start. The
registry is not read; prims come from the stage's own TF publisher target list.

Written from scratch. The loopback-command-server idea is credited to
DROIDSTACK in docs/PROVENANCE.md; no code from it was read for this file.
"""

from __future__ import annotations

import json
import os
import queue
import socket
import threading
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from isaac.cell_command_server import layouts as L
from isaac.common import kit_api as K

TF_PUBLISHER_NODE = "/Graph/Objects_TF/PublisherTF"


def wall_time_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


def repo_root() -> Path:
    env = os.environ.get("CELL_REPO_ROOT", "").strip()
    if env:
        return Path(env).expanduser()
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "config").is_dir() and (parent / "ros2_ws").is_dir():
            return parent
    raise RuntimeError("repo root not found; set CELL_REPO_ROOT")


def load_scene_config(platform: str = "sim") -> dict[str, Any]:
    path = repo_root() / "config" / "platforms" / f"{platform}.yaml"
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    scene = dict(data.get("scene") or {})
    scene["_platform_path"] = str(path)
    scene["_arm_joints"] = list((data.get("joints") or {}).get("arm") or [])
    scene["_gripper_master"] = str((data.get("joints") or {}).get("gripper_master") or "")
    scene["_gripper_open"] = (data.get("gripper") or {}).get("open_position")
    return scene


class CellCommandServer:
    def __init__(self, scene: dict[str, Any] | None = None) -> None:
        self.scene = scene or load_scene_config()
        self.host = str(self.scene.get("host", "127.0.0.1"))
        self.port = int(self.scene.get("port", 0) or 0)
        if self.port <= 0:
            raise RuntimeError("scene.port is not set in config/platforms/sim.yaml")
        self._queue: "queue.Queue[tuple[dict[str, Any], dict[str, Any], threading.Event]]" = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sub: Any = None
        self._sock: socket.socket | None = None
        self._last_sim_time = -1.0
        self._last_sim_time_wall = 0.0
        self._sim_advancing: bool | None = None
        self._layout: dict[str, Any] | None = None
        self._placements: dict[str, dict[str, Any]] | None = None
        self._contacts: list[dict[str, Any]] = []
        self._contact_seq = 0
        self._contact_sub: Any = None
        self._contacts_enabled = False
        self._articulation: Any = None
        self._started_wall = wall_time_iso()
        self.commands = {
            "health": self._cmd_health,
            "report_poses": self._cmd_report_poses,
            "load_layout": self._cmd_load_layout,
            "reset": self._cmd_reset,
            "verify_reset": self._cmd_verify_reset,
            "reload_stage": self._cmd_reload_stage,
            "contacts_enable": self._cmd_contacts_enable,
            "contacts_poll": self._cmd_contacts_poll,
            "capture_layout": self._cmd_capture_layout,
            "list_commands": lambda p: {"ok": True, "commands": sorted(self.commands)},
        }

    # -- lifecycle -----------------------------------------------------------------

    def start(self) -> "CellCommandServer":
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.listen(8)
        self._sock.settimeout(0.5)
        self._thread = threading.Thread(target=self._serve, name="cell_command_server", daemon=True)
        self._thread.start()
        self._sub = K.subscribe_update(self._on_update)
        print(f"[cell_command_server] listening on {self.host}:{self.port} (Kit {K.kit_version()})")
        return self

    def stop(self) -> None:
        self._stop.set()
        self._sub = None
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass

    # -- socket thread ---------------------------------------------------------------

    def _serve(self) -> None:
        assert self._sock is not None
        while not self._stop.is_set():
            try:
                conn, _addr = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._handle, args=(conn,), daemon=True).start()

    def _handle(self, conn: socket.socket) -> None:
        with conn:
            conn.settimeout(10.0)
            data = b""
            try:
                while b"\n" not in data:
                    chunk = conn.recv(65536)
                    if not chunk:
                        break
                    data += chunk
                line = data.split(b"\n", 1)[0]
                request = json.loads(line.decode("utf-8")) if line else {}
            except Exception as exc:  # noqa: BLE001
                self._reply(conn, {"ok": False, "error": f"bad request: {exc}"})
                return
            if not isinstance(request, dict) or not request.get("command"):
                self._reply(conn, {"ok": False, "error": "request needs a command"})
                return
            done = threading.Event()
            box: dict[str, Any] = {}
            self._queue.put((request, box, done))
            timeout = float(request.get("timeout_s", 60.0))
            if not done.wait(timeout):
                self._reply(conn, {"ok": False, "error": f"command {request['command']} did not finish in {timeout}s (is the app updating?)"})
                return
            self._reply(conn, box.get("response", {"ok": False, "error": "no response"}))

    @staticmethod
    def _reply(conn: socket.socket, response: dict[str, Any]) -> None:
        response.setdefault("wall_time_iso", wall_time_iso())
        try:
            conn.sendall((json.dumps(response) + "\n").encode("utf-8"))
        except OSError:
            pass

    # -- main thread -------------------------------------------------------------------

    def _on_update(self, _event: Any = None) -> None:
        now = time.monotonic()
        try:
            t = K.sim_time_s()
            if t != self._last_sim_time:
                self._sim_advancing = True
                self._last_sim_time = t
                self._last_sim_time_wall = now
            elif now - self._last_sim_time_wall > 2.0:
                self._sim_advancing = False
        except Exception:
            self._sim_advancing = None
        for _ in range(8):
            try:
                request, box, done = self._queue.get_nowait()
            except queue.Empty:
                return
            try:
                handler = self.commands.get(str(request["command"]))
                if handler is None:
                    box["response"] = {"ok": False, "error": f"unknown command {request['command']!r}", "commands": sorted(self.commands)}
                else:
                    box["response"] = handler(dict(request.get("params") or {}))
            except Exception as exc:  # noqa: BLE001
                box["response"] = {"ok": False, "error": f"{type(exc).__name__}: {exc}", "traceback": traceback.format_exc()[-2000:]}
            finally:
                done.set()

    # -- helpers ---------------------------------------------------------------------------

    def _tf_prims(self) -> list[str]:
        return K.relationship_targets(TF_PUBLISHER_NODE, "inputs:targetPrims")

    def _intruders(self) -> list[str]:
        root = str(self.scene.get("intruder_root", "") or "")
        return K.children(root) if root and K.prim_exists(root) else []

    def _articulation_handle(self) -> Any:
        if self._articulation is None:
            root = str(self.scene.get("robot_prim", "") or "") or K.articulation_root_path()
            self._articulation = K.ArticulationHandle(root)
        return self._articulation

    # -- commands ---------------------------------------------------------------------------

    def _cmd_health(self, _p: dict[str, Any]) -> dict[str, Any]:
        intruders = self._intruders()
        return {
            "ok": K.is_playing() and bool(self._sim_advancing),
            "playing": K.is_playing(),
            "sim_time_s": K.sim_time_s(),
            "sim_advancing": self._sim_advancing,
            "stage": K.stage_path(),
            "version": K.kit_version(),
            "tf_prims": len(self._tf_prims()),
            "intruders": intruders,
            "episode_joints": K.episode_joints(),
            "layout_loaded": (self._layout or {}).get("name"),
            "contacts_enabled": self._contacts_enabled,
            "server_started_wall_time_iso": self._started_wall,
        }

    def _cmd_report_poses(self, _p: dict[str, Any]) -> dict[str, Any]:
        poses: dict[str, dict[str, Any]] = {}
        for path in self._tf_prims():
            try:
                poses[path.rsplit("/", 1)[-1]] = {"prim": path, **K.world_pose(path)}
            except Exception as exc:  # noqa: BLE001
                poses[path.rsplit("/", 1)[-1]] = {"prim": path, "error": str(exc)}
        tcp = str(self.scene.get("tcp_prim", "") or "")
        if tcp and K.prim_exists(tcp):
            poses["tcp"] = {"prim": tcp, **K.world_pose(tcp)}
        return {"ok": True, "frame": "world", "sim_time_s": K.sim_time_s(), "poses": poses}

    def _cmd_load_layout(self, p: dict[str, Any]) -> dict[str, Any]:
        assembly = str(p.get("assembly", "") or "")
        name = str(p.get("layout", "") or "")
        seed = int(p.get("seed", 0) or 0)
        path = repo_root() / "config" / "layouts" / assembly / f"{name}.yaml"
        if not path.is_file():
            return {"ok": False, "error": f"layout not found: {path}"}
        with path.open("r", encoding="utf-8") as handle:
            layout = yaml.safe_load(handle) or {}
        problems = L.validate_layout(layout)
        if problems:
            return {"ok": False, "error": "layout invalid", "problems": problems, "path": str(path)}
        placements = L.seeded_placements(layout, seed)
        missing = [pid for pid, pl in placements.items() if not K.prim_exists(pl["prim"])]
        if missing:
            return {"ok": False, "error": f"layout names prims not on the stage: {missing}"}
        for pid, pl in placements.items():
            K.teleport(pl["prim"], pl["position"], pl["orientation_xyzw"])
        self._layout = {"name": name, "assembly": assembly, "seed": seed, "path": str(path), "raw": layout}
        self._placements = placements
        reset = self._cmd_reset({"keep_parts": True})
        return {"ok": bool(reset.get("ok")), "layout": name, "seed": seed, "placed": sorted(placements), "jittered": [k for k, v in placements.items() if v["jittered"]], "reset": reset}

    def _cmd_reset(self, p: dict[str, Any]) -> dict[str, Any]:
        report: dict[str, Any] = {"removed_intruders": [], "removed_episode_joints": [], "homed": False, "gripper_opened": False, "zeroed": []}
        for path in self._intruders():
            if K.remove_prim(path):
                report["removed_intruders"].append(path)
        for path in K.episode_joints():
            if K.remove_prim(path):
                report["removed_episode_joints"].append(path)
        if not p.get("keep_parts") and self._placements:
            for pid, pl in self._placements.items():
                K.teleport(pl["prim"], pl["position"], pl["orientation_xyzw"])
        for path in self._tf_prims():
            try:
                K.zero_velocities(path)
                report["zeroed"].append(path.rsplit("/", 1)[-1])
            except Exception:
                pass
        errors: list[str] = []
        if not K.is_playing():
            errors.append("timeline is not playing: joints cannot be set (press Play / run headless)")
        else:
            home = dict(self.scene.get("home_joint_positions") or {})
            if not home:
                errors.append("scene.home_joint_positions is empty")
            else:
                try:
                    missing = self._articulation_handle().set_positions({k: float(v) for k, v in home.items()})
                    report["homed"] = True
                    if missing:
                        report["home_joints_missing"] = missing
                except Exception as exc:  # noqa: BLE001
                    errors.append(f"homing failed: {exc}")
            master = str(self.scene.get("_gripper_master", "") or "")
            open_pos = self.scene.get("_gripper_open")
            if master and open_pos is not None:
                try:
                    self._articulation_handle().set_positions({master: float(open_pos)})
                    report["gripper_opened"] = True
                except Exception as exc:  # noqa: BLE001
                    errors.append(f"gripper open failed: {exc}")
            else:
                errors.append("gripper master joint or open position missing from the platform config")
        report["ok"] = not errors
        if errors:
            report["errors"] = errors
        report["sim_time_s"] = K.sim_time_s()
        return report

    def _cmd_verify_reset(self, p: dict[str, Any]) -> dict[str, Any]:
        if not self._placements:
            return {"ok": False, "error": "no layout loaded; nothing to verify against"}
        tol = dict(p.get("tolerances") or (self._layout or {}).get("raw", {}).get("verify_tolerances") or {})
        pos_tol = tol.get("position_m")
        ori_tol = tol.get("orientation_deg")
        if not pos_tol:
            return {"ok": False, "error": "tolerances.position_m is required (TODO(mohammad) in the layout file)"}
        rows: dict[str, Any] = {}
        ok = True
        for pid, pl in self._placements.items():
            try:
                actual = K.world_pose(pl["prim"])
            except Exception as exc:  # noqa: BLE001
                rows[pid] = {"status": "missing", "error": str(exc)}
                ok = False
                continue
            dp, dq = L.pose_error(pl, actual)
            passed = dp <= float(pos_tol) and (ori_tol is None or dq <= float(ori_tol))
            rows[pid] = {"status": "pass" if passed else "fail", "position_error_m": dp, "orientation_error_deg": dq}
            ok = ok and passed
        speeds = {}
        try:
            speeds = self._articulation_handle().speeds()
        except Exception:
            pass
        return {"ok": ok, "parts": rows, "tolerances": tol, "intruders": self._intruders(), "episode_joints": K.episode_joints(), "joint_speeds": speeds}

    def _cmd_reload_stage(self, _p: dict[str, Any]) -> dict[str, Any]:
        path = K.stage_path()
        self._articulation = None
        self._contacts_enabled = False
        self._contact_sub = None
        ok = K.reopen_stage()
        if ok and bool(self.scene.get("play_after_reload", True)):
            K.play()
        return {"ok": ok, "stage": path, "playing": K.is_playing()}

    def _cmd_contacts_enable(self, p: dict[str, Any]) -> dict[str, Any]:
        enable = bool(p.get("enable", True))
        pads = [str(x) for x in (self.scene.get("contact_pads") or [])]
        if not enable:
            self._contact_sub = None
            self._contacts_enabled = False
            return {"ok": True, "enabled": False}
        if not pads:
            return {"ok": False, "error": "scene.contact_pads is empty (TODO(mohammad))"}
        applied = K.enable_contact_report(pads)
        if not applied:
            return {"ok": False, "error": f"none of the contact pads exist on the stage: {pads}"}

        def _on_contacts(events: list[dict[str, Any]]) -> None:
            t = K.sim_time_s()
            stamp = wall_time_iso()
            for e in events:
                self._contact_seq += 1
                e.update({"seq": self._contact_seq, "sim_time_s": t, "wall_time_iso": stamp})
                self._contacts.append(e)
            del self._contacts[:-2000]

        self._contact_sub = K.subscribe_contact_reports(_on_contacts)
        self._contacts_enabled = True
        return {"ok": True, "enabled": True, "pads": applied}

    def _cmd_contacts_poll(self, p: dict[str, Any]) -> dict[str, Any]:
        since = int(p.get("since_seq", 0) or 0)
        events = [e for e in self._contacts if e["seq"] > since]
        return {"ok": True, "enabled": self._contacts_enabled, "events": events[-500:], "last_seq": self._contact_seq}

    def _cmd_capture_layout(self, p: dict[str, Any]) -> dict[str, Any]:
        """Write the current poses of the stage's parts as a layout file."""
        assembly = str(p.get("assembly", "") or "")
        name = str(p.get("name", "") or "captured")
        if not assembly:
            return {"ok": False, "error": "assembly is required"}
        poses = self._cmd_report_poses({})["poses"]
        parts: dict[str, Any] = {}
        for pid, pose in poses.items():
            if "error" in pose or pid == "tcp":
                continue
            if "/" in pose["prim"].rsplit("/World/", 1)[-1]:
                continue  # frames under fixtures are not placed parts
            parts[pid] = {"prim": pose["prim"], "position": pose["position"], "orientation_xyzw": pose["orientation_xyzw"], "jitter": {"xy_m": 0.0, "yaw_deg": 0.0}}
        layout = {
            "assembly": assembly, "name": name, "frame": "world",
            "captured_wall_time_iso": wall_time_iso(), "stage": K.stage_path(),
            "verify_tolerances": {"position_m": None, "orientation_deg": None},  # TODO(mohammad)
            "parts": parts,
            "robot": {"home_joint_positions": dict(self.scene.get("home_joint_positions") or {}), "gripper_open_position": self.scene.get("_gripper_open")},
        }
        out = repo_root() / "config" / "layouts" / assembly / f"{name}.yaml"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text("# Captured by cell_command_server capture_layout; fill verify_tolerances and jitter.\n" + yaml.safe_dump(layout, sort_keys=False), encoding="utf-8")
        return {"ok": True, "path": str(out), "parts": sorted(parts)}


_SERVER: CellCommandServer | None = None


def start_server() -> CellCommandServer:
    """Idempotent module-level starter (Script Editor and headless both use it)."""
    global _SERVER
    if _SERVER is None:
        _SERVER = CellCommandServer().start()
    return _SERVER
