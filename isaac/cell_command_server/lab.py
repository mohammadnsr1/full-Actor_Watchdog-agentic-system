"""Lab verbs (EB3): what the authoring agent may do to the simulated cell between
episodes, run inside Kit by the cell command server.

Each verb is a command server command named `lab_<verb>` (TCP 9010, for host
code) and a bridge tool `lab.<verb>` (isaac/lab_tools, reaching the agent as
`lab__<verb>`). One implementation: the bridge tool calls the server's command.

    lab_resolve        a prim's or object's world pose, or relative to another
    lab_checkpoint     snapshot parts (poses) and robot (all joint positions)
    lab_restore        put a checkpoint back
    lab_set            teleport objects, switch an object kinematic/dynamic, set joints
    lab_reset_scene    load_layout(seed) + verify_reset
    lab_rest           are the objects at rest? (speeds over the last window of sim time)
    lab_contacts_arm   start contact reports on bodies (any rigid bodies, pairs kept)
    lab_contacts_read  contact events since a sequence number
    lab_robot_health   the simulator's side of robot health (playing, sim advancing, joints)
    lab_verify_assembly the episode checker's rule on the live poses (runner/episode_runner/checker.py)
    lab_camera_view    create or move a lab camera (session layer: never saved into the stage file)
    lab_events_poll    the lab-event journal (host relay -> /cell/lab_events)

Every verb that changes the world appends a lab event (kind, objects, detail,
sim time) to the journal, so the WatchDog does not read a teleport as physics.
Async verbs (camera capture, until_at_rest) live in isaac/lab_tools because a
command runs inside one app update.

Pure helpers at the top run off Kit (tests: test_lab.py).
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any, Callable, Sequence

Vec = Sequence[float]
Quat = Sequence[float]  # x, y, z, w


# -- pure helpers ---------------------------------------------------------------

def qmul(a: Quat, b: Quat) -> list[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz]


def qconj(q: Quat) -> list[float]:
    return [-q[0], -q[1], -q[2], q[3]]


def rotate(q: Quat, v: Vec) -> list[float]:
    x, y, z, _ = qmul(qmul(q, [v[0], v[1], v[2], 0.0]), qconj(q))
    return [x, y, z]


def relative(parent: dict[str, Any], child: dict[str, Any]) -> dict[str, Any]:
    """parent_T_child from two world poses {position, orientation_xyzw}."""
    qi = qconj(parent["orientation_xyzw"])
    d = [c - p for c, p in zip(child["position"], parent["position"])]
    q = qmul(qi, child["orientation_xyzw"])
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return {"position": rotate(qi, d), "orientation_xyzw": [c / n for c in q]}


def angle_deg(a: Quat, b: Quat) -> float:
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


class PoseHistory:
    """Per-object (sim_time, position, orientation) samples over the last few seconds of sim time."""

    def __init__(self, keep_s: float = 3.0) -> None:
        self.keep_s = keep_s
        self.samples: dict[str, deque] = {}

    def add(self, t: float, poses: dict[str, dict[str, Any]]) -> None:
        for oid, p in poses.items():
            d = self.samples.setdefault(oid, deque())
            if d and t <= d[-1][0]:
                if t < d[-1][0]:        # time went back (stop/play, reload): start over
                    d.clear()
                else:
                    continue
            d.append((t, list(p["position"]), list(p["orientation_xyzw"])))
            while d and d[-1][0] - d[0][0] > self.keep_s:
                d.popleft()

    def clear(self) -> None:
        self.samples.clear()

    def speeds(self, oid: str, window_s: float) -> dict[str, Any] | None:
        """Max linear (m/s) and angular (rad/s) speed between consecutive samples in the window."""
        d = self.samples.get(oid)
        if not d or len(d) < 2:
            return None
        t_end = d[-1][0]
        pts = [s for s in d if s[0] >= t_end - window_s]
        if len(pts) < 2:
            return None
        lin = ang = 0.0
        for (t0, p0, q0), (t1, p1, q1) in zip(pts, pts[1:]):
            dt = t1 - t0
            if dt <= 0:
                continue
            lin = max(lin, math.dist(p0, p1) / dt)
            ang = max(ang, math.radians(angle_deg(q0, q1)) / dt)
        return {"linear_m_s": lin, "angular_rad_s": ang, "samples": len(pts), "span_s": pts[-1][0] - pts[0][0]}


def at_rest(history: PoseHistory, ids: Sequence[str], window_s: float, linear_m_s: float,
            angular_rad_s: float, min_samples: int = 5) -> dict[str, Any]:
    """The labmate's rest rule (task.yaml referee.at_rest): every object under both speed
    bounds over the last window_s of sim time, with enough samples spanning the window."""
    rows, ok = {}, True
    for oid in ids:
        s = history.speeds(oid, window_s)
        if s is None or s["samples"] < min_samples or s["span_s"] < 0.9 * window_s:
            rows[oid] = {"at_rest": False, "reason": "not enough samples in the window yet", **(s or {})}
            ok = False
            continue
        r = s["linear_m_s"] <= linear_m_s and s["angular_rad_s"] <= angular_rad_s
        rows[oid] = {"at_rest": r, **s}
        ok = ok and r
    return {"at_rest": ok, "objects": rows,
            "rule": {"window_s": window_s, "linear_m_s": linear_m_s, "angular_rad_s": angular_rad_s, "min_samples": min_samples}}


class Journal:
    """Sequence-numbered event list, capped."""

    def __init__(self, cap: int = 2000) -> None:
        self.cap, self.seq, self.events = cap, 0, []

    def add(self, event: dict[str, Any]) -> dict[str, Any]:
        self.seq += 1
        e = {"seq": self.seq, **event}
        self.events.append(e)
        del self.events[:-self.cap]
        return e

    def since(self, seq: int, limit: int = 500) -> list[dict[str, Any]]:
        return [e for e in self.events if e["seq"] > seq][-limit:]


# -- the lab, bound to a running CellCommandServer --------------------------------

def episode_running(lock_path) -> bool:
    """True while the episode runner holds records/.cell.lock: the lab must not change the world then."""
    import fcntl

    try:
        lock = open(lock_path, "a")
    except OSError:
        return False
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.flock(lock, fcntl.LOCK_UN)
        return False
    except OSError:
        return True
    finally:
        lock.close()


BUSY = {"ok": False, "error": "an episode is running on the cell (records/.cell.lock is held); the lab waits"}
REST_DEFAULT = {"window_s": 1.0, "linear_m_s": 0.002, "angular_rad_s": 0.02, "min_samples": 5}
LAB_CAMERA_ROOT = "/World/LabCameras"


class Lab:
    def __init__(self, server: Any, kit: Any, now_iso: Callable[[], str], lock_path: Any = None) -> None:
        self.s, self.K, self.now_iso = server, kit, now_iso
        self.lock_path = lock_path
        self.history = PoseHistory()
        self.events = Journal()
        self.contacts = Journal()
        self.checkpoints: dict[str, dict[str, Any]] = {}
        self._contact_sub: Any = None
        self._armed: set[str] = set()
        self._touching: dict[tuple[str, str], float] = {}

    # called from the server's update tick
    def sample(self) -> None:
        if not self.K.is_playing():
            return
        poses = {}
        for path in self.s._tf_prims():
            try:
                poses[path.rsplit("/", 1)[-1]] = self.K.world_pose(path)
            except Exception:
                pass
        self.history.add(self.K.sim_time_s(), poses)

    def event(self, kind: str, objects: Sequence[str], detail: dict[str, Any] | None = None) -> dict[str, Any]:
        return self.events.add({"kind": kind, "objects": list(objects), "detail": detail or {},
                                "sim_time_s": self.K.sim_time_s(), "wall_time_iso": self.now_iso()})

    # -- names ------------------------------------------------------------------
    def _path(self, ref: str) -> str:
        ref = str(ref)
        if ref.startswith("/"):
            return ref
        if ref == "tcp":
            return str(self.s.scene.get("tcp_prim"))
        for path in self.s._tf_prims():
            if path.rsplit("/", 1)[-1] == ref:
                return path
        if self.K.prim_exists(f"/World/{ref}"):
            return f"/World/{ref}"
        raise ValueError(f"unknown object or frame {ref!r}: give an object id (report_poses names), 'tcp' or a prim path")

    def _objects(self) -> dict[str, str]:
        """object id -> prim, for the rigid bodies the cell tracks (no frames under fixtures)."""
        out = {}
        for path in self.s._tf_prims():
            if path.count("/") == 2 and self.K.is_rigid_body(path):
                out[path.rsplit("/", 1)[-1]] = path
        return out

    def _joints(self) -> dict[str, float]:
        return self.s._articulation_handle().positions()

    # -- verbs ------------------------------------------------------------------
    def resolve(self, p: dict[str, Any]) -> dict[str, Any]:
        ref = p.get("ref")
        if not ref:
            return {"ok": False, "error": "ref is required"}
        pose = self.K.world_pose(self._path(ref))
        out = {"ok": True, "ref": ref, "frame": "world", **pose, "sim_time_s": self.K.sim_time_s()}
        rel = p.get("relative_to")
        if rel:
            out.update(relative(self.K.world_pose(self._path(rel)), pose), frame=str(rel))
        return out

    def checkpoint(self, p: dict[str, Any]) -> dict[str, Any]:
        name = str(p.get("name") or f"cp{len(self.checkpoints) + 1}")
        objs = {oid: {"prim": path, **self.K.world_pose(path)} for oid, path in self._objects().items()}
        cp = {"name": name, "sim_time_s": self.K.sim_time_s(), "objects": objs, "joints": self._joints(),
              "kinematic": {oid: self.K.is_kinematic(path) for oid, path in self._objects().items()}}
        self.checkpoints[name] = cp
        return {"ok": True, "checkpoint": name, "objects": sorted(objs), "joints": len(cp["joints"])}

    def busy(self) -> bool:
        return bool(self.lock_path) and episode_running(self.lock_path)

    def restore(self, p: dict[str, Any]) -> dict[str, Any]:
        if self.busy():
            return dict(BUSY)
        name = str(p.get("checkpoint") or "")
        cp = self.checkpoints.get(name)
        if cp is None:
            return {"ok": False, "error": f"no checkpoint {name!r}", "checkpoints": sorted(self.checkpoints)}
        for oid, o in cp["objects"].items():
            self.K.set_kinematic(o["prim"], bool(cp["kinematic"].get(oid)))
            self.K.teleport(o["prim"], o["position"], o["orientation_xyzw"])
            self.K.zero_velocities(o["prim"])
        missing = self.s._articulation_handle().set_positions(cp["joints"])
        self.history.clear()
        self.event("restore", list(cp["objects"]), {"checkpoint": name})
        return {"ok": True, "checkpoint": name, "joints_missing": missing or []}

    def set(self, p: dict[str, Any]) -> dict[str, Any]:
        if self.busy():
            return dict(BUSY)
        changed, report = [], {}
        for oid, spec in (p.get("dynamics") or {}).items():
            if spec not in ("kinematic", "dynamic"):
                return {"ok": False, "error": f"dynamics[{oid}] must be 'kinematic' or 'dynamic'"}
            self.K.set_kinematic(self._path(oid), spec == "kinematic")
            changed.append(oid)
        for oid, pose in (p.get("objects") or {}).items():
            path = self._path(oid)
            if not self.K.is_rigid_body(path):
                return {"ok": False, "error": f"{oid} is not a rigid body (frames cannot be set)"}
            self.K.teleport(path, pose["position"], pose.get("orientation_xyzw", self.K.world_pose(path)["orientation_xyzw"]))
            self.K.zero_velocities(path)
            changed.append(oid)
        joints = dict(p.get("robot") or {})
        if p.get("gripper") is not None:
            joints[str(self.s.scene.get("_gripper_master"))] = float(p["gripper"])
        if joints:
            report["joints_missing"] = self.s._articulation_handle().set_positions({k: float(v) for k, v in joints.items()}) or []
            changed.append("robot")
        if not changed:
            return {"ok": False, "error": "nothing to set: give objects, dynamics, robot and/or gripper"}
        self.history.clear()
        self.event("set", changed, {k: p[k] for k in ("objects", "dynamics", "robot", "gripper") if p.get(k) is not None})
        return {"ok": True, "changed": changed, **report}

    def reset_scene(self, p: dict[str, Any]) -> dict[str, Any]:
        if self.busy():
            return dict(BUSY)
        layout = self.s._cmd_load_layout({"assembly": p.get("assembly", "fmb_assembly_2"),
                                          "layout": p.get("layout", "nominal"), "seed": int(p.get("seed", 0) or 0)})
        verify = self.s._cmd_verify_reset({}) if layout.get("ok") else {"ok": False}
        self.history.clear()
        self.event("reset", layout.get("placed", []), {"seed": p.get("seed", 0), "layout": p.get("layout", "nominal")})
        return {"ok": bool(layout.get("ok") and verify.get("ok")), "load_layout": layout, "verify": verify}

    def rest(self, p: dict[str, Any]) -> dict[str, Any]:
        rule = {**REST_DEFAULT, **{k: p[k] for k in REST_DEFAULT if p.get(k) is not None}}
        ids = list(p.get("objects") or [oid for oid in self._objects() if not self.K.is_kinematic(self._path(oid))])
        r = at_rest(self.history, ids, float(rule["window_s"]), float(rule["linear_m_s"]),
                    float(rule["angular_rad_s"]), int(rule["min_samples"]))
        return {"ok": True, "sim_time_s": self.K.sim_time_s(), **r}

    def contacts_arm(self, p: dict[str, Any]) -> dict[str, Any]:
        bodies = [self._path(b) for b in (p.get("bodies") or [])]
        if not bodies:
            return {"ok": False, "error": "bodies is required (object ids or prim paths)"}
        applied = self.K.enable_contact_report(bodies)
        self._armed.update(applied)
        if self._contact_sub is None:
            self._contact_sub = self.K.subscribe_contact_reports(self._on_contacts)
        return {"ok": True, "armed": sorted(self._armed), "not_found": [b for b in bodies if b not in applied]}

    def _on_contacts(self, reports: list[dict[str, Any]]) -> None:
        t = self.K.sim_time_s()
        for r in reports:
            a, b = r["body0"], r["body1"]
            if a not in self._armed and b not in self._armed:
                continue
            key = tuple(sorted((a, b)))
            kind = "lost" if "LOST" in r["event"].upper() else ("found" if "FOUND" in r["event"].upper() else "persist")
            if kind == "persist" and key in self._touching:
                self._touching[key] = max(self._touching[key], r["impulse"])
                continue
            if kind == "lost":
                peak = self._touching.pop(key, 0.0)
            else:
                self._touching[key] = r["impulse"]
                peak = r["impulse"]
            self.contacts.add({"event": "contact_lost" if kind == "lost" else "contact_made", "bodies": list(key),
                               "impulse": r["impulse"], "peak_impulse": peak,
                               "point": r["points"][0] if r["points"] else None, "sim_time_s": t})

    def contacts_read(self, p: dict[str, Any]) -> dict[str, Any]:
        since = int(p.get("since_seq", 0) or 0)
        return {"ok": True, "events": self.contacts.since(since), "last_seq": self.contacts.seq,
                "touching": [list(k) for k in self._touching], "armed": sorted(self._armed)}

    def robot_health(self, _p: dict[str, Any]) -> dict[str, Any]:
        h = self.s._cmd_health({})
        try:
            joints = self._joints()
        except Exception as exc:  # noqa: BLE001
            joints, h["ok"] = {"error": str(exc)}, False
        return {"ok": bool(h.get("ok")), "side": "simulator (controllers: the platform MCP's robot_health)",
                "playing": h.get("playing"), "sim_advancing": h.get("sim_advancing"), "sim_time_s": h.get("sim_time_s"),
                "joints": joints, "episode_joints": h.get("episode_joints")}

    def verify_assembly(self, p: dict[str, Any]) -> dict[str, Any]:
        """The runner's ground-truth checker on the live poses (no fixture or bystander
        'before', so only the parts' seating is judged)."""
        import importlib.util

        import yaml

        from isaac.cell_command_server.server import repo_root
        from runner.episode_runner import checker

        root = repo_root()
        spec = importlib.util.spec_from_file_location(
            "_lab_placements", root / "ros2_ws/src/cell_platform_interfaces/cell_platform_interfaces/placements.py")
        P = importlib.util.module_from_spec(spec)
        import sys
        sys.modules[spec.name] = P          # dataclasses look their module up while the class is built
        spec.loader.exec_module(P)
        assembly = str(p.get("assembly", "fmb_assembly_2"))
        req = yaml.safe_load((root / "config/requirements" / f"{assembly}.yaml").read_text())
        parts = yaml.safe_load((root / "config/parts/watchdog_parts.yaml").read_text())["parts"]
        after = {k: v for k, v in self.s._cmd_report_poses({})["poses"].items() if "error" not in v}
        P.derive_into(after, P.load_placements(root, assembly))
        r = checker.check({**req, "bystanders": []}, parts, {}, after, P.load_symmetries(root, assembly))
        return {"ok": True, "pass": r["pass"], "parts_placed": r["parts_placed"], "parts_required": r["parts_required"],
                "parts": r["parts"], "sim_time_s": self.K.sim_time_s()}

    def camera_view(self, p: dict[str, Any]) -> dict[str, Any]:
        """Create or move /World/LabCameras/<name> in the session layer, looking from eye at target."""
        name = str(p.get("name") or "view")
        eye, target = p.get("eye"), p.get("target")
        if not (isinstance(eye, list) and isinstance(target, list) and len(eye) == 3 and len(target) == 3):
            return {"ok": False, "error": "eye [x,y,z] and target [x,y,z] (world, metres) are required"}
        path = self.K.lab_camera(f"{LAB_CAMERA_ROOT}/{name}", eye, target, float(p.get("focal_length_mm", 18.0)))
        return {"ok": True, "camera": path, "eye": eye, "target": target}

    def events_poll(self, p: dict[str, Any]) -> dict[str, Any]:
        since = int(p.get("since_seq", 0) or 0)
        return {"ok": True, "events": self.events.since(since), "last_seq": self.events.seq}

    def commands(self) -> dict[str, Callable[[dict[str, Any]], dict[str, Any]]]:
        return {"lab_resolve": self.resolve, "lab_checkpoint": self.checkpoint, "lab_restore": self.restore,
                "lab_set": self.set, "lab_reset_scene": self.reset_scene, "lab_rest": self.rest,
                "lab_contacts_arm": self.contacts_arm, "lab_contacts_read": self.contacts_read,
                "lab_robot_health": self.robot_health, "lab_verify_assembly": self.verify_assembly,
                "lab_camera_view": self.camera_view, "lab_events_poll": self.events_poll}
