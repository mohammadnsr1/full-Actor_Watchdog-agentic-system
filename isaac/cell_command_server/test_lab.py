"""Lab verbs off Kit: pure helpers, and the Lab driven through a fake Kit + server."""
import math

import pytest

from isaac.cell_command_server import lab as LB

Q0 = [0.0, 0.0, 0.0, 1.0]
Z90 = [0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)]


def test_relative_pose():
    parent = {"position": [1.0, 0.0, 0.0], "orientation_xyzw": Z90}
    child = {"position": [1.0, 1.0, 0.0], "orientation_xyzw": Z90}
    r = LB.relative(parent, child)
    assert r["position"] == pytest.approx([1.0, 0.0, 0.0], abs=1e-9)        # +y world is +x in a frame turned 90 deg
    assert LB.angle_deg(r["orientation_xyzw"], Q0) == pytest.approx(0.0, abs=1e-6)


def test_rest_rule_needs_a_full_window_and_low_speed():
    h = LB.PoseHistory()
    for i in range(12):                                   # 0.1 s steps, still
        h.add(i * 0.1, {"a": {"position": [0, 0, 0], "orientation_xyzw": Q0}})
    assert LB.at_rest(h, ["a"], 1.0, 0.002, 0.02)["at_rest"]
    h.add(1.2, {"a": {"position": [0.01, 0, 0], "orientation_xyzw": Q0}})   # 10 cm/s
    r = LB.at_rest(h, ["a"], 1.0, 0.002, 0.02)
    assert not r["at_rest"] and r["objects"]["a"]["linear_m_s"] == pytest.approx(0.1)
    h.clear()
    h.add(0.0, {"a": {"position": [0, 0, 0], "orientation_xyzw": Q0}})
    assert LB.at_rest(h, ["a"], 1.0, 0.002, 0.02)["objects"]["a"]["at_rest"] is False   # too few samples


def test_history_restarts_when_time_goes_back():
    h = LB.PoseHistory()
    h.add(5.0, {"a": {"position": [0, 0, 0], "orientation_xyzw": Q0}})
    h.add(0.5, {"a": {"position": [1, 0, 0], "orientation_xyzw": Q0}})
    assert len(h.samples["a"]) == 1


class FakeKit:
    def __init__(self):
        self.poses = {"/World/u_brown": {"position": [0.3, 0, 0.04], "orientation_xyzw": Q0},
                      "/World/base1": {"position": [0.4, -0.2, 0.02], "orientation_xyzw": Q0},
                      "/World/base1/u_brown_placement": {"position": [0.4, -0.15, 0.04], "orientation_xyzw": Q0},
                      "/World/ur5e/tcp": {"position": [0.3, 0, 0.4], "orientation_xyzw": Q0}}
        self.kinematic = {"/World/base1": True}
        self.t = 1.0
        self.sub = None

    def is_playing(self): return True
    def sim_time_s(self): return self.t
    def world_pose(self, p): return {k: list(v) for k, v in self.poses[p].items()}
    def prim_exists(self, p): return p in self.poses
    def is_rigid_body(self, p): return p in ("/World/u_brown", "/World/base1")
    def is_kinematic(self, p): return self.kinematic.get(p, False)
    def set_kinematic(self, p, k): self.kinematic[p] = k
    def teleport(self, p, pos, q): self.poses[p] = {"position": list(pos), "orientation_xyzw": list(q)}
    def zero_velocities(self, p): pass
    def enable_contact_report(self, paths): return [p for p in paths if p in self.poses]
    def subscribe_contact_reports(self, cb): self.sub = cb; return object()


class FakeArt:
    def __init__(self): self.q = {"shoulder_pan_joint": 0.0, "finger_joint": -0.55}
    def positions(self): return dict(self.q)
    def set_positions(self, d): self.q.update(d); return []


class FakeServer:
    def __init__(self):
        self.scene = {"tcp_prim": "/World/ur5e/tcp", "_gripper_master": "finger_joint"}
        self.art = FakeArt()
    def _tf_prims(self): return ["/World/u_brown", "/World/base1", "/World/base1/u_brown_placement"]
    def _articulation_handle(self): return self.art


@pytest.fixture
def lab():
    return LB.Lab(FakeServer(), FakeKit(), lambda: "2026-11-25T00:00:00Z")


def test_resolve_names_and_relative(lab):
    assert lab.resolve({"ref": "tcp"})["position"] == [0.3, 0, 0.4]
    r = lab.resolve({"ref": "u_brown_placement", "relative_to": "base1"})
    assert r["frame"] == "base1" and r["position"] == pytest.approx([0.0, 0.05, 0.02])
    with pytest.raises(ValueError):
        lab.resolve({"ref": "nothing"})


def test_set_checkpoint_restore_and_events(lab):
    cp = lab.checkpoint({"name": "start"})
    assert cp["objects"] == ["base1", "u_brown"]                   # rigid bodies only, no frames
    r = lab.set({"objects": {"u_brown": {"position": [0.5, 0, 0.1], "orientation_xyzw": Q0}}, "gripper": 0.2})
    assert r["ok"] and lab.K.poses["/World/u_brown"]["position"] == [0.5, 0, 0.1]
    assert lab.s.art.q["finger_joint"] == 0.2
    assert not lab.set({"objects": {"u_brown_placement": {"position": [0, 0, 0]}}})["ok"]   # frames are not bodies
    lab.restore({"checkpoint": "start"})
    assert lab.K.poses["/World/u_brown"]["position"] == [0.3, 0, 0.04] and lab.s.art.q["finger_joint"] == -0.55
    kinds = [e["kind"] for e in lab.events_poll({})["events"]]
    assert kinds == ["set", "restore"]
    assert not lab.restore({"checkpoint": "nope"})["ok"]


def test_contacts_made_once_then_lost(lab):
    lab.contacts_arm({"bodies": ["u_brown"]})
    rep = {"body0": "/World/u_brown", "body1": "/World/base1", "points": [[0, 0, 0]], "impulse": 0.1}
    lab.K.sub([{**rep, "event": "ContactEventType.CONTACT_FOUND"}])
    lab.K.sub([{**rep, "event": "ContactEventType.CONTACT_PERSIST", "impulse": 0.5}])
    lab.K.sub([{**rep, "event": "ContactEventType.CONTACT_LOST", "impulse": 0.0}])
    ev = lab.contacts_read({})["events"]
    assert [e["event"] for e in ev] == ["contact_made", "contact_lost"] and ev[1]["peak_impulse"] == 0.5
    lab.K.sub([{"body0": "/World/x", "body1": "/World/y", "points": [], "impulse": 1, "event": "CONTACT_FOUND"}])
    assert len(lab.contacts_read({})["events"]) == 2              # unarmed pair ignored


def test_rest_excludes_kinematic_by_default(lab):
    for i in range(12):
        lab.K.t = i * 0.1
        lab.sample()
    r = lab.rest({})
    assert r["at_rest"] and list(r["objects"]) == ["u_brown"]


def test_lab_tools_registers_every_verb():
    import isaac.lab_tools as T

    names = []

    class Reg:
        def tool(self, description, parameters=None, name=None, internal=False):
            def deco(fn):
                names.append(fn.__name__)
                return fn
            return deco

    T.register(Reg())
    assert T.MCP_NAMESPACE == "lab"
    assert set(names) == {"resolve", "checkpoint", "restore", "set", "reset_scene", "is_at_rest", "until_at_rest",
                          "contacts_arm", "contacts_read", "robot_health", "verify_assembly", "camera_view",
                          "camera_capture", "events"}


def test_world_changing_verbs_wait_for_the_episode(tmp_path):
    import fcntl

    lock_path = tmp_path / ".cell.lock"
    lab = LB.Lab(FakeServer(), FakeKit(), lambda: "t", lock_path)
    holder = open(lock_path, "w")
    fcntl.flock(holder, fcntl.LOCK_EX | fcntl.LOCK_NB)
    try:
        assert not lab.set({"gripper": 0.1})["ok"] and not lab.reset_scene({})["ok"]
        assert lab.resolve({"ref": "tcp"})["ok"]                 # reading is fine
    finally:
        fcntl.flock(holder, fcntl.LOCK_UN)
        holder.close()
    assert lab.set({"gripper": 0.1})["ok"]


def test_verify_assembly_loads_the_checker_and_placements():
    class S:
        def _cmd_report_poses(self, _p):
            return {"poses": {"base1": {"position": [0.4028, -0.1986, 0.0224], "orientation_xyzw": Q0}}}

    r = LB.Lab(S(), FakeKit(), lambda: "t").verify_assembly({})
    assert r["ok"] and not r["pass"] and r["parts_required"] == 4
    assert {v["status"] for v in r["parts"].values()} == {"missing"}
