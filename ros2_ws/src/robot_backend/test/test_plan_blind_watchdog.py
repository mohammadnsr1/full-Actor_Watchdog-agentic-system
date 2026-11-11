"""Plan-blind WatchDog (Milestone 5): the behaviours that replaced the
executor-coupled ones.

Replaces (removed with the behaviour they tested):
  test_watchdog_release_lifecycle.py      held span and release auto-disarm
                                           now driven by physical events
  test_watchdog_run_verdict.py            rollup validation is per part
  test_executor_terminal_status_source.py / test_run_ended_terminal_event.py
                                           termination is the episode_end notice
  test_final_run_verdict_cadence.py / test_watchdog_forced_termination.py
                                           the backstop runs only after episode_end
  test_tf_time_alignment.py               object poses come from the pose
                                           channel; the TCP sample is aligned
                                           to the observations' stamp
"""

from __future__ import annotations

import os
import threading
import time

import pytest

os.environ.setdefault("_ROBOT_BACKEND_MCP_VENV_READY", "1")

from robot_backend.watchdog.engine import MonitorEngine  # noqa: E402
from robot_backend.watchdog.monitors import MonitorPrimitive  # noqa: E402
from robot_backend.watchdog.physical import WakeCondition  # noqa: E402

import robot_backend.mcp_servers.watchdog_mcp_server as ws  # noqa: E402


class _Bus:
    def __init__(self):
        self.events = []
        self.lock = threading.Lock()

    def publish(self, event):
        with self.lock:
            self.events.append(event)

    def of_type(self, t):
        return [e for e in self.events if e.event_type == t]


class _Slip(MonitorPrimitive):
    feed = "tf"

    def __init__(self, monitor_id, child_frame):
        super().__init__("relative_frame", {"parent_frame": "tcp", "child_frame": child_frame}, monitor_id=monitor_id)
        self.child_frame = child_frame
        self.threshold, self.comparison = 0.01, "le"

    @property
    def subject_object_id(self):
        return self.child_frame

    def evaluate(self, sample):
        return None


class _Effort(MonitorPrimitive):
    feed = "joint_states"

    def __init__(self, monitor_id):
        super().__init__("effort_event", {"joint_name": "finger_joint"}, monitor_id=monitor_id)
        self.threshold, self.comparison = 16.0, "le"

    def evaluate(self, sample):
        return None


# -- held span and release auto-disarm from physical events --------------------


def test_lift_and_release_bracket_the_held_span():
    engine = MonitorEngine(event_bus=_Bus())
    assert engine.held_object_id() is None
    engine.on_physical_event("part_lifted", ["u_brown"], "ground_truth", 1.0)
    assert engine.held_object_id() == "u_brown"
    engine.on_physical_event("part_released", ["u_brown"], "ground_truth", 1.0)
    assert engine.held_object_id() is None


def test_release_disarms_only_that_parts_slip_monitor():
    bus = _Bus()
    engine = MonitorEngine(event_bus=bus)
    engine.arm(_Slip("slip_brown", "u_brown"))
    engine.arm(_Slip("slip_orange", "u_orange"))
    engine.arm(_Effort("effort"))
    engine.on_physical_event("part_lifted", ["u_brown"], "ground_truth", 1.0)
    engine.on_physical_event("part_released", ["u_brown"], "ground_truth", 1.0)
    armed = {m["monitor_id"] for m in engine.snapshot()}
    assert armed == {"slip_orange", "effort"}
    reasons = [e.payload["reason"] for e in bus.of_type("monitor_disarmed")]
    assert reasons == ["auto_disarm_release"]


def test_effort_fire_attributes_to_the_held_part_and_none_when_empty():
    engine = MonitorEngine(event_bus=_Bus())
    effort = _Effort("effort")
    assert engine._resolve_fire_object_id(effort) is None
    engine.on_physical_event("part_lifted", ["hex_blue"], "camera2_pose", 0.8)
    assert engine._resolve_fire_object_id(effort) == "hex_blue"


def test_skill_events_are_not_a_path_any_more():
    assert not hasattr(MonitorEngine, "on_skill_event")


# -- wake conditions ---------------------------------------------------------------


def test_wake_condition_matches_and_is_delivered_once():
    engine = MonitorEngine(event_bus=_Bus())
    armed = engine.arm_wake({"kinds": ["part_engaged_feature"], "scene_ids": ["u_brown"], "min_confidence": 0.5})
    engine.on_physical_event("part_engaged_feature", ["u_orange", "u_orange_placement"], "ground_truth", 1.0)
    assert engine.pop_wakes() == []
    engine.on_physical_event("part_engaged_feature", ["u_brown", "u_brown_placement"], "ground_truth", 1.0)
    wakes = engine.pop_wakes()
    assert len(wakes) == 1 and wakes[0]["wake_id"] == armed["wake_id"]
    assert engine.wake_snapshot() == []  # once=True removed it


def test_wake_condition_validation():
    with pytest.raises(ValueError):
        WakeCondition("w", {"kinds": ["teleport"]}, 0)
    with pytest.raises(ValueError):
        WakeCondition("w", {"min_confidence": 0.5}, 0)
    with pytest.raises(ValueError):
        WakeCondition("w", {"kinds": ["part_lifted"], "bogus": 1}, 0)


# -- rollup validation is per part ---------------------------------------------------


def test_run_clause_results_need_a_part():
    with pytest.raises(ValueError, match="part"):
        ws._validate_run_clause_results([{"clause_id": "effects.x", "step_id": "pick_u_brown"}])
    ws._validate_run_clause_results([{"clause_id": "effects.x", "part": "u_brown"}])


def test_evidence_sources_are_a_closed_vocabulary():
    with pytest.raises(ValueError):
        ws._validate_evidence([], None)
    with pytest.raises(ValueError):
        ws._validate_evidence(["telepathy"], None)
    sources, missing = ws._validate_evidence(["camera2_pose", "look:camera1"], ["u_orange occluded"])
    assert ws._is_shared_source(sources) and missing == ["u_orange occluded"]
    assert not ws._is_shared_source(["camera2_pose", "joint_state"])


# -- termination only after episode_end -------------------------------------------


class _Recorder:
    def __init__(self, episode_ended=False):
        self.episode_ended = episode_ended


class _Ctx:
    def __init__(self, episode_ended=False):
        self.recorder = _Recorder(episode_ended)
        self.event_bus = _Bus()

        class _Node:
            def get_logger(self):
                class _L:
                    def error(self, *_a):
                        pass
                return _L()

        self.node = _Node()


@pytest.fixture(autouse=True)
def _fresh_run():
    with ws._RUN_LOCK:
        ws._RUN.__init__()
    yield
    with ws._RUN_LOCK:
        ws._RUN.__init__()


def test_timeouts_before_episode_end_never_finalize():
    ctx = _Ctx(episode_ended=False)
    for _ in range(10):
        out = ws._timeout_or_finalize(ctx, "wait_for_event", time.monotonic())
        assert out["wake"] == "timeout" and out["episode_ended"] is False
    assert not ws._RUN.finalized and ctx.event_bus.of_type("run_verdict") == []


def test_backstop_after_episode_end_records_incomplete_once():
    ctx = _Ctx(episode_ended=True)
    first = ws._timeout_or_finalize(ctx, "wait_for_event", time.monotonic())
    assert first["wake"] == "timeout" and first["timeouts_before_forced_finalize"] == 1
    second = ws._timeout_or_finalize(ctx, "wait_for_event", time.monotonic())
    assert second["wake"] == "terminated" and second["verdict"] == "incomplete"
    third = ws._timeout_or_finalize(ctx, "wait_for_event", time.monotonic())
    assert third["wake"] == "terminated" and third["run_finalized"]
    verdicts = ctx.event_bus.of_type("run_verdict")
    assert len(verdicts) == 1
    assert verdicts[0].payload["executor_terminal_status"] == "episode_end"
    assert verdicts[0].payload["parts_covered"] == []


def test_run_verdict_carries_counts_and_recovery():
    ctx = _Ctx(episode_ended=True)
    ws._note_finding("u_orange")
    ws._note_evidence(shared=True, missing=["u_orange occluded"])
    event = ws._publish_run_verdict(
        ctx, verdict="conformant", clause_results=[{"clause_id": "effects.x", "part": "u_orange"}],
        parts_covered=["u_orange"], explanation="recovered", claimed_outcome=None,
        executor_terminal_status="episode_end", evidence_sources=["acceptance_inspection"],
    )
    p = event.payload
    assert p["recovered"] is True and p["findings_during_run"] == ["u_orange"]
    assert p["shared_source_count"] == 1 and p["missing_evidence_count"] == 1
    assert p["shared_source"] is False  # this verdict itself cites only its own inspection


# -- recorder: objects from the pose channel, TCP aligned to their stamp ----------


def test_recorder_samples_objects_from_pose_channel_and_aligns_tcp():
    try:
        import tf2_ros
        from geometry_msgs.msg import TransformStamped
    except ImportError:
        pytest.skip("tf2_ros not importable")
    from cell_platform_interfaces.types import PoseObservation
    from robot_backend.watchdog.recorder import EvidenceRecorder

    NS = 1_000_000_000
    base = 1000 * NS
    registry = {"objects": {"cube": {"enabled": True, "tf_frame": "cube", "lookup_frame": "world"}}}
    rec = EvidenceRecorder(node=None, registry=registry)
    rec.cell_frames = {"tcp": {"tf_frame": "tcp", "lookup_frame": "world"}}
    rec.frame_samples = {"tcp": __import__("collections").deque(maxlen=100)}
    buf = tf2_ros.Buffer()
    for step in range(0, 650, 50):  # tcp moves at 0.2 m/s; keyframes to +600 ms
        ts = TransformStamped()
        ts.header.frame_id = "world"
        ts.header.stamp.sec = (base + step * 1_000_000) // NS
        ts.header.stamp.nanosec = (base + step * 1_000_000) % NS
        ts.child_frame_id = "tcp"
        ts.transform.translation.x = 0.2 * step / 1000.0
        ts.transform.rotation.w = 1.0
        buf.set_transform(ts, "test")
    rec.tf_buffer = buf

    class _Src:
        def latest_all(self, max_age_s=None):
            # cube observed 150 ms behind the tcp's latest, rigidly 0.1 m behind it
            stamp = base + 450 * 1_000_000
            return {"cube": PoseObservation("cube", "world", (0.2 * 0.45 - 0.1, 0.0, 0.0), (0, 0, 0, 1), 1.0, "ground_truth", "camera2", stamp)}

        def latest(self, object_id, max_age_s=None):
            return self.latest_all().get(object_id)

    rec.pose_source = _Src()
    rec._on_tf_timer()
    tcp = rec.latest_frame_pose("tcp")
    cube = rec.latest_object_pose("cube")
    assert cube is not None and tcp is not None
    # aligned: both at the observation's stamp, separation is the rigid 0.100 m
    tx, cx = tcp["pose"]["position"]["x"], cube["pose"]["position"]["x"]
    assert abs((tx - cx) - 0.100) < 1e-6
    assert tcp["tf_stamp_ns"] == cube["tf_stamp_ns"] == base + 450 * 1_000_000
    # the object frame is answered from the channel, never TF
    t = rec.lookup_transform("world", "cube")
    assert abs(t.transform.translation.x - cx) < 1e-9


def test_acceptance_inspection_passes_fails_and_reports_missing():
    from robot_backend.watchdog import requirements as req

    class _Rec:
        def __init__(self):
            self.poses = {}

        def latest_object_pose(self, oid):
            return self.poses.get(oid)

        def pose_source_name(self):
            return "ground_truth"

    def pose(x, z=0.0):
        return {"wall_ns": time.time_ns(), "tf_stamp_ns": 1, "object_id": "o",
                "pose": {"position": {"x": x, "y": 0.0, "z": z}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}

    rec = _Rec()
    rec.poses = {"a": pose(0.0), "a_placement": pose(0.004), "b": pose(1.0), "b_placement": pose(1.5)}
    requirements = {"acceptance": {"parts": {
        "a": {"placement_frame": "a_placement", "grasp_width_m": 0.02},
        "b": {"placement_frame": "b_placement", "grasp_width_m": 0.02},
        "c": {"placement_frame": "c_placement", "grasp_width_m": 0.02},
    }}}
    parts = {"parts": {k: {"tolerances": {"position_m": 0.01, "orientation_deg": 5.0}} for k in "abc"}}
    out = req.inspect_acceptance(rec, requirements, parts)
    assert out["parts"]["a"]["status"] == "pass"
    assert out["parts"]["b"]["status"] == "fail" and out["parts"]["b"]["position_error_m"] == pytest.approx(0.5)
    assert out["parts"]["c"]["status"] == "missing_evidence" and out["missing_evidence"] == ["c"]
    assert out["overall"] == "fail" and out["evidence_sources"] == ["ground_truth"]
