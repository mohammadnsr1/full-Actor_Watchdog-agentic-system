"""The v2 episode start gate (Milestone 7): RequestGo(phase="run") waits for
run-scope readiness alone, and a new episode_id clears run-scoped state."""

from __future__ import annotations

import threading
import time

import pytest
import rclpy
from cell_sync_interfaces.srv import RequestGo, SignalReady

from cell_orchestrator.node import OrchestratorNode


@pytest.fixture(autouse=True)
def _rclpy_context():
    rclpy.init(args=None)
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _node(tmp_path, episode_id="ep1", guard_mode="guarded") -> OrchestratorNode:
    node = OrchestratorNode(log_dir=tmp_path)
    node.set_parameters([rclpy.parameter.Parameter("guard_mode", value=guard_mode),
                         rclpy.parameter.Parameter("episode_id", value=episode_id)])
    return node


def _start(node, episode_id="ep1", timeout=0.3) -> RequestGo.Response:
    req = RequestGo.Request()
    req.step_id, req.phase, req.timeout_sec = episode_id, "run", timeout
    return node._handle_request_go(req, RequestGo.Response())


def _signal(node, scope="run", detail=""):
    req = SignalReady.Request()
    req.scope = scope
    if detail:
        req.detail = detail
    return node._handle_signal_ready(req, SignalReady.Response())


def test_run_readiness_alone_releases_the_start(tmp_path):
    node = _node(tmp_path)
    _signal(node)
    resp = _start(node)
    assert (resp.proceed, resp.timed_out, resp.reason) == (True, False, "released")
    node.destroy_node()


def test_start_blocks_until_the_watchdog_is_ready(tmp_path):
    node = _node(tmp_path)
    threading.Timer(0.15, lambda: _signal(node)).start()
    t0 = time.monotonic()
    resp = _start(node, timeout=2.0)
    assert resp.reason == "released" and 0.1 < time.monotonic() - t0 < 1.0
    node.destroy_node()


def test_no_watchdog_times_out_and_says_so(tmp_path):
    node = _node(tmp_path)
    resp = _start(node, timeout=0.2)
    assert resp.proceed is True and resp.timed_out is True and resp.reason == "unguarded_latched"
    node.destroy_node()


def test_unguarded_episode_starts_at_once(tmp_path):
    node = _node(tmp_path, guard_mode="unguarded")
    resp = _start(node)
    assert resp.reason == "unguarded_by_config" and resp.waited_sec == 0.0
    node.destroy_node()


def test_a_new_episode_forgets_the_previous_readiness(tmp_path):
    node = _node(tmp_path, episode_id="ep1")
    _signal(node)
    assert _start(node, "ep1").reason == "released"
    node.set_parameters([rclpy.parameter.Parameter("episode_id", value="ep2")])
    resp = _start(node, "ep2", timeout=0.2)
    assert resp.timed_out is True, "ep1's readiness must not open ep2's start"
    node.destroy_node()


def test_a_new_episode_clears_an_abort(tmp_path):
    node = _node(tmp_path, episode_id="ep1")
    _signal(node, "abort", detail="stop")
    assert _start(node, "ep1").reason == "aborted"
    node.set_parameters([rclpy.parameter.Parameter("episode_id", value="ep2")])
    _signal(node)
    assert _start(node, "ep2").reason == "released"
    node.destroy_node()
