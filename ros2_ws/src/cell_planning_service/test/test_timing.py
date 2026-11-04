"""Offline tests for the time parameterization and failure mapping."""

from __future__ import annotations

import pytest

from cell_planning_service import failure_codes
from cell_planning_service.timing import (
    JointLimit,
    interpolate_joint_path,
    load_joint_limits_yaml,
    time_parameterize,
)

LIMITS = [JointLimit(max_velocity=1.0, max_acceleration=2.0)] * 2


def _max_vel(timed):
    return max(abs(v) for row in timed.velocities for v in row)


def _max_acc(timed):
    worst = 0.0
    for i in range(1, len(timed.times)):
        dt = timed.times[i] - timed.times[i - 1]
        for k in range(len(timed.velocities[i])):
            worst = max(worst, abs(timed.velocities[i][k] - timed.velocities[i - 1][k]) / dt)
    return worst


def test_single_point_has_zero_time():
    timed = time_parameterize([[0.1, 0.2]], LIMITS)
    assert timed.times == [0.0]
    assert timed.velocities == [[0.0, 0.0]]


def test_velocity_bound_holds():
    path = [[0.0, 0.0], [0.5, 0.0], [1.0, 0.0], [1.5, 0.0]]
    timed = time_parameterize(path, LIMITS)
    assert timed.times[0] == 0.0
    assert all(b > a for a, b in zip(timed.times, timed.times[1:]))
    assert _max_vel(timed) <= 1.0 + 1e-9
    assert timed.velocities[0] == [0.0, 0.0] and timed.velocities[-1] == [0.0, 0.0]


def test_acceleration_bound_holds_after_smoothing():
    path = [[0.0, 0.0], [0.05, 0.0], [1.0, 0.0], [1.05, 0.0]]
    timed = time_parameterize(path, LIMITS)
    assert _max_acc(timed) <= 2.0 + 1e-6


def test_scaling_slows_the_path():
    path = [[0.0, 0.0], [1.0, 0.0]]
    full = time_parameterize(path, LIMITS)
    half = time_parameterize(path, LIMITS, velocity_scaling=0.5)
    assert half.times[-1] >= 2 * full.times[-1] - 1e-9


def test_bad_scaling_falls_back_to_one():
    path = [[0.0, 0.0], [1.0, 0.0]]
    a = time_parameterize(path, LIMITS, velocity_scaling=0.0)
    b = time_parameterize(path, LIMITS, velocity_scaling=1.0)
    assert a.times == b.times


def test_joint_count_mismatch_raises():
    with pytest.raises(ValueError):
        time_parameterize([[0.0, 0.0, 0.0]], LIMITS)


def test_interpolate_densifies():
    states = interpolate_joint_path([[0.0], [1.0]], 0.25)
    assert [s for _, s in states] == [[0.0], [0.25], [0.5], [0.75], [1.0]]
    assert [i for i, _ in states] == [0, 1, 1, 1, 1]


def test_interpolate_zero_resolution_keeps_points():
    states = interpolate_joint_path([[0.0], [1.0]], 0.0)
    assert len(states) == 2


def test_load_joint_limits_yaml():
    text = """
joint_limits:
  a:
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: true
    max_acceleration: 3.0
  b:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: false
"""
    limits = load_joint_limits_yaml(text, ["a"])
    assert limits["a"] == JointLimit(2.0, 3.0)
    with pytest.raises(ValueError):
        load_joint_limits_yaml(text, ["b"])
    with pytest.raises(ValueError):
        load_joint_limits_yaml(text, ["missing"])


def test_moveit_error_code_mapping():
    assert failure_codes.from_moveit(1) == failure_codes.NONE
    assert failure_codes.from_moveit(-31) == failure_codes.NO_IK_SOLUTION
    assert failure_codes.from_moveit(-10) == failure_codes.START_STATE_IN_COLLISION
    assert failure_codes.from_moveit(-26) == failure_codes.START_STATE_INVALID
    assert failure_codes.from_moveit(-6) == failure_codes.TIMEOUT
    assert failure_codes.from_moveit(424242) == failure_codes.BACKEND_ERROR
    assert all(code in failure_codes.ALL or code == "" for code in failure_codes.MOVEIT_ERROR_CODE_MAP.values())
