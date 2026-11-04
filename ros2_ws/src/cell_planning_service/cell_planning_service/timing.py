"""Conservative time parameterization for a joint-space path.

Used for Cartesian segments when the backend returns an untimed path, and
by the cuRobo backend's IK-chain Cartesian planner. It is not a
time-optimal parameterization; it is a per-segment velocity bound followed
by a few smoothing passes that stretch segments until the acceleration
bound holds. The result is a trajectory the bridge can follow at a modest
pace, which is what approach and insertion segments want.

Inputs are explicit: joint limits come from the caller (the node reads
them from a joint_limits.yaml the config names). Nothing here is hardcoded.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class JointLimit:
    max_velocity: float      # rad/s (or m/s), > 0
    max_acceleration: float  # rad/s^2, > 0


@dataclass(frozen=True)
class TimedPath:
    times: list[float]                  # time_from_start per point, seconds
    positions: list[list[float]]        # N x J
    velocities: list[list[float]]       # N x J, zero at both ends


def load_joint_limits_yaml(text: str, joint_names: Sequence[str]) -> dict[str, JointLimit]:
    """Parse a MoveIt-style joint_limits.yaml (already read as text).

    Only joints in joint_names are returned; a joint without both velocity and
    acceleration limits raises, because the parameterization cannot invent
    them.
    """
    import yaml

    data = yaml.safe_load(text) or {}
    table = data.get("joint_limits", data)
    out: dict[str, JointLimit] = {}
    for name in joint_names:
        entry = table.get(name)
        if not isinstance(entry, dict):
            raise ValueError(f"joint_limits.yaml has no entry for {name}")
        if not entry.get("has_velocity_limits") or not entry.get("has_acceleration_limits"):
            raise ValueError(f"joint_limits.yaml: {name} needs velocity and acceleration limits")
        out[name] = JointLimit(
            max_velocity=float(entry["max_velocity"]),
            max_acceleration=float(entry["max_acceleration"]),
        )
    return out


def time_parameterize(
    positions: Sequence[Sequence[float]],
    limits: Sequence[JointLimit],
    *,
    velocity_scaling: float = 1.0,
    acceleration_scaling: float = 1.0,
    min_segment_s: float = 1e-3,
    smoothing_passes: int = 8,
) -> TimedPath:
    """Assign times to a path so velocity and acceleration bounds hold.

    positions: N points of J joints, N >= 1.
    limits: J entries, same order as the joints.
    scaling factors in (0, 1]; values outside are clamped to that range.
    """
    n = len(positions)
    if n == 0:
        raise ValueError("empty path")
    j = len(limits)
    if any(len(p) != j for p in positions):
        raise ValueError("every point must have one value per joint limit")
    vs = _clamp_scale(velocity_scaling)
    as_ = _clamp_scale(acceleration_scaling)
    vmax = [lim.max_velocity * vs for lim in limits]
    amax = [lim.max_acceleration * as_ for lim in limits]
    if any(v <= 0 for v in vmax) or any(a <= 0 for a in amax):
        raise ValueError("joint limits must be positive")

    if n == 1:
        return TimedPath([0.0], [list(map(float, positions[0]))], [[0.0] * j])

    # 1. Segment durations from the velocity bound.
    durations: list[float] = []
    for i in range(1, n):
        dt = min_segment_s
        for k in range(j):
            dq = abs(float(positions[i][k]) - float(positions[i - 1][k]))
            dt = max(dt, dq / vmax[k])
        durations.append(dt)

    # 2. Stretch segments until the acceleration bound holds at every
    #    interior point and at both ends (where velocity is zero).
    for _ in range(smoothing_passes):
        changed = False
        seg_vel = _segment_velocities(positions, durations)
        for i in range(n - 1):
            prev_v = seg_vel[i - 1] if i > 0 else [0.0] * j
            next_v = seg_vel[i]
            # Acceleration across the boundary before segment i.
            span = (durations[i - 1] if i > 0 else 0.0) * 0.5 + durations[i] * 0.5
            for k in range(j):
                dv = abs(next_v[k] - prev_v[k])
                if span <= 0 or dv / span <= amax[k]:
                    continue
                needed = dv / amax[k]
                extra = needed - span
                durations[i] += extra
                if i > 0:
                    durations[i - 1] += extra
                changed = True
        # End boundary (decelerate to rest after the last segment).
        last_v = _segment_velocities(positions, durations)[-1]
        span = durations[-1] * 0.5
        for k in range(j):
            if span > 0 and abs(last_v[k]) / span > amax[k]:
                durations[-1] += abs(last_v[k]) / amax[k] - span
                changed = True
        if not changed:
            break

    times = [0.0]
    for dt in durations:
        times.append(times[-1] + dt)
    seg_vel = _segment_velocities(positions, durations)
    velocities: list[list[float]] = [[0.0] * j]
    for i in range(1, n - 1):
        velocities.append([0.5 * (seg_vel[i - 1][k] + seg_vel[i][k]) for k in range(j)])
    velocities.append([0.0] * j)
    return TimedPath(times, [list(map(float, p)) for p in positions], velocities)


def _segment_velocities(
    positions: Sequence[Sequence[float]], durations: Sequence[float]
) -> list[list[float]]:
    out: list[list[float]] = []
    for i, dt in enumerate(durations, start=1):
        out.append(
            [
                (float(positions[i][k]) - float(positions[i - 1][k])) / dt
                for k in range(len(positions[i]))
            ]
        )
    return out


def _clamp_scale(value: float) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 1.0
    if v <= 0.0 or v > 1.0:
        return 1.0
    return v


def interpolate_joint_path(
    positions: Sequence[Sequence[float]], max_step: float
) -> list[tuple[int, list[float]]]:
    """Densify a joint path so no joint moves more than max_step between
    consecutive states. Returns (source_index, state) pairs, where
    source_index is the index of the original point the state belongs to
    (interpolated states carry the index of the point they lead to)."""
    if max_step <= 0:
        return [(i, list(map(float, p))) for i, p in enumerate(positions)]
    out: list[tuple[int, list[float]]] = []
    for i, point in enumerate(positions):
        if i == 0:
            out.append((0, list(map(float, point))))
            continue
        prev = positions[i - 1]
        biggest = max(abs(float(a) - float(b)) for a, b in zip(point, prev))
        steps = max(1, int(biggest / max_step + 0.999999))
        for s in range(1, steps + 1):
            t = s / steps
            out.append((i, [float(b) + (float(a) - float(b)) * t for a, b in zip(point, prev)]))
    return out
