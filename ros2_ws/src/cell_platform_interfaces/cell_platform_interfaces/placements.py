"""Fixture-relative placements: where each part goes, without placement TF.

An assembly's placement frames are fixed relative to its fixture (base1 for
fmb_assembly_2). They are measured once (tools/measure_placements.py) into
config/placements/<assembly>.yaml and from then on DERIVED from the fixture's
observed pose:

    world_T_placement = world_T_fixture (x) fixture_T_placement

so nothing has to publish placement frames, on sim or on hardware (where no
pipeline observes a virtual frame). WatchDog-side only: the WatchDog's pose
source (camera2), cell_physical_events and the runner's checker use this; the
executor never gets the file (config/settings/executor.*.json deny it) and its
camera1 source never derives placements.

File format:

    assembly: fmb_assembly_2
    fixture: base1                      # default fixture for every entry
    measured: {...}                     # provenance, free form
    placements:
      u_brown_placement:
        fixture: base1                  # optional, overrides the default
        position: [x, y, z]             # in the fixture's frame, metres
        orientation_xyzw: [x, y, z, w]

Pure Python, Python 3.10.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

Vec = tuple[float, float, float]
Quat = tuple[float, float, float, float]   # x, y, z, w


def _normalize(q: Quat) -> Quat:
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat_mul(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_conj(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def rotate(q: Quat, v: Vec) -> Vec:
    x, y, z, _ = quat_mul(quat_mul(q, (v[0], v[1], v[2], 0.0)), quat_conj(q))
    return (x, y, z)


def compose(p1: Vec, q1: Quat, p2: Vec, q2: Quat) -> tuple[Vec, Quat]:
    """(p1, q1) (x) (p2, q2): the second pose expressed in the first's frame."""
    r = rotate(q1, p2)
    return (p1[0] + r[0], p1[1] + r[1], p1[2] + r[2]), _normalize(quat_mul(q1, q2))


def relative(p_parent: Vec, q_parent: Quat, p_child: Vec, q_child: Quat) -> tuple[Vec, Quat]:
    """parent_T_child from world_T_parent and world_T_child."""
    qi = quat_conj(_normalize(q_parent))
    d = (p_child[0] - p_parent[0], p_child[1] - p_parent[1], p_child[2] - p_parent[2])
    return rotate(qi, d), _normalize(quat_mul(qi, q_child))


@dataclass(frozen=True)
class Placement:
    frame_id: str
    fixture: str
    position: Vec
    orientation_xyzw: Quat

    def world(self, fixture_position: Vec, fixture_orientation_xyzw: Quat) -> tuple[Vec, Quat]:
        return compose(fixture_position, fixture_orientation_xyzw, self.position, self.orientation_xyzw)


def parse_placements(data: dict[str, Any], where: str = "placements") -> dict[str, Placement]:
    default_fixture = str(data.get("fixture") or "")
    out: dict[str, Placement] = {}
    for frame_id, entry in (data.get("placements") or {}).items():
        fixture = str((entry or {}).get("fixture") or default_fixture)
        pos, ori = (entry or {}).get("position"), (entry or {}).get("orientation_xyzw")
        if not fixture or not isinstance(pos, list) or len(pos) != 3 or not isinstance(ori, list) or len(ori) != 4:
            raise ValueError(f"{where}: placement {frame_id!r} needs a fixture, position [x,y,z] and orientation_xyzw [x,y,z,w]")
        out[str(frame_id)] = Placement(str(frame_id), fixture, tuple(float(v) for v in pos), _normalize(tuple(float(v) for v in ori)))
    return out


def load_placements(repo_root: str | Path, assembly: str | None = None) -> dict[str, Placement]:
    """Every config/placements/*.yaml (or only <assembly>.yaml), merged by frame id."""
    folder = Path(repo_root) / "config" / "placements"
    files = [folder / f"{assembly}.yaml"] if assembly else sorted(folder.glob("*.yaml"))
    out: dict[str, Placement] = {}
    for path in files:
        if not path.is_file():
            continue
        out.update(parse_placements(yaml.safe_load(path.read_text(encoding="utf-8")) or {}, str(path)))
    return out


def derive_into(poses: dict[str, dict[str, Any]], placements: dict[str, Placement]) -> list[str]:
    """Add (or replace) each placement in a {id: {"position": [...],
    "orientation_xyzw": [...]}} map from its fixture's entry; returns the ids
    derived. A placement whose fixture is missing is left out."""
    done = []
    for frame_id, pl in placements.items():
        fx = poses.get(pl.fixture)
        if not fx:
            poses.pop(frame_id, None)
            continue
        p, q = pl.world(tuple(fx["position"]), tuple(fx.get("orientation_xyzw", (0.0, 0.0, 0.0, 1.0))))
        poses[frame_id] = {**{k: v for k, v in fx.items() if k not in ("position", "orientation_xyzw")},
                           "position": list(p), "orientation_xyzw": list(q), "derived_from": pl.fixture}
        done.append(frame_id)
    return done
