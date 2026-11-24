#!/usr/bin/env python3
"""Measure an assembly's placements relative to its fixture, once.

Run while the simulator still publishes the placement frames (cell up, Isaac
playing, ground-truth pose publisher on):

    source ros2_ws/install/setup.bash
    python3 tools/measure_placements.py fmb_assembly_2            # writes the file
    python3 tools/measure_placements.py fmb_assembly_2 --dry-run  # prints it

For every acceptance part in config/requirements/<assembly>.yaml it averages
the fixture's and the placement frame's pose on the WatchDog pose topic
(camera2, sim ground truth) over --samples messages and writes
fixture_T_placement to config/placements/<assembly>.yaml
(cell_platform_interfaces.placements). From then on the WatchDog, the
physical-event node and the runner's checker derive the placements from the
fixture's observed pose, and the simulator no longer needs to publish them.
The file is WatchDog-side: the executor's settings deny it.
"""

from __future__ import annotations

import argparse
import datetime
import sys
import threading
import time
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("assembly")
    ap.add_argument("--platform", default="sim")
    ap.add_argument("--samples", type=int, default=20)
    ap.add_argument("--timeout-s", type=float, default=20.0)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    from cell_platform_interfaces.placements import relative
    from cell_platform_interfaces.platform_config import load_platform_config
    from cell_platform_interfaces.topic_poses import TopicPoseSource

    req = yaml.safe_load((REPO / "config" / "requirements" / f"{args.assembly}.yaml").read_text()) or {}
    acc = req.get("acceptance") or {}
    fixtures = list(acc.get("fixtures") or [])
    if len(fixtures) != 1:
        print(f"expected exactly one fixture in acceptance.fixtures, got {fixtures}", file=sys.stderr)
        return 2
    fixture = fixtures[0]
    frames = {pid: str(spec["placement_frame"]) for pid, spec in (acc.get("parts") or {}).items()}

    platform = load_platform_config(args.platform)
    rclpy.init()
    node = Node("measure_placements")
    src = TopicPoseSource(node, platform.get("poses", "watchdog_topic"), camera="camera2", source="measure")
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    samples: dict[str, list] = {f: [] for f in frames.values()}
    deadline = time.monotonic() + args.timeout_s
    last = -1
    try:
        while time.monotonic() < deadline and min(len(v) for v in samples.values()) < args.samples:
            if src.message_count != last:
                last = src.message_count
                fx = src.latest(fixture, max_age_s=1.0)
                for f in frames.values():
                    obs = src.latest(f, max_age_s=1.0)
                    if fx is not None and obs is not None:
                        samples[f].append(relative(fx.position, fx.orientation_xyzw, obs.position, obs.orientation_xyzw))
            time.sleep(0.05)
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

    missing = [f for f, v in samples.items() if not v]
    if missing:
        print(f"no samples of {missing} with {fixture} on {platform.get('poses', 'watchdog_topic')}: "
              "are the placement frames still published (and the cell up)?", file=sys.stderr)
        return 2
    placements = {}
    for f, v in samples.items():
        n = len(v)
        pos = [round(sum(p[0][i] for p in v) / n, 6) for i in range(3)]
        q0 = v[0][1]
        qs = [q if sum(a * b for a, b in zip(q, q0)) >= 0 else tuple(-c for c in q) for _, q in v]
        q = [sum(x[i] for x in qs) / n for i in range(4)]
        norm = sum(c * c for c in q) ** 0.5
        placements[f] = {"position": pos, "orientation_xyzw": [round(c / norm, 6) for c in q]}
    doc = {
        "assembly": args.assembly,
        "fixture": fixture,
        "measured": {"wall_time_iso": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                     "tool": "tools/measure_placements.py", "platform": args.platform,
                     "topic": str(platform.get("poses", "watchdog_topic")), "samples_per_frame": min(len(v) for v in samples.values())},
        "placements": placements,
    }
    header = (f"# Placements of {args.assembly} relative to its fixture ({fixture}): WATCHDOG SIDE ONLY.\n"
              "# Measured by tools/measure_placements.py; read by cell_platform_interfaces.placements.\n"
              "# The executor's settings deny this file.\n")
    text = header + yaml.safe_dump(doc, sort_keys=False)
    if args.dry_run:
        print(text)
        return 0
    out = REPO / "config" / "placements" / f"{args.assembly}.yaml"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text)
    print(f"wrote {out} ({len(placements)} placements relative to {fixture})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
