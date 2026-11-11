#!/usr/bin/env python3
"""Milestone 3/4 live check: plan with the planning service, execute through
the sim motion backend, cancel mid-motion through the platform cancel service.

    source ros2_ws/install/setup.bash
    python3 runner/tools/plan_execute_cancel_check.py --goal 0.0 -1.57 1.57 -1.57 -1.57 0.0 --cancel-after 0.6

Steps and what to expect:
  1. plan_to_joints      -> ok, backend, point count, duration; the arm must NOT move
  2. execute (bridge)    -> the arm starts moving
  3. cancel_all after N s (platform motion.cancel_service) -> goals_cancelled=1
  4. execute result      -> state 'canceled' and the arm stopped where it was
Pass --no-cancel to let the motion finish (state 'succeeded').
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cell_backend_sim.motion import SimMotion
from cell_planner_msgs.srv import PlanToJoints
from cell_platform_interfaces.platform_config import load_platform_config


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal", nargs=6, type=float, required=True, help="six joint positions (rad)")
    parser.add_argument("--group", default="ur_manipulator")
    parser.add_argument("--cancel-after", type=float, default=0.6, help="seconds after the goal is sent")
    parser.add_argument("--no-cancel", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    platform = load_platform_config()
    node = Node("plan_execute_cancel_check")
    group = ReentrantCallbackGroup()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        ns = platform.service("planning_namespace")
        client = node.create_client(PlanToJoints, f"{ns}/plan_to_joints", callback_group=group)
        if not client.wait_for_service(timeout_sec=5.0):
            print(f"planning service {ns}/plan_to_joints not available", file=sys.stderr)
            return 2
        req = PlanToJoints.Request()
        req.planning_group = args.group
        req.joint_names = platform.joints("arm")
        req.joint_positions = list(args.goal)
        future = client.call_async(req)
        while not future.done():
            time.sleep(0.01)
        resp = future.result()
        last = resp.trajectory.points[-1].time_from_start if resp.trajectory.points else None
        print(
            f"[1] plan: ok={resp.ok} backend={resp.report.backend} planning_time_s={resp.report.planning_time_s:.3f} "
            f"failure_code={resp.report.failure_code!r} points={len(resp.trajectory.points)} "
            f"duration_s={(last.sec + last.nanosec / 1e9) if last else 0:.2f}"
        )
        if not resp.ok:
            print(f"    reason: {resp.report.failure_reason}")
            return 1

        motion = SimMotion(node, platform, group)
        time.sleep(0.5)  # let the joint-state subscription warm up
        js = motion.joint_state(max_age_s=2.0)
        print(f"    joint_states fresh: {js is not None}")

        result_box: dict = {}

        def run() -> None:
            result_box["result"] = motion.execute(resp.trajectory)

        worker = threading.Thread(target=run, daemon=True)
        worker.start()
        print(f"[2] goal sent to {motion.action_name}")
        if not args.no_cancel:
            time.sleep(args.cancel_after)
            cancel = motion.cancel_all()
            print(f"[3] cancel_all via {cancel.target}: ok={cancel.ok} goals_cancelled={cancel.goals_cancelled} ({cancel.detail})")
        worker.join(timeout=60.0)
        result = result_box.get("result")
        if result is None:
            print("[4] execute did not return within 60 s", file=sys.stderr)
            return 1
        print(f"[4] execute result: state={result.state} error_code={result.error_code} detail={result.detail!r} elapsed_s={result.elapsed_s:.2f}")
        expected = "succeeded" if args.no_cancel else "canceled"
        print("PASS" if result.state == expected else f"UNEXPECTED (wanted {expected})")
        return 0 if result.state == expected else 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
