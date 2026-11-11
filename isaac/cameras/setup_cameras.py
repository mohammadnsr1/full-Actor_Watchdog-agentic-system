"""Sim cameras: add camera1 (wrist) and check camera2 (table) against
config/cameras/sim.yaml. Run inside Kit (Script Editor, or through the kit
MCP bridge's run_python) with the stage open, then SAVE the stage so the
prims and graphs persist.

camera2 already exists in the stage as /World/Camera with /Graph/ROS_Camera
publishing rgb, depth and camera_info; this script only verifies that its
topics and frame match the config and reports any mismatch.

camera1 does not exist yet (BUILD_PLAN q5). This script creates it under the
configured parent prim at the configured pose and builds a ROS 2 graph of the
same shape as /Graph/ROS_Camera. It refuses while the config still carries
the TODO values (empty parent_frame, zero size).
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import yaml

REPO = Path(os.environ.get("CELL_REPO_ROOT", "")).expanduser() if os.environ.get("CELL_REPO_ROOT") else Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from isaac.common import kit_api as K  # noqa: E402

# Prim path of the wrist link to parent camera1 to, read from the stage
# inventory of 2026-11-11. TODO(mohammad): confirm or change (tcp vs wrist_3_link).
DEFAULT_CAMERA1_PARENT_PRIM = "/World/ur5e/wrist_3_link"
CAMERA2_PRIM = "/World/Camera"
CAMERA2_GRAPH = "/Graph/ROS_Camera"
CAMERA1_GRAPH = "/Graph/ROS_Camera1"


def load_cameras() -> dict:
    with (REPO / "config" / "cameras" / "sim.yaml").open("r", encoding="utf-8") as handle:
        return (yaml.safe_load(handle) or {}).get("cameras", {})


def check_camera2(cfg: dict) -> dict:
    topics = cfg.get("topics", {})
    expected = {
        f"{CAMERA2_GRAPH}/RGBPublish": ("inputs:topicName", topics.get("rgb")),
        f"{CAMERA2_GRAPH}/DepthPublish": ("inputs:topicName", topics.get("depth")),
        f"{CAMERA2_GRAPH}/CameraInfoPublish": ("inputs:topicName", topics.get("camera_info")),
        f"{CAMERA2_GRAPH}/RGBPublish#frame": ("inputs:frameId", cfg.get("optical_frame")),
    }
    report = {"prim_exists": K.prim_exists(CAMERA2_PRIM), "mismatches": [], "pose": None}
    for node, (attr, want) in expected.items():
        node_path = node.split("#")[0]
        have = K.graph_node_value(node_path, attr)
        if want and str(have) != str(want):
            report["mismatches"].append({"node": node_path, "attr": attr, "stage": have, "config": want})
    try:
        report["pose"] = K.world_pose(CAMERA2_PRIM)
    except Exception as exc:  # noqa: BLE001
        report["pose_error"] = str(exc)
    return report


def setup_camera1(cfg: dict) -> dict:
    if not cfg.get("enabled", False):
        return {"skipped": "camera1 is disabled in config/cameras/sim.yaml; enable it once the pose is known"}
    parent = str(cfg.get("parent_frame") or "")
    width, height = int(cfg.get("width", 0) or 0), int(cfg.get("height", 0) or 0)
    if not parent or width <= 0 or height <= 0:
        return {"error": "camera1 config still has TODO values (parent_frame, width, height)"}
    parent_prim = str(cfg.get("parent_prim") or DEFAULT_CAMERA1_PARENT_PRIM)
    if not K.prim_exists(parent_prim):
        return {"error": f"parent prim {parent_prim} not on the stage"}
    cam_path = f"{parent_prim}/camera1"
    topics = cfg.get("topics", {})
    created = K.create_camera(cam_path, cfg.get("xyz", [0, 0, 0]), cfg.get("quat_xyzw", [0, 0, 0, 1]), cfg.get("focal_length_mm"))
    graph = K.build_ros2_camera_graph(
        CAMERA1_GRAPH, created, width, height, str(cfg.get("optical_frame", "camera1_optical_frame")),
        str(topics.get("rgb")), topics.get("depth"), str(topics.get("camera_info")),
    )
    return {"camera": created, "graph": graph, "note": "save the stage to persist; the optical-frame TF is published by camera_tf.launch.py from cameras config"}


def main() -> dict:
    cams = load_cameras()
    out = {"camera2": check_camera2(cams.get("camera2", {})), "camera1": setup_camera1(cams.get("camera1", {}))}
    print(out)
    return out


if __name__ == "__main__" or "omni" in sys.modules:
    main()
