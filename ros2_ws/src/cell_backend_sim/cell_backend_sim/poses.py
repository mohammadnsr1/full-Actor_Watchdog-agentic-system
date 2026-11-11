"""Sim pose sources: two TopicPoseSource instances, one per camera role.

`poses.source` in platforms/sim.yaml says who feeds the topics:
    ground_truth    cell_backend_sim ground_truth_pose_publisher (Isaac TF -> topics)
    foundationpose  a perception pipeline on the sim cameras (not available yet)
Either way the executor reads `poses.executor_topic` (camera1) and WatchDog
reads `poses.watchdog_topic` (camera2); the two instances never share state.
"""

from __future__ import annotations

from typing import Any

from cell_platform_interfaces.platform_config import PlatformConfig
from cell_platform_interfaces.topic_poses import TopicPoseSource


def make_pose_sources(node: Any, platform: PlatformConfig, callback_group: Any = None) -> tuple[TopicPoseSource, TopicPoseSource]:
    source = str(platform.get("poses", "source"))
    executor = TopicPoseSource(node, platform.get("poses", "executor_topic"), camera="camera1", source=source, callback_group=callback_group)
    watchdog = TopicPoseSource(node, platform.get("poses", "watchdog_topic"), camera="camera2", source=source, callback_group=callback_group)
    return executor, watchdog
