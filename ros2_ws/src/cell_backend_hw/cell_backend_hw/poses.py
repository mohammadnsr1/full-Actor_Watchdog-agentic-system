"""Hardware pose sources: /camera1/poses (executor) and /camera2/poses (WatchDog).

Fed by the two FoundationPose pipelines once they exist (camera1 on the
Orin, camera2 on the workstation). Until then nothing publishes and every
lookup returns None: missing evidence, never an estimate.
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
