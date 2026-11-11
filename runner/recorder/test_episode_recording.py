"""Command construction and manifest, without launching anything."""

from __future__ import annotations

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from cell_platform_interfaces.platform_config import load_platform_config  # noqa: E402
from runner.recorder.episode_recording import EpisodeRecording, bag_topics  # noqa: E402

import yaml  # noqa: E402


def _cell():
    return yaml.safe_load((REPO / "config" / "cell.yaml").read_text(encoding="utf-8"))


def test_bag_topics_resolve_from_platform():
    topics = bag_topics(load_platform_config("sim", REPO), _cell())
    assert "/joint_states" in topics and "/cell/physical_events" in topics and "/camera2/poses" in topics
    assert len(topics) == len(set(topics))


def test_hardware_drops_missing_topics():
    topics = bag_topics(load_platform_config("hardware", REPO), _cell())
    assert "/cell/pad_contacts" not in topics and "/joint_states" in topics


def test_commands_and_manifest(tmp_path: Path):
    rec = EpisodeRecording(tmp_path / "ep", load_platform_config("sim", REPO), _cell())
    bag = rec.bag_command()
    assert bag[:5] == ["ros2", "bag", "record", "-s", "mcap"] and str(tmp_path / "ep" / "bag") in bag
    frames = rec.frames_command()
    assert "frame_recorder.py" in frames[1] and "image_topic:=/recorder_rgb" in frames
    manifest = rec._write_manifest(status="test")
    assert json.loads((tmp_path / "ep" / "recording.json").read_text())["bag"]["storage"] == "mcap"
    assert manifest["frames"]["topic"] == "/recorder_rgb"
