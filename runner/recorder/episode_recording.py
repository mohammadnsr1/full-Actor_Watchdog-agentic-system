"""Per-episode recording (Milestone 6): the MCAP bag and the frame recorder,
started and stopped together, into records/<episode_id>/.

    from runner.recorder.episode_recording import EpisodeRecording
    rec = EpisodeRecording(episode_dir, platform, cell_config).start()
    ...
    rec.stop()

Layout written:
    <episode_dir>/bag/               rosbag2 MCAP (-s mcap on both distros)
    <episode_dir>/frames/<camera>/   JPEG frames + frames.jsonl (wall_time_iso, header_stamp = sim time)
    <episode_dir>/recording.json     what was recorded, when, and where

Topics: the platform's core topics plus the Milestone 5 and 6 channels, read
from config/cell.yaml recording.bag.extra_topic_keys (keys into the platform
topics/poses blocks), so nothing is named here. Works on hardware: no sim
time is assumed; the frame index simply carries the message stamps.
Python 3.10 safe.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from cell_platform_interfaces.compat import bag_record_command
from cell_platform_interfaces.platform_config import PlatformConfig

FRAME_RECORDER = Path(__file__).resolve().parent / "frame_recorder.py"


def _iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


def bag_topics(platform: PlatformConfig, cell_config: dict[str, Any]) -> list[str]:
    rec = dict((cell_config.get("recording") or {}).get("bag") or {})
    topics: list[str] = []
    for key in rec.get("topic_keys") or []:
        value = platform.get_optional("topics", str(key), default="")
        if value:
            topics.append(str(value))
    for key in rec.get("pose_topic_keys") or []:
        value = platform.get_optional("poses", str(key), default="")
        if value:
            topics.append(str(value))
    seen: set[str] = set()
    return [t for t in topics if not (t in seen or seen.add(t))]


class EpisodeRecording:
    def __init__(self, episode_dir: str | os.PathLike[str], platform: PlatformConfig, cell_config: dict[str, Any], env: dict[str, str] | None = None) -> None:
        self.dir = Path(episode_dir)
        self.platform = platform
        self.cell = cell_config
        self.env = dict(env or os.environ)
        self.topics = bag_topics(platform, cell_config)
        frames_cfg = dict((cell_config.get("recording") or {}).get("frames") or {})
        self.frames_enabled = bool(frames_cfg.get("enabled", True))
        self.frame_topic = str(frames_cfg.get("image_topic") or "") or platform.camera_topic("recorder", "rgb")
        self.frame_params = {
            "camera_name": str(frames_cfg.get("camera_name", "recorder")),
            "rate_hz": float(frames_cfg.get("rate_hz", 10.0)),
            "jpeg_quality": int(frames_cfg.get("jpeg_quality", 95)),
            "max_width": int(frames_cfg.get("max_width", 1280)),
        }
        self.video_enabled = bool(((cell_config.get("recording") or {}).get("video") or {}).get("enabled", False))
        self._bag: subprocess.Popen | None = None
        self._frames: subprocess.Popen | None = None
        self.started_wall_time_iso: str | None = None
        self.stopped_wall_time_iso: str | None = None

    # -- commands (pure, testable) ----------------------------------------------

    def bag_command(self) -> list[str]:
        return bag_record_command(self.dir / "bag", self.topics)

    def frames_command(self) -> list[str]:
        cmd = [sys.executable, str(FRAME_RECORDER), "--ros-args",
               "-p", f"out_dir:={self.dir / 'frames'}", "-p", f"image_topic:={self.frame_topic}"]
        for key, value in self.frame_params.items():
            cmd += ["-p", f"{key}:={value}"]
        return cmd

    # -- lifecycle --------------------------------------------------------------------

    def start(self) -> "EpisodeRecording":
        self.dir.mkdir(parents=True, exist_ok=True)
        if not self.topics:
            raise RuntimeError("no bag topics resolved; check config/cell.yaml recording.bag and the platform topics")
        self._bag = subprocess.Popen(self.bag_command(), env=self.env, stdout=subprocess.DEVNULL, stderr=open(self.dir / "bag.stderr.log", "ab"))
        if self.frames_enabled:
            self._frames = subprocess.Popen(self.frames_command(), env=self.env, stdout=subprocess.DEVNULL, stderr=open(self.dir / "frames.stderr.log", "ab"))
        self.started_wall_time_iso = _iso()
        self._write_manifest(status="recording")
        return self

    def stop(self, timeout_s: float = 15.0) -> dict[str, Any]:
        for proc in (self._frames, self._bag):
            if proc is None or proc.poll() is not None:
                continue
            try:
                proc.send_signal(signal.SIGINT)
            except OSError:
                pass
        deadline = time.monotonic() + timeout_s
        for proc in (self._frames, self._bag):
            if proc is None:
                continue
            while proc.poll() is None and time.monotonic() < deadline:
                time.sleep(0.1)
            if proc.poll() is None:
                proc.kill()
        self.stopped_wall_time_iso = _iso()
        video = self._encode_video() if self.video_enabled else None
        return self._write_manifest(status="stopped", video=video)

    def _encode_video(self) -> dict[str, Any]:
        """Optional, off by default: ffmpeg over the JPEG frames of each camera."""
        out: dict[str, Any] = {}
        for cam_dir in (self.dir / "frames").glob("*"):
            if not cam_dir.is_dir():
                continue
            target = self.dir / f"{cam_dir.name}.mp4"
            cmd = ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(self.frame_params["rate_hz"]),
                   "-pattern_type", "glob", "-i", str(cam_dir / "*.jpg"), "-pix_fmt", "yuv420p", str(target)]
            try:
                subprocess.run(cmd, check=True, timeout=600)
                out[cam_dir.name] = str(target)
            except Exception as exc:  # noqa: BLE001
                out[cam_dir.name] = f"failed: {exc}"
        return out

    def _write_manifest(self, status: str, video: dict[str, Any] | None = None) -> dict[str, Any]:
        manifest = {
            "status": status,
            "platform": self.platform.name,
            "use_sim_time": self.platform.use_sim_time,
            "started_wall_time_iso": self.started_wall_time_iso,
            "stopped_wall_time_iso": self.stopped_wall_time_iso,
            "bag": {"dir": str(self.dir / "bag"), "storage": "mcap", "topics": self.topics},
            "frames": {"dir": str(self.dir / "frames"), "topic": self.frame_topic, **self.frame_params} if self.frames_enabled else None,
            "video": video,
        }
        self.dir.mkdir(parents=True, exist_ok=True)
        (self.dir / "recording.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        return manifest
