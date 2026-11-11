"""Signal availability vocabulary.

A platform config lists, under `signals:`, which physical signals exist and
where they come from:

    available : the signal exists with its native meaning
    proxy     : something correlated stands in for it; `source` names it
    missing   : nothing provides it; nothing is ever published or faked for it

Consumers: the physical-events node (Milestone 5) publishes nothing for a
missing signal; WatchDog's available_evidence_sources() reports this list
verbatim; monitors that need a missing signal refuse to arm and say so.
"""

from __future__ import annotations

from typing import Any

from cell_platform_interfaces.platform_config import PlatformConfig

AVAILABLE = "available"
PROXY = "proxy"
MISSING = "missing"
STATUSES = (AVAILABLE, PROXY, MISSING)

# The signals every platform file must declare, so a forgotten entry is
# reported as a config error rather than treated as missing by accident.
REQUIRED_SIGNALS = (
    "finger_effort",
    "gripper_width",
    "grip_detected",
    "pad_contacts",
    "part_poses",
    "robot_tf",
    "sim_time",
    "camera1_frames",
    "camera2_frames",
    "camera2_depth",
)


class SignalConfigError(ValueError):
    pass


def signal_availability(platform: PlatformConfig) -> dict[str, dict[str, str]]:
    """Validated copy of the platform's `signals` block."""
    declared = platform.signals
    problems: list[str] = []
    for name in REQUIRED_SIGNALS:
        if name not in declared:
            problems.append(f"missing entry {name!r}")
    for name, entry in declared.items():
        if entry.get("status") not in STATUSES:
            problems.append(f"{name}: status {entry.get('status')!r} not one of {STATUSES}")
    if problems:
        raise SignalConfigError(f"{platform.path} signals: " + "; ".join(problems))
    return declared


def status_of(platform: PlatformConfig, signal: str) -> str:
    return signal_availability(platform).get(signal, {}).get("status", MISSING)


def is_available(platform: PlatformConfig, signal: str, allow_proxy: bool = False) -> bool:
    status = status_of(platform, signal)
    return status == AVAILABLE or (allow_proxy and status == PROXY)


def evidence_sources_report(platform: PlatformConfig) -> dict[str, Any]:
    """What the WatchDog read-only tool returns: signals plus camera roles.

    Says what exists, never what to do with it.
    """
    cameras: dict[str, Any] = {}
    for name, entry in platform.cameras.items():
        cameras[name] = {
            "role": entry.get("role", ""),
            "enabled": bool(entry.get("enabled", False)),
            "rgb_topic": (entry.get("topics") or {}).get("rgb", ""),
            "depth_topic": (entry.get("topics") or {}).get("depth", ""),
            "poses_topic": (entry.get("topics") or {}).get("poses", ""),
        }
    return {
        "platform": platform.name,
        "use_sim_time": platform.use_sim_time,
        "signals": signal_availability(platform),
        "pose_source": platform.get_optional("poses", "source", default=""),
        "cameras": cameras,
    }
