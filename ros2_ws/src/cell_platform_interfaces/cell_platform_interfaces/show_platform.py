"""`ros2 run cell_platform_interfaces show_platform [sim|hardware]`: print the
resolved platform config and its signal list as JSON. Read-only."""

from __future__ import annotations

import json
import sys

from cell_platform_interfaces.platform_config import load_platform_config
from cell_platform_interfaces.signals import evidence_sources_report


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    name = args[0] if args else None
    try:
        platform = load_platform_config(name)
        report = {
            "platform": platform.name,
            "path": str(platform.path),
            "cameras_path": str(platform.cameras_path),
            "evidence_sources": evidence_sources_report(platform),
            "motion": platform.section("motion"),
            "gripper": {k: v for k, v in platform.section("gripper").items() if k != "urscript"},
            "topics": platform.section("topics"),
            "services": platform.section("services"),
            "frames": platform.section("frames"),
        }
    except Exception as exc:  # noqa: BLE001
        print(f"show_platform: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
