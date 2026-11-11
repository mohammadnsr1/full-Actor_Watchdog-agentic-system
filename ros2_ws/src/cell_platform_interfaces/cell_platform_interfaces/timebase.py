"""Wall-clock stamps. wall_time_iso is the only cross-process ordering field."""

from __future__ import annotations

import time
from datetime import datetime, timezone


def wall_ns() -> int:
    return time.time_ns()


def wall_time_iso(ns: int | None = None) -> str:
    """UTC ISO-8601 with microseconds and a trailing Z, e.g. 2026-11-10T14:03:11.123456Z."""
    value = wall_ns() if ns is None else int(ns)
    stamp = datetime.fromtimestamp(value / 1e9, tz=timezone.utc)
    return stamp.strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


def parse_wall_time_iso(text: str) -> int:
    """Inverse of wall_time_iso, in nanoseconds. Accepts a trailing Z (Python 3.10 safe)."""
    cleaned = text.strip()
    if cleaned.endswith("Z"):
        cleaned = cleaned[:-1] + "+00:00"
    parsed = datetime.fromisoformat(cleaned)
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return int(parsed.timestamp() * 1_000_000_000)
