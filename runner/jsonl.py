"""JSON Lines for everything the runner and its processes write to records/.

    JsonlLog(path).write(event, **fields)   one JSON object per line, with
                                            wall_time_iso; thread-safe, append
    StderrPump(path, source)                a write end to hand a subprocess as
                                            stderr; a thread wraps every line it
                                            writes as {"wall_time_iso", "source", "line"}
    read_jsonl(path, tail=None)             the records back, skipping a torn last line

Pure Python, Python 3.10.
"""

from __future__ import annotations

import json
import os
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


class JsonlLog:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def write(self, event: str, **fields: Any) -> dict[str, Any]:
        record = {"wall_time_iso": utc_iso(), "event": event, **fields}
        line = json.dumps(record, default=str)
        with self._lock, self.path.open("a", encoding="utf-8") as f:
            f.write(line + "\n")
        return record


class StderrPump:
    """Popen(..., stderr=pump.fileno()) then pump.started(); close() after the process exits."""

    def __init__(self, path: str | Path, source: str) -> None:
        self.log = JsonlLog(path)
        self.source = source
        self._r, self._w = os.pipe()
        self._thread = threading.Thread(target=self._run, name=f"stderr-{source}", daemon=True)
        self._thread.start()
        self._w_file: Any = None

    def fileno(self) -> int:
        return self._w

    def file(self) -> Any:
        """A text file object over the write end (for APIs that want one)."""
        if self._w_file is None:
            self._w_file = os.fdopen(os.dup(self._w), "w", buffering=1)
        return self._w_file

    def started(self) -> None:
        """Close the parent's copy of the write end once the child holds it,
        so the reader sees EOF when the child exits."""
        if self._w >= 0:
            os.close(self._w)
            self._w = -1

    def _run(self) -> None:
        with os.fdopen(self._r, "rb") as r:
            for raw in r:
                line = raw.decode("utf-8", errors="replace").rstrip("\n")
                if line:
                    self.log.write("stderr", source=self.source, line=line)

    def close(self, timeout_s: float = 5.0) -> None:
        if self._w_file is not None:
            self._w_file.close()
            self._w_file = None
        self.started()
        self._thread.join(timeout_s)


def read_jsonl(path: str | Path, tail: int | None = None) -> list[dict[str, Any]]:
    p = Path(path)
    if not p.is_file():
        return []
    lines = p.read_text(encoding="utf-8", errors="replace").splitlines()
    if tail is not None:
        lines = lines[-tail:]
    out = []
    for line in lines:
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return out
