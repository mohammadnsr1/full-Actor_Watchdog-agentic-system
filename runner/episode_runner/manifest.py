"""Episode manifest helpers: hash every input so an episode can be tied to the
exact files it ran with. Pure Python, Python 3.10."""

from __future__ import annotations

import hashlib
import subprocess
from pathlib import Path
from typing import Any

_SKIP_DIRS = {"__pycache__", ".pytest_cache", ".git"}
_SKIP_SUFFIXES = {".pyc", ".pyo", ".log"}


def sha256_file(path: str | Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def tree_hash(root: str | Path) -> dict[str, Any]:
    """sha256 over (relative path, file sha256) of every file, sorted; caches skipped."""
    root = Path(root)
    entries = []
    for p in sorted(root.rglob("*")):
        if not p.is_file() or any(part in _SKIP_DIRS for part in p.relative_to(root).parts) or p.suffix in _SKIP_SUFFIXES:
            continue
        entries.append((p.relative_to(root).as_posix(), sha256_file(p)))
    h = hashlib.sha256()
    for rel, digest in entries:
        h.update(f"{rel}\0{digest}\n".encode("utf-8"))
    return {"sha256": h.hexdigest(), "files": len(entries), "listing": dict(entries)}


def file_record(path: str | Path, repo: str | Path) -> dict[str, Any]:
    p = Path(path)
    rel = p.resolve().relative_to(Path(repo).resolve()).as_posix() if p.resolve().is_relative_to(Path(repo).resolve()) else str(p)
    return {"path": rel, "sha256": sha256_file(p) if p.is_file() else None}


def git_state(repo: str | Path) -> dict[str, Any]:
    def run(*args: str) -> str:
        try:
            return subprocess.run(["git", *args], cwd=str(repo), capture_output=True, text=True, timeout=10).stdout.strip()
        except Exception:
            return ""

    status = run("status", "--porcelain")
    return {"commit": run("rev-parse", "HEAD") or None, "branch": run("rev-parse", "--abbrev-ref", "HEAD") or None,
            "dirty": bool(status), "dirty_files": len(status.splitlines()) if status else 0}
