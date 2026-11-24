"""Reference views of a finished assembly, for both agents.

    config/assembly_views/<assembly>/
        *.png | *.jpg | *.jpeg      pictures of the finished assembly, any views
        views.yaml                  optional: {<file name>: <caption>} and/or
                                    {"notes": <text about the set>}

Provided by Mohammad (photos or renders). The executor's authoring agent gets
them through get_assembly_views (executor/authoring_mcp) and the WatchDog
through assembly_reference; both see the same pictures. Placement poses are
never in this folder (they are WatchDog-only, config/placements/).

load_views() returns each picture ready to send: downscaled to MAX_SIDE_PX
(JPEG) when Pillow is importable, else the original bytes when under
MAX_RAW_BYTES, else skipped with the reason. Pure Python + optional Pillow.
"""

from __future__ import annotations

import io
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

EXTENSIONS = {".png": "png", ".jpg": "jpeg", ".jpeg": "jpeg"}
MAX_SIDE_PX = 1024
MAX_RAW_BYTES = 1_500_000


@dataclass
class View:
    name: str
    caption: str
    format: str
    data: bytes
    note: str = ""


def views_dir(repo_root: str | Path, assembly: str) -> Path:
    return Path(repo_root) / "config" / "assembly_views" / assembly


def _shrink(raw: bytes, fmt: str) -> tuple[bytes, str, str]:
    try:
        from PIL import Image  # system Python has it; the bare venv may not
    except Exception:
        if len(raw) <= MAX_RAW_BYTES:
            return raw, fmt, "original"
        raise ValueError(f"{len(raw)} bytes and no Pillow to downscale; keep views under {MAX_RAW_BYTES} bytes")
    img = Image.open(io.BytesIO(raw))
    img.load()
    if max(img.size) > MAX_SIDE_PX:
        img.thumbnail((MAX_SIDE_PX, MAX_SIDE_PX))
    out = io.BytesIO()
    img.convert("RGB").save(out, format="JPEG", quality=88)
    return out.getvalue(), "jpeg", f"{img.size[0]}x{img.size[1]}"


def load_views(repo_root: str | Path, assembly: str) -> tuple[list[View], dict[str, Any]]:
    """(views, info): info carries notes, the folder, and any file skipped and why."""
    folder = views_dir(repo_root, assembly)
    meta: dict[str, Any] = {}
    if (folder / "views.yaml").is_file():
        meta = yaml.safe_load((folder / "views.yaml").read_text(encoding="utf-8")) or {}
    info: dict[str, Any] = {"folder": str(folder), "notes": str(meta.get("notes", "") or ""), "skipped": {}}
    views: list[View] = []
    if not folder.is_dir():
        info["skipped"]["*"] = "no reference views provided for this assembly yet"
        return views, info
    for path in sorted(folder.iterdir()):
        fmt = EXTENSIONS.get(path.suffix.lower())
        if fmt is None:
            continue
        try:
            data, out_fmt, note = _shrink(path.read_bytes(), fmt)
        except Exception as exc:  # noqa: BLE001
            info["skipped"][path.name] = str(exc)
            continue
        views.append(View(path.name, str(meta.get(path.name, "") or ""), out_fmt, data, note))
    if not views and not info["skipped"]:
        info["skipped"]["*"] = f"no pictures in {folder} yet"
    return views, info
