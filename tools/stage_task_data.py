#!/usr/bin/env python3
"""Stage the authoring agent's read-only task view (EB4): the labmate's /data layout.

    python3 tools/stage_task_data.py <dest> [--task fmb_assembly_2]

    <dest>/tasks/<id>/            task_data/tasks/<id>/ (tools/make_task_data.py)
    <dest>/environments/<env>/    the environment the task names
    <dest>/robots/<robot>/        the robot the environment names
    <dest>/assemblies/<name>/     benchmark/assemblies/<name>/ (the labmate's, pinned)
    <dest>/prompts/docs/          executor/prompts/baseline/docs/ (EB5), when present
    <dest>/STAGED.json            what was staged, with digests

Files are copied, never linked (the container cannot follow host links). The view holds
nothing WatchDog-side; the stager refuses if a forbidden path pattern appears.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
FORBIDDEN = ("requirements", "placements", "watchdog", "model_registry", ".cell.lock")


def _digest(root: Path) -> str:
    h = hashlib.sha256()
    for p in sorted(root.rglob("*")):
        if p.is_file():
            h.update(f"{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.relative_to(root).as_posix()}\n".encode())
    return h.hexdigest()


def stage(dest: Path, task_id: str) -> dict:
    src = REPO / "task_data"
    task = yaml.safe_load((src / "tasks" / task_id / "task.yaml").read_text())
    env = yaml.safe_load((src / "environments" / task["environment"] / "declaration.yaml").read_text())
    pieces = {
        f"tasks/{task_id}": src / "tasks" / task_id,
        f"environments/{task['environment']}": src / "environments" / task["environment"],
        **{f"robots/{r['name']}": src / "robots" / r["name"] for r in env["robots"]},
        f"assemblies/{task['assembly']}": REPO / "benchmark" / "assemblies" / task["assembly"],
    }
    docs = REPO / "executor" / "prompts" / "baseline" / "docs"
    if docs.is_dir():
        pieces["prompts/docs"] = docs
    if dest.exists():
        shutil.rmtree(dest)
    for rel, path in pieces.items():
        if not path.is_dir():
            raise SystemExit(f"missing {path} (run tools/make_task_data.py / tools/fetch_benchmark.sh)")
        shutil.copytree(path, dest / rel, symlinks=False, ignore=shutil.ignore_patterns("__pycache__"))
    bad = [str(p.relative_to(dest)) for p in dest.rglob("*") if any(w in p.name.lower() for w in FORBIDDEN)]
    if bad:
        shutil.rmtree(dest)
        raise SystemExit(f"refusing: WatchDog-side names in the staged view: {bad[:10]}")
    record = {"task": task_id, "pieces": {rel: {"from": str(path.relative_to(REPO)), "sha256": _digest(dest / rel)}
                                          for rel, path in pieces.items()}}
    (dest / "STAGED.json").write_text(json.dumps(record, indent=2))
    return record


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("dest")
    ap.add_argument("--task", default="fmb_assembly_2")
    args = ap.parse_args()
    print(json.dumps(stage(Path(args.dest).resolve(), args.task), indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
