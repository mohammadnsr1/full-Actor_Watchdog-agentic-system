#!/usr/bin/env python3
"""Derive the plan-free task file from a full (plan-given) executor task file.

    python3 tools/derive_plan_free_task.py config/tasks/fmb_assembly_2.yaml
        -> writes config/tasks/fmb_assembly_2.plan_free.yaml

The plan-free condition of an authoring session (runner/run_authoring.sh
--condition plan_free) gives the agent the goal and constraints and the object
descriptions, never the plan. KEEP lists the top-level fields carried over
verbatim except for the plan references listed in REDACT; everything else (step_plan, steps: sequence, grasp offsets, approach
distances, insertion offsets, widths, motion scaling) is dropped. Object
geometry and grasp force reach the planner separately, from
config/parts/executor_parts.yaml.

The output is committed and checked against a fresh derivation by
tools/test_derive_plan_free_task.py, so an edit to the full file that is not
re-derived fails the tests.
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

KEEP = ("name", "instruction_spec", "object_ids", "run_notes")

# Plan references inside kept text, removed for the plan-free condition:
# (field, exact text in the full file, replacement). A pattern that no longer
# matches fails the derivation, so an edit to the full file cannot silently
# re-introduce a plan reference.
REDACT = {
    "fmb_assembly_2": [
        ("instruction_spec",
         "grasped securely on the way there: measured width at\n    plateau matched the step's grasp_width_m within tolerance; no slippage in hand;",
         "grasped securely on the way there: no slippage in hand;"),
    ],
}


def _field(key: str, value: object) -> str:
    """One top-level field; multi-line text as a block literal, verbatim
    (PyYAML refuses block style for text with trailing spaces)."""
    if isinstance(value, str) and "\n" in value:
        body = value.rstrip("\n")
        tail = len(value) - len(body)
        chomp = "-" if tail == 0 else ("" if tail == 1 else "+")
        lines = [("  " + ln) if ln else "" for ln in body.split("\n")] + [""] * max(0, tail - 1)
        return f"{key}: |{chomp}\n" + "\n".join(lines) + "\n"
    return yaml.dump({key: value}, sort_keys=False, allow_unicode=True, width=1000)


def out_path(full: Path) -> Path:
    return full.with_name(f"{full.stem}.plan_free.yaml")


def derive(full: Path) -> str:
    data = yaml.safe_load(full.read_text(encoding="utf-8")) or {}
    missing = [k for k in ("name", "instruction_spec", "object_ids") if k not in data]
    if missing:
        raise ValueError(f"{full}: missing {missing}")
    kept = {k: v for k, v in data.items() if k in KEEP}
    for field, old, new in REDACT.get(data["name"], []):
        if old not in kept.get(field, ""):
            raise ValueError(f"{full}: redaction for {field} no longer matches; update REDACT: {old!r}")
        kept[field] = kept[field].replace(old, new)
    dropped = sorted(k for k in data if k not in KEEP)
    header = (
        f"# PLAN-FREE executor task for {data['name']}: GENERATED, do not edit.\n"
        f"# Derived from {full.name} by tools/derive_plan_free_task.py: keeps {', '.join(KEEP)};\n"
        f"# drops {', '.join(dropped) or 'nothing'}; redacts {len(REDACT.get(data['name'], []))} plan reference(s) in the kept text\n"
        f"# (REDACT in the tool). Executor-only, like the full file.\n\n"
    )
    text = header + "".join(_field(k, v) for k, v in kept.items())
    if yaml.safe_load(text) != kept:
        raise AssertionError("derived file does not round-trip")
    return text


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    full = Path(sys.argv[1])
    out = out_path(full)
    out.write_text(derive(full), encoding="utf-8")
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
