#!/usr/bin/env bash
# The CAD -> assembly-graph tool's own venv (docs/EXECUTOR_BASELINE.md, EB2).
# Pins are prompt-to-assembly's [cad] extra and runtime deps (benchmark/pyproject.toml)
# for Python < 3.13, without its droidstack dependency (the assembly verbs do not use it).
# Needs benchmark/ (tools/fetch_benchmark.sh).
#
# Run the ingest with:
#   PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=benchmark/src tools/ingest/.venv/bin/python \
#     -m prompt_to_assembly_ingest assembly <sources dir> --out <dir> --jobs 4
#
# Checked 2026-11-24 on Python 3.12.3: fmb1 re-derived in 22 s, all verdicts PASS;
# graph, part graphs, colliders and declaration byte-identical to the shipped
# ones; Body1.usda differs only in a ~1e-17 principal-axes component (numpy
# build), which also changes tier-b/sim-plan's bundle digest.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
python3.12 -m venv "$here/.venv"
"$here/.venv/bin/pip" install -q --upgrade pip
"$here/.venv/bin/pip" install -q \
  "cadquery-ocp==7.9.3.1.1" "numpy==2.2.6" "python-fcl==0.7.0.11" "rtree==1.4.1" \
  "scipy==1.15.3" "trimesh==4.7.4" "PyYAML==6.0.3" "usd-core==26.8" \
  "mcap==1.4.0" "mcap-ros2-support==0.5.7"
echo "ingest venv ready: $here/.venv"
