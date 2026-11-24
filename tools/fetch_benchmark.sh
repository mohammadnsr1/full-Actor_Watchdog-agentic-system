#!/usr/bin/env bash
# Rebuild benchmark/ (git-ignored) from the labmate's prompt-to-assembly clone
# at the pinned commit: only the pieces the executor baseline uses
# (docs/EXECUTOR_BASELINE.md, EB1). Tracked files only, byte-identical.
#
#   tools/fetch_benchmark.sh            # refuses if benchmark/ exists
#   tools/fetch_benchmark.sh --replace  # deletes benchmark/ first
#
# P2A_CLONE overrides the clone path. Writes benchmark/UPSTREAM.json (commit,
# paths, file count, tree digest).
set -euo pipefail

COMMIT=f36678cb4029e514fca5bdbf784b8c7c7873e1fc
CLONE="${P2A_CLONE:-/home/mohammad/prompt-to-assembly}"
PATHS=(
  assemblies/fmb1                          # worked example of the graph format; possible second task
  tasks/fmb1_assemble_ur5e_rg2_flat_gen    # template for this cell's task bundle
  prompts                                  # source for the ported prompts (EB5)
  src/prompt_to_assembly_ingest            # CAD -> assembly graph (EB2)
  src/droidstack_assembly_checker          # imported by the ingest (symmetry math); imports no DroidStack
  src/assembly_resolver                    # planner-side (object, feature) -> frame
  LICENSE THIRD_PARTY_NOTICES.md           # Apache-2.0 and notices
  pyproject.toml                           # the [cad] pins the ingest venv installs
)

repo="$(cd "$(dirname "$0")/.." && pwd)"
dst="$repo/benchmark"

if [ -e "$dst" ]; then
  if [ "${1:-}" = "--replace" ]; then rm -rf "$dst"; else
    echo "benchmark/ exists; pass --replace to rebuild it" >&2; exit 1; fi
fi
git -C "$CLONE" cat-file -e "$COMMIT^{commit}" || {
  echo "commit $COMMIT not in $CLONE (git -C $CLONE fetch)" >&2; exit 1; }

mkdir -p "$dst"
git -C "$CLONE" archive "$COMMIT" "${PATHS[@]}" | tar -x -C "$dst"

python3 - "$dst" "$COMMIT" "$CLONE" "${PATHS[@]}" <<'EOF'
import hashlib, json, pathlib, sys
dst, commit, clone, paths = pathlib.Path(sys.argv[1]), sys.argv[2], sys.argv[3], sys.argv[4:]
files = sorted(p for p in dst.rglob("*") if p.is_file())
lines = "".join(f"{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.relative_to(dst).as_posix()}\n" for p in files)
doc = {
    "schema": "integration.upstream-copy/1",
    "upstream": {"name": "prompt-to-assembly", "local_clone": clone, "commit": commit,
                 "licence": "Apache-2.0 (LICENSE, THIRD_PARTY_NOTICES.md)"},
    "made_by": "tools/fetch_benchmark.sh",
    "method": "git archive <commit> <paths> | tar -x (tracked files only, byte-identical)",
    "paths": paths,
    "file_count": len(files),
    "tree_sha256": hashlib.sha256(lines.encode()).hexdigest(),
    "tree_sha256_definition": "sha256 over lines '<file sha256>  <path>\\n', paths relative to benchmark/, sorted, before UPSTREAM.json is written",
}
(dst / "UPSTREAM.json").write_text(json.dumps(doc, indent=2) + "\n")
print(f"benchmark/: {len(files)} files from {commit[:12]}, tree {doc['tree_sha256'][:16]}")
EOF
