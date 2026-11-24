#!/usr/bin/env bash
# Regenerate assets/cad/fmb/{board_1,board_2,board_3,peg_boards,pegs}/ from the FMB
# downloads in assets/cad/fmb/source/ (assets/cad/fmb/README.md). Native scale (mm).
# Byte-identical on rerun. Needs the ingest venv (tools/ingest/setup_venv.sh).
set -euo pipefail
repo="$(cd "$(dirname "$0")/../.." && pwd)"
d="$repo/assets/cad/fmb"
py="$repo/tools/ingest/.venv/bin/python"
split=("$py" -I "$repo/tools/ingest/split_step.py")
export PYTHONDONTWRITEBYTECODE=1

( cd "$d/source" && sha256sum -c --quiet SHA256SUMS )
rm -rf "$d"/board_1 "$d"/board_2 "$d"/board_3 "$d"/peg_boards "$d"/pegs
for b in 1 2 3; do
  "${split[@]}" "$d/source/board_$b.step" --out "$d/board_$b" --assembly --label "fmb_board_$b" \
    --prefix "board$b" --name "1=board${b}_base" --anchor "board${b}_base" 2>&1 | grep -v -E '\*\*\*|\[32;1m|^$'
done
"${split[@]}" "$d/source/peg_boards.step" --out "$d/peg_boards" --prefix peg_board 2>&1 | grep -v -E '\*\*\*|\[32;1m|^$'
"${split[@]}" "$d/source/pegs.step" --out "$d/pegs" --prefix peg 2>&1 | grep -v -E '\*\*\*|\[32;1m|^$'
