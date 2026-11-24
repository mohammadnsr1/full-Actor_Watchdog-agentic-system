# FMB CAD

CAD of the Functional Manipulation Benchmark (FMB), downloaded 2026-11-24 from
<https://functional-manipulation-benchmark.github.io/files/index.html>, split into
one STEP per part for the assembly ingest (`docs/EXECUTOR_BASELINE.md`, EB2).

**Licence:** Creative Commons Attribution 4.0 International
(<https://creativecommons.org/licenses/by/4.0/>), as the FMB site states for all its
data. Attribution: Jianlan Luo, Charles Xu, et al., "FMB: A Functional Manipulation
Benchmark for Generalizable Robotic Learning", The International Journal of Robotics
Research, 2024, <https://doi.org/10.1177/02783649241276017> (arXiv:2401.08553).
Changes made here: each file split into one STEP per solid, re-framed, and the
boards also written as named assemblies; the downloads themselves are unchanged
in `source/`.

## `source/`: the downloads, unchanged (renamed only)

| Here | Downloaded as | What the FMB site calls it |
|---|---|---|
| `board_1.step` | `Board 1.step` | Assembly 1 (product "Teaser Board 2.2") |
| `board_2.step` | `Board 2.step` | Assembly 2 ("Teaser Board 4") |
| `board_3.step` | `Board 3.step` | Assembly 3 ("Teaser Board 5") |
| `peg_boards.step` | `peg_board.step` | Assembly Boards: 3 boards with 9 holes each |
| `pegs.step` | `peg.step` | Assembly Objects: 54 pegs |
| `board_fixture.step` | `board fixture.step` | Environment Fixture for multi-object tasks |
| `fixture.step` | `fixture.step` | Environment Fixture for single-object tasks (`peg fixture.step` on the site) |

`SHA256SUMS` pins them. Each download is one STEP product holding loose solids, in
millimetres; the boards' solids sit in their assembled positions.

## The splits

`tools/ingest/split_fmb.sh` regenerates every folder below from `source/`, byte for
byte, with `tools/ingest/split_step.py` (its docstring has the details). Native scale.

| Folder | Parts | Also |
|---|---|---|
| `board_1/` | `board1_base` + 4 | `_assembly.step`; `board1_02_purple` and `board1_05_blue` congruent |
| `board_2/` | `board2_base` + 4 | `_assembly.step`; `board2_04_red` and `board2_05_blue` congruent |
| `board_3/` | `board3_base` + 4 | `_assembly.step`; two congruent pairs |
| `peg_boards/` | 3 boards | no assembly (no assembled state ships) |
| `pegs/` | 54 pegs | no assembly; 27 cross-sections, each at 100 mm and 150 mm long |

- **Part frame**: axes parallel to the download's, origin at the centre of the
  part's XY bounding box with its lowest point at z = 0. Congruent parts share one
  local geometry (their rotation, identity in every pair here, goes into the
  placement), so the ingest puts them in one equivalence class.
- **`_assembly.step`**: each part a named product, placed in the base's part frame;
  the form the ingest's `target` step reads. Every board passes its terminal
  interpenetration gate, and every placed part lands on its original solid
  (worst deviation 2e-13 mm).
- **`index.json`**: per part, source solid number, colour, volume, size,
  placement, congruence group; and the hash of every file.
- Names carry the CAD colour, which is not this cell's colour (below).

## Which of this cell's parts they are

| This cell | Is | Scale | Notes |
|---|---|---|---|
| `fmb_assembly_2`: `base1`, `u_brown`, `u_orange`, `line_green`, `inverted_u_yellow` | Board 1 | 0.7 | base ↔ `board1_base`, the congruent U pair ↔ `board1_02_purple` / `board1_05_blue`, `line_green` ↔ `board1_03_amber`, `inverted_u_yellow` ↔ `board1_04_green`. Volumes agree with `config/parts/executor_parts.yaml` meshes. The labmate's `fmb1` assembly is this board too: Board 1 split here at 0.7 is identical to fmb1's part STEPs (every vertex) and assembled placements (1e-17 m). |
| `base2`, `u_green_2x`, `hex_red_2x`, `hex_blue_2x`, `inverted_u_brown_2x` | Board 2 | 1.4 | brown, green (lime), red and blue match by colour; the hexes are the congruent pair. Volumes at 1.4: base and green U equal the meshes', brown +0.09 %, hexes -0.34 % (the meshes are not exactly the CAD there) |
