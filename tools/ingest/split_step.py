#!/usr/bin/env python3
"""Split a one-product, many-solid STEP (as the FMB dataset ships them) into one
STEP per part, and optionally an assembled STEP with one named product per part.

Run it in the ingest venv (tools/ingest/setup_venv.sh):

    tools/ingest/.venv/bin/python tools/ingest/split_step.py "Board 1.step" --out DIR \
        --assembly --label fmb_board_1 [--prefix board1] [--scale 1.0] [--name 1=base ...] \
        [--anchor base] [--frame footprint|centroid]

What it writes in DIR:
  <name>.step        one per solid: that solid alone, in its own part frame
  _assembly.step     (--assembly) every part as a named product, placed by an
                     assembly transform; the form prompt-to-assembly's ingest reads
                     (`target` verb; fmb1's sources/_assembly.step has the same shape)
  index.json         per part: source solid number, colour, volume, size, its
                     placement (in the source frame, or in --anchor's part frame),
                     and its congruence group

Part frame (--frame footprint, the default): axes parallel to the source frame,
origin at the centre of the solid's XY bounding box with its lowest point at z = 0,
the convention of the labmate's fmb1 parts ("XY footprint centred, minimum Z on the
ground plane"; Board 1 x 0.7 split this way matches fmb1's Body STEPs to 5e-10 mm).
--frame centroid puts the origin at the centroid instead. Congruent solids (equal volume and
inertia, and an axis-aligned rotation that maps one vertex set onto the other)
share the first member's local geometry; the rotation goes into the placement, so
their part files are geometrically identical and the ingest puts them in one
equivalence class. A congruent pair that no axis-aligned rotation maps (a mirror
image, a general rotation) keeps separate geometry and is reported.

Units: the source's (FMB: millimetres), written back as millimetres; --scale
multiplies geometry (0.7 for this cell's Board 1, 1.4 for its Board 2). Python 3.10+.
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import re
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from OCP.BRep import BRep_Tool
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCP.STEPCAFControl import STEPCAFControl_Reader, STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_SOLID, TopAbs_VERTEX
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool
from OCP.gp import gp_Trsf

VOLUME_REL_TOL = 1e-6        # congruence candidates: relative volume and inertia agreement
INERTIA_REL_TOL = 1e-5
VERTEX_TOL = 1e-3            # source units (mm): vertex sets must agree to a micrometre

PALETTE = {                  # nearest-name colour labels, for file names only
    "red": (1.0, 0.0, 0.0), "coral": (1.0, 0.27, 0.2), "amber": (1.0, 0.75, 0.01),
    "yellow": (1.0, 1.0, 0.0), "olive": (0.87, 0.81, 0.0), "green": (0.05, 0.6, 0.01),
    "lime": (0.06, 1.0, 0.0), "steelblue": (0.32, 0.55, 0.81), "blue": (0.01, 0.05, 0.9),
    "purple": (0.35, 0.06, 1.0), "brown": (0.25, 0.07, 0.02), "black": (0.05, 0.05, 0.05),
}

_TIMESTAMP = re.compile(rb"(FILE_NAME\('[^']*',')[^']*(')")


def colour_name(rgb):
    if rgb is None:
        return "nocolour"
    return min(PALETTE, key=lambda k: sum((a - b) ** 2 for a, b in zip(PALETTE[k], rgb)))


def read_solids(path: Path):
    doc = TDocStd_Document(TCollection_ExtendedString("split"))
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        sys.exit(f"cannot read {path}")
    reader.Transfer(doc)
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    colours = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    free = TDF_LabelSequence()
    shapes.GetFreeShapes(free)
    solids = []
    for i in range(1, free.Length() + 1):
        exp = TopExp_Explorer(shapes.GetShape_s(free.Value(i)), TopAbs_SOLID)
        while exp.More():
            solid = TopoDS.Solid_s(exp.Current())
            rgb = None
            c = Quantity_Color()
            for kind in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
                if colours.GetColor(solid, kind, c):
                    rgb = (round(c.Red(), 4), round(c.Green(), 4), round(c.Blue(), 4))
                    break
            solids.append((solid, rgb))
            exp.Next()
    return solids


def transformed(shape, trsf: gp_Trsf):
    return BRepBuilderAPI_Transform(shape, trsf, True).Shape()


def mass_props(shape):
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, g)
    c = g.CentreOfMass()
    m = g.MatrixOfInertia()
    inertia = np.array([[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    return g.Mass(), np.array([c.X(), c.Y(), c.Z()]), np.sort(np.linalg.eigvalsh(inertia))


def bounds(shape):
    """Tight axis-aligned bounds from the exact geometry (no tolerance gap)."""
    b = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, b, False, False)
    return b.Get()


def size_of(shape):
    x0, y0, z0, x1, y1, z1 = bounds(shape)
    return [x1 - x0, y1 - y0, z1 - z0]


def frame_origin(shape, com, frame):
    if frame == "centroid":
        return com
    x0, y0, z0, x1, y1, z1 = bounds(shape)
    return np.array([(x0 + x1) / 2.0, (y0 + y1) / 2.0, z0])


def vertices(shape):
    pts = set()
    exp = TopExp_Explorer(shape, TopAbs_VERTEX)
    while exp.More():
        p = BRep_Tool.Pnt_s(TopoDS.Vertex_s(exp.Current()))
        pts.add((round(p.X(), 9), round(p.Y(), 9), round(p.Z(), 9)))
        exp.Next()
    return np.array(sorted(pts))


def cube_rotations():
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            r = np.zeros((3, 3))
            for row, (col, s) in enumerate(zip(perm, signs)):
                r[row, col] = s
            if np.linalg.det(r) > 0:
                out.append(r)
    out.sort(key=lambda r: (not np.allclose(r, np.eye(3)),))   # identity first
    return out


def same_set(a, b, tol):
    if len(a) != len(b):
        return False
    d, _ = cKDTree(b).query(a)
    return float(d.max()) <= tol


def trsf_from(rot, trans):
    t = gp_Trsf()
    t.SetValues(rot[0][0], rot[0][1], rot[0][2], trans[0],
                rot[1][0], rot[1][1], rot[1][2], trans[1],
                rot[2][0], rot[2][1], rot[2][2], trans[2])
    return t


def write_doc(path: Path, parts, assembly_name=None):
    """parts: list of (name, local_shape, rgb, placement gp_Trsf or None)."""
    doc = TDocStd_Document(TCollection_ExtendedString("out"))
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    root = None
    if assembly_name is not None:
        root = st.NewShape()
        TDataStd_Name.Set_s(root, TCollection_ExtendedString(assembly_name))
    for name, shape, rgb, placement in parts:
        label = st.AddShape(shape, False)
        TDataStd_Name.Set_s(label, TCollection_ExtendedString(name))
        if rgb is not None:
            ct.SetColor(label, Quantity_Color(*rgb, Quantity_TOC_RGB), XCAFDoc_ColorType.XCAFDoc_ColorSurf)
        if root is not None:
            comp = st.AddComponent(root, label, TopLoc_Location(placement))
            TDataStd_Name.Set_s(comp, TCollection_ExtendedString(f"{name}:1"))
    if root is not None:
        st.UpdateAssemblies()
    if not Interface_Static.SetCVal_s("write.step.unit", "MM"):
        sys.exit("cannot set STEP unit")
    w = STEPCAFControl_Writer()
    w.SetColorMode(True)
    w.SetNameMode(True)
    if not w.Transfer(doc, STEPControl_AsIs) or w.Write(str(path)) != IFSelect_RetDone:
        sys.exit(f"cannot write {path}")
    # OCCT stamps the wall clock into FILE_NAME; fix it so reruns are byte-identical.
    data = path.read_bytes()
    head, marker, body = data.partition(b"DATA;")
    head = _TIMESTAMP.sub(rb"\g<1>1970-01-01T00:00:00\g<2>", head, count=1)
    path.write_bytes(head + marker + body)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--label", help="assembly label (default: the file stem)")
    ap.add_argument("--prefix", default="part", help="file-name prefix for unnamed solids")
    ap.add_argument("--name", action="append", default=[], metavar="N=NAME",
                    help="name solid N (1-based, source order); repeatable")
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--assembly", action="store_true", help="also write _assembly.step")
    ap.add_argument("--frame", choices=("footprint", "centroid"), default="footprint",
                    help="part-frame origin (default: XY bounding-box centre, lowest z)")
    ap.add_argument("--anchor", metavar="NAME",
                    help="express every placement in this part's frame (fmb1 uses its base's); "
                         "default: the source file's frame")
    args = ap.parse_args(argv)

    names = {}
    for item in args.name:
        n, _, v = item.partition("=")
        names[int(n)] = v
    label = args.label or re.sub(r"[^A-Za-z0-9_]+", "_", args.step.stem).strip("_").lower()
    args.out.mkdir(parents=True, exist_ok=True)

    solids = read_solids(args.step)
    if args.scale != 1.0:
        s = gp_Trsf()
        s.SetScale(gp_Pnt0(), args.scale)
        solids = [(transformed(sh, s), rgb) for sh, rgb in solids]

    records, used = [], set()
    for k, (shape, rgb) in enumerate(solids, start=1):
        vol, com, inertia = mass_props(shape)
        name = names.get(k) or f"{args.prefix}_{k:02d}_{colour_name(rgb)}"
        if name in used:
            sys.exit(f"duplicate part name {name}")
        used.add(name)
        records.append(dict(n=k, name=name, shape=shape, rgb=rgb, vol=vol, com=com, inertia=inertia,
                            origin=frame_origin(shape, com, args.frame), verts=vertices(shape) - com))

    # congruence: the first member of each group is the representative
    rotations = cube_rotations()
    for r in records:
        r["group"], r["rot"] = r["name"], np.eye(3)
        for rep in records[: r["n"] - 1]:
            if rep["group"] != rep["name"]:
                continue
            if abs(rep["vol"] - r["vol"]) > VOLUME_REL_TOL * rep["vol"]:
                continue
            if not np.allclose(rep["inertia"], r["inertia"], rtol=INERTIA_REL_TOL, atol=1e-9):
                continue
            for rot in rotations:          # member = rot @ representative (both centred)
                if same_set(rep["verts"] @ rot.T, r["verts"], VERTEX_TOL * args.scale):
                    r["group"], r["rot"] = rep["name"], rot
                    break
            else:
                r["congruence_note"] = (f"same volume and inertia as {rep['name']} but no "
                                        "axis-aligned rotation maps it (mirror or general rotation)")
            if r["group"] != r["name"]:
                break

    by_name = {r["name"]: r for r in records}
    # member = rot @ (rep - rep_com) + com, and local = rep - rep_origin, so the part
    # frame sits in the source frame at com + rot @ (rep_origin - rep_com).
    for r in records:
        rep = by_name[r["group"]]
        r["place_t"] = r["com"] + r["rot"] @ (rep["origin"] - rep["com"])
    if args.anchor is not None:
        if args.anchor not in by_name:
            sys.exit(f"--anchor {args.anchor}: no such part ({', '.join(by_name)})")
        a = by_name[args.anchor]
        a_rot, a_t = a["rot"].copy(), a["place_t"].copy()
        for r in records:                       # P' = P_anchor^-1 * P
            r["rot"] = a_rot.T @ r["rot"]
            r["place_t"] = a_rot.T @ (r["place_t"] - a_t)

    parts, index = [], []
    for r in records:
        rep = by_name[r["group"]]
        local = transformed(rep["shape"], trsf_from(np.eye(3), -rep["origin"]))
        origin_in_source = r["place_t"]
        placement = trsf_from(r["rot"], origin_in_source)
        write_doc(args.out / f"{r['name']}.step", [(r["name"], local, r["rgb"], None)])
        parts.append((r["name"], local, r["rgb"], placement))
        entry = {
            "name": r["name"], "file": f"{r['name']}.step", "source_solid": r["n"],
            "colour_rgb": r["rgb"], "colour_name": colour_name(r["rgb"]),
            "volume_mm3": round(r["vol"], 6), "size_mm": [round(v, 4) for v in size_of(r["shape"])],
            "centroid_in_source_mm": [round(v, 6) for v in r["com"]],
            "placement": {"translation_mm": [round(v, 6) for v in origin_in_source],
                                    "rotation_row_major": r["rot"].astype(int).tolist()},
            "congruence_group": r["group"],
        }
        if "congruence_note" in r:
            entry["congruence_note"] = r["congruence_note"]
        index.append(entry)

    if args.assembly:
        # No colours here: OCCT writes several styled items in hash order, so they
        # would make reruns differ byte-wise. Each part file keeps its colour.
        write_doc(args.out / "_assembly.step", [(n, s, None, p) for n, s, _, p in parts],
                  assembly_name=label)

    src = args.step.read_bytes()
    doc = {
        "schema": "integration.step-split/1",
        "tool": "tools/ingest/split_step.py",
        "source": {"file": args.step.name, "sha256": hashlib.sha256(src).hexdigest()},
        "label": label, "scale": args.scale, "unit": "mm",
        "part_frame": ("axes parallel to the source frame; origin at the XY bounding-box centre, "
                       "lowest z" if args.frame == "footprint" else
                       "axes parallel to the source frame; origin at the centroid")
                      + "; congruent solids share their group representative's local geometry",
        "assembly_step": "_assembly.step" if args.assembly else None,
        "placements_in": f"the part frame of {args.anchor}" if args.anchor else "the source file's frame",
        "parts": index,
        "files_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                         for p in sorted(args.out.glob("*.step"))},
    }
    (args.out / "index.json").write_text(json.dumps(doc, indent=2) + "\n")
    groups = {}
    for e in index:
        groups.setdefault(e["congruence_group"], []).append(e["name"])
    print(f"{args.step.name}: {len(index)} parts ({args.frame} frames) -> {args.out}"
          + (" (+ _assembly.step)" if args.assembly else ""))
    for g, members in groups.items():
        if len(members) > 1:
            print(f"  congruent: {', '.join(members)}")
    for e in index:
        if "congruence_note" in e:
            print(f"  note: {e['name']}: {e['congruence_note']}")


def gp_Pnt0():
    from OCP.gp import gp_Pnt
    return gp_Pnt(0.0, 0.0, 0.0)


if __name__ == "__main__":
    main()
