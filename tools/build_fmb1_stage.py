#!/usr/bin/env python3
"""Put the labmate's fmb1 parts (FMB Board 1 at 0.7) on assets/usd/assembly_room.usd
and take the Board 2 (1.4x) parts off. docs/EXECUTOR_BASELINE.md, EB2.

Run with Isaac Sim's own USD (0.24.5), so the stage is written by the library that
reads it, not by a newer usd-core:

    L=$(ls -d ~/isaacsim/extscache/omni.usd.libs-*)
    PYTHONPATH=$L LD_LIBRARY_PATH=$L/bin ~/isaacsim/python.sh tools/build_fmb1_stage.py [--check-only]

Prim names and prim frames stay as they are, so every pose, placement, grasp offset
and mesh in the configs keeps its meaning. For each Board 1 prim in
assets/usd/fmb1/binding.json it:
  1. writes assets/usd/fmb1/<name>.usda: /<name>/<frame_prim> is an Xform at fmb1's
     object frame (t, R from the binding) holding a copy of the fmb1 bundle's visual
     mesh, box colliders and physics material (printed_plastic, bound there);
  2. on the stage: payload -> that file; drops the overs on the old asset's internals
     and the old `physics:mass = 0`; authors the bundle's mass, centre of mass and
     inertia rotated into the prim frame, and its rigid-body solver settings; binds
     the prim's colour from /World/Looks.
Then it deletes the Board 2 prims and their reorient frames, and sets the TF publisher's
targets: Board 2 targets out, the five fmb1 frames in (published as fmb1_occurrence_NN).

Checks (always, and alone with --check-only): every part's visual mesh lands in world
where the old mesh was (max vertex distance), collider boxes cover the mesh box, mass
props equal the bundle's, no rigid body is nested, no Board 2 prim or target is left,
every TF target resolves. Writes the measured numbers into binding.json.
"""
import argparse
import hashlib
import json
import math
import os
import shutil
import sys

from pxr import Gf, Sdf, Usd, UsdGeom

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STAGE = os.environ.get("FMB1_STAGE", os.path.join(REPO, "assets/usd/assembly_room.usd"))
OUT_DIR = os.path.join(REPO, "assets/usd/fmb1")
BINDING = os.path.join(OUT_DIR, "binding.json")
TF_NODE = "/Graph/Objects_TF/PublisherTF"
BOARD2 = ["u_green", "base2", "hex_blue", "hex_red", "inverted_u_brown",
          "u_green_reorient", "inverted_u_brown_reorient"]
MASS_ATTRS = ("physics:mass", "physics:centerOfMass", "physics:diagonalInertia", "physics:principalAxes")
SOLVER_ATTRS = ("physxRigidBody:sleepThreshold", "physxRigidBody:solverPositionIterationCount",
                "physxRigidBody:solverVelocityIterationCount")


def quat_from_R(R):
    """Unit quaternion (w, x, y, z) of a proper rotation matrix (column-vector convention)."""
    m = R
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        w, x, y, z = (m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        w, x, y, z = (m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        w, x, y, z = (m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return Gf.Quatd(w / n, x / n, y / n, z / n)


def world_mesh(stage, mesh_path):
    """World-space points and triangles (fan-triangulated) of one mesh."""
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(mesh_path))
    xf = UsdGeom.Xformable(mesh.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pts = [xf.Transform(Gf.Vec3d(p)) for p in mesh.GetPointsAttr().Get()]
    tris, k = [], 0
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    for n in mesh.GetFaceVertexCountsAttr().Get():
        f = idx[k:k + n]; k += n
        tris += [(f[0], f[i], f[i + 1]) for i in range(1, n - 1)]
    return pts, tris


def _point_triangle(p, a, b, c):
    """Distance from p to triangle abc (Ericson, Real-Time Collision Detection 5.1.5)."""
    ab, ac, ap = b - a, c - a, p - a
    d1, d2 = ab * ap, ac * ap
    if d1 <= 0 and d2 <= 0: return (p - a).GetLength()
    bp = p - b; d3, d4 = ab * bp, ac * bp
    if d3 >= 0 and d4 <= d3: return (p - b).GetLength()
    vc = d1 * d4 - d3 * d2
    if vc <= 0 and d1 >= 0 and d3 <= 0: return (p - (a + ab * (d1 / (d1 - d3)))).GetLength()
    cp = p - c; d5, d6 = ab * cp, ac * cp
    if d6 >= 0 and d5 <= d6: return (p - c).GetLength()
    vb = d5 * d2 - d1 * d6
    if vb <= 0 and d2 >= 0 and d6 <= 0: return (p - (a + ac * (d2 / (d2 - d6)))).GetLength()
    va = d3 * d6 - d5 * d4
    if va <= 0 and (d4 - d3) >= 0 and (d5 - d6) >= 0:
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6)); return (p - (b + (c - b) * w)).GetLength()
    den = 1.0 / (va + vb + vc); v, w = vb * den, vc * den
    return (p - (a + ab * v + ac * w)).GetLength()


def surface_distance(m1, m2):
    """Max distance from either mesh's vertices to the other mesh's surface (symmetric)."""
    def one(src, dst):
        pts, (dpts, dtris) = src[0], dst
        return max(min(_point_triangle(q, dpts[i], dpts[j], dpts[k]) for i, j, k in dtris) for q in pts)
    return max(one(m1, m2), one(m2, m1))


def open_parts_stage(path, parts):
    st = Usd.Stage.Open(path, load=Usd.Stage.LoadNone)       # the room and table payloads stay unloaded
    for name in parts:
        st.Load(f"/World/{name}")
    return st


def write_part_asset(name, p):
    src = Sdf.Layer.FindOrOpen(os.path.join(REPO, p["bundle"]))
    body = src.defaultPrim
    path = os.path.join(OUT_DIR, f"{name}.usda")
    if os.path.exists(path):
        os.remove(path)
    lay = Sdf.Layer.CreateNew(path)
    lay.defaultPrim = name
    lay.pseudoRoot.SetInfo("metersPerUnit", 1.0)
    lay.pseudoRoot.SetInfo("kilogramsPerUnit", 1.0)
    lay.pseudoRoot.SetInfo("upAxis", "Z")
    lay.documentation = (f"{name}: the labmate's fmb1 {p['body']} ({p['occurrence']}), FMB Board 1 at 0.7. "
                         "Written by tools/build_fmb1_stage.py from the bundle named in customData; "
                         "the root is this cell's prim frame, the child is fmb1's object frame.")
    root = Sdf.CreatePrimInLayer(lay, f"/{name}")
    root.specifier, root.typeName = Sdf.SpecifierDef, "Xform"
    root.customData = {"fmb1": {"body": p["body"], "occurrence": p["occurrence"], "bundle": p["bundle"],
                                "bundle_sha256": p["bundle_sha256"]}}
    frame = Sdf.CreatePrimInLayer(lay, f"/{name}/{p['frame_prim']}")
    frame.specifier, frame.typeName = Sdf.SpecifierDef, "Xform"
    frame.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(["MaterialBindingAPI"]))
    t = Sdf.AttributeSpec(frame, "xformOp:translate", Sdf.ValueTypeNames.Double3)
    t.default = Gf.Vec3d(*p["t_prim_from_object_m"])
    o = Sdf.AttributeSpec(frame, "xformOp:orient", Sdf.ValueTypeNames.Quatd)
    o.default = quat_from_R(p["R_prim_from_object"])
    order = Sdf.AttributeSpec(frame, "xformOpOrder", Sdf.ValueTypeNames.TokenArray, variability=Sdf.VariabilityUniform)
    order.default = ["xformOp:translate", "xformOp:orient"]
    for child in ("visual", "colliders", "PhysicsMaterials"):
        if not Sdf.CopySpec(src, f"/{body}/{child}", lay, f"/{name}/{p['frame_prim']}/{child}"):
            sys.exit(f"{name}: could not copy {body}/{child}")
    rel = Sdf.RelationshipSpec(frame, "material:binding:physics", custom=False)
    rel.targetPathList.explicitItems = [Sdf.Path(f"/{name}/{p['frame_prim']}/PhysicsMaterials/printed_plastic")]
    rel.SetInfo("bindMaterialAs", "strongerThanDescendants")
    vis = lay.GetPrimAtPath(f"/{name}/{p['frame_prim']}/visual")
    sub = Sdf.AttributeSpec(vis, "subdivisionScheme", Sdf.ValueTypeNames.Token, variability=Sdf.VariabilityUniform)
    sub.default = "none"
    pts = vis.attributes["points"].default
    lo = Gf.Vec3f(*(min(q[i] for q in pts) for i in range(3)))
    hi = Gf.Vec3f(*(max(q[i] for q in pts) for i in range(3)))
    ext = Sdf.AttributeSpec(vis, "extent", Sdf.ValueTypeNames.Float3Array)
    ext.default = [lo, hi]
    lay.Save()
    # what the body root carried in the bundle, to move onto the stage prim
    b = src.GetPrimAtPath(f"/{body}")
    return {k: b.attributes[k].default for k in MASS_ATTRS + SOLVER_ATTRS if k in b.attributes}


def remove_spec(lay, path):
    edit = Sdf.BatchNamespaceEdit()
    edit.Add(path, Sdf.Path.emptyPath)
    if not lay.Apply(edit):
        sys.exit(f"could not remove {path}")


def set_attr(spec, name, type_name, value):
    a = spec.attributes.get(name) if name in spec.attributes else Sdf.AttributeSpec(spec, name, type_name)
    a.default = value


def edit_stage(lay, binding, bundle_root_attrs):
    world = lay.GetPrimAtPath("/World")
    for name, p in binding["parts"].items():
        spec = lay.GetPrimAtPath(f"/World/{name}")
        if spec is None:
            sys.exit(f"/World/{name} not in the stage root layer")
        spec.payloadList.ClearEdits()
        spec.payloadList.prependedItems = [Sdf.Payload(f"./fmb1/{name}.usda")]
        if name in spec.nameChildren:                       # overs on the old asset's internals
            remove_spec(lay, spec.path.AppendChild(name))
        schemas = spec.GetInfo("apiSchemas") if spec.HasInfo("apiSchemas") else Sdf.TokenListOp()
        prepend = list(schemas.prependedItems)
        for s in ("PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysicsMassAPI", "MaterialBindingAPI"):
            if s not in prepend:
                prepend.append(s)
        schemas.prependedItems = prepend
        schemas.deletedItems = [s for s in schemas.deletedItems if s not in prepend]
        spec.SetInfo("apiSchemas", schemas)
        R = Gf.Matrix3d(*[x for row in p["R_prim_from_object"] for x in row])   # row-major = column-vector R
        a = bundle_root_attrs[name]
        q_pa = a["physics:principalAxes"]
        q_R = quat_from_R(p["R_prim_from_object"])
        pa = q_R * Gf.Quatd(q_pa.GetReal(), *q_pa.GetImaginary())
        pa.Normalize()
        t = Gf.Vec3d(*p["t_prim_from_object_m"])
        com = R * Gf.Vec3d(a["physics:centerOfMass"]) + t
        set_attr(spec, "physics:mass", Sdf.ValueTypeNames.Float, float(a["physics:mass"]))
        set_attr(spec, "physics:centerOfMass", Sdf.ValueTypeNames.Point3f, Gf.Vec3f(com))
        set_attr(spec, "physics:diagonalInertia", Sdf.ValueTypeNames.Float3, Gf.Vec3f(a["physics:diagonalInertia"]))
        set_attr(spec, "physics:principalAxes", Sdf.ValueTypeNames.Quatf, Gf.Quatf(pa))
        set_attr(spec, "physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float, float(a["physxRigidBody:sleepThreshold"]))
        set_attr(spec, "physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int,
                 int(a["physxRigidBody:solverPositionIterationCount"]))
        set_attr(spec, "physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.Int,
                 int(a["physxRigidBody:solverVelocityIterationCount"]))
        rel = spec.relationships.get("material:binding") if "material:binding" in spec.relationships else \
            Sdf.RelationshipSpec(spec, "material:binding", custom=False)
        rel.targetPathList.explicitItems = [Sdf.Path(p["visual_look"])]
        rel.SetInfo("bindMaterialAs", "weakerThanDescendants")
    for name in BOARD2:
        if name in world.nameChildren:
            remove_spec(lay, world.path.AppendChild(name))
    # Kit's metrics-assembler bookkeeping: drop entries for prims that are no longer on the stage
    data = dict(lay.customLayerData)
    ma = dict(data.get("metricsAssembler", {}))
    for key in [k for k in ma if k.startswith("/World/") and k.split("/")[2] not in world.nameChildren]:
        del ma[key]
    data["metricsAssembler"] = ma
    lay.customLayerData = data
    node = lay.GetPrimAtPath(TF_NODE)
    rel = node.relationships["inputs:targetPrims"]
    old = list(rel.targetPathList.explicitItems)
    keep = [t for t in old if not any(str(t) == f"/World/{b}" or str(t).startswith(f"/World/{b}/") for b in BOARD2)]
    add = [Sdf.Path(f"/World/{n}/{p['frame_prim']}") for n, p in binding["parts"].items()]
    rel.targetPathList.explicitItems = keep + [a for a in add if a not in keep]
    return len(old), len(rel.targetPathList.explicitItems)


def check(stage_path, binding, old_world=None, bundle_root_attrs=None):
    names = list(binding["parts"])
    st = open_parts_stage(stage_path, names)
    problems, report = [], {}
    for name, p in binding["parts"].items():
        root = st.GetPrimAtPath(f"/World/{name}")
        frame = root.GetPrimAtPath(p["frame_prim"]) if root else None
        if not frame or not frame.IsValid():
            problems.append(f"{name}: {p['frame_prim']} missing"); continue
        r = {}
        new = world_mesh(st, frame.GetPath().AppendChild("visual"))
        if old_world is not None:
            r["world_vertex_dev_mm"] = surface_distance(old_world[name], new) * 1000.0
            if r["world_vertex_dev_mm"] > 1e-3:
                problems.append(f"{name}: moved {r['world_vertex_dev_mm']:.6f} mm")
        # colliders: union of the box bounds equals the visual mesh box (in the frame)
        cache = UsdGeom.XformCache()
        to_frame = cache.GetLocalToWorldTransform(frame).GetInverse()
        lo, hi = [1e9] * 3, [-1e9] * 3
        for c in Usd.PrimRange(frame.GetChild("colliders")):
            if c.GetTypeName() != "Cube":
                continue
            m = cache.GetLocalToWorldTransform(c) * to_frame
            for corner in [Gf.Vec3d(x, y, z) for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)]:
                q = m.Transform(corner)
                lo = [min(lo[i], q[i]) for i in range(3)]; hi = [max(hi[i], q[i]) for i in range(3)]
        mlo, mhi = frame.GetChild("visual").GetAttribute("extent").Get()
        r["collider_box_vs_mesh_box_mm"] = max(max(abs(lo[i] - mlo[i]), abs(hi[i] - mhi[i])) for i in range(3)) * 1000
        if r["collider_box_vs_mesh_box_mm"] > 1e-3:
            problems.append(f"{name}: collider box differs from mesh box by {r['collider_box_vs_mesh_box_mm']:.4f} mm")
        # mass properties: inertia tensor in the prim frame equals R I_bundle R^T
        mass = root.GetAttribute("physics:mass").Get()
        r["mass_kg"] = mass
        if bundle_root_attrs is not None:
            a = bundle_root_attrs[name]
            if abs(mass - a["physics:mass"]) > 1e-9:
                problems.append(f"{name}: mass {mass} != bundle {a['physics:mass']}")

            def tensor(q, d):
                M = Gf.Matrix3d().SetRotate(q)         # Gf row-vector form: v' = v * M
                D = Gf.Matrix3d(d[0], 0, 0, 0, d[1], 0, 0, 0, d[2])
                return M.GetTranspose() * D * M        # = Rc D Rc^T with Rc = M^T
            pa_new = root.GetAttribute("physics:principalAxes").Get()
            d = root.GetAttribute("physics:diagonalInertia").Get()
            I_new = tensor(Gf.Quatd(pa_new.GetReal(), *pa_new.GetImaginary()), d)
            qb = a["physics:principalAxes"]
            I_b = tensor(Gf.Quatd(qb.GetReal(), *qb.GetImaginary()), a["physics:diagonalInertia"])
            Rrow = Gf.Matrix3d(*[x for row in p["R_prim_from_object"] for x in row]).GetTranspose()  # row-vector form
            I_expect = Rrow.GetTranspose() * I_b * Rrow
            r["inertia_max_abs_diff"] = max(abs(I_new[i][j] - I_expect[i][j]) for i in range(3) for j in range(3))
            scale = max(abs(I_expect[i][j]) for i in range(3) for j in range(3))
            if r["inertia_max_abs_diff"] > 1e-6 * scale:            # float32 storage of the quaternion
                problems.append(f"{name}: inertia off by {r['inertia_max_abs_diff']:.3e}")
        nested = [str(d.GetPath()) for d in Usd.PrimRange(root)
                  if d != root and "PhysicsRigidBodyAPI" in d.GetAppliedSchemas()]
        if nested:
            problems.append(f"{name}: nested rigid bodies {nested}")
        report[name] = r
    for b in BOARD2:
        if st.GetPrimAtPath(f"/World/{b}").IsValid():
            problems.append(f"Board 2 prim /World/{b} still on the stage")
    targets = st.GetPrimAtPath(TF_NODE).GetRelationship("inputs:targetPrims").GetTargets()
    for t in targets:
        if any(str(t).startswith(f"/World/{b}") for b in BOARD2):
            problems.append(f"TF target {t} is Board 2")
        top = str(t).split("/")[2]
        if top in binding["parts"] and not st.GetPrimAtPath(t).IsValid():
            problems.append(f"TF target {t} does not resolve")
    return problems, report, [str(t) for t in targets]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check-only", action="store_true")
    args = ap.parse_args()
    binding = json.load(open(BINDING))
    print("USD", Usd.GetVersion())
    if args.check_only:
        attrs = {}
        for n, p in binding["parts"].items():
            src = Sdf.Layer.FindOrOpen(os.path.join(REPO, p["bundle"]))     # keep the layer alive while reading
            attrs[n] = {k: v.default for k, v in src.GetPrimAtPath("/" + p["body"]).attributes.items()}
        problems, report, targets = check(STAGE, binding, bundle_root_attrs=attrs)
        print(json.dumps(report, indent=1)); print("TF targets:", targets)
        print("PROBLEMS:" if problems else "all checks passed", *problems, sep="\n  ")
        return 1 if problems else 0
    lay = Sdf.Layer.FindOrOpen(STAGE)
    if any(str(pl.assetPath).startswith("./fmb1/") for pl in lay.GetPrimAtPath("/World/base1").payloadList.prependedItems):
        sys.exit("the stage already carries the fmb1 parts; run with --check-only")
    backup = os.path.join(os.environ.get("FMB1_BACKUP_DIR", "/tmp"), "assembly_room.before_fmb1.usd")
    shutil.copy2(STAGE, backup)
    st_old = open_parts_stage(STAGE, binding["parts"])
    old_world = {}
    for name in binding["parts"]:
        meshes = [p for p in Usd.PrimRange(st_old.GetPrimAtPath(f"/World/{name}")) if p.IsA(UsdGeom.Mesh)]
        if len(meshes) != 1:
            sys.exit(f"{name}: expected one mesh in the old asset, found {len(meshes)}")
        old_world[name] = world_mesh(st_old, meshes[0].GetPath())
    del st_old
    os.makedirs(OUT_DIR, exist_ok=True)
    attrs = {name: write_part_asset(name, p) for name, p in binding["parts"].items()}
    lay = Sdf.Layer.FindOrOpen(STAGE)
    lay.Reload(force=True)
    before, after = edit_stage(lay, binding, attrs)
    lay.Save()
    print(f"TF targets {before} -> {after}")
    problems, report, targets = check(STAGE, binding, old_world, attrs)
    binding["built"] = {"by": "tools/build_fmb1_stage.py", "usd": list(Usd.GetVersion()),
                        "checks": report, "tf_targets": targets,
                        "stage_sha256": hashlib.sha256(open(STAGE, "rb").read()).hexdigest(),
                        "previous_stage": "git (the committed assets/usd/assembly_room.usd); a copy was left at " + backup}
    json.dump(binding, open(BINDING, "w"), indent=2)
    print(json.dumps(report, indent=1))
    if problems:
        print("PROBLEMS (stage written; previous stage kept at", backup + "):", *problems, sep="\n  ")
        return 1
    print("all checks passed; previous stage kept at", backup)
    return 0


if __name__ == "__main__":
    sys.exit(main())
