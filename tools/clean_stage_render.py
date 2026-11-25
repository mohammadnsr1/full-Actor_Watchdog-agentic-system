#!/usr/bin/env python3
"""Strip the render products Isaac saved into the stage file (2026-11-25).

Every Play runs the ROS camera graphs (/Graph/ROS_Camera, /Graph/ROS_Camera1) and the
recorder, whose create-render-product nodes add /Render/OmniverseKit/HydraTextures/Replicator_N
prims. Saving the stage from Isaac while they exist writes them into the root layer, so each
save adds more; the stage had 173 by cd4d6c7 and 176 by aac7ee7, and each load brings them back
as RenderProduct prims with full RTX settings. Nothing outside /Render points at them: the
graphs make new ones on Play.

This removes, from the root layer only:
  /Render/OmniverseKit/HydraTextures/Replicator*        the saved products
  their entries in /Render/OmniverseGlobalRenderSettings.products
  /Render/PostProcess/SDGPipeline children named after them (overs that delete connections)
The viewport textures, the render settings, /Render/Vars and everything outside /Render stay.

Run with Isaac Sim's own USD, like tools/build_fmb1_stage.py:

    L=$(ls -d ~/isaacsim/extscache/omni.usd.libs-*)
    PYTHONPATH=$L LD_LIBRARY_PATH=$L/bin ~/isaacsim/python.sh tools/clean_stage_render.py [--check-only] [stage]

Do not save the stage from Isaac after Play if you want it to stay clean; the TF/camera
graphs do not need anything saved under /Render.
"""
import argparse
import os
import re
import sys

from pxr import Sdf

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HYDRA = Sdf.Path("/Render/OmniverseKit/HydraTextures")
SETTINGS = Sdf.Path("/Render/OmniverseGlobalRenderSettings")
SDG = Sdf.Path("/Render/PostProcess/SDGPipeline")
REPLICATOR = re.compile(r"^Replicator(_\d+)?(_|$)")


def stale(lay):
    hydra = lay.GetPrimAtPath(HYDRA)
    products = [c.path for c in hydra.nameChildren if REPLICATOR.match(c.name)] if hydra else []
    sdg = lay.GetPrimAtPath(SDG)
    sdg_children = [c.path for c in sdg.nameChildren if REPLICATOR.match(c.name)] if sdg else []
    return products, sdg_children


def settings_refs(lay):
    spec = lay.GetPrimAtPath(SETTINGS)
    if not spec or "products" not in spec.relationships:
        return None, []
    rel = spec.relationships["products"]
    return rel, [p for p in rel.targetPathList.explicitItems if p.GetParentPath() == HYDRA and REPLICATOR.match(p.name)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check-only", action="store_true")
    ap.add_argument("stage", nargs="?", default=os.path.join(REPO, "assets/usd/assembly_room.usd"))
    args = ap.parse_args()
    lay = Sdf.Layer.FindOrOpen(args.stage)
    products, sdg_children = stale(lay)
    rel, refs = settings_refs(lay)
    print(f"{args.stage}: {len(products)} saved Replicator render products, "
          f"{len(refs)} render-settings references to them, {len(sdg_children)} SDGPipeline overs")
    if args.check_only or not (products or refs or sdg_children):
        return 1 if (args.check_only and (products or refs or sdg_children)) else 0
    edit = Sdf.BatchNamespaceEdit()
    for p in products + sdg_children:
        edit.Add(p, Sdf.Path.emptyPath)
    if not lay.Apply(edit):
        sys.exit("could not remove the render products")
    if rel is not None:
        rel.targetPathList.explicitItems = [p for p in rel.targetPathList.explicitItems if p not in refs]
    lay.Save()
    products, sdg_children = stale(lay)
    _, refs = settings_refs(lay)
    print(f"after: {len(products)} products, {len(refs)} references, {len(sdg_children)} SDGPipeline overs")
    return 0


if __name__ == "__main__":
    sys.exit(main())
