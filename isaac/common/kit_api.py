"""The one place Kit-version-specific calls live (Isaac Sim 5.1 today, 6.x next).

Everything under isaac/ talks to Isaac through this module. Rules:
  * USD and omni.* calls that are stable across 5.x and 6.x are used directly
    (pxr, omni.usd, omni.timeline, omni.kit.app, omni.graph.core, omni.physx).
  * Prim manipulation uses isaacsim.core.experimental.prims, which exists on
    5.1 and is the primary API on 6.x; the classic isaacsim.core.prims API is
    the fallback when the experimental one is missing.
  * Anything that still differs is behind a function here with a comment
    naming both variants. Moving to 6.1/6.2 should touch this file only.

Runs INSIDE Kit (its Python 3.11+); never imported by ROS-side code.
"""

from __future__ import annotations

import math
from typing import Any, Callable, Sequence

# -- version -------------------------------------------------------------------


def kit_version() -> dict[str, Any]:
    """Isaac Sim / Kit version strings, best effort."""
    out: dict[str, Any] = {"isaacsim": None, "kit": None, "prims_api": prims_api_name()}
    try:
        from isaacsim.core.version import get_version  # 5.x and 6.x

        v = get_version()
        out["isaacsim"] = ".".join(str(x) for x in v[:3]) if isinstance(v, (tuple, list)) else str(v)
    except Exception:
        pass
    try:
        import omni.kit.app

        out["kit"] = omni.kit.app.get_app().get_kit_version()
    except Exception:
        pass
    return out


def prims_api_name() -> str:
    try:
        import isaacsim.core.experimental.prims  # noqa: F401

        return "isaacsim.core.experimental.prims"
    except Exception:
        return "isaacsim.core.prims"


# -- app, stage, timeline ------------------------------------------------------------


def app() -> Any:
    import omni.kit.app

    return omni.kit.app.get_app()


def enable_extension(name: str) -> bool:
    """Enable an extension now (same call on 5.x and 6.x)."""
    manager = app().get_extension_manager()
    if manager.is_extension_enabled(name):
        return True
    manager.set_extension_enabled_immediate(name, True)
    return manager.is_extension_enabled(name)


def subscribe_update(callback: Callable[[Any], None], name: str = "cell_command_server") -> Any:
    """Run callback on every app update (the main thread). Keep the returned
    subscription alive; dropping it unsubscribes."""
    stream = app().get_update_event_stream()
    return stream.create_subscription_to_pop(callback, name=name)


def usd_context() -> Any:
    import omni.usd

    return omni.usd.get_context()


def stage() -> Any:
    return usd_context().get_stage()


def stage_path() -> str:
    try:
        return str(usd_context().get_stage_url())
    except Exception:
        root = stage().GetRootLayer() if stage() else None
        return str(root.identifier) if root else ""


def open_stage(path: str) -> bool:
    """Blocking open (5.x and 6.x: omni.usd)."""
    ok, _err = usd_context().open_stage(path)
    return bool(ok)


def reopen_stage() -> bool:
    path = stage_path()
    if not path:
        return False
    return open_stage(path)


def timeline() -> Any:
    import omni.timeline

    return omni.timeline.get_timeline_interface()


def is_playing() -> bool:
    return bool(timeline().is_playing())


def play() -> None:
    timeline().play()


def stop() -> None:
    timeline().stop()


def sim_time_s() -> float:
    return float(timeline().get_current_time())


# -- prims: discovery and poses (pure USD, stable) -------------------------------


def prim_exists(path: str) -> bool:
    st = stage()
    return bool(st and st.GetPrimAtPath(path).IsValid())


def prim_type(path: str) -> str:
    return str(stage().GetPrimAtPath(path).GetTypeName())


def children(path: str) -> list[str]:
    prim = stage().GetPrimAtPath(path)
    return [str(c.GetPath()) for c in prim.GetChildren()] if prim.IsValid() else []


def remove_prim(path: str) -> bool:
    import omni.usd

    if not prim_exists(path):
        return False
    omni.usd.commands.DeletePrimsCommand([path]).do()
    return not prim_exists(path)


def find_prims_with_api(api_name: str) -> list[str]:
    from pxr import UsdPhysics

    api = getattr(UsdPhysics, api_name)
    return [str(p.GetPath()) for p in stage().Traverse() if p.HasAPI(api)]


def articulation_root_path(prefer: str = "ur") -> str:
    roots = find_prims_with_api("ArticulationRootAPI")
    if not roots:
        raise RuntimeError("no articulation root on the stage")
    return next((r for r in roots if prefer in r.lower()), roots[0])


def relationship_targets(prim_path: str, rel_name: str) -> list[str]:
    prim = stage().GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return []
    rel = prim.GetRelationship(rel_name)
    return [str(t) for t in rel.GetTargets()] if rel else []


def world_pose(path: str) -> dict[str, list[float]]:
    """position xyz and orientation xyzw from the USD transform stack."""
    from pxr import Usd, UsdGeom

    prim = stage().GetPrimAtPath(path)
    if not prim.IsValid():
        raise LookupError(f"no prim at {path}")
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat()
    im = q.GetImaginary()
    return {"position": [float(t[0]), float(t[1]), float(t[2])],
            "orientation_xyzw": [float(im[0]), float(im[1]), float(im[2]), float(q.GetReal())]}


def is_rigid_body(path: str) -> bool:
    from pxr import UsdPhysics

    prim = stage().GetPrimAtPath(path)
    return prim.IsValid() and prim.HasAPI(UsdPhysics.RigidBodyAPI)


# -- prims: teleport, velocities, joints (version-adapted) ---------------------


def _xyzw_to_wxyz(q: Sequence[float]) -> list[float]:
    return [float(q[3]), float(q[0]), float(q[1]), float(q[2])]


def teleport(path: str, position: Sequence[float], orientation_xyzw: Sequence[float]) -> None:
    """Set a prim's world pose and, for a rigid body, zero its velocities.

    experimental: RigidPrim/XformPrim.set_world_poses(positions, orientations[wxyz])
    classic:      isaacsim.core.prims.RigidPrim/XFormPrim.set_world_poses(positions, orientations[wxyz])
    """
    import numpy as np

    pos = np.array([list(position)], dtype=np.float32)
    quat = np.array([_xyzw_to_wxyz(orientation_xyzw)], dtype=np.float32)
    rigid = is_rigid_body(path)
    if prims_api_name().endswith("experimental.prims"):
        from isaacsim.core.experimental.prims import RigidPrim, XformPrim

        prim = RigidPrim(path) if rigid else XformPrim(path)
        prim.set_world_poses(positions=pos, orientations=quat)
        if rigid:
            zeros = np.zeros((1, 3), dtype=np.float32)
            prim.set_velocities(linear_velocities=zeros, angular_velocities=zeros)
    else:
        from isaacsim.core.prims import RigidPrim, XFormPrim

        prim = RigidPrim(path) if rigid else XFormPrim(path)
        prim.set_world_poses(positions=pos, orientations=quat)
        if rigid:
            prim.set_velocities(np.zeros((1, 6), dtype=np.float32))


def zero_velocities(path: str) -> None:
    import numpy as np

    if not is_rigid_body(path):
        return
    if prims_api_name().endswith("experimental.prims"):
        from isaacsim.core.experimental.prims import RigidPrim

        zeros = np.zeros((1, 3), dtype=np.float32)
        RigidPrim(path).set_velocities(linear_velocities=zeros, angular_velocities=zeros)
    else:
        from isaacsim.core.prims import RigidPrim

        RigidPrim(path).set_velocities(np.zeros((1, 6), dtype=np.float32))


class ArticulationHandle:
    """Joint access on the robot. Needs the timeline PLAYING (tensor backend)."""

    def __init__(self, root_path: str) -> None:
        self.root_path = root_path
        if prims_api_name().endswith("experimental.prims"):
            from isaacsim.core.experimental.prims import Articulation

            self._art = Articulation(root_path)
            self._experimental = True
        else:
            from isaacsim.core.prims import Articulation

            self._art = Articulation(prim_paths_expr=root_path, name="cell_command_server_robot")
            self._experimental = False

    @property
    def dof_names(self) -> list[str]:
        names = self._art.dof_names if self._experimental else self._art.joint_names
        return [str(n) for n in names]

    def set_positions(self, by_name: dict[str, float], targets_too: bool = True) -> list[str]:
        import numpy as np

        names = self.dof_names
        idx = [names.index(n) for n in by_name if n in names]
        missing = [n for n in by_name if n not in names]
        vals = np.array([[by_name[names[i]] for i in idx]], dtype=np.float32)
        if not idx:
            return missing
        if self._experimental:
            self._art.set_dof_positions(vals, dof_indices=idx)
            if targets_too:
                self._art.set_dof_position_targets(vals, dof_indices=idx)
            self._art.set_dof_velocities(np.zeros_like(vals), dof_indices=idx)
        else:
            self._art.set_joint_positions(vals, joint_indices=idx)
            if targets_too:
                self._art.set_joint_position_targets(vals, joint_indices=idx)
            self._art.set_joint_velocities(np.zeros_like(vals), joint_indices=idx)
        return missing

    def positions(self) -> dict[str, float]:
        import numpy as np

        raw = self._art.get_dof_positions() if self._experimental else self._art.get_joint_positions()
        arr = np.asarray(raw.numpy() if hasattr(raw, "numpy") else raw).reshape(-1)
        return {n: float(v) for n, v in zip(self.dof_names, arr)}

    def speeds(self) -> dict[str, float]:
        import numpy as np

        raw = self._art.get_dof_velocities() if self._experimental else self._art.get_joint_velocities()
        arr = np.asarray(raw.numpy() if hasattr(raw, "numpy") else raw).reshape(-1)
        return {n: abs(float(v)) for n, v in zip(self.dof_names, arr)}


# -- joints created during an episode -----------------------------------------


EPISODE_JOINT_ATTR = "cell:episode_joint"


def episode_joints() -> list[str]:
    """Physics joints marked as created during an episode (attribute
    cell:episode_joint = true), anywhere on the stage."""
    out = []
    for prim in stage().Traverse():
        attr = prim.GetAttribute(EPISODE_JOINT_ATTR)
        if attr and attr.Get():
            out.append(str(prim.GetPath()))
    return out


# -- contact reports ------------------------------------------------------------------


def enable_contact_report(paths: Sequence[str]) -> list[str]:
    """Apply PhysxContactReportAPI to the given rigid bodies (5.x and 6.x: PhysxSchema)."""
    from pxr import PhysxSchema

    applied = []
    for p in paths:
        prim = stage().GetPrimAtPath(p)
        if not prim.IsValid():
            continue
        api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
        api.CreateThresholdAttr().Set(0.0)
        applied.append(p)
    return applied


def subscribe_contact_reports(callback: Callable[[list[dict[str, Any]]], None]) -> Any:
    """callback(list of {body0, body1, normal_force, event}) per physics step.
    omni.physx: get_physx_simulation_interface().subscribe_contact_report_events
    (same on 5.x and 6.x)."""
    from omni.physx import get_physx_simulation_interface
    from pxr import PhysicsSchemaTools

    def _on_report(contact_headers, contact_data):
        out = []
        for h in contact_headers:
            try:
                b0 = PhysicsSchemaTools.intToSdfPath(h.actor0)
                b1 = PhysicsSchemaTools.intToSdfPath(h.actor1)
            except Exception:
                b0, b1 = str(h.actor0), str(h.actor1)
            force = 0.0
            try:
                for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                    force += float(contact_data[i].impulse.GetLength()) if hasattr(contact_data[i], "impulse") else 0.0
            except Exception:
                pass
            out.append({"body0": str(b0), "body1": str(b1), "event": str(h.type), "impulse": force})
        if out:
            callback(out)

    return get_physx_simulation_interface().subscribe_contact_report_events(_on_report)


# -- cameras and ROS 2 camera graphs ----------------------------------------------


def create_camera(path: str, translate: Sequence[float], orient_xyzw: Sequence[float], focal_length_mm: float | None = None) -> str:
    """A UsdGeom.Camera at `path` (parented by the path), posed locally."""
    from pxr import Gf, Sdf, UsdGeom, Vt

    cam = UsdGeom.Camera.Define(stage(), Sdf.Path(path))
    prim = cam.GetPrim()
    prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3, custom=False).Set(Gf.Vec3d(*translate))
    q = Gf.Quatd(float(orient_xyzw[3]), Gf.Vec3d(float(orient_xyzw[0]), float(orient_xyzw[1]), float(orient_xyzw[2])))
    prim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatd, custom=False).Set(q)
    prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray, custom=False).Set(Vt.TokenArray(["xformOp:translate", "xformOp:orient"]))
    if focal_length_mm:
        cam.CreateFocalLengthAttr().Set(float(focal_length_mm))
    return str(prim.GetPath())


def build_ros2_camera_graph(
    graph_path: str,
    camera_prim: str,
    width: int,
    height: int,
    frame_id: str,
    rgb_topic: str,
    depth_topic: str | None,
    info_topic: str,
) -> dict[str, Any]:
    """The same graph shape the stage already uses for /Graph/ROS_Camera:
    IsaacCreateRenderProduct -> ROS2CameraHelper (rgb, depth) + ROS2CameraInfoHelper,
    with a ROS2Context. Node type names verified against the 5.1 install;
    they are unchanged in 6.x (isaacsim.ros2.bridge.*)."""
    import omni.graph.core as og

    keys = og.Controller.Keys
    nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
    ]
    if depth_topic:
        nodes.append(("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"))
    connections = [
        ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
        ("RenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
        ("RenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
        ("RenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
        ("RenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
        ("Context.outputs:context", "RGBPublish.inputs:context"),
        ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
    ]
    values = [
        ("RenderProduct.inputs:cameraPrim", [camera_prim]),
        ("RenderProduct.inputs:width", int(width)),
        ("RenderProduct.inputs:height", int(height)),
        ("RGBPublish.inputs:type", "rgb"),
        ("RGBPublish.inputs:topicName", rgb_topic),
        ("RGBPublish.inputs:frameId", frame_id),
        ("CameraInfoPublish.inputs:topicName", info_topic),
        ("CameraInfoPublish.inputs:frameId", frame_id),
    ]
    if depth_topic:
        connections += [
            ("RenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),
            ("Context.outputs:context", "DepthPublish.inputs:context"),
        ]
        values += [
            ("DepthPublish.inputs:type", "depth"),
            ("DepthPublish.inputs:topicName", depth_topic),
            ("DepthPublish.inputs:frameId", frame_id),
        ]
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {keys.CREATE_NODES: nodes, keys.CONNECT: connections, keys.SET_VALUES: values},
    )
    return {"graph": graph_path, "nodes": [n for n, _ in nodes], "rgb": rgb_topic, "depth": depth_topic, "info": info_topic}


def graph_node_value(node_path: str, attr: str) -> Any:
    prim = stage().GetPrimAtPath(node_path)
    if not prim.IsValid():
        return None
    a = prim.GetAttribute(attr)
    return a.Get() if a else None


def quat_angle_deg(a_xyzw: Sequence[float], b_xyzw: Sequence[float]) -> float:
    dot = abs(sum(float(x) * float(y) for x, y in zip(a_xyzw, b_xyzw)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))
