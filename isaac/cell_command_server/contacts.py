"""Finger-pad contact aggregation: pure Python, no Kit, Python 3.10 safe.

PhysX reports contacts per pair of collision shapes and per physics step
(found / persist / lost). A finger and a part each have several shapes, so one
touch arrives as several shape-pair reports, and `persist` repeats every step.
PadContactTracker folds that into one event per (pad body, other body) pair:

    found   the first shape pair of the two bodies touched
    lost    the last shape pair separated

PhysX contact reports belong to rigid bodies (the finger links), and a finger
link's collider is the whole finger, so a touch on the back of the finger is
reported like a touch on the pad. Each pad may name a `surface_frame` (an Xform
on the pad face, e.g. left_contact_frame); the tracker expresses the contact
points in that frame and flags `on_pad_surface` when a point lies inside the
pad face box `pad_face` (ranges per axis of the surface frame, metres).

Config (config/platforms/sim.yaml scene):

    contact_pads:
      - body: /World/.../left_inner_finger                       # rigid body
        surface_frame: /World/.../left_inner_finger/left_contact_frame
    pad_face: {x_m: [lo, hi], y_m: [lo, hi], z_m: [lo, hi]}

A plain string entry is a body without a surface frame. Without `pad_face`,
`on_pad_surface` is always false (the point and distance are still reported).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Optional, Sequence

Vec3 = Sequence[float]
ToPadFrame = Callable[[str, Sequence[Vec3]], Sequence[Vec3]]
Box = tuple[tuple[float, float], tuple[float, float], tuple[float, float]]


@dataclass(frozen=True)
class Pad:
    body: str
    surface_frame: str = ""


def parse_pads(raw: Any) -> list[Pad]:
    out: list[Pad] = []
    for item in raw or []:
        if isinstance(item, str) and item:
            out.append(Pad(body=item))
        elif isinstance(item, dict) and item.get("body"):
            out.append(Pad(body=str(item["body"]), surface_frame=str(item.get("surface_frame") or "")))
        else:
            raise ValueError(f"scene.contact_pads entry needs a body: {item!r}")
    return out


def parse_face(raw: Any) -> Optional[Box]:
    """{x_m: [lo, hi], y_m: [lo, hi], z_m: [lo, hi]} -> box, or None when unset."""
    if not raw:
        return None
    try:
        box = tuple((float(raw[k][0]), float(raw[k][1])) for k in ("x_m", "y_m", "z_m"))
    except (KeyError, TypeError, IndexError) as exc:
        raise ValueError(f"scene.pad_face needs x_m, y_m, z_m as [lo, hi]: {raw!r}") from exc
    for lo, hi in box:
        if lo > hi:
            raise ValueError(f"scene.pad_face range [{lo}, {hi}] is reversed")
    return box  # type: ignore[return-value]


def normalize_event_type(raw: Any) -> str:
    """'ContactEventType.CONTACT_FOUND' -> 'found' (also persist, lost)."""
    s = str(raw).lower()
    for kind in ("found", "persist", "lost"):
        if kind in s:
            return kind
    return s


def _norm(v: Vec3) -> float:
    return math.sqrt(sum(float(x) * float(x) for x in v))


def _inside(p: Vec3, box: Box) -> bool:
    return all(lo <= float(v) <= hi for v, (lo, hi) in zip(p, box))


class PadContactTracker:
    def __init__(self, pads: Sequence[Pad], face: Optional[Box], to_pad_frame: ToPadFrame) -> None:
        self.pads = {p.body: p for p in pads}
        self.face = face
        self.to_pad_frame = to_pad_frame
        self._active: dict[tuple[str, str], dict[str, Any]] = {}

    def active(self) -> list[dict[str, Any]]:
        return [
            {"pad": pad, "body": body, "shape_pairs": len(st["shapes"]), "on_pad_surface": st["last"]["on_pad_surface"],
             "distance_m": st["last"]["distance_m"], "peak_impulse": st["peak_impulse"]}
            for (pad, body), st in sorted(self._active.items())
        ]

    def clear(self) -> None:
        self._active.clear()

    def _geometry(self, pad: str, points: Sequence[Vec3]) -> dict[str, Any]:
        """Report a point inside the pad face if there is one, else the point
        closest to the surface frame origin."""
        frame = self.pads[pad].surface_frame
        geo: dict[str, Any] = {"pad_frame": frame, "point_world": None, "point_in_pad_frame": None, "distance_m": None, "on_pad_surface": False}
        if not points:
            return geo
        if not frame:
            geo["point_world"] = [float(x) for x in points[0]]
            return geo
        local = list(self.to_pad_frame(frame, points))
        order = sorted(range(len(points)), key=lambda i: _norm(local[i]))
        on_face = [i for i in order if self.face is not None and _inside(local[i], self.face)]
        best = on_face[0] if on_face else order[0]
        geo.update(
            point_world=[float(x) for x in points[best]],
            point_in_pad_frame=[float(x) for x in local[best]],
            distance_m=_norm(local[best]),
            on_pad_surface=bool(on_face),
        )
        return geo

    @staticmethod
    def _event(kind: str, pad: str, body: str, geo: dict[str, Any], impulse: float, shape_pairs: int) -> dict[str, Any]:
        return {"event": kind, "pad": pad, "body": body, "impulse": float(impulse), "shape_pairs": shape_pairs, **geo}

    def feed(self, reports: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
        """reports: [{body0, body1, collider0, collider1, event, points, impulse}] from one step."""
        events: list[dict[str, Any]] = []
        for r in reports:
            b0, b1 = str(r.get("body0", "")), str(r.get("body1", ""))
            if b0 in self.pads:
                pad, other = b0, b1
            elif b1 in self.pads:
                pad, other = b1, b0
            else:
                continue
            key = (pad, other)
            shape = tuple(sorted((str(r.get("collider0", "")), str(r.get("collider1", "")))))
            kind = normalize_event_type(r.get("event", ""))
            st = self._active.get(key)
            if kind in ("found", "persist"):
                geo = self._geometry(pad, r.get("points") or [])
                impulse = float(r.get("impulse", 0.0) or 0.0)
                is_new = st is None
                if st is None:
                    st = {"shapes": set(), "peak_impulse": 0.0, "on_pad_ever": False, "last": geo}
                    self._active[key] = st
                st["shapes"].add(shape)
                st["peak_impulse"] = max(st["peak_impulse"], impulse)
                if geo["distance_m"] is not None or geo["point_world"] is not None:
                    st["last"] = geo
                st["on_pad_ever"] = st["on_pad_ever"] or bool(geo["on_pad_surface"])
                if is_new:
                    events.append(self._event("found", pad, other, geo, impulse, len(st["shapes"])))
            elif kind == "lost":
                if st is None:
                    continue
                st["shapes"].discard(shape)
                if not st["shapes"]:
                    del self._active[key]
                    ev = self._event("lost", pad, other, st["last"], st["peak_impulse"], 0)
                    # on a lost event: was any point of this contact on the pad face
                    ev["on_pad_surface"] = st["on_pad_ever"]
                    events.append(ev)
        return events
