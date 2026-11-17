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
on the pad surface, e.g. left_contact_frame); the tracker then expresses the
contact point closest to that frame in the frame's coordinates and flags
`on_pad_surface` when it is within `surface_radius_m` of the frame origin.

Config (config/platforms/sim.yaml scene):

    contact_pads:
      - body: /World/.../left_inner_finger                       # rigid body
        surface_frame: /World/.../left_inner_finger/left_contact_frame
    pad_surface_radius_m: 0.015

A plain string entry is a body without a surface frame.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Optional, Sequence

Vec3 = Sequence[float]
ToPadFrame = Callable[[str, Sequence[Vec3]], Sequence[Vec3]]


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


def normalize_event_type(raw: Any) -> str:
    """'ContactEventType.CONTACT_FOUND' -> 'found' (also persist, lost)."""
    s = str(raw).lower()
    for kind in ("found", "persist", "lost"):
        if kind in s:
            return kind
    return s


def _norm(v: Vec3) -> float:
    return math.sqrt(sum(float(x) * float(x) for x in v))


class PadContactTracker:
    def __init__(self, pads: Sequence[Pad], surface_radius_m: Optional[float], to_pad_frame: ToPadFrame) -> None:
        self.pads = {p.body: p for p in pads}
        self.radius = float(surface_radius_m) if surface_radius_m else None
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
        frame = self.pads[pad].surface_frame
        geo: dict[str, Any] = {"pad_frame": frame, "point_world": None, "point_in_pad_frame": None, "distance_m": None, "on_pad_surface": False}
        if not points:
            return geo
        if not frame:
            geo["point_world"] = [float(x) for x in points[0]]
            return geo
        local = list(self.to_pad_frame(frame, points))
        best = min(range(len(points)), key=lambda i: _norm(local[i]))
        dist = _norm(local[best])
        geo.update(
            point_world=[float(x) for x in points[best]],
            point_in_pad_frame=[float(x) for x in local[best]],
            distance_m=dist,
            on_pad_surface=self.radius is not None and dist <= self.radius,
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
                    # on a lost event: was any point of this contact on the pad surface
                    ev["on_pad_surface"] = st["on_pad_ever"]
                    events.append(ev)
        return events
