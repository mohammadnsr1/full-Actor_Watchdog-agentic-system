"""Small SVG drawing kit for docs/diagrams/make_diagrams.py (standard library only)."""
import shutil
import subprocess
import sys
import textwrap
from pathlib import Path

SANS = "Ubuntu Sans, Noto Sans, Helvetica Neue, Arial, sans-serif"
MONO = "Ubuntu Sans Mono, DejaVu Sans Mono, Menlo, Consolas, monospace"

INK = "#1F2329"
INK2 = "#495057"
MUTED = "#6C757D"
LINE = "#DEE2E6"
PAPER = "#F7F8FA"
WALL = "#E03131"

# theme: (strong, soft, mid)
TH = {
    "exec": ("#3B5BDB", "#EDF2FF", "#BAC8FF"),
    "wd": ("#E8590C", "#FFF4E6", "#FFC078"),
    "harn": ("#7048E8", "#F3F0FF", "#D0BFFF"),
    "ros": ("#0C8599", "#E3FAFC", "#99E9F2"),
    "sim": ("#495057", "#F1F3F5", "#CED4DA"),
    "data": ("#2F9E44", "#EBFBEE", "#B2F2BB"),
    "wall": (WALL, "#FFF5F5", "#FFC9C9"),
}
STATUS = {
    "done": ("#2F9E44", "done"),
    "built": ("#F59F00", "built, live check pending"),
    "progress": ("#1C7ED6", "in progress"),
    "planned": ("#ADB5BD", "planned"),
}

ICONS = {
    # 24x24 stroke icons
    "code": '<polyline points="8,6 2,12 8,18"/><polyline points="16,6 22,12 16,18"/><line x1="14" y1="4" x2="10" y2="20"/>',
    "eye": '<path d="M1 12s4-7 11-7 11 7 11 7-4 7-11 7S1 12 1 12z"/><circle cx="12" cy="12" r="3.2"/>',
    "robot": '<rect x="3" y="18" width="10" height="4" rx="1"/><line x1="8" y1="18" x2="8" y2="12"/><line x1="8" y1="12" x2="16" y2="6"/><line x1="16" y1="6" x2="19" y2="11"/><circle cx="8" cy="12" r="1.8"/><circle cx="16" cy="6" r="1.8"/><path d="M17 13v3M21 13v3M17 13h4"/>',
    "spark": '<path d="M12 2l2.4 7.6L22 12l-7.6 2.4L12 22l-2.4-7.6L2 12l7.6-2.4z"/>',
    "shield": '<path d="M12 2l8 3v6c0 5-3.5 9-8 11-4.5-2-8-6-8-11V5z"/><polyline points="8.5,12 11,14.5 15.5,9.5"/>',
    "camera": '<rect x="2" y="7" width="20" height="13" rx="2"/><path d="M8 7l2-3h4l2 3"/><circle cx="12" cy="13.5" r="3.5"/>',
    "clock": '<circle cx="12" cy="12" r="9.5"/><polyline points="12,6.5 12,12 16,14"/>',
    "db": '<ellipse cx="12" cy="5" rx="8" ry="3"/><path d="M4 5v14c0 1.7 3.6 3 8 3s8-1.3 8-3V5"/><path d="M4 12c0 1.7 3.6 3 8 3s8-1.3 8-3"/>',
    "box": '<path d="M12 2l9 5v10l-9 5-9-5V7z"/><polyline points="3,7 12,12 21,7"/><line x1="12" y1="12" x2="12" y2="22"/>',
    "gear": '<circle cx="12" cy="12" r="3.5"/><path d="M12 1.5v3M12 19.5v3M1.5 12h3M19.5 12h3M4.6 4.6l2.1 2.1M17.3 17.3l2.1 2.1M4.6 19.4l2.1-2.1M17.3 6.7l2.1-2.1"/><circle cx="12" cy="12" r="7"/>',
    "flag": '<line x1="5" y1="22" x2="5" y2="3"/><path d="M5 3h12l-3 4 3 4H5"/>',
    "chat": '<path d="M21 12a8 8 0 0 1-11.6 7.1L3 21l1.9-6A8 8 0 1 1 21 12z"/>',
    "check": '<circle cx="12" cy="12" r="10"/><polyline points="7,12.5 10.5,16 17,9"/>',
    "layers": '<path d="M12 2l10 5-10 5L2 7z"/><path d="M2 12l10 5 10-5"/><path d="M2 17l10 5 10-5"/>',
    "cube": '<path d="M12 2l9 5v10l-9 5-9-5V7z"/><polyline points="3,7 12,12 21,7"/><line x1="12" y1="12" x2="12" y2="22"/>',
    "play": '<circle cx="12" cy="12" r="10"/><polygon points="10,8 16,12 10,16"/>',
    "plug": '<path d="M9 2v6M15 2v6"/><path d="M6 8h12v4a6 6 0 0 1-12 0z"/><line x1="12" y1="18" x2="12" y2="22"/>',
    "doc": '<path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"/><polyline points="14,2 14,8 20,8"/><line x1="8" y1="13" x2="16" y2="13"/><line x1="8" y1="17" x2="14" y2="17"/>',
    "lab": '<path d="M9 2h6M10 2v6L4 20a1.5 1.5 0 0 0 1.3 2h13.4a1.5 1.5 0 0 0 1.3-2L14 8V2"/><line x1="6.5" y1="15" x2="17.5" y2="15"/>',
    "lock": '<rect x="4" y="11" width="16" height="11" rx="2"/><path d="M8 11V7a4 4 0 0 1 8 0v4"/>',
    "target": '<circle cx="12" cy="12" r="10"/><circle cx="12" cy="12" r="6"/><circle cx="12" cy="12" r="2"/>',
}


def esc(s):
    return str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def tw(s, size, weight="normal", mono=False):
    k = 0.61 if mono else (0.575 if weight in ("bold", "600", "700") else 0.535)
    return len(s) * size * k


def wrap(s, width, size, weight="normal", mono=False):
    k = 0.61 if mono else (0.575 if weight in ("bold", "600", "700") else 0.535)
    n = max(6, int(width / (size * k)))
    return textwrap.wrap(s, n, break_on_hyphens=False) or [""]


class Fig:
    def __init__(self, name, w, h, outdir):
        self.name, self.w, self.h, self.outdir = name, w, h, Path(outdir)
        self.o = []
        self.colors = set()
        self.warnings = []

    # ------------------------------------------------------------ primitives
    def add(self, s):
        self.o.append(s)

    def text(self, x, y, s, size=14, weight="normal", anchor="start", fill=INK,
             mono=False, italic=False, rotate=None, halo=None, ls=None, opacity=None):
        a = ""
        if italic:
            a += ' font-style="italic"'
        if rotate is not None:
            a += f' transform="rotate({rotate} {x:.1f} {y:.1f})"'
        if halo:
            a += f' stroke="{halo}" stroke-width="5" stroke-linejoin="round" paint-order="stroke"'
        if ls is not None:
            a += f' letter-spacing="{ls}"'
        if opacity is not None:
            a += f' opacity="{opacity}"'
        self.add(f'<text x="{x:.1f}" y="{y:.1f}" font-family="{MONO if mono else SANS}" '
                 f'font-size="{size}" font-weight="{weight}" text-anchor="{anchor}" '
                 f'fill="{fill}"{a}>{esc(s)}</text>')

    def para(self, x, y, w, s, size=14, fill=INK, weight="normal", mono=False,
             lh=None, anchor="start", italic=False):
        """Wrapped text; y is the top of the block. Returns the bottom."""
        lh = lh or size * 1.38
        for ln in wrap(s, w, size, weight, mono):
            y += lh
            xx = x + (w / 2 if anchor == "middle" else 0)
            self.text(xx, y - lh * 0.27, ln, size=size, fill=fill, weight=weight,
                      mono=mono, anchor=anchor, italic=italic)
        return y

    def rect(self, x, y, w, h, fill="#FFFFFF", stroke="none", rx=12, sw=1.5,
             dash=None, shadow=False, opacity=None):
        a = f' stroke-dasharray="{dash}"' if dash else ""
        if shadow:
            a += ' filter="url(#sh)"'
        if opacity is not None:
            a += f' opacity="{opacity}"'
        self.add(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{rx}" '
                 f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{a}/>')

    def circle(self, cx, cy, r, fill, stroke="none", sw=1.5, opacity=None):
        o = f' opacity="{opacity}"' if opacity is not None else ""
        self.add(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r}" fill="{fill}" stroke="{stroke}" '
                 f'stroke-width="{sw}"{o}/>')

    def line(self, x1, y1, x2, y2, color=LINE, sw=1.5, dash=None, cap="round"):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{color}" '
                 f'stroke-width="{sw}" stroke-linecap="{cap}"{d}/>')

    def path(self, d, fill="none", stroke=INK, sw=2, extra=""):
        self.add(f'<path d="{d}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" '
                 f'stroke-linecap="round" stroke-linejoin="round"{extra}/>')

    def icon(self, name, x, y, size=24, color=INK, sw=2):
        s = size / 24
        self.add(f'<g transform="translate({x:.1f} {y:.1f}) scale({s:.3f})" fill="none" '
                 f'stroke="{color}" stroke-width="{sw / s * s:.2f}" stroke-linecap="round" '
                 f'stroke-linejoin="round">{ICONS[name]}</g>')

    def badge(self, name, cx, cy, r, theme, solid=False):
        st, soft, _ = TH[theme]
        self.circle(cx, cy, r, st if solid else soft)
        self.icon(name, cx - r * 0.55, cy - r * 0.55, r * 1.1, "#FFFFFF" if solid else st, sw=2)

    # ------------------------------------------------------------ connectors
    def _marker(self, color):
        self.colors.add(color)
        return f"url(#m{color.lstrip('#')})"

    def arrow(self, d, color=INK2, sw=2, dash=None, start=False, end=True, opacity=None):
        """d: SVG path data."""
        a = ""
        if end:
            a += f' marker-end="{self._marker(color)}"'
        if start:
            a += f' marker-start="{self._marker(color)}"'
        if dash:
            a += f' stroke-dasharray="{dash}"'
        if opacity is not None:
            a += f' opacity="{opacity}"'
        self.add(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{sw}" '
                 f'stroke-linecap="round" stroke-linejoin="round"{a}/>')

    def label(self, x, y, s, color=INK2, size=12.5, anchor="middle", italic=False, weight="normal",
              bg=PAPER):
        self.text(x, y, s, size=size, fill=color, anchor=anchor, italic=italic, weight=weight,
                  halo=bg)

    # ------------------------------------------------------------ composites
    def card(self, x, y, w, h, theme, title=None, icon=None, kicker=None, title_size=17,
             fill="#FFFFFF", strip=True, rx=14):
        st, soft, _ = TH[theme]
        self.rect(x, y, w, h, fill, rx=rx, shadow=True)
        if strip:
            self.rect(x, y, w, 2 * rx, st, rx=rx)
            self.rect(x, y + 5, w, 2 * rx, fill, rx=0)
        cy = y + 14
        tx = x + 18
        if icon:
            self.badge(icon, x + 34, y + 36, 17, theme)
            tx = x + 62
        if kicker:
            self.text(tx, y + 27, kicker.upper(), size=10.5, weight="bold", fill=st, ls=1.2)
            cy = y + 30
        if title:
            ts = title_size
            for i, ln in enumerate(wrap(title, w - (tx - x) - 16, ts, "bold")):
                self.text(tx, cy + ts + 4 + i * (ts + 5), ln, size=ts, weight="bold", fill=INK)
            cy = cy + 4 + len(wrap(title, w - (tx - x) - 16, ts, "bold")) * (ts + 5)
        return max(cy + 8, y + 62 if icon else cy + 8)

    def chip(self, x, y, s, theme="sim", size=12.5, mono=True, solid=False, h=None, pad=10):
        st, soft, mid = TH[theme]
        w = tw(s, size, mono=mono) + 2 * pad
        h = h or size + 12
        self.rect(x, y, w, h, st if solid else soft, rx=h / 2)
        self.text(x + w / 2, y + h / 2 + size * 0.36, s, size=size, mono=mono,
                  fill="#FFFFFF" if solid else st, anchor="middle",
                  weight="bold" if solid else "normal")
        return w

    def pill(self, x, y, status, text=None, size=11.5, anchor="start"):
        c, default = STATUS[status]
        s = text or default
        w = tw(s, size, "bold") + 30
        if anchor == "end":
            x -= w
        self.rect(x, y, w, size + 11, c, rx=(size + 11) / 2)
        self.circle(x + 11, y + (size + 11) / 2, 3.4, "#FFFFFF")
        self.text(x + 20, y + size + 4.5, s, size=size, weight="bold", fill="#FFFFFF")
        return w

    def header(self, kicker, title, subtitle, as_of, accent="#3B5BDB"):
        self.rect(0, 0, self.w, 8, accent, rx=0)
        self.text(60, 62, kicker.upper(), size=14, weight="bold", fill=accent, ls=2.2)
        self.text(58, 108, title, size=38, weight="bold", fill=INK)
        self.para(60, 120, self.w - 520, subtitle, size=17, fill=INK2)
        self.text(self.w - 60, 62, f"state of the repo · {as_of}", size=13, fill=MUTED,
                  anchor="end")

    def legend_status(self, x, y, keys=("done", "built", "progress", "planned")):
        for k in keys:
            x += self.pill(x, y, k) + 10
        return x

    # ------------------------------------------------------------ output
    def _defs(self):
        d = ['<defs>',
             '<filter id="sh" x="-8%" y="-8%" width="116%" height="130%">'
             '<feGaussianBlur in="SourceAlpha" stdDeviation="7"/><feOffset dy="5" result="b"/>'
             '<feComponentTransfer><feFuncA type="linear" slope="0.13"/></feComponentTransfer>'
             '<feMerge><feMergeNode/><feMergeNode in="SourceGraphic"/></feMerge></filter>',
             '<pattern id="dots" width="26" height="26" patternUnits="userSpaceOnUse">'
             '<circle cx="2" cy="2" r="1.15" fill="#DADEE3"/></pattern>',
             '<pattern id="hatch" width="10" height="10" patternUnits="userSpaceOnUse" '
             'patternTransform="rotate(45)"><line x1="0" y1="0" x2="0" y2="10" stroke="#CED4DA" '
             'stroke-width="3"/></pattern>',
             '<linearGradient id="bg" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#FBFCFD"/>'
             '<stop offset="1" stop-color="#F1F3F6"/></linearGradient>']
        for k, (st, soft, mid) in TH.items():
            d.append(f'<linearGradient id="g{k}" x1="0" y1="0" x2="1" y2="1">'
                     f'<stop offset="0" stop-color="{st}"/><stop offset="1" stop-color="{mid}"/>'
                     '</linearGradient>')
            d.append(f'<linearGradient id="s{k}" x1="0" y1="0" x2="0" y2="1">'
                     f'<stop offset="0" stop-color="{soft}"/><stop offset="1" stop-color="#FFFFFF" '
                     'stop-opacity="0.6"/></linearGradient>')
        for c in sorted(self.colors):
            i = c.lstrip("#")
            d.append(f'<marker id="m{i}" viewBox="0 0 12 12" refX="10" refY="6" markerWidth="12" '
                     f'markerHeight="12" markerUnits="userSpaceOnUse" orient="auto-start-reverse">'
                     f'<path d="M1,1.5 L11,6 L1,10.5 L3.5,6 z" fill="{c}"/></marker>')
        d.append('</defs>')
        return "".join(d)

    def save(self, png_zoom=2):
        bg = (f'<rect width="{self.w}" height="{self.h}" fill="url(#bg)"/>'
              f'<rect width="{self.w}" height="{self.h}" fill="url(#dots)" opacity="0.55"/>')
        svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{self.h}" '
               f'viewBox="0 0 {self.w} {self.h}">{self._defs()}{bg}{"".join(self.o)}</svg>\n')
        p = self.outdir / f"{self.name}.svg"
        p.write_text(svg)
        for wmsg in self.warnings:
            print(f"WARNING [{self.name}]: {wmsg}", file=sys.stderr)
        if shutil.which("rsvg-convert"):
            subprocess.run(["rsvg-convert", "-z", str(png_zoom), "-o",
                            str(self.outdir / f"{self.name}.png"), str(p)], check=True)
        return p
