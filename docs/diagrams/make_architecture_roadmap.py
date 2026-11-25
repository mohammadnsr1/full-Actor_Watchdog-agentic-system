#!/usr/bin/env python3
"""Draw docs/diagrams/architecture_roadmap.{svg,png}.

One poster: layered architecture, one episode, the authoring loop, design
rules, and the two roadmap tracks (build milestones M1-M8 and the
executor baseline phases EB1-EB8). Content is taken from docs/ARCHITECTURE.md,
docs/NEXT_STEPS.md, docs/PROGRESS.md, docs/EXECUTOR_BASELINE.md and
docs/INDEPENDENCE.md; edit the CONTENT below when those change, then run:

    python3 docs/diagrams/make_architecture_roadmap.py

Needs only the standard library; the PNG needs rsvg-convert (librsvg2-bin).
"""
import shutil
import subprocess
import sys
import textwrap
from pathlib import Path

OUT = Path(__file__).resolve().parent
AS_OF = "2026-11-25"
W, H = 2400, 2040

SANS = "DejaVu Sans, Inter, Helvetica, Arial, sans-serif"
MONO = "DejaVu Sans Mono, Menlo, Consolas, monospace"

# fill, stroke, text accent
TH = {
    "exec": ("#E9F1FC", "#3767B1", "#244A86"),
    "wd": ("#FDEFE6", "#C2622D", "#8E4317"),
    "harn": ("#F0EBFA", "#6A4FA8", "#4B347F"),
    "ros": ("#E5F4F0", "#2C8A78", "#1D6153"),
    "plat": ("#EAF6F3", "#2C8A78", "#1D6153"),
    "sim": ("#F1F2F4", "#5F6670", "#3A3F46"),
    "note": ("#FFFFFF", "#C9CDD3", "#3A3F46"),
}
STATUS = {"done": "#2E9D5B", "progress": "#D99A00", "planned": "#9AA0A6"}
INK = "#22262B"
MUTED = "#5D646D"
WALL = "#C0392B"

out = []
warnings = []


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def text(x, y, s, size=13, weight="normal", anchor="start", fill=INK,
         font=SANS, italic=False, rotate=None):
    extra = ' font-style="italic"' if italic else ""
    if rotate is not None:
        extra += f' transform="rotate({rotate} {x} {y})"'
    out.append(
        f'<text x="{x}" y="{y}" font-family="{font}" font-size="{size}" '
        f'font-weight="{weight}" text-anchor="{anchor}" fill="{fill}"{extra}>'
        f"{esc(s)}</text>")


def rect(x, y, w, h, fill, stroke, rx=10, sw=1.6, dash=None):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" '
               f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{d}/>')


def wrap(s, width, size, mono=False):
    cw = size * (0.61 if mono else 0.56)
    return textwrap.wrap(s, max(8, int(width / cw))) or [""]


def lines_block(x, y, width, items, size=13, gap=4, color=INK):
    """items: str or (str, style) with style in sans|mono|bold|muted|bullet."""
    for it in items:
        s, style = (it, "sans") if isinstance(it, str) else it
        mono = style == "mono"
        bullet = style == "bullet"
        indent = 16 if bullet else 0
        wl = wrap(s, width - indent, size, mono)
        for i, ln in enumerate(wl):
            y += size + gap
            if bullet and i == 0:
                out.append(f'<circle cx="{x + 5}" cy="{y - size * 0.35}" '
                           f'r="2.6" fill="{color}"/>')
            text(x + indent, y, ln, size=size,
                 weight="bold" if style == "bold" else "normal",
                 font=MONO if mono else SANS,
                 fill=MUTED if style == "muted" else INK)
        y += 2
    return y


def box(x, y, w, h, theme, title, items=(), title_size=16, size=13,
        sub=None, pad=14, rx=10, sw=1.6):
    fill, stroke, accent = TH[theme]
    rect(x, y, w, h, fill, stroke, rx=rx, sw=sw)
    ty = y + pad + title_size
    for ln in wrap(title, w - 2 * pad, title_size):
        text(x + pad, ty, ln, size=title_size, weight="bold", fill=accent)
        ty += title_size + 4
    ty -= title_size + 4
    if sub:
        ty = lines_block(x + pad, ty, w - 2 * pad, [(sub, "mono")], size=12)
    end = lines_block(x + pad, ty + 2, w - 2 * pad, items, size=size)
    if end > y + h - 4:
        warnings.append(f"overflow by {end - (y + h - 4):.0f}px: {title}")
    return end


def arrow(x1, y1, x2, y2, color=MUTED, label=None, lx=None, ly=None,
          dash=None, both=False, size=12, anchor="start", sw=1.8):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    mid = color.lstrip("#")
    ms = f' marker-start="url(#a{mid}s)"' if both else ""
    out.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" '
               f'stroke-width="{sw}"{d} marker-end="url(#a{mid})"{ms}/>')
    if label:
        text(lx if lx is not None else x1 + 8,
             ly if ly is not None else (y1 + y2) / 2 + 4,
             label, size=size, fill=color, anchor=anchor, italic=True)


def section(x, y, label):
    text(x, y, label, size=15, weight="bold", fill=MUTED)


def band_label(x, y, h, label):
    """Vertical layer label at the left edge of a band."""
    text(x, y + h / 2, label, size=12, weight="bold", fill=MUTED,
         anchor="middle", rotate=-90)


def status_pill(x, y, status, label):
    c = STATUS[status]
    w = int(len(label) * 6.9) + 18
    rect(x, y, w, 20, c, c, rx=10, sw=1)
    text(x + w / 2, y + 14, label, size=11, weight="bold", fill="#FFFFFF",
         anchor="middle")


def markers():
    cols = {MUTED, WALL, "#3767B1", "#C2622D", "#6A4FA8", "#2C8A78", "#5F6670"}
    defs = []
    for c in cols:
        i = c.lstrip("#")
        defs.append(
            f'<marker id="a{i}" viewBox="0 0 10 10" refX="9" refY="5" '
            f'markerWidth="7" markerHeight="7" orient="auto">'
            f'<path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>')
        defs.append(
            f'<marker id="a{i}s" viewBox="0 0 10 10" refX="1" refY="5" '
            f'markerWidth="7" markerHeight="7" orient="auto">'
            f'<path d="M10,0 L0,5 L10,10 z" fill="{c}"/></marker>')
    return "<defs>" + "".join(defs) + "</defs>"


# ---------------------------------------------------------------- header
rect(0, 0, W, H, "#FAFAF8", "#FAFAF8", rx=0)
text(40, 58, "Actor / WatchDog Assembly Cell", size=34, weight="bold")
text(40, 90, "Architecture, design, phases and milestones  ·  two agents that "
     "never talk to each other: an executor that writes a planner, and a "
     "plan-blind WatchDog that judges each episode", size=15, fill=MUTED)
text(40, 113, f"State as of {AS_OF}  ·  sources: docs/ARCHITECTURE.md, "
     "NEXT_STEPS.md, PROGRESS.md, EXECUTOR_BASELINE.md, INDEPENDENCE.md",
     size=12, fill=MUTED)

# legend
lx, ly = 1560, 34
for i, (th, name) in enumerate([("exec", "executor side"), ("wd", "WatchDog side"),
                                ("harn", "harness"), ("ros", "ROS 2 / platform"),
                                ("sim", "sim / hardware")]):
    xx = lx + (i % 3) * 190
    yy = ly + (i // 3) * 26
    rect(xx, yy, 18, 16, TH[th][0], TH[th][1], rx=4)
    text(xx + 26, yy + 13, name, size=12, fill=INK)
xx, yy = lx + 2 * 190, ly + 26
out.append(f'<line x1="{xx}" y1="{yy + 8}" x2="{xx + 18}" y2="{yy + 8}" '
           f'stroke="{WALL}" stroke-width="2.4" stroke-dasharray="6 4"/>')
text(xx + 26, yy + 13, "independence wall", size=12)
for i, (st, name) in enumerate([("done", "done"), ("progress", "in progress"),
                                ("planned", "planned")]):
    xx = lx + i * 190
    out.append(f'<circle cx="{xx + 9}" cy="{ly + 68}" r="7" fill="{STATUS[st]}"/>')
    text(xx + 26, ly + 73, name, size=12)

# ---------------------------------------------------------------- 1. layers
section(40, 152, "1 · LAYERED ARCHITECTURE  (read bottom up; arrows = who calls / starts whom)")
LX, LW = 70, 1420           # layer area
EX, EW = 90, 640            # executor column
WX, WW = 830, 660           # WatchDog column
WALLX = (EX + EW + WX) / 2  # 780

# band 1: agents
y1, h1 = 172, 132
band_label(52, y1, h1, "AGENTS")
box(EX, y1, EW, h1, "exec", "Executor authoring agent  (Claude Code)", [
    "Writes a planner (a program); never moves the arm itself.",
    ("Study conditions: baseline | consult (EB).  M7 conditions: plan_given | plan_free.", "muted"),
    ("runner/run_authoring.sh  ·  executor/prompts/", "mono"),
])
box(WX, y1, WW, h1, "wd", "WatchDog agent  (claude -p, one session per episode)", [
    "Plan-blind: judges against the requirements, never the steps.",
    ("Roles: conformance & safety · risk analysis (halt only for damage risk / unrecoverable) · free-form consultation", "muted"),
    ("agents/prompts/watchdog_*_plan_blind.txt", "mono"),
])

# band 2: MCP servers
y2, h2 = 336, 108
band_label(52, y2, h2, "MCP")
box(EX, y2, EW, h2, "exec", "Authoring MCP", [
    ("get_task · evaluate_planner · read_verdict · submit_work", "mono"),
    ("Feedback filtered by cell.yaml executor_feedback (watchdog | checker | both)", "muted"),
], sub="executor/authoring_mcp/server.py")
box(WX, y2, WW, h2, "wd", "WatchDog MCP  (also a ROS node)", [
    ("monitors · evidence · look · halt · rulings · issue_consultation · record_run_verdict", "mono"),
], sub="robot_backend/mcp_servers/watchdog_mcp_server.py")
arrow(EX + EW / 2, y1 + h1, EX + EW / 2, y2, both=True, label="MCP (stdio)")
arrow(WX + WW / 2, y1 + h1, WX + WW / 2, y2, both=True, label="MCP (stdio)")

# band 3: harness
y3, h3 = 484, 110
band_label(52, y3, h3, "HARNESS")
fill, stroke, acc = TH["harn"]
rect(EX, y3, WX + WW - EX, h3, fill, stroke)
text(EX + 14, y3 + 26, "Episode runner", size=16, weight="bold", fill=acc)
text(EX + 160, y3 + 26, "runner/episode_runner/run_episode.py  ·  owns exactly one episode",
     size=12, font=MONO, fill=MUTED)
steps = ["lock + health", "reset + verify", "recording", "start WatchDog",
         "START PERMISSION", "execute_task ×1", "episode_end", "checker + verdict",
         "records/<id>/"]
sx, sw_ = EX + 14, (WX + WW - EX - 28 - 8 * 10) / 9
for i, s in enumerate(steps):
    bx = sx + i * (sw_ + 10)
    hot = s == "START PERMISSION"
    rect(bx, y3 + 42, sw_, 32, "#FFFFFF" if not hot else "#6A4FA8", stroke, rx=6, sw=1.2)
    text(bx + sw_ / 2, y3 + 63, s, size=11.5, weight="bold",
         fill=acc if not hot else "#FFFFFF", anchor="middle")
    if i < len(steps) - 1:
        arrow(bx + sw_ + 1, y3 + 58, bx + sw_ + 9, y3 + 58, color="#6A4FA8", sw=1.4)
text(EX + 14, y3 + 98, "Sim budget + wall guard start at the permission; when exceeded, the "
     "runner cancels the arm's goal and stops the call.", size=12, fill=MUTED)
arrow(EX + 200, y2 + h2, EX + 200, y3, color="#3767B1",
      label="evaluate_planner → one episode each")
arrow(WX + 470, y3, WX + 470, y2 + h2, color="#C2622D",
      label="starts the session", lx=WX + 478)

# band 4: planner | WatchDog ROS side
y4, h4 = 634, 168
band_label(52, y4, h4, "RUN TIME")
box(EX, y4, EW, h4, "exec", "Planner  (stdio MCP server, one tool: execute_task)", [
    ("planner.py → worker.py (one ROS node) + geometry.py", "mono"),
    ("plan: /cell/planning/*   move: backends.motion / gripper", "mono"),
    ("poses: camera1   consult: w.consultations()", "mono"),
    ("facts: /cell/executor_facts (recorded only, never evidence)", "mono"),
], sub="executor/planner_templates/{common,plan_given,plan_free} or a snapshot")
box(WX, y4, WW, h4, "wd", "WatchDog ROS side", [
    ("wakes on: own monitors over /camera2/poses, /cell/physical_events, /joint_states, /tf, images", "mono"),
    ("halt: cancel the arm's goal   ready: /cell/signal_ready", "mono"),
    ("rulings, consultations: /cell/watchdog_notices", "mono"),
    ("verdict: run_verdict on /watchdog/events", "mono"),
])
arrow(EX + 200, y3 + h3, EX + 200, y4, color="#3767B1",
      label="MCP client: execute_task(task_ref), once")
arrow(WX + 470, y3 + h3, WX + 470, y4, color="#C2622D", both=True,
      label="episode_end in, run_verdict out", lx=WX + 478)

# independence wall (bands 1-4)
for a_, b_ in ((y1 - 6, y2 + h2 + 8), (y4 - 8, y4 + h4 + 6)):
    out.append(f'<line x1="{WALLX}" y1="{a_}" x2="{WALLX}" y2="{b_}" '
               f'stroke="{WALL}" stroke-width="2.6" stroke-dasharray="9 6"/>')
text(WALLX - 8, (y1 + y2 + h2) / 2 + 8, "INDEPENDENCE WALL", size=12,
     weight="bold", fill=WALL, anchor="middle", rotate=-90)
text(WALLX + 16, (y1 + y2 + h2) / 2 + 8, "no agent-to-agent channel", size=11,
     fill=WALL, anchor="middle", rotate=-90, italic=True)
# consultation across the wall
cy = y4 + h4 - 18
arrow(WX, cy, EX + EW, cy, color=WALL, dash="5 4")
rect(WALLX - 46, cy - 52, 92, 40, "#FAFAF8", "#FAFAF8", rx=3, sw=0)
text(WALLX, cy - 38, "consultations", size=11, fill=WALL, anchor="middle", italic=True)
text(WALLX, cy - 18, "(two-way: consult)", size=10, fill=WALL, anchor="middle", italic=True)

# band 5: platform layer
y5, h5 = 842, 96
band_label(52, y5, h5, "PLATFORM")
fill, stroke, acc = TH["plat"]
rect(EX, y5, WX + WW - EX, h5, fill, stroke)
text(EX + 14, y5 + 26, "Platform layer", size=16, weight="bold", fill=acc)
text(EX + 160, y5 + 26, "cell_platform_interfaces  ·  build_backends(node, platform) → "
     "motion, gripper, executor_poses, watchdog_poses, scene", size=12.5, font=MONO)
for i, (nm, d) in enumerate([("cell_backend_sim", "Isaac bridge, RG2 controller"),
                             ("cell_backend_hw", "UR driver, RG6 via URScript"),
                             ("config/platforms/{sim,hardware}.yaml",
                              "every topic, frame, joint, service name")]):
    pw = (WX + WW - EX - 28 - 20) / 3
    bx = EX + 14 + i * (pw + 10)
    rect(bx, y5 + 40, pw, 42, "#FFFFFF", stroke, rx=6, sw=1.2)
    text(bx + 10, y5 + 58, nm, size=12, weight="bold", font=MONO, fill=acc)
    text(bx + 10, y5 + 75, d, size=11.5, fill=MUTED)
arrow(EX + 200, y4 + h4, EX + 200, y5, color="#2C8A78", both=True,
      label="planner and runner use the same backends: unchanged on sim and hardware")
arrow(WX + 470, y4 + h4, WX + 470, y5, color="#2C8A78", both=True,
      label="plain ROS 2 topics + services", lx=WX + 478)

# band 6: cell nodes
y6, h6 = 978, 176
band_label(52, y6, h6, "CELL NODES")
fill, stroke, acc = TH["ros"]
rect(EX, y6, WX + WW - EX, h6, "#F6FBFA", stroke, sw=1.4)
text(EX + 14, y6 + 24, "ROS 2 cell nodes  (sim_moveit_bringup; Humble baseline, Python 3.10)",
     size=14, weight="bold", fill=acc)
nodes = [
    ("isaac_trajectory_bridge", "FollowJointTrajectory + RG2 open / close / guarded_move"),
    ("cell_planning_service", "/cell/planning/*: MoveIt now, cuRobo later. Plans only"),
    ("cell_orchestrator", "start permission: request_go waits for signal_ready"),
    ("cell_physical_events", "lifted · released · contact · stationary, from cell signals"),
    ("ground_truth_pose_pub", "/camera1/poses (executor), /camera2/poses (WatchDog)"),
    ("pad_contacts_relay", "/cell/pad_contacts, polled from the command server"),
    ("scene_tf_sync_node", "registry geometry → MoveIt planning scene"),
]
cw_, gx = (WX + WW - EX - 28 - 3 * 10) / 4, 10
for i, (nm, d) in enumerate(nodes):
    r, c = divmod(i, 4)
    bx, by = EX + 14 + c * (cw_ + gx), y6 + 36 + r * 68
    rect(bx, by, cw_, 60, "#FFFFFF", stroke, rx=6, sw=1.2)
    text(bx + 10, by + 19, nm, size=12, weight="bold", font=MONO, fill=acc)
    lines_block(bx + 10, by + 22, cw_ - 18, [(d, "muted")], size=11.5, gap=2)
bx, by = EX + 14 + 3 * (cw_ + gx), y6 + 36 + 68
text(bx + 6, by + 22, "cuRobo: cell_planning_curobo", size=11.5, fill=MUTED, font=MONO)
text(bx + 6, by + 40, "written, not installed yet", size=11.5, fill=MUTED, italic=True)

# band 7: sim / hardware
y7, h7 = 1194, 150
band_label(52, y7, h7, "SIM | HARDWARE")
box(EX, y7, EW, h7, "sim", "Isaac Sim 5.1  (today)", [
    ("assets/usd/assembly_room.usd: UR5e + RG2, fmb1 board (FMB Board 1 at 0.7)", "sans"),
    ("ROS 2 bridge: /clock /joint_states /joint_command /tf, camera1 (wrist) + camera2 (table)", "muted"),
    ("cell command server, TCP 9010: load_layout, reset, verify_reset, poses, contacts, timeline · GUI or headless", "muted"),
])
box(WX, y7, WW, h7, "sim", "Hardware platform  (cell_backend_hw)", [
    ("UR7e via the UR driver · OnRobot RG6 via URCap / URScript", "sans"),
    ("two Orbbec Gemini 335Lg: camera1 wrist (executor), camera2 table (WatchDog)", "muted"),
    ("UR AI Accelerator (Jetson Orin, Ubuntu 22, Humble); WatchDog + camera2 in the workstation Humble container", "muted"),
])
arrow(LX + 640, y6 + h6, LX + 640, y7, color="#5F6670", both=True,
      label="ROS 2 bridge + TCP 9010", lx=LX + 630, anchor="end")
arrow(WX + 470, y6 + h6, WX + 470, y7, color="#5F6670", both=True,
      label="UR driver, URScript, Orbbec", lx=WX + 478)

# ---------------------------------------------------------------- right column
RX, RW = 1540, 820

# 2. one episode
section(RX, 152, "2 · ONE EPISODE  (one synchronisation point: the start permission)")
ep = [
    ("BEFORE ANYTHING MOVES", "harn", [
        ("runner", "lock, wait for /clock, health check"),
        ("runner", "load_layout(assembly, layout, seed) + verify_reset"),
        ("runner", "'before' poses; start bag + frames"),
        ("runner", "declare episode_id + guard_mode to the orchestrator"),
        ("runner", "start WatchDog (plan-blind); it prepares monitors + wake conditions"),
        ("WatchDog", "signal_ready → runner's request_go released: START PERMISSION"),
    ]),
    ("DURING THE PLANNER CALL  (budgets running, no gating)", "exec", [
        ("planner", "plan via /cell/planning/*, execute via bridge, grip"),
        ("planner", "intent / hold / carry / release → /cell/executor_facts (recorded only)"),
        ("physics", "monitor fires → WatchDog wakes: halt (cancel goal), rule, consult"),
        ("planner", "returns {state: done | stopped, reason}"),
    ]),
    ("AFTER THE PLANNER", "wd", [
        ("runner", "episode_end → WatchDog's run_verdict on /watchdog/events"),
        ("runner", "'after' poses + ground-truth checker"),
        ("runner", "records/<id>/: manifest.json, verdict.json, runner.jsonl, bag, frames"),
        ("runner", "filtered verdict + halts, rulings, consultations → authoring agent"),
    ]),
]
y = 172
n = 1
for head, th, rows in ep:
    fill, stroke, acc = TH[th]
    hh = 34 + len(rows) * 24
    rect(RX, y, RW, hh, fill, stroke)
    text(RX + 14, y + 22, head, size=13, weight="bold", fill=acc)
    for j, (who, what) in enumerate(rows):
        ry = y + 46 + j * 24
        out.append(f'<circle cx="{RX + 26}" cy="{ry - 4}" r="9" fill="{stroke}"/>')
        text(RX + 26, ry, str(n), size=10.5, weight="bold", fill="#FFFFFF", anchor="middle")
        text(RX + 44, ry, who, size=12, weight="bold", fill=acc)
        text(RX + 118, ry, what, size=12.5)
        n += 1
    y += hh + 10

# 3. authoring loop
y += 18
section(RX, y, "3 · AUTHORING LOOP  (the agent improves its planner from episode feedback)")
y += 14
ly0 = y
fill, stroke, acc = TH["exec"]
cards = [
    (RX, ly0, "get_task", "task file (plan_given) or goal + constraints (plan_free)"),
    (RX + 280, ly0, "agent edits planner", "executor/workspace/<condition>/<session>/planner/"),
    (RX + 560, ly0, "evaluate_planner", "frozen snapshots/NNN → N episodes (section 2)"),
]
for cx, cy_, t, d in cards:
    rect(cx, cy_, 260, 74, fill, stroke, rx=8)
    text(cx + 12, cy_ + 22, t, size=13.5, weight="bold", fill=acc)
    lines_block(cx + 12, cy_ + 26, 240, [(d, "muted")], size=11.5, gap=2)
arrow(RX + 260, ly0 + 37, RX + 279, ly0 + 37, color="#3767B1")
arrow(RX + 540, ly0 + 37, RX + 559, ly0 + 37, color="#3767B1")
fb_y = ly0 + 96
rect(RX + 280, fb_y, 540, 50, TH["harn"][0], TH["harn"][1], rx=8)
text(RX + 292, fb_y + 21, "feedback: read_verdict", size=13.5, weight="bold", fill=TH["harn"][2])
text(RX + 292, fb_y + 40, "filtered by executor_feedback; + WatchDog consultations (consult)",
     size=11.5, fill=MUTED)
arrow(RX + 690, ly0 + 74, RX + 690, fb_y, color="#6A4FA8")
arrow(RX + 410, fb_y, RX + 410, ly0 + 75, color="#6A4FA8", label="iterate", lx=RX + 418)
rect(RX, fb_y, 260, 50, "#FFFFFF", stroke, rx=8, dash="5 3")
text(RX + 12, fb_y + 21, "submit_work → final", size=13.5, weight="bold", fill=acc)
text(RX + 12, fb_y + 40, "EB: manifest.yaml, ≤18 episodes, 8 h", size=11.5, fill=MUTED)
y = fb_y + 50 + 30

# 4. design rules
section(RX, y, "4 · DESIGN RULES")
y += 12
rules = [
    "Every ROS name comes from config/platforms/<platform>.yaml; the same code runs on sim and hardware.",
    "A planner is a stdio MCP server with one tool, execute_task(task_ref) (docs/PLANNER_CONTRACT.md).",
    "WatchDog is plan-blind: requirements file, no steps; its own camera (camera2). Executor sees camera1 only.",
    "One synchronisation point per episode (start permission); halt is a clean cancel of the arm's goal.",
    "The planning service only plans; motion goes through the backends to the bridge / UR driver.",
    "Python 3.10 / ROS 2 Humble for shared packages. Records are JSON / JSONL only, under records/.",
    "Borrow DROIDSTACK's ideas (one-tool contract, intent journal, verdict loop), not its runtime.",
]
rh = 8 + len(rules) * 20 + 14
rect(RX, y, RW, rh, TH["note"][0], TH["note"][1])
end = lines_block(RX + 14, y + 2, RW - 28, [(r, "bullet") for r in rules], size=12.5, gap=3,
                  color="#2C8A78")
if end > y + rh:
    warnings.append("rules overflow")
y += rh + 30

# what each agent can reach
section(RX, y, "WHAT EACH AGENT CAN REACH  (M7 setup; docs/INDEPENDENCE.md)")
y += 12
reach = [
    ("Tools", "executor-authoring MCP only", "robot-backend-watchdog MCP only"),
    ("Files", "read executor config + sources; edit executor/workspace/ only",
     "no Read, Glob, Grep or Bash"),
    ("Task", "config/tasks/... via get_task", "config/requirements/<assembly>.yaml (no steps)"),
    ("Part data", "config/parts/executor_parts.yaml", "config/parts/watchdog_parts.yaml"),
    ("Poses", "camera1 (/camera1/poses) via the planner",
     "camera2 (/camera2/poses); camera1 only as a flagged cue"),
    ("Never", "records/, agents/, WatchDog code + config",
     "executor's task, planner, facts as evidence, records/"),
]
c0, c1 = 96, (RW - 96) / 2
rowh = (1344 - y - 30) / len(reach)
rect(RX, y, RW, 1344 - y, "#FFFFFF", TH["note"][1])
rect(RX + c0, y, c1, 30, TH["exec"][0], TH["exec"][0], rx=0, sw=0)
rect(RX + c0 + c1, y, c1, 30, TH["wd"][0], TH["wd"][0], rx=0, sw=0)
text(RX + c0 + 12, y + 20, "Authoring agent", size=13, weight="bold", fill=TH["exec"][2])
text(RX + c0 + c1 + 12, y + 20, "WatchDog agent", size=13, weight="bold", fill=TH["wd"][2])
for i, (k, a_, b_) in enumerate(reach):
    ry = y + 30 + i * rowh
    out.append(f'<line x1="{RX}" y1="{ry}" x2="{RX + RW}" y2="{ry}" stroke="#E3E6EA"/>')
    text(RX + 12, ry + rowh / 2 + 5, k, size=12.5, weight="bold", fill=MUTED)
    for j, v in enumerate((a_, b_)):
        wl = wrap(v, c1 - 24, 12, True)
        ty = ry + rowh / 2 - (len(wl) - 1) * 8 + 4
        for ln in wl:
            text(RX + c0 + j * c1 + 12, ty, ln, size=12, font=MONO)
            ty += 16
out.append(f'<line x1="{RX + c0}" y1="{y}" x2="{RX + c0}" y2="1344" stroke="#E3E6EA"/>')
out.append(f'<line x1="{RX + c0 + c1}" y1="{y}" x2="{RX + c0 + c1}" y2="1344" stroke="{WALL}" '
           f'stroke-width="2" stroke-dasharray="6 4"/>')

# ---------------------------------------------------------------- 5. roadmap
RY = 1392
out.append(f'<line x1="40" y1="{RY - 22}" x2="{W - 40}" y2="{RY - 22}" stroke="#D5D8DC" stroke-width="1.5"/>')
section(40, RY, "5 · PHASES AND MILESTONES")


def track(y, title, subtitle, theme, items):
    fill, stroke, acc = TH[theme]
    text(40, y + 16, title, size=16, weight="bold", fill=acc)
    text(40, y + 36, subtitle, size=12, fill=MUTED)
    n_ = len(items)
    x0, gap_ = 40, 12
    cw = (W - 80 - gap_ * (n_ - 1)) / n_
    cy0 = y + 50
    ch = 132
    # spine
    out.append(f'<line x1="{x0}" y1="{cy0 + 12}" x2="{W - 40}" y2="{cy0 + 12}" '
               f'stroke="{stroke}" stroke-width="3"/>')
    for i, (code, name, st, when, desc) in enumerate(items):
        bx = x0 + i * (cw + gap_)
        c = STATUS[st]
        out.append(f'<circle cx="{bx + 22}" cy="{cy0 + 12}" r="11" fill="{c}" '
                   f'stroke="#FFFFFF" stroke-width="2.5"/>')
        rect(bx, cy0 + 30, cw, ch, "#FFFFFF" if st == "planned" else fill,
             c, rx=8, sw=2.2 if st == "progress" else 1.5,
             dash="6 4" if st == "planned" else None)
        text(bx + 12, cy0 + 56, code, size=18, weight="bold", fill=acc)
        status_pill(bx + cw - 12 - (int(len(when) * 6.9) + 18), cy0 + 40, st, when)
        ty = lines_block(bx + 12, cy0 + 62, cw - 24, [(name, "bold")], size=13, gap=3)
        end = lines_block(bx + 12, ty, cw - 24, [(desc, "muted")], size=12, gap=2)
        if end > cy0 + 30 + ch:
            warnings.append(f"card overflow {code}")
    return cy0 + 30 + ch


ya = track(RY + 14, "Track A · Build milestones  (docs/PROGRESS.md)",
           "Builds the cell: platform, planning, WatchDog independence, Isaac tooling, harness",
           "ros", [
               ("M1", "Build plan + portability audit", "done", "11-10",
                "Inventory of WatchDog_V1, 3.10/Humble audit, 12 questions answered (v1 11-03)."),
               ("M2", "Bring WatchDog in", "done", "live ✓",
                "robot_backend copied; unchanged S3 session checked live."),
               ("M3", "Platform backends + names in config", "done", "11-10",
                "cell_platform_interfaces, backend_sim / hw, config/platforms, docker/humble."),
               ("M4", "Planning service", "done", "11-10",
                "cell_planning_service (MoveIt, plan only); cuRobo package written; halt = bridge cancel."),
               ("M5", "Physical events + plan-blind WatchDog", "done", "11-11",
                "cell_physical_events; requirements file, camera2 poses, executor events dropped."),
               ("M6", "Command server, headless, cameras, recording", "done", "11-17",
                "TCP 9010 server, kit_api adapter, camera1 on wrist, MCAP + frames; 7 live checks pass."),
               ("M7", "Planner contract, authoring MCP, episode runner", "progress", "sim 11-24",
                "Sim side built and live-tested (templates, start permission, placements). Hardware side next."),
               ("M8", "Supervisor, operations, index", "planned", "next",
                "runner/supervisor, records index, docs/OPERATIONS.md; budgets + health thresholds."),
           ])

yb = track(ya + 34, "Track B · Executor baseline for the study  (docs/EXECUTOR_BASELINE.md)",
           "Rebuilds the executor like the labmate's prompt-to-assembly workspace; keeps M7's runner, contract, backends, recorder, checker",
           "exec", [
               ("EB1", "Bring in workspace pieces", "done", "11-24",
                "benchmark/ (git-ignored, pinned), 7 assemblies, ingest + resolver; fmb1 = fmb_assembly_2."),
               ("EB2", "Task bundles for this cell", "progress", "now",
                "fmb1 stage, fmb1 seats, checker modulo symmetry done (4/4); task.yaml + env/robot records next."),
               ("EB3", "Lab verbs", "planned", "next",
                "lab__* on omni-kit-mcp + command server: reset, checkpoint, contacts, camera, move_robot."),
               ("EB4", "Authoring container", "planned", "planned",
                "Docker on docker/humble, /app rw, /data read-only, fresh config, 8 h wall, egress allowlist."),
               ("EB5", "Prompts", "planned", "planned",
                "platform.md, assembly.md, docs/*.md rewritten for this cell; consult.md layer."),
               ("EB6", "Platform MCP + delivery", "planned", "planned",
                "evaluate_planner (budget 18), evidence dirs, submit_work, manifest.yaml, check-delivery."),
               ("EB7", "WatchDog in the lab", "planned", "planned",
                "consult only, no halts; standing session, watchdog_notes, request_consultation."),
               ("EB8", "Protocol + metrics", "planned", "planned",
                "3 runs per condition, same model; then a 9-seed panel per frozen planner."),
           ])

# bottom: research question + open items
by_ = yb + 30
bw = (W - 80 - 20) / 2
box(40, by_, bw, H - 30 - by_, "harn", "Research question: do free-form WatchDog consultations speed up authoring?", [
    ("baseline: no WatchDog.  consult: WatchDog consults during lab work, evaluation episodes and the panel (no halts); the executor may request consultations.", "sans"),
    ("Metric: evaluation episodes to the first PASS, wall time, tokens / cost, lab calls, consultations. Results in records/study/.", "muted"),
], title_size=15, size=12.5)
box(40 + bw + 20, by_, bw, H - 30 - by_, "note", "Open items", [
    ("cuRobo: written, not installed or run; every episode so far planned with MoveIt.", "bullet"),
    ("Hardware: camera2 pose pipeline (no FoundationPose yet); M7 hardware side not started.", "bullet"),
    ("Layout jitter ranges (9-seed panel); table physics material; resolver frame check (16.7 mm).", "bullet"),
], title_size=15, size=12.5)

svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
       f'viewBox="0 0 {W} {H}">' + markers() + "".join(out) + "</svg>\n")
(OUT / "architecture_roadmap.svg").write_text(svg)
for w_ in warnings:
    print("WARNING:", w_, file=sys.stderr)
if shutil.which("rsvg-convert"):
    subprocess.run(["rsvg-convert", "-z", "2", "-o", str(OUT / "architecture_roadmap.png"),
                    str(OUT / "architecture_roadmap.svg")], check=True)
    print("wrote", OUT / "architecture_roadmap.svg", "and .png")
else:
    print("wrote", OUT / "architecture_roadmap.svg", "(rsvg-convert missing: no PNG)")
