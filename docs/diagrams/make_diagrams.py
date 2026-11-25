#!/usr/bin/env python3
"""Draw the figures in docs/diagrams/ (SVG + PNG at 2x).

    01_overview            the whole system on one slide
    02_architecture        every layer, from Isaac / hardware up to the two agents
    03_episode             one episode as a swim-lane sequence
    04_executor_baseline   the study: host vs container, conditions, delivery loop
    05_assembly_pipeline   fmb1 data -> stage -> acceptance modulo symmetry
    06_roadmap             milestones M1-M8 and phases EB1-EB8 on a dated timeline

Content comes from docs/ARCHITECTURE.md, NEXT_STEPS.md, PROGRESS.md,
EXECUTOR_BASELINE.md and INDEPENDENCE.md. When those change, edit the text
below and run (standard library only; PNGs need rsvg-convert):

    python3 docs/diagrams/make_diagrams.py            # all
    python3 docs/diagrams/make_diagrams.py 06_roadmap # one
"""
import sys
from datetime import date
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from svgkit import INK, INK2, MUTED, LINE, PAPER, WALL, TH, STATUS, Fig, tw, wrap  # noqa: E402

OUT = Path(__file__).resolve().parent
AS_OF = "2026-11-25"
FIGS = {}


def figure(fn):
    FIGS[fn.__name__.lstrip("f")] = fn
    return fn


def sub_box(f, x, y, w, h, theme, title, body, mono_body=False, size=14):
    st, soft, mid = TH[theme]
    f.rect(x, y, w, h, soft, rx=10)
    f.rect(x, y, 4, h, st, rx=2)
    f.text(x + 16, y + 26, title, size=15.5, weight="bold", fill=st)
    end = f.para(x + 16, y + 34, w - 30, body, size=size, fill=INK2, mono=mono_body)
    if end > y + h:
        f.warnings.append(f"sub_box overflow {title} by {end - y - h:.0f}")
    return end


# =====================================================================================
@figure
def f01_overview():
    f = Fig("01_overview", 1920, 1200, OUT)
    f.header("Actor / WatchDog assembly cell · overview",
             "Two agents, one robot cell, no shared channel",
             "An executor writes a planner program and a harness runs it on the cell; a plan-blind "
             "WatchDog watches every episode and judges it against the requirements. The study asks "
             "whether the WatchDog's free-form consultations make the executor's authoring faster.",
             AS_OF)

    # ---- executor card
    ex, ey, ew, eh = 60, 250, 540, 540
    f.card(ex, ey, ew, eh, "exec", "Writes a program, not motions", icon="code", kicker="executor side")
    sub_box(f, ex + 20, ey + 82, ew - 40, 118, "exec", "Authoring agent",
            "Claude Code in an isolated container, up to 8 h. Explores the cell with lab verbs, "
            "then writes and tests a planner.")
    f.arrow(f"M{ex + ew / 2} {ey + 204} L{ex + ew / 2} {ey + 236}", TH["exec"][0], sw=2.4)
    f.label(ex + ew / 2 + 12, ey + 224, "writes + evaluates", TH["exec"][0], anchor="start",
            italic=True, bg="#FFFFFF")
    sub_box(f, ex + 20, ey + 240, ew - 40, 118, "exec", "Planner",
            "A stdio MCP server with exactly one tool, execute_task(task_ref). Plans through the "
            "planning service, moves through the backends.")
    y = ey + 384
    f.text(ex + 20, y, "TOOLS", size=11, weight="bold", fill=MUTED, ls=1.2)
    f.chip(ex + 20, y + 10, "kit MCP · 12 lab verbs", "exec")
    f.chip(ex + 230, y + 10, "platform MCP · budget 18", "exec")
    y += 62
    f.text(ex + 20, y, "SEES", size=11, weight="bold", fill=MUTED, ls=1.2)
    f.chip(ex + 20, y + 10, "camera1 · wrist", "exec")
    f.chip(ex + 180, y + 10, "task view /data", "exec")
    f.chip(ex + 340, y + 10, "no records/", "wall")

    # ---- WatchDog card
    wx, wy, ww, wh = 1320, 250, 540, 540
    f.card(wx, wy, ww, wh, "wd", "Judges outcomes, never steps", icon="eye", kicker="WatchDog side")
    sub_box(f, wx + 20, wy + 82, ww - 40, 118, "wd", "WatchDog agent",
            "claude -p, one session per episode. Plan-blind: it reads the requirements, never the "
            "task, the planner or its facts.")
    f.arrow(f"M{wx + ww / 2} {wy + 204} L{wx + ww / 2} {wy + 236}", TH["wd"][0], sw=2.4)
    f.label(wx + ww / 2 + 12, wy + 224, "acts through", TH["wd"][0], anchor="start", italic=True,
            bg="#FFFFFF")
    sub_box(f, wx + 20, wy + 240, ww - 40, 118, "wd", "Its own MCP server",
            "monitors · evidence · look · halt · rulings · consultations · run_verdict. "
            "Halts only for damage risk.")
    y = wy + 384
    f.text(wx + 20, y, "ROLES", size=11, weight="bold", fill=MUTED, ls=1.2)
    f.chip(wx + 20, y + 10, "conformance", "wd", mono=False)
    f.chip(wx + 150, y + 10, "risk analysis", "wd", mono=False)
    f.chip(wx + 285, y + 10, "consultation", "wd", mono=False)
    y += 62
    f.text(wx + 20, y, "SEES", size=11, weight="bold", fill=MUTED, ls=1.2)
    f.chip(wx + 20, y + 10, "camera2 · table", "wd")
    f.chip(wx + 182, y + 10, "physical events", "wd")
    f.chip(wx + 352, y + 10, "requirements", "wd")

    # ---- the cell (illustration)
    cx, cy, cw, ch = 690, 300, 540, 490
    f.rect(cx, cy, cw, ch, "url(#ssim)", rx=22)
    f.text(cx + cw / 2, cy + 36, "THE CELL", size=13, weight="bold", fill=INK2, anchor="middle", ls=2.5)
    f.text(cx + cw / 2, cy + 58, "Isaac Sim 5.1 today  ·  UR5e + RG2  ·  fmb1 board", size=13.5,
           fill=MUTED, anchor="middle")
    ty = cy + 360  # table top
    # camera2 view cone + pole
    f.add(f'<polygon points="{cx + 470},{cy + 118} {cx + 250},{ty} {cx + 440},{ty}" '
          f'fill="{TH["wd"][0]}" opacity="0.10"/>')
    f.rect(cx + 478, cy + 118, 8, ty - cy - 118, "#ADB5BD", rx=3)
    f.rect(cx + 452, cy + 104, 44, 26, TH["wd"][0], rx=6)
    f.circle(cx + 468, cy + 117, 7, "#FFFFFF")
    f.circle(cx + 468, cy + 117, 4, TH["wd"][0])
    f.text(cx + 446, cy + 98, "camera2", size=12.5, weight="bold", fill=TH["wd"][0], anchor="end")
    # table
    f.rect(cx + 40, ty, cw - 80, 16, "#ADB5BD", rx=4)
    f.rect(cx + 70, ty + 16, 14, 90, "#CED4DA", rx=3)
    f.rect(cx + cw - 84, ty + 16, 14, 90, "#CED4DA", rx=3)
    # board + seated parts
    bx = cx + 300
    f.rect(bx, ty - 22, 130, 22, "#868E96", rx=4)
    f.rect(bx + 10, ty - 36, 22, 14, "#A0522D", rx=3)   # u_brown seated
    f.rect(bx + 42, ty - 36, 50, 14, "#2F9E44", rx=3)   # line_green seated
    # parts waiting on the table
    f.rect(cx + 70, ty - 14, 26, 14, "#F76707", rx=3)  # u_orange
    # robot
    rb = (cx + 190, ty)
    f.rect(rb[0] - 34, ty - 46, 68, 46, "#343A40", rx=8)
    j1 = (rb[0], ty - 58)
    j2 = (rb[0] + 70, cy + 170)
    j3 = (cx + 345, cy + 205)
    j4 = (cx + 365, cy + 262)
    for a, b, wdt in ((j1, j2, 26), (j2, j3, 22), (j3, j4, 16)):
        f.line(a[0], a[1], b[0], b[1], "#CED4DA", sw=wdt)
        f.line(a[0], a[1], b[0], b[1], "#E9ECEF", sw=wdt - 10)
    for j, r in ((j1, 17), (j2, 15), (j3, 12), (j4, 10)):
        f.circle(j[0], j[1], r, "#495057")
        f.circle(j[0], j[1], r - 6, "#74C0FC")
    # gripper + held part
    gx, gy = j4
    f.rect(gx - 16, gy + 8, 32, 12, "#495057", rx=3)
    f.line(gx - 11, gy + 20, gx - 11, gy + 44, "#495057", sw=5)
    f.line(gx + 11, gy + 20, gx + 11, gy + 44, "#495057", sw=5)
    f.rect(gx - 8, gy + 30, 16, 20, "#FAB005", rx=2)  # inverted_u_yellow in hand
    # camera1 on the wrist + cone
    f.add(f'<polygon points="{gx + 22},{gy - 6} {gx - 30},{ty} {gx + 90},{ty}" '
          f'fill="{TH["exec"][0]}" opacity="0.10"/>')
    f.rect(gx + 12, gy - 16, 24, 16, TH["exec"][0], rx=4)
    f.circle(gx + 24, gy - 8, 4, "#FFFFFF")
    f.text(gx + 42, gy - 18, "camera1", size=12.5, weight="bold", fill=TH["exec"][0])
    f.text(cx + cw / 2, ty + 70, "ROS 2 · every topic, frame and joint name from",
           size=13, fill=INK2, anchor="middle")
    f.text(cx + cw / 2, ty + 90, "config/platforms/<platform>.yaml", size=13, fill=INK2,
           anchor="middle", mono=True)
    f.text(cx + cw / 2, ty + 116, "hardware next: UR7e · OnRobot RG6 · 2× Orbbec Gemini 335Lg",
           size=12.5, fill=MUTED, anchor="middle", italic=True)

    # ---- flows between the three
    f.arrow(f"M{ex + ew} {ey + 300} C{ex + ew + 60} {ey + 300} {cx - 40} {cy + 250} {cx + 150} {cy + 250}",
            TH["exec"][0], sw=2.6)
    f.label(ex + ew + 46, ey + 282, "runs on the arm", TH["exec"][0], anchor="start", italic=True)
    f.arrow(f"M{cx + cw - 50} {cy + 250} C{cx + cw + 30} {cy + 250} {wx - 60} {wy + 300} {wx} {wy + 300}",
            TH["wd"][0], sw=2.6)
    f.label(wx - 46, wy + 282, "observes physics", TH["wd"][0], anchor="end", italic=True)
    # consultation arc over the top
    f.arrow(f"M{wx + 60} {wy} C{wx - 40} {wy - 60} {ex + ew + 40} {wy - 60} {ex + ew - 60} {ey}",
            WALL, sw=2.4, dash="8 6")
    f.label(960, wy - 74, "consultations: the only channel between the two sides", WALL, size=13.5,
            weight="bold")
    f.label(960, wy - 54, "WatchDog → executor; requests back only in the consult condition", WALL,
            size=12.5, italic=True)

    # ---- episode runner bar
    ry = 830
    f.rect(60, ry, 1800, 92, "url(#gharn)", rx=18, shadow=True)
    f.badge("play", 104, ry + 46, 22, "harn")
    f.text(140, ry + 40, "EPISODE RUNNER", size=13, weight="bold", fill="#FFFFFF", ls=2)
    f.text(140, ry + 64, "owns exactly one episode", size=14, fill="#F3F0FF")
    steps = ["lock + health", "reset + verify", "record", "start WatchDog", "START PERMISSION",
             "execute_task ×1", "episode_end", "checker + verdict", "records/<id>/"]
    x = 370
    for s in steps:
        w = tw(s, 13, "bold") + 26
        hot = s == "START PERMISSION"
        f.rect(x, ry + 30, w, 32, "#FFFFFF" if hot else "#FFFFFF", rx=16, opacity=None if hot else 0.18)
        f.text(x + w / 2, ry + 51, s, size=13, weight="bold", fill=TH["harn"][0] if hot else "#FFFFFF",
               anchor="middle")
        x += w + 8
        if s != steps[-1]:
            f.text(x - 5, ry + 51, "›", size=16, fill="#FFFFFF", anchor="middle", weight="bold")
            x += 6
    if x > 1850:
        f.warnings.append(f"runner chips end at {x}")
    f.arrow(f"M{ex + 150} {ry} L{ex + 150} {ey + eh + 4}", TH["harn"][0], sw=2.4, start=True)
    f.label(ex + 162, ry - 12, "execute_task(task_ref), once · verdict back", TH["harn"][0], anchor="start",
            italic=True)
    f.arrow(f"M{wx + ww - 150} {ry} L{wx + ww - 150} {wy + wh + 4}", TH["harn"][0], sw=2.4, start=True)
    f.label(wx + ww - 162, ry - 12, "starts it · episode_end · run_verdict", TH["harn"][0], anchor="end",
            italic=True)
    f.arrow(f"M{960} {ry} L{960} {cy + ch + 4}", TH["harn"][0], sw=2.4, start=True)
    f.label(972, ry - 12, "reset · record · check", TH["harn"][0], anchor="start", italic=True)

    # ---- footer tiles
    fy, fh = 960, 205
    f.card(60, fy, 400, fh, "data", strip=False)
    f.text(90, fy + 44, "LATEST RESULT", size=11.5, weight="bold", fill=TH["data"][0], ls=1.5)
    f.text(88, fy + 122, "4/4", size=78, weight="bold", fill=TH["data"][0])
    f.para(250, fy + 66, 190, "parts placed on the fmb1 board by the plan-given template",
           size=14, fill=INK2)
    f.text(90, fy + 178, "fmb1-given-s1 · seeds 0 and 2 · 2026-11-25", size=12.5, fill=MUTED, mono=True)

    f.card(480, fy, 920, fh, "sim", strip=False)
    f.text(510, fy + 44, "WHERE THE BUILD STANDS", size=11.5, weight="bold", fill=INK2, ls=1.5)
    tracks = [
        ("Build", ["done"] * 6 + ["progress", "planned"],
         ["M1", "M2", "M3", "M4", "M5", "M6", "M7", "M8"], {6: "M7 sim ✓"}),
        ("Baseline", ["done", "done", "built", "built", "built", "built", "planned", "planned"],
         ["EB1", "EB2", "EB3", "EB4", "EB5", "EB6", "EB7", "EB8"], {}),
    ]
    for i, (name, sts, codes, ovr) in enumerate(tracks):
        yy = fy + 64 + i * 52
        f.text(510, yy + 24, name, size=14, weight="bold", fill=INK)
        sx, sw_ = 600, 96
        for j, (st, cd) in enumerate(zip(sts, codes)):
            c = STATUS[st][0]
            f.rect(sx + j * (sw_ + 4), yy + 4, sw_, 30, c, rx=8)
            f.text(sx + j * (sw_ + 4) + sw_ / 2, yy + 24, ovr.get(j, cd), size=13, weight="bold",
                   fill="#FFFFFF", anchor="middle")
    f.legend_status(510, fy + 170)

    f.card(1420, fy, 440, fh, "harn", strip=False)
    f.text(1450, fy + 44, "NEXT", size=11.5, weight="bold", fill=TH["harn"][0], ls=1.5)
    nxt = [("1", "Live check of EB3–EB6 on the cell"),
           ("2", "EB7 · WatchDog in the lab (consult)"),
           ("3", "EB8 · 3 runs per condition + 9-seed panel"),
           ("·", "M7 hardware side · M8 operations")]
    for i, (n, s) in enumerate(nxt):
        yy = fy + 72 + i * 32
        f.circle(1462, yy - 5, 11, TH["harn"][0] if n != "·" else TH["harn"][2])
        f.text(1462, yy, n, size=12, weight="bold", fill="#FFFFFF", anchor="middle")
        f.text(1484, yy, s, size=14.5, fill=INK)
    return f


# =====================================================================================
@figure
def f02_architecture():
    f = Fig("02_architecture", 1920, 1700, OUT)
    f.header("Detailed view · architecture",
             "Every layer, from the simulator to the two agents",
             "Read bottom up. Arrows show who calls or starts whom. The red line is the independence "
             "wall: nothing crosses it except the WatchDog's consultations.", AS_OF)
    L, R = 150, 1860
    EX, EW = L, 880
    WX = 1090
    WW = R - WX
    WALLX = (EX + EW + WX) / 2

    def rail(y, h, label, theme):
        st, soft, _ = TH[theme]
        f.rect(60, y, 54, h, soft, rx=12)
        f.text(87, y + h / 2, label, size=12.5, weight="bold", fill=st, anchor="middle", rotate=-90, ls=1.6)

    # ---- A. agents
    ya, ha = 210, 158
    rail(ya, ha, "AGENTS", "exec")
    f.card(EX, ya, EW, ha, "exec", "Executor authoring agent", icon="spark", kicker="Claude Code")
    f.para(EX + 20, ya + 66, EW - 40,
           "Study path (EB): claude --print in the cell-authoring container, prompt on stdin, up to 8 h; "
           "runner/run_baseline.sh baseline|consult. M7 path: runner/run_authoring.sh plan_given|plan_free.",
           size=14, fill=INK2)
    f.text(EX + 20, ya + ha - 16, "writes a planner; touches the arm only through lab tools", size=13.5,
           fill=TH["exec"][0], italic=True)
    f.card(WX, ya, WW, ha, "wd", "WatchDog agent", icon="eye", kicker="claude -p · one session per episode")
    f.para(WX + 20, ya + 66, WW - 40,
           "Plan-blind: judges against config/requirements (no steps). Roles: conformance and safety, "
           "risk analysis (halt only for damage risk or unrecoverable), free-form consultation.",
           size=14, fill=INK2)
    f.text(WX + 20, ya + ha - 16, "agents/prompts/watchdog_*_plan_blind.txt", size=13, fill=TH["wd"][0],
           mono=True)

    # ---- B. MCP servers
    yb, hb = 420, 214
    rail(yb, hb, "MCP", "exec")
    cw3 = (EW - 2 * 14) / 3
    mcps = [
        ("kit MCP", "gateway → omni.kit.mcp bridge :9009",
         "lab.resolve checkpoint restore set reset_scene rest contacts_arm contacts_read "
         "camera_capture camera_view verify_assembly events · run_python · files", "built"),
        ("platform MCP", "HTTP :9020 · executor/platform_mcp",
         "check_delivery · evaluate_planner (budget 18) · submit_work · move_robot · robot_health", "built"),
        ("authoring MCP", "stdio · M7 path · executor/authoring_mcp",
         "get_task · evaluate_planner · read_verdict · submit_work · feedback filtered by executor_feedback",
         "done"),
    ]
    for i, (nm, where, tools, st) in enumerate(mcps):
        x = EX + i * (cw3 + 14)
        f.card(x, yb, cw3, hb, "exec", nm, icon="plug")
        f.text(x + 18, yb + 82, where, size=11.5, fill=MUTED, mono=True)
        end = f.para(x + 18, yb + 88, cw3 - 32, tools, size=12.5, fill=INK, mono=True)
        f.pill(x + cw3 - 14, yb + 16, st, "built" if st == "built" else None, anchor="end")
        if end > yb + hb - 6:
            f.warnings.append(f"mcp card overflow {nm}")
    f.card(WX, yb, WW, hb, "wd", "WatchDog MCP", icon="plug", kicker="also a ROS node")
    f.text(WX + 20, yb + 84, "robot_backend/mcp_servers/watchdog_mcp_server.py", size=12, fill=MUTED, mono=True)
    f.para(WX + 20, yb + 92, WW - 40,
           "monitors · wake conditions · evidence · look · halt · issue_ruling · issue_consultation · "
           "inspect_acceptance · assembly_reference · record_run_verdict", size=13, mono=True)
    for x in (EX + EW / 2, WX + WW / 2):
        f.arrow(f"M{x} {ya + ha + 6} L{x} {yb - 6}", INK2, sw=2, start=True)
        f.label(x + 10, (ya + ha + yb) / 2 + 5, "MCP", INK2, anchor="start", italic=True)

    # ---- C. harness
    yc, hc = 690, 126
    rail(yc, hc, "HARNESS", "harn")
    f.rect(EX, yc, R - EX, hc, "url(#gharn)", rx=16, shadow=True)
    f.badge("play", EX + 34, yc + 34, 18, "harn")
    f.text(EX + 62, yc + 30, "EPISODE RUNNER", size=13, weight="bold", fill="#FFFFFF", ls=2)
    f.text(EX + 62, yc + 50, "runner/episode_runner/run_episode.py · one episode, one cell lock",
           size=13, fill="#F3F0FF", mono=True)
    steps = ["lock + health", "reset + verify", "record bag + frames", "start WatchDog",
             "START PERMISSION", "execute_task ×1", "episode_end", "checker", "records/<id>/"]
    x = EX + 24
    for s in steps:
        w = tw(s, 13, "bold") + 26
        hot = s == "START PERMISSION"
        f.rect(x, yc + 72, w, 32, "#FFFFFF", rx=16, opacity=None if hot else 0.18)
        f.text(x + w / 2, yc + 93, s, size=13, weight="bold", fill=TH["harn"][0] if hot else "#FFFFFF",
               anchor="middle")
        x += w + 20
        if s != steps[-1]:
            f.text(x - 10, yc + 93, "›", size=17, weight="bold", fill="#FFFFFF", anchor="middle")
    f.text(R - 20, yc + 30, "sim budget + wall guard start at the permission",
           size=13, fill="#F3F0FF", anchor="end", italic=True)
    f.text(R - 20, yc + 50, "exceeded → the runner cancels the arm's goal", size=13, fill="#F3F0FF",
           anchor="end", italic=True)
    f.arrow(f"M{EX + 300} {yb + hb + 6} L{EX + 300} {yc - 6}", TH["exec"][0], sw=2.2)
    f.label(EX + 312, (yb + hb + yc) / 2 + 5, "evaluate_planner → episodes", TH["exec"][0],
            anchor="start", italic=True)
    f.arrow(f"M{WX + 240} {yc - 6} L{WX + 240} {yb + hb + 6}", TH["wd"][0], sw=2.2)
    f.label(WX + 252, (yb + hb + yc) / 2 + 5, "starts the session", TH["wd"][0], anchor="start", italic=True)

    # ---- D. run time
    yd, hd = 870, 214
    rail(yd, hd, "RUN TIME", "exec")
    f.card(EX, yd, EW, hd, "exec", "Planner", icon="code", kicker="stdio MCP server · one tool: execute_task")
    rows = [("from", "artifact/ (manifest.yaml) · planner_templates/{baseline, plan_given, plan_free}"),
            ("shape", "planner.py → worker.py (one ROS node) + geometry.py"),
            ("plan", "/cell/planning/*  (plans only, never moves the arm)"),
            ("move", "backends.motion / gripper · Worker.move_joints / jog"),
            ("poses", "camera1 (/camera1/poses) · RGB-D frames"),
            ("facts", "/cell/executor_facts (recorded, never evidence)")]
    for i, (k, v) in enumerate(rows):
        yy = yd + 92 + i * 20
        f.text(EX + 20, yy, k, size=12.5, weight="bold", fill=TH["exec"][0])
        f.text(EX + 82, yy, v, size=12.5, fill=INK, mono=True)
    f.card(WX, yd, WW, hd, "wd", "WatchDog ROS side", icon="eye", kicker="wakes on physics only")
    rows = [("wakes", "monitors over /camera2/poses, /cell/physical_events"),
            ("", "/joint_states, /tf, images, /cell/lab_events"),
            ("halt", "cancel the arm's goal"),
            ("ready", "/cell/signal_ready (start permission)"),
            ("notes", "/cell/watchdog_notices (rulings, consults)"),
            ("verdict", "run_verdict on /watchdog/events")]
    for i, (k, v) in enumerate(rows):
        yy = yd + 92 + i * 20
        f.text(WX + 20, yy, k, size=12.5, weight="bold", fill=TH["wd"][0])
        f.text(WX + 86, yy, v, size=12.5, fill=INK, mono=True)
    f.arrow(f"M{EX + 300} {yc + hc + 6} L{EX + 300} {yd - 6}", TH["harn"][0], sw=2.2)
    f.label(EX + 312, (yc + hc + yd) / 2 + 5, "execute_task(task_ref), once", TH["harn"][0], anchor="start",
            italic=True)
    f.arrow(f"M{WX + 240} {yc + hc + 6} L{WX + 240} {yd - 6}", TH["harn"][0], sw=2.2, start=True)
    f.label(WX + 252, (yc + hc + yd) / 2 + 5, "episode_end in · run_verdict out", TH["harn"][0],
            anchor="start", italic=True)
    # consultation across the wall
    f.arrow(f"M{WX - 4} {yd + hd - 40} L{EX + EW + 6} {yd + hd - 40}", WALL, sw=2.4, dash="7 5")
    f.label(WALLX, yd + hd - 52, "consults", WALL, size=12.5, weight="bold")
    # wall segments
    for a, b in ((ya - 10, yb + hb + 10), (yd - 10, yd + hd + 10)):
        f.line(WALLX, a, WALLX, b, WALL, sw=3, dash="10 7")
    f.text(WALLX - 8, (ya + yb + hb) / 2, "INDEPENDENCE WALL", size=12, weight="bold", fill=WALL,
           anchor="middle", rotate=-90, ls=1.5, halo=PAPER)

    # ---- E. platform layer
    ye, he = 1136, 104
    rail(ye, he, "PLATFORM", "ros")
    f.rect(EX, ye, R - EX, he, "#FFFFFF", rx=14, shadow=True)
    f.rect(EX, ye, 6, he, TH["ros"][0], rx=3)
    f.text(EX + 24, ye + 30, "cell_platform_interfaces", size=15, weight="bold", fill=TH["ros"][0], mono=True)
    f.text(EX + 270, ye + 30, "build_backends(node, platform) → motion · gripper · executor_poses · "
           "watchdog_poses · scene", size=13.5, fill=INK, mono=True)
    items = [("cell_backend_sim", "Isaac bridge · RG2 controller · lab_events_relay"),
             ("cell_backend_hw", "UR driver · RG6 through URScript"),
             ("config/platforms/{sim,hardware}.yaml", "every topic, frame, joint, service name")]
    pw = (R - EX - 48 - 20) / 3
    for i, (nm, d) in enumerate(items):
        x = EX + 24 + i * (pw + 10)
        f.rect(x, ye + 46, pw, 44, TH["ros"][1], rx=8)
        f.text(x + 12, ye + 64, nm, size=12.5, weight="bold", fill=TH["ros"][0], mono=True)
        f.text(x + 12, ye + 82, d, size=12.5, fill=INK2)
    f.arrow(f"M{EX + 300} {yd + hd + 6} L{EX + 300} {ye - 6}", TH["ros"][0], sw=2.2, start=True)
    f.label(EX + 312, (yd + hd + ye) / 2 + 5, "same backends for planner and runner → unchanged on sim and hardware",
            TH["ros"][0], anchor="start", italic=True)
    f.arrow(f"M{WX + 240} {yd + hd + 6} L{WX + 240} {ye - 6}", TH["ros"][0], sw=2.2, start=True)
    f.label(WX + 252, (yd + hd + ye) / 2 + 5, "plain ROS 2 topics + services", TH["ros"][0], anchor="start",
            italic=True)

    # ---- F. cell nodes
    yf, hf = 1290, 172
    rail(yf, hf, "CELL NODES", "ros")
    nodes = [("isaac_trajectory_bridge", "FollowJointTrajectory · RG2 open / close / guarded_move"),
             ("cell_planning_service", "/cell/planning/* · MoveIt now, cuRobo written"),
             ("cell_orchestrator", "start permission: request_go waits for signal_ready"),
             ("cell_physical_events", "lifted · released · contact · stationary"),
             ("ground_truth_pose_publisher", "/camera1/poses (executor) · /camera2/poses (WatchDog)"),
             ("pad_contacts_relay", "/cell/pad_contacts from the command server"),
             ("lab_events_relay", "/cell/lab_events: reset, set, teleport, move"),
             ("scene_tf_sync_node", "registry geometry → MoveIt planning scene")]
    nw = (R - EX - 3 * 12) / 4
    for i, (nm, d) in enumerate(nodes):
        r_, c_ = divmod(i, 4)
        x, y = EX + c_ * (nw + 12), yf + r_ * 90
        f.rect(x, y, nw, 80, "#FFFFFF", rx=12, shadow=True)
        f.circle(x + 20, y + 22, 6, TH["ros"][0])
        f.text(x + 34, y + 27, nm, size=13, weight="bold", fill=TH["ros"][0], mono=True)
        f.para(x + 16, y + 34, nw - 28, d, size=12.5, fill=INK2)
    f.arrow(f"M{EX + 300} {ye + he + 6} L{EX + 300} {yf - 6}", TH["ros"][0], sw=2.2, start=True)

    # ---- G. sim | hardware
    yg, hg = 1500, 168
    rail(yg, hg, "SIM | HW", "sim")
    f.card(EX, yg, EW, hg, "sim", "Isaac Sim 5.1 · today", icon="robot", kicker="sim platform · ROS 2 Jazzy")
    f.para(EX + 20, yg + 66, EW - 40,
           "assets/usd/assembly_room.usd: UR5e + RG2, fmb1 board. ROS 2 bridge: /clock /joint_states "
           "/joint_command /tf, camera1 (wrist) + camera2 (table). Cell command server, TCP 9010: "
           "load_layout, reset, verify_reset, poses, contacts, timeline, 12 lab_* verbs. "
           "omni.kit.mcp bridge :9009. GUI or headless.", size=13.5, fill=INK2)
    f.card(WX, yg, WW, hg, "sim", "Hardware platform · next", icon="robot", kicker="hardware · ROS 2 Humble")
    f.para(WX + 20, yg + 66, WW - 40,
           "UR7e through the UR driver on the UR AI Accelerator (Jetson Orin). OnRobot RG6 via URCap / "
           "URScript. Two Orbbec Gemini 335Lg (FoundationPose not built yet). WatchDog + camera2 in the "
           "workstation Humble container.", size=13.5, fill=INK2)
    f.arrow(f"M{EX + 300} {yf + hf + 6} L{EX + 300} {yg - 6}", INK2, sw=2.2, start=True)
    f.label(EX + 312, (yf + hf + yg) / 2 + 5, "ROS 2 bridge · TCP 9010 · bridge 9009", INK2, anchor="start",
            italic=True)
    f.arrow(f"M{WX + 240} {yf + hf + 6} L{WX + 240} {yg - 6}", INK2, sw=2.2, start=True)
    f.label(WX + 252, (yf + hf + yg) / 2 + 5, "UR driver · URScript · Orbbec", INK2, anchor="start",
            italic=True)
    return f


# =====================================================================================
@figure
def f03_episode():
    f = Fig("03_episode", 1920, 1590, OUT)
    f.header("Detailed view · one episode",
             "One episode, one synchronisation point",
             "The WatchDog prepares while nothing moves. The start permission is the only point where "
             "the two sides wait for each other; the budgets start there. After it, only physics wakes "
             "the WatchDog.", AS_OF, accent=TH["harn"][0])
    lanes = [("Authoring /\nplatform MCP", "exec", "spark"), ("Episode runner", "harn", "play"),
             ("Orchestrator", "ros", "lock"), ("WatchDog", "wd", "eye"),
             ("Planner", "exec", "code"), ("Cell · Isaac + ROS 2", "sim", "robot")]
    X = [230, 530, 830, 1130, 1430, 1730]
    top = 210
    bottom = 1560
    for (nm, th, ic), x in zip(lanes, X):
        f.line(x, top + 80, x, bottom, TH[th][2], sw=2.5, dash="2 7")
        f.rect(x - 125, top, 250, 72, "#FFFFFF", rx=14, shadow=True)
        f.rect(x - 125, top, 250, 6, TH[th][0], rx=3)
        f.badge(ic, x - 90, top + 39, 18, th)
        parts = nm.split("\n")
        for i, p in enumerate(parts):
            f.text(x - 62, top + 44 - (len(parts) - 1) * 9 + i * 19, p, size=15.5, weight="bold")

    y = top + 120
    n = [0]

    def phase(title, theme, rows):
        nonlocal y
        st, soft, _ = TH[theme]
        h = 46 + rows * 46
        f.rect(40, y, 1840, h, soft, rx=16, opacity=0.75)
        f.text(62, y + 28, title, size=12.5, weight="bold", fill=st, ls=1.8)
        y0 = y
        y += 52
        return y0, h

    def num(x, yy, color):
        n[0] += 1
        f.circle(x, yy, 11, color)
        f.text(x, yy + 4.5, str(n[0]), size=11.5, weight="bold", fill="#FFFFFF", anchor="middle")

    def msg(a, b, s, color=INK2, dash=None, hot=False):
        nonlocal y
        x1, x2 = X[a], X[b]
        d = 1 if x2 > x1 else -1
        f.arrow(f"M{x1 + d * 8} {y + 16} L{x2 - d * 10} {y + 16}", color, sw=3 if hot else 2, dash=dash)
        mx = (x1 + x2) / 2
        if hot:
            w = tw(s, 13.5, "bold") + 30
            f.rect(mx - w / 2, y - 12, w, 26, color, rx=13)
            f.text(mx, y + 5, s, size=13.5, weight="bold", fill="#FFFFFF", anchor="middle")
        else:
            f.label(mx, y + 6, s, color, size=13.5, bg="#FFFFFF")
        num(min(x1, x2) + 2 if False else x1 - d * 24, y + 16, color)
        y += 46

    def self_(a, s, color):
        nonlocal y
        x = X[a]
        w = tw(s, 13.5) + 28
        f.rect(x + 18, y + 2, w, 28, "#FFFFFF", stroke=color, rx=8, sw=1.5)
        f.text(x + 32, y + 21, s, size=13.5, fill=INK)
        f.circle(x, y + 16, 6, color)
        num(x - 24, y + 16, color)
        y += 46

    E, H_, O, W_, P, C = range(6)
    ex, hv, ros, wd, sim = TH["exec"][0], TH["harn"][0], TH["ros"][0], TH["wd"][0], TH["sim"][0]
    phase("BEFORE ANYTHING MOVES", "harn", 10)
    msg(E, H_, "run.sh --planner snapshot --task --seed", ex)
    self_(H_, "lock records/.cell.lock · wait for /clock · health check", hv)
    msg(H_, C, "load_layout(assembly, layout, seed) + verify_reset", hv)
    msg(H_, C, "report_poses → 'before' snapshot · start bag + frames", hv)
    msg(H_, O, "declare episode_id + guard_mode", hv)
    msg(H_, W_, "start claude -p (plan-blind), session dir records/<id>/watchdog", hv)
    self_(W_, "prepare: requirements, contracts, monitors + wake conditions", wd)
    msg(W_, O, "signal_ready", wd)
    msg(H_, O, "request_go(phase=run)", hv)
    msg(O, H_, "START PERMISSION", hv, hot=True)
    y += 18
    y0, h0 = phase("DURING THE PLANNER CALL · budgets running · no gating", "exec", 7)
    msg(H_, P, "execute_task(task_ref) · exactly once", hv)
    msg(P, C, "plan /cell/planning/* · execute trajectory · grip", ex)
    msg(P, C, "facts → /cell/executor_facts (recorded only)", ex, dash="6 5")
    msg(C, W_, "a monitor fires → wait_for_event returns", sim)
    msg(W_, C, "halt = cancel the arm's goal (damage risk only)", wd, dash="6 5")
    msg(W_, P, "ruling · consultation", WALL, dash="6 5")
    msg(P, H_, "{state: done | stopped, reason}", ex)
    # budget bracket
    bx = 1868
    f.path(f"M{bx - 10} {y0 + 50} L{bx} {y0 + 50} L{bx} {y0 + h0 - 10} L{bx - 10} {y0 + h0 - 10}",
           stroke=hv, sw=2)
    f.text(bx - 10, y0 + h0 / 2 + 20, "sim budget + wall guard", size=12.5, weight="bold", fill=hv,
           anchor="middle", rotate=-90, halo=TH["exec"][1])
    y += 18
    phase("AFTER THE PLANNER", "wd", 5)
    msg(H_, W_, "episode_end notice", hv)
    msg(W_, H_, "run_verdict on /watchdog/events", wd)
    msg(H_, C, "report_poses → 'after' snapshot · ground-truth checker", hv)
    self_(H_, "records/<id>/ · manifest.json · verdict.json · runner.jsonl · bag/ · frames/", hv)
    msg(H_, E, "filtered verdict + consultations", hv)
    if y > f.h - 20:
        f.warnings.append(f"episode rows end at {y}")
    return f


# =====================================================================================
@figure
def f04_executor_baseline():
    f = Fig("04_executor_baseline", 1920, 1290, OUT)
    f.header("Detailed view · the executor baseline study (EB1–EB8)",
             "Does a second agent's perspective speed up authoring?",
             "The executor is set up like the labmate's prompt-to-assembly workspace, so the only "
             "difference between the two conditions is the WatchDog's consultations.", AS_OF)

    # ---- host / container
    top = 200
    hx, hw = 60, 560
    cx, cw = 700, 520
    hh = 660
    f.card(hx, top, hw, hh, "sim", "Host · the cell", icon="gear", kicker="Isaac + ROS 2 Jazzy")
    host = [
        ("Isaac Sim + cell command server", "TCP 9010 · lab.py: 12 lab_* verbs, one implementation",
         "sim"),
        ("omni.kit.mcp bridge :9009", "isaac/lab_tools → lab.* tools · run_python · files", "sim"),
        ("platform MCP :9020", "check_delivery · evaluate_planner · submit_work · move_robot · robot_health",
         "exec"),
        ("episode runner (M7)", "runs the artifact's run.sh → execute_task, records evidence", "harn"),
        ("lab_events_relay", "/cell/lab_events: a teleport is never read as physics", "ros"),
    ]
    hy = top + 84
    for nm, d, th in host:
        sub_box(f, hx + 20, hy, hw - 40, 100, th, nm, d, size=13.5)
        hy += 112
    f.pill(hx + hw - 18, top + 18, "built", "built 11-25", anchor="end")

    f.card(cx, top, cw, hh, "exec", "Container · cell-authoring", icon="box", kicker="docker/authoring · host network")
    f.pill(cx + cw - 18, top + 18, "built", "built 11-25", anchor="end")
    sub_box(f, cx + 20, top + 84, cw - 40, 128, "exec", "Claude Code --print",
            "Fresh config, no memory or skills. First message on stdin: platform.md + assembly.md; "
            "docs/*.md read from /data. Up to 8 h.", size=13.5)
    sub_box(f, cx + 20, top + 224, cw - 40, 100, "exec", "MCP servers",
            "kit → bridge :9009 (lab verbs) · platform → :9020", size=13.5)
    f.text(cx + 20, top + 360, "MOUNTS", size=11, weight="bold", fill=MUTED, ls=1.4)
    mounts = [("/app", "rw", "artifact/ · report.md"), ("/app/evaluations", "ro", "evidence per evaluation"),
              ("/data", "ro", "task view: task, graph, prompts"), ("/opt/cell", "ro", "planner-side packages")]
    for i, (m, mode, d) in enumerate(mounts):
        yy = top + 376 + i * 40
        f.rect(cx + 20, yy, cw - 40, 32, TH["exec"][1], rx=8)
        f.text(cx + 34, yy + 21, m, size=13.5, weight="bold", fill=TH["exec"][0], mono=True)
        f.chip(cx + 200, yy + 5, mode, "data" if mode == "rw" else "sim", size=11.5, h=22)
        f.text(cx + 250, yy + 21, d, size=13, fill=INK2)
    f.para(cx + 20, top + 548, cw - 40,
           "ROS 2 topics through host networking (raw data streams, accepted). Open: egress allowlist; a "
           "filter in front of run_python.", size=13, fill=MUTED, italic=True)
    # arrows container -> host
    for yy, s, c in ((top + 252, "lab.* via kit", TH["exec"][0]),
                     (top + 312, "platform (HTTP)", TH["exec"][0])):
        f.arrow(f"M{cx - 4} {yy} L{hx + hw + 6} {yy}", c, sw=2.4)
        f.label((cx + hx + hw) / 2, yy - 10, s, c, size=12.5, italic=True)

    # ---- conditions (right)
    rx, rw = 1260, 600
    f.card(rx, top, rw, 360, "harn", "Two conditions, everything else identical", icon="layers",
           kicker="same model · same seeds · 8 h · budget 18")
    cols = [("baseline", TH["exec"]), ("consult", TH["wd"])]
    c0 = 150
    cwid = (rw - 40 - c0) / 2
    for i, (nm, th) in enumerate(cols):
        x = rx + 20 + c0 + i * cwid
        f.rect(x + 4, top + 80, cwid - 8, 30, th[0], rx=10)
        f.text(x + cwid / 2, top + 100, nm, size=14, weight="bold", fill="#FFFFFF", anchor="middle")
    rows = [("prompt", "base layers", "+ consult layer"),
            ("lab work", "no WatchDog", "standing session, consults"),
            ("episodes", "no WatchDog", "per-episode session"),
            ("halts", "–", "off: consult only"),
            ("WD → exec", "–", "consultations"),
            ("exec → WD", "–", "request_consultation")]
    for j, (k, a, b) in enumerate(rows):
        yy = top + 140 + j * 36
        f.line(rx + 20, yy - 22, rx + rw - 20, yy - 22, LINE, sw=1)
        f.text(rx + 24, yy, k, size=13.5, weight="bold", fill=INK2)
        f.text(rx + 20 + c0 + cwid / 2, yy, a, size=13.5, fill=MUTED, anchor="middle")
        f.text(rx + 20 + c0 + cwid + cwid / 2, yy, b, size=13.5, fill=INK, anchor="middle")

    f.card(rx, top + 384, rw, 276, "data", "What is measured", icon="target", kicker="EB8 · planned")
    ms = [("episodes to the first PASS", "the headline"), ("wall time · tokens · cost", "stream-json usage"),
          ("lab calls · consultations", "how the agent worked"),
          ("9-seed panel", "each frozen planner, unseen seeds")]
    for i, (a, b) in enumerate(ms):
        yy = top + 384 + 92 + i * 42
        f.circle(rx + 32, yy - 5, 5, TH["data"][0])
        f.text(rx + 46, yy, a, size=15, weight="bold", fill=INK)
        f.text(rx + rw - 22, yy, b, size=13, fill=MUTED, anchor="end")
    f.text(rx + 20, top + 384 + 262, "3 runs per condition · results in records/study/", size=13,
           fill=TH["data"][0], italic=True)

    # ---- delivery loop (bottom)
    by = 900
    f.text(60, by, "THE AUTHORING LOOP", size=13, weight="bold", fill=INK2, ls=2)
    f.text(290, by, "what the agent does inside its 8 h, and what the platform enforces", size=14,
           fill=MUTED)
    steps = [("1", "Explore", "lab verbs: resolve, set, checkpoint / restore, contacts, camera, move_robot",
              "exec", "lab"),
             ("2", "Write", "artifact/manifest.yaml + a stdio server with execute_task(task_ref)", "exec",
              "code"),
             ("3", "check_delivery", "manifest schema · contract lint · live tool surface = execute_task only",
              "harn", "shield"),
             ("4", "evaluate_planner", "1–3 episodes per call · budget 18, charged per completed episode",
              "harn", "play"),
             ("5", "Read evidence", "/app/evaluations/NNNN/: verdict.txt, planner.jsonl, bag, frames", "exec",
              "doc"),
             ("6", "submit_work", "done is refused unless the current artifact was evaluated", "data",
              "flag")]
    n = len(steps)
    gap = 26
    sw_ = (1800 - gap * (n - 1)) / n
    yy = by + 36
    for i, (k, t, d, th, ic) in enumerate(steps):
        x = 60 + i * (sw_ + gap)
        f.card(x, yy, sw_, 180, th, t, icon=ic, kicker=f"step {k}")
        f.para(x + 18, yy + 76, sw_ - 34, d, size=14.5, fill=INK2)
        if i < n - 1:
            f.arrow(f"M{x + sw_ + 3} {yy + 110} L{x + sw_ + gap - 3} {yy + 110}", INK2, sw=2.2)
    # iterate arc from 5 back to 2
    x2 = 60 + 1 * (sw_ + gap) + sw_ / 2
    x5 = 60 + 4 * (sw_ + gap) + sw_ / 2
    f.arrow(f"M{x5} {yy + 184} C{x5} {yy + 250} {x2} {yy + 250} {x2} {yy + 186}", TH["exec"][0], sw=2.4,
            dash="7 5")
    f.label((x2 + x5) / 2, yy + 262, "iterate until it passes or the budget runs out", TH["exec"][0],
            size=13.5, italic=True)
    f.text(60 + 5 * (sw_ + gap) + sw_ / 2, yy + 214, "→ frozen planner → 9-seed panel", size=13.5,
           fill=TH["data"][0], anchor="middle", weight="bold")
    f.legend_status(60, 1240, ("done", "built", "planned"))
    f.text(560, 1256, "EB1–EB2 done · EB3–EB6 built offline, waiting for the live check · EB7–EB8 planned",
           size=13.5, fill=INK2)
    return f


# =====================================================================================
@figure
def f05_assembly_pipeline():
    f = Fig("05_assembly_pipeline", 1920, 1300, OUT)
    f.header("Detailed view · the assembly, the stage and acceptance",
             "From the labmate's fmb1 data to a judged episode",
             "This cell's fmb_assembly_2 is the labmate's fmb1 (FMB Board 1 at 0.7). Its bundles, seats "
             "and proved symmetries are put on the stage by one tool, and both judges use the same "
             "acceptance rule.", AS_OF, accent=TH["data"][0])
    top, h = 200, 520
    # source
    sx, sw = 60, 400
    f.card(sx, top, sw, h, "data", "benchmark/assemblies/fmb1", icon="db", kicker="prompt-to-assembly · pinned")
    src = [("bundles/BodyN.usda", "visual mesh, box colliders, printed_plastic 0.8/0.6, mass, inertia"),
           ("fmb1.assembly-graph", "assembled poses (the seats), mates, interchangeable classes"),
           ("BodyN.part-graph", "proved symmetry certificates per part")]
    yy = top + 84
    for a, b in src:
        f.rect(sx + 20, yy, sw - 40, 100, TH["data"][1], rx=10)
        f.text(sx + 36, yy + 28, a, size=14, weight="bold", fill=TH["data"][0], mono=True)
        f.para(sx + 36, yy + 34, sw - 72, b, size=13.5, fill=INK2)
        yy += 112
    f.para(sx + 20, yy + 4, sw - 40, "Apache-2.0 · git-ignored, rebuilt byte-identical by "
           "tools/fetch_benchmark.sh", size=12.5, fill=MUTED, italic=True)

    # build
    bx, bw = 540, 480
    f.card(bx, top, bw, h, "harn", "tools/build_fmb1_stage.py", icon="gear", kicker="one tool, Isaac's own USD")
    f.text(bx + 20, top + 96, "BINDING · assets/usd/fmb1/binding.json", size=11.5, weight="bold", fill=MUTED, ls=1)
    binds = [("Body1", "base1", "#868E96", "fixture, Rz(-90)"), ("Body2", "u_brown", "#A0522D", "U pair"),
             ("Body5", "u_orange", "#F76707", "U pair"), ("Body3", "line_green", "#2F9E44", ""),
             ("Body4", "inverted_u_yellow", "#FAB005", "")]
    for i, (b, p, col, note) in enumerate(binds):
        yy = top + 112 + i * 42
        f.chip(bx + 20, yy, b, "sim", size=13)
        f.arrow(f"M{bx + 100} {yy + 12} L{bx + 140} {yy + 12}", INK2, sw=1.8)
        f.rect(bx + 150, yy, 22, 24, col, rx=5)
        f.text(bx + 182, yy + 17, p, size=14, weight="bold", fill=INK, mono=True)
        f.text(bx + bw - 20, yy + 17, note, size=12.5, fill=MUTED, anchor="end")
    f.text(bx + 20, top + 344, "CELL PHYSICS", size=11.5, weight="bold", fill=MUTED, ls=1)
    for i, s in enumerate(["base1 kinematic", "depenetration 0.5 m/s", "speculative CCD"]):
        f.chip(bx + 20 + [0, 148, 330][i], top + 356, s, "harn", size=12, mono=False)
    f.para(bx + 20, top + 396, bw - 40,
           "Every prim name and frame kept, so no pose, grasp offset or config changed. "
           "Also: --placements, --cell-physics, --check-only; clean_stage_render.py strips the render "
           "products Isaac saves into the file.", size=13, fill=INK2)
    f.arrow(f"M{sx + sw + 4} {top + h / 2} L{bx - 6} {top + h / 2}", TH["data"][0], sw=2.6)

    # outputs
    ox, ow = 1100, 380
    outs = [("assets/usd/assembly_room.usd", "the stage: fmb1 parts, frames, physics", "sim"),
            ("/World/base1/<part>_placement", "placement frames = fmb1's seats", "sim"),
            ("config/placements/fmb_assembly_2.yaml", "seats relative to base1 + each part's symmetries", "wd"),
            ("task_data/ (make_task_data.py)", "task, environment, robot records in the labmate's shapes", "exec")]
    oh = (h - 3 * 16) / 4
    for i, (a, b, th) in enumerate(outs):
        y = top + i * (oh + 16)
        f.rect(ox, y, ow, oh, "#FFFFFF", rx=12, shadow=True)
        f.rect(ox, y, 6, oh, TH[th][0], rx=3)
        f.text(ox + 22, y + 34, a, size=13.5, weight="bold", fill=TH[th][0], mono=True)
        f.para(ox + 22, y + 42, ow - 40, b, size=13.5, fill=INK2)
        f.arrow(f"M{bx + bw + 4} {top + h / 2} C{bx + bw + 40} {top + h / 2} {ox - 40} {y + oh / 2} "
                f"{ox - 6} {y + oh / 2}", TH["harn"][0], sw=2)

    # consumers
    kx, kw = 1560, 300
    f.card(kx, top, kw, 240, "exec", "Executor sees", icon="camera")
    f.para(kx + 20, top + 66, kw - 40, "camera1 / TF poses as Isaac publishes them; the task view in /data "
           "(graph, seats in the graph); pictures of the finished board.", size=13.5, fill=INK2)
    f.card(kx, top + 280, kw, 240, "wd", "WatchDog side judges", icon="shield")
    f.para(kx + 20, top + 346, kw - 40, "frames derived from base1's observed pose (camera2): "
           "inspect_acceptance, cell_physical_events, the runner's checker.", size=13.5, fill=INK2)
    y1 = top + 1 * (oh + 16) + oh / 2
    y2 = top + 2 * (oh + 16) + oh / 2
    f.arrow(f"M{ox + ow + 4} {y1} C{ox + ow + 40} {y1} {kx - 40} {top + 120} {kx - 6} {top + 120}",
            TH["exec"][0], sw=2)
    f.arrow(f"M{ox + ow + 4} {y2} C{ox + ow + 40} {y2} {kx - 40} {top + 400} {kx - 6} {top + 400}",
            TH["wd"][0], sw=2)

    # ---- acceptance illustration
    ay, ah = 770, 490
    f.card(60, ay, 860, ah, "wd", "Acceptance, modulo symmetry", icon="target",
           kicker="runner checker = WatchDog inspect_acceptance")
    # board (top view, schematic)
    ox_, oy_ = 110, ay + 120
    f.rect(ox_, oy_, 380, 250, "#868E96", rx=14)
    f.rect(ox_ + 10, oy_ + 10, 360, 230, "#ADB5BD", rx=10)
    f.text(ox_ + 190, oy_ + 274, "fmb1 board · top view · schematic, not to scale", size=12, fill=MUTED,
           anchor="middle", italic=True)

    def u(x, y, w, h, col, inverted=False):
        t = 16
        if not inverted:
            d = (f"M{x} {y} h{t} v{h - t} h{w - 2 * t} v{-(h - t)} h{t} v{h} h{-w} z")
        else:
            d = (f"M{x} {y + h} v{-h} h{w} v{h} h{-t} v{-(h - t)} h{-(w - 2 * t)} v{h - t} z")
        f.path(d, fill=col, stroke="#FFFFFF", sw=2)

    u(ox_ + 40, oy_ + 40, 80, 80, "#A0522D")
    u(ox_ + 40, oy_ + 140, 80, 80, "#F76707")
    f.rect(ox_ + 160, oy_ + 50, 40, 160, "#2F9E44", rx=4, stroke="#FFFFFF", sw=2)
    u(ox_ + 240, oy_ + 80, 100, 100, "#FAB005", inverted=True)
    # swap arrow
    f.arrow(f"M{ox_ + 24} {oy_ + 84} C{ox_ - 20} {oy_ + 110} {ox_ - 20} {oy_ + 150} {ox_ + 24} {oy_ + 176}",
            WALL, sw=2.4, start=True)
    # tolerance ring
    f.add(f'<circle cx="{ox_ + 290}" cy="{oy_ + 130}" r="72" fill="none" stroke="{TH["wd"][0]}" '
          f'stroke-width="2" stroke-dasharray="5 5"/>')
    tx = 540
    rules = [("Within tolerance", "each part at its placement frame, within its registry band", TH["wd"][0]),
             ("Modulo symmetry", "a part at frame ⊗ S occupies the same space: it counts as placed",
              TH["harn"][0]),
             ("Interchangeable", "u_brown / u_orange may fill each other's frames, one per frame", WALL),
             ("Same rule twice", "runner ground-truth checker and WatchDog inspect_acceptance", TH["data"][0])]
    for i, (a, b, c) in enumerate(rules):
        yy = ay + 120 + i * 82
        f.rect(tx, yy, 6, 62, c, rx=3)
        f.text(tx + 20, yy + 22, a, size=15.5, weight="bold", fill=INK)
        f.para(tx + 20, yy + 28, 340, b, size=13.5, fill=INK2)

    # ---- from 0/4 to 4/4
    fx, fw = 960, 900
    f.card(fx, ay, fw, ah, "data", "From 0/4 to 4/4 on the fmb1 stage", icon="check",
           kicker="what the first fmb1 episode taught")
    cols = [("fmb1-given-s0", "0/4", WALL, [
        "1.6 mm lateral error vs ±1.4 mm slot play",
        "free-space approach swept into a seated part",
        "overlap resolved by throwing a part 4.9 m",
        "seats 1–6.55 mm off fmb1's CAD"]),
        ("fmb1-given-s1", "4/4", TH["data"][0], [
            "hover, then a straight descent",
            "in-hand correction before each descent",
            "kinematic base · depenetration cap · CCD",
            "placement frames re-derived from fmb1's seats"])]
    for i, (ep, sc, c, items) in enumerate(cols):
        x = fx + 24 + i * 440
        f.text(x, ay + 120, sc, size=54, weight="bold", fill=c)
        f.text(x + 120, ay + 104, ep, size=15, weight="bold", fill=INK, mono=True)
        f.text(x + 120, ay + 124, "2026-11-24" if i == 0 else "2026-11-25 · seeds 0 and 2", size=13,
               fill=MUTED)
        for j, s in enumerate(items):
            yy = ay + 170 + j * 64
            f.rect(x, yy, 400, 52, TH["wall"][1] if i == 0 else TH["data"][1], rx=10)
            f.icon("target" if i == 0 else "check", x + 12, yy + 15, 22, c)
            f.para(x + 44, yy + 13, 345, s, size=14, fill=INK)
    f.arrow(f"M{fx + 434} {ay + 330} L{fx + 458} {ay + 330}", TH["data"][0], sw=2.6)
    return f


# =====================================================================================
@figure
def f06_roadmap():
    f = Fig("06_roadmap", 1920, 1480, OUT)
    f.header("Phases and milestones",
             "Five weeks in: the cell works, the study harness is built",
             "Build milestones M1–M8 and executor-baseline phases EB1–EB8 on one dated axis. Past work "
             "sits at its real dates; what comes next is ordered, not dated.", AS_OF, accent=TH["ros"][0])
    f.legend_status(60, 150, ("done", "built", "planned"))

    D0 = date(2026, 10, 30)
    X0, X1 = 380, 1400     # 10-20 .. 11-26
    days = (date(2026, 11, 26) - D0).days
    pxd = (X1 - X0) / days

    def X(m, d):
        return X0 + (date(2026, m, d) - D0).days * pxd

    FZ0, FZ1 = 1440, 1860
    slot = (FZ1 - FZ0) / 3

    def S(i):
        return FZ0 + i * slot

    top = 230
    rows_top = top + 110
    rh = 40
    groups = [
        ("BUILD TRACK", "ros", [
            ("M1", "Build plan", [((11, 2), (11, 3), "done", ""), ((11, 10), (11, 10), "done", "v2")]),
            ("M2", "Bring WatchDog in", [((11, 3), (11, 3), "done", "live ✓")]),
            ("M3", "Platforms, names in config", [((11, 10), (11, 10), "done", "live ✓")]),
            ("M4", "Planning service", [((11, 3), (11, 10), "done", "MoveIt · cuRobo written")]),
            ("M5", "Physical events, plan-blind WD", [((11, 11), (11, 11), "done", "")]),
            ("M6", "Command server, cameras, recording", [((11, 11), (11, 17), "done", "7 live checks ✓")]),
            ("M7", "Planner contract, runner, authoring", [((11, 17), (11, 24), "done", "sim ✓"),
                                                          ("F", 0, 3, "planned", "hardware side")]),
            ("M8", "Supervisor, operations, index", [("F", 1, 3, "planned", "ops doc · records index")]),
        ]),
        ("EXECUTOR BASELINE", "exec", [
            ("EB1", "Workspace pieces (benchmark/)", [((11, 24), (11, 24), "done", "")]),
            ("EB2", "Task bundles, fmb1 stage, checker", [((11, 24), (11, 25), "done", "4/4")]),
            ("EB3", "Lab verbs", [((11, 25), (11, 25), "built", ""), ("F", 0, 1, "built", "live check")]),
            ("EB4", "Authoring container", [((11, 25), (11, 25), "built", ""), ("F", 0, 1, "built", "live check")]),
            ("EB5", "Prompts (baseline layers)", [((11, 25), (11, 25), "built", ""),
                                                 ("F", 1, 2, "planned", "consult layer")]),
            ("EB6", "Platform MCP + delivery", [((11, 25), (11, 25), "built", ""), ("F", 0, 1, "built", "live check")]),
            ("EB7", "WatchDog in the lab (consult)", [("F", 1, 2, "planned", "consult only")]),
            ("EB8", "Protocol + metrics, the study", [("F", 2, 3, "planned", "runs + panel")]),
        ]),
    ]
    nrows = sum(len(g[2]) for g in groups) + len(groups)
    bottom = rows_top + nrows * rh + 10

    # axis background, weeks, future zone
    f.rect(X0 - 10, top + 60, FZ1 - X0 + 20, bottom - top - 60, "#FFFFFF", rx=16, shadow=True)
    f.rect(FZ0 - 14, top + 60, FZ1 - FZ0 + 24, bottom - top - 60, "url(#hatch)", rx=16, opacity=0.35)
    d = date(2026, 10, 30)
    from datetime import timedelta
    while d <= date(2026, 11, 26):
        x = X(d.month, d.day)
        if d.weekday() == 0:
            f.line(x, top + 70, x, bottom - 6, "#E9ECEF", sw=1.2)
            f.text(x, top + 90, d.strftime("%b %d").replace(" 0", " "), size=12.5, fill=MUTED, anchor="middle")
        d += timedelta(days=1)
    for i, s in enumerate(["next", "then", "later"]):
        f.text(S(i) + slot / 2, top + 90, s, size=13, weight="bold", fill=INK2, anchor="middle")
    f.text((FZ0 + FZ1) / 2, top + 50, "AHEAD · ordered, not dated", size=12, weight="bold", fill=MUTED,
           anchor="middle", ls=1.4)
    # today line
    xt = X(11, 25) + pxd / 2
    f.line(xt, top + 70, xt, bottom - 6, TH["harn"][0], sw=2.5, dash="4 4")
    f.rect(xt - 34, top + 60, 68, 22, TH["harn"][0], rx=11)
    f.text(xt, top + 75, "today", size=12, weight="bold", fill="#FFFFFF", anchor="middle")

    # event flags
    f.text(X0 + 4, top + 50, "← repo started Oct 20", size=12.5, fill=MUTED, italic=True)
    events = [((11, 2), "survey: borrow ideas, not runtime"),
              ((11, 10), "v2 plan: hardware platform"), ((11, 24), "decision: executor = baseline")]
    for i, ((m, dd), s) in enumerate(events):
        x = X(m, dd) + pxd / 2
        yy = top + 4 + (i % 2) * 24
        f.line(x, yy + 4, x, top + 62, TH["sim"][2], sw=1.5)
        f.circle(x, yy + 4, 4, INK2)
        f.text(x + (8 if i < 2 else -8), yy + 8, s, size=12.5, fill=INK2, anchor="start" if i < 2 else "end")

    y = rows_top
    for gname, gth, rows in groups:
        st = TH[gth][0]
        f.text(80, y + 26, gname, size=12, weight="bold", fill=st, ls=1.8)
        f.line(X0, y + 20, FZ1, y + 20, TH[gth][2], sw=1.5)
        y += rh
        for code, name, bars in rows:
            f.text(80, y + 26, code, size=15, weight="bold", fill=st)
            f.text(132, y + 26, name, size=14, fill=INK)
            for b in bars:
                if b[0] == "F":
                    _, i0, i1, stt, lab = b
                    x0, x1 = S(i0) + 6, S(i1) - 6
                else:
                    (m0, d0), (m1, d1), stt, lab = b
                    x0, x1 = X(m0, d0) + 2, X(m1, d1) + pxd - 2
                c = STATUS[stt][0]
                if stt == "planned":
                    f.rect(x0, y + 8, x1 - x0, 24, "#FFFFFF", stroke=c, rx=12, sw=2, dash="5 4")
                elif b[0] == "F":
                    f.rect(x0, y + 8, x1 - x0, 24, c, rx=12, opacity=0.35)
                    f.rect(x0, y + 8, x1 - x0, 24, "none", stroke=c, rx=12, sw=2, dash="5 4")
                else:
                    f.rect(x0, y + 8, max(x1 - x0, 14), 24, c, rx=12)
                if lab:
                    inside = tw(lab, 12.5, "bold") < (x1 - x0 - 16)
                    if inside:
                        solid = b[0] != "F" and stt != "planned"
                        f.text((x0 + x1) / 2, y + 25, lab, size=12.5, weight="bold",
                               fill="#FFFFFF" if solid else (INK2 if stt != "built" else "#8A5A00"),
                               anchor="middle")
                    else:
                        f.text(x1 + 8, y + 25, lab, size=12.5, fill=INK2, weight="bold")
            y += rh

    # ---- episodes chart
    cy0 = bottom + 40
    f.card(60, cy0, 900, 1480 - cy0 - 30, "exec", "Parts placed per episode (of 4)", icon="robot",
           kicker="episodes on the cell, in order")
    gx0, gx1 = 140, 920
    gy0, gy1 = cy0 + 90, cy0 + 300  # top, baseline
    for v in range(0, 5):
        yy = gy1 - v * (gy1 - gy0) / 4
        f.line(gx0, yy, gx1, yy, "#E9ECEF" if v else "#ADB5BD", sw=1.2)
        f.text(gx0 - 12, yy + 4, str(v), size=12.5, fill=MUTED, anchor="end")
    eps = [("m7-first-s0-r3", "11-23 · old stage", 1), ("m7-given-s0", "11-24 · old stage", 3),
           ("fmb1-given-s0", "11-24 · fmb1 stage", 0), ("fmb1-given-s1", "11-25 · fmb1 + fixes", 4)]
    bwid = 92
    step = (gx1 - gx0) / len(eps)
    for i, (nm, sub, v) in enumerate(eps):
        xc = gx0 + step * (i + 0.5)
        hgt = v * (gy1 - gy0) / 4
        if v:
            f.path(f"M{xc - bwid / 2} {gy1} V{gy1 - hgt + 4} Q{xc - bwid / 2} {gy1 - hgt} {xc - bwid / 2 + 4} "
                   f"{gy1 - hgt} H{xc + bwid / 2 - 4} Q{xc + bwid / 2} {gy1 - hgt} {xc + bwid / 2} "
                   f"{gy1 - hgt + 4} V{gy1} Z", fill=TH["exec"][0], stroke="none", sw=0)
        else:
            f.line(xc - bwid / 2, gy1, xc + bwid / 2, gy1, TH["exec"][0], sw=3)
        f.text(xc, gy1 - hgt - 10, f"{v}/4", size=16, weight="bold", fill=INK, anchor="middle")
        f.text(xc, gy1 + 24, nm, size=13, weight="bold", fill=INK, anchor="middle", mono=True)
        f.text(xc, gy1 + 44, sub, size=12.5, fill=MUTED, anchor="middle")
    f.text(gx0 + step * 2.5, gy0 - 6, "new stage, exact colliders: everything had to be precise",
           size=12.5, fill=MUTED, anchor="middle", italic=True)

    # ---- open decisions
    ox = 1000
    f.card(ox, cy0, 860, 1480 - cy0 - 30, "wd", "Open decisions and loose ends", icon="flag",
           kicker="from docs/NEXT_STEPS.md")
    items = [
        ("decide", "Referee bands: 10–20 mm / 10° against fmb1's own 1 mm / 1°"),
        ("decide", "Keep or drop the placement frames on camera1 for baseline sessions"),
        ("decide", "Layout jitter per part, needed for the 9-seed panel"),
        ("loose", "cuRobo written, never installed: every episode planned with MoveIt"),
        ("loose", "run_python can read host files: filter or containerise Isaac"),
        ("hw", "Hardware: hand-eye calibration, UR driver, RG6 readings, FoundationPose"),
    ]
    colr = {"decide": TH["wd"][0], "loose": INK2, "hw": TH["sim"][0]}
    for i, (k, s) in enumerate(items):
        yy = cy0 + 84 + i * 42
        f.chip(ox + 20, yy, k, "wd" if k == "decide" else "sim", size=11.5, mono=False, h=24)
        f.text(ox + 100, yy + 17, s, size=14, fill=INK)
    return f


def main(argv):
    names = argv or list(FIGS)
    for n in names:
        fig = FIGS[n]()
        p = fig.save()
        print("wrote", p, "and .png")


if __name__ == "__main__":
    main(sys.argv[1:])
