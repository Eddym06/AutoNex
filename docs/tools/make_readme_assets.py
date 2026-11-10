"""Genera los graficos y las animaciones del README a partir de datos REALES del simulador.

    node simulator/tools/export_readme_data.js 5 ccw          # ronda de ejemplo (con y sin IT)
    python docs/tools/make_readme_assets.py                   # escribe docs/assets/*.png y *.svg

Nada esta dibujado a mano: la trayectoria, la velocidad, el mapa y los tiempos salen de la simulacion con el mismo firmware C que va en el robot.
"""
import base64
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.patches import Rectangle, Circle, Polygon

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "docs" / "assets"
OUT.mkdir(parents=True, exist_ok=True)
MED = ROOT / "simulator" / "mediciones"

NAVY, TEAL, ORANGE, RED, GREEN, BLUE, GREY = "#0B1F33", "#12A37F", "#F59E0B", "#E5484D", "#2FB344", "#2D7FF9", "#8A94A0"
plt.rcParams.update({"font.family": "DejaVu Sans", "axes.edgecolor": "#C9D1D9", "axes.labelcolor": NAVY, "text.color": NAVY,
                     "xtick.color": "#57606A", "ytick.color": "#57606A", "axes.spines.top": False, "axes.spines.right": False, "figure.dpi": 100})

data = json.loads((MED / "readme_ronda.json").read_text())
base, it = data["base"], data["it"]


def tr(run):
    a = run["trace"]
    return dict(t=np.array([r[0] for r in a]), x=np.array([r[1] for r in a]), z=np.array([r[2] for r in a]), yaw=np.array([r[3] for r in a]),
                v=np.array([r[4] for r in a]), ph=[r[5] for r in a], lap=np.array([r[6] for r in a]), act=np.array([r[7] for r in a]))


B, I = tr(base), tr(it)
line = np.array(it["line"])  # x, z, v


def draw_field(ax, run, lw=1.0):
    ax.add_patch(Rectangle((-150, -150), 300, 300, fill=False, ec=NAVY, lw=2.2, zorder=1))
    ax.add_patch(Rectangle((-50, -50), 100, 100, fc="#E8EDF2", ec=NAVY, lw=2.2, zorder=1))
    for b in (run["field"]["lot"] or {}).get("limiters", []):
        ax.add_patch(Rectangle((b["x"] - b["w"] / 2, b["z"] - b["d"] / 2), b["w"], b["d"], fc="#C026D3", ec="none", zorder=2))
    for s in run["field"]["signs"]:
        col = RED if s["color"] == "red" else GREEN
        ax.add_patch(Rectangle((s["x"] - 3.5, s["z"] - 3.5), 7, 7, fc=col, ec=NAVY, lw=0.8, zorder=5))
    ax.set_xlim(-158, 158)
    ax.set_ylim(-158, 158)
    ax.set_aspect("equal")
    ax.axis("off")


# --------------------------------------------------------------------------- 1) trayectoria descubierta por IT
fig = plt.figure(figsize=(11.5, 5.2))
gs = fig.add_gridspec(1, 2, width_ratios=[1, 1.25], wspace=0.08)
ax = fig.add_subplot(gs[0])
draw_field(ax, it)
m = (B["lap"] == 2) & np.array([p == "RUN" for p in B["ph"]])
ax.plot(B["x"][m], B["z"][m], color=GREY, lw=1.6, ls=(0, (4, 3)), zorder=3, label="Reactive (~35 cm/s)")
pts = np.vstack([line[:, :2], line[:1, :2]])
segs = np.stack([pts[:-1], pts[1:]], axis=1)
lc = LineCollection(segs, cmap="RdYlGn", norm=Normalize(20, 150), lw=3.4, zorder=4)
lc.set_array(line[:, 2])
ax.add_collection(lc)
ax.plot([], [], color=TEAL, lw=3.4, label="Intelligent Trajectory")
ax.legend(loc="center", frameon=False, fontsize=8.5)
ax.set_title("The line the car discovers by itself", fontsize=11.5, fontweight="bold", color=NAVY)
cb = fig.colorbar(lc, ax=ax, orientation="horizontal", fraction=0.045, pad=0.02, shrink=0.8)
cb.set_label("target speed [cm/s]", fontsize=8.5)
cb.outline.set_visible(False)

ax2 = fig.add_subplot(gs[1])
ph = np.array(I["ph"])
t = I["t"]
ax2.fill_between(t, 0, I["v"], where=(I["act"] == 1), color=TEAL, alpha=0.18, step="mid", lw=0)
ax2.plot(t, I["v"], color=NAVY, lw=1.6)
ax2.plot(B["t"], B["v"], color=GREY, lw=1.2, ls=(0, (4, 3)), label="Reactive only")
t_it = t[I["act"] == 1]
if len(t_it):
    ax2.annotate("IT drives, up to 150 cm/s", xy=(t_it.mean(), 158), ha="center", fontsize=9, color=TEAL, fontweight="bold")
ax2.annotate("Lap 1: builds the map", xy=(2, 44), fontsize=9, color=NAVY)
ax2.annotate("Hand-off + parking", xy=(t[-1] * 0.82, 52), fontsize=9, color=NAVY, ha="center")
ax2.set_xlabel("time [s]")
ax2.set_ylabel("speed [cm/s]")
ax2.set_ylim(-5, 172)
ax2.set_title("Same round, speed over time (simulation)", fontsize=11.5, fontweight="bold", color=NAVY)
ax2.legend(loc="upper left", frameon=False, fontsize=8.5)
ax2.grid(alpha=0.25)
fig.savefig(OUT / "trajectory.png", dpi=170, bbox_inches="tight", facecolor="white")
plt.close(fig)

# --------------------------------------------------------------------------- 2) tiempos (pareado, 24 rondas)
def load(name):
    return {(r["direction"], r["seed"]): r for r in json.loads((MED / name).read_text())["res"]}


a, b, c = load("v35_base_23.json"), load("v35_base_35.json"), load("v35_it_35.json")
keys = [k for k in a if all(d.get(k, {}).get("end") == "finished" for d in (a, b, c))]


def avg(d, f):
    return float(np.mean([f(d[k]) for k in keys]))


labels = ["Reactive\n23 cm/s\n(original)", "Reactive\n35 cm/s", "Reactive 35 cm/s\n+ Intelligent\nTrajectory"]
l1 = [avg(d, lambda r: r["laps"].get("1", 0)) for d in (a, b, c)]
tot = [avg(d, lambda r: r["elapsed"]) for d in (a, b, c)]
l2 = [avg(d, lambda r: r["laps"].get("2", 0)) for d in (a, b, c)]
fig, axs = plt.subplots(1, 2, figsize=(10.5, 4.2), gridspec_kw=dict(wspace=0.28))
cols = [GREY, BLUE, TEAL]
for ax_, vals, title, unit in ((axs[0], l1, "Lap 1 — builds the map", "s"), (axs[1], tot, "Whole round (3 laps + parking)", "s")):
    bars = ax_.bar(labels, vals, color=cols, width=0.62)
    for r_, v_ in zip(bars, vals):
        ax_.text(r_.get_x() + r_.get_width() / 2, v_ + max(vals) * 0.02, f"{v_:.0f} s", ha="center", fontweight="bold", fontsize=10.5)
    ax_.set_title(title, fontsize=11.5, fontweight="bold")
    ax_.set_ylim(0, max(vals) * 1.18)
    ax_.set_ylabel("time [s]")
    ax_.tick_params(axis="x", labelsize=8.5)
tot_gain = 100 * (1 - tot[2] / tot[0])
fig.suptitle(f"Simulation, {len(keys)} paired rounds that finish with full score  ·  round time {tot_gain:.0f}% lower  ·  lap 2 drops to {l2[2]:.0f} s", fontsize=10.5, y=1.02, color="#57606A")
fig.savefig(OUT / "lap_times.png", dpi=170, bbox_inches="tight", facecolor="white")
plt.close(fig)
stats = dict(n=len(keys), lap1=l1, total=tot, lap2=l2)
(OUT / "stats.json").write_text(json.dumps(stats, indent=1))

# --------------------------------------------------------------------------- 3) mapa que construye el robot en la vuelta 1
mp = it["map"]
n = mp["size"]
cells = np.frombuffer(base64.b64decode(mp["cells"]), dtype=np.uint8).reshape(n, n)
occ = (cells & 1) > 0
fig, axs = plt.subplots(1, 2, figsize=(10.5, 5.0), gridspec_kw=dict(wspace=0.04))
o0, cs = mp["originCm"], mp["cellCm"]
ext = [o0, o0 + n * cs, o0, o0 + n * cs]
axs[0].imshow(np.where(occ, 0, 1), cmap="gray", origin="lower", extent=ext, vmin=0, vmax=1.35, interpolation="nearest")
for ob in mp["objects"]:
    if ob["kind"] == 1:
        continue
    colr = {1: RED, 2: GREEN, 3: "#C026D3"}.get(ob["color"], ORANGE)
    axs[0].add_patch(Circle((ob["x"], ob["z"]), 9, fill=False, ec=colr, lw=2.2))
sp = mp["spot"]
if sp["status"] == 2:
    axs[0].add_patch(Rectangle((sp["x"] - sp["length"] / 2, sp["z"] - sp["width"] / 2), sp["length"], sp["width"], fill=False, ec=ORANGE, lw=2, ls="--"))
axs[0].set_xlim(-170, 170)
axs[0].set_ylim(-170, 170)
axs[0].axis("off")
axs[0].set_title("What the car stores after lap 1 (trace_map)", fontsize=11, fontweight="bold")
axs[0].text(0, -178, "black = occupied · grey = free / unknown · circles = detected objects · dashed = parking bay", ha="center", va="top", fontsize=8, color="#57606A")
axs[0].text(0, -178, "black = occupied · grey = free / unknown · circles = detected objects · dashed = parking bay", ha="center", va="top", fontsize=8, color="#57606A")
draw_field(axs[1], it)
for ob in mp["objects"]:
    if ob["kind"] == 1:
        continue
    colr = {1: RED, 2: GREEN, 3: "#C026D3"}.get(ob["color"], ORANGE)
    axs[1].add_patch(Circle((ob["x"], ob["z"]), 9, fill=False, ec=colr, lw=2.2, zorder=6))
axs[1].set_title("Ground truth (simulator) with the objects it found", fontsize=11, fontweight="bold")
fig.savefig(OUT / "map.png", dpi=170, bbox_inches="tight", facecolor="white")
plt.close(fig)


# --------------------------------------------------------------------------- 4) animacion SVG de la ronda completa
def svg_round():
    W, H = 920, 470
    S = 1.2                       # cm -> px
    cx, cz = 215, 262             # centro de la pista en px
    px = lambda x: cx + x * S
    pz = lambda z: cz - z * S
    step = 5                      # 1 muestra de cada 5 (0,25 s de simulacion)
    t, x, z, yaw, v = I["t"][::step], I["x"][::step], I["z"][::step], I["yaw"][::step], I["v"][::step]
    ph_ = np.array(I["ph"])[::step]
    act = I["act"][::step]
    total = float(t[-1])
    speedup = 6.0
    dur = total / speedup
    n_ = len(t)
    # trayectoria de IT coloreada
    segs = []
    pts_ = np.vstack([line[:, :2], line[:1, :2]])
    for i in range(len(pts_) - 1):
        col = cm.RdYlGn(Normalize(20, 150)(line[i, 2]))
        segs.append(f'<line x1="{px(pts_[i,0]):.1f}" y1="{pz(pts_[i,1]):.1f}" x2="{px(pts_[i+1,0]):.1f}" y2="{pz(pts_[i+1,1]):.1f}" stroke="rgb({int(col[0]*255)},{int(col[1]*255)},{int(col[2]*255)})" stroke-width="3.5" stroke-linecap="round" opacity="0.9"/>')
    signs = []
    for s in it["field"]["signs"]:
        c_ = "#E5484D" if s["color"] == "red" else "#2FB344"
        signs.append(f'<rect x="{px(s["x"])-5:.1f}" y="{pz(s["z"])-5:.1f}" width="10" height="10" fill="{c_}" stroke="#0B1F33" stroke-width="1"/>')
    lot = []
    for b in (it["field"]["lot"] or {}).get("limiters", []):
        lot.append(f'<rect x="{px(b["x"]-b["w"]/2):.1f}" y="{pz(b["z"]+b["d"]/2):.1f}" width="{b["w"]*S:.1f}" height="{b["d"]*S:.1f}" fill="#C026D3"/>')
    tx = ";".join(f"{px(a):.1f},{pz(b_):.1f}" for a, b_ in zip(x, z))
    rot = ";".join(f"{a:.1f}" for a in yaw)
    # grafica de velocidad (derecha)
    gx0, gy0, gw, gh = 470, 104, 400, 170
    vmax = 160
    gpx = lambda tt: gx0 + gw * tt / total
    gpy = lambda vv: gy0 + gh - gh * vv / vmax
    vpath = " ".join(("M" if i == 0 else "L") + f"{gpx(a):.1f},{gpy(b_):.1f}" for i, (a, b_) in enumerate(zip(t, v)))
    # franja de IT
    bands = []
    i0 = None
    for i in range(n_):
        if act[i] and i0 is None:
            i0 = i
        if (not act[i] or i == n_ - 1) and i0 is not None:
            bands.append(f'<rect x="{gpx(t[i0]):.1f}" y="{gy0}" width="{gpx(t[i])-gpx(t[i0]):.1f}" height="{gh}" fill="#12A37F" opacity="0.16"/>')
            i0 = None
    # lecturas de velocidad (texto que conmuta)
    labels = []
    for i in range(0, n_, 2):
        k0 = i / (n_ - 1)
        k1 = min(1.0, (i + 2) / (n_ - 1))
        kt = f"0;{k0:.4f};{k1:.4f};1" if k0 > 0 else f"0;{k1:.4f};1;1"
        vals = "hidden;visible;hidden;hidden" if k0 > 0 else "visible;hidden;hidden;hidden"
        if k0 > 0:
            kt = f"0;{k0:.4f};{k1:.4f};1"
        labels.append(f'<text x="{gx0+22}" y="368" class="big" visibility="hidden">{v[i]:.0f}<animate attributeName="visibility" values="{vals}" keyTimes="{kt}" dur="{dur:.2f}s" repeatCount="indefinite" calcMode="discrete"/></text>')
    ph_names = {"PARK_EXIT": "Leaving the bay", "RUN": "Driving", "PARK_SEARCH": "Looking for the bay", "PARKING": "Parking", "FINISH": "Finish", "DONE": "Done", "UTURN": "U-turn"}
    return f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="DejaVu Sans, Verdana, sans-serif">
<style>
  .t {{ font-size: 15px; font-weight: 700; fill: #0B1F33; }}
  .s {{ font-size: 11px; fill: #57606A; }}
  .big {{ font-size: 46px; font-weight: 800; fill: #0B1F33; }}
  .u {{ font-size: 13px; fill: #57606A; }}
</style>
<rect width="{W}" height="{H}" rx="14" fill="#FFFFFF" stroke="#D0D7DE"/>
<text x="24" y="34" class="t">One full round — simulated, {speedup:.0f}× speed</text>
<text x="24" y="54" class="s">Same firmware as the robot · lap 1 maps the field at 35 cm/s · laps 2–3 follow the line the car computed · then it parks</text>
<rect x="{px(-150)}" y="{pz(150)}" width="{300*S:.1f}" height="{300*S:.1f}" fill="#F6F8FA" stroke="#0B1F33" stroke-width="3"/>
<rect x="{px(-50)}" y="{pz(50)}" width="{100*S:.1f}" height="{100*S:.1f}" fill="#E3E8EE" stroke="#0B1F33" stroke-width="3"/>
{''.join(lot)}
{''.join(segs)}
{''.join(signs)}
<g>
  <g>
    <g transform="translate(0,0)">
      <rect x="{-6.5*S:.1f}" y="{-22.2*S:.1f}" width="{13*S:.1f}" height="{26*S:.1f}" rx="3" fill="#0B1F33" stroke="#FFFFFF" stroke-width="1.2"/>
      <rect x="{-4.5*S:.1f}" y="{-18*S:.1f}" width="{9*S:.1f}" height="{5*S:.1f}" rx="1.5" fill="#7DD3FC"/>
    </g>
    <animateTransform attributeName="transform" type="rotate" values="{rot}" keyTimes="{';'.join(f'{i/(n_-1):.4f}' for i in range(n_))}" dur="{dur:.2f}s" repeatCount="indefinite" calcMode="linear"/>
  </g>
  <animateTransform attributeName="transform" type="translate" values="{tx}" keyTimes="{';'.join(f'{i/(n_-1):.4f}' for i in range(n_))}" dur="{dur:.2f}s" repeatCount="indefinite" calcMode="linear"/>
</g>
<text x="{gx0}" y="84" class="t">Speed [cm/s]</text>
<text x="{gx0+112}" y="84" class="s">green band = Intelligent Trajectory is driving</text>
<rect x="{gx0}" y="{gy0}" width="{gw}" height="{gh}" fill="#FFFFFF" stroke="#D0D7DE"/>
{''.join(bands)}
<line x1="{gx0}" x2="{gx0+gw}" y1="{gpy(35):.1f}" y2="{gpy(35):.1f}" stroke="#D0D7DE" stroke-dasharray="4 4"/>
<text x="{gx0+gw+4}" y="{gpy(35)+4:.1f}" class="s">35</text>
<line x1="{gx0}" x2="{gx0+gw}" y1="{gpy(150):.1f}" y2="{gpy(150):.1f}" stroke="#D0D7DE" stroke-dasharray="4 4"/>
<text x="{gx0+gw+4}" y="{gpy(150)+4:.1f}" class="s">150</text>
<path d="{vpath}" fill="none" stroke="#0B1F33" stroke-width="1.8"/>
<line x1="{gx0}" x2="{gx0}" y1="{gy0}" y2="{gy0+gh}" stroke="#E5484D" stroke-width="2">
  <animate attributeName="x1" values="{gx0};{gx0+gw}" dur="{dur:.2f}s" repeatCount="indefinite"/>
  <animate attributeName="x2" values="{gx0};{gx0+gw}" dur="{dur:.2f}s" repeatCount="indefinite"/>
</line>
<text x="{gx0}" y="{gy0+gh+18}" class="s">0 s</text>
<text x="{gx0+gw}" y="{gy0+gh+18}" class="s" text-anchor="end">{total:.0f} s</text>
<text x="{gx0}" y="{gy0+gh+40}" class="s">Lap 1 · mapping</text>
<text x="{gx0+gw}" y="{gy0+gh+40}" class="s" text-anchor="end">parking</text>
<rect x="{gx0}" y="318" width="{gw}" height="116" rx="10" fill="#F6F8FA"/>
{''.join(labels)}
<text x="{gx0+112}" y="368" class="u">cm/s</text>
<text x="{gx0+170}" y="342" class="s">Lap 1 is slow on purpose:</text>
<text x="{gx0+170}" y="360" class="s">it builds the map of the field.</text>
<text x="{gx0+170}" y="384" class="s">From lap 2 it trusts that map and</text><text x="{gx0+170}" y="402" class="s">brakes only where a pillar or wall</text><text x="{gx0+170}" y="420" class="s">leaves no margin.</text>
<text x="24" y="{H-16}" class="s">Generated by docs/tools/make_readme_assets.py from simulator data · red pillar → pass on the right · green → left · magenta = parking bay limiters</text>
</svg>"""


(OUT / "round.svg").write_text(svg_round(), encoding="utf-8")
print("ok", sorted(p.name for p in OUT.iterdir()), stats)
