"""Dibuja una traza de user_case.js: python plot_trace.py traza.json plano.json salida.png [t0 t1]
Color = velocidad (cm/s); los letreros con su color; el cajon de parqueo y los muros."""
import json, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
rows = json.load(open(sys.argv[1])); plan = json.load(open(sys.argv[2]))
t0 = float(sys.argv[4]) if len(sys.argv) > 4 else 0; t1 = float(sys.argv[5]) if len(sys.argv) > 5 else 1e9
rows = [r for r in rows if t0 <= r[0] <= t1]
fig, ax = plt.subplots(figsize=(9, 9))
h = plan["inner"][0] / 2
ax.add_patch(Rectangle((-h, -plan["inner"][1] / 2), plan["inner"][0], plan["inner"][1], fc="#bbb", ec="k"))
ax.add_patch(Rectangle((-150, -150), 300, 300, fc="none", ec="k"))
for s in plan["signs"]:
    ax.add_patch(Rectangle((s["x"] - 2.5, s["z"] - 2.5), 5, 5, fc=s["color"], ec="k", zorder=5))
sc = ax.scatter([r[2] for r in rows], [r[3] for r in rows], c=[r[5] for r in rows], s=6, cmap="turbo", zorder=3)
plt.colorbar(sc, label="cm/s")
ax.set_xlim(-155, 155); ax.set_ylim(-155, 155); ax.set_aspect("equal"); ax.grid(alpha=.3)
plt.savefig(sys.argv[3], dpi=90, bbox_inches="tight")
