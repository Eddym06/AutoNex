"""python plot_plan.py linea.json plano.json salida.png [linea2.json]  - dibuja la linea de IT coloreada por velocidad."""
import json, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
plan = json.load(open(sys.argv[2]))
n = 2 if len(sys.argv) > 4 else 1
fig, axs = plt.subplots(1, n, figsize=(9 * n, 9), squeeze=False)
for k in range(n):
    ax = axs[0][k]; line = json.load(open(sys.argv[1] if k == 0 else sys.argv[4]))
    ih = plan["inner"][0]
    ax.add_patch(Rectangle((-ih, -ih), 2 * ih, 2 * ih, fc="#bbb", ec="k"))
    ax.add_patch(Rectangle((-150, -150), 300, 300, fc="none", ec="k"))
    for s in plan["signs"]:
        ax.add_patch(Rectangle((s["x"] - 2.5, s["z"] - 2.5), 5, 5, fc=s["color"], ec="k", zorder=5))
    sc = ax.scatter([l["x"] for l in line], [l["z"] for l in line], c=[l["v"] for l in line], s=8, cmap="turbo", vmin=20, vmax=150, zorder=3)
    ax.set_xlim(-155, 155); ax.set_ylim(-155, 155); ax.set_aspect("equal"); ax.grid(alpha=.3)
plt.colorbar(sc, ax=axs[0][-1], label="cm/s")
plt.savefig(sys.argv[3], dpi=70, bbox_inches="tight")
