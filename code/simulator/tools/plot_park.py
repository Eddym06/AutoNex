"""python plot_park.py traza.json salida.png  - dibuja la huella del chasis durante PARK_SEARCH/PARKING/DONE (ccw: u hacia +x en el suelo)."""
import json, sys, math
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
rows = [r for r in json.load(open(sys.argv[1])) if r[1] in ('PARK_SEARCH', 'PARKING', 'DONE')]
fig, ax = plt.subplots(figsize=(11, 6))
ax.add_patch(Rectangle((-150, -155), 300, 5, fc='k'))
t0, t1 = rows[0][0], rows[-1][0]
for r in rows[::8]:
    x, z, yaw = r[2], r[3], math.radians(r[4]); s, c = math.sin(yaw), math.cos(yaw)
    P = [(x + u * s + v * c, z + u * c - v * s) for u, v in [(-3.775, -6.5), (22.225, -6.5), (22.225, 6.5), (-3.775, 6.5)]]
    ax.add_patch(Polygon(P, fc='none', ec=plt.cm.viridis((r[0] - t0) / (t1 - t0)), lw=1))
ax.plot([r[2] for r in rows], [r[3] for r in rows], 'r-', lw=.7)
ax.set_xlim(-60, 100); ax.set_ylim(-152, -90); ax.set_aspect('equal'); ax.grid(alpha=.3)
plt.savefig(sys.argv[2], dpi=80, bbox_inches='tight')
