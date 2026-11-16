"""python plot_lines.py plano.json salida.png etiqueta1=a.json etiqueta2=b.json ...  (cada json con x, z, v)  - lineas coloreadas por velocidad,
con la linea roja del usuario (leida de su imagen) en trazo grueso translucido."""
import sys, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

plan = json.load(open(sys.argv[1]))
items = [a.split('=', 1) for a in sys.argv[3:]]
n = len(items)
fig, axs = plt.subplots(1, n, figsize=(7.5 * n, 7.8), squeeze=False)
import os
red = np.array(json.load(open(os.environ.get('RED_LINE', 'red_line.json'))))
for ax, (name, path) in zip(axs[0], items):
    L = json.load(open(path))
    if isinstance(L, list):
        L = dict(x=[l['x'] for l in L], z=[l['z'] for l in L], v=[l['v'] for l in L])
    ih = plan['inner'][0]
    ax.add_patch(Rectangle((-ih, -ih), 2 * ih, 2 * ih, fc='#bbb', ec='k'))
    ax.add_patch(Rectangle((-150, -150), 300, 300, fc='none', ec='k'))
    for s in plan['signs']:
        ax.add_patch(Rectangle((s['x'] - 2.5, s['z'] - 2.5), 5, 5, fc=s['color'], ec='k', zorder=6))
    ax.plot(red[:, 0], red[:, 1], '-', color='red', lw=6, alpha=0.25, zorder=2, label='linea del usuario')
    sc = ax.scatter(L['x'], L['z'], c=L['v'], s=9, cmap='turbo', vmin=20, vmax=150, zorder=4)
    ax.set_xlim(-155, 155)
    ax.set_ylim(-155, 155)
    ax.set_aspect('equal')
    ax.grid(alpha=.3)
    ax.set_title(name)
plt.colorbar(sc, ax=axs[0][-1], label='cm/s', shrink=0.8)
plt.savefig(sys.argv[2], dpi=70, bbox_inches='tight')
