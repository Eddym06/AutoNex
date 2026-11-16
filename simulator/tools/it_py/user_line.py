"""Evalua en el modelo la linea roja dibujada a mano por el usuario (seccion tras el pasillo del verde), empalmada al plan del C.
python user_line.py plano.json plan_C.json salida.png"""
import sys, json
import numpy as np
sys.path.insert(0, '.')
from it_model import Track, evaluate
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon

plan = json.load(open(sys.argv[1]))
T = Track(plan['signs'], plan['direction'])
L = json.load(open(sys.argv[2]))
dC = np.array([l['d'] for l in L])

# Linea roja leida de la imagen del usuario (500 px = 300 cm; centro (250,250)); (x, z) en cm.
red = np.array(json.load(open(sys.argv[4]))) if len(sys.argv) > 4 else None


def project(i):
    """d sobre la normal de la estacion i donde corta la polilinea roja (la interseccion mas cercana al plan C), o None."""
    best = None
    for k in range(len(red) - 1):
        p, q = red[k], red[k + 1]
        r = q - p
        # cx + d*nx = p + s*r
        A = np.array([[T.nx[i], -r[0]], [T.nz[i], -r[1]]])
        if abs(np.linalg.det(A)) < 1e-9:
            continue
        d, s = np.linalg.solve(A, np.array([p[0] - T.cx[i], p[1] - T.cz[i]]))
        if 0 <= s <= 1 and (best is None or abs(d - dC[i]) < abs(best - dC[i])):
            best = d
    return best


d = dC.copy()
rng = []
for i in range(T.n):
    dd = project(i)
    if dd is not None and abs(dd - dC[i]) < 45:
        rng.append(i)
        d[i] = dd
print('estaciones sustituidas', rng[0], '-', rng[-1], 'n', len(rng))
for name, dv in (('plan C', dC), ('C + linea roja del usuario', d)):
    r = evaluate(T, dv)
    print('%s: t=%.3f viol=%.3f clip=%.2f largo=%.1f' % (name, r['time'], r['viol'], r['clip'], r['length']))
    if name != 'plan C':
        r2 = r
for i in rng:
    print(i, 'x=%.0f z=%.0f d=%.1f (corredor %.1f..%.1f) gap=%.1f room=%.1f v=%.0f k=%.3f' % (r2['x'][i], r2['z'][i], d[i], T.lo0[i], T.hi0[i], r2['gap'][i], r2['room'][i], r2['v'][i], r2['kappa'][i] * 100))
