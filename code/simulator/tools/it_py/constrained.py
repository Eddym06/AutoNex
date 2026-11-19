"""Mejor tiempo del modelo si el tramo recto de abajo se obliga a seguir la linea del usuario (d congelado), y el resto se optimiza.
python constrained.py plano.json plan_C.json red_line.json salida.json [--evals 1e6] [--shift cm]
--shift desplaza la linea del usuario (cm, + hacia el exterior) para ver cuanto cuesta/gana separarse mas del borde."""
import sys, json
import numpy as np
sys.path.insert(0, '.')
from it_model import Track, evaluate
from optimize import anneal

a = sys.argv
plan = json.load(open(a[1]))
T = Track(plan['signs'], plan['direction'])
dC = np.array([l['d'] for l in json.load(open(a[2]))])
red = np.array(json.load(open(a[3])))
fl = lambda k, d: a[a.index(k) + 1] if k in a else d
evals = int(float(fl('--evals', 1e6)))
shift = float(fl('--shift', 0))
frozen = [i for i in range(T.n) if abs(T.cz[i] + 100) < 1e-6 and -60 <= T.cx[i] <= 60]
d0 = np.clip(dC, T.lo0, T.hi0)
lo = T.lo0.copy()
hi = T.hi0.copy()
for i in frozen:
    # interseccion de la normal con la polilinea
    best = None
    for k in range(len(red) - 1):
        p, q = red[k], red[k + 1]
        r = q - p
        A = np.array([[T.nx[i], -r[0]], [T.nz[i], -r[1]]])
        if abs(np.linalg.det(A)) < 1e-9:
            continue
        d, s = np.linalg.solve(A, np.array([p[0] - T.cx[i], p[1] - T.cz[i]]))
        if 0 <= s <= 1:
            best = d
    if best is not None:
        best += shift
        d0[i] = lo[i] = hi[i] = np.clip(best, T.lo0[i], T.hi0[i])
print('estaciones congeladas', len(frozen), 'x de', T.cx[frozen[0]], 'a', T.cx[frozen[-1]])
P = T.params()
bd, c = anneal(P, T.cx, T.cz, T.nx, T.nz, lo, hi, T.obs[:, :2].copy(), d0, evals, 0.05, 0.0005, 3)
r = evaluate(T, bd)
print('shift %.0f: t=%.4f viol=%.3f clip=%.2f largo=%.1f' % (shift, r['time'], r['viol'], r['clip'], r['length']))
json.dump(dict(d=bd.tolist(), x=r['x'].tolist(), z=r['z'].tolist(), v=r['v'].tolist(), time=r['time']), open(a[4], 'w'))
