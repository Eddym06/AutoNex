"""IT actual vs optimo, con los ajustes REALES de la mision (grip aprendido, limites del motor de 800 rpm). python compare_now.py plano.json plan_mision.json [--grip 0.5]
plan_mision.json = linea que construyo el controlador en la ronda (tools/user_case.js --plan). Imprime una tabla y escribe los json en $TEMP."""
import sys, json, os
import numpy as np
sys.path.insert(0, '.')
from it_model import Track, evaluate
from optimize import anneal
from optimal_control import solve

a = sys.argv
fl = lambda k, d: a[a.index(k) + 1] if k in a else d
grip = float(fl('--grip', 0.5))
plan = json.load(open(a[1]))
L = json.load(open(a[2]))
kw = dict(grip=grip, vmax=188.7, a_motor=300.0, motor_slope=3.7)
T = Track(plan['signs'], plan['direction'], **kw)
d0 = np.array([(L[i]['x'] - T.cx[i]) * T.nx[i] + (L[i]['z'] - T.cz[i]) * T.nz[i] for i in range(T.n)])
d0 = np.clip(d0, T.lo0, T.hi0)
tmp = os.environ.get('TEMP', '.')
rows = []
e = evaluate(T, d0)
rows.append(('IT de la mision (plan del C, mapa del robot)', e['time'], e['viol'], 'modelo del C'))
P = T.params()
obs = T.obs[:, :2].copy()
bd, _ = anneal(P, T.cx, T.cz, T.nx, T.nz, T.lo0, T.hi0, obs, d0, 1500000, 0.03, 0.0003, 3)
e = evaluate(T, bd)
rows.append(('mejor con las reglas del C (recocido)', e['time'], e['viol'], 'modelo del C'))
plan_pts = [dict(x=float(T.cx[i] + d0[i] * T.nx[i]), z=float(T.cz[i] + d0[i] * T.nz[i]), v=float(e['v'][i]), d=float(d0[i])) for i in range(T.n)]
for margin, label in (('c', 'control optimo, margen del C (14*(v/100)^2) + huella'), ('c0', 'control optimo, huella solo con holgura estatica'),
                      ('meas', 'control optimo, margen MEDIDO (1+5v/100) + huella')):
    r = solve(T, plan_pts, 28.8, 189.0, margin, 188.7, False, True)
    rows.append((label, r['time'], 0.0 if r['ok'] else -1.0, 'continuo' + ('' if r['ok'] else ' (NO convergio)')))
    json.dump(dict(x=r['x'].tolist(), z=r['z'].tolist(), v=r['v'].tolist()), open(os.path.join(tmp, 'now_oc_%s.json' % margin), 'w'))
json.dump(dict(x=[p['x'] for p in plan_pts], z=[p['z'] for p in plan_pts], v=[p['v'] for p in plan_pts]), open(os.path.join(tmp, 'now_plan.json'), 'w'))
print('grip %.2f, vmax 188.7, a_motor 300 con curva del motor (3.7*(vmax-v))' % grip)
for name, t, v, how in rows:
    print('%-58s %.3f s  (%s)' % (name, t, how))
