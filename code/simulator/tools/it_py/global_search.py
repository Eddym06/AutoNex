"""Busqueda global: d(s) como spline periodica de K nudos (suave por construccion) optimizada con evolucion diferencial varias veces;
despues pulido con recocido sobre las d completas. python global_search.py plano.json salida.json [--knots 30] [--runs 6] [--polish 1e6]"""
import sys, json, time
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import differential_evolution
sys.path.insert(0, '.')
from it_model import Track, evaluate, cost
from optimize import anneal


def main():
    a = sys.argv
    plan = json.load(open(a[1]))
    T = Track(plan['signs'], plan['direction'])
    fl = lambda k, d: a[a.index(k) + 1] if k in a else d
    K = int(fl('--knots', 30))
    runs = int(fl('--runs', 6))
    polish = int(float(fl('--polish', 1e6)))
    P = T.params()
    obs = T.obs[:, :2].copy()
    n = T.n
    kx = np.linspace(0, n, K + 1)
    idx = np.arange(n)
    # cotas de cada nudo: la interseccion del corredor en las estaciones vecinas
    lo_k = np.array([T.lo0[int(round(k)) % n] for k in kx[:-1]])
    hi_k = np.array([T.hi0[int(round(k)) % n] for k in kx[:-1]])

    def to_d(z):
        zz = np.append(z, z[0])
        return np.clip(CubicSpline(kx, zz, bc_type='periodic')(idx), T.lo0, T.hi0)

    def f(z):
        return cost(P, T.cx, T.cz, T.nx, T.nz, T.lo0, T.hi0, to_d(z), obs)

    bounds = list(zip(lo_k, hi_k))
    best = (1e18, None)
    t = time.time()
    for r in range(runs):
        res = differential_evolution(f, bounds, popsize=25, maxiter=500, tol=1e-9, mutation=(0.4, 1.0), recombination=0.8, seed=100 + r, polish=False, init='sobol')
        d = to_d(res.x)
        e = evaluate(T, d)
        print('DE %d: t=%.3f viol=%.3f clip=%.2f largo=%.1f (%.0f s)' % (r, e['time'], e['viol'], e['clip'], e['length'], time.time() - t), flush=True)
        if res.fun < best[0]:
            best = (res.fun, d)
    d, c = anneal(P, T.cx, T.cz, T.nx, T.nz, T.lo0, T.hi0, obs, best[1], polish, 0.02, 0.0003, 7)
    e = evaluate(T, d)
    print('pulido: t=%.4f viol=%.3f clip=%.2f largo=%.1f vmin=%.0f' % (e['time'], e['viol'], e['clip'], e['length'], e['v'].min()))
    json.dump(dict(d=d.tolist(), x=e['x'].tolist(), z=e['z'].tolist(), v=e['v'].tolist(), time=e['time']), open(a[2], 'w'))


if __name__ == '__main__':
    main()
