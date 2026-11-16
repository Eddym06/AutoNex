"""Optimiza d (desplazamiento lateral por estacion) con recocido simulado de 'jorobas' en numba, con el MISMO modelo de coste que el C.
python optimize.py plano.json plan_C.json salida.json [--evals 3000000] [--start c|center|short] [--seed 1]"""
import sys, json, time, math
import numpy as np
from numba import njit
sys.path.insert(0, '.')
from it_model import Track, evaluate, cost


@njit(cache=True)
def anneal(P, cx, cz, nx, nz, lo, hi, obs, d0, evals, t0, t1, seed):
    np.random.seed(seed)
    n = d0.shape[0]
    d = d0.copy()
    cur = cost(P, cx, cz, nx, nz, lo, hi, d, obs)
    best = cur
    bd = d.copy()
    trial = d.copy()
    for it in range(evals):
        temp = t0 * (t1 / t0) ** (it / evals)
        ci = np.random.randint(0, n)
        w = 2 + int(np.random.random() ** 2 * 30)
        amp = 0.2 * (30.0 ** np.random.random()) * (1.0 if np.random.random() < 0.5 else -1.0)
        for i in range(n):
            trial[i] = d[i]
        for k in range(-w, w + 1):
            j = (ci + k) % n
            wt = 0.5 * (1.0 + math.cos(math.pi * k / (w + 1)))
            v = d[j] + amp * wt
            if v < lo[j]:
                v = lo[j]
            if v > hi[j]:
                v = hi[j]
            trial[j] = v
        c = cost(P, cx, cz, nx, nz, lo, hi, trial, obs)
        if c < cur or np.random.random() < math.exp(-(c - cur) / temp):
            cur = c
            for i in range(n):
                d[i] = trial[i]
            if cur < best:
                best = cur
                for i in range(n):
                    bd[i] = d[i]
    return bd, best


def main():
    a = sys.argv
    plan = json.load(open(a[1]))
    kw = {k: float(v) for k, v in (x.split('=') for x in a if '=' in x and x.split('=')[0] in __import__('it_model').DEF)}
    T = Track(plan['signs'], plan['direction'], **kw)
    fl = lambda k, dflt: a[a.index(k) + 1] if k in a else dflt
    evals = int(float(fl('--evals', 3e6)))
    start = fl('--start', 'c')
    seed = int(fl('--seed', 1))
    P = T.params()
    obs = T.obs[:, :2].copy()
    if start == 'c':
        d0 = np.clip(np.array([l['d'] for l in json.load(open(a[2]))]), T.lo0, T.hi0)
    elif start == 'center':
        d0 = np.clip(np.zeros(T.n), T.lo0, T.hi0)
    else:  # linea corta: pegada al borde interior (izquierda) del corredor
        d0 = np.clip(T.lo0 + 8.0, T.lo0, T.hi0)
    r0 = evaluate(T, d0)
    print('inicio %s: t=%.3f viol=%.3f clip=%.2f' % (start, r0['time'], r0['viol'], r0['clip']))
    t = time.time()
    best_d, best = anneal(P, T.cx, T.cz, T.nx, T.nz, T.lo0, T.hi0, obs, d0, evals, 0.05, 0.0005, seed)
    r = evaluate(T, best_d)
    print('final: t=%.4f viol=%.3f clip=%.2f largo=%.1f vmin=%.0f (%.1f s, %d evals)' % (r['time'], r['viol'], r['clip'], r['length'], r['v'].min(), time.time() - t, evals))
    out = a[3] if len(a) > 3 and not a[3].startswith('--') else None
    if out:
        json.dump(dict(d=best_d.tolist(), x=r['x'].tolist(), z=r['z'].tolist(), v=r['v'].tolist(), time=r['time']), open(out, 'w'))


if __name__ == '__main__':
    main()
