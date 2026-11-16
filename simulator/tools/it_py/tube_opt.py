"""Optimiza la trayectoria obligandola a seguir la linea del usuario dentro de un tubo de +-W cm (en el tramo donde la dibujo) y libre en el resto.
python tube_opt.py plano.json plan_C.json red_line.json salida.json [--tube 4] [--evals 2e6] [--seed 1]
Compara: plan del C, la linea del usuario tal cual (proyectada a las estaciones), y la mejor dentro del tubo."""
import sys, json, math, time
import numpy as np
from numba import njit
sys.path.insert(0, '.')
from it_model import Track, evaluate, cost
from eval_path import resample


@njit(cache=True)
def tube_pen(cx, cz, nx, nz, d, mask, poly, w):
    pen = 0.0
    for i in range(d.shape[0]):
        if not mask[i]:
            continue
        x = cx[i] + d[i] * nx[i]
        z = cz[i] + d[i] * nz[i]
        best = 1e9
        for k in range(poly.shape[0]):
            dd = math.hypot(x - poly[k, 0], z - poly[k, 1])
            if dd < best:
                best = dd
        if best > w:
            pen += (best - w) ** 2
    return pen


@njit(cache=True)
def anneal_tube(P, cx, cz, nx, nz, lo, hi, obs, d0, mask, poly, w, evals, t0, t1, seed, kpen0, kpen1):
    np.random.seed(seed)
    n = d0.shape[0]
    d = d0.copy()
    kpen = kpen0
    cur = cost(P, cx, cz, nx, nz, lo, hi, d, obs) + kpen * tube_pen(cx, cz, nx, nz, d, mask, poly, w)
    best = 1e18
    bd = d.copy()
    trial = d.copy()
    for it in range(evals):
        temp = t0 * (t1 / t0) ** (it / evals)
        knew = kpen0 * (kpen1 / kpen0) ** (it / evals)
        if knew != kpen:
            kpen = knew
            cur = cost(P, cx, cz, nx, nz, lo, hi, d, obs) + kpen * tube_pen(cx, cz, nx, nz, d, mask, poly, w)
        ci = np.random.randint(0, n)
        wd = 2 + int(np.random.random() ** 2 * 20)
        amp = 0.15 * (25.0 ** np.random.random()) * (1.0 if np.random.random() < 0.5 else -1.0)
        for i in range(n):
            trial[i] = d[i]
        for k in range(-wd, wd + 1):
            j = (ci + k) % n
            wt = 0.5 * (1.0 + math.cos(math.pi * k / (wd + 1)))
            v = d[j] + amp * wt
            if v < lo[j]:
                v = lo[j]
            if v > hi[j]:
                v = hi[j]
            trial[j] = v
        c = cost(P, cx, cz, nx, nz, lo, hi, trial, obs) + kpen * tube_pen(cx, cz, nx, nz, trial, mask, poly, w)
        if c < cur or np.random.random() < math.exp(-(c - cur) / temp):
            cur = c
            for i in range(n):
                d[i] = trial[i]
            if it > evals * 0.7 and cur < best:
                best = cur
                for i in range(n):
                    bd[i] = d[i]
    if best > 1e17:
        for i in range(n):
            bd[i] = d[i]
    return bd, best


def main():
    a = sys.argv
    fl = lambda k, dflt: a[a.index(k) + 1] if k in a else dflt
    plan = json.load(open(a[1]))
    T = Track(plan['signs'], plan['direction'])
    L = json.load(open(a[2]))
    dC = np.clip(np.array([l['d'] for l in L]), T.lo0, T.hi0)
    red = np.array(json.load(open(a[3])))
    poly = resample(red, 1.0)
    w = float(fl('--tube', 4))
    evals = int(float(fl('--evals', 2e6)))
    seed = int(fl('--seed', 1))
    n = T.n
    # estaciones del tramo: las que tienen la linea del usuario a menos de 40 cm de su punto central
    ia = int(np.argmin(np.hypot(np.array([T.cx[i] + dC[i] * T.nx[i] for i in range(n)]) - red[0, 0], np.array([T.cz[i] + dC[i] * T.nz[i] for i in range(n)]) - red[0, 1])))
    ib = int(np.argmin(np.hypot(np.array([T.cx[i] + dC[i] * T.nx[i] for i in range(n)]) - red[-1, 0], np.array([T.cz[i] + dC[i] * T.nz[i] for i in range(n)]) - red[-1, 1])))
    mask = np.array([((i - ia) % n) <= ((ib - ia) % n) for i in range(n)])
    # inicio: el punto del C proyectado sobre la linea del usuario (punto mas cercano)
    d0 = dC.copy()
    for i in range(n):
        if False:
            x = T.cx[i] + dC[i] * T.nx[i]
            z = T.cz[i] + dC[i] * T.nz[i]
            k = int(np.argmin(np.hypot(poly[:, 0] - x, poly[:, 1] - z)))
            d0[i] = np.clip((poly[k, 0] - T.cx[i]) * T.nx[i] + (poly[k, 1] - T.cz[i]) * T.nz[i], T.lo0[i], T.hi0[i])
    P = T.params()
    obs = T.obs[:, :2].copy()
    rC = evaluate(T, dC)
    rU = evaluate(T, d0)
    print('estaciones en el tubo: %d de %d' % (mask.sum(), n))
    print('plan del C            : t=%.3f viol=%.3f clip=%.2f largo=%.1f' % (rC['time'], rC['viol'], rC['clip'], rC['length']))
    print('linea del usuario tal cual: t=%.3f viol=%.3f clip=%.2f largo=%.1f vmin=%.0f' % (rU['time'], rU['viol'], rU['clip'], rU['length'], rU['v'].min()))
    t = time.time()
    bd, _ = anneal_tube(P, T.cx, T.cz, T.nx, T.nz, T.lo0, T.hi0, obs, d0, mask, poly, w, evals, 0.05, 0.0004, seed, 0.2, 60.0)
    r = evaluate(T, bd)
    pen = tube_pen(T.cx, T.cz, T.nx, T.nz, bd, mask, poly, w)
    dev = max(np.min(np.hypot(poly[:, 0] - r['x'][i], poly[:, 1] - r['z'][i])) for i in range(n) if mask[i])
    print('mejor dentro del tubo +-%.0f cm: t=%.4f viol=%.3f clip=%.2f largo=%.1f vmin=%.0f desviacion maxima %.1f cm (%.1f s)' % (w, r['time'], r['viol'], r['clip'], r['length'], r['v'].min(), dev, time.time() - t))
    json.dump(dict(d=bd.tolist(), x=r['x'].tolist(), z=r['z'].tolist(), v=r['v'].tolist(), time=r['time']), open(a[4], 'w'))
    json.dump(dict(x=rU['x'].tolist(), z=rU['z'].tolist(), v=rU['v'].tolist(), time=rU['time']), open(a[4].replace('.json', '_user.json'), 'w'))


if __name__ == '__main__':
    main()
