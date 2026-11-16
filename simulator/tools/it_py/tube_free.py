"""Trayectoria LIBRE (polilinea de puntos cada ~5 cm) = plan del C con el tramo del usuario empalmado, mas un desplazamiento lateral e_i
acotado a +-W cm en el tramo del usuario y +-F cm en el resto. Se optimiza e con recocido y el mismo modelo de coste del C.
python tube_free.py plano.json plan_C.json red_line.json salida.json [--tube 4] [--free 15] [--evals 2e6] [--seed 1]"""
import sys, json, math, time
import numpy as np
from numba import njit
sys.path.insert(0, '.')
from it_model import Track, profile
from eval_path import resample, smooth, splice


@njit(cache=True)
def cost_free(P, bx, bz, bnx, bnz, e, st, cx, cz, nx, nz, lo0, hi0, obs):
    n = e.shape[0]
    x = np.empty(n)
    z = np.empty(n)
    d = np.empty(n)
    lo = np.empty(n)
    hi = np.empty(n)
    for i in range(n):
        x[i] = bx[i] + e[i] * bnx[i]
        z[i] = bz[i] + e[i] * bnz[i]
        s = st[i]
        d[i] = (x[i] - cx[s]) * nx[s] + (z[i] - cz[s]) * nz[s]
        lo[i] = lo0[s]
        hi[i] = hi0[s]
    t, viol, clip, v, gap, ks, room, h = profile(P, x, z, lo, hi, d, obs)
    return t + 2000.0 * viol + 4000.0 * clip


@njit(cache=True)
def anneal_free(P, bx, bz, bnx, bnz, e0, elo, ehi, st, cx, cz, nx, nz, lo0, hi0, obs, evals, t0, t1, seed):
    np.random.seed(seed)
    n = e0.shape[0]
    e = e0.copy()
    cur = cost_free(P, bx, bz, bnx, bnz, e, st, cx, cz, nx, nz, lo0, hi0, obs)
    best = cur
    be = e.copy()
    trial = e.copy()
    for it in range(evals):
        temp = t0 * (t1 / t0) ** (it / evals)
        ci = np.random.randint(0, n)
        wd = 2 + int(np.random.random() ** 2 * 24)
        amp = 0.15 * (25.0 ** np.random.random()) * (1.0 if np.random.random() < 0.5 else -1.0)
        for i in range(n):
            trial[i] = e[i]
        for k in range(-wd, wd + 1):
            j = (ci + k) % n
            wt = 0.5 * (1.0 + math.cos(math.pi * k / (wd + 1)))
            v = e[j] + amp * wt
            if v < elo[j]:
                v = elo[j]
            if v > ehi[j]:
                v = ehi[j]
            trial[j] = v
        c = cost_free(P, bx, bz, bnx, bnz, trial, st, cx, cz, nx, nz, lo0, hi0, obs)
        if c < cur or np.random.random() < math.exp(-(c - cur) / temp):
            cur = c
            for i in range(n):
                e[i] = trial[i]
            if cur < best:
                best = cur
                for i in range(n):
                    be[i] = e[i]
    return be, best


def main():
    a = sys.argv
    fl = lambda k, dflt: a[a.index(k) + 1] if k in a else dflt
    plan = json.load(open(a[1]))
    T = Track(plan['signs'], plan['direction'])
    C = np.array([[l['x'], l['z']] for l in json.load(open(a[2]))])
    red = np.array(json.load(open(a[3])))
    W = float(fl('--tube', 4))
    F = float(fl('--free', 15))
    evals = int(float(fl('--evals', 2e6)))
    seed = int(fl('--seed', 1))
    m = len(resample(red, 5.0))
    B, ia, ib = splice(T, C, red, red[0], red[-1])
    B = smooth(B, 2)
    n = len(B)
    # normales a la derecha del avance
    t = np.roll(B, -1, axis=0) - np.roll(B, 1, axis=0)
    t /= np.hypot(t[:, 0], t[:, 1])[:, None]
    bnx, bnz = t[:, 1].copy(), -t[:, 0].copy()
    st = np.array([int(np.argmin(np.hypot(T.cx - B[i, 0], T.cz - B[i, 1]))) for i in range(n)], dtype=np.int64)
    elo = np.full(n, -F)
    ehi = np.full(n, F)
    inner = slice(4, m - 4)      # el tramo del usuario, sin los extremos (donde se empalma)
    elo[inner] = -W
    ehi[inner] = W
    P = T.params()
    obs = T.obs[:, :2].copy()
    bx = np.ascontiguousarray(B[:, 0])
    bz = np.ascontiguousarray(B[:, 1])
    e0 = np.zeros(n)
    c0 = cost_free(P, bx, bz, bnx, bnz, e0, st, T.cx, T.cz, T.nx, T.nz, T.lo0, T.hi0, obs)
    print('empalme: %d puntos (%d del usuario); coste inicial %.3f (el plan del C es 7.810)' % (n, m, c0))
    t0 = time.time()
    be, _ = anneal_free(P, bx, bz, bnx, bnz, e0, elo, ehi, st, T.cx, T.cz, T.nx, T.nz, T.lo0, T.hi0, obs, evals, 0.05, 0.0004, seed)
    x = bx + be * bnx
    z = bz + be * bnz
    d = (x - T.cx[st]) * T.nx[st] + (z - T.cz[st]) * T.nz[st]
    tt, viol, clip, v, gap, ks, room, h = profile(P, x, z, T.lo0[st], T.hi0[st], d, obs)
    # desviacion real respecto a la linea del usuario
    poly = resample(red, 1.0)
    dev = max(np.min(np.hypot(poly[:, 0] - x[i], poly[:, 1] - z[i])) for i in range(4, m - 4))
    print('tubo +-%.0f cm: t=%.4f viol=%.3f clip=%.2f largo=%.1f vmin=%.0f desviacion maxima %.1f cm (%.0f s)' % (W, tt, viol, clip, h.sum(), v.min(), dev, time.time() - t0))
    json.dump(dict(x=x.tolist(), z=z.tolist(), v=v.tolist(), time=tt), open(a[4], 'w'))


if __name__ == '__main__':
    main()
