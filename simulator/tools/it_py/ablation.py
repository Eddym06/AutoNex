"""Por que el evaluador del C da mas tiempo que el control optimo para la MISMA trayectoria: ablacion regla por regla.
Reimplementa profile() de it_plan.c con interruptores y evalua la trayectoria del control optimo (oc_c.json: mismo margen que el C, sin huella).
python ablation.py plano.json oc.json"""
import sys, json, math
import numpy as np
sys.path.insert(0, '.')
from it_model import Track, footprint_gap, G


def prof(T, d, smooth_k=True, raw08=True, rate=True, foot=True, vcap=True, curv_pass=True, steer_use=None, obs_half=None, vmin_floor=True):
    c = dict(T.c)
    if steer_use is not None: c['steer_use'] = steer_use
    P = T.params().copy()
    if obs_half is not None: P[5] = obs_half
    x = T.cx + d * T.nx
    z = T.cz + d * T.nz
    n = len(x)
    nx = lambda a: np.roll(a, -1)
    pv = lambda a: np.roll(a, 1)
    h = np.maximum(np.hypot(nx(x) - x, nx(z) - z), 0.01)
    ux, uz, vx, nz_ = x - pv(x), z - pv(z), nx(x) - x, nx(z) - z
    wx, wz = nx(x) - pv(x), nx(z) - pv(z)
    la, lb, lc = np.hypot(ux, uz), np.hypot(vx, nz_), np.hypot(wx, wz)
    cross = ux * nz_ - uz * vx
    ks = -2 * cross / (la * lb * lc)
    hdg = np.arctan2(wx, wz)
    gap = np.array([footprint_gap(P, x[i], z[i], hdg[i], T.obs[:, :2].copy()) for i in range(n)]) - c['wall_m']
    kt = 0.25 * pv(ks) + 0.5 * ks + 0.25 * nx(ks) if smooth_k else ks.copy()
    if smooth_k and raw08:
        raw = 0.8 * np.abs(ks)
        kt = np.where(np.abs(kt) >= raw, kt, np.where(kt < 0, -raw, raw))
    kap = np.abs(kt)
    mu_e = c['mu'] * c['grip']
    a_lim = mu_e * G * c['safety']
    a_full = mu_e * G
    hL = c['cg'] / c['wb']
    cap_acc = min(c['a_motor'], mu_e * G * c['rear_w'] / (1 - mu_e * hL))
    cap_dec = mu_e * G * c['rear_w'] / (1 + mu_e * hL)
    rate_v = c['rate_use'] * c['steer_rate'] * math.pi / 180
    dl = np.arctan(c['wb'] * kt)
    v = np.full(n, c['vmax'])
    for i in range(n):
        a_, b_ = (i - 1) % n, (i + 1) % n
        k = kap[i]
        if k > 1e-4: v[i] = min(v[i], math.sqrt(a_lim / k))
        dds = abs(dl[b_] - dl[a_]) / (h[a_] + h[i])
        if rate and dds > 1e-5: v[i] = min(v[i], rate_v / dds)
        room = min(d[i] - T.lo0[i], T.hi0[i] - d[i])
        allowed = min(room, gap[i] if foot else 1e9) - 0.3
        if vcap:
            vc = c['vmin'] if allowed <= c['pose_err'] else 100 * math.sqrt((allowed - c['pose_err']) / c['track_err'])
            v[i] = min(v[i], vc)
        if vmin_floor: v[i] = max(v[i], c['vmin'])
    for _ in range(2):
        for i in range(n):
            j = (i + 1) % n
            ay = v[i] ** 2 * kap[i]
            ar = math.sqrt(max(0, a_full ** 2 - ay ** 2))
            v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2 * min(cap_acc, ar) * h[i]))
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            ay = v[j] ** 2 * kap[i]
            ar = math.sqrt(max(0, a_full ** 2 - ay ** 2))
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2 * min(cap_dec, ar) * h[i]))
    t = sum(h[i] / (0.5 * (v[i] + v[(i + 1) % n])) for i in range(n))
    return t, v, kap


def smooth_d(d, k=1):
    for _ in range(k):
        d = 0.25 * np.roll(d, 1) + 0.5 * d + 0.25 * np.roll(d, -1)
    return d


if __name__ == '__main__':
    plan = json.load(open(sys.argv[1]))
    T = Track(plan['signs'], plan['direction'])
    r = json.load(open(sys.argv[2]))
    d = np.array(r['d'])
    print('control optimo dice: %.3f s (misma geometria)' % r['time'])
    base, _, _ = prof(T, d)
    print('evaluador del C (replica): %.3f' % base)
    cases = [('sin suavizado de curvatura [1 2 1]/4 ni piso 0,8', dict(smooth_k=False)),
             ('solo sin el piso 0,8 de la curvatura cruda', dict(raw08=False)),
             ('sin limite de velocidad del servo (rate_use)', dict(rate=False)),
             ('sin huella (solo pasillo de punto)', dict(foot=False)),
             ('sin tope por holgura (vcap)', dict(vcap=False)),
             ('radio de letrero 2,5 en vez de 3,5', dict(obs_half=2.5)),
             ('sin suavizado + sin servo', dict(smooth_k=False, rate=False)),
             ('sin suavizado + sin servo + sin huella', dict(smooth_k=False, rate=False, foot=False)),
             ('todo apagado salvo friccion y aceleracion', dict(smooth_k=False, rate=False, foot=False, vcap=False))]
    for name, kw in cases:
        t, v, kap = prof(T, d, **kw)
        print('%-52s %.3f s  (%+.2f)' % (name, t, t - base))
    for k in (1, 2, 3):
        t, v, kap = prof(T, smooth_d(d, k))
        print('d alisado %d veces (quita el ruido de la trayectoria)  %.3f s' % (k, t))
