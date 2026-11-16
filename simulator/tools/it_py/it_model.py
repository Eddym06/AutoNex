"""Replica en Python (numba) del modelo de IT (LIDAR System/main/it_plan.c): corredor, huella, perfil de velocidad y tiempo de vuelta.
Sirve para investigar la optimizacion fuera del firmware. Mismas constantes y mismas formulas que el C."""
import math, json
import numpy as np
from numba import njit

G = 981.0
DEF = dict(outer=150.0, inner=50.0, rc=40.0, wb=18.45, hw=6.5, front=22.225, rear=3.775, steer_max=40.0, steer_rate=420.0,
           steer_use=0.72, rate_use=0.45, mu=0.8, rear_w=0.62, cg=4.0, grip=0.7, safety=0.85, a_motor=350.0, vmax=150.0, vmin=15.0,
           wall_m=1.0, obs_m=1.0, obs_half=3.5, pose_err=0.8, track_err=14.0, curtain=100.0, side_red=1, side_green=-1, station=5.0)


def cfg(**kw):
    c = dict(DEF)
    c.update(kw)
    return c


def center_at(c, s):
    h = 0.5 * (c['outer'] + c['inner'])
    rc = c['rc']
    ln = 2 * (h - rc)
    al = 0.5 * math.pi * rc
    period = 4 * (ln + al)
    s = s % period
    piece = min(int(s / (ln + al)), 3)
    t = s - piece * (ln + al)
    DX = [1, 0, -1, 0]
    DZ = [0, 1, 0, -1]
    sx = [-(h - rc), h, h - rc, -h]
    sz = [-h, -(h - rc), h, h - rc]
    ax = [h - rc, h - rc, -(h - rc), -(h - rc)]
    az = [-(h - rc), h - rc, h - rc, -(h - rc)]
    if t <= ln:
        return sx[piece] + DX[piece] * t, sz[piece] + DZ[piece] * t, DX[piece], DZ[piece]
    t -= ln
    th = (-0.5 + piece * 0.5) * math.pi + t / rc
    return ax[piece] + rc * math.cos(th), az[piece] + rc * math.sin(th), -math.sin(th), math.cos(th)


class Track:
    """Estaciones, normales, obstaculos y corredor libre (lo0, hi0) como en it_plan_begin/compute_corridor."""

    def __init__(self, signs, direction, **kw):
        self.c = c = cfg(**kw)
        self.direction = 1 if direction in (1, 'cw') else -1
        h = 0.5 * (c['outer'] + c['inner'])
        rc = c['rc']
        period = 4 * (2 * (h - rc) + 0.5 * math.pi * rc)
        n = min(256, max(16, int(period / c['station'] + 0.5)))
        self.n = n
        step = period / n
        self.cx = np.zeros(n)
        self.cz = np.zeros(n)
        self.nx = np.zeros(n)
        self.nz = np.zeros(n)
        for i in range(n):
            sp = i * step if self.direction < 0 else period - i * step
            x, z, tx, tz = center_at(c, sp)
            if self.direction > 0:
                tx, tz = -tx, -tz
            self.cx[i], self.cz[i], self.nx[i], self.nz[i] = x, z, tz, -tx
        self.obs = np.array([[s['x'], s['z'], 1 if s['color'] == 'red' else 2] for s in signs], float).reshape(-1, 3)
        self.orx = np.zeros(len(self.obs))
        self.orz = np.zeros(len(self.obs))
        for j, o in enumerate(self.obs):
            b = np.argmin(np.hypot(self.cx - o[0], self.cz - o[1]))
            self.orx[j], self.orz[j] = self.nx[b], self.nz[b]
        self.lo0, self.hi0 = self._corridor()

    def free(self, x, z):
        c = self.c
        rho = max(abs(x), abs(z))
        m = c['hw'] + c['wall_m']
        if rho > c['outer'] - m or rho < c['inner'] + m:
            return False
        rs = c['obs_half'] + c['hw'] + c['obs_m']
        for j, o in enumerate(self.obs):
            if math.hypot(x - o[0], z - o[1]) < rs:
                return False
            side = c['side_red'] if o[2] == 1 else c['side_green']
            if side == 0:
                continue
            rx, rz = self.orx[j], self.orz[j]
            fx, fz, tx, tz = -side * rx, -side * rz, -rz, rx
            px, pz = x - o[0], z - o[1]
            lam = px * fx + pz * fz
            ell = px * tx + pz * tz
            if 0 <= lam <= c['curtain'] and -rs <= ell <= rs:
                return False
        return True

    def _corridor(self):
        c = self.c
        n = self.n
        reach = 0.5 * (c['outer'] - c['inner']) + 5
        ivs = []
        for i in range(n):
            iv = []
            run = None
            last = 0
            d = -reach
            while d <= reach + 0.01:
                f = self.free(self.cx[i] + d * self.nx[i], self.cz[i] + d * self.nz[i])
                if f and run is None:
                    run = d
                if not f and run is not None:
                    iv.append((run, last))
                    run = None
                last = d
                d += 1.0
            if run is not None:
                iv.append((run, last))
            if not iv:
                raise ValueError('sin paso en la estacion %d' % i)
            ivs.append(iv)

        def mid(i, k):
            m = 0.5 * (ivs[i][k][0] + ivs[i][k][1])
            return self.cx[i] + m * self.nx[i], self.cz[i] + m * self.nz[i]

        best = (1e18, None)
        for s0 in range(len(ivs[0])):
            cost = [{s0: 0.0}]
            frm = [{}]
            for i in range(1, n):
                ci = {}
                fi = {}
                for k in range(len(ivs[i])):
                    mx, mz = mid(i, k)
                    for q, cq in cost[i - 1].items():
                        qx, qz = mid(i - 1, q)
                        v = cq + math.hypot(mx - qx, mz - qz)
                        if k not in ci or v < ci[k]:
                            ci[k] = v
                            fi[k] = q
                cost.append(ci)
                frm.append(fi)
            for k, ck in cost[n - 1].items():
                qx, qz = mid(n - 1, k)
                mx, mz = mid(0, s0)
                tot = ck + math.hypot(mx - qx, mz - qz)
                if tot < best[0]:
                    pick = k
                    path = [0] * n
                    for i in range(n - 1, -1, -1):
                        path[i] = pick
                        pick = frm[i].get(pick, 0)
                    best = (tot, path)
        if best[1] is None:
            raise ValueError('sin corredor')
        lo = np.array([ivs[i][best[1][i]][0] for i in range(n)])
        hi = np.array([ivs[i][best[1][i]][1] for i in range(n)])
        return lo, hi

    def params(self):
        c = self.c
        return np.array([c['hw'], c['front'], c['rear'], c['outer'], c['inner'], c['obs_half'], c['wall_m'], c['wb'], c['steer_use'], c['steer_max'],
                         c['steer_rate'], c['rate_use'], c['mu'], c['grip'], c['safety'], c['rear_w'], c['cg'], c['a_motor'], c['vmax'], c['vmin'],
                         c['pose_err'], c['track_err']])


@njit(cache=True)
def footprint_gap(P, x, z, psi, obs):
    hw, f, r, outer, q, rs = P[0], P[1], P[2], P[3], P[4], P[5]
    s = math.sin(psi)
    co = math.cos(psi)
    minx = 1e9
    maxx = -1e9
    minz = 1e9
    maxz = -1e9
    for k in range(4):
        U = f if k < 2 else -r
        V = hw if k % 2 == 0 else -hw
        cx = x + U * s + V * co
        cz = z + U * co - V * s
        minx = min(minx, cx)
        maxx = max(maxx, cx)
        minz = min(minz, cz)
        maxz = max(maxz, cz)
    gap = outer - max(max(maxx, -minx), max(maxz, -minz))
    g_in = max(max(minx - q, -q - maxx), max(minz - q, -q - maxz))
    if g_in < 0:
        umin = 1e9
        umax = -1e9
        vmin = 1e9
        vmax = -1e9
        for k in range(4):
            sx = -q if (k == 0 or k == 3) else q
            sz = -q if k < 2 else q
            u = (sx - x) * s + (sz - z) * co
            v = (sx - x) * co - (sz - z) * s
            umin = min(umin, u)
            umax = max(umax, u)
            vmin = min(vmin, v)
            vmax = max(vmax, v)
        g_in = max(g_in, max(max(umin - f, -r - umax), max(vmin - hw, -hw - vmax)))
    if g_in < gap:
        gap = g_in
    for j in range(obs.shape[0]):
        dx = obs[j, 0] - x
        dz = obs[j, 1] - z
        u = dx * s + dz * co
        v = dx * co - dz * s
        du = max(max(-r - u, u - f), 0.0)
        dv = max(abs(v) - hw, 0.0)
        dd = math.hypot(du, dv) - rs
        if dd < gap:
            gap = dd
    return gap


@njit(cache=True)
def profile(P, x, z, lo0, hi0, d, obs):
    """(tiempo, violacion, clip, v, gap, kappa, room, h). Igual que profile()+eval() del C."""
    n = x.shape[0]
    hw, wall_m = P[0], P[6]
    wb = P[7]
    steer_use, steer_max, steer_rate, rate_use = P[8], P[9], P[10], P[11]
    mu, grip, safety, rear_w, cg, a_motor, vmax, vmin, pose_err, track_err = P[12], P[13], P[14], P[15], P[16], P[17], P[18], P[19], P[20], P[21]
    h = np.empty(n)
    ks = np.empty(n)
    kt = np.empty(n)
    hdg = np.empty(n)
    gap = np.empty(n)
    for i in range(n):
        j = (i + 1) % n
        h[i] = max(math.hypot(x[j] - x[i], z[j] - z[i]), 0.01)
    for i in range(n):
        a = (i + n - 1) % n
        b = (i + 1) % n
        ux = x[i] - x[a]
        uz = z[i] - z[a]
        vx = x[b] - x[i]
        vz = z[b] - z[i]
        wx = x[b] - x[a]
        wz = z[b] - z[a]
        la = math.hypot(ux, uz)
        lb = math.hypot(vx, vz)
        lc = math.hypot(wx, wz)
        cross = ux * vz - uz * vx
        ks[i] = -2 * cross / (la * lb * lc) if (la > 0.01 and lb > 0.01 and lc > 0.01) else 0.0
        hdg[i] = math.atan2(wx, wz)
    for i in range(n):
        gap[i] = footprint_gap(P, x[i], z[i], hdg[i], obs) - wall_m
    for i in range(n):
        kt[i] = 0.25 * ks[(i + n - 1) % n] + 0.5 * ks[i] + 0.25 * ks[(i + 1) % n]
    for i in range(n):
        raw = 0.8 * abs(ks[i])
        if abs(kt[i]) < raw:
            kt[i] = -raw if kt[i] < 0 else raw
    mu_e = mu * grip
    hL = cg / wb
    a_lim = mu_e * G * safety
    a_full = mu_e * G
    cap_acc = min(a_motor, mu_e * G * rear_w / (1 - mu_e * hL))
    cap_dec = mu_e * G * rear_w / (1 + mu_e * hL)
    kmax = math.tan(steer_use * steer_max * math.pi / 180) / wb
    rate = rate_use * steer_rate * math.pi / 180
    seg = np.empty(n)
    viol = 0.0
    for i in range(n):
        j = (i + 1) % n
        seg[i] = math.atan2(x[j] - x[i], z[j] - z[i])
    for i in range(n):
        a = (i + n - 1) % n
        dth = seg[i] - seg[a]
        while dth > math.pi:
            dth -= 2 * math.pi
        while dth < -math.pi:
            dth += 2 * math.pi
        kr = abs(dth) / (0.5 * (h[a] + h[i]))
        if kr > 1.05 * kmax:
            viol += (kr - 1.05 * kmax) * 0.5 * (h[a] + h[i])
    kap = np.abs(kt)
    dl = np.arctan(wb * kt)
    v = np.empty(n)
    room = np.empty(n)
    for i in range(n):
        a = (i + n - 1) % n
        b = (i + 1) % n
        vi = vmax
        k = kap[i]
        if k > 1e-4:
            vi = min(vi, math.sqrt(a_lim / k))
        dds = abs(dl[b] - dl[a]) / (h[a] + h[i])
        if dds > 1e-5:
            vi = min(vi, rate / dds)
        rl = d[i] - lo0[i]
        rr = hi0[i] - d[i]
        room[i] = min(rl, rr)
        allowed = min(room[i], gap[i]) - 0.3
        vc = vmin if allowed <= pose_err else 100.0 * math.sqrt((allowed - pose_err) / max(track_err, 0.01))
        vi = min(vi, vc)
        vi = max(vi, vmin)
        v[i] = vi
    for _ in range(2):
        for i in range(n):
            j = (i + 1) % n
            ay = v[i] * v[i] * kap[i]
            ar = math.sqrt(max(0.0, a_full * a_full - ay * ay))
            vm = math.sqrt(v[i] * v[i] + 2 * min(cap_acc, ar) * h[i])
            if v[j] > vm:
                v[j] = vm
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            ay = v[j] * v[j] * kap[i]
            ar = math.sqrt(max(0.0, a_full * a_full - ay * ay))
            vm = math.sqrt(v[j] * v[j] + 2 * min(cap_dec, ar) * h[i])
            if v[i] > vm:
                v[i] = vm
    t = 0.0
    for i in range(n):
        j = (i + 1) % n
        t += h[i] / (0.5 * (v[i] + v[j]))
    clip = 0.0
    for i in range(n):
        if gap[i] < 0:
            clip -= gap[i]
    return t, viol, clip, v, gap, kt, room, h


@njit(cache=True)
def cost(P, cx, cz, nx, nz, lo0, hi0, d, obs):
    x = cx + d * nx
    z = cz + d * nz
    t, viol, clip, v, gap, ks, room, h = profile(P, x, z, lo0, hi0, d, obs)
    return t + 2000.0 * viol + 4000.0 * clip


def evaluate(T, d):
    x = T.cx + d * T.nx
    z = T.cz + d * T.nz
    t, viol, clip, v, gap, ks, room, h = profile(T.params(), x, z, T.lo0, T.hi0, d, T.obs[:, :2].copy())
    return dict(time=t, viol=viol, clip=clip, v=v, gap=gap, kappa=ks, room=room, x=x, z=z, length=h.sum())
