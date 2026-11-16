"""Tiempo minimo REAL por control optimo (colocacion directa + IPOPT con CasADi), no por heuristica ni por aprendizaje.
Modelo: bicicleta cinematica en coordenadas de la linea central (Frenet), estados (n, xi, v^2, delta), controles (aceleracion, velocidad del volante),
restricciones: pasillo libre (mismo corredor que el C), margen de seguimiento dependiente de v, circulo de friccion, aceleracion y frenada maximas,
volante y velocidad del servo, vuelta periodica. Minimiza el tiempo de vuelta.
python optimal_control.py plano.json plan_C.json salida.json [--steer 28.8] [--rate 189] [--margin c|none|meas] [--vmax 150]"""
import sys, json, math, time
import numpy as np
import casadi as ca
sys.path.insert(0, '.')
from it_model import Track, evaluate, G


SMOOTH = 1e-3


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def solve(T, C_plan, steer_deg=28.8, rate_dps=189.0, margin='c', vmax=None, verbose=False, footprint=True):
    c = T.c
    n = T.n
    L = c['wb']
    period = None
    # linea central: paso y rumbo/curvatura (izquierda positiva), en el sentido de la marcha
    tx = np.roll(T.cx, -1) - T.cx
    tz = np.roll(T.cz, -1) - T.cz
    ds_i = np.hypot(tx, tz)
    ds = float(ds_i.mean())
    th = np.arctan2(tz, tx)
    kc = np.array([wrap(th[(i + 1) % n] - th[i]) / ds for i in range(n)])
    mu_e = c['mu'] * c['grip']
    a_full = mu_e * G
    a_lim = a_full * c['safety']
    hL = c['cg'] / L
    cap_acc = min(c['a_motor'], a_full * c['rear_w'] / (1 - mu_e * hL))
    cap_dec = a_full * c['rear_w'] / (1 + mu_e * hL)
    dmax = math.radians(steer_deg)
    wmax = math.radians(rate_dps)
    vmx = vmax or c['vmax']
    lo = -T.hi0          # n (izquierda +) = -d (derecha +)
    hi = -T.lo0

    d0_init = np.array([p['d'] for p in C_plan])
    opti = ca.Opti()
    N = n
    nn = opti.variable(N)
    xi = opti.variable(N)
    u = opti.variable(N)
    de = opti.variable(N)
    a = opti.variable(N)
    om = opti.variable(N)

    def f(i, nn_, xi_, u_, de_, a_, om_):
        g = 1 - kc[i] * nn_
        v = ca.sqrt(u_)
        cx_ = ca.cos(xi_)
        kap = ca.tan(de_) / L
        dt = g / (v * cx_)
        return ca.vertcat(g * ca.tan(xi_), kap * g / cx_ - kc[i], 2 * a_ * g / cx_, om_ * dt), dt

    T_total = 0
    for i in range(N):
        j = (i + 1) % N
        fi, dti = f(i, nn[i], xi[i], u[i], de[i], a[i], om[i])
        fj, dtj = f(j, nn[j], xi[j], u[j], de[j], a[j], om[j])
        s_i = ca.vertcat(nn[i], xi[i], u[i], de[i])
        s_j = ca.vertcat(nn[j], xi[j], u[j], de[j])
        opti.subject_to(s_j - s_i == 0.5 * ds * (fi + fj))
        T_total += 0.5 * ds * (dti + dtj)
    for i in range(N):
        ay = u[i] * ca.tan(de[i]) / L
        opti.subject_to(opti.bounded(-dmax, de[i], dmax))
        opti.subject_to(opti.bounded(-wmax, om[i], wmax))
        opti.subject_to(opti.bounded(-cap_dec, a[i], cap_acc))
        opti.subject_to(ay ** 2 <= a_lim ** 2)
        opti.subject_to(a[i] ** 2 + ay ** 2 <= a_full ** 2)
        opti.subject_to(opti.bounded(c['vmin'] ** 2, u[i], vmx ** 2))
        opti.subject_to(opti.bounded(-1.2, xi[i], 1.2))
        if margin in ('c', 'c0'):
            m = 0.3 + c['pose_err'] + c['track_err'] * u[i] / 1e4
        elif margin == 'meas':
            m = 0.3 + c['pose_err'] + 1.0 + 5.0 * ca.sqrt(u[i]) / 100.0
        else:
            m = 0.0
        opti.subject_to(nn[i] <= hi[i] - m)
        opti.subject_to(nn[i] >= lo[i] + m)
    sf = opti.variable(N)
    opti.subject_to(sf >= 0)
    opti.set_initial(sf, 1.0)
    if footprint:
        # huella del chasis = 4 circulos sobre el eje (cubren el rectangulo de 26 x 13 con ~0,8 cm de sobra), como el gap de huella del C
        rc_ = math.hypot(c['hw'], 0.5 * 6.5)
        us = [-c['rear'] + 3.25 + 6.5 * j for j in range(4)]
        Xc = np.array([[p['x'], p['z']] for p in C_plan])
        for i in range(N):
            hx = Xc[(i + 1) % N] - Xc[(i - 1) % N]
            psi0 = math.atan2(hx[1], hx[0])
            psi = th[i] + xi[i]
            px = T.cx[i] - nn[i] * T.nx[i]
            pz = T.cz[i] - nn[i] * T.nz[i]
            if margin == 'c0':
                mf = 1.3 + c['pose_err']
            elif margin == 'c':
                mf = 1.0 + 0.3 + c['pose_err'] + c['track_err'] * u[i] / 1e4
            elif margin == 'meas':
                mf = 1.0 + 0.3 + c['pose_err'] + 1.0 + 5.0 * ca.sqrt(u[i]) / 100.0
            else:
                mf = c['wall_m']
            q = c['inner']
            O = c['outer']
            for uj in us:
                qx = px + uj * ca.cos(psi)
                qz = pz + uj * ca.sin(psi)
                opti.subject_to(ca.fabs(qx) <= O - rc_ - mf + sf[i])
                opti.subject_to(ca.fabs(qz) <= O - rc_ - mf + sf[i])
                # bloque interior: el semiplano segun donde esta el circulo en el plan de partida
                bx = T.cx[i] + d0_init[i] * T.nx[i] + uj * math.cos(psi0)
                bz = T.cz[i] + d0_init[i] * T.nz[i] + uj * math.sin(psi0)
                slack = [bx - q, -q - bx, bz - q, -q - bz]
                k = int(np.argmax(slack))
                if k == 0: opti.subject_to(qx >= q + rc_ + mf - sf[i])
                elif k == 1: opti.subject_to(qx <= -q - rc_ - mf + sf[i])
                elif k == 2: opti.subject_to(qz >= q + rc_ + mf - sf[i])
                else: opti.subject_to(qz <= -q - rc_ - mf + sf[i])
                for o in T.obs:
                    if abs(o[0] - bx) < 60 and abs(o[1] - bz) < 60:
                        opti.subject_to((qx - o[0]) ** 2 + (qz - o[1]) ** 2 >= (rc_ + c['obs_half'] + mf - sf[i]) ** 2)
    reg = 0
    for i in range(N):
        reg += (nn[(i + 1) % N] - 2 * nn[i] + nn[(i - 1) % N]) ** 2
    opti.minimize(T_total + SMOOTH * reg + 50.0 * ca.sum1(sf))

    # punto de partida: el plan del C
    X = np.array([[p['x'], p['z']] for p in C_plan])
    v0 = np.array([p['v'] for p in C_plan])
    d0 = -np.array([p['d'] for p in C_plan])
    xi0 = np.zeros(N)
    for i in range(N):
        hx = X[(i + 1) % N] - X[(i - 1) % N]
        xi0[i] = wrap(math.atan2(hx[1], hx[0]) - th[i])
    opti.set_initial(nn, d0)
    opti.set_initial(xi, np.clip(xi0, -1.0, 1.0))
    opti.set_initial(u, np.clip(v0, 20, vmx) ** 2)
    opti.set_initial(de, np.zeros(N))
    opti.set_initial(a, np.zeros(N))
    opti.set_initial(om, np.zeros(N))
    opti.solver('ipopt', {'expand': True, 'print_time': False}, {'max_iter': 3000, 'tol': 1e-6, 'print_level': 5 if verbose else 0, 'mu_strategy': 'adaptive'})
    t0 = time.time()
    try:
        sol = opti.solve()
        get = sol.value
        ok = True
    except RuntimeError:
        get = opti.debug.value
        ok = False
    nv, uv = np.array(get(nn)), np.array(get(u))
    d = -nv
    x = T.cx + d * T.nx
    z = T.cz + d * T.nz
    print('holgura de huella incumplida (suma, cm):', float(np.sum(get(sf))))
    return dict(ok=ok, time=float(get(T_total)), d=d, x=x, z=z, v=np.sqrt(uv), secs=time.time() - t0)


if __name__ == '__main__':
    a = sys.argv
    if '--smooth' in a:
        SMOOTH = float(a[a.index('--smooth') + 1])
    fl = lambda k, dflt: a[a.index(k) + 1] if k in a else dflt
    plan = json.load(open(a[1]))
    T = Track(plan['signs'], plan['direction'])
    Cp = json.load(open(a[2]))
    r = solve(T, [dict(p, d=p['d']) for p in Cp], float(fl('--steer', 28.8)), float(fl('--rate', 189)), fl('--margin', 'c'), float(fl('--vmax', 0)) or None, '-v' in a, '--nofoot' not in a)
    e = evaluate(T, r['d'])
    print('control optimo (%s): convergio=%s tiempo=%.3f s | reevaluado con el modelo del C: t=%.3f viol=%.3f clip=%.2f largo=%.1f vmax=%.0f vmin=%.0f (%.0f s)' %
          (fl('--margin', 'c'), r['ok'], r['time'], e['time'], e['viol'], e['clip'], e['length'], r['v'].max(), r['v'].min(), r['secs']))
    json.dump(dict(x=r['x'].tolist(), z=r['z'].tolist(), v=r['v'].tolist(), d=r['d'].tolist(), time=r['time']), open(a[3], 'w'))
