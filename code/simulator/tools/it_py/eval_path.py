"""Evalua en el modelo una trayectoria LIBRE (polilinea): el plan del C con el tramo del usuario (linea roja digitalizada de su imagen) en su lugar.
python eval_path.py plano.json plan_C.json red_line.json salida.json"""
import sys, json
import numpy as np
sys.path.insert(0, '.')
from it_model import Track, profile


def resample(poly, step=5.0):
    poly = np.asarray(poly, float)
    seg = np.hypot(*np.diff(poly, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    t = np.arange(0, s[-1], step)
    return np.stack([np.interp(t, s, poly[:, 0]), np.interp(t, s, poly[:, 1])], 1)


def smooth(poly, it=2):
    p = poly.copy()
    for _ in range(it):
        p[1:-1] = 0.25 * p[:-2] + 0.5 * p[1:-1] + 0.25 * p[2:]
    return p


def eval_free(T, x, z):
    """Evalua la polilinea cerrada (x, z): room y corredor por la estacion mas cercana."""
    n = len(x)
    lo = np.empty(n)
    hi = np.empty(n)
    d = np.empty(n)
    for k in range(n):
        i = np.argmin(np.hypot(T.cx - x[k], T.cz - z[k]))
        d[k] = (x[k] - T.cx[i]) * T.nx[i] + (z[k] - T.cz[i]) * T.nz[i]
        lo[k], hi[k] = T.lo0[i], T.hi0[i]
    t, viol, clip, v, gap, ks, room, h = profile(T.params(), np.ascontiguousarray(x), np.ascontiguousarray(z), lo, hi, d, T.obs[:, :2].copy())
    return dict(time=t, viol=viol, clip=clip, v=v, gap=gap, kappa=ks, room=room, length=h.sum(), x=x, z=z)


def splice(T, C, red, a_pt, b_pt):
    """C: (n,2) plan del C en orden de vuelta; sustituye el tramo entre los puntos mas cercanos a a_pt y b_pt por red (remuestreada)."""
    ia = int(np.argmin(np.hypot(C[:, 0] - a_pt[0], C[:, 1] - a_pt[1])))
    ib = int(np.argmin(np.hypot(C[:, 0] - b_pt[0], C[:, 1] - b_pt[1])))
    n = len(C)
    keep = []
    k = (ib + 1) % n
    while k != ia:
        keep.append(C[k])
        k = (k + 1) % n
    keep = np.array(keep)
    return np.vstack([resample(red, 5.0), keep]), ia, ib


if __name__ == '__main__':
    plan = json.load(open(sys.argv[1]))
    T = Track(plan['signs'], plan['direction'])
    L = json.load(open(sys.argv[2]))
    C = np.array([[l['x'], l['z']] for l in L])
    red = np.array(json.load(open(sys.argv[3])))
    r0 = eval_free(T, C[:, 0].copy(), C[:, 1].copy())
    print('plan C       : t=%.3f viol=%.3f clip=%.2f largo=%.1f' % (r0['time'], r0['viol'], r0['clip'], r0['length']))
    P, ia, ib = splice(T, C, red, red[0], red[-1])
    print('empalme: estaciones C %d..%d sustituidas por %d puntos del usuario' % (ib, ia, len(resample(red))))
    for it in (0, 2):
        Q = smooth(P, it) if it else P
        r = eval_free(T, Q[:, 0].copy(), Q[:, 1].copy())
        print('C + usuario (suavizado %d): t=%.3f viol=%.3f clip=%.2f largo=%.1f vmin=%.0f' % (it, r['time'], r['viol'], r['clip'], r['length'], r['v'].min()))
    json.dump(dict(x=r['x'].tolist(), z=r['z'].tolist(), v=r['v'].tolist(), time=r['time']), open(sys.argv[4], 'w'))
    bad = [(k, r['x'][k], r['z'][k], r['gap'][k], r['v'][k], r['kappa'][k]) for k in range(len(r['x'])) if r['gap'][k] < 0.3 or r['v'][k] < 40]
    print('puntos con holgura <0.3 o v<40:')
    for b in bad[:30]:
        print('  %d (%.0f,%.0f) gap=%.1f v=%.0f k=%.3f' % (b[0], b[1], b[2], b[3], b[4], b[5] * 100))
