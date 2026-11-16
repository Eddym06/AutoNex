"""python run_baseline.py plano.json [linea_C.json]: valida la replica contra el plan del C (tiempo y violacion deben coincidir)."""
import sys, json
import numpy as np
sys.path.insert(0, '.')
from it_model import Track, evaluate

plan = json.load(open(sys.argv[1]))
T = Track(plan['signs'], plan['direction'])
print('estaciones', T.n)
d0 = np.clip(np.zeros(T.n), T.lo0, T.hi0)
r = evaluate(T, d0)
print('linea central acotada: t=%.3f viol=%.3f clip=%.2f' % (r['time'], r['viol'], r['clip']))
if len(sys.argv) > 2:
    L = json.load(open(sys.argv[2]))
    d = np.array([l['d'] for l in L])
    r = evaluate(T, d)
    print('plan del C reevaluado en Python: t=%.3f viol=%.3f clip=%.2f largo=%.1f' % (r['time'], r['viol'], r['clip'], r['length']))
