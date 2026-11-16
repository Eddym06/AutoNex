/* Mide SOLO el tramo conducido por IT en la ronda completa (el tiempo total lo domina el parqueo, que es caotico): tiempo desde que IT toma el
 * mando hasta que lo cede, distancia recorrida, tiempo por cada 100 cm de recorrido, contactos durante IT y fraccion del tramo en que realmente conduce.
 *   node simulator/tools/it_laps.js [semillas=12] [signs=5] [--cfg "clave valor;clave valor"] [--out f.json]  */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const { runJobs, workerJob, workerDone } = require('./par');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
async function doJob(job) {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = W.field.randomLayout(job.seed, { direction: job.direction, signs: job.signs, parking: true, startInLot: true, feasible: true, allowGates: false });
  const robot = Object.assign({}, W.robot.PROFILES.largo26d, { speedTauS: 0.08, speedGain: 1 });
  const sim = new W.sim.Simulation(core, { field, seed: job.seed, robot, ctrlConfig: ('it_enabled 1\n' + (job.cfg || '')).trim() });
  let hit = null, tFirst = null, tLast = null, active = 0, ticks = 0, dist = 0, drops = 0, c0 = 0, cFirst = null, last = null, vmaxIt = 0, e2 = 0;
  const DT = 0.05;
  while (sim.t < 240 && !sim.finished) {
    sim.run(DT);
    const c = sim.cmd; if (!c || c.phase !== 'RUN') continue;
    const st = core.itState(false);
    const cont = sim.judge && sim.judge.contacts ? sim.judge.contacts.sign + sim.judge.contacts.wall : 0;
    if (st.active) {
      if (tFirst === null) { tFirst = sim.t; cFirst = cont; }
      tLast = sim.t; active++; dist += sim.robot.v * DT; vmaxIt = Math.max(vmaxIt, sim.robot.v); e2 = Math.max(e2, Math.abs(st.e));
    }
    if (tFirst !== null && cont > c0 && !hit) { const q = sim.robot.pose(); hit = { t: +sim.t.toFixed(1), x: +q.x.toFixed(0), z: +q.z.toFixed(0), v: +sim.robot.v.toFixed(0), e: +st.e.toFixed(1), idx: st.idx, vPlan: +st.vPlan.toFixed(0), act: st.active }; }
    if (tFirst !== null) { ticks++; drops = st.drops; c0 = cont; }
  }
  const r = sim.run(0.1);
  const span = tFirst === null ? 0 : tLast - tFirst;
  return { seed: job.seed, direction: job.direction, points: r.points, end: r.endReason, engaged: tFirst !== null, span, dist, activeFrac: span > 0 ? Math.min(1, active * DT / span) : 0, drops,
           contacts: tFirst === null ? 0 : c0 - cFirst, sPer100: dist > 0 ? 100 * active * DT / dist : null, maxErr: e2, vmax: vmaxIt, hit };
}
(async () => {
  const job = workerJob(); if (job) return doJob(job).then(workerDone);
  const a = process.argv.slice(2).filter(x => !x.startsWith('--')), fl = (n, d) => { const i = process.argv.indexOf(n); return i >= 0 ? process.argv[i + 1] : d; };
  const seeds = Number(a[0] || 12), signs = Number(a[1] || 5), jobs = [];
  for (const direction of ['ccw', 'cw']) for (let seed = 1; seed <= seeds; seed++) jobs.push({ direction, seed, signs, cfg: (fl('--cfg', '') || '').split(';').join('\n') });
  const res = (await runJobs(__filename, jobs, { log: () => {} })).filter(r => r.result).map(r => r.result);
  const eng = res.filter(r => r.engaged), m = (f, v = eng) => v.length ? v.reduce((s, r) => s + f(r), 0) / v.length : 0;
  console.log(`rondas ${res.length} con IT ${eng.length} | s por 100 cm conducidos ${m(r => r.sPer100 || 0).toFixed(3)} | tramo IT ${m(r => r.span).toFixed(1)} s | fraccion conduciendo ${(100 * m(r => r.activeFrac)).toFixed(0)} % | cedidas ${m(r => r.drops).toFixed(2)} | contactos en IT ${eng.reduce((s, r) => s + r.contacts, 0)} (rondas ${eng.filter(r => r.contacts > 0).length}) | err lateral max medio ${m(r => r.maxErr).toFixed(1)} cm | puntos ${m(r => r.points, res).toFixed(1)}`);
  res.filter(r => r.hit).forEach(r => console.log('  contacto', r.direction + r.seed, JSON.stringify(r.hit)));
  if (fl('--out')) fs.writeFileSync(fl('--out'), JSON.stringify(res, null, 1));
})();
