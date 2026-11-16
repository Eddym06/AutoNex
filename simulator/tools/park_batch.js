/* Parqueo de la ronda en paralelo: tiempo en PARKING, repasadas (cambios de sentido de la orden), contactos y resultado.
 *   node simulator/tools/park_batch.js [semillas=8] [signs=0] [--it] [--cfg "clave valor"] [--out f.json]
 * Mide solo el parqueo (la ronda completa se corre igual: arranca en el cajon, 3 vueltas, aparca). */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const { runJobs, workerJob, workerDone } = require('./par');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
async function doJob(job) {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = W.field.randomLayout(job.seed, { direction: job.direction, signs: job.signs, parking: true, startInLot: true, feasible: true, allowGates: false });
  const robot = Object.assign({}, W.robot.PROFILES.largo26d, { speedTauS: 0.08, speedGain: 1 });
  const sim = new W.sim.Simulation(core, { field, seed: job.seed, robot, ctrlConfig: ((job.cfg || '') + (job.it ? '\nit_enabled 1' : '')).trim() });
  let tIn = null, tOut = null, last = 0, rev = 0, phase = '', np = 0; const exp = [];
  while (sim.t < 240 && !sim.finished) {
    sim.tick();
    const c = sim.cmd; if (!c) continue;
    if (c.phase === 'PARKING' && tIn === null) tIn = sim.t;
    if (c.phase === 'PARKING' || c.phase === 'PARK_SEARCH') { const s = Math.abs(c.speed) >= 2 ? Math.sign(c.speed) : 0; if (s) { if (last && s !== last) rev++; last = s; } }
    if (c.phase === 'DONE' && tOut === null) tOut = sim.t;
    if (c.phase === 'PARKING') { const q = core.parkPlan(); if (q.plans !== np) { np = q.plans; exp.push(q.expanded); } }
  }
  const r = sim.run(0.1);
  const pp = core.parkPlan();
  return { exp, plans: pp.plans, planNodes: pp.planNodes, plansS2: pp.plansS2, seed: job.seed, direction: job.direction, end: r.endReason, points: r.points, parking: r.parking, tPark: tIn !== null ? +((tOut ?? sim.t) - tIn).toFixed(1) : null, rev,
           contacts: r.contacts.wall + r.contacts.limiter + r.contacts.sign };
}
(async () => {
  const job = workerJob(); if (job) return doJob(job).then(workerDone);
  const a = process.argv.slice(2).filter(x => !x.startsWith('--') ), fl = (n, d) => { const i = process.argv.indexOf(n); return i >= 0 ? process.argv[i + 1] : d; };
  const seeds = Number(a[0] || 8), signs = Number(a[1] || 0), jobs = [];
  for (const direction of ['ccw', 'cw']) for (let seed = 1; seed <= seeds; seed++) jobs.push({ direction, seed, signs, it: process.argv.includes('--it'), cfg: (fl('--cfg', '') || '').split(';').join('\n') });
  const res = (await runJobs(__filename, jobs, { log: () => {} })).filter(r => r.result).map(r => r.result);
  const m = (f) => res.reduce((s, r) => s + f(r), 0) / res.length;
  const ok = res.filter(r => r.parking === 'full' && r.contacts === 0);
  console.log(`rondas ${res.length} completas-sin-contacto ${ok.length} | tPark medio ${m(r => r.tPark || 0).toFixed(1)} s (max ${Math.max(...res.map(r => r.tPark || 0))}) | repasadas medias ${m(r => r.rev).toFixed(1)} (max ${Math.max(...res.map(r => r.rev))}) | planes ${m(r => r.plans).toFixed(1)} (max ${Math.max(...res.map(r => r.plans))}, desde la entrada ${m(r => r.plansS2).toFixed(1)}) nodos ${(m(r => r.planNodes)/1000).toFixed(0)}k | contactos ${res.reduce((s, r) => s + r.contacts, 0)} | puntos ${m(r => r.points).toFixed(1)}`);
  if (fl('--out')) fs.writeFileSync(fl('--out'), JSON.stringify(res, null, 1));
  { const all = res.flatMap(r => r.exp).sort((a, b) => a - b); const q = f => all[Math.min(all.length - 1, Math.floor(f * all.length))]; console.log(`nodos por plan: n ${all.length} p50 ${q(0.5)} p90 ${q(0.9)} p99 ${q(0.99)} max ${all[all.length - 1]} | >24k: ${all.filter(x => x > 24000).length} | suma ${all.reduce((a, b) => a + b, 0)}`); }
  res.filter(r => !(r.parking === 'full' && r.contacts === 0)).forEach(r => console.log('  fallo:', JSON.stringify(r)));
})();
