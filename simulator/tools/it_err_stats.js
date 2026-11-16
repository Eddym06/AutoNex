/* Error de seguimiento real de IT segun la velocidad: en cada tick con IT al mando guarda (v, |error lateral|). Sirve para calibrar track_err_cm.
 *   node simulator/tools/it_err_stats.js [semillas=12] [signs=5] [--cfg "clave valor;clave valor"] */
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
  const samples = []; let actRun = 0;
  while (sim.t < 240 && !sim.finished) {
    sim.run(0.05);
    const c = sim.cmd; if (!c || c.phase !== 'RUN') continue;
    const st = core.itState(false);
    if (st.active) { actRun++; if (actRun > 40) samples.push([+sim.robot.v.toFixed(0), +Math.abs(st.e).toFixed(2)]); } else actRun = 0;   /* sin los 2 s de enganche */
  }
  const r = sim.run(0.1);
  return { seed: job.seed, direction: job.direction, points: r.points, contacts: r.contacts.sign + r.contacts.wall, t: sim.t, samples };
}
(async () => {
  const job = workerJob(); if (job) return doJob(job).then(workerDone);
  const a = process.argv.slice(2).filter(x => !x.startsWith('--')), fl = (n, d) => { const i = process.argv.indexOf(n); return i >= 0 ? process.argv[i + 1] : d; };
  const seeds = Number(a[0] || 12), signs = Number(a[1] || 5), jobs = [];
  for (const direction of ['ccw', 'cw']) for (let seed = 1; seed <= seeds; seed++) jobs.push({ direction, seed, signs, cfg: (fl('--cfg', '') || '').split(';').join('\n') });
  const res = (await runJobs(__filename, jobs, { log: () => {} })).filter(r => r.result).map(r => r.result);
  const all = res.flatMap(r => r.samples), bins = {};
  for (const [v, e] of all) (bins[Math.min(15, Math.floor(v / 10))] = bins[Math.min(15, Math.floor(v / 10))] || []).push(e);
  console.log(`rondas ${res.length}, muestras ${all.length}, contactos ${res.reduce((s, r) => s + r.contacts, 0)}, puntos medios ${(res.reduce((s, r) => s + r.points, 0) / res.length).toFixed(1)}, t medio ${(res.reduce((s, r) => s + r.t, 0) / res.length).toFixed(1)}`);
  console.log('v(cm/s)  n     p50   p95   p99   max   | modelo 14*(v/100)^2');
  for (const k of Object.keys(bins).map(Number).sort((x, y) => x - y)) {
    const b = bins[k].sort((x, y) => x - y), q = p => b[Math.min(b.length - 1, Math.floor(p * b.length))], vm = k * 10 + 5;
    console.log(`${String(k * 10).padStart(3)}-${String(k * 10 + 9).padEnd(4)} ${String(b.length).padStart(5)}  ${q(0.5).toFixed(2).padStart(5)} ${q(0.95).toFixed(2).padStart(5)} ${q(0.99).toFixed(2).padStart(5)} ${b[b.length - 1].toFixed(2).padStart(5)}   | ${(14 * (vm / 100) ** 2).toFixed(1)}`);
  }
})();
