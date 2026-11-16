/* Calidad de la adquisicion del cajon (park_localize_acquire): en cada ronda, al empezar PARKING compara la pose estimada con la verdadera y guarda los puntos de
 * pared/topes y la puntuacion del ajuste. Sirve para fijar el umbral que separa un ajuste bueno de una coincidencia con la geometria equivocada.
 *   node simulator/tools/park_acq_stats.js [semillas=24] [signs=5] [--it 1] [--cfg "..."] */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const { runJobs, workerJob, workerDone } = require('./par');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
async function doJob(job) {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = W.field.randomLayout(job.seed, { direction: job.direction, signs: job.signs, parking: true, startInLot: true, feasible: true, allowGates: false });
  const sim = new W.sim.Simulation(core, { field, seed: job.seed, robot: Object.assign({}, W.robot.PROFILES.largo26d), ctrlConfig: ((job.it ? 'it_enabled 1\n' : '') + (job.cfg || '')).trim() });
  const lot = field.lot, ccw = field.direction !== 'cw', centre = (lot.a0 + lot.a1) / 2;
  let ph = '', out = null;
  while (sim.t < 200 && !sim.finished && !out) {
    sim.run(0.1); const c = sim.cmd; if (!c) continue;
    if (c.phase === 'PARKING' && ph !== 'PARKING') {
      const st = core.itState(false), q = sim.robot.pose(), e = c.park || {};
      const u = (ccw ? 1 : -1) * (q.x - centre), w = q.z - lot.wallCoord, yd = q.yaw * 180 / Math.PI, th = ((ccw ? 90 - yd : yd + 90) + 540) % 360 - 180;
      out = { seed: job.seed, direction: job.direction, t: +sim.t.toFixed(1), errU: +((e.u || 0) - u).toFixed(1), errW: +((e.w || 0) - w).toFixed(1), errTh: +(((e.theta || 0) - th)).toFixed(1), trueU: +u.toFixed(1),
              wall: st.acq.wall, rear: st.acq.rear, front: st.acq.front, score: +st.acq.score.toFixed(1) };
    }
    ph = c.phase;
  }
  return out || { seed: job.seed, direction: job.direction, none: true };
}
(async () => {
  const job = workerJob(); if (job) return doJob(job).then(workerDone);
  const a = process.argv.slice(2).filter(x => !x.startsWith('--')), fl = (n, d) => { const i = process.argv.indexOf(n); return i >= 0 ? process.argv[i + 1] : d; };
  const seeds = Number(a[0] || 24), signs = Number(a[1] || 5), jobs = [];
  for (const direction of ['ccw', 'cw']) for (let seed = 1; seed <= seeds; seed++) jobs.push({ direction, seed, signs, it: fl('--it', '1') === '1', cfg: (fl('--cfg', '') || '').split(';').join('\n') });
  const res = (await runJobs(__filename, jobs, { log: () => {} })).filter(r => r.result).map(r => r.result).filter(r => !r.none);
  const bad = res.filter(r => Math.abs(r.errU) > 6), good = res.filter(r => Math.abs(r.errU) <= 6);
  const rng = (v, f) => v.length ? `${Math.min(...v.map(f))}..${Math.max(...v.map(f))}` : '-';
  console.log(`rondas ${res.length}: buenas ${good.length} (|error u| <= 6 cm), malas ${bad.length}`);
  console.log(`buenas : pared ${rng(good, r => r.wall)} trasero ${rng(good, r => r.rear)} delantero ${rng(good, r => r.front)} puntuacion ${rng(good, r => r.score)}`);
  bad.forEach(r => console.log('mala   :', JSON.stringify(r)));
  if (fl('--out')) fs.writeFileSync(fl('--out'), JSON.stringify(res, null, 1));
})();
