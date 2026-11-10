/* Que estimador del centro de un letrero se acerca mas a la verdad, a partir del propio mapa (hits / superficie / soporte).
 *   node simulator/tools/diagnose_sign_center.js [semillas=10]   (en paralelo; instantanea al cerrar la vuelta 3)
 * Solo diagnostico: compara formulas sobre las celdas de alrededor del letrero verdadero, sin tocar el controlador. */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const { runJobs, workerJob, workerDone } = require('./par');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const R = 7;                                         /* radio de busqueda alrededor del letrero verdadero (cm) */

async function doJob(job) {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = W.field.randomLayout(job.seed, { direction: job.direction, signs: 3, parking: true, startInLot: true });
  const sim = new W.sim.Simulation(core, { field, seed: job.seed, robot: W.robot.PROFILES.largo26 });
  let tm = null;
  while (sim.t < 200 && !sim.finished) { sim.run(0.5); const l = core.traceMap().laps; if (l >= job.lap) { tm = core.traceMap(); break; } }
  if (!tm) return { rows: [], log: 'no llego a la vuelta ' + job.lap };
  const rows = [];
  for (const s of field.signs) {
    const cells = [];
    for (let r = 0; r < tm.size; r++) for (let c = 0; c < tm.size; c++) {
      const i = r * tm.size + c, v = tm.cells[i];
      if (!(v & 1)) continue;
      const x = tm.originCm + (c + .5) * tm.cellCm, z = tm.originCm + (r + .5) * tm.cellCm;
      if (Math.hypot(x - s.x, z - s.z) > R) continue;
      cells.push({ x, z, w: tm.hits[i], sup: v >> 4, surf: (tm.surface[i >> 3] >> (i & 7)) & 1 });
    }
    const est = (sel, weighted, bbox) => {
      const C = cells.filter(sel); if (!C.length) return null;
      if (bbox) return [(Math.min(...C.map(c => c.x)) + Math.max(...C.map(c => c.x))) / 2, (Math.min(...C.map(c => c.z)) + Math.max(...C.map(c => c.z))) / 2];
      const W_ = C.reduce((a, c) => a + (weighted ? c.w : 1), 0);
      return [C.reduce((a, c) => a + (weighted ? c.w : 1) * c.x, 0) / W_, C.reduce((a, c) => a + (weighted ? c.w : 1) * c.z, 0) / W_];
    };
    const e = {
      raw_sin_peso: est(c => c.sup >= 2, false), raw_con_peso: est(c => c.sup >= 2, true), raw_bbox: est(c => c.sup >= 2, false, true),
      sup_sin_peso: est(c => c.surf, false), sup_con_peso: est(c => c.surf, true), sup_bbox: est(c => c.surf, false, true)
    };
    const out = {};
    for (const k of Object.keys(e)) out[k] = e[k] ? Math.hypot(e[k][0] - s.x, e[k][1] - s.z) : null;
    rows.push(out);
  }
  return { rows, log: `${rows.length} letreros` };
}
(async () => {
  const job = workerJob();
  if (job) return doJob(job).then(workerDone);
  const seeds = Number(process.argv[2] || 10), jobs = [];
  for (const lap of [1, 3]) for (const direction of ['ccw', 'cw']) for (let seed = 1; seed <= seeds; seed++) jobs.push({ direction, seed, lap });
  const res = await runJobs(__filename, jobs);
  for (const lap of [1, 3]) {
    const all = res.filter(r => r.result && r.job.lap === lap).flatMap(r => r.result.rows);
    console.log(`\nvuelta ${lap}: error medio al centro verdadero (cm) por estimador, ${all.length} letreros`);
    for (const k of Object.keys(all[0] || {})) {
      const v = all.map(r => r[k]).filter(x => x !== null);
      console.log(`  ${k.padEnd(14)} media ${(v.reduce((a, b) => a + b, 0) / v.length).toFixed(2)}  max ${Math.max(...v).toFixed(2)}  n=${v.length}`);
    }
  }
})().catch(e => { console.error(e); process.exit(1); });
