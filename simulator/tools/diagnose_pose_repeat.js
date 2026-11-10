/* Por que la 3.a vuelta no baja el error: el error de la pose del campo (arena_localizer) se repite en cada vuelta en el mismo sitio
 * (sistematico, no se promedia) o cambia de una vuelta a otra (aleatorio, si se promedia)?
 *   node simulator/tools/diagnose_pose_repeat.js [semillas=8] [salida.json]
 * Se agrupa la trayectoria en casillas de 25 cm. Por casilla se compara el error medio de la pose (arena - verdad) de la vuelta 1 y de la 3. */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const { runJobs, workerJob, workerDone } = require('./par');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const BIN = 25;

async function doJob(job) {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = W.field.randomLayout(job.seed, { direction: job.direction, signs: 3, parking: true, startInLot: true });
  const sim = new W.sim.Simulation(core, { field, seed: job.seed, robot: W.robot.PROFILES.largo26 });
  const acc = {};                                    /* "lap|bx|bz" -> {n, ex, ez} */
  while (sim.t < 200 && !sim.finished) {
    sim.tick();
    const c = sim.cmd;
    if (!c || !c.arena || !c.arena.valid || c.phase !== 'RUN') continue;
    const lap = Math.floor(c.corners / 4);
    if (lap > 2) break;
    const p = sim.robot.pose();
    const k = `${lap}|${Math.floor(p.x / BIN)}|${Math.floor(p.z / BIN)}`;
    const a = acc[k] || (acc[k] = { n: 0, ex: 0, ez: 0 });
    a.n++; a.ex += c.arena.x - p.x; a.ez += c.arena.z - p.z;
  }
  return { bins: acc, log: `fin ${sim.result().endReason} t=${sim.t.toFixed(0)}s casillas=${Object.keys(acc).length}` };
}

const rms = (v) => v.length ? +Math.sqrt(v.reduce((a, b) => a + b * b, 0) / v.length).toFixed(2) : null;

(async () => {
  const job = workerJob();
  if (job) return doJob(job).then(workerDone);
  const seeds = Number(process.argv[2] || 8), jobs = [];
  for (const direction of ['ccw', 'cw']) for (let seed = 1; seed <= seeds; seed++) jobs.push({ direction, seed });
  const results = await runJobs(__filename, jobs);
  const e1 = [], e3 = [], diff = [], pairs = [];
  for (const r of results) {
    if (!r.result) continue;
    const b = r.result.bins;
    for (const k of Object.keys(b)) {
      const [lap, bx, bz] = k.split('|');
      if (lap !== '0' || b[k].n < 5) continue;
      const k3 = `2|${bx}|${bz}`;
      if (!b[k3] || b[k3].n < 5) continue;
      const m1 = [b[k].ex / b[k].n, b[k].ez / b[k].n], m3 = [b[k3].ex / b[k3].n, b[k3].ez / b[k3].n];
      e1.push(m1[0], m1[1]); e3.push(m3[0], m3[1]); diff.push(m1[0] - m3[0], m1[1] - m3[1]); pairs.push([m1, m3]);
    }
  }
  let sxy = 0, sxx = 0, syy = 0;
  for (const [m1, m3] of pairs) for (let i = 0; i < 2; i++) { sxy += m1[i] * m3[i]; sxx += m1[i] * m1[i]; syy += m3[i] * m3[i]; }
  const out = { casillas_pareadas_vuelta1_vuelta3: pairs.length,
    rms_error_pose_vuelta1_cm: rms(e1), rms_error_pose_vuelta3_cm: rms(e3), rms_diferencia_vuelta1_menos_vuelta3_cm: rms(diff),
    correlacion_error_vuelta1_vuelta3: +(sxy / Math.sqrt(sxx * syy)).toFixed(3) };
  console.log(JSON.stringify(out, null, 2));
  if (process.argv[3]) fs.writeFileSync(process.argv[3], JSON.stringify(out, null, 2) + '\n');
})().catch((e) => { console.error(e); process.exit(1); });
