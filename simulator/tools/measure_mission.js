/* Mide rondas completas del controlador (en paralelo, con log): fin, puntos, tiempo total y por vuelta, contactos.
 *   node simulator/tools/measure_mission.js [semillas=10] [perfil=largo26d] ["clave valor;clave valor"] [salida.json] [--signs N] [--noparking]
 * Una ronda por proceso. Sirve para comparar perfiles de chasis (cinematico / con dinamica) y configuraciones del controlador. */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const { runJobs, workerJob, workerDone } = require('./par');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));

async function doJob(job) {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = W.field.randomLayout(job.seed, { direction: job.direction, signs: job.signs, parking: !job.noparking, startInLot: !job.noparking });
  const sim = new W.sim.Simulation(core, { field, seed: job.seed, robot: W.robot.PROFILES[job.profile], ctrlConfig: job.config || '' });
  const lapT = {}; let lastLap = 0, vmax = 0, vsum = 0, vn = 0, slipMax = 0, tractionLim = 0, ticks = 0;
  while (sim.t < job.limit && !sim.finished) {
    sim.run(0.1);
    const c = sim.cmd;
    if (!c) continue;
    if (c.phase === 'RUN') {
      const lap = Math.floor(c.corners / 4) + 1;
      if (lap !== lastLap) { if (lastLap) lapT[lastLap] = sim.t - (lapT._t0 || 0); lapT._t0 = sim.t; lastLap = lap; }
      vmax = Math.max(vmax, sim.robot.v); vsum += sim.robot.v; vn++;
      slipMax = Math.max(slipMax, Math.abs(sim.robot.slipAngleDeg || 0));
      ticks++; if (sim.robot.tractionLimited) tractionLim++;
    }
  }
  const r = sim.run(0.1);
  const out = { seed: job.seed, direction: job.direction, end: r.endReason, points: r.points, elapsed: sim.t, violations: r.violations.length,
    contacts: r.contacts.sign + r.contacts.wall, parking: r.parking, vmax, vmean: vn ? vsum / vn : 0, slipMax, tractionLimited: ticks ? tractionLim / ticks : 0,
    laps: Object.fromEntries(Object.entries(lapT).filter(([k]) => k !== '_t0')) };
  out.log = `${job.direction} ${job.seed}: ${out.end} ${out.points} pts ${out.elapsed.toFixed(0)} s v medio ${out.vmean.toFixed(1)} max ${out.vmax.toFixed(0)} contactos ${out.contacts}`;
  return out;
}

(async () => {
  const job = workerJob();
  if (job) return doJob(job).then(workerDone);
  const a = process.argv.slice(2).filter(x => !x.startsWith('--'));
  const seeds = Number(a[0] || 10), profile = a[1] || 'largo26d', config = (a[2] && a[2] !== '-' ? a[2] : '').split(';').join('\n'), outFile = a[3];
  const fl = (n, d) => { const i = process.argv.indexOf(n); return i >= 0 ? process.argv[i + 1] : d; };
  const signs = Number(fl('--signs', 3)), noparking = process.argv.includes('--noparking'), limit = Number(fl('--limit', 240));
  const jobs = [];
  for (const direction of ['ccw', 'cw']) for (let seed = 1; seed <= seeds; seed++) jobs.push({ direction, seed, profile, config, signs, noparking, limit });
  const res = (await runJobs(__filename, jobs)).filter(r => r.result).map(r => r.result);
  const full = res.filter(r => r.points >= 0);
  const mean = (f, arr = res) => arr.length ? arr.reduce((s, r) => s + f(r), 0) / arr.length : 0;
  const lapMean = (k) => { const v = res.map(r => r.laps[k]).filter(x => x > 0); return v.length ? v.reduce((s, x) => s + x, 0) / v.length : null; };
  const ends = {}; for (const r of res) ends[r.end] = (ends[r.end] || 0) + 1;
  const summary = { rondas: res.length, profile, config: a[2] || '', fin: ends, puntos_medios: +mean(r => r.points).toFixed(2), tiempo_medio_s: +mean(r => r.elapsed).toFixed(1),
    vuelta1_s: lapMean(1) && +lapMean(1).toFixed(1), vuelta2_s: lapMean(2) && +lapMean(2).toFixed(1), vuelta3_s: lapMean(3) && +lapMean(3).toFixed(1),
    v_media_cm_s: +mean(r => r.vmean).toFixed(1), v_max_cm_s: +Math.max(...res.map(r => r.vmax)).toFixed(0), contactos_medios: +mean(r => r.contacts).toFixed(2),
    deriva_trasera_max_deg: +Math.max(...res.map(r => r.slipMax)).toFixed(1), fraccion_con_tope_de_traccion: +mean(r => r.tractionLimited).toFixed(3) };
  console.log(JSON.stringify(summary, null, 2));
  if (outFile) fs.writeFileSync(outFile, JSON.stringify({ summary, res }, null, 2) + '\n');
})().catch(e => { console.error(e); process.exit(1); });
