/* Precision del trace_map al cerrar la 1.a vuelta (construccion) y la 3.a (refinada), contra la geometria verdadera.
 *   node simulator/tools/measure_trace_laps.js [semillas=10] [letreros=3] [salida.json]
 * Una ronda por proceso, en paralelo, con una linea de log por ronda. Se informan TODAS las rondas (tambien las que fallan).
 * La verdad solo se usa para evaluar; el controlador no la ve. Instantaneas: justo cuando laps_mapped pasa a 1 y a 3. */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const { runJobs, workerJob, workerDone } = require('./par');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));

function boxDistance(x, z, b) {
  const a = W.geometry.boxAxes(b), dx = x - b.x, dz = z - b.z;
  const qx = dx * a.right[0] + dz * a.right[1], qz = dx * a.forward[0] + dz * a.forward[1];
  return Math.hypot(Math.max(0, Math.abs(qx) - b.w / 2), Math.max(0, Math.abs(qz) - b.d / 2));
}
function perimeter(b) {
  const a = W.geometry.boxAxes(b), pts = [];
  for (const [length, fixed, alongRight] of [[b.w, -b.d / 2, true], [b.w, b.d / 2, true], [b.d, -b.w / 2, false], [b.d, b.w / 2, false]]) {
    const n = Math.max(1, Math.ceil(length));
    for (let i = 0; i < n; i++) {
      const along = -length / 2 + (i + .5) * length / n, x = alongRight ? along : fixed, z = alongRight ? fixed : along;
      pts.push([b.x + x * a.right[0] + z * a.forward[0], b.z + x * a.right[1] + z * a.forward[1]]);
    }
  }
  return pts;
}
function stats(v) {
  if (!v.length) return null;
  const s = [...v].sort((a, b) => a - b), q = f => s[Math.min(s.length - 1, Math.ceil(s.length * f) - 1)];
  const mean = v.reduce((a, b) => a + b, 0) / v.length;
  return { n: v.length, mean: +mean.toFixed(2), rms: +Math.sqrt(v.reduce((a, b) => a + b * b, 0) / v.length).toFixed(2),
    p50: +q(.5).toFixed(2), p95: +q(.95).toFixed(2), max: +s[s.length - 1].toFixed(2) };
}

function snapshot(core, sim) {
  const field = sim.field, tm = core.traceMap(), objs = core.traceObjects(), spot = core.traceSpot();
  const pillars = objs.filter(o => o.kind === 2);
  const truth = field.signs.map(s => ({ x: s.x, z: s.z, color: s.color === 'red' ? 1 : s.color === 'green' ? 2 : 0 }));
  const used = new Set(), posErr = [];
  let found = 0, colorOk = 0;
  for (const t of truth) {
    let best = -1, bd = 10;
    pillars.forEach((o, i) => { const d = Math.hypot(o.x - t.x, o.z - t.z); if (!used.has(i) && d < bd) { bd = d; best = i; } });
    if (best >= 0) { used.add(best); found++; posErr.push(bd); if (pillars[best].color === t.color) colorOk++; }
  }
  const lot = field.lot;
  const ts = lot.axis === 'x' ? { x: (lot.a0 + lot.a1) / 2, z: lot.lateralCenter } : { x: lot.lateralCenter, z: (lot.a0 + lot.a1) / 2 };
  const spotErr = spot.status === 2 ? Math.hypot(spot.x - ts.x, spot.z - ts.z) : null;
  const boxes = sim.renderLidarBoxes().filter(b => b.h >= sim.lidar.cfg.height);
  const rawErr = [], surfErr = [], surfCells = [];
  for (let r = 0; r < tm.size; r++) for (let c = 0; c < tm.size; c++) {
    const i = r * tm.size + c, v = tm.cells[i];
    if (!(v & 1)) continue;
    const p = [tm.originCm + (c + .5) * tm.cellCm, tm.originCm + (r + .5) * tm.cellCm];
    if ((v >> 4) >= 2) rawErr.push(Math.min(...boxes.map(b => boxDistance(p[0], p[1], b))));     /* criterio anterior: >= 2 retornos */
    if ((tm.surface[i >> 3] >> (i & 7)) & 1) { surfErr.push(Math.min(...boxes.map(b => boxDistance(p[0], p[1], b)))); surfCells.push(p); }
  }
  const wallSamples = sim.field.walls.flatMap(perimeter);
  const wallNear = wallSamples.filter(([x, z]) => surfCells.some(p => Math.hypot(p[0] - x, p[1] - z) <= 3.75)).length;
  return { t: +sim.t.toFixed(1), laps: tm.laps, signs: truth.length, found, colorOk, posErr, falseObjects: pillars.length - found,
    spotStatus: spot.status, spotErr, spotLenErr: spot.status === 2 ? Math.abs(spot.length - (lot.a1 - lot.a0)) : null,
    rawErr, surfErr, wallCoveragePct: +(100 * wallNear / wallSamples.length).toFixed(1), clearedCells: tm.cleared };
}

async function doJob(job) {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = W.field.randomLayout(job.seed, { direction: job.direction, signs: job.signs, parking: true, startInLot: true });
  const sim = new W.sim.Simulation(core, { field, seed: job.seed, robot: W.robot.PROFILES.largo26 });
  const snaps = {};
  while (sim.t < 200 && !sim.finished) {
    sim.run(0.5);
    const laps = core.traceMap().laps;
    for (const k of [1, 3]) if (laps >= k && !snaps[k]) snaps[k] = snapshot(core, sim);
    if (snaps[3]) break;                              /* la 3.a vuelta ya esta medida: no hace falta seguir hasta aparcar */
  }
  return { direction: job.direction, seed: job.seed, end: sim.result().endReason, lap1: snaps[1] || null, lap3: snaps[3] || null,
    log: `vuelta1 ${snaps[1] ? 'ok' : 'NO'} vuelta3 ${snaps[3] ? 'ok' : 'NO'} fin=${sim.result().endReason} t=${sim.t.toFixed(0)}s` };
}

function summarize(rows, k) {
  const S = rows.map(r => r[k]).filter(Boolean);
  const sum = f => S.reduce((a, s) => a + f(s), 0);
  const spots = S.filter(s => s.spotStatus === 2);
  return {
    rondas: S.length,
    letreros: { encontrados: `${sum(s => s.found)}/${sum(s => s.signs)}`, color_correcto: `${sum(s => s.colorOk)}/${sum(s => s.found)}`,
      falsos_objetos: sum(s => s.falseObjects), error_posicion_cm: stats(S.flatMap(s => s.posErr)) },
    plaza: { confirmada: `${spots.length}/${S.length}`, error_posicion_cm: stats(spots.map(s => s.spotErr)), error_largo_cm: stats(spots.map(s => s.spotLenErr)) },
    celdas_criterio_anterior_cm: stats(S.flatMap(s => s.rawErr)),
    celdas_superficie_promediada_cm: stats(S.flatMap(s => s.surfErr)),
    cobertura_paredes_pct: stats(S.map(s => s.wallCoveragePct))
  };
}

(async () => {
  const job = workerJob();
  if (job) return doJob(job).then(workerDone);
  const seeds = Number(process.argv[2] || 10), signs = Number(process.argv[3] || 3), jobs = [];
  for (const direction of ['ccw', 'cw']) for (let seed = 1; seed <= seeds; seed++) jobs.push({ direction, seed, signs });
  const rows = (await runJobs(__filename, jobs)).map(r => r.result).filter(Boolean);
  const paired = rows.filter(r => r.lap1 && r.lap3);
  const report = { rondas_totales: rows.length, rondas_que_llegan_a_la_vuelta_3: paired.length,
    pareado_vuelta1: summarize(paired, 'lap1'), pareado_vuelta3: summarize(paired, 'lap3') };
  console.log(JSON.stringify(report, null, 2));
  if (process.argv[4]) fs.writeFileSync(process.argv[4], JSON.stringify({ report, rows }, null, 2) + '\n');
})().catch(e => { console.error(e); process.exit(1); });
