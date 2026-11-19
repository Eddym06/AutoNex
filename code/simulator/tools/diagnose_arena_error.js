/* Descompone el error del localizador del campo (arena_localizer) contra la verdad del simulador.
 *   node simulator/tools/diagnose_arena_error.js [semillas=8] [salida.json] [config="clave valor;..."]
 * Por muestra (cada tick en RUN con pose valida): error en el marco del robot (a lo largo del avance / transversal), error de rumbo y
 * donde esta el robot (pared que tiene delante: lejana o cercana). Una ronda por proceso, en paralelo, con log. */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const { runJobs, workerJob, workerDone } = require('./par');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const wrap = (a) => { while (a > 180) a -= 360; while (a <= -180) a += 360; return a; };

async function doJob(job) {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = W.field.randomLayout(job.seed, { direction: job.direction, signs: 3, parking: true, startInLot: true });
  const sim = new W.sim.Simulation(core, { field, seed: job.seed, robot: W.robot.PROFILES.largo26, ctrlConfig: job.config || '' });
  const S = [], LAT = {};
  while (sim.t < 200 && !sim.finished) {
    sim.tick();
    const c = sim.cmd;
    if (!c || !c.arena || !c.arena.valid || c.phase !== 'RUN') continue;
    if (Math.floor(c.corners / 4) > 2) break;
    const p = sim.robot.pose(), th = wrap(p.yaw * 180 / Math.PI), a = c.arena;
    const r = th * Math.PI / 180, fx = Math.sin(r), fz = Math.cos(r);           /* avance del robot en el campo */
    /* pose adelantada igual que la usa el mapa (wro_controller: map_pose / trace_map_update): v * latency_s hacia delante y rumbo + vel. angular * latency_s */
    const lat = job.latency === undefined ? 0.10 : job.latency, lead = (c.odom && c.odom.inUse ? c.odom.v : c.speed) * lat;
    const ax = a.x + lead * fx, az = a.z + lead * fz, ath = a.theta + (c.yawRate || 0) * lat;
    const ex = ax - p.x, ez = az - p.z;
    for (const L of [0.06, 0.08, 0.10, 0.12, 0.14]) {              /* que latencia deja menos error a lo largo del avance */
      const e = (a.x + (c.odom && c.odom.inUse ? c.odom.v : c.speed) * L * fx - p.x) * fx + (a.z + (c.odom && c.odom.inUse ? c.odom.v : c.speed) * L * fz - p.z) * fz;
      (LAT[L] || (LAT[L] = { n: 0, s2: 0, s: 0 })); LAT[L].n++; LAT[L].s2 += e * e; LAT[L].s += e;
    }
    S.push({ along: ex * fx + ez * fz, cross: ex * fz - ez * fx, th: wrap(ath - th), ex, ez, v: sim.robot.v, x: p.x, z: p.z, rms: a.rms, turning: Math.abs(c.yawRate || 0) > 8 });
  }
  const st = S.filter(s => !s.turning), tu = S.filter(s => s.turning);
  const rm = (arr, f) => arr.length ? Math.sqrt(arr.reduce((s, x) => s + f(x) ** 2, 0) / arr.length) : 0;
  const parts = { rectas_n: st.length, rectas_pos: rm(st, s => Math.hypot(s.ex, s.ez)), rectas_th: rm(st, s => s.th),
                  giro_n: tu.length, giro_pos: rm(tu, s => Math.hypot(s.ex, s.ez)), giro_th: rm(tu, s => s.th) };
  /* se resume en el proceso para no mandar cientos de miles de muestras */
  const sm = (arr, f) => arr.length ? Math.sqrt(arr.reduce((s, x) => s + f(x) ** 2, 0) / arr.length) : null;
  const mean = (arr, f) => arr.length ? arr.reduce((s, x) => s + f(x), 0) / arr.length : null;
  return { parts, LAT, n: S.length, along_rms: sm(S, s => s.along), cross_rms: sm(S, s => s.cross), th_rms: sm(S, s => s.th), pos_rms: sm(S, s => Math.hypot(s.ex, s.ez)),
    along_mean: mean(S, s => s.along), cross_mean: mean(S, s => s.cross), th_mean: mean(S, s => s.th), ex_mean: mean(S, s => s.ex), ez_mean: mean(S, s => s.ez),
    v_mean: mean(S, s => s.v), samples_pos: S.filter((_, i) => i % 20 === 0).map(s => [s.x, s.z, Math.hypot(s.ex, s.ez)]),
    log: `${S.length} muestras, error de posicion RMS ${sm(S, s => Math.hypot(s.ex, s.ez))?.toFixed(2)} cm, fin ${sim.result().endReason}` };
}

(async () => {
  const job = workerJob();
  if (job) return doJob(job).then(workerDone);
  const seeds = Number(process.argv[2] || 8), config = process.argv[4] || '', jobs = [];
  for (const direction of ['ccw', 'cw']) for (let seed = 1; seed <= seeds; seed++) jobs.push({ direction, seed, config });
  const res = (await runJobs(__filename, jobs)).filter(r => r.result && r.result.n > 0).map(r => r.result);
  const tot = res.reduce((a, r) => a + r.n, 0), w = (k) => Math.sqrt(res.reduce((a, r) => a + r.n * r[k] ** 2, 0) / tot);
  const wm = (k) => res.reduce((a, r) => a + r.n * r[k], 0) / tot;
  const out = { rondas: res.length, muestras: tot, config,
    rms_cm: { posicion: +w('pos_rms').toFixed(3), a_lo_largo_del_avance: +w('along_rms').toFixed(3), transversal: +w('cross_rms').toFixed(3) },
    rms_rumbo_deg: +w('th_rms').toFixed(3),
    a_lo_largo_segun_latencia_asumida: Object.fromEntries(['0.06', '0.08', '0.1', '0.12', '0.14'].map(L => {
      const n = res.reduce((a, r) => a + r.LAT[L].n, 0);
      return [L + ' s', { rms_cm: +Math.sqrt(res.reduce((a, r) => a + r.LAT[L].s2, 0) / n).toFixed(3), sesgo_cm: +(res.reduce((a, r) => a + r.LAT[L].s, 0) / n).toFixed(3) }]; })),
    rectas_y_giros: (() => { const g = (n, v) => Math.sqrt(res.reduce((a, r) => a + r.parts[n] * r.parts[v] ** 2, 0) / res.reduce((a, r) => a + r.parts[n], 0));
      return { rectas_muestras: res.reduce((a, r) => a + r.parts.rectas_n, 0), rectas_pos_rms_cm: +g('rectas_n', 'rectas_pos').toFixed(3), rectas_rumbo_rms_deg: +g('rectas_n', 'rectas_th').toFixed(3),
               giros_muestras: res.reduce((a, r) => a + r.parts.giro_n, 0), giros_pos_rms_cm: +g('giro_n', 'giro_pos').toFixed(3), giros_rumbo_rms_deg: +g('giro_n', 'giro_th').toFixed(3) }; })(),
    sesgo_medio: { a_lo_largo_cm: +wm('along_mean').toFixed(3), transversal_cm: +wm('cross_mean').toFixed(3), rumbo_deg: +wm('th_mean').toFixed(3), x_cm: +wm('ex_mean').toFixed(3), z_cm: +wm('ez_mean').toFixed(3) } };
  console.log(JSON.stringify(out, null, 2));
  if (process.argv[3]) fs.writeFileSync(process.argv[3], JSON.stringify({ out, res }, null, 2) + '\n');
})().catch(e => { console.error(e); process.exit(1); });
