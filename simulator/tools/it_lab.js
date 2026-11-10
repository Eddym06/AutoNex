/* Banco de la trayectoria inteligente (IT): el planificador C (it_plan.c, en wasm) descubre la trayectoria y el perfil de velocidad
 * sobre el mapa de letreros, y un seguidor lo conduce con el robot del simulador CON dinamica (servo MG90S, neumaticos que saturan,
 * peso detras). El agarre se aprende vuelta a vuelta.
 *   node simulator/tools/it_lab.js [semillas=6] [--signs 3] [--laps 4] [--mu 0.8] [--grip0 0.6] [--noise 1] [--accel 350]
 *                                  [--vmax 150] [--map-err 0.9] [--out archivo.json]
 * --mu: agarre REAL del simulador (el planificador siempre supone 0,8: la diferencia la descubre el aprendizaje del agarre).
 * --noise: escala del error de pose que ve el seguidor (1 = 0,8 cm RMS y 0,3 grados, como arena_localizer con el adelanto de latencia).
 * --map-err: error (cm) de la posicion de los letreros en el mapa (0,9 = lo medido en trace_map).
 * Es un banco: no usa el controlador de la mision (sin vuelta de construccion ni parqueo). Mide lo que IT puede dar por si solo. */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const { runJobs, workerJob, workerDone } = require('./par');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const DEG = Math.PI / 180;
const wrap = (a) => { while (a > 180) a -= 360; while (a <= -180) a += 360; return a; };

async function doJob(job) {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = W.field.randomLayout(job.seed, { direction: job.direction, signs: job.signs, parking: false });
  let s = (job.seed * 2654435761) >>> 0;
  const rnd = () => { s ^= s << 13; s >>>= 0; s ^= s >> 17; s ^= s << 5; s >>>= 0; return s / 4294967296; };
  const gauss = () => Math.sqrt(-2 * Math.log(rnd() + 1e-12)) * Math.cos(2 * Math.PI * rnd());
  const colorOf = (c) => (c === 'red' || c === 1) ? 1 : (c === 'green' || c === 2) ? 2 : 0;
  /* el mapa conoce cada letrero con un error (trace_map: 0,9 cm de media) */
  const known = field.signs.map(q => ({ x: q.x + gauss() * job.mapErr / 1.25, z: q.z + gauss() * job.mapErr / 1.25, color: colorOf(q.color) }));
  const base = Object.assign({}, W.robot.DEFAULT_ROBOT, W.robot.PROFILES.largo26d, { speedGain: 1, maxAccel: job.accel, speedTauS: 0.08 });
  base.tire = Object.assign({}, base.tire, { mu: job.mu });
  const robot = new W.robot.Robot(base);
  const params = { grip: job.grip0, vMax: job.vmax, aMotor: job.accel * 0.9, trackErr: job.trackErr, poseErr: job.poseErr, ctrlMode: job.mode + 1, kPsi: job.kpsi, kE: job.ke };
  let plan = core.itBuild(job.direction, params, known);
  if (!plan.ok) return { seed: job.seed, direction: job.direction, error: 'plan no valido', log: `${job.direction} ${job.seed}: PLAN NO VALIDO` };
  const buildMs = [];
  const p0 = plan.line[0], p1 = plan.line[1];
  robot.reset({ x: p0.x, z: p0.z, yaw: Math.atan2(p1.x - p0.x, p1.z - p0.z) });
  robot.v = 0;
  const world = { solids: field.walls, signs: field.signs };
  const dt = 0.005, ctl = job.ctl;
  let t = 0, nextCtl = 0, lap = 0, lapT0 = 0, last = -1, grip = job.grip0, fallback = 0, stuck = 0;
  let st = { maxErr: 0, minRoom: 1e9, contacts: 0, slip: 0, traction: 0, ticks: 0, vsum: 0, vmax: 0 };
  const laps = [];
  let firstHit = null;
  let cmd = { steer: 0, speed: 0 }, pending = [], pose = { x: 0, z: 0, yaw: 0 };
  let nx = 0, nz = 0, nth = 0;                                 /* ruido de pose correlacionado (AR1, ~0,3 s) */
  const ar = Math.exp(-ctl / 0.3), sg = Math.sqrt(1 - ar * ar);
  while (t < job.limit && laps.length < job.laps) {
    if (t >= nextCtl) {
      nextCtl += ctl;
      const p = robot.pose();
      nx = ar * nx + sg * gauss() * 0.57 * job.noise; nz = ar * nz + sg * gauss() * 0.57 * job.noise; nth = ar * nth + sg * gauss() * 0.3 * job.noise;
      pose = { x: p.x + nx, z: p.z + nz, yaw: p.yaw / DEG + nth };
      const r = core.itTrack(pose.x, pose.z, pose.yaw, Math.max(0, robot.v));
      if (r.ok) pending.push([t + 0.02, { steer: r.steer, speed: r.speed }]);       /* 20 ms de calculo y bus */
      else { fallback++; pending.push([t + 0.02, { steer: 0, speed: 12 }]); }
      if (r.ok) {
        if (last >= 0 && last - r.idx > plan.n / 2) {                              /* cierre de vuelta */
          const lt = t - lapT0; lapT0 = t;
          const rec = { lap: laps.length + 1, time: lt, grip, maxErr: st.maxErr, minRoom: st.minRoom, contacts: st.contacts, slipDeg: st.slip,
                        traction: st.traction, vmean: st.vsum / Math.max(1, st.ticks), vmax: st.vmax, fallback };
          laps.push(rec);
          const ng = core.itAdapt(st.contacts, st.maxErr, st.minRoom, st.slip, st.traction, job.gripMax);
          if (Math.abs(ng - grip) > 1e-6 && laps.length < job.laps) {
            grip = ng; params.grip = grip;
            const tb = Date.now(); plan = core.itBuild(job.direction, params, known); buildMs.push(Date.now() - tb);
            if (!plan.ok) return { seed: job.seed, direction: job.direction, error: 'replan no valido', laps, log: `${job.direction} ${job.seed}: REPLAN NO VALIDO` };
          }
          st = { maxErr: 0, minRoom: 1e9, contacts: 0, slip: 0, traction: 0, ticks: 0, vsum: 0, vmax: 0 };
        }
        last = r.idx;
        st.maxErr = Math.max(st.maxErr, Math.abs(r.e)); st.minRoom = Math.min(st.minRoom, r.room - Math.abs(r.e));
      }
    }
    while (pending.length && pending[0][0] <= t) cmd = pending.shift()[1];
    robot.cmd = cmd;
    const ev = robot.step(dt, world);
    for (const e of ev) if (e.kind === 'sign' || e.kind === 'wall') {
      st.contacts++;
      if (!firstHit) { const q = robot.pose(); firstHit = { t: +t.toFixed(2), kind: e.kind, x: +q.x.toFixed(1), z: +q.z.toFixed(1), v: +robot.v.toFixed(0), steer: +robot.steer.toFixed(1), idx: last, lap: laps.length + 1 }; }
    }
    t += dt;
    st.slip = Math.max(st.slip, Math.abs(robot.slipAngleDeg || 0));
    if (robot.tractionLimited) st.traction++;
    st.ticks++; st.vsum += robot.v; st.vmax = Math.max(st.vmax, robot.v);
    stuck = robot.v < 2 ? stuck + dt : 0;
    if (stuck > 3) break;
  }
  const done = laps.length >= job.laps;
  return { seed: job.seed, direction: job.direction, laps, planTime: plan.time, planTimeCenter: plan.timeCenter, planLength: plan.length, buildMs, evals: plan.evals, finished: done, stuck: stuck > 3,
    firstHit, signs: field.signs.map(q => [colorOf(q.color), +q.seat.x.toFixed(0), +q.seat.z.toFixed(0)]),
    log: `${job.direction} ${job.seed}: ${laps.map(l => `${l.time.toFixed(1)}s(g${l.grip.toFixed(2)} c${l.contacts})`).join(' ')} plan ${plan.time.toFixed(1)}s${done ? '' : ' INCOMPLETO t=' + t.toFixed(1)}${firstHit ? ' 1er contacto ' + JSON.stringify(firstHit) : ''}` };
}

(async () => {
  const job = workerJob();
  if (job) return doJob(job).then(workerDone);
  const a = process.argv.slice(2);
  const fl = (n, d) => { const i = a.indexOf(n); return i >= 0 ? a[i + 1] : d; };
  const first = a.length && !a[0].startsWith('--') ? Number(a[0]) : 6;
  const o = { signs: Number(fl('--signs', 3)), laps: Number(fl('--laps', 4)), mu: Number(fl('--mu', 0.8)), grip0: Number(fl('--grip0', 0.6)), gripMax: Number(fl('--gripmax', 1)),
    noise: Number(fl('--noise', 1)), accel: Number(fl('--accel', 350)), vmax: Number(fl('--vmax', 150)), mapErr: Number(fl('--map-err', 0.9)),
    ctl: Number(fl('--ctl', 0.02)), mode: Number(fl('--mode', 1)), kpsi: Number(fl('--kpsi', 0)), ke: Number(fl('--ke', 0)), trackErr: Number(fl('--track-err', 0)), poseErr: Number(fl('--pose-err', 0)), limit: Number(fl('--limit', 150)) };
  const jobs = [];
  for (const direction of ['ccw', 'cw']) for (let seed = 1; seed <= first; seed++) jobs.push(Object.assign({ direction, seed }, o));
  const res = (await runJobs(__filename, jobs)).filter(r => r.result).map(r => r.result);
  const ok = res.filter(r => r.laps && r.laps.length);
  const lapK = (k) => ok.map(r => r.laps[k]).filter(Boolean);
  const mean = (v, f) => v.length ? v.reduce((s, x) => s + f(x), 0) / v.length : null;
  const out = { opciones: o, rondas: res.length, completas: res.filter(r => r.finished).length, atascadas: res.filter(r => r.stuck).length,
    por_vuelta: [0, 1, 2, 3].filter(k => lapK(k).length).map(k => ({ vuelta: k + 1, n: lapK(k).length, tiempo_s: +mean(lapK(k), l => l.time).toFixed(2), agarre: +mean(lapK(k), l => l.grip).toFixed(2),
      contactos_medios: +mean(lapK(k), l => l.contacts).toFixed(2), rondas_con_contacto: lapK(k).filter(l => l.contacts > 0).length, v_media: +mean(lapK(k), l => l.vmean).toFixed(0), v_max: +Math.max(...lapK(k).map(l => l.vmax)).toFixed(0),
      error_lateral_max_cm: +mean(lapK(k), l => l.maxErr).toFixed(1), holgura_min_cm: +mean(lapK(k), l => l.minRoom).toFixed(1), deriva_max_deg: +mean(lapK(k), l => l.slipDeg).toFixed(1) })),
    plan_medio_s: +mean(ok, r => r.planTime).toFixed(2), construccion_ms_wasm: +mean(ok.filter(r => r.buildMs && r.buildMs.length), r => r.buildMs.reduce((s, x) => s + x, 0) / r.buildMs.length).toFixed(0),
    sin_plan: res.filter(r => r.error && !(r.laps && r.laps.length)).length };
  console.log(JSON.stringify(out, null, 2));
  const outFile = fl('--out', null);
  if (outFile) fs.writeFileSync(outFile, JSON.stringify({ out, res }, null, 2) + '\n');
})().catch(e => { console.error(e); process.exit(1); });
