/* Cuenta las repasadas (cambios de sentido) de la entrada al cajon y de la salida,
 * con la pose verdadera y la orden del firmware. Herramienta de medida y de regresion.
 *   node simulator/tests/parkentry.js [perfil] [modo] [segundos] [semillas]
 * MAX_REVERSALS=3 exige las repasadas y resultado completo en cada ronda.
 * modo: entrada (arranca en la pista y aparca) | salida (arranca dentro del cajon) | ambos */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const F = WRO.field;
const bytes = fs.readFileSync(path.join(web, 'wro_core.wasm'));
let core;

/* Una repasada es un cambio de sentido con movimiento: se ignoran las ordenes de 0 y las
 * vibraciones menores que el motor no llega a ejecutar. */
function reversals(seq) {
  let n = 0, last = 0;
  for (const s of seq) {
    if (Math.abs(s) < 2.0) continue;        // cm/s: por debajo no arranca
    const sign = s > 0 ? 1 : -1;
    if (last && sign !== last) n++;
    last = sign;
  }
  return n;
}

async function run(profile, mode, seconds, seeds) {
  const robot = WRO.robot.PROFILES[profile];
  const rows = [];
  for (const direction of (process.env.DIR ? [process.env.DIR] : ['ccw', 'cw'])) for (const seed of seeds) {
    const field = mode === 'ronda'
      ? F.randomLayout(seed, { direction, signs: 0, parking: true, startInLot: true })
      : F.randomLayout(seed, { direction, signs: 0, parking: true,
          startOnTrack: mode === 'entrada', startInLot: mode === 'salida' });
    const sim = new WRO.sim.Simulation(core, { field, seed, robot, mic: process.env.MIC === '1', ctrlConfig: process.env.EXACT === '1' ? 'park_exact_moves 1' : '' });
    const lot = field.lot, ccw = field.direction !== 'cw';
    const centre = (lot.a0 + lot.a1) / 2;
    const cmd = [], truth = [];
    const phases = [];
    let lastPhase = '';
    while (sim.t < seconds && !sim.finished) {
      sim.tick();
      const ph = sim.cmd ? sim.cmd.phase : '';
      if (ph !== lastPhase) {
        const jc = sim.judge && sim.judge.contacts ? sim.judge.contacts : { wall: 0, limiter: 0, sign: 0 };
        phases.push(`${ph}@${sim.t.toFixed(1)}(w${jc.wall}/l${jc.limiter}/s${jc.sign})`);
        lastPhase = ph;
      }      const wanted = mode === 'salida' ? 'PARK_EXIT' : 'PARKING';      if (ph !== wanted) continue;
      cmd.push(sim.cmd.speed);
      truth.push(ccw ? sim.robot.v : -sim.robot.v);   // avance positivo en el marco del cajon
      if (process.env.TRACE && sim.t - (globalThis.__lastPrint || 0) >= 0.5) {
        globalThis.__lastPrint = sim.t;
        const p = sim.robot.pose(), e = sim.cmd.park || {}, k = sim.core.parkPlan();
        const u = (ccw ? 1 : -1) * (p.x - centre), w = p.z - lot.wallCoord;
        const yawDeg = p.yaw * 180 / Math.PI;
        const th = ((ccw ? 90 - yawDeg : yawDeg + 90) + 540) % 360 - 180;
        let prev = 0, planRev = 0;
        for (const [d] of k.arcs) { if (prev && d !== prev) planRev++; prev = d; }
        console.log(`  t=${sim.t.toFixed(1)} ${ph} v=${sim.robot.v.toFixed(1)} verdad(u=${u.toFixed(1)},w=${w.toFixed(1)},th=${th.toFixed(0)}) estima(u=${(e.u || 0).toFixed(1)},w=${(e.w || 0).toFixed(1)},th=${(e.theta || 0).toFixed(0)}) etapa=${k.stage} plan=${k.found ? 'si' : 'no'} n=${k.count} i=${k.index} exp=${k.expanded} rev=${planRev}`);
      }
    }
    const r = sim.result();
    rows.push({ direction, seed, cmd: reversals(cmd), truth: reversals(truth), end: r.endReason,
                parking: r.parking, pts: r.points, t: +sim.t.toFixed(1),
                contacts: r.contacts.wall + r.contacts.limiter + r.contacts.sign, phases: phases.join(' ') });
    console.log(JSON.stringify(rows[rows.length - 1]));
  }
  return rows;
}

(async () => {
  core = await WRO.core.load(bytes);
  const profile = process.argv[2] || 'largo26';
  const mode = process.argv[3] || 'entrada';
  const seconds = Number(process.argv[4] || 90);
  const n = Number(process.argv[5] || 3);
  const seeds = [];
  if (process.env.SEED) seeds.push(Number(process.env.SEED));
  else for (let i = 1; i <= n; i++) seeds.push(i);
  if (profile === 'largo26' || profile === 'ackermann' || profile === 'differential17') {
    const rows = await run(profile, mode, seconds, seeds);
    if (process.env.MAX_REVERSALS !== undefined) {
      const max = Number(process.env.MAX_REVERSALS);
      if (!Number.isInteger(max) || max < 0 || mode !== 'ronda') throw new Error('MAX_REVERSALS requiere un entero no negativo y modo ronda');
      for (const r of rows) {
        if (r.end !== 'finished' || r.parking !== 'full' || r.pts !== 62 || r.contacts !== 0 || r.cmd > max || r.truth > max)
          throw new Error(`${r.direction} semilla ${r.seed}: resultado o repasadas fuera de limite (${JSON.stringify(r)})`);
      }
    }
    console.log(`${profile}/${mode}: repasadas cmd=[${rows.map(r => r.cmd).join(',')}] verdad=[${rows.map(r => r.truth).join(',')}]`);
  } else {
    console.error('perfil: largo26 | ackermann | differential17');
    process.exitCode = 2;
  }
})().catch(e => { console.error(e); process.exitCode = 1; });
