/* Tiempos por vuelta y de la ronda completa, para comparar el motor (rpm) y los topes de velocidad.
 *   node simulator/tools/lap_times.js [semillas=5] [sentidos=ccw,cw] [letreros=3]
 * Para cada variante imprime: duracion de la vuelta 1, 2 y 3, tiempo hasta cerrar la 3.a vuelta, ronda total (con parqueo),
 * puntos, contactos y velocidad maxima. Variantes: cinematica (sin MIC) y MIC con el motor fisico a 600 y a 800 rpm,
 * con el tope de IT por defecto (150 cm/s) y con el tope que da el motor (core.micVMax()). */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));

const seeds = (process.argv[2] || '5').split(',').map(Number);
const dirs = (process.argv[3] || 'ccw,cw').split(',');
const signs = Number(process.argv[4] || 3);

async function motorLimits(core, rpm) {
  const field = W.field.randomLayout(1, { direction: 'ccw', signs: 0, parking: true, startInLot: true });
  new W.sim.Simulation(core, { field, seed: 1, robot: Object.assign({}, W.robot.PROFILES.largo26d), mic: true, motor: { rpm } });
  const vmax = core.micVMax();
  return { vmax, a0: core.micMaxAccel(0), aMid: core.micMaxAccel(0.5 * vmax), aTop: core.micMaxAccel(0.9 * vmax) };
}

async function run(core, v, seed, direction) {
  const field = W.field.randomLayout(seed, { direction, signs, parking: true, startInLot: true });
  const robot = Object.assign({}, W.robot.PROFILES.largo26d, { speedTauS: 0.08, speedGain: 1 }, v.robot || {});
  const cfg = [v.it ? 'it_enabled 1' : '', v.cfg || ''].filter(Boolean).join('\n');
  const o = { field, seed, robot, ctrlConfig: cfg };
  if (v.mic) { o.mic = true; o.motor = { rpm: v.rpm }; }
  const sim = new W.sim.Simulation(core, o);
  const laps = [];
  let lastLap = 0, t0 = 0, vmax = 0, t3 = 0, wasRun = false;
  while (sim.t < 300 && !sim.finished) {
    sim.run(0.1);
    const c = sim.cmd;
    if (wasRun && c && c.phase !== 'RUN' && lastLap >= 1 && !laps[lastLap - 1]) laps[lastLap - 1] = sim.t - t0;
    if (!c || c.phase !== 'RUN') continue;
    wasRun = true;
    const lap = Math.floor(c.corners / 4) + 1;
    if (lap !== lastLap) { if (lastLap >= 1) laps[lastLap - 1] = sim.t - t0; if (lap === 4 && !t3) t3 = sim.t; t0 = sim.t; lastLap = lap; }
    vmax = Math.max(vmax, sim.robot.v);
  }
  const r = sim.run(0.1);
  return { laps, t3, total: sim.t, end: r.endReason, points: r.points, contacts: r.contacts.sign + r.contacts.wall, vmax };
}

(async () => {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const lim = {};
  for (const rpm of [600, 800]) lim[rpm] = await motorLimits(core, rpm);
  for (const rpm of [600, 800]) console.log(`modelo del motor ${rpm} rpm: v_max ${lim[rpm].vmax.toFixed(0)} cm/s | aceleracion max a 0 / 50 % / 90 % de v_max: ${lim[rpm].a0.toFixed(0)} / ${lim[rpm].aMid.toFixed(0)} / ${lim[rpm].aTop.toFixed(0)} cm/s2`);
  const FIJO = { robot: { maxAccel: 350, maxSpeed: Infinity }, cfg: 'it.v_max_cm_s 150\nit.a_motor_cm_s2 350' };   /* los valores sueltos de antes */
  const variants = [
    { name: 'sin IT (referencia)', it: false },
    { name: 'ANTES: cinematica, IT 150 / 350 fijos', it: true, robot: FIJO.robot, cfg: FIJO.cfg },
    { name: 'DESPUES: cinematica, limites 800 rpm', it: true },
    { name: 'MIC 600 rpm, limites del motor', it: true, mic: true, rpm: 600 },
    { name: 'MIC 800 rpm, limites del motor', it: true, mic: true, rpm: 800 },
  ];
  const f = x => (x ? x.toFixed(1) : '  - ');
  for (const v of variants) {
    const rows = [];
    for (const seed of seeds) for (const d of dirs) {
      const r = await run(core, v, seed, d);
      rows.push(r);
      console.log(`${v.name.padEnd(46)} ${d} ${seed}: v1 ${f(r.laps[0])} v2 ${f(r.laps[1])} v3 ${f(r.laps[2])} | 3 vueltas ${f(r.t3)} s | ronda ${r.total.toFixed(1)} s | ${r.end} ${r.points} pts, contactos ${r.contacts}, vmax ${r.vmax.toFixed(0)}`);
    }
  }
  process.exit(0);
})();
