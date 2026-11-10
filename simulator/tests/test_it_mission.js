/* Trayectoria inteligente (IT) dentro de la mision completa: el controlador la activa al cerrar la 1.a vuelta, conduce con el lazo rapido
 * (encoders + giroscopio), no toca nada, cede el mando donde esta previsto (dos esquinas antes del final) y el parqueo termina con todos los puntos.
 *   node simulator/tests/test_it_mission.js
 * Rondas con letreros y cajon de parqueo, chasis con dinamica (largo26d). Se compara con la misma ronda sin IT. */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
let fails = 0;
const ok = (c, m) => { if (!c) { fails++; console.log('FALLA:', m); } else console.log('ok   :', m); };

async function run(core, seed, direction, it) {
  const field = W.field.randomLayout(seed, { direction, signs: 3, parking: true, startInLot: true });
  const robot = Object.assign({}, W.robot.PROFILES.largo26d, { maxAccel: 350, speedTauS: 0.08, speedGain: 1 });
  const sim = new W.sim.Simulation(core, { field, seed, robot, ctrlConfig: it ? 'it_enabled 1' : '' });
  let vmax = 0, active = 0, ticks = 0, drops = 0, valid = false, lap2 = 0, lastLap = 0, t0 = 0;
  while (sim.t < 240 && !sim.finished) {
    sim.run(0.1);
    const c = sim.cmd;
    if (!c || c.phase !== 'RUN') continue;
    const lap = Math.floor(c.corners / 4) + 1;
    if (lap !== lastLap) { if (lastLap === 2) lap2 = sim.t - t0; t0 = sim.t; lastLap = lap; }
    vmax = Math.max(vmax, sim.robot.v);
    if (it) { const s = core.itState(false); ticks++; if (s.active) active++; drops = s.drops; valid = valid || s.valid; }
  }
  const r = sim.run(0.1);
  return { end: r.endReason, points: r.points, t: sim.t, contacts: r.contacts.sign + r.contacts.wall, vmax, active: ticks ? active / ticks : 0, drops, valid, lap2 };
}

(async () => {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  for (const [seed, dir] of [[5, 'ccw'], [5, 'cw']]) {
    const base = await run(core, seed, dir, false), it = await run(core, seed, dir, true);
    console.log(`  ${dir} ${seed}: sin IT ${base.end} ${base.points} pts ${base.t.toFixed(0)} s | con IT ${it.end} ${it.points} pts ${it.t.toFixed(0)} s, vuelta 2 ${it.lap2.toFixed(1)} s, v max ${it.vmax.toFixed(0)}, activo ${(100 * it.active).toFixed(0)} %, cedio ${it.drops}`);
    ok(it.valid, `${dir} ${seed}: el plan de IT es valido`);
    ok(it.active > 0.05, `${dir} ${seed}: IT conduce parte de la ronda`);
    ok(it.end === 'finished' && it.points === base.points, `${dir} ${seed}: termina con los mismos puntos que sin IT (${it.points})`);
    ok(it.contacts === 0, `${dir} ${seed}: sin contactos con letreros ni paredes`);
    ok(it.drops === 1, `${dir} ${seed}: cede el mando una sola vez, donde esta previsto (${it.drops})`);
    ok(it.vmax > 100, `${dir} ${seed}: llega a mas de 100 cm/s (${it.vmax.toFixed(0)})`);
    ok(it.t < base.t * 0.85, `${dir} ${seed}: la ronda dura menos de 0,85 veces la de sin IT (${it.t.toFixed(0)} frente a ${base.t.toFixed(0)} s)`);
  }
  console.log(fails ? `FALLAN ${fails}` : 'PASS: IT dentro de la mision (plan, lazo rapido, sin contactos, entrega al parqueo)');
  process.exit(fails ? 1 : 0);
})();
