/* Entrada al cajon: cuenta las repasadas (cambios de sentido) que cuesta aparcar desde la pista
 * y las acota. Es la medida que faltaba: la salida esta medida en 8-9 s y 1 repasada, pero la
 * entrada se movia entre 3 y 8 sin que ninguna prueba lo notara.
 *   node simulator/tests/test_parking_entry.js
 * Para medir a mano (con traza por barrido): simulator/tests/parkentry.js */
const assert = require('assert/strict');
const fs = require('fs');
const path = require('path');
const root = path.resolve(__dirname, '../..');
const W = require(path.join(root, 'simulator/web/sim.js'));
require(path.join(root, 'simulator/web/core.js'));
const F = W.field;

/* Una repasada es un cambio de sentido con movimiento: se ignoran las ordenes de 0 y las
 * vibraciones que el motor no llega a ejecutar. */
function reversals(seq) {
  let n = 0, last = 0;
  for (const s of seq) {
    if (Math.abs(s) < 2.0) continue;
    const sign = s > 0 ? 1 : -1;
    if (last && sign !== last) n++;
    last = sign;
  }
  return n;
}

(async () => {
  const core = await W.core.load(fs.readFileSync(path.join(root, 'simulator/web/wro_core.wasm')));
  for (const direction of ['ccw', 'cw']) {
    /* Lo mismo que la demostracion de la interfaz: arranca en el carril y solo aparca. */
    const field = F.randomLayout(1, { direction, signs: 0, parking: true, startOnTrack: true });
    const sim = new W.sim.Simulation(core, { field, seed: 1, robot: W.robot.PROFILES.largo26 });
    const cmd = [];
    while (sim.t < 45 && !sim.finished) {
      sim.tick();
      if (sim.cmd && sim.cmd.phase === 'PARKING') cmd.push(sim.cmd.speed);
    }
    const r = sim.result();
    const rep = reversals(cmd);
    assert.equal(sim.finished, true, `${direction}: termina la maniobra`);
    assert.equal(r.parking, 'full', `${direction}: aparcado dentro y en paralelo`);
    assert.equal(r.contacts.wall + r.contacts.limiter + r.contacts.sign, 0, `${direction}: sin contactos`);
    assert(rep <= 4, `${direction}: la entrada gasta ${rep} repasadas (maximo acordado: 4)`);
    console.log(`ok  entrada ${direction}: ${rep} repasadas, ${r.points} puntos`);
  }
  console.log('PASS: entrada al cajon medida y acotada');
})().catch(e => { console.error(e.message || e); process.exit(1); });
