'use strict';

// Solo prueba la salida del cajón: termina al entrar en RUN, sin correr la pista.
const assert = require('assert/strict');
const fs = require('fs');
const path = require('path');
const root = path.resolve(__dirname, '../..');
const W = require(path.join(root, 'simulator/web/sim.js'));
require(path.join(root, 'simulator/web/core.js'));

(async () => {
  const core = await W.core.load(fs.readFileSync(path.join(root, 'simulator/web/wro_core.wasm')));
  for (const direction of ['ccw', 'cw']) for (const seed of [1, 2, 3]) {
    const field = W.field.randomLayout(seed, { direction, signs: 0, parking: true, startInLot: true });
    const sim = new W.sim.Simulation(core, { field, seed });
    // Presupuesto provisional que incluye 1,5 s de espera inicial.
    while (sim.t < 60 && !sim.finished && (!sim.cmd || sim.cmd.phase !== 'RUN')) sim.tick();
    const result = sim.result();
    assert.equal(sim.cmd && sim.cmd.phase, 'RUN', `${direction}, semilla ${seed}: no salió en 60 s (${result.endReason})`);
    assert.equal(result.contacts.wall + result.contacts.limiter + result.contacts.sign, 0,
      `${direction}, semilla ${seed}: contacto durante la salida`);
    console.log(`PASS: salida ${direction}, semilla ${seed}, ${sim.t.toFixed(2)} s`);
  }
})().catch(err => { console.error(err); process.exitCode = 1; });
