/* Por que IT no arranca en una ronda: node simulator/tools/it_diag.js semilla direccion [signs=5] [--cfg "it_enabled 1"] */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
(async () => {
  const [seed, direction, signs] = [Number(process.argv[2]), process.argv[3], Number(process.argv[4] || 5)];
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = W.field.randomLayout(seed, { direction, signs, parking: true, startInLot: true, feasible: true, allowGates: false });
  const robot = Object.assign({}, W.robot.PROFILES.largo26d, { speedTauS: 0.08, speedGain: 1 });
  const sim = new W.sim.Simulation(core, { field, seed, robot, ctrlConfig: 'it_enabled 1' });
  let shown = false;
  while (sim.t < 240 && !sim.finished) {
    sim.run(0.5);
    const c = sim.cmd; if (!c) continue;
    const st = core.itState(false);
    if (!shown && c.corners >= 5) { shown = true;
      const objs = core.traceObjects().filter(o => o.kind === 2 || o.kind === 3);
      console.log('esq', c.corners, 'IT:', JSON.stringify({ valid: st.valid, building: st.building, failed: st.failed, why: st.why, failWhy: st.why >> 8, n: st.obs.length }));
      console.log('verdad:', field.signs.map(s => `${s.color[0]}(${s.x.toFixed(0)},${s.z.toFixed(0)})`).join(' '));
      console.log('mapa  :', objs.map(o => `${['', 'p', 'P', 'b'][o.kind]}${['?', 'r', 'g', 'm'][o.color] || '?'}(${o.x.toFixed(0)},${o.z.toFixed(0)})`).join(' '));
    }
  }
  const st = core.itState(false); const r = sim.run(0.1);
  console.log('final', r.endReason, r.points, sim.t.toFixed(0), JSON.stringify({ valid: st.valid, failed: st.failed, failWhy: st.why >> 8, planTime: st.planTime, drops: st.drops, reason: st.dropReason }));
})();
