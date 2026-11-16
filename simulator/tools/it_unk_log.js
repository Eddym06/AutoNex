/* Cesiones de IT por objeto desconocido (motivo 7): donde ocurren y que punto vio. node simulator/tools/it_unk_log.js semillas [signs=5] */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
(async () => {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const N = Number(process.argv[2] || 6), signs = Number(process.argv[3] || 5);
  for (const direction of ['ccw', 'cw']) for (let seed = 1; seed <= N; seed++) {
    const field = W.field.randomLayout(seed, { direction, signs, parking: true, startInLot: true, feasible: true, allowGates: false });
    const robot = Object.assign({}, W.robot.PROFILES.largo26d, { speedTauS: 0.08, speedGain: 1 });
    const sim = new W.sim.Simulation(core, { field, seed, robot, ctrlConfig: 'it_enabled 1' });
    let lastDrops = 0; const ev = [];
    while (sim.t < 120 && !sim.finished) {
      sim.run(0.05); const c = sim.cmd; if (!c || c.phase !== 'RUN') continue;
      const st = core.itState(false);
      if (st.drops > lastDrops) { lastDrops = st.drops; if (st.dropReason === 7) { const p = sim.robot.pose(); ev.push(`t${sim.t.toFixed(1)} pose(${p.x.toFixed(0)},${p.z.toFixed(0)}) v${sim.robot.v.toFixed(0)} punto(${st.unk.x.toFixed(0)},${st.unk.z.toFixed(0)}) n${st.unk.n}`); } }
    }
    console.log(direction + seed, 'verdad:', field.signs.map(s => `${s.color[0]}(${s.x.toFixed(0)},${s.z.toFixed(0)})`).join(' '), '|', ev.length ? ev.join(' ; ') : 'sin cesiones por desconocido');
  }
})();
