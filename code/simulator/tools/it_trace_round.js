/* Marcas (fase, esquinas, IT activo, cedidas) de una ronda aleatoria: node simulator/tools/it_trace_round.js semilla direccion [signs=5] [--cfg "..."] */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
(async () => {
  const a = process.argv.slice(2).filter(x => !x.startsWith('--')), fl = (n, d) => { const i = process.argv.indexOf(n); return i >= 0 ? process.argv[i + 1] : d; };
  const [seed, direction, signs] = [Number(a[0]), a[1], Number(a[2] || 5)];
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = W.field.randomLayout(seed, { direction, signs, parking: true, startInLot: true, feasible: true, allowGates: false });
  const robot = Object.assign({}, W.robot.PROFILES.largo26d, { speedTauS: 0.08, speedGain: 1 });
  const sim = new W.sim.Simulation(core, { field, seed, robot, ctrlConfig: ('it_enabled 1\n' + (fl('--cfg', '') || '').split(';').join('\n')).trim() });
  let ph = '', corners = -1, act = null, drops = 0, lastPrint = 0;
  const out = [];
  while (sim.t < Number(fl('--limit', 200)) && !sim.finished) {
    sim.run(0.1);
    const c = sim.cmd; if (!c) continue;
    const st = core.itState(false);
    if (c.phase !== ph) { out.push(`${c.phase}@${sim.t.toFixed(1)}`); ph = c.phase; }
    if (c.corners !== corners) { out.push(`esq${c.corners}@${sim.t.toFixed(1)}`); corners = c.corners; }
    if (st.active !== act) { out.push(st.active ? `IT+@${sim.t.toFixed(1)}` : `IT-@${sim.t.toFixed(1)}(r${st.dropReason})`); act = st.active; }
    if (sim.t - lastPrint > 20) { lastPrint = sim.t; const p = sim.robot.pose(); out.push(`[t=${sim.t.toFixed(0)} (${p.x.toFixed(0)},${p.z.toFixed(0)}) v${sim.robot.v.toFixed(0)} e${st.e.toFixed(1)}]`); }
  }
  const r = sim.run(0.1);
  console.log(out.join(' '));
  console.log('fin', r.endReason, r.points, 'contactos', JSON.stringify(r.contacts));
})();
