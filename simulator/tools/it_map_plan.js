/* Planifica IT con una lista de obstaculos DADA (p.ej. la del mapa del robot: tools/user_case.js --plan guarda *_obs.json) y escribe la linea.
 *   node simulator/tools/it_map_plan.js obs.json [ccw|cw] [--grip 0.5] [--sweeps N] [--passes N] [--out linea.json] */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const fl = (n, d) => { const i = process.argv.indexOf(n); return i >= 0 ? process.argv[i + 1] : d; };
(async () => {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const obs = JSON.parse(fs.readFileSync(process.argv[2], 'utf8')).map(o => ({ x: o.x, z: o.z, color: o.color === 'red' ? 1 : o.color === 'green' ? 2 : 0 }));
  const dir = process.argv[3] && !process.argv[3].startsWith('--') ? process.argv[3] : 'ccw';
  const params = { grip: Number(fl('--grip', 0.5)), vMax: 188.7, aMotor: 300, sweeps: Number(fl('--sweeps', 0)), passes: Number(fl('--passes', 0)) };
  const t0 = Date.now(), p = core.itBuild(dir, params, obs);
  console.log(JSON.stringify({ ok: p.ok, why: p.why, time: +p.time.toFixed(3), evals: p.evals, ms: Date.now() - t0 }));
  if (fl('--out')) fs.writeFileSync(fl('--out'), JSON.stringify(p.line));
})();
