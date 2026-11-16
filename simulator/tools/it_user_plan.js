/* Planifica IT para un plano JSON y escribe la linea; compara configuraciones de busqueda.
 *   node simulator/tools/it_user_plan.js plano.json [--sweeps N --passes N --out linea.json] */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const fl = (n, d) => { const i = process.argv.indexOf(n); return i >= 0 ? process.argv[i + 1] : d; };
(async () => {
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = W.field.parse(fs.readFileSync(process.argv[2], 'utf8'));
  const obs = field.signs.map(q => ({ x: q.x, z: q.z, color: q.color === 'red' ? 1 : 2 }));
  const params = { grip: Number(fl('--grip', 0.7)), sweeps: Number(fl('--sweeps', 0)), passes: Number(fl('--passes', 0)), trackErr: Number(fl('--trackerr', 0)) };
  const t0 = Date.now(), plan = core.itBuild(field.direction, params, obs);
  console.log(JSON.stringify({ ok: plan.ok, why: plan.why, time: +plan.time.toFixed(3), timeCenter: +plan.timeCenter.toFixed(3), evals: plan.evals, length: +plan.length.toFixed(1), ms: Date.now() - t0,
    vmin: Math.min(...plan.line.map(l => l.v)).toFixed(0) }));
  if (fl('--out')) fs.writeFileSync(fl('--out'), JSON.stringify(plan.line));
})();
