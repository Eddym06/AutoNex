/* Valida el planificador IT sobre muchas pistas aleatorias (sin simular): cuantas salen validas, ganancia sobre la linea central y tiempo.
 *   node simulator/tools/it_plan_check.js [pistas=60] [--signs 3] [--grip 0.7] [--vmax 150] */
'use strict';
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web'), W = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
(async () => {
  const a = process.argv.slice(2), fl = (n, d) => { const i = a.indexOf(n); return i >= 0 ? Number(a[i + 1]) : d; };
  const N = a.length && !a[0].startsWith('--') ? Number(a[0]) : 60, signs = fl('--signs', 3), grip = fl('--grip', 0.7), vMax = fl('--vmax', 150);
  const core = await W.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  let ok = 0, bad = [], gain = [], ms = [], tl = [];
  for (const direction of ['ccw', 'cw']) for (let seed = 1; seed <= N; seed++) {
    const field = W.field.randomLayout(seed, { direction, signs, parking: false });
    const obs = field.signs.map(q => ({ x: q.x, z: q.z, color: q.color === 'red' ? 1 : 2 }));
    const t0 = Date.now(), plan = core.itBuild(direction, { grip, vMax }, obs);
    ms.push(Date.now() - t0);
    if (plan.ok) { ok++; gain.push(1 - plan.time / plan.timeCenter); tl.push(plan.time); }
    else bad.push(`${direction}${seed}(viol ${plan.violation.toFixed(2)} why ${plan.why}) ` + obs.map(o => `${o.color}(${o.x.toFixed(0)},${o.z.toFixed(0)})`).join(' '));
  }
  const m = (v) => v.reduce((s, x) => s + x, 0) / Math.max(1, v.length);
  console.log(`validas ${ok} de ${2 * N}; ganancia media sobre linea central ${(100 * m(gain)).toFixed(0)} %; vuelta planificada ${m(tl).toFixed(2)} s; construccion ${m(ms).toFixed(0)} ms (max ${Math.max(...ms)})`);
  bad.slice(0, 12).forEach(b => console.log('  invalida: ' + b));
})();
