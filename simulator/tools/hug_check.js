/* Pegado a la pared interior: cuanto tiempo va activo, desplazamiento pedido y tiempo por vuelta frente al robot centrado.
 *   node simulator/tools/hug_check.js [semilla] [ccw|cw] [--signs N] [--cfg "clave valor;clave valor"] [--trace]
 * Sin --cfg activa hug_enabled 1. La velocidad real sale de la verdad del simulador. */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const args = process.argv.slice(2);
const flag = (name, def) => { const i = args.indexOf(name); return i >= 0 ? args[i + 1] : def; };
const pos = args.filter((a, i) => !a.startsWith('--') && !(i > 0 && args[i - 1].startsWith('--')));
const seed = Number(pos[0] || 21), dir = pos[1] || 'ccw', signs = Number(flag('--signs', 7));
const cfg = (flag('--cfg', 'hug_enabled 1') || '').split(';').join('\n');

(async () => {
  const core = await WRO.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = WRO.field.randomLayout(seed, { direction: dir, signs });
  const sim = new WRO.sim.Simulation(core, { field, seed, ctrlConfig: cfg });
  const lap = {}; let last = null, maxOff = 0;
  while (sim.t < 200 && !sim.finished) {
    sim.run(0.1);
    const c = sim.cmd;
    if (!c || c.phase !== 'RUN') continue;
    const L = Math.min(3, Math.floor(c.corners / 4) + 1), p = sim.robot.pose();
    lap[L] = lap[L] || { t: 0, d: 0, act: 0, n: 0, off: 0 };
    if (last) { lap[L].d += Math.hypot(p.x - last.x, p.z - last.z); lap[L].t += 0.1; lap[L].n++; if (c.hug.active) lap[L].act++; lap[L].off += Math.abs(c.hug.cmd); }
    last = p; maxOff = Math.max(maxOff, Math.abs(c.hug.cmd));
    if (args.includes('--trace') && Math.round(sim.t * 10) % 5 === 0)
      console.log(`t=${sim.t.toFixed(1)} vuelta ${L} pos(${p.x.toFixed(0)},${p.z.toFixed(0)}) hug=${c.hug.active ? 'si' : 'no'} cmd=${c.hug.cmd.toFixed(1)} objetivo=${c.hug.target.toFixed(1)} espera=${c.hug.hold.toFixed(1)} ${c.action}`);
  }
  const r = sim.run(0.1);
  console.log('letreros: ' + field.signs.map(q => `${q.color}(${q.x.toFixed(0)},${q.z.toFixed(0)})`).join(' '));
  for (const L of [1, 2, 3]) {
    const l = lap[L];
    if (l && l.n) console.log(`vuelta ${L}: ${l.t.toFixed(1)} s, ${l.d.toFixed(0)} cm, hug activo ${(100 * l.act / l.n).toFixed(0)} % del tiempo, desplazamiento medio ${(l.off / l.n).toFixed(1)} cm`);
  }
  for (const v of r.violations) console.log('infraccion: ' + JSON.stringify(v));
  console.log(`fin: ${r.endReason}, ${r.points} puntos, ${r.sections} secciones, ${r.violations.length} infracciones, contactos ${r.contacts.sign + r.contacts.wall}, ${r.elapsed.toFixed(1)} s, parqueo ${r.parking}; desplazamiento maximo ${maxOff.toFixed(1)} cm`);
})();
