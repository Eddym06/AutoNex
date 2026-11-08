/* Linea de carrera: cuando se calcula, si sale valida, cuanto tiempo va activa y a que velocidad real se conduce en
 * cada vuelta, frente a la velocidad base.
 *   node simulator/tools/race_check.js [semilla] [ccw|cw] [--signs N] [--cfg "clave valor;clave valor"] [--plan archivo.json]
 * La velocidad "real" sale de la verdad del simulador (en el robot real saldria de los encoders). */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const F = WRO.field;
const args = process.argv.slice(2);
const flag = (name, def) => { const i = args.indexOf(name); return i >= 0 ? args[i + 1] : def; };
const pos = args.filter((a, i) => !a.startsWith('--') && !(i > 0 && args[i - 1].startsWith('--')));
const seed = Number(pos[0] || 21), dir = pos[1] || 'ccw';
const signs = Number(flag('--signs', 7));
const cfg = (flag('--cfg', '') || '').split(';').join('\n');
const plan = flag('--plan', null);

(async () => {
  const core = await WRO.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = plan ? F.parse(fs.readFileSync(plan, 'utf8')) : F.randomLayout(seed, { direction: dir, signs });
  const sim = new WRO.sim.Simulation(core, { field, seed, ctrlConfig: cfg });
  const lap = { 1: { t: 0, d: 0, act: 0, n: 0, vmax: 0 }, 2: { t: 0, d: 0, act: 0, n: 0, vmax: 0 }, 3: { t: 0, d: 0, act: 0, n: 0, vmax: 0 } };
  let built = null, last = null, lastLap = 1, tLap = 0;
  const lapTimes = {};
  while (sim.t < 200 && !sim.finished) {
    sim.run(0.1);
    const c = sim.cmd;
    if (!c || c.phase !== 'RUN') continue;
    const L = Math.min(3, Math.floor(c.corners / 4) + 1);
    if (L !== lastLap) { lapTimes[lastLap] = sim.t - tLap; tLap = sim.t; lastLap = L; }
    const p = sim.robot.pose();
    if (last) {
      lap[L].d += Math.hypot(p.x - last.x, p.z - last.z);
      lap[L].t += 0.1; lap[L].n++;
      if (c.race && c.race.active) lap[L].act++;
      lap[L].vmax = Math.max(lap[L].vmax, sim.robot.v);
    }
    last = p;
    if (args.includes('--trace') && L === Number(flag('--lap', 3)) && Math.round(sim.t * 10) % 5 === 0)
      console.log(`t=${sim.t.toFixed(1)} pos(${p.x.toFixed(0)},${p.z.toFixed(0)}) v=${sim.robot.v.toFixed(1)} cmd=${c.speed.toFixed(1)} line=${c.race.vLine.toFixed(1)} vCmd=${c.race.vCmd.toFixed(1)} act=${c.race.active ? 1 : 0} hold=${c.race.hold.toFixed(1)} pass=${c.pass.toFixed(0)} lat=${c.race.lat.toFixed(1)} dist=${c.race.dist.toFixed(1)} ref=[${c.race.ref.map(v => Number.isFinite(v) ? v.toFixed(0) : '-').join(',')}] esc=${c.escaping ? 1 : 0} steer=${c.steer.toFixed(0)} ${c.action}`);
    if (c.race && c.race.valid && !built) {
      built = { t: sim.t, r: c.race };
      console.log('reales: ' + field.signs.map(q => `${q.color}(${q.x.toFixed(0)},${q.z.toFixed(0)})`).join(' '));
      console.log('conocidos: ' + core.raceSigns().map(q => `${q.color === 1 ? 'red' : 'green'}(${q.x.toFixed(0)},${q.z.toFixed(0)})`).join(' '));
      console.log(`letreros reales ${field.signs.filter(q => q.color).length}, conocidos por la linea ${c.race.signs}`);
      console.log(`linea construida en t=${sim.t.toFixed(1)} s: ${c.race.n} estaciones, ${c.race.length.toFixed(0)} cm, vuelta estimada ${c.race.time.toFixed(1)} s (linea central ${c.race.timeCenter.toFixed(1)} s), mezcla ${c.race.mix}`);
    }
  }
  const r = sim.run(0.1);
  lapTimes[lastLap] = sim.t - tLap;
  if (!built) console.log(`la linea NO se construyo (tried=${sim.cmd && sim.cmd.race ? sim.cmd.race.tried : '?'})`);
  for (const L of [1, 2, 3]) {
    const l = lap[L];
    if (!l.n) continue;
    console.log(`vuelta ${L}: ${(l.t).toFixed(1)} s, ${l.d.toFixed(0)} cm recorridos, velocidad media ${(l.d / l.t).toFixed(1)} cm/s, maxima ${l.vmax.toFixed(1)}, linea activa ${(100 * l.act / l.n).toFixed(0)} % del tiempo`);
  }
  for (const v of r.violations) console.log('infraccion: ' + JSON.stringify(v));
  console.log(`fin: ${r.endReason}, ${r.points} puntos, ${r.sections} secciones, ${r.violations.length} infracciones, contactos ${r.contacts.sign + r.contacts.wall}, ${(r.elapsed == null ? sim.t : r.elapsed).toFixed(1)} s, parqueo ${r.parking}`);
})();
