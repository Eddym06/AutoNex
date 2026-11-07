/* Mapa local que ve el planificador en la ronda de un plano exportado, en el instante T.
 * node simulator/tests/dumpat.js plano.json T [alcance_cm=45] */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
(async () => {
  const core = await WRO.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const field = WRO.field.parse(fs.readFileSync(process.argv[2], 'utf8'));
  const T = Number(process.argv[3] || 20), reach = Number(process.argv[4] || 45);
  const sim = new WRO.sim.Simulation(core, { field, seed: field.seed || 1, ctrlConfig: (process.env.CTRL || '').split(';').join('\n') });
  while (sim.t < T && !sim.finished) sim.tick();
  const p = sim.robot.pose(), c = sim.cmd, d = core.debug();
  console.log(`t=${sim.t.toFixed(2)} pos=(${p.x.toFixed(1)},${p.z.toFixed(1)}) yaw=${WRO.geometry.wrapDeg(p.yaw * 180 / Math.PI).toFixed(0)} ${c.action} steer=${(c.steer || 0).toFixed(0)} lane=${c.lane} pass=${c.pass} curtains=${d.curtains}`);
  console.log('colored', JSON.stringify(d.colored.map(o => [o.color, +o.x.toFixed(1), +o.forward.toFixed(1), o.confirmed])));
  const n = core.core ? 0 : 0;
  const map = core.map(), N = Math.round(Math.sqrt(map.length)), cell = 2.5, C = N / 2;
  const cols = Math.round(reach / cell);
  for (let fr = Math.round(reach / cell); fr >= -Math.round(15 / cell); fr--) {
    let row = (Math.round(fr * cell) + '').padStart(4) + ' ';
    for (let xr = -cols; xr <= cols; xr++) {
      const v = map[(C - fr) * N + C + xr];
      row += (xr === 0 && fr === 0) ? 'R' : (v & 2) ? ((v & 4) ? 'V' : '#') : (v & 1) ? '.' : ' ';
    }
    console.log(row);
  }
})();
