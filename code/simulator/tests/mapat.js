/* Mapa del planificador en el instante T de una ronda: node simulator/tests/mapat.js '<signs>' T [ccw|cw] */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
(async () => {
  const core = await WRO.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const signs = JSON.parse(process.argv[2] || '[]'), T = Number(process.argv[3] || 20);
  const field = WRO.field.create({ signs, direction: process.argv[4] === 'cw' ? 'cw' : 'ccw', start: { along: 20 } });
  const sim = new WRO.sim.Simulation(core, { field, seed: 7 });
  while (sim.t < T && !sim.finished) sim.tick();
  const c = sim.cmd, p = sim.robot.pose();
  console.log(`t=${sim.t.toFixed(2)} pos=(${p.x.toFixed(1)},${p.z.toFixed(1)}) yaw=${(p.yaw*180/Math.PI).toFixed(1)} action=${c.action} pass=${c.pass} lane=${c.lane} esc=${c.escaping}/${c.escapePhase}`);
  const raw = (sim.snapshot.scan || []).filter(q => q.valid).map(q => { const a = q.angle * Math.PI / 180; return [q.distance * Math.sin(a), 12 + q.distance * Math.cos(a), q.angle]; }).filter(([x, f]) => f > 8 && f < 28 && Math.abs(x) < 22);
  console.log('raw lidar (x,fwd,angle):', raw.map(([x, f, a]) => `(${x.toFixed(1)},${f.toFixed(1)},${a.toFixed(1)})`).join(' '));
  const map = core.map(), N = core.mapSize(), CELL = core.mapCellCm(), dbg = core.debug();
  console.log('arcClear straight 20/40/60:', core.arcClear(0,20), core.arcClear(0,40), core.arcClear(0,60), ' nudge L/R 40:', core.arcClear(-4,40), core.arcClear(4,40));
  console.log('colored', JSON.stringify(dbg.colored), 'route', dbg.route.length, 'curtains', dbg.curtains);
  const occ = []; for (let fc = -6; fc <= 28; fc++) for (let xc = -28; xc <= 28; xc++) if (map[(N / 2 - fc) * N + N / 2 + xc] & 2) occ.push(`(${xc * CELL},${fc * CELL})`);
  console.log('occupied (x,fwd cm):', occ.join(' '));
  for (let fc = 32; fc >= -12; fc--) {
    let row = (fc * CELL + '').padStart(5) + ' ';
    for (let xc = -28; xc <= 28; xc++) {
      const v = map[(N / 2 - fc) * N + N / 2 + xc];
      row += (xc === 0 && fc === 0) ? 'R' : (v & 2) ? ((v & 4) ? 'V' : '#') : (v & 1) ? '.' : ' ';
    }
    console.log(row);
  }
})();
