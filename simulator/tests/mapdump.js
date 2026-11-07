/* Muestra el mapa local que ve el planificador en una pose dada.
 * node simulator/tests/mapdump.js '<signs json>' x z yawDeg [ccw|cw] */
const fs = require('node:fs'), path = require('node:path');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
(async () => {
  const core = await WRO.core.load(fs.readFileSync(path.join(web, 'wro_core.wasm')));
  const signs = JSON.parse(process.argv[2] || '[]');
  const x = Number(process.argv[3]), z = Number(process.argv[4]), yaw = Number(process.argv[5]) * Math.PI / 180;
  const field = WRO.field.create({ signs, direction: process.argv[6] === 'cw' ? 'cw' : 'ccw' });
  const sim = new WRO.sim.Simulation(core, { field, seed: 1, lidar: { sigmaNearCm: 0, sigmaFarCm: 0, dropout: 0, angleJitterDeg: 0 }, camera: { noiseSigma: 0 } });
  sim.robot.reset({ x, z, yaw });
  /* un par de segundos quieto, para que el controlador calibre el giroscopio y arranque */
  for (let i = 0; i < 400 && !sim.cmd; i++) sim.tick();
  const pts = [];
  for (let i = 0; i < 400; i++) pts.push(sim.lidar.measure(i, sim.robot.pose(), sim.renderLidarBoxes()));
  const cmd = core.scan(pts, sim.t * 1e6 + 1000);
  const map = core.map(), dbg = core.debug();
  console.log('action', cmd.action, 'pass', cmd.pass, 'lane', cmd.lane, 'curtains', dbg.curtains, 'colored', JSON.stringify(dbg.colored));
  for (let fc = 14; fc >= -6; fc--) {
    let row = (fc * 5 + '').padStart(4) + ' ';
    for (let xc = -14; xc <= 14; xc++) {
      const v = map[(50 - fc) * 100 + 50 + xc];
      row += (xc === 0 && fc === 0) ? 'R' : (v & 2) ? ((v & 4) ? 'V' : '#') : (v & 1) ? '.' : ' ';
    }
    console.log(row);
  }
})();
