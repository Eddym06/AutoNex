/* Microbenchmarks de CPU, sin ejecutar rondas ni cargar WebAssembly.
 * Uso: node simulator/tools/profile_components.js */
const { performance } = require('node:perf_hooks');
const W = require('../web/sim.js');
const { field: F, sensors: S, robot: R } = W;
const field = F.randomLayout(17, { signs: 7, parking: false });
const rng = W.rng(17);
const boxes = field.walls.concat(field.signs);
const pose = { x: 0, z: -100, yaw: Math.PI / 2 };
const lidar = new S.Lidar({}, rng);
const robot = new R.Robot({});
robot.reset(pose);
robot.cmd = { speed: 20, steer: 10 };

function bench(name, n, fn) {
  for (let i = 0; i < 8; i++) fn();
  const start = performance.now();
  for (let i = 0; i < n; i++) fn();
  console.log(JSON.stringify({ name, iterations: n, ms_per_call: (performance.now() - start) / n }));
}

bench('camera', 100, () => S.renderCamera(field, boxes, pose, S.DEFAULT_CAMERA, rng, { yawRate: 0, light: 1 }));
bench('camera_sin_ruido', 100, () => S.renderCamera(field, boxes, pose, { ...S.DEFAULT_CAMERA, noiseSigma: 0 }, rng, { yawRate: 0, light: 1 }));
bench('camera_sin_lineas', 100, () => S.renderCamera({ ...field, lines: [] }, boxes, pose, S.DEFAULT_CAMERA, rng, { yawRate: 0, light: 1 }));
bench('lidar_400_rayos', 100, () => { for (let i = 0; i < 400; i++) lidar.measure(i, pose, boxes); });
bench('fisica_paso', 10000, () => robot.step(0.01, { solids: field.walls, signs: field.signs }));
bench('avisos_plano', 3000, () => F.checkLayout(field));
