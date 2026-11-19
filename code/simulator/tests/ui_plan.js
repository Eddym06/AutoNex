/* Carga la interfaz con un DOM de mentira y la hace dibujar varios cuadros: detecta
 * errores de arranque, ids que faltan y excepciones de dibujo sin necesitar un
 * navegador. node simulator/tests/test_ui_smoke.js */
const fs = require('node:fs'), path = require('node:path'), vm = require('node:vm'), assert = require('node:assert/strict');
const root = path.join(__dirname, '..');
const html = fs.readFileSync(path.join(root, 'web/index.template.html'), 'utf8');
const ids = [...html.matchAll(/id="([^"]+)"/g)].map(m => m[1]);

const ctx2d = new Proxy({}, { get: (t, k) => k === 'getImageData' ? () => ({ data: new Uint8ClampedArray(4) }) : (typeof k === 'string' ? (() => {}) : undefined), set: () => true });
function element(id) {
  const el = { id, value: '', checked: true, style: {}, dataset: {}, textContent: '', innerHTML: '', width: 760, height: 760, children: [],
    getContext: () => ctx2d, add(o) { this.children.push(o); }, querySelector: () => element(id + '-q'), getBoundingClientRect: () => ({ left: 0, top: 0, width: 760, height: 760 }) };
  if (id === 'speed') el.value = '1';
  if (id === 'seed') el.value = '3';
  if (id === 'signCount') el.value = '6';
  if (id === 'direction') el.value = 'ccw';
  if (id === 'preset') el.value = 'random';
  for (const n of ['redSide']) if (id === n) el.value = '1';
  if (id === 'greenSide') el.value = '-1';
  if (id === 'rSpeed') el.value = '20';
  for (const [k, v] of Object.entries({ rLidarNoise: 1, rCamNoise: 1, rSpeedGain: 0.94, rTrim: 0.8, rLight: 1, rLatency: 45, rBitErr: 0 })) if (id === k) el.value = String(v);
  if (id === 'world') el.width = 760;
  if (id === 'camera') { el.width = 160; el.height = 120; }
  if (id === 'local') { el.width = 300; el.height = 300; }
  return el;
}
const cache = {};
const document = { getElementById: id => cache[id] || (cache[id] = element(id)), execCommand: () => true };
let frames = 0, pending = null;
const sandbox = { window: null, document, console, atob: s => Buffer.from(s, 'base64').toString('binary'), performance: { now: () => Date.now() },
  requestAnimationFrame: f => { pending = f; }, ImageData: class { constructor(d, w, h) { this.data = d; this.width = w; this.height = h; } },
  Option: class { constructor(t, v) { this.text = t; this.value = v; } },
  Uint8Array, Uint8ClampedArray, Float32Array, Math, JSON, Number, Object, Array, TextEncoder, TextDecoder, WebAssembly, Promise, setTimeout };
sandbox.window = sandbox; sandbox.globalThis = sandbox;
vm.createContext(sandbox);
const wasm = fs.readFileSync(path.join(root, 'web/wro_core.wasm')).toString('base64');
sandbox.WASM_B64 = wasm;
for (const f of ['geometry', 'rng', 'field', 'sensors', 'robot', 'judge', 'core', 'sim', 'app'])
  vm.runInContext(fs.readFileSync(path.join(root, `web/${f}.js`), 'utf8').replace(/typeof module !== 'undefined'/g, 'false'), sandbox, { filename: f + '.js' });

(async () => {
  for (let i = 0; i < 200 && !sandbox.WROSim; i++) await new Promise(r => setTimeout(r, 10));
  assert(sandbox.WROSim, 'la interfaz no arranco');
  for (const id of ids) assert(document.getElementById(id), id);
  /* El plano del usuario por el camino de la interfaz: Plano <-> JSON, Aplicar, Simular. */
  cache.ioText.value = fs.readFileSync(process.argv[2], 'utf8');
  cache.ioApply.onclick();
  console.log('estado:', cache.ioStatus.textContent);
  cache.speed.value = '4';
  cache.play.onclick();
  let lastT = -1;
  for (let i = 0; i < 4000; i++) {
    const f = pending; pending = null; if (!f) break; f(i * 33);
    const sim = sandbox.WROSim.sim;
    if (sim && sim.t - lastT >= 1 && sim.cmd) { lastT = sim.t; const p = sim.robot.pose(); console.log('t=' + sim.t.toFixed(1), sim.cmd.phase, sim.cmd.action, p.x.toFixed(1), p.z.toFixed(1)); }
    if (sim && sim.t > 14) break;
  }
  const sim = sandbox.WROSim.sim;
  console.log('fin', sim.finished, JSON.stringify(sim.result().violations.map(v => v.kind)), 'lot', !!sim.field.lot, 'startInLot', sim.field.startInLot);
  process.exit(0);
})().catch(e => { console.error(e); process.exit(1); });
