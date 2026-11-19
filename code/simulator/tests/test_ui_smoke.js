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
  if (id === 'startInLot' || id === 'startOnTrack') el.checked = false;
  for (const n of ['redSide']) if (id === n) el.value = '1';
  if (id === 'greenSide') el.value = '-1';
  if (id === 'rSpeed') el.value = '20';
  for (const [k, v] of Object.entries({ rLidarNoise: 1, rCamNoise: 1, rSpeedGain: 0.94, rTrim: 0.8, rLight: 1, rLatency: 45, rCamFps: 30, rBitErr: 0 })) if (id === k) el.value = String(v);
  if (id === 'world') el.width = 760;
  if (id === 'camera') { el.width = 160; el.height = 120; }
  if (id === 'local') { el.width = 300; el.height = 300; }
  return el;
}
const cache = {};
const idSet = new Set(ids);
const document = { getElementById: id => idSet.has(id) ? (cache[id] || (cache[id] = element(id))) : null, execCommand: () => true };
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
  /* Pulsa "Simular" y deja correr unos cuadros. */
  cache.play.onclick();
  for (let i = 0; i < 40; i++) { const f = pending; pending = null; assert(f, 'se detuvo el bucle de dibujo'); f(i * 33); }
  const sim = sandbox.WROSim.sim;
  assert(sim && sim.t > 0.5, 'la simulacion no avanza');
  assert(cache.telemetry.innerHTML.includes('Fase'), 'sin telemetria');
  assert(cache.judge.innerHTML.includes('Secciones'), 'sin panel del juez');
  /* Con la pose del localizador ya adquirida y la odometria en marcha, el panel las muestra con su error y el
   * mapa dibuja la pose estimada sin fallar. */
  sim.run(8);
  for (let i = 0; i < 6; i++) { const f = pending; pending = null; assert(f, 'se detuvo el bucle de dibujo'); f(2000 + i * 33); }
  assert(sim.cmd && sim.cmd.arena && sim.cmd.arena.valid, 'el localizador no adquirio pose');
  assert(/\d+ celdas LiDAR/.test(cache.traceInfo.textContent), 'el mapa persistente no muestra sus datos');
  assert(/candidato|evidencia/i.test(cache.traceParkingInfo.textContent), 'falta el diagnóstico experimental');
  assert(cache.telemetry.innerHTML.includes('Odometría (encoders)') && cache.telemetry.innerHTML.includes('Pose en el campo (LiDAR)'), 'faltan las filas de odometria y pose');
  assert(/error [0-9.]+ cm/.test(cache.telemetry.innerHTML), 'sin el error contra la verdad');
  /* El panel debe mostrar la linea aunque la mision aun no la haya calculado. */
  for (let i = 0; i < 6; i++) { const f = pending; pending = null; assert(f, 'se detuvo el bucle de dibujo'); f(5000 + i * 33); }
  assert(cache.telemetry.innerHTML.includes('Línea de carrera'), 'falta la fila de la linea de carrera');
  if (sim.cmd.race && sim.cmd.race.valid) {
    assert(cache.telemetry.innerHTML.includes('estaciones'), 'la linea existe pero el panel no la describe');
    assert(sandbox.WROSim.sim.racePts && sandbox.WROSim.sim.racePts.length > 50, 'no se leyo la linea de carrera');
  }
  /* Trayectoria inteligente (la casilla esta marcada en el DOM de mentira): tras la 1.a vuelta el panel la describe y el mapa la dibuja sin fallar. */
  assert(cache.telemetry.innerHTML.includes('Trayectoria inteligente'), 'falta la fila de la trayectoria inteligente');
  sim.run(75);
  for (let i = 0; i < 6; i++) { const f = pending; pending = null; assert(f, 'se detuvo el bucle de dibujo'); f(9000 + i * 33); }
  const it = sim.core.itState(false);
  assert(it.enabled, 'IT no esta activada en el controlador');
  assert(it.valid || it.failed || it.building, 'IT no intento planear tras la 1.a vuelta');
  if (it.valid) assert(/estaciones/.test(cache.telemetry.innerHTML) && /vuelta estimada/.test(cache.telemetry.innerHTML), 'el panel no describe el plan de IT');
  console.log('PASS: la interfaz arranca, simula ' + sim.t.toFixed(1) + ' s y dibuja sin errores (' + ids.length + ' elementos)');
  process.exit(0);
})().catch(e => { console.error(e); process.exit(1); });
