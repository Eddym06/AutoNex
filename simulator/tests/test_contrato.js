/* Contrato entre la simulacion (JS) y el firmware (WASM): la ABI, la unica fuente de la
 * frecuencia de camara, un solo perro guardian por paso y el cajon a medida del chasis.
 * node simulator/tests/test_contrato.js */
const fs = require('node:fs'), path = require('node:path'), assert = require('node:assert/strict');
const web = path.join(__dirname, '../web');
const WRO = require(path.join(web, 'sim.js'));
require(path.join(web, 'core.js'));
const F = WRO.field;
const bytes = fs.readFileSync(path.join(web, 'wro_core.wasm'));
const near = (a, b, tol, msg) => assert(Math.abs(a - b) <= tol, `${msg || ''} ${a} != ${b} (+-${tol})`);

(async () => {
  /* --- ABI declarada por el C: la comprueba core.js al cargar; aqui se deja explicito --- */
  const core = await WRO.core.load(bytes);
  const raw = core.raw;
  assert.equal(raw.core_abi_version(), 2, 'version de la ABI');
  assert.equal(raw.core_cmd_fields(), 86);
  assert.equal(raw.core_vision_fields(), 8 + (12 + 1) * 9);
  assert.equal(raw.core_debug_fields(), 9 + 3 * 48 * 2 + 12 * 4);
  assert.equal(raw.core_max_points(), 512);
  /* Si el C declara otros tamaños, core.js aborta la carga con un mensaje claro. */

  /* --- Una sola fuente para la frecuencia de camara --- */
  const field = F.create({});
  const sim = new WRO.sim.Simulation(core, { field, camera: { fps: 20 } });
  near(sim.frameHz, 20, 1e-9, 'la frecuencia sale de camera.fps');
  sim.run(1.0);
  const received = sim.stats.frames + sim.stats.framesDropped;
  assert(received >= 18 && received <= 21, 'en un segundo llegan ~20 cuadros (' + received + ')');
  /* `cameraFps` ya no existe como opcion: pasarle basura no cambia nada ni revienta. */
  const sim2 = new WRO.sim.Simulation(core, { field, cameraFps: 'basura', camera: { fps: 10 } });
  near(sim2.frameHz, 10, 1e-9);
  assert.throws(() => new WRO.sim.Simulation(core, { field, camera: { fps: 0 } }), /frecuencia de camara/);

  /* --- Un solo perro guardian por paso --- */
  let guardCalls = 0;
  const counted = Object.assign({}, core, { watchdog(now) { guardCalls++; return core.watchdog(now); } });
  const sim3 = new WRO.sim.Simulation(counted, { field, seed: 3 });
  for (let i = 0; i < 300; i++) sim3.tick();
  assert(guardCalls > 0, 'el perro guardian se consulta');
  assert(guardCalls <= 300, 'como mucho una vez por paso (' + guardCalls + ' en 300 pasos)');

  /* --- El cajon y la pose de arranque siguen al chasis simulado --- */
  const scene = F.randomLayout(5, { direction: 'ccw', signs: 0, parking: true, startInLot: true });
  assert.equal(scene.parking.robotLength, null, 'el plano no fija el largo del robot');
  const legacy = { drive: 'differential', wheelbase: 6, bodyWidth: 13, bodyLength: 17, rearOverhang: 7 };
  const sim4 = new WRO.sim.Simulation(core, { field: scene, seed: 5, robot: legacy });
  near(sim4.field.lot.length, 25.5, 1e-9, 'cajon = 1,5 x 17 cm');
  const lot = sim4.field.lot, sp = sim4.start;
  near(sp.x, (lot.a0 + lot.a1) / 2 - (17 / 2 - 7), 1e-6, 'el cuerpo queda centrado entre los topes');
  assert.equal(sim4.judge.startSectionResult(sim4.robot.body()), true, 'arranca dentro del cajon');
  /* Un plano que trae sus medidas manda sobre el chasis. */
  const pinned = F.create({ parking: { section: 0, along: 0, robotLength: 30 } });
  near(pinned.lot.length, 45, 1e-9, 'cajon de 1,5 x 30 cm');
  WRO.field.fitToRobot(pinned, { length: 17, rear: 7 });
  near(pinned.lot.length, 45, 1e-9, 'fitToRobot no toca un plano con medidas propias');
  /* Ida y vuelta por JSON sin cambiar el plano. */
  const empty = F.randomLayout(7, { direction: 'cw', signs: 6 });
  assert.equal(F.toJSON(F.parse(F.toJSON(empty))), F.toJSON(empty));
  assert.equal(F.parse(F.toJSON(pinned)).lot.length, 45);

  /* --- Perfil medido del chasis (largo26): 260 x 130, batalla 184,5, rueda 66 --- */
  {
    const p = WRO.robot.PROFILES.largo26;
    assert(p, 'existe el perfil largo26');
    near(p.bodyLength, 26, 1e-9); near(p.bodyWidth, 13, 1e-9);
    near(p.wheelbase, 18.45, 1e-9); near(p.rearOverhang, 3.775, 1e-9);
    near(p.wheelDiameterL, 6.6, 1e-9); near(p.maxSteerDeg, 40, 1e-9);
    const core2 = await WRO.core.load(bytes);
    const field2 = F.randomLayout(1, { direction: 'ccw', signs: 0, parking: true, startInLot: true });
    const sim5 = new WRO.sim.Simulation(core2, { field: field2, seed: 1, robot: p });
    near(sim5.robot.cfg.bodyLength, 26, 1e-9, 'corre con el largo del perfil');
    near(sim5.field.lot.length, 39, 1e-9, 'cajon = 1,5 x 26 cm');
    const lot2 = sim5.field.lot, sp2 = sim5.start;
    near(sp2.x, (lot2.a0 + lot2.a1) / 2 - (26 / 2 - 3.775), 1e-6, 'cuerpo centrado entre los topes');
    assert.equal(sim5.judge.startSectionResult(sim5.robot.body()), true, 'arranca dentro del cajon de 39 cm');
    /* El cajon tambien se declara a la pista: el plano puede fijar el perfil. */
    const plan = F.parse(F.toJSON(Object.assign({}, field2, { robotProfile: 'largo26' })));
    assert.equal(plan.robotProfile, 'largo26', 'el perfil viaja en el JSON del plano');
  }

  console.log('PASS: ABI, fuente unica de la frecuencia de camara, un perro guardian por paso, cajon a medida del chasis y perfil medido largo26');
})().catch(e => { console.error(e.message || e); process.exit(1); });
