/* Orquestador del simulador: el mundo (pista, letreros, fisica del carrito, juez),
 * los sensores con sus retardos y el firmware real (WebAssembly). Cada decimo de
 * segundo el firmware recibe lo que recibiria en la pista -el barrido del LiDAR,
 * la trama del S3 con su retraso y los giroscopios a 100 Hz- y devuelve volante y
 * velocidad. Nada de lo que decide el firmware se calcula aqui. */
(function (root) {
  'use strict';
  const WRO = root.WRO = root.WRO || {};
  const req = n => (typeof require !== 'undefined' ? require('./' + n + '.js') : null);
  if (!WRO.geometry) req('geometry');
  if (!WRO.rng) req('rng');
  if (!WRO.field) req('field');
  if (!WRO.sensors) req('sensors');
  if (!WRO.robot) req('robot');
  if (!WRO.judge) req('judge');
  const G = WRO.geometry, F = WRO.field, S = WRO.sensors, R = WRO.robot, J = WRO.judge;

  /* Lo que el equipo sabe del carrito por defecto en el simulador: una camara
   * calibrada (altura y horizonte medidos) para que la distancia salga del suelo. */
  const DEFAULT_VISION_TEXT = [
    'use_ground_distance on', 'camera_height_cm 5', 'horizon_row 30', 'focal_px 118.6'
  ].join('\n');

  class Simulation {
    constructor(core, options) {
      const o = this.options = Object.assign({
        seed: 1, dt: 0.01, startDelay: 1.5, cameraFps: 30, procDelayMs: 8, cmdDelayMs: 12,
        visionConfig: DEFAULT_VISION_TEXT, ctrlConfig: '', lidar: {}, camera: {}, imu: {}, robot: {}, judge: {},
        lightDrift: 0.06, record: true
      }, options || {});
      this.core = core;
      this.field = o.field || F.create({});
      this.rng = WRO.rng(o.seed);
      this.dt = o.dt;
      this.camCfg = Object.assign({}, S.DEFAULT_CAMERA, o.camera);
      this.lidar = new S.Lidar(o.lidar, this.rng);
      this.gyro = new S.Gyro(o.imu, this.rng);
      /* Cada rueda real mide distinto del nominal (6,4 cm): el firmware lo ignora hasta calibrar. */
      const encRng = WRO.rng(o.seed * 7919 + 101);       // propio: no altera las secuencias del LiDAR, la camara ni el giroscopio
      const enc = Object.assign({}, S.DEFAULT_ENCODER, o.encoder || {});
      const dia = (o.robot && o.robot.wheelDiameterL) || R.DEFAULT_ROBOT.wheelDiameterL;
      const robotCfg = Object.assign({}, o.robot);
      if (o.robot == null || (o.robot.wheelDiameterL == null && o.robot.wheelDiameterR == null)) {
        robotCfg.wheelDiameterL = dia * (1 + encRng.range(-1, 1) * enc.diameterErrorPct / 100);
        robotCfg.wheelDiameterR = dia * (1 + encRng.range(-1, 1) * enc.diameterErrorPct / 100);
      }
      this.robot = new R.Robot(robotCfg);
      this.robot.rng = encRng;
      this.encoders = new S.Encoders(o.encoder, encRng);
      this.judge = new J.Judge(this.field, Object.assign({ wheelbase: this.robot.cfg.wheelbase }, o.judge));
      this.reset();
    }

    reset() {
      const o = this.options, core = this.core;
      this.t = 0;
      this.queue = [];
      this.nextCapture = 0;
      this.nextEncoder = 0;
      this.sequence = 0;
      this.started = false;
      this.cmd = null;
      this.lastCommandTime = 0;
      this.log = { poses: [], events: [], phases: [] };
      this.lastPhase = '';
      this.stats = { scans: 0, frames: 0, framesDropped: 0, framesRejected: 0, maxLatencyMs: 0 };
      this.field.signs.forEach(s => { s.x = s.seat.x; s.z = s.seat.z; });
      const start = F.startPose(this.field);
      this.start = start;                     // para llevar la odometria (marco del giroscopio) al del campo
      this.robot.reset(start);
      this.prev = this.robot.pose();
      this.lidar.reset(0);
      this.judge.reset();
      core.visionDefaults();
      let r = core.visionConfig(o.visionConfig);
      if (!r.ok) throw new Error('configuracion de vision: ' + r.error);
      core.ctrlDefaults();
      /* Lo que el equipo sabe de la sede: si hay cajon de parqueo y si el robot arranca dentro. */
      const venue = 'parking_enabled ' + (this.field.lot && o.parking !== false ? 1 : 0) + '\n' +
                    'start_in_lot ' + (this.field.lot && this.field.startInLot ? 1 : 0) + '\n' +
                    'park_demo_dir ' + (this.field.lot && this.field.startOnTrack ? (this.field.direction === 'cw' ? 1 : -1) : 0) + '\n';
      /* Despues de la configuracion del usuario: el cuadro de texto de la interfaz guarda el volcado
       * de los valores por defecto (`start_in_lot false`) y taparia lo que la pista dice. */
      /* El parqueo y el pivote del controlador se basan en girar en el sitio.
       * El encoder simulado sigue siendo doble: no presentarlo como el JGB real. */
      const ackermann = this.robot.cfg.drive === 'bicycle'
        ? '\npivot_enabled 0\nsidestep_enabled 0\npeek_enabled 0\nparking_enabled 0\nstart_in_lot 0\nodometry_enabled 0\n' : '';
      r = core.ctrlConfig((o.ctrlConfig || '') + '\n' + venue + ackermann);
      if (!r.ok) throw new Error('configuracion del controlador: ' + r.error);
      core.ctrlInit();
      core.cameraFocal(this.camCfg.focal);
      core.cameraPitch(S.camState(this.camCfg).pitch * 180 / Math.PI);
      this.snapshot = { scan: [], camera: null, vision: null, debug: null, map: null };
    }

    solids() {
      const f = this.field;
      return f.walls.concat(f.lot ? f.lot.limiters : []);
    }
    renderBoxes() {
      const f = this.field, lim = (f.lot ? f.lot.limiters : []).map(b => Object.assign({}, b, { kind: 'magenta' }));
      return f.walls.concat(f.signs, lim);
    }

    schedule(time, type, payload) {
      const e = { time, type, payload };
      let i = this.queue.length;
      while (i > 0 && this.queue[i - 1].time > time) i--;
      this.queue.splice(i, 0, e);
    }

    poseBetween(a, b, u) {
      let dyaw = G.wrapPi(b.yaw - a.yaw);
      return { x: a.x + (b.x - a.x) * u, z: a.z + (b.z - a.z) * u, yaw: a.yaw + dyaw * u };
    }

    note(text) {
      if (this.options.record) this.log.events.push({ t: +this.t.toFixed(3), text });
    }

    handle(e) {
      const core = this.core;
      if (e.type === 'scan') {
        const cmd = core.scan(e.payload.points, e.payload.tEnd * 1e6);
        this.stats.scans++;
        this.lastCommandTime = this.t;
        this.cmd = cmd;
        this.snapshot.scan = e.payload.points;
        this.snapshot.debug = core.debug();
        if (cmd.phase !== this.lastPhase) {
          this.note('fase ' + this.lastPhase + ' -> ' + cmd.phase);
          this.lastPhase = cmd.phase;
        }
        this.schedule(this.t + this.options.cmdDelayMs / 1000, 'cmd', { steer: cmd.steer, speed: cmd.speed, yawRate: cmd.yawRate });
      } else if (e.type === 'cmd') {
        this.robot.cmd = { steer: e.payload.steer, speed: e.payload.speed, yawRate: e.payload.yawRate };
      } else if (e.type === 'enc') {
        core.encoders(e.payload.t * 1e6, e.payload.left, e.payload.right);
      } else if (e.type === 'frame') {
        const ok = core.cameraFeed(e.payload.wire, this.t * 1e6);
        if (ok) this.stats.frames++; else this.stats.framesRejected++;
        const latency = (this.t - e.payload.captured) * 1000;
        if (latency > this.stats.maxLatencyMs) this.stats.maxLatencyMs = latency;
      }
    }

    capture() {
      const cfg = this.camCfg, t = this.t;
      const pose = this.poseBetween(this.prev, this.robot.pose(), 1);
      const light = 1 + this.options.lightDrift * Math.sin(2 * Math.PI * t / 17) + this.rng.gauss() * 0.01;
      const img = S.renderCamera(this.field, this.renderBoxes(), pose, cfg, this.rng, { yawRate: this.robot.yawRate, light });
      const out = this.core.cameraFrame(img.rgb565, this.sequence++ & 0xffff, Math.round(t * 1000) >>> 0, false);
      this.snapshot.camera = img.rgba;
      this.snapshot.vision = out.vision;
      let wire = out.wire;
      if (cfg.bitErrorRate > 0 && this.rng() < cfg.bitErrorRate) {
        wire = wire.slice();
        wire[Math.floor(this.rng() * wire.length)] ^= 1 << Math.floor(this.rng() * 8);
      }
      if (cfg.dropRate > 0 && this.rng() < cfg.dropRate) { this.stats.framesDropped++; return; }
      const latency = (cfg.latencyMs + this.rng.gauss() * cfg.latencyJitterMs) / 1000;
      this.schedule(t + Math.max(0.01, latency), 'frame', { wire, captured: t });
    }

    /* Un paso de fisica (dt). */
    tick() {
      const dt = this.dt, t0 = this.t, t1 = t0 + dt, core = this.core, o = this.options;
      this.t = t1;
      if (!this.started && t1 >= o.startDelay) {
        this.started = true;
        const calibrated = core.start(t1 * 1e6);
        this.judge.start(t1);
        this.note('arranque (giroscopio calibrado: ' + calibrated + ')');
      }
      /* Ordenes y tramas con hora <= ahora. */
      while (this.queue.length && this.queue[0].time <= t1) this.handle(this.queue.shift());
      this.prev = this.robot.pose();
      const events = this.robot.step(dt, { solids: this.solids(), signs: this.field.signs });
      const now = this.robot.pose();
      const poseAt = tt => this.poseBetween(this.prev, now, Math.max(0, Math.min(1, (tt - t0) / dt)));
      core.imu(t1 * 1e6, this.gyro.sample(this.robot.yawRate, dt), NaN);
      if (this.encoders.cfg.enabled) {
        while (this.nextEncoder <= t1) {
          this.nextEncoder += 1 / this.encoders.cfg.hz;
          if (this.encoders.rng() < this.encoders.cfg.dropRate) continue;
          const s = this.encoders.sample(this.robot);
          this.schedule(t1 + this.encoders.cfg.latencyMs / 1000, 'enc', { left: s.left, right: s.right, t: t1 });
        }
      }
      const rev = this.lidar.advance(t1, poseAt, this.renderLidarBoxes());
      if (rev) {
        this.schedule(rev.tEnd + o.procDelayMs / 1000, 'scan', { points: rev.points, tEnd: rev.tEnd });
      }
      while (this.nextCapture <= t1) {
        this.capture();
        this.nextCapture += 1 / o.cameraFps;
      }
      /* Perro guardian del P4: sin escaneos recientes, la orden pasa a parar. */
      if (this.started && this.cmd && t1 - this.lastCommandTime > 0.3) {
        const stop = core.watchdog(t1 * 1e6);
        if (stop) { this.cmd = stop; this.robot.cmd = { steer: 0, speed: 0 }; }
      }
      const body = this.robot.body();
      this.judge.update(t1, { x: now.x, z: now.z, yaw: now.yaw, v: this.robot.v }, body, events, !!(this.cmd && this.cmd.done));
      for (const e of events) if (e.kind === 'wall' || e.kind === 'limiter') this.note('contacto con ' + e.kind);
      if (o.record && Math.round(t1 * 100) % 5 === 0)
        this.log.poses.push([+t1.toFixed(2), +now.x.toFixed(1), +now.z.toFixed(1), +now.yaw.toFixed(3)]);
    }

    renderLidarBoxes() {
      /* Para el LiDAR basta lo que mide mas de su plano: paredes, letreros y topes. */
      if (!this._lidarBoxes) this._lidarBoxes = this.field.walls.concat(this.field.lot ? this.field.lot.limiters : []);
      return this._lidarBoxes.concat(this.field.signs);
    }

    get finished() { return !!this.judge.ended; }

    /* Avanza hasta `seconds` de simulacion o hasta que termine la ronda. */
    run(seconds) {
      const end = this.t + seconds;
      while (this.t < end && !this.finished) this.tick();
      return this.result();
    }
    runUntil(predicate, maxSeconds) {
      const end = this.t + maxSeconds;
      while (this.t < end && !this.finished && !predicate(this)) this.tick();
      return this.result();
    }

    result() {
      const rep = this.judge.report(this.robot.body());
      rep.t = this.t;
      rep.command = this.cmd;
      rep.travelled = this.robot.travelled;
      rep.stats = this.stats;
      return rep;
    }
  }

  WRO.sim = { Simulation, DEFAULT_VISION_TEXT };
  if (typeof module !== 'undefined') module.exports = WRO;
})(typeof window !== 'undefined' ? window : globalThis);
