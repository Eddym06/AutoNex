/* Dinamica del carrito. Dos modelos, ambos con el EJE MOTRIZ como pose:
 *  - 'differential' (el robot del equipo): dos ruedas motrices atras y una rueda libre
 *    delante. Cada rueda tiene su motor con retardo, zona muerta, una ganancia de
 *    velocidad que el controlador NO conoce (no hay odometria) y un desajuste entre las
 *    dos (motores TT reales). Puede girar en el sitio.
 *  - 'bicycle': eje trasero motriz y volante delantero con servo (retardo, limite de
 *    velocidad de giro y desvio del centro).
 * Colisiona con paredes, topes y letreros; los letreros se empujan. */
(function (root) {
  'use strict';
  const WRO = root.WRO = root.WRO || {};
  const G = WRO.geometry || require('./geometry.js').geometry;
  const DEG = Math.PI / 180;
  const G9 = 981;                 /* gravedad (cm/s2) */

  /* Perfil provisional para el 12-móvil; no son medidas oficiales ni reales.
   * Todos los valores se reemplazarán después de medir el vehículo montado. */
  const DEFAULT_ROBOT = {
    drive: 'bicycle', wheelbase: 14, bodyWidth: 14, bodyLength: 22, rearOverhang: 4,
    trackWidth: 12, wheelTauS: 0.09, wheelDeadband: 2.5, wheelMismatch: 0.03, maxWheelSpeed: 60,
    maxSteerDeg: 40, steerRateDegS: 420, steerTauS: 0.045, steerTrimDeg: 0.8,
    speedTauS: 0.16, maxAccel: 180, speedGain: 0.94, pushSigns: true,
    /* Ruedas reales (los encoders miden el giro de la rueda, no el avance del suelo): diametro de cada una,
     * patinaje esporadico (rafagas en que una rueda gira de mas) y lo poco que gira una rueda contra un obstaculo. */
    wheelDiameterL: 6.6, wheelDiameterR: 6.6, wheelSlipPerMin: 1.0, wheelSlipFactor: 1.3, wheelSlipS: 0.25, contactSlip: 0.15
  };

  class Robot {
    constructor(config) {
      this.cfg = Object.assign({}, DEFAULT_ROBOT, config || {});
      this.rng = null;                        // lo fija la simulacion: sin el, no hay rafagas de patinaje
      this.reset({ x: 0, z: 0, yaw: 0 });
    }
    reset(pose) {
      this.steerQ = null; this.tNow = 0; this.dvy = 0; this.dr = 0; this.vLat = 0;
      this.x = pose.x; this.z = pose.z; this.yaw = pose.yaw;
      this.v = 0; this.steer = 0; this.yawRate = 0; this.wl = 0; this.wr = 0;
      this.cmd = { steer: 0, speed: 0 };
      this.encL = 0; this.encR = 0;           // angulo acumulado de cada rueda (grados)
      this.axleS = 0;                         // avance del eje motriz (cm) que ve el encoder del motor (MIC)
      this.slipL = 0; this.slipR = 0;         // segundos que le quedan a la rafaga de patinaje de cada rueda
      this.travelled = 0;
      this.contacts = [];
    }
    pose() { return { x: this.x, z: this.z, yaw: this.yaw }; }
    /* Motor fisico del JGB37-520 (R, Ke, masa, friccion, caida del L298N) gobernado por un controlador externo
     * (MIC). Los parametros reales son DISTINTOS de los que MIC supone: ese es el punto de la prueba. */
    enableMotor(drive, cfg) {
      const w_nl = 1000 * 10 * 2 * Math.PI / 60, r_w = 0.033, R0 = 2.6;
      const ke = (12 - 0.15 * R0) / w_nl;
      const k = Object.assign({ kR: 1.10, kMass: 1.08, kFric: 1.2, kKe: 0.98, kWheel: 1.002, vbat: 11.1, vdrop: 2.0 }, cfg || {});
      const refl = 10 / r_w;
      this.drive = drive;
      this.motor = {
        r: R0 * k.kR, keLin: ke * 10 / r_w * k.kKe, kfLin: ke * 10 * 0.75 / r_w * k.kKe,
        mass: (1.2 + 2e-6 * refl * refl) * k.kMass, fc: 0.6 * k.kFric, b: 0.5 * k.kFric,
        vbat: k.vbat, vdrop: k.vdrop, cmPerCount: Math.PI * 6.6 / 440 * k.kWheel
      };
    }
    motorStep(dt) {
      const m = this.motor, n = Math.max(1, Math.round(dt / 0.001)), h = dt / n;
      for (let i = 0; i < n; i++) {
        const counts = Math.floor(this.axleS / m.cmPerCount);
        const o = this.drive(counts, m.vbat, h);
        const vm = o.brakeShort ? 0 : o.duty * (m.vbat - m.vdrop);
        let v = this.v / 100;
        const cur = (vm - m.keLin * v) / m.r, f = m.kfLin * cur;
        const fr = m.fc * Math.tanh(v / 0.01) + m.b * v;
        let net = f - fr;
        if (Math.abs(v) < 1e-4 && Math.abs(f) <= m.fc) { net = 0; v = 0; }
        let vn = v + net / m.mass * h;
        if (v !== 0 && ((vn > 0) !== (v > 0)) && Math.abs(f) < m.fc) vn = 0;
        this.v = vn * 100;
        this.axleS += this.v * h;
        this.motorCurrent = cur;
      }
    }
    /* Bicicleta dinamica con neumaticos que saturan (por unidad de masa, cm y s). Estados: velocidad lateral del centro de masa y
     * giro. Convenio interno estandar (izquierda positiva); se devuelve el giro horario y la deriva lateral del eje trasero a
     * la derecha. A baja velocidad (o marcha atras) es cinematico. */
    dyn(dt, steerRight, ax) {
      const c = this.cfg, t = c.tire, L = c.wheelbase, v = this.v;
      const a = t.wr * L, b = L - a, k2 = (t.kz * L) * (t.kz * L);   /* a: centro de masa-eje delantero; b: -eje trasero */
      if (this.dvy === undefined) { this.dvy = 0; this.dr = 0; }
      if (v < t.vMin) { this.dvy = 0; this.dr = -v / L * Math.tan(steerRight); this.vLat = 0; return { yawRate: -this.dr, vLat: 0 }; }
      const delta = -steerRight, hL = t.h / L;
      const nf = Math.max(0.05 * G9, G9 * (1 - t.wr) - hL * ax), nr = Math.max(0.05 * G9, G9 * t.wr + hL * ax);
      /* El eje trasero reparte su agarre entre empujar y sujetarse de lado (circulo de friccion). */
      const lim = (mu, n, fx) => { const m = mu * n; return Math.sqrt(Math.max(0, m * m - fx * fx)); };
      const lf = t.mu * nf, lr = lim(t.mu, nr, Math.min(Math.abs(ax), t.mu * nr));
      const n = Math.max(1, Math.ceil(dt / 0.001)), h = dt / n;
      let vy = this.dvy, r = this.dr;
      for (let i = 0; i < n; i++) {
        const af = delta - Math.atan2(vy + a * r, v), ar = -Math.atan2(vy - b * r, v);
        const ff = lf * Math.tanh(t.kc * af * nf / Math.max(lf, 1e-6)), fr = lr * Math.tanh(t.kc * ar * nr / Math.max(lr, 1e-6));
        vy += ((ff * Math.cos(delta) + fr) - v * r) * h;
        r += ((a * ff * Math.cos(delta) - b * fr) / k2) * h;
      }
      this.dvy = vy; this.dr = r;
      this.slipAngleDeg = Math.atan2(vy - b * r, v) / DEG;     /* del eje trasero, izquierda positiva */
      return { yawRate: -r, vLat: -(vy - b * r) };
    }
    body(pose) {
      const p = pose || this, c = this.cfg, cx = c.bodyLength / 2 - c.rearOverhang;
      return { x: p.x + Math.sin(p.yaw) * cx, z: p.z + Math.cos(p.yaw) * cx, w: c.bodyWidth, d: c.bodyLength, rot: p.yaw, h: 10, kind: 'robot' };
    }
    /* Un paso de dt segundos. world = {solids: [...], signs: [...]}; devuelve los
     * contactos de este paso. */
    step(dt, world) {
      const c = this.cfg, events = [];
      let yawRate;
      if (c.drive === 'differential') {
        /* La orden es (velocidad del eje, giro). Sin giro explicito, el volante virtual del
         * planificador da la curvatura: w = v tan(volante) / batalla. */
        const v = this.cmd.speed;
        const w = Number.isFinite(this.cmd.yawRate) ? this.cmd.yawRate * DEG
          : v * Math.tan(Math.max(-c.maxSteerDeg, Math.min(c.maxSteerDeg, this.cmd.steer)) * DEG) / c.wheelbase;
        const half = c.trackWidth / 2;
        const targets = [v + w * half, v - w * half];            // izquierda, derecha
        const gains = [c.speedGain * (1 - c.wheelMismatch / 2), c.speedGain * (1 + c.wheelMismatch / 2)];
        const out = [this.wl, this.wr];
        for (let i = 0; i < 2; i++) {
          let t = Math.max(-c.maxWheelSpeed, Math.min(c.maxWheelSpeed, targets[i]));
          if (Math.abs(t) < c.wheelDeadband) t = 0;               // el motor no arranca con tan poca tension
          t *= gains[i];
          const dv = (t - out[i]) * (1 - Math.exp(-dt / c.wheelTauS));
          out[i] += Math.max(-c.maxAccel * dt, Math.min(c.maxAccel * dt, dv));
        }
        this.wl = out[0]; this.wr = out[1];
        this.v = (this.wl + this.wr) / 2;
        yawRate = (this.wl - this.wr) / c.trackWidth;
        this.steer = Math.abs(this.v) > 1 ? Math.atan(yawRate * c.wheelbase / this.v) / DEG : 0;
      } else {
        let target = Math.max(-c.maxSteerDeg, Math.min(c.maxSteerDeg, this.cmd.steer));
        /* Servo (MG90S): la orden llega con retardo (PWM de 50 Hz + electronica) y no se mueve por debajo de su zona muerta. */
        if (c.steerDelayS > 0) {
          this.tNow = (this.tNow || 0) + dt;
          const q = this.steerQ || (this.steerQ = [[-1e9, this.steer]]);
          q.push([this.tNow, target]);
          while (q.length > 1 && q[1][0] <= this.tNow - c.steerDelayS) q.shift();
          target = q[0][1];
        }
        const err = target - this.steer, maxStep = c.steerRateDegS * dt;
        if (!(c.steerDeadbandDeg > 0) || Math.abs(err) > c.steerDeadbandDeg)
          this.steer += Math.max(-maxStep, Math.min(maxStep, err * (1 - Math.exp(-dt / c.steerTauS))));
        const vPrev = this.v;
        if (this.drive) {
          this.axleBefore = this.axleS;
          this.motorStep(dt);
        } else {
        const vt = this.cmd.speed * c.speedGain;
        const dv = Math.max(-c.maxAccel * dt, Math.min(c.maxAccel * dt, (vt - this.v) * (1 - Math.exp(-dt / c.speedTauS))));
        this.v += dv;
        }
        const steerEff = (this.steer + c.steerTrimDeg) * DEG;
        if (c.tire) {
          /* Traccion trasera: el eje motriz solo agarra lo que le toca de peso, y al acelerar o frenar el peso se
           * traslada. La rueda gira de mas (el encoder lo ve) y el coche no acelera mas que esto. */
          const t = c.tire, L = c.wheelbase, hL = t.h / L, nr = G9 * t.wr;
          const capAcc = t.mu * nr / (1 - t.mu * hL), capDec = t.mu * nr / (1 + t.mu * hL);
          const ax = (this.v - vPrev) / dt;
          this.tractionLimited = false;
          if (this.v > 0 && ax > capAcc) { this.v = vPrev + capAcc * dt; this.tractionLimited = true; }
          else if (this.v > 0 && ax < -capDec) { this.v = vPrev - capDec * dt; this.tractionLimited = true; }
          const d = this.dyn(dt, steerEff, ax);
          yawRate = d.yawRate; this.vLat = d.vLat;
        } else {
          yawRate = this.v / c.wheelbase * Math.tan(steerEff);
          this.vLat = 0;
        }
      }
      /* Velocidad de la superficie de cada rueda (cm/s) para los encoders. */
      const halfTrack = c.trackWidth / 2;
      const wheelL = c.drive === 'differential' ? this.wl : this.v + yawRate * halfTrack;
      const wheelR = c.drive === 'differential' ? this.wr : this.v - yawRate * halfTrack;
      const mid = this.yaw + yawRate * dt / 2;
      const vl = this.vLat || 0;       /* deriva lateral del eje trasero (cm/s, derecha positiva): 0 sin modelo de neumaticos */
      const next = { x: this.x + (this.v * Math.sin(mid) + vl * Math.cos(mid)) * dt, z: this.z + (this.v * Math.cos(mid) - vl * Math.sin(mid)) * dt, yaw: this.yaw + yawRate * dt };
      const box = this.body(next);
      let blocked = null;
      for (const s of world.solids) {
        if (G.overlapBoxes(box, s)) { blocked = s; break; }
      }
      if (this.rng && c.wheelSlipPerMin > 0) {
        const p = c.wheelSlipPerMin / 60 * dt;
        if (this.slipL <= 0 && this.rng() < p) this.slipL = c.wheelSlipS;
        if (this.slipR <= 0 && this.rng() < p) this.slipR = c.wheelSlipS;
      }
      const turnL = (this.slipL > 0 ? c.wheelSlipFactor : 1) * (blocked ? c.contactSlip : 1);
      const turnR = (this.slipR > 0 ? c.wheelSlipFactor : 1) * (blocked ? c.contactSlip : 1);
      this.slipL -= dt; this.slipR -= dt;
      this.encL += wheelL * dt * turnL * 360 / (Math.PI * c.wheelDiameterL);
      this.encR += wheelR * dt * turnR * 360 / (Math.PI * c.wheelDiameterR);
      if (blocked) {
        if (this.drive) this.axleS = this.axleBefore;            // las ruedas no avanzan contra el obstaculo
        events.push({ kind: blocked.kind, box: blocked });
        /* La pared no se atraviesa, pero el chasis puede seguir haciendo algo: girar sobre
         * el sitio (diferencial) o deslizar por la cara con la que roza, quitando la
         * componente del avance que entra en ella. Antes se quedaba clavado de golpe, que
         * no es lo que hace el carrito y ademas le quitaba el giro al diferencial. */
        const free = q => !world.solids.some(s => G.overlapBoxes(this.body(q), s));
        let resolved = null;
        if (c.drive === 'differential' && Math.abs(yawRate) > 1e-9 && free({ x: this.x, z: this.z, yaw: next.yaw }))
          resolved = { x: this.x, z: this.z, yaw: next.yaw };
        const out = G.separation(blocked, box);        // saca el chasis del obstaculo
        if (out) {
          const L = Math.hypot(out[0], out[1]);
          const dx = next.x - this.x, dz = next.z - this.z;
          if (L > 1e-9) {
            const nx = out[0] / L, nz = out[1] / L, into = dx * nx + dz * nz;
            if (into < 0) {                            // el avance empuja contra la cara
              const slid = { x: this.x + dx - into * nx, z: this.z + dz - into * nz,
                             yaw: resolved ? resolved.yaw : this.yaw };
              if (free(slid)) resolved = slid;
            }
          }
        }
        const turned = resolved && resolved.yaw !== this.yaw;
        if (resolved) {
          const dx = resolved.x - this.x, dz = resolved.z - this.z;
          this.travelled += Math.hypot(dx, dz);
          this.x = resolved.x; this.z = resolved.z; this.yaw = resolved.yaw;
          this.v = (dx * Math.sin(this.yaw) + dz * Math.cos(this.yaw)) / dt;
        } else {
          this.v = 0;
        }
        this.yawRate = turned ? yawRate : 0;
      } else {
        this.travelled += Math.abs(this.v * dt);
        this.x = next.x; this.z = next.z; this.yaw = next.yaw;
        this.yawRate = yawRate;
      }
      for (const sign of world.signs) {
        const b = this.body();
        const push = G.separation(b, sign);
        if (!push) continue;
        events.push({ kind: 'sign', box: sign });
        if (c.pushSigns) { sign.x += push[0]; sign.z += push[1]; }
      }
      return events;
    }
  }

  /* Perfiles de chasis con nombre. Un plano puede declarar para cúal se dibujó (`robot` en
   * el JSON) y la simulación corre con ese, en vez de con el modelo por defecto. Un cajón
   * de 1,5 x 17 cm no es el de un robot de 22 cm: reproducir el plano con otro chasis deja
   * escenas que no tienen solución. */
  const PROFILES = {
    /* El vehiculo del equipo (perfil provisional, ver PERFIL_PROVISIONAL.md). */
    ackermann: DEFAULT_ROBOT,
    /* Chasis medido del equipo: 260 x 130 mm, batalla 184,5 mm, rueda de 66 mm. Voladizos
     * simetricos de 37,75 mm ((260 - 184,5) / 2). Es el mismo robot que `ackermann` en todo
     * lo demas; se separa para poder comparar y para que los planos puedan declararlo. */
    largo26: {
      drive: 'bicycle', wheelbase: 18.45, bodyWidth: 13, bodyLength: 26, rearOverhang: 3.775,
      wheelDiameterL: 6.6, wheelDiameterR: 6.6, maxSteerDeg: 40
    },
    /* Chasis anterior de 17 x 13 cm con traccion diferencial: los planos y las cifras
     * historicas se dibujaron para el. */
    /* El mismo chasis con dinamica: servo MG90S (60 grados en 0,10-0,12 s sin carga a 4,8 V; cargado ~420 grados/s; el PWM de 50 Hz
     * y la electronica suman ~20 ms; zona muerta ~0,5 grados) y neumaticos que saturan con el peso detras (62 % sobre el eje
     * trasero). TODO provisional: mu, reparto de peso y rigidez de curva se miden con el chasis montado (ver MEDICION_*). */
    largo26d: {
      drive: 'bicycle', wheelbase: 18.45, bodyWidth: 13, bodyLength: 26, rearOverhang: 3.775,
      wheelDiameterL: 6.6, wheelDiameterR: 6.6, maxSteerDeg: 40,
      steerRateDegS: 420, steerDelayS: 0.02, steerDeadbandDeg: 0.5,
      tire: { mu: 0.8, wr: 0.62, kc: 8, h: 4, kz: 0.38, vMin: 6 }
    },
    differential17: {
      drive: 'differential', wheelbase: 6, bodyWidth: 13, bodyLength: 17, rearOverhang: 7,
      trackWidth: 11.5, wheelDiameterL: 5.5, wheelDiameterR: 5.5
    }
  };

  WRO.robot = { Robot, DEFAULT_ROBOT, PROFILES };
  if (typeof module !== 'undefined') module.exports = WRO;
})(typeof window !== 'undefined' ? window : globalThis);
