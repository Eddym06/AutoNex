/* Sensores simulados. Ninguno conoce la verdad del mundo mas alla de lo que el
 * sensor fisico mediria: el LiDAR devuelve distancias con ruido, huecos y
 * deformacion por movimiento; la camara devuelve pixeles; el giroscopio, una
 * velocidad angular con sesgo y deriva. */
(function (root) {
  'use strict';
  const WRO = root.WRO = root.WRO || {};
  const G = WRO.geometry || require('./geometry.js').geometry;
  const FIELD = WRO.field || require('./field.js').field;
  const DEG = Math.PI / 180;

  /* ------------------------------------------------------------------- LiDAR
   * WitMotion D6 (segun su ficha): vuelta completa a 10 Hz, 0,9 grados de
   * resolucion, rango 5 a 1200 cm. El barrido es CONTINUO: cada rayo se mide en su
   * instante, con el robot ya movido, asi que un giro deforma las paredes. */
  const DEFAULT_LIDAR = {
    mountForward: 3, mountLateral: 0, minCm: 5, maxCm: 1200, hz: 10, rays: 400, height: 6,
    sigmaNearCm: 0.5, sigmaFarCm: 2.0, dropout: 0.015, grazeDropout: 0.5, angleJitterDeg: 0.15,
    selfMask: [[162, 180], [-180, -162]], flagBlind: true
  };

  class Lidar {
    constructor(config, rng) {
      this.cfg = Object.assign({}, DEFAULT_LIDAR, config || {});
      this.rng = rng;
      this.reset(0);
    }
    reset(t) {
      this.revStart = t;
      this.next = 0;           // proximo rayo de la vuelta
      this.points = [];
      this.revolutions = 0;
    }
    /* Mide los rayos cuyo instante cae en [t0, t1). `poseAt(t)` da la pose del eje
     * trasero {x, z, yaw}. Devuelve la vuelta si se completo, o null. */
    advance(t1, poseAt, boxes) {
      const c = this.cfg, period = 1 / c.hz, step = period / c.rays;
      let done = null;
      for (;;) {
        const tRay = this.revStart + (this.next + 0.5) * step;
        if (tRay >= t1) break;
        this.points.push(this.measure(this.next, poseAt(tRay), boxes));
        this.next++;
        if (this.next >= c.rays) {
          done = { points: this.points, tEnd: this.revStart + period, tStart: this.revStart };
          this.revStart += period;
          this.next = 0;
          this.points = [];
          this.revolutions++;
        }
      }
      return done;
    }
    measure(index, pose, boxes) {
      const c = this.cfg, rng = this.rng;
      const angle = -180 + index * (360 / c.rays);
      const masked = c.selfMask.some(([a, b]) => angle >= a && angle <= b);
      const point = { angle, distance: 0, valid: false, blind: false };
      if (masked) return point;
      const s = Math.sin(pose.yaw), co = Math.cos(pose.yaw);
      const ox = pose.x + s * c.mountForward + co * c.mountLateral, oz = pose.z + co * c.mountForward - s * c.mountLateral;
      const a = (angle + (c.angleJitterDeg ? rng.gauss() * c.angleJitterDeg : 0)) * DEG + pose.yaw;
      const dx = Math.sin(a), dz = Math.cos(a);
      let best = Infinity, normal = null;
      for (const b of boxes) {
        if (b.h < c.height) continue;
        const hit = G.rayBox(ox, oz, dx, dz, b);
        if (hit && hit.t < best) { best = hit.t; normal = hit; }
      }
      if (!isFinite(best)) return point;
      if (best < c.minCm) { point.blind = c.flagBlind; return point; }
      if (best > c.maxCm) return point;
      const cosInc = normal && (normal.nx || normal.nz) ? Math.abs(dx * normal.nx + dz * normal.nz) : 1;
      if (rng() < c.dropout) return point;
      if (cosInc < 0.15 && rng() < c.grazeDropout) return point;
      const sigma = (best < 100 ? c.sigmaNearCm : c.sigmaFarCm) * (cosInc < 0.3 ? 2 : 1);
      point.distance = Math.max(c.minCm, best + rng.gauss() * sigma);
      point.valid = true;
      return point;
    }
  }

  /* ------------------------------------------------------------------ camara */
  const DEFAULT_CAMERA = {
    width: 160, height: 120, focal: 118.6, mountForward: 9, mountLateral: 0, camHeight: 5, horizonRow: 30,
    gain: 1.0, tint: [1, 1, 1], noiseSigma: 3.5, vignette: 0.12, exposureSeconds: 0.008,
    background: [88, 92, 104], fps: 40, latencyMs: 45, latencyJitterMs: 8, bitErrorRate: 0
  };

  function projectRow(cam, fwd, up) {
    /* Fila de la imagen de un punto a `fwd` cm delante y `up` cm sobre la camara. */
    const th = cam.pitch, yd = -up;
    const zc = fwd * Math.cos(th) + yd * Math.sin(th), yc = -fwd * Math.sin(th) + yd * Math.cos(th);
    return cam.height / 2 + cam.focal * yc / zc;
  }

  function camState(cfg) {
    const v0 = cfg.height / 2;
    const pitch = Math.atan((v0 - cfg.horizonRow) / cfg.focal);
    return Object.assign({ pitch, cosPitch: Math.cos(pitch), sinPitch: Math.sin(pitch) }, cfg);
  }

  function colorOf(kind) {
    const c = FIELD.RULES.colors;
    return c[kind] || c.wall;
  }

  /* Pinta un cuadro. Devuelve {rgb565: Uint8Array, rgba: Uint8ClampedArray}. */
  function renderCamera(field, boxes, pose, config, rng, dynamics) {
    const cam = camState(config), W = cam.width, H = cam.height, f = cam.focal;
    const dyn = dynamics || {};
    const s = Math.sin(pose.yaw), co = Math.cos(pose.yaw);
    const cx = pose.x + s * cam.mountForward + co * cam.mountLateral;
    const cz = pose.z + co * cam.mountForward - s * cam.mountLateral;
    const img = new Float32Array(W * H * 3);
    const v0 = H / 2;
    const bg = cam.background;
    const wallRGB = colorOf('wall'), floorRGB = colorOf('floor');
    const lines = field.lines || [];
    const lineRGB = { orange: colorOf('orange'), blue: colorOf('blue') };
    const fwdVec = [s, co], rightVec = [co, -s];

    /* Fondo y suelo, fila por fila. */
    for (let v = 0; v < H; v++) {
      const yn = (v + 0.5 - v0) / f;
      const yw = yn * cam.cosPitch + cam.sinPitch;      // componente hacia abajo
      const zw = cam.cosPitch - yn * cam.sinPitch;      // componente hacia delante
      for (let u = 0; u < W; u++) {
        const k = (v * W + u) * 3;
        if (yw <= 1e-4) {
          const t = v / H;
          img[k] = bg[0] * (0.8 + 0.3 * t); img[k + 1] = bg[1] * (0.8 + 0.3 * t); img[k + 2] = bg[2] * (0.8 + 0.3 * t);
          continue;
        }
        const t = cam.camHeight / yw, xn = (u + 0.5 - W / 2) / f;
        const fwd = zw * t, lat = xn * t;
        let r = floorRGB[0], g = floorRGB[1], b = floorRGB[2];
        if (lines.length) {
          const wx = cx + fwdVec[0] * fwd + rightVec[0] * lat, wz = cz + fwdVec[1] * fwd + rightVec[1] * lat;
          for (const ln of lines) {
            const ax = G.boxAxes(ln), dx = wx - ln.x, dz = wz - ln.z;
            if (Math.abs(dx * ax.right[0] + dz * ax.right[1]) <= ln.w / 2 &&
                Math.abs(dx * ax.forward[0] + dz * ax.forward[1]) <= ln.d / 2) { [r, g, b] = lineRGB[ln.kind]; break; }
          }
        }
        img[k] = r; img[k + 1] = g; img[k + 2] = b;
      }
    }
    /* Objetos del fondo (gente, bolsas): franjas de color detras de las paredes. */
    for (const c of field.clutter || []) {
      const u0 = Math.round(W / 2 + f * Math.tan((c.bearingDeg - pose.yaw / DEG) * DEG) / cam.cosPitch) - Math.round(c.widthPx / 2);
      for (let u = Math.max(0, u0); u < Math.min(W, u0 + c.widthPx); u++)
        for (let v = Math.max(0, c.topRow); v < Math.min(H, c.bottomRow); v++) {
          const k = (v * W + u) * 3; img[k] = c.color[0]; img[k + 1] = c.color[1]; img[k + 2] = c.color[2];
        }
    }
    /* Prismas verticales: un rayo por columna, de lejos a cerca. */
    const hits = [];
    for (let u = 0; u < W; u++) {
      const xn = (u + 0.5 - W / 2) / f, phi = Math.atan(xn * cam.cosPitch);
      const a = pose.yaw + phi, dx = Math.sin(a), dz = Math.cos(a);
      hits.length = 0;
      for (const b of boxes) {
        const h = G.rayBox(cx, cz, dx, dz, b);
        if (h && h.t > 0.5) hits.push({ t: h.t, b, nx: h.nx, nz: h.nz });
      }
      hits.sort((p, q) => q.t - p.t);
      const cosPhi = Math.cos(phi);
      for (const h of hits) {
        const fwd = h.t * cosPhi;
        if (fwd < 1) continue;
        const top = projectRow(cam, fwd, h.b.h - cam.camHeight), base = projectRow(cam, fwd, -cam.camHeight);
        const rgb = colorOf(h.b.kind);
        /* Iluminacion lateral: las caras no frontales son algo mas oscuras. */
        const facing = Math.abs(dx * h.nx + dz * h.nz);
        const shade = 0.82 + 0.18 * facing;
        const r0 = Math.max(0, Math.ceil(top - 0.5)), r1 = Math.min(H - 1, Math.floor(base - 0.5));
        for (let v = r0; v <= r1; v++) {
          const k = (v * W + u) * 3;
          img[k] = rgb[0] * shade; img[k + 1] = rgb[1] * shade; img[k + 2] = rgb[2] * shade;
        }
      }
    }
    /* Desenfoque de movimiento por giro y optica. */
    const blur = Math.round(Math.abs((dyn.yawRate || 0)) * cam.exposureSeconds * f);
    let src = img;
    if (blur >= 1) {
      src = new Float32Array(img.length);
      const half = blur;
      for (let v = 0; v < H; v++) for (let u = 0; u < W; u++) {
        let r = 0, g = 0, b = 0, n = 0;
        for (let d = -half; d <= half; d++) {
          const uu = u + d; if (uu < 0 || uu >= W) continue;
          const k = (v * W + uu) * 3; r += img[k]; g += img[k + 1]; b += img[k + 2]; n++;
        }
        const k = (v * W + u) * 3; src[k] = r / n; src[k + 1] = g / n; src[k + 2] = b / n;
      }
    }
    const gain = cam.gain * (dyn.light == null ? 1 : dyn.light);
    const rgb565 = new Uint8Array(W * H * 2), rgba = new Uint8ClampedArray(W * H * 4);
    for (let v = 0; v < H; v++) {
      for (let u = 0; u < W; u++) {
        const k = (v * W + u) * 3;
        const dxv = (u + 0.5 - W / 2) / (W / 2), dyv = (v + 0.5 - H / 2) / (H / 2);
        const vig = 1 - cam.vignette * (dxv * dxv + dyv * dyv) * 0.5;
        const m = gain * vig;
        const n = cam.noiseSigma;
        const r = Math.min(255, Math.max(0, src[k] * m * cam.tint[0] + (n ? rng.gauss() * n : 0)));
        const g = Math.min(255, Math.max(0, src[k + 1] * m * cam.tint[1] + (n ? rng.gauss() * n : 0)));
        const b = Math.min(255, Math.max(0, src[k + 2] * m * cam.tint[2] + (n ? rng.gauss() * n : 0)));
        const r5 = Math.round(r) >> 3, g6 = Math.round(g) >> 2, b5 = Math.round(b) >> 3;
        const px = (r5 << 11) | (g6 << 5) | b5, o = (v * W + u) * 2;
        rgb565[o] = px >> 8; rgb565[o + 1] = px & 255;            // el byte alto primero, como esp32-camera
        const p = (v * W + u) * 4;
        rgba[p] = (r5 * 255 / 31) | 0; rgba[p + 1] = (g6 * 255 / 63) | 0; rgba[p + 2] = (b5 * 255 / 31) | 0; rgba[p + 3] = 255;
      }
    }
    return { rgb565, rgba, pitchDeg: cam.pitch / DEG };
  }

  /* ------------------------------------------------------------ giroscopio */
  const DEFAULT_IMU = {
    hz: 100, biasDps: null, biasSigma: 0.6, biasWalkDpsPerSqrtS: 0.02, noiseDps: 0.3,
    scaleError: 0.004, saturateDps: 2000
  };
  class Gyro {
    constructor(config, rng) {
      this.cfg = Object.assign({}, DEFAULT_IMU, config || {});
      this.rng = rng;
      this.bias = this.cfg.biasDps == null ? rng.gauss() * this.cfg.biasSigma : this.cfg.biasDps;
      this.scale = 1 + (rng.range(-1, 1) * this.cfg.scaleError);
    }
    /* Velocidad angular horaria real (rad/s) -> lectura del giroscopio (grados/s). */
    sample(trueRateRad, dt) {
      const c = this.cfg;
      this.bias += this.rng.gauss() * c.biasWalkDpsPerSqrtS * Math.sqrt(dt);
      const dps = trueRateRad / DEG * this.scale + this.bias + this.rng.gauss() * c.noiseDps;
      return Math.max(-c.saturateDps, Math.min(c.saturateDps, dps));
    }
  }

  /* Encoders de las ruedas, como los entrega el encoder de cada rueda: angulo acumulado de cada motor en grados
   * enteros (resolucion configurable), a una frecuencia y con un retardo. El diametro real de cada rueda lo pone
   * el robot; aqui solo se anade lo que hace el sensor: cuantizar, ruido y perdida de muestras. */
  const DEFAULT_ENCODER = { enabled: true, resolutionDeg: 1, noiseDeg: 0.15, hz: 50, latencyMs: 15, dropRate: 0.01,
                            diameterErrorPct: 1.5 };
  class Encoders {
    constructor(config, rng) {
      this.cfg = Object.assign({}, DEFAULT_ENCODER, config || {});
      this.rng = rng;
    }
    sample(robot) {
      const c = this.cfg, q = v => Math.round((v + this.rng.gauss() * c.noiseDeg) / c.resolutionDeg) * c.resolutionDeg;
      return { left: q(robot.encL), right: q(robot.encR) };
    }
  }

  WRO.sensors = { Lidar, Gyro, Encoders, DEFAULT_ENCODER, renderCamera, DEFAULT_LIDAR, DEFAULT_CAMERA, DEFAULT_IMU, projectRow, camState };
  if (typeof module !== 'undefined') module.exports = WRO;
})(typeof window !== 'undefined' ? window : globalThis);
