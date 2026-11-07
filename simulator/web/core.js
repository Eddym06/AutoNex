/* Envoltorio del firmware compilado a WebAssembly (wro_core.wasm). Todo lo que
 * decide (vision del S3, mapa, planificador, controlador de la mision del P4) es
 * el codigo C real; este archivo solo copia datos hacia y desde su memoria. */
(function (root) {
  'use strict';
  const WRO = root.WRO = root.WRO || {};

  const PHASES = ['IDLE', 'RUN', 'UTURN', 'FINISH', 'PARK_SEARCH', 'PARKING', 'DONE', 'FAULT', 'PARK_EXIT'];
  const ACTIONS = ['STOP', 'FORWARD', 'RIGHT_SOFT', 'RIGHT_HARD', 'LEFT_SOFT', 'LEFT_HARD', 'PARK_LEFT',
                   'PARK_RIGHT', 'PARK_APPROACH', 'PARK_READY', 'RETURN_LEFT', 'RETURN_RIGHT', 'REVERSE', 'NUDGE_RIGHT', 'NUDGE_LEFT'];
  const VISION_CLASSES = ['none', 'red', 'green', 'magenta', 'orange', 'blue'];

  async function load(source) {
    const imports = { wasi_snapshot_preview1: { fd_close: () => 0, fd_seek: () => 0, fd_write: () => 0 } };
    const result = await WebAssembly.instantiate(source, imports);
    const x = (result.instance || result).exports;
    const mem = () => x.memory.buffer;
    const f32 = (ptr, n) => new Float32Array(mem(), ptr, n);
    const u8 = (ptr, n) => new Uint8Array(mem(), ptr, n);
    const text = {
      set(s) {
        const bytes = new TextEncoder().encode(s), cap = x.core_text_size() - 1;
        if (bytes.length > cap) throw new Error('texto de configuracion demasiado largo');
        const p = u8(x.core_text_ptr(), bytes.length + 1);
        p.set(bytes); p[bytes.length] = 0;
      },
      get(n) { return new TextDecoder().decode(u8(x.core_text_ptr(), n).slice()); },
      error() {
        const p = u8(x.core_error_ptr(), 160);
        let n = 0; while (n < 160 && p[n]) n++;
        return new TextDecoder().decode(p.slice(0, n));
      }
    };
    const frameBytes = x.core_frame_size();
    const POINT_MAX = 512;

    function readVision() {
      const o = f32(x.core_vision_out_ptr(), 8 + 13 * 9).slice();
      const objects = [];
      for (let i = 0; i < o[2]; i++) {
        const p = 8 + i * 9;
        objects.push({ color: VISION_CLASSES[o[p]] || 'none', colorId: o[p], minX: o[p + 1], maxX: o[p + 2],
                       minY: o[p + 3], maxY: o[p + 4], samples: o[p + 5], distance: o[p + 6],
                       flags: o[p + 7], area: o[p + 8] });
      }
      return { valid: o[0] > 0, action: ACTIONS[o[1]] || 'STOP', orangeRow: o[3], blueRow: o[4], objects };
    }

    function readCommand() {
      const o = f32(x.core_cmd_ptr(), 86).slice();
      return {
        action: ACTIONS[o[0]] || 'STOP', actionId: o[0], steer: o[1], speed: o[2], phase: PHASES[o[3]] || 'FAULT',
        done: o[4] > 0, corners: o[5], laps: o[6], direction: o[7], reversed: o[8] > 0, yaw: o[9], course: o[10],
        front: o[11], left: o[12], right: o[13], lane: o[14], pass: o[15], elapsed: o[16], lastSign: o[17],
        escapePhase: o[18], escapes: o[19], escaping: o[20] > 0, cameraHits: o[21], cameraUnmatched: o[22], cameraFresh: o[23] > 0, yawRate: o[24], explicitYawRate: o[25] > 0, freeCm: o[26], sidestep: o[32], sidestepSide: o[33], pivoting: o[34] > 0, arena: { x: o[48], z: o[49], theta: o[50], valid: o[51] > 0, inliers: o[52], used: o[53], rms: o[54], updates: o[55] }, odom: { x: o[40], y: o[41], path: o[42], slipCm: o[43], inUse: o[44] > 0, v: o[45], heading: o[46], slips: o[47] }, lapMem: { inject: o[35], anchored: o[36], live: o[37], err: o[38], off: o[39] }, park: { u: o[27], w: o[28], theta: o[29], lost: o[30], move: o[31] },
        hug: { active: o[82] > 0, cmd: o[83], target: o[84], hold: o[85] },
        race: { valid: o[56] > 0, idx: o[57], active: o[58] > 0, vLine: o[59], vCmd: o[60], lat: o[61], dist: o[62], hold: o[63], time: o[64], timeCenter: o[65], length: o[66], mix: o[67], n: o[68], tried: o[69] > 0, signs: o[70], grip: o[80], slips: o[81], refRows: o[71], ref: [o[72], o[73], o[74], o[75], o[76], o[77], o[78], o[79]] }
      };
    }

    const api = {
      raw: x,
      PHASES, ACTIONS,
      /* --- vision --- */
      visionDefaults() { x.core_vision_defaults(); },
      visionConfig(textValue) {
        text.set(textValue);
        const ok = x.core_vision_config_apply() === 1;
        return { ok, error: ok ? '' : text.error() };
      },
      visionConfigDump() { return text.get(x.core_vision_config_dump()); },
      processFrame(rgb565, parking) {
        if (rgb565.length !== 160 * 120 * 2) throw new Error('el cuadro debe ser RGB565 de 160 x 120');
        u8(x.core_frame_ptr(), frameBytes).set(rgb565);
        x.core_vision_process(parking ? 1 : 0);
        return readVision();
      },
      /* Como el S3: procesa y devuelve la trama del enlace UART. */
      cameraFrame(rgb565, sequence, timestampMs, parking) {
        u8(x.core_frame_ptr(), frameBytes).set(rgb565);
        const n = x.core_camera_frame(sequence, timestampMs, parking ? 1 : 0);
        return { wire: u8(x.core_wire_ptr(), n).slice(), vision: readVision() };
      },
      /* --- controlador --- */
      ctrlDefaults() { x.core_ctrl_defaults(); },
      ctrlConfig(textValue) {
        text.set(textValue);
        const ok = x.core_ctrl_config_apply() === 1;
        return { ok, error: ok ? '' : text.error() };
      },
      ctrlConfigDump() { return text.get(x.core_ctrl_config_dump()); },
      ctrlInit() { x.core_ctrl_init(); },
      ctrlReconfigure() { x.core_ctrl_reconfigure(); },
      imu(tUs, gyroDps, magDeg) { x.core_ctrl_imu(tUs, gyroDps, magDeg == null ? NaN : magDeg); },
      encoders(tUs, leftDeg, rightDeg) { x.core_ctrl_encoders(tUs, leftDeg, rightDeg); },
      start(tUs) { return x.core_ctrl_start(tUs) === 1; },
      cameraFocal(f) { x.core_camera_focal(f); },
      cameraPitch(deg) { x.core_camera_pitch(deg); },
      /* La trama llega por el enlace UART (posiblemente corrupta). */
      cameraFeed(wire, arrivalUs) {
        u8(x.core_wire_in_ptr(), wire.length).set(wire);
        return x.core_camera_feed(wire.length, arrivalUs) === 1;
      },
      /* points: [{angle, distance, valid, blind}] en orden de barrido. */
      scan(points, tUs) {
        const n = Math.min(points.length, POINT_MAX);
        const buf = f32(x.core_points_ptr(), n * 4);
        for (let i = 0; i < n; i++) {
          const p = points[i];
          buf[i * 4] = p.angle; buf[i * 4 + 1] = p.distance; buf[i * 4 + 2] = p.valid ? 1 : 0; buf[i * 4 + 3] = p.blind ? 1 : 0;
        }
        x.core_ctrl_scan(n, tUs);
        return readCommand();
      },
      /* La linea de carrera calculada: [{x, z, v}] (vacia si aun no hay). */
      raceLine() {
        const n = f32(x.core_race_ptr(), 1)[0] | 0;
        if (n <= 0) return [];
        const o = f32(x.core_race_ptr(), 1 + 3 * n).slice(), pts = [];
        for (let i = 0; i < n; i++) pts.push({ x: o[1 + 3 * i], z: o[2 + 3 * i], v: o[3 + 3 * i] });
        return pts;
      },
      /* Letreros que uso la linea: [{x, z, color}] (1 rojo, 2 verde). */
      raceSigns() {
        const o = f32(x.core_race_ptr(), 2 + 3 * 192 + 3 * 16).slice(), b = 1 + 3 * 192, n = o[b] | 0, out = [];
        for (let i = 0; i < n; i++) out.push({ x: o[b + 1 + 3 * i], z: o[b + 2 + 3 * i], color: o[b + 3 + 3 * i] });
        return out;
      },
      watchdog(nowUs) { return x.core_ctrl_watchdog(nowUs) === 1 ? readCommand() : null; },
      arcClear(steerDeg, distance) { return x.core_arc_clear(steerDeg, distance) === 1; },
      mapSize() { return x.core_map_size(); },
      mapCellCm() { return x.core_map_cell_cm(); },
      map() { const n = x.core_map_size(); return u8(x.core_map_ptr(), n * n).slice(); },
      debug() {
        const o = f32(x.core_debug_ptr(), 8 + 3 * 48 * 2 + 12 * 4).slice();
        const route = [], probe = [], colored = [], plan = [];
        for (let i = 0; i < o[4]; i++) route.push([o[8 + i * 2], o[9 + i * 2]]);
        for (let i = 0; i < o[5]; i++) probe.push([o[8 + 96 + i * 2], o[9 + 96 + i * 2]]);
        for (let i = 0; i < o[3]; i++) plan.push([o[8 + 192 + i * 2], o[9 + 192 + i * 2]]);
        for (let i = 0; i < o[7]; i++) {
          const p = 8 + 288 + i * 4;
          colored.push({ x: o[p], forward: o[p + 1], color: o[p + 2], confirmed: o[p + 3] > 0 });
        }
        return { lane: o[0], reference: o[1], pass: o[2], planPoints: o[3], route, probe, plan, curtains: o[6], colored };
      }
    };
    return api;
  }

  WRO.core = { load, PHASES, ACTIONS };
  if (typeof module !== 'undefined') module.exports = WRO;
})(typeof window !== 'undefined' ? window : globalThis);
