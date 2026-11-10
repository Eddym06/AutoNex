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
    if (typeof module !== 'undefined' && typeof __dirname !== 'undefined') {
      const fs = require('node:fs'), path = require('node:path'), crypto = require('node:crypto');
      const repo = path.resolve(__dirname, '../..');
      const manifestPath = path.join(__dirname, 'wro_core.manifest.json');
      if (!fs.existsSync(manifestPath)) throw new Error('WASM sin manifiesto: recompila simulator/wasm/build.ps1');
      const manifest = JSON.parse(fs.readFileSync(manifestPath, 'utf8'));
      const files = ['simulator/wasm/core_api.c', 'simulator/wasm/build.ps1', 'simulator/wasm/manifest.py', 'simulator/tools/bench/rebuild_wasm.sh'];
      const visit = dir => {
        for (const entry of fs.readdirSync(path.join(repo, dir), { withFileTypes: true })) {
          const rel = dir + '/' + entry.name;
          if (entry.isDirectory()) visit(rel);
          else if (/\.(c|h)$/.test(entry.name)) files.push(rel);
        }
      };
      for (const dir of ['LIDAR System/main', 'LIDAR System/orchestrator', 'Vision System/main', 'shared']) visit(dir);
      const hash = crypto.createHash('sha256');
      for (const file of files.sort()) {
        hash.update(file).update(Buffer.from([0]));
        hash.update(fs.readFileSync(path.join(repo, file), 'utf8').replace(/\r\n/g, '\n'));
        hash.update(Buffer.from([0]));
      }
      const wasmHash = crypto.createHash('sha256').update(source).digest('hex');
      if (hash.digest('hex') !== manifest.sources_sha256 || wasmHash !== manifest.wasm_sha256)
        throw new Error('WASM desactualizado: recompila simulator/wasm/build.ps1');
    }
    const imports = { wasi_snapshot_preview1: { fd_close: () => 0, fd_seek: () => 0, fd_write: () => 0 } };
    const result = await WebAssembly.instantiate(source, imports);
    const x = (result.instance || result).exports;
    const expected = { core_abi_version: 2, core_cmd_fields: 88, core_vision_fields: 125,
      core_debug_fields: 345, core_debug_route_points: 48, core_debug_camera_objects: 12, core_max_points: 512 };
    for (const [name, value] of Object.entries(expected)) {
      if (typeof x[name] !== 'function' || x[name]() !== value)
        throw new Error('ABI WASM incompatible (' + name + '): recompila el simulador y revisa core.js');
    }
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
      const o = f32(x.core_cmd_ptr(), 88).slice();
      return {
        action: ACTIONS[o[0]] || 'STOP', actionId: o[0], steer: o[1], speed: o[2], phase: PHASES[o[3]] || 'FAULT',
        done: o[4] > 0, corners: o[5], laps: o[6], direction: o[7], reversed: o[8] > 0, yaw: o[9], course: o[10],
        front: o[11], left: o[12], right: o[13], lane: o[14], pass: o[15], elapsed: o[16], lastSign: o[17],
        escapePhase: o[18], escapes: o[19], escaping: o[20] > 0, cameraHits: o[21], cameraUnmatched: o[22], cameraFresh: o[23] > 0, yawRate: o[24], explicitYawRate: o[25] > 0, freeCm: o[26], sidestep: o[32], sidestepSide: o[33], pivoting: o[34] > 0, arena: { x: o[48], z: o[49], theta: o[50], valid: o[51] > 0, inliers: o[52], used: o[53], rms: o[54], updates: o[55] }, odom: { x: o[40], y: o[41], path: o[42], slipCm: o[43], inUse: o[44] > 0, v: o[45], heading: o[46], slips: o[47] }, lapMem: { inject: o[35], anchored: o[36], live: o[37], err: o[38], off: o[39] }, park: { u: o[27], w: o[28], theta: o[29], lost: o[30], move: o[31] },
        hug: { active: o[82] > 0, cmd: o[83], target: o[84], hold: o[85] },
        moveCm: o[86], moveId: o[87],
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
      axleEncoder(tUs, axleDeg) {
        if (!x.core_ctrl_axle_encoder) throw new Error('WASM anterior a la odometría Ackermann: recompílalo');
        x.core_ctrl_axle_encoder(tUs, axleDeg);
      },
      hasAxleEncoder() { return typeof x.core_ctrl_axle_encoder === 'function'; },
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
      /* MIC: control del motor (Motor Control/MIC). El simulador lo ejecuta a 1 kHz dentro de cada paso de fisica. */
      hasMic() { return typeof x.core_mic_init === 'function'; },
      micInit() { x.core_mic_init(); },
      micCmPerCount() { return x.core_mic_cm_per_count(); },
      micRequestSpeed(v, decel) { x.core_mic_request_speed(v, decel ? 1 : 0); },
      micStop(decel) { x.core_mic_stop(decel ? 1 : 0); },
      micRequestMove(distCm, vCruise, vEnd, decel) { x.core_mic_request_move(distCm, vCruise, vEnd, decel ? 1 : 0); },
      micUpdate(counts, vbat, dt) {
        x.core_mic_update(counts, vbat, dt);
        const o = f32(x.core_mic_ptr(), 10);
        return { duty: o[0], brakeShort: o[1] > 0, phase: o[2], v: o[3], pos: o[4], vRef: o[5],
                 emergency: o[6] > 0, current: o[7], done: o[8] > 0, disturbance: o[9] };
      },
      /* IT (banco de pruebas): planifica una vuelta sobre los obstaculos [{x, z, color}] y sigue la trayectoria. */
      itBuild(direction, params, obstacles) {
        const p = f32(x.core_it_params_ptr(), 24); p.fill(0);
        ['grip', 'vMax', 'aMotor', 'mu', 'rearWeight', 'trackErr', 'poseErr', 'lag', 'steerRate', 'wheelbase', 'halfWidth', 'frontExt', 'safety',
         'sweeps', 'passes', 'steerMax', 'vMin', 'cgHeight'].forEach((k, i) => { if (params && params[k] > 0) p[i] = params[k]; });
        const ob = f32(x.core_it_obs_ptr(), 72);
        obstacles.slice(0, 24).forEach((q, i) => { ob[3 * i] = q.x; ob[3 * i + 1] = q.z; ob[3 * i + 2] = q.color || 0; });
        const ok = x.core_it_build(direction === 'cw' ? 1 : -1, Math.min(24, obstacles.length)) === 1;
        const o = f32(x.core_it_out_ptr(), 8 + 8 * 256), n = o[0] | 0, line = [];
        for (let i = 0; i < n; i++) { const b = 8 + 8 * i; line.push({ x: o[b], z: o[b + 1], v: o[b + 2], heading: o[b + 3], kappa: o[b + 4], room: o[b + 5], margin: o[b + 6], d: o[b + 7] }); }
        return { ok, n, time: o[2], timeCenter: o[3], evals: o[4], length: o[5], violation: o[6], why: o[7] | 0, line };
      },
      itTrack(px, pz, yawDeg, v) {
        const ok = x.core_it_track(px, pz, yawDeg, v) === 1, t = f32(x.core_it_trk_ptr(), 8);
        return { ok, steer: t[0], speed: t[1], e: t[2], idx: t[3] | 0, psiErr: t[4], room: t[5], vPlan: t[6] };
      },
      itAdapt(contacts, maxErr, minRoom, slip, traction, gripMax) { return x.core_it_adapt(contacts, maxErr, minRoom, slip, traction, gripMax); },
      arcClear(steerDeg, distance) { return x.core_arc_clear(steerDeg, distance) === 1; },
      /* Objetos del mapa persistente: [{id, kind (1 pared, 2 pilar, 3 barra), color (1 rojo, 2 verde, 3 magenta), confidence, x, z, length, width, axis, cells}]. */
      traceObjects() {
        const n = f32(x.core_trace_objects_ptr(), 1)[0] | 0, o = f32(x.core_trace_objects_ptr(), 1 + 10 * n).slice(), out = [];
        for (let i = 0; i < n; i++) { const p = 1 + 10 * i; out.push({ id: o[p], kind: o[p + 1], color: o[p + 2], confidence: o[p + 3], x: o[p + 4], z: o[p + 5], length: o[p + 6], width: o[p + 7], axis: o[p + 8], cells: o[p + 9] }); }
        return out;
      },
      /* Plaza de parqueo del mapa: status 0 ninguna, 1 hipotesis, 2 confirmada. */
      traceSpot() {
        const o = f32(x.core_trace_spot_ptr(), 12).slice();
        return { status: o[0], x: o[1], z: o[2], axis: o[3], length: o[4], width: o[5], bars: [[o[6], o[7]], [o[8], o[9]]], confidence: o[10], wall: o[11] };
      },
      traceMap() {
        const size=x.core_trace_map_size();
        return { size, cellCm:x.core_trace_map_cell_cm(), originCm:x.core_trace_map_origin_cm(),
          bytes:x.core_trace_map_bytes(), laps:x.core_trace_map_laps(), occupied:x.core_trace_map_occupied(), scans:x.core_trace_map_scans(),
          clipped:x.core_trace_map_clipped(), cleared:x.core_trace_map_cleared(), cells:u8(x.core_trace_map_ptr(),size*size).slice(),
          miss:u8(x.core_trace_map_miss_ptr(),size*size).slice(),
          hits:u8(x.core_trace_map_hits_ptr(),size*size).slice(),
          surface:u8(x.core_trace_map_surface_ptr(),size*size/8).slice() };   /* superficie promediada: bit i = celda i (fila*size+col) */
      },
      traceParking() {
        const p=f32(x.core_trace_parking_ptr(),19).slice();
        return { seen:!!p[0],status:p[1]|0,points:[{x:p[2],z:p[3]},{x:p[4],z:p[5]}],spacing:p[6],alternatives:p[7]|0,
          wallCells:p[8]|0,cameraPose:{x:p[9],z:p[10],theta:p[11]},bearing:[p[12],p[13]],
          memory:p[14]?[{x:p[15],z:p[16]},{x:p[17],z:p[18]}]:null };
      },
      mapSize() { return x.core_map_size(); },
      mapCellCm() { return x.core_map_cell_cm(); },
      map() { const n = x.core_map_size(); return u8(x.core_map_ptr(), n * n).slice(); },
      /* Plan de la maniobra de parqueo (ver core_park_dbg_ptr): si hay plan, nodos expandidos,
       * paso en cm, si ya esta en la meta y los arcos que quedan por ejecutar. */
      parkPlan() {
        const d = f32(x.core_park_dbg_ptr(), 5).slice();
        const p = f32(x.core_park_plan_ptr(), 2 + 2 * 128).slice();
        const arcs = [];
        for (let i = 0; i < p[0]; i++) arcs.push([p[2 + 2 * i], p[3 + 2 * i]]);
        return { found: d[0] > 0, expanded: d[1] | 0, atGoal: d[2] > 0, count: d[3] | 0,
                 stepCm: d[4] / 10, index: p[1] | 0, arcs };
      },
      debug() {
        const o = f32(x.core_debug_ptr(), x.core_debug_fields()).slice();
        const route = [], probe = [], colored = [], plan = [];
        for (let i = 0; i < o[4]; i++) route.push([o[9 + i * 2], o[10 + i * 2]]);
        for (let i = 0; i < o[5]; i++) probe.push([o[9 + 96 + i * 2], o[10 + 96 + i * 2]]);
        for (let i = 0; i < o[8]; i++) plan.push([o[9 + 192 + i * 2], o[10 + 192 + i * 2]]);
        for (let i = 0; i < o[7]; i++) {
          const p = 9 + 288 + i * 4;
          colored.push({ x: o[p], forward: o[p + 1], color: o[p + 2], confirmed: o[p + 3] > 0 });
        }
        return { lane: o[0], reference: o[1], pass: o[2], cost: o[3], planPoints: o[8], route, probe, plan, curtains: o[6], colored };
      }
    };
    return api;
  }

  WRO.core = { load, PHASES, ACTIONS };
  if (typeof module !== 'undefined') module.exports = WRO;
})(typeof window !== 'undefined' ? window : globalThis);
