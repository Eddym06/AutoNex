#!/bin/bash
# Rondas aleatorias en paralelo: tests/pbatch.sh DESDE HASTA [LETREROS] [JOBS]
# Variables: CTRL="clave valor;clave valor" (configuracion del controlador), SEAT_ALONG, SEAT_LAT.
from=${1:-1}; to=${2:-8}; signs=${3:-7}; jobs=${4:-20}
dir="$(cd "$(dirname "$0")" && pwd)"
for s in $(seq "$from" "$to"); do for d in ccw cw; do echo "$s $d"; done; done |
  xargs -P "$jobs" -L1 bash -c 'node "'"$dir"'/batch.js" $0 $0 $1 '"$signs"' | grep "^{"' |
  sort -t: -k2 -n |
  node -e '
    const rows = require("fs").readFileSync(0, "utf8").trim().split("\n").filter(Boolean).map(l => JSON.parse(l));
    const ok = rows.filter(r => r.end === "finished" && !r.viol);
    const parked = rows.filter(r => r.parking === "full").length;
    for (const r of rows) if (!(r.end === "finished" && !r.viol)) console.log(r.seed + " " + r.direction + " pts=" + r.pts + " park=" + r.parking + " sec=" + r.sec + " " + r.end + " " + (r.viol || ""));
    const cat = {}; for (const r of rows) cat[r.parking] = (cat[r.parking] || 0) + 1;
    console.log("aparcado: " + JSON.stringify(cat));
    console.log(ok.length + "/" + rows.length + " completas; media " + (rows.reduce((a, r) => a + r.pts, 0) / rows.length).toFixed(1) + " puntos; " + parked + " aparcados del todo; tiempo medio " + (ok.reduce((a, r) => a + r.t, 0) / Math.max(1, ok.length)).toFixed(0) + " s");
  '
