#!/bin/bash
# Tabla de sensibilidad del modelo de IT: reoptimiza (recocido, 1e6 evaluaciones) la pista con un parametro cambiado cada vez.
#   bash simulator/tools/it_py/sensitivity.sh [plano.json]      (por defecto simulator/tests/plano_it_ovalo.json)
# Necesita node, python con numpy/scipy/numba y el wasm compilado. Tarda ~40 s (las 8 variantes corren en paralelo).
cd "$(dirname "${BASH_SOURCE[0]}")" || exit 1
PLANO="${1:-../../tests/plano_it_ovalo.json}"
OUT="${TMPDIR:-${TEMP:-/tmp}}"
# plan del C como punto de partida del recocido
node ../it_user_plan.js "$PLANO" --out "$OUT/plan_c.json" > /dev/null
run() { tag=$1; shift; python optimize.py "$PLANO" "$OUT/plan_c.json" "$OUT/w_$tag.json" --evals 1e6 "$@" 2>&1 | tail -1 | sed "s/^/$tag: /" & }
run base                       # hoy (track_err 14)
run te8   track_err=8
run te5   track_err=5
run marg0 wall_m=0.0 obs_m=0.0 # margenes de pared y letrero en 0
run obs25 obs_half=2.5         # radio de letrero 2,5 cm en vez de 3,5
run su100 steer_use=1.0
run vmax200 vmax=200
run grip10  grip=1.0
wait
