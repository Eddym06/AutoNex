#!/bin/bash
set -euo pipefail
# uso: [FROM=21 TO=70] big.sh TAG "cfg1;cfg2"  -> 100 pistas (semillas FROM..TO, ambos sentidos) en 22 procesos
S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # carpeta del script
O="${OUT:-$S}"; mkdir -p "$O"                      # donde se escriben los resultados b_TAG_*.txt (OUT=carpeta; por defecto junto al script)
TAG=${1:?Falta el nombre del banco}; export CFG="${2:-}"
[[ "$TAG" =~ ^[A-Za-z0-9_-]+$ ]] || { echo 'Tag invalido' >&2; exit 2; }
rm -f -- "$O"/b_"${TAG}"_*.txt
jobs=(); pids=(); for s in $(seq ${FROM:-21} ${TO:-70}); do for d in ccw cw; do jobs+=("$s:$d"); done; done
for k in $(seq 0 21); do
  sub=(); i=$k; while [ $i -lt ${#jobs[@]} ]; do sub+=("${jobs[$i]}"); i=$((i+22)); done
  node "$S/ab2.js" "$TAG" "${sub[@]}" > "$O/b_${TAG}_$k.txt" 2>&1 &
  pids+=("$!")
done
failed=0
for pid in "${pids[@]}"; do wait "$pid" || failed=1; done
if (( failed )); then echo 'falló al menos un proceso' >&2; exit 1; fi
echo "done $TAG"
