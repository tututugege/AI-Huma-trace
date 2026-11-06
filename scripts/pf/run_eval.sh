#!/usr/bin/env bash
# run_eval.sh <sim> <outdir> [train|hold|all] [jobs]   -> <outdir>/<bench>/<ckpt>.res ; skips finished points
HERE=$(cd "$(dirname "$0")" && pwd)
SIM=$1; OUT=$2; SET=${3:-train}; JOBS=${4:-8}
mkdir -p "$OUT"
awk -v s="$SET" '$1==s || s=="all" {print $3, $2}' "$HERE/${POINTS:-points.txt}" | while read n b; do
  [ -f "$OUT/$b/$n.res" ] || echo "$n $b"; done | xargs -P "$JOBS" -L 1 "$HERE/run_pair.sh" "$SIM" "$OUT"
