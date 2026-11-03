#!/usr/bin/env bash
# Usage: gate.sh <simulator-binary> <out.csv> <test.bin>...
# Runs each test, records exit status, wall seconds, and the full log tail.
# Cycle count is parsed from the simulator's final output (see parse_cycles).
set -uo pipefail
SIM=$1; OUT=$2; shift 2
TO=$(command -v timeout || command -v gtimeout || true)
LOGDIR=$(dirname "$OUT")/logs; mkdir -p "$LOGDIR"
echo "test,status,wall_s,cycles" > "$OUT"
for t in "$@"; do
  n=$(basename "$t")
  s=$(date +%s.%N 2>/dev/null || python3 -c 'import time;print(time.time())')
  ${TO:-perl -e 'alarm shift; exec @ARGV'} "${TIMEOUT:-600}" "$SIM" "$t" > "$LOGDIR/$n.log" 2>&1; rc=$?
  e=$(date +%s.%N 2>/dev/null || python3 -c 'import time;print(time.time())')
  cyc=$(grep -a -o 'sim-time(cycle)= *[0-9]*' "$LOGDIR/$n.log" | tail -1 | grep -o '[0-9]*$')
  echo "$n,$rc,$(python3 -c "print(round($e-$s,2))"),${cyc:-NA}" >> "$OUT"
done
cat "$OUT"
