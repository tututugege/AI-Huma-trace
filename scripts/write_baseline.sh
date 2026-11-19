#!/usr/bin/env bash
# write_baseline.sh <simulator> <outdir (new)> <baseline csv to write> [jobs]: run the run_gate.sh program set and write its cycles as a
# baseline table (test,status,wall_s,cycles). Environment (e.g. PF_ENABLE=0 for baseline_nopf.csv) is passed through.
SIM=$1; OUT=$2; CSV=$3; JOBS=${4:-4}
T=$(cd "$(dirname "$0")/.." && pwd)
BASELINE=$T/results/baseline/baseline.csv DEADLINE=${DEADLINE:-60} bash "$T/scripts/run_gate.sh" "$SIM" "$OUT" "$JOBS" > /dev/null
{ echo "test,status,wall_s,cycles"
  for f in "$OUT"/logs/*.bin.log; do
    t=$(basename "$f" .log); c=$(grep -a -o 'sim-time(cycle)= *[0-9]*' "$f" | tail -1 | grep -o '[0-9]*$')
    echo "$t,0,0,${c:-NA}"
  done; } > "$CSV"
echo "wrote $CSV: $(($(wc -l < "$CSV") - 1)) programs, $(grep -c ',NA$' "$CSV") without a cycle count"
