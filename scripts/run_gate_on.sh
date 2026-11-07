#!/usr/bin/env bash
# Difftest gate for configurations that change timing: every test must print "Success" (cycles are not compared).
# Usage: run_gate_on.sh <simulator-binary> <outdir> <test-root> [jobs]    (env of the caller selects the BPU switches)
# <test-root> holds build/tests/*/*.bin and build/vm/*/*.bin (same set as run_gate.sh).
SIM=$1; OUT=$2; ROOT=$3; JOBS=${4:-4}
T=$(cd "$(dirname "$0")/.." && pwd); BASE=$T/results/baseline/baseline.csv
rm -rf "$OUT"; mkdir -p "$OUT/logs"
one() {
  b=$1; t=$(basename "$b"); base=$(grep "^$t," "$BASE" | cut -d, -f4)
  lim=$(( base * 6 > 6000 ? base * 6 : 6000 ))
  SIM_MAX_CYCLES=$lim perl -e 'alarm shift; exec @ARGV' "${DEADLINE:-60}" "$SIM" "$b" > "$OUT/logs/$t.log" 2>&1; rc=$?
  c=$(grep -a -o 'sim-time(cycle)= *[0-9]*' "$OUT/logs/$t.log" | tail -1 | grep -o '[0-9]*$')
  if grep -aq "Success" "$OUT/logs/$t.log" && [ $rc -eq 0 -o $rc -eq 1 ]; then echo "PASS $t $c (base $base)"; else echo "FAIL $t base=$base got=${c:-none} rc=$rc"; fi
}
export -f one; export SIM OUT BASE DEADLINE
ls "$ROOT"/build/tests/*/*.bin "$ROOT"/build/vm/*/*.bin 2>/dev/null | grep -v "memory-stress/\|load-store/\|smc-" | xargs -P "$JOBS" -I{} bash -c 'one {}' > "$OUT/result.txt"
sort "$OUT/result.txt" -o "$OUT/result.txt"
grep FAIL "$OUT/result.txt"
echo "gate: pass=$(grep -c ^PASS "$OUT/result.txt") fail=$(grep -c ^FAIL "$OUT/result.txt")"
grep -q FAIL "$OUT/result.txt" && exit 1 || exit 0
