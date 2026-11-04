#!/usr/bin/env bash
# Parallel gate with per-test limits derived from the baseline.
# Usage: run_gate.sh <simulator-binary> <outdir> [jobs]
# - each test gets SIM_MAX_CYCLES = max(3000, 4 x baseline cycles): a hang is reported at once (exit 3)
# - one hard deadline per test as a last resort (default 20 s)
# - prints PASS/FAIL per test, a summary, and exits non-zero on any failure
SIM=$1; OUT=$2; JOBS=${3:-8}
T=$(cd "$(dirname "$0")/.." && pwd); BASE=$T/results/baseline/baseline.csv
rm -rf "$OUT"; mkdir -p "$OUT/logs"
one() {
  b=$1; t=$(basename "$b"); base=$(grep "^$t," "$BASE" | cut -d, -f4)
  lim=$(( base * 4 > 3000 ? base * 4 : 3000 ))
  SIM_MAX_CYCLES=$lim perl -e 'alarm shift; exec @ARGV' "${DEADLINE:-20}" "$SIM" "$b" > "$OUT/logs/$t.log" 2>&1; rc=$?
  c=$(grep -a -o 'sim-time(cycle)= *[0-9]*' "$OUT/logs/$t.log" | tail -1 | grep -o '[0-9]*$')
  if [ "$c" = "$base" ] && [ $rc -eq 0 -o $rc -eq 1 ]; then echo "PASS $t $c"; else echo "FAIL $t base=$base got=${c:-none} rc=$rc"; fi
}
export -f one; export SIM OUT BASE DEADLINE
ls "$T"/build/tests/*/*.bin "$T"/build/vm/*/*.bin 2>/dev/null | grep -v "memory-stress/\|load-store/\|smc-" | xargs -P "$JOBS" -I{} bash -c 'one {}' > "$OUT/result.txt"
sort "$OUT/result.txt" -o "$OUT/result.txt"
grep FAIL "$OUT/result.txt"
echo "gate: pass=$(grep -c ^PASS "$OUT/result.txt") fail=$(grep -c ^FAIL "$OUT/result.txt")"
grep -q FAIL "$OUT/result.txt" && exit 1 || exit 0
