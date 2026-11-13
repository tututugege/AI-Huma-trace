#!/usr/bin/env bash
# Gate of the RVC=1 build on the C-compiled programs (scripts/build_rvc_tests.sh): every program must finish (exit 0 / 1) with exactly the
# cycle count of results/baseline/baseline_c.csv; difftest mismatches show up as a failure.
#   usage: [BASELINE=<csv>] [FULL=1] [WRITE=<csv>] run_gate_c.sh <simulator> <outdir> [jobs]
#   FULL=1 also runs the long generated tests (rvc-alu / rvc-mem / fd-rvc-mem, a few million cycles each); WRITE=<csv> records the cycles
#   instead of comparing (new baseline).
SIM=$1; OUT=$2; JOBS=${3:-4}
T=$(cd "$(dirname "$0")/.." && pwd); BASE=${BASELINE:-$T/results/baseline/baseline_c.csv}
rm -rf "$OUT"; mkdir -p "$OUT/logs"
one() {
  b=$1; t=$(basename "$b"); base=$(grep "^$t," "$BASE" 2>/dev/null | cut -d, -f4)
  lim=$(( ${base:-1000000} * 4 > 3000 ? ${base:-1000000} * 4 : 3000 )); [ -n "$WRITE" ] && lim=100000000
  SIM_MAX_CYCLES=$lim perl -e 'alarm shift; exec @ARGV' "${DEADLINE:-900}" "$SIM" "$b" > "$OUT/logs/$t.log" 2>&1; rc=$?
  c=$(grep -a -o 'sim-time(cycle)= *[0-9]*' "$OUT/logs/$t.log" | tail -1 | grep -o '[0-9]*$')
  if [ -n "$WRITE" ]; then echo "$t,$rc,0,${c:-NA}"; return; fi
  if [ "$c" = "$base" ] && [ $rc -eq 0 -o $rc -eq 1 ]; then echo "PASS $t $c"; else echo "FAIL $t base=$base got=${c:-none} rc=$rc"; fi
}
export -f one; export SIM OUT BASE DEADLINE WRITE
list=$(ls "$T"/build_c/tests/*/*.bin | grep -v "memory-stress/\|load-store/\|smc-")
list="$list $T/build_c/gen/rvc-flow/rvc-flow.bin $T/build_c/gen/rvc-straddle/rvc-straddle.bin $T/build_c/gen/rvc-sv32/rvc-sv32.bin"
[ -n "$FULL" ] && list="$list $T/build_c/gen/rvc-alu/rvc-alu.bin $T/build_c/gen/rvc-mem/rvc-mem.bin $T/build_c/gen/fd-rvc-mem/fd-rvc-mem.bin"
echo $list | tr ' ' '\n' | xargs -P "$JOBS" -I{} bash -c 'one {}' > "$OUT/result.txt"
if [ -n "$WRITE" ]; then { echo "test,status,wall_s,cycles"; sort "$OUT/result.txt"; } > "$WRITE"; echo "wrote $WRITE ($(wc -l < "$WRITE") lines)"; exit 0; fi
sort "$OUT/result.txt" -o "$OUT/result.txt"
grep FAIL "$OUT/result.txt"
echo "gate (C): pass=$(grep -c ^PASS "$OUT/result.txt") fail=$(grep -c ^FAIL "$OUT/result.txt")"
grep -q FAIL "$OUT/result.txt" && exit 1 || exit 0
