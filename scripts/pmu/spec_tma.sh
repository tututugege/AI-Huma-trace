#!/usr/bin/env bash
# PMU/TMA of the highest-weight SimPoint of every SPEC program (front-door restore, 10M warm-up + 10M measured).
#   usage: spec_tma.sh <outdir> [jobs]     env: SIM (default build/simulator), DATA (default ~/personal/AI-Huma-data/spec-rv32imab)
# writes <outdir>/<bench>/{pmu_*.csv, run.log}; summary: python3 scripts/pmu/spec_summary.py <outdir>
set -u
OUT=$(cd "$1" 2>/dev/null && pwd || { mkdir -p "$1"; cd "$1" && pwd; }); JOBS=${2:-6}
T=$(cd "$(dirname "$0")/../.." && pwd); SIM=${SIM:-$T/build/simulator}
DATA=${DATA:-$HOME/personal/AI-Huma-data/spec-rv32imab}
one() {
  b=$1; id=$(awk '{ if ($1+0>m){m=$1+0;i=$2} } END{print i}' "$DATA/sp/$b.weights")
  c=$(ls "$DATA"/ckpt/$b/ckpt_sp${id}_*.gz | head -1); d=$OUT/$b; mkdir -p "$d"
  python3 -I "$T/function-model/tools/ckpt2stub.py" "$c" "$d/stub" > "$d/stub.log" 2>&1 || { echo "STUBFAIL $b"; return; }
  PMU=1 PMU_OUT=$d/pmu PMU_INTERVAL=1000000 "$SIM" --mode ckpt --restore-stub "$d/stub" -c 10000000 "$c" > "$d/run.log" 2>&1
  echo "done $b sp$id"
}
export -f one; export OUT T SIM DATA
ls "$DATA/ckpt" | grep -v '\.log$' | xargs -P "$JOBS" -I{} bash -c 'one {}'
