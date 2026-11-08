#!/usr/bin/env bash
# local_run.sh <jobs> <points.txt> <cfg>...   (Mac) run every point of <points.txt> for each config with PMU=1.
# Simulators: trace-model-dse/build_dse_<cfg>/simulator. Results: $D/res/<cfg>/<bench>/<ckpt>.{res,events.csv,hist.csv,log}
# (same layout as the server runs, so dse_report.py works on it).
T=$(cd "$(dirname "$0")/../.." && pwd); J=$1; PT=$2; shift 2
SPEC=${SPEC:-$HOME/personal/AI-Huma-data/spec-rv32imab}; D=${D:-$HOME/personal/AI-Huma-data/dse}
export T SPEC D
one() {
  C=$1; B=$2; N=$3; O=$D/res/$C/$B; mkdir -p "$O"; [ -f "$O/$N.res" ] && return 0
  PMU=1 PMU_OUT=$O/$N.pmu "$T/build_dse_$C/simulator" --mode ckpt --restore-stub "$SPEC/tm/pf_stubs/$B/$N.stub" -c 10000000 "$SPEC/ckpt/$B/$N.gz" > "$O/$N.log" 2>&1
  rc=$?; cl() { sed 's/\x1b\[[0-9;]*m//g' "$O/$N.log"; }
  cyc=$(cl | grep -a "cycle       num:" | tail -1 | awk '{print $3}'); ins=$(cl | grep -a "instruction num:" | tail -1 | awk '{print $3}')
  if [ $rc -eq 0 ] && [ -n "$cyc" ] && cl | grep -aq Success; then
    echo "$N $ins $cyc" > "$O/$N.res"; mv "$O/$N.pmu_events.csv" "$O/$N.events.csv"; mv "$O/$N.pmu_hist.csv" "$O/$N.hist.csv" 2>/dev/null; rm -f "$O/$N".pmu_*
    echo "ok $C $B $N $cyc"
  else echo "FAIL $C $B $N rc=$rc" | tee -a "$D/fail_local.txt"; fi
}
export -f one
for p in 429.mcf 462.libquantum 401.bzip2 403.gcc 473.astar 445.gobmk 471.omnetpp 483.xalancbmk 464.h264ref 400.perlbench 458.sjeng 456.hmmer; do
  for c in "$@"; do awk -v b=$p -v c=$c '$2==b{print c, $2, $3}' "$PT"; done
done | xargs -P "$J" -L 1 bash -c 'one "$@"' _
