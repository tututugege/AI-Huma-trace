#!/usr/bin/env bash
# EXP-046: PMU=1 on every RVC gate program, collect the ISS_REF_* / ISS_LOST_* counters (issue grants refused after select).
#   usage: wb_refuse_stats.sh <simulator> <outdir (new)> [jobs]
SIM=$(cd "$(dirname "$1")" && pwd)/$(basename "$1"); OUT=$2; JOBS=${3:-4}
T=$(cd "$(dirname "$0")/.." && pwd)
mkdir "$OUT" || exit 1
one() {
  b=$1; t=$(basename "$b" .bin); d="$OUT/$t"; mkdir -p "$d"
  (cd "$d" && PMU=1 PMU_PCPROF=0 SIM_MAX_CYCLES=50000000 "$SIM" "$b" > run.log 2>&1)
}
export -f one; export SIM OUT
list=$(ls "$T"/build_c/tests/*/*.bin | grep -v "memory-stress/\|load-store/\|smc-")
echo $list | tr ' ' '\n' | xargs -P "$JOBS" -I{} bash -c 'one {}'
python3 - "$OUT" <<'PY'
import csv, glob, os, sys
out = sys.argv[1]
keys = ["CYCLES","INSTRET","ISS_REF_FU","ISS_REF_WB","ISS_REF_FPWB","ISS_REF_AGU","ISS_LOST_FU","ISS_LOST_WB","ISS_LOST_FPWB","ISS_LOST_AGU"]
rows = []
for f in sorted(glob.glob(os.path.join(out, "*", "pmu_events.csv"))):
    ev = {}
    for r in csv.reader(open(f)):
        if len(r) >= 3 and r[0] in keys:
            try: ev[r[0]] = int(r[2])
            except ValueError: pass
    rows.append((os.path.basename(os.path.dirname(f)), ev))
with open(os.path.join(out, "summary.csv"), "w") as w:
    w.write("prog," + ",".join(keys) + "\n")
    for n, ev in rows: w.write(n + "," + ",".join(str(ev.get(k, 0)) for k in keys) + "\n")
tot = {k: sum(ev.get(k, 0) for _, ev in rows) for k in keys}
print("programs", len(rows)); print({k: tot[k] for k in keys})
PY
