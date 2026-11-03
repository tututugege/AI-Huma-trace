#!/usr/bin/env bash
# Usage: check_run.sh <simulator-binary> <outdir> <test.bin>...
# Runs each test with TB_CHECK=1 (bridge check mode), records cycles and the [TB_CHECK] summary.
SIM=$1; OUT=$2; shift 2; mkdir -p "$OUT/logs"
echo "test,status,cycles,rd_cmp,br_cmp,addr_cmp,target_cmp,mis_rd,mis_br,mis_addr,mis_target,nofact,uid_conflict,commit_front_miss" > "$OUT/check.csv"
for t in "$@"; do n=$(basename "$t")
  TB_CHECK=1 perl -e 'alarm shift; exec @ARGV' "${TIMEOUT:-300}" "$SIM" "$t" > "$OUT/logs/$n.log" 2>&1; rc=$?
  L="$OUT/logs/$n.log"
  cyc=$(grep -a -o 'sim-time(cycle)= *[0-9]*' "$L" | tail -1 | grep -o '[0-9]*$')
  g() { grep -a "TB_CHECK\] compared" "$L" | tail -1 | grep -o "$1=[0-9]*" | head -1 | cut -d= -f2; }
  m() { grep -a "TB_CHECK\] compared" "$L" | tail -1 | sed 's/.*mismatches//' | grep -o "$1=[0-9]*" | cut -d= -f2; }
  nf=$(grep -a "skipped nofact" "$L" | grep -o 'nofact=[0-9]*' | cut -d= -f2)
  uc=$(grep -a "uid_conflict(" "$L" | grep -o 'rob map)=[0-9]*' | grep -o '[0-9]*$')
  cf=$(grep -a "TB_CHECK\] dispatch" "$L" | grep -o 'commit_front_miss=[0-9]*' | cut -d= -f2)
  echo "$n,$rc,${cyc:-NA},$(g rd),$(g br),$(g addr),$(g target),$(m rd),$(m br),$(m addr),$(m target),${nf:-NA},${uc:-NA},${cf:-NA}" >> "$OUT/check.csv"
done
python3 - "$OUT/check.csv" <<'PY'
import csv,sys
rows=list(csv.DictReader(open(sys.argv[1])))
def s(k): return sum(int(r[k]) for r in rows if r[k] not in('','NA'))
print("tests",len(rows),"| compared rd",s('rd_cmp'),"br",s('br_cmp'),"addr",s('addr_cmp'),"target",s('target_cmp'))
print("mismatches rd",s('mis_rd'),"br",s('mis_br'),"addr",s('mis_addr'),"target",s('mis_target'),"| nofact",s('nofact'),"uid_conflict",s('uid_conflict'),"commit_front_miss",s('commit_front_miss'))
bad=[r['test'] for r in rows if any(r[k] not in('','NA','0') for k in('mis_rd','mis_br','mis_addr','mis_target','uid_conflict','commit_front_miss','nofact')) or r['cycles']=='NA']
print("tests with issues:",len(bad),bad)
PY
