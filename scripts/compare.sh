#!/usr/bin/env bash
# Usage: compare.sh <baseline-dir> <new-dir>  (each has <x>.csv + logs/)
# Gate: per-test cycle count equal. Also reports whole-log byte differences.
B=$1; N=$2; bad=0; logdiff=0
bc=$(ls $B/*.csv | head -1); nc=$(ls $N/*.csv | head -1)
while IFS=, read -r t st w c; do
  [ "$t" = test ] && continue
  nl=$(grep "^$t," "$nc"); [ -z "$nl" ] && continue; nst=$(echo "$nl" | cut -d, -f2); nc_=$(echo "$nl" | cut -d, -f4)
  if [ "$c" != "$nc_" ] || [ "$st" != "$nst" ]; then echo "CYCLE MISMATCH $t base=$c/$st new=$nc_/$nst"; bad=$((bad+1)); fi
  cmp -s "$B/logs/$t.log" "$N/logs/$t.log" || logdiff=$((logdiff+1))
done < "$bc"
echo "tests=$(($(wc -l < $bc)-1)) cycle_mismatch=$bad logs_not_byte_identical=$logdiff"
awk -F, 'NR>1{w+=$3} END{print "wall_sum",w}' $bc; awk -F, 'NR>1{w+=$3} END{print "wall_sum_new",w}' $nc
[ $bad = 0 ]
