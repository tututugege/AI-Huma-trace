#!/usr/bin/env bash
# usage: tm_one.sh <ckpt.gz>   -> tm/res/<bench>/<name>.log  (+ .done line in tm/res/<bench>.tsv)
SIM=${SIM:-/Users/ruyaoh/personal/AI-Huma/trace-model/build/simulator}
C=$1; b=$(basename $(dirname $C)); n=$(basename $C .gz)
O=${OUT:-/tmp/specref/tm/res}/$b; mkdir -p $O
[ -f $O/$n.ok ] && exit 0
python3 -I /Users/ruyaoh/personal/AI-Huma/function-model/tools/ckpt2stub.py $C $O/$n.stub > $O/$n.stub.log 2>&1 || { echo "$b $n STUBFAIL" >> ${OUT:-/tmp/specref/tm/res}/fail.txt; exit 1; }
case $n in *nowarmup*) ARGS="-w 0 -c 10000000";; *) ARGS="-c 10000000";; esac
$SIM --mode ckpt --restore-stub $O/$n.stub $ARGS $C > $O/$n.log 2>&1
rc=$?
cyc=$(grep -a "^.*cycle       num:" $O/$n.log | tail -1 | sed 's/\x1b\[[0-9;]*m//g' | awk '{print $3}')
ins=$(grep -a "instruction num:" $O/$n.log | tail -1 | sed 's/\x1b\[[0-9;]*m//g' | awk '{print $3}')
mm=$(grep -a "compared rd" $O/$n.log | tail -1 | sed 's/.*mismatches //')
if [ $rc -eq 0 ] && [ -n "$cyc" ] && grep -aq "Success" $O/$n.log; then
  echo "$n $ins $cyc $mm" > $O/$n.ok; rm -f $O/$n.stub
else echo "$b $n rc=$rc cyc=$cyc" >> ${OUT:-/tmp/specref/tm/res}/fail.txt; fi
