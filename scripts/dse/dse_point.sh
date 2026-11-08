#!/usr/bin/env bash
# dse_point.sh (ON the server) <cfg-name> <bench> <ckpt>: one SPEC point with PMU=1 -> /root/dse/res/<cfg>/<bench>/<ckpt>.{res,events.csv,hist.csv,log}
C=$1; B=$2; N=$3; T=/root/dse/AI-Huma/trace-model; S=/root/spec
O=/root/dse/res/$C/$B; mkdir -p $O; [ -f $O/$N.res ] && exit 0
STUB=$S/tm/pf_stubs/$B/$N.stub
PMU=1 PMU_OUT=$O/$N.pmu "$T/build_dse_$C/simulator" --mode ckpt --restore-stub $STUB -c 10000000 $S/ckpt/$B/$N.gz > $O/$N.log 2>&1
rc=$?; cl() { sed 's/\x1b\[[0-9;]*m//g' $O/$N.log; }
cyc=$(cl | grep -a "cycle       num:" | tail -1 | awk '{print $3}'); ins=$(cl | grep -a "instruction num:" | tail -1 | awk '{print $3}')
if [ $rc -eq 0 ] && [ -n "$cyc" ] && cl | grep -aq Success; then
  echo "$N $ins $cyc" > $O/$N.res; mv $O/$N.pmu_events.csv $O/$N.events.csv; mv $O/$N.pmu_hist.csv $O/$N.hist.csv 2>/dev/null; rm -f $O/$N.pmu_*
else echo "$C $B $N rc=$rc cyc=$cyc" >> /root/dse/res/fail.txt; fi
