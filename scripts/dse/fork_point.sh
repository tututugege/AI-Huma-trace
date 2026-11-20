#!/bin/bash
# fork_point.sh <run> <bench> <ckpt>   (on an eda node; EXP-047, docs/DSE_FORK.md)
# One rv64gc SPEC06 SimPoint, all configurations of $R/tm/dse/<run>/cfg.txt from one warm-up: 10M detailed warm-up, fork at
# warm-up - DSE_SETTLE (200k), then 10M measured per configuration (same flow as rv64_final_one.sh: stub, PF on, PMU on).
# Output: $R/tm/dse/<run>/<bench>/<ckpt>/<cfg>.res = "<ckpt> <insts> <cycles>", the child log <cfg>/sim.log.gz, `done` when complete.
# BIN (default sim64_dsebig3) is the binary; its compiled sizes are the upper limits of the caps. MEAS = measured instructions (default 10M).
# The first line of cfg.txt is the baseline: the warm-up before the fork runs with it.
R=/nfs_global/S/houruyao/spec06_rv64gc; RUN=$1; B=$2; N=$3; BIN=${BIN:-$R/tm/bin/sim64_dsebig3}; MEAS=${MEAS:-10000000}
W=$R/tm/dse/$RUN; O=$W/$B/$N; S=$R/tm/stubs_llc1/$B
[ -f $O/done ] && exit 0
mkdir -p $O $S
[ -f /tmp/llcplru1_ds3/configs/DDR4_8Gb_x16_3200.ini ] || { mkdir -p /tmp/llcplru1_ds3/configs; cp $R/tm/ds3cfg/configs/DDR4_8Gb_x16_3200.ini /tmp/llcplru1_ds3/configs/; }
if [ ! -f $S/$N.stub ]; then
  python3 -I $R/tools/ckpt2stub.py $R/ckpt/$B/$N.gz $S/$N.stub.tmp$$ > /dev/null 2>&1 && mv $S/$N.stub.tmp$$ $S/$N.stub
fi
[ -f $S/$N.stub ] || { echo "$B $N STUBFAIL" >> $W/fail.txt; exit 1; }
case $N in *nowarmup*) ARGS="-w 0 -c $MEAS"; SETTLE=0;; *) ARGS="-c $MEAS"; SETTLE=${DSE_SETTLE:-200000};; esac
t0=$(date +%s)
cd $O && env PF_ENABLE=${PF_ENABLE:-1} PMU=1 PMU_OUT=pmu DS3_OUT=${DS3_OUT:-$O/ds3} DSE_CONFIGS=$W/cfg.txt DSE_OUT=$O DSE_SETTLE=$SETTLE \
  /usr/bin/time -v $BIN --mode ckpt --restore-stub $S/$N.stub $ARGS $R/ckpt/$B/$N.gz > $O/parent.log 2>&1
rc=$?
ok=1
for d in $(awk '!/^ *#/ && NF {print $1}' $W/cfg.txt); do
  L=$O/$d/sim.log
  [ -f $L ] || { ok=0; continue; }
  cyc=$(sed 's/\x1b\[[0-9;]*m//g' $L | grep -a "cycle       num:" | tail -1 | awk '{print $3}')
  ins=$(sed 's/\x1b\[[0-9;]*m//g' $L | grep -a "instruction num:" | tail -1 | awk '{print $3}')
  if [ -n "$cyc" ] && grep -aq Success $L; then echo "$N $ins $cyc" > $O/$d.res; gzip -f $L; else ok=0; echo "$B $N $d cyc=$cyc" >> $W/fail.txt; fi
done
[ $rc -eq 0 ] && [ $ok -eq 1 ] && echo "$(( $(date +%s) - t0 ))s" > $O/done
