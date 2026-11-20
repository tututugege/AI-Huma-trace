#!/bin/bash
# worker.sh <run>: take points from tm/dse/<run>/jobs.txt one at a time (claim = mkdir claim/<bench>.<ckpt>, atomic), run fork_point.sh on each.
# Start several per node (setsid nohup); a point already claimed or done is skipped. MEAS etc. pass through the environment.
D=/nfs_global/S/houruyao/spec06_rv64gc/tm/dse; RUN=$1; W=$D/$RUN; mkdir -p $W/claim
while read b n w; do
  [ -n "$n" ] || continue
  mkdir $W/claim/$b.$n 2>/dev/null || continue
  echo "$(date +%T) $(hostname -s) start $b $n" >> $W/worker.log
  $D/fork_point.sh $RUN $b $n
  echo "$(date +%T) $(hostname -s) end $b $n rc=$?" >> $W/worker.log
done < $W/jobs.txt
