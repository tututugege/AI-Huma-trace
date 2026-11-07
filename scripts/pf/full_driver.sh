#!/usr/bin/env bash
# full_driver.sh: full SPEC (580 points) with fast warm-up, pipelined with the checkpoint upload (fleet.sh pushall).
# For every benchmark in bench_order.txt (longest simulation first): wait until pushall.log says it is uploaded, then start the
# prefetcher-on configuration (full_h1, results in /root/res/full_h1) on the cloud host. The prefetcher-off baseline is the exact
# full run of 2026-11-05 (tm/res_dram60, identical to the exact cloud run on all 20 checked points); fast warm-up itself moves the
# absolute IPC by about -0.5% (geomean, -3.1%..+2.5% per point), which the comparison has to state.
# usage: full_driver.sh [pushall.log]      env: JOBS_BIG (mcf, default 24), JOBS (others, default 16), SKIP="bench ..." (already started)
HERE=$(cd "$(dirname "$0")" && pwd)
LOG=${1:-$HOME/personal/AI-Huma-data/cloud/pushall.log}
for b in $(cat "$HERE/bench_order.txt"); do
  case " ${SKIP:-} " in *" $b "*) continue;; esac
  until grep -q " $b uploaded" "$LOG" 2>/dev/null; do sleep 30; done
  j=${JOBS:-16}; [ "$b" = 429.mcf ] && j=${JOBS_BIG:-24}   # fast warm-up: mcf peaks at 3.25 GB per process (121 GB host)
  echo "$(date +%H:%M:%S) $b uploaded: starting full_h1 (jobs $j)"
  for cfg in full_h1:PF_ENABLE=1; do
    POINTS=points_full_$b.txt JOBS=$j "$HERE/cloud/fleet.sh" run all "${cfg%%:*}@build_fw:${cfg#*:},FASTWARM=1000000" 2>&1 | tail -1
  done
done
echo "$(date +%H:%M:%S) all benchmarks started"
