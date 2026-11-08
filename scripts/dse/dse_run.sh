#!/usr/bin/env bash
# dse_run.sh (ON the server) <configs.txt> <points.txt> <jobs> [only-config...]: all (config,point) pairs, slow benchmarks first, <jobs> in parallel
HERE=$(cd "$(dirname "$0")" && pwd); CF=$1; PT=$2; J=${3:-60}; shift 3
cfgs=$(grep -v '^#' $CF | grep -v '^$' | cut -d'|' -f1); [ $# -gt 0 ] && cfgs=$*
for p in $(cat /root/dse/AI-Huma/trace-model/scripts/pf/bench_order.txt); do awk -v b=$p '$2==b{print $2, $3}' $PT; done > /root/dse/pts.txt
for c in $cfgs; do while read b n; do [ -f /root/dse/res/$c/$b/$n.res ] || echo "$c $b $n"; done < /root/dse/pts.txt; done \
  | xargs -P $J -L 1 bash $HERE/dse_point.sh
echo "done $(date)" >> /root/dse/DONE.txt
