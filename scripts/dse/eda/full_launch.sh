#!/bin/bash
# full_launch.sh <node> <workers> <run>...: EXP-048 full SPEC06 int (386 points, 10M warm-up + 10M measure, L1 MSHR 8 / 16 forked)
#   base: sim64_dsebig3 (h1, LLC MSHR 8 = main default); bxp: sim64_bop12 (Berti VA + DTLB-port translation + LLC BOP, LLC MSHR 16)
# run dirs tm/dse/full_<run>; jobs tm/dse/full/jobs.txt
D=/nfs_global/S/houruyao/spec06_rv64gc/tm/dse; N=$1; K=$2; shift 2
B="rob=128 prf=128 fprf=96 br=32 ldq=64 stq=64 iq_int=32 iq_mem=64 iq_std=16 iq_br=32 iq_fp=32"
for v in "$@"; do
  W=$D/full_$v; mkdir -p $W
  [ -f $W/jobs.txt ] || { printf "base $B mshr=8\nmshr16 $B mshr=16\n" > $W/cfg.txt; cp $D/full/jobs.txt $W/jobs.txt; }
  case $v in
    base) BIN=$D/../bin/sim64_dsebig3; E="PF_ENABLE=1";;
    bxp)  BIN=$D/../bin/sim64_bop12; E="PF_KIND=2 PF_BERTI_VA=1 PF_BERTI_XPORT=1 PF_BERTI_LATEFIX=0 PF_BERTI_RFILT=0 LLC_BOP=1";;
    *) echo "unknown $v"; continue;;
  esac
  for i in $(seq 1 $K); do ssh -o BatchMode=yes eda-$N "cd $D && (env BIN=$BIN MEAS=10000000 $E setsid nohup ./worker.sh full_$v > /dev/null 2>&1 < /dev/null &)"; done
done
