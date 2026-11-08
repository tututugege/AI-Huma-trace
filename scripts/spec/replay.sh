#!/usr/bin/env bash
# SPECint2006 checkpoint replay on the trace model (front-door restore, warmup = checkpoint interval, measure 10M insts).
#   replay.sh quick   fast iteration: 6 programs (perlbench gcc mcf hmmer sjeng h264ref), SimPoints covering 90% weight (178 points; count it with select_points.py, it changes with the checkpoint set)
#   replay.sh full    all 12 programs, all SimPoints (496 points, ~75 min @ 8 parallel)
# env: SPEC=~/personal/AI-Huma-data/spec-rv32imab (ckpt/<bench>/*.gz and sp/<bench>.weights)  OUT=results dir  JOBS=8  SIM=simulator binary
# Results go to $OUT/<bench>/<ckpt>.ok (done points are skipped); failures to $OUT/fail.txt. Then agg.py prints weighted IPC.
set -u
MODE=${1:-quick}
HERE=$(cd "$(dirname "$0")" && pwd)
export SPEC=${SPEC:-$HOME/personal/AI-Huma-data/spec-rv32imab}; export OUT=${OUT:-$SPEC/tm/res} SIM=${SIM:-$HERE/../../build/simulator}
JOBS=${JOBS:-8}
case $MODE in
  quick) COV=0.9; BENCH="400.perlbench 403.gcc 429.mcf 456.hmmer 458.sjeng 464.h264ref";;
  full)  COV=1.0; BENCH="400.perlbench 401.bzip2 403.gcc 429.mcf 445.gobmk 456.hmmer 458.sjeng 462.libquantum 464.h264ref 471.omnetpp 473.astar 483.xalancbmk";;
  *) echo "usage: $0 quick|full"; exit 2;;
esac
mkdir -p "$OUT"
python3 -I "$HERE/select_points.py" "$SPEC" $COV $BENCH | xargs -P "$JOBS" -n 1 "$HERE/replay_one.sh"
python3 -I "$HERE/agg.py" "$SPEC" "$OUT" $COV $BENCH
