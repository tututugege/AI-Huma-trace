#!/usr/bin/env bash
# FE_PAIR=1 on the C programs (RVC=1 build): the committed instruction stream must equal the FE_PAIR=0 one (scripts/stream_cmp.sh) and the run must have no
# TB_CHECK mismatches.  The programs whose behaviour depends on the interrupt timing are left out.
#   usage: run_pair_c.sh <rvc simulator> [jobs=4]
SIM=$1; JOBS=${2:-4}; T=$(cd "$(dirname "$0")/.." && pwd); cd "$T"
ls build_c/tests/*/*.bin build_c/gen/*/*.bin | grep -v "memory-stress/\|load-store/\|smc-\|rvc-mem\|interrupt\|timer\|wfi" \
  | xargs -P "$JOBS" -I{} bash scripts/stream_cmp.sh "$SIM" {} "FE_PAIR=0" "FE_PAIR=1" 3000000 > /tmp/run_pair_c.txt 2>&1
grep DIFF /tmp/run_pair_c.txt | cut -c1-200
echo "FE_PAIR stream check: same=$(grep -c ^SAME /tmp/run_pair_c.txt) different=$(grep -c ^DIFF /tmp/run_pair_c.txt)"
! grep -q ^DIFF /tmp/run_pair_c.txt
