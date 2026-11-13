#!/usr/bin/env bash
# The committed instruction stream must not depend on the front end's timing.  Run a program twice (two environment settings) with TB_PCTRACE and
# compare the pc streams; an entry whose fact is gone (pc 0) matches anything.
#   usage: scripts/stream_cmp.sh <simulator> <program.bin> "<env A>" "<env B>" [max cycles=4000000]
#   example: scripts/stream_cmp.sh build_rvc/simulator build_c/tests/quick-sort/quick-sort.bin "FE_PAIR=0" "FE_PAIR=1"
SIM=$1; BIN=$2; EA=$3; EB=$4; CYC=${5:-4000000}; T=$(mktemp -d)
env $EA TB_PCTRACE=$T/a SIM_MAX_CYCLES=$CYC "$SIM" "$BIN" > /dev/null 2>&1
env $EB TB_PCTRACE=$T/b SIM_MAX_CYCLES=$CYC "$SIM" "$BIN" > /dev/null 2>&1
python3 -I - "$T/a" "$T/b" "$(basename "$BIN")" <<'PY'
import sys
a=[l.split()[1] for l in open(sys.argv[1])]; b=[l.split()[1] for l in open(sys.argv[2])]
n=min(len(a),len(b)); i=0
# a run that ends earlier (cycle cap) only has to be a prefix of the other
while i<n and (a[i]=='0' or b[i]=='0' or a[i]==b[i]): i+=1
if i<n: print("DIFF %s at commit %d: A %s ... B %s" % (sys.argv[3], i, a[max(0,i-3):i+3], b[max(0,i-3):i+3])); sys.exit(1)
print("SAME %s (%d / %d commits compared)" % (sys.argv[3], n, max(len(a),len(b))))
PY
rc=$?; rm -rf "$T"; exit $rc
