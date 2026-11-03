#!/usr/bin/env bash
# Usage: bridge_selftest.sh <simulator-repo> [wrong_path_percent]   builds and runs the bridge self-test on every built test
SIM=$1; P=${2:-50}; T=$(cd "$(dirname "$0")/.." && pwd)
cd "$SIM" && clang++ -std=c++2a -O1 -w -I GenSimpoint/include/api -I "$T/src" "$T/tests/bridge_selftest.cpp" "$T/src/TraceBridge.cpp" GenSimpoint/librefcpu.a libs/softfloat.a -lz -o "$T/build/bridge_selftest" || exit 1
pass=0; fail=0
for b in "$T"/build/tests/*/*.bin; do t=$(basename "$b" .bin); case $t in memory-stress|load-store) continue;; esac
  r=$(perl -e 'alarm 120; exec @ARGV' "$T/build/bridge_selftest" "$b" 300000 "$P" 3 2>&1 | grep -a "insts=" | head -1)
  if echo "$r" | grep -q "mismatches=0 regs_equal=1 csr/pc_equal=1 mem_equal=1"; then pass=$((pass+1)); else fail=$((fail+1)); echo "FAIL $t: $r"; fi
done
echo "bridge selftest wrong_path=$P%: pass=$pass fail=$fail"
