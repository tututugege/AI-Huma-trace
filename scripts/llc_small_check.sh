#!/usr/bin/env bash
# Usage: llc_small_check.sh <simulator-repo> <baseline-sim-src>
# The default 8 MiB LLC never evicts in the small tests, so DDR writes / victim write-backs are unexercised by the gate.
# This builds both simulators with a 64 KiB LLC and compares cycle counts on tests that do evict.
SIM=$1; BASE=$2; T=$(cd "$(dirname "$0")/.." && pwd)
F="-DCONFIG_AXI_LLC_SIZE_BYTES=65536"
(cd "$SIM" && gmake BUILD_DIR=build_small EXTRA_CXXFLAGS="$F" LDFLAGS="-lz" -j8 2>&1 | grep -E " error|Linking") &
(cd "$BASE" && gmake BUILD_DIR=build_small EXTRA_CXXFLAGS="$F" LDFLAGS="-lz" -j8 2>&1 | grep -E " error|Linking") &
wait
for t in memory-stress-small dcache-conflict-writeback dcache-mshr-pressure memory-boundary load-store; do
  b=$(SIM_MAX_CYCLES=5000000 "$BASE/build_small/simulator" "$T/build/tests/$t/$t.bin" 2>&1 | grep -a -o 'sim-time(cycle)= *[0-9]*' | tail -1 | grep -o '[0-9]*$')
  n=$(SIM_MAX_CYCLES=5000000 "$SIM/build_small/simulator" "$T/build/tests/$t/$t.bin" 2>&1 | grep -a -o 'sim-time(cycle)= *[0-9]*' | tail -1 | grep -o '[0-9]*$')
  echo "$t base=$b new=${n:-none} $([ "$b" = "$n" ] && echo OK || echo FAIL)"
done
