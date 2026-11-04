#!/usr/bin/env bash
# Usage: bpu_diff.sh <simulator-repo> [cycles] [seeds...]   builds the BPU differential test and runs it
SIM=$1; CYC=${2:-1000000}; shift 2; SEEDS=${@:-1 2 3 4}
T=$(cd "$(dirname "$0")/.." && pwd); mkdir -p "$T/build"
cd "$SIM" || exit 1
INC="-I./include -I./libs/include -I./back-end/include -I./back-end/Exu/include -I./back-end/Lsu/include -I./back-end/tools/include -I./MemSubSystem/include -I./diff/include -I./function-model/include/api -I./legacy/mmu/include -I./front-end -I"$T/tests" -I./axi-interconnect-kit/include -I./axi-interconnect-kit/axi_interconnect/include -I./axi-interconnect-kit/sim_ddr/include -I./axi-interconnect-kit/mmio/include"
clang++ -std=c++2a -O2 -w $INC "$T/tests/bpu_diff.cpp" -o "$T/build/bpu_diff" || exit 1
for s in $SEEDS; do "$T/build/bpu_diff" "$CYC" "$s" & done; wait
