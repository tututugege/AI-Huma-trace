#!/usr/bin/env bash
# Usage: llc_diff.sh <simulator-repo> [cycles] [seeds...]   builds the LLC differential (live vs frozen original) and runs it
SIM=$1; CYC=${2:-2000000}; shift 2; SEEDS=${@:-1 2 3 4}
T=$(cd "$(dirname "$0")/.." && pwd); mkdir -p "$T/build"
cd "$SIM" || exit 1
INC="-I./include -I./libs/include -I./back-end/include -I./back-end/Exu/include -I./back-end/Lsu/include -I./back-end/tools/include -I./MemSubSystem/include -I./diff/include -I./GenSimpoint/include/api -I./legacy/mmu/include -I./front-end -I./axi-interconnect-kit/include -I./axi-interconnect-kit/axi_interconnect/include -I./axi-interconnect-kit/sim_ddr/include -I./axi-interconnect-kit/mmio/include -I$T/tests/llc_ref"
clang++ -std=c++2a -O2 -w -DNDEBUG $EXTRA $INC "$T/tests/llc_diff.cpp" axi-interconnect-kit/axi_interconnect/AXI_LLC.cpp "$T/tests/llc_ref/AXI_LLCRef.cpp" -o "$T/build/llc_diff" || exit 1
for s in $SEEDS; do "$T/build/llc_diff" "$CYC" "$s" & done; wait
