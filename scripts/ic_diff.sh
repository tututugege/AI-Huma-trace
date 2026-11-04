#!/usr/bin/env bash
# Usage: ic_diff.sh <simulator-repo> [cycles] [seeds...]   builds the interconnect differential (live vs frozen original) and runs it
SIM=$1; CYC=${2:-1000000}; shift 2; SEEDS=${@:-1 2 3 4}
T=$(cd "$(dirname "$0")/.." && pwd); mkdir -p "$T/build"
cd "$SIM" || exit 1
INC="-I./include -I./libs/include -I./back-end/include -I./back-end/Exu/include -I./back-end/Lsu/include -I./back-end/tools/include -I./MemSubSystem/include -I./diff/include -I./function-model/include/api -I./legacy/mmu/include -I./front-end -I./axi-interconnect-kit/include -I./axi-interconnect-kit/axi_interconnect/include -I./axi-interconnect-kit/sim_ddr/include -I./axi-interconnect-kit/mmio/include -I$T/tests/ic_ref"
K=axi-interconnect-kit/axi_interconnect
clang++ -std=c++2a -O2 -w -DNDEBUG -DAXI_KIT_DDR_BASE=0x80000000u -DAXI_KIT_SIM_DDR_BEAT_BYTES=32 $INC "$T/tests/ic_diff.cpp" $K/AXI_Interconnect.cpp "$T/tests/ic_ref/AXI_InterconnectRef.cpp" $K/AXI_LLC.cpp -o "$T/build/ic_diff" || exit 1
for s in $SEEDS; do "$T/build/ic_diff" "$CYC" "$s" & done; wait
