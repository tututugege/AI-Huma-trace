#!/usr/bin/env bash
# Usage: ddr_diff.sh <simulator-repo> [cycles]   random AXI masters against the compact SimDDR and the frozen original
SIM=$1; CYC=${2:-3000000}; T=$(cd "$(dirname "$0")/.." && pwd); mkdir -p "$T/build"
cd "$SIM" || exit 1
INC="-I./include -I./libs/include -I./back-end/include -I./MemSubSystem/include -I./diff/include -I./front-end -I./axi-interconnect-kit/include -I./axi-interconnect-kit/axi_interconnect/include -I./axi-interconnect-kit/sim_ddr/include -I$T/tests/ddr_ref"
clang++ -std=c++2a -O2 -w $INC "$T/tests/ddr_diff.cpp" axi-interconnect-kit/sim_ddr/SimDDR.cpp "$T/tests/ddr_ref/SimDDRRef.cpp" -o "$T/build/ddr_diff" || exit 1
# seed, ar%, aw%, w%, rready%, bready%, max burst, read-phase quarters (4 = reads always allowed)
"$T/build/ddr_diff" "$CYC" 1 30 20 60 70 70 4 4 &
"$T/build/ddr_diff" "$CYC" 2 80 80 90 30 30 8 4 &
"$T/build/ddr_diff" "$CYC" 3 10 60 95 90 20 2 4 &
"$T/build/ddr_diff" "$CYC" 4 95 10 20 10 90 6 4 &
"$T/build/ddr_diff" "$CYC" 5 50 50 50 50 50 16 4 &
"$T/build/ddr_diff" "$CYC" 6 100 100 100 100 100 3 4 &
"$T/build/ddr_diff" "$CYC" 7 40 70 90 80 80 4 1 &
"$T/build/ddr_diff" "$CYC" 8 60 90 100 50 50 8 1 &
"$T/build/ddr_diff" "$CYC" 9 90 100 100 90 90 3 2 &
"$T/build/ddr_diff" "$CYC" 10 30 100 100 100 100 16 1 &
"$T/build/ddr_diff" "$CYC" 11 20 40 100 70 20 2 1 &
wait
