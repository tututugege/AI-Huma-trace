#!/usr/bin/env bash
# Build the C-extension (RVC=1) test programs: the cpu-tests and extras compiled with a C (rv32imac) and the function-model's generated
# stress tests (all 16-bit encodings, loads / stores, control transfers, instructions straddling a cache line / page).
#   usage: scripts/build_rvc_tests.sh      -> build_c/tests/<t>/<t>.bin, build_c/gen/<t>/<t>.bin
set -uo pipefail
T=$(cd "$(dirname "$0")/.." && pwd); cd "$T"
GEN=${GEN_RVC:-$T/function-model/tests/rv64/gen_rvc.py}
export ARCH=rv32imac_zicsr_zifencei
bash scripts/build_tests.sh . build_c/tests | grep -v ' OK$'
bash scripts/build_tests.sh . build_c/tests memory-stress-small smc-addi smc-div pf-stress itt-dispatch load-store-aligned | grep -v ' OK$'
bash scripts/build_tests.sh . build_c/tests fd-basic fd-fcvt fd-fdiv fd-fsqrt fd-matmul fd-cold fdoff | grep -v ' OK$'
W=$(mktemp -d); python3 "$GEN" "$W" 32 >/dev/null
EXTRA_DIR="$T/tests/rvc" bash scripts/build_tests.sh . build_c/gen rvc-fencei rvc-irq | grep -v " OK$"
EXTRA_DIR="$W" bash scripts/build_tests.sh . build_c/gen rvc-alu rvc-mem fd-rvc-mem rvc-flow rvc-straddle rvc-sv32 | grep -v ' OK$'
rm -rf "$W"
