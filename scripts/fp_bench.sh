#!/usr/bin/env bash
# Build the FP kernels of tests/fp/fpbench.c (rv32 or rv64, XLEN env) and run them on a simulator with the given FP_WB_PORTS values.
#   usage: [XLEN=32|64] scripts/fp_bench.sh <simulator> <work-dir> [ports...]      (ports default: 0 1 2 3 4 5 9; 0 = unlimited)
set -uo pipefail
SIM=$(cd "$(dirname "$1")" && pwd)/$(basename "$1"); W=${2:?work dir}; shift 2; PORTS=${*:-0 1 2 3 4 5 9}
T=$(cd "$(dirname "$0")/.." && pwd); XLEN=${XLEN:-32}; L=${LLVM:-/opt/homebrew/opt/llvm/bin}; LD=${LLD:-/opt/homebrew/bin/ld.lld}
if [ "$XLEN" = 64 ]; then CT="--target=riscv64 -march=rv64imafd_zicsr_zifencei -mabi=lp64d -mcmodel=medany"; export CMODEL=medany TARGET=riscv64 ABI=lp64d; else CT="--target=riscv32 -march=rv32imafd_zicsr_zifencei -mabi=ilp32d -mcmodel=medlow"; export CMODEL=medlow TARGET=riscv32 ABI=ilp32d; fi
mkdir -p "$W"; bash "$T/scripts/build_tests.sh" "$T" "$W/t" dummy >/dev/null 2>&1
$L/clang $CT -c "$T/baremetal/start.S" -o "$W/start.o"
NAMES=(dgemm saxpy-dot jacobi nbody fir mixed); REPS=(2 4 4 1 2 8)
printf "%-10s" kernel; for p in $PORTS; do printf " %10s" "ports=$p"; done; echo
for k in 0 1 2 3 4 5; do
  $L/clang $CT -O2 -ffp-contract=fast -fno-math-errno -ffreestanding -nostdlib -DKERNEL=$k -DREPS=${REPS[$k]} -c "$T/tests/fp/fpbench.c" -o "$W/k$k.o" &&
  $LD -T "$W/t/link.lds" --gc-sections "$W/start.o" "$W/k$k.o" -o "$W/k$k.elf" && $L/llvm-objcopy -O binary "$W/k$k.elf" "$W/k$k.bin" || { echo "build fail $k"; continue; }
  printf "%-10s" "${NAMES[$k]}"
  for p in $PORTS; do c=$(FP_WB_PORTS=$p SIM_MAX_CYCLES=5000000 "$SIM" "$W/k$k.bin" 2>&1 | grep -a -o 'sim-time(cycle)= *[0-9]*' | tail -1 | grep -o '[0-9]*$'); printf " %10s" "${c:-fail}"; done; echo
done
