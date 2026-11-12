#!/usr/bin/env bash
# F/D (FP=1) checks of the timing model: the function model's FP tests (directed, trap / status, compiler-generated kernels, the
# official riscv-tests rv64uf / rv64ud) must run to the end (difftest Success, every instruction committed), and the micro-benchmarks
# in tests/fp/fpmicro.c must show the configured latencies.
#   usage: [XLEN=32|64] scripts/fp_check.sh <simulator built with the same XLEN and FP=1 (the default)> <work-dir>      (XLEN default 64)
#   env:   RISCV_TESTS (checkout with isa/, default ~/personal/simulator/baremetal/riscv-tests), LLVM, LLD
set -uo pipefail
SIM=$(cd "$(dirname "$1")" && pwd)/$(basename "$1"); W=${2:?work dir}; T=$(cd "$(dirname "$0")/.." && pwd); FM=$T/function-model
RT=${RISCV_TESTS:-$HOME/personal/simulator/baremetal/riscv-tests}; L=${LLVM:-/opt/homebrew/opt/llvm/bin}; LD=${LLD:-/opt/homebrew/bin/ld.lld}
XLEN=${XLEN:-64}
if [ "$XLEN" = 64 ]; then export CMODEL=medany TARGET=riscv64 ABI=lp64d; CT="--target=riscv64 -march=rv64imafd_zicsr_zifencei -mabi=lp64d -mcmodel=medany"; else export CMODEL=medlow TARGET=riscv32 ABI=ilp32d; CT="--target=riscv32 -march=rv32imafd_zicsr_zifencei -mabi=ilp32d -mcmodel=medlow"; fi
mkdir -p "$W/gen" "$W/env"
python3 "$FM/tests/rv64/gen_fp.py" "$W/gen" $XLEN >/dev/null; python3 "$FM/tests/rv64/gen_fp_trap.py" "$W/gen" $XLEN >/dev/null; cp "$FM/tests/rv64/fp-kernels.c" "$W/gen/"
export EXTRA_DIR="$W/gen"
ARCH=rv${XLEN}imafd_zicsr_zifencei bash "$T/scripts/build_tests.sh" "$T" "$W/t" rv$XLEN-fp rv$XLEN-fp-trap fp-kernels | grep -v ' OK$'
cp "$W/t/link.lds" "$W/env/link.lds"; $L/clang $CT -c "$T/baremetal/start.S" -o "$W/env/start.o"
pass=0; fail=0
run() {  # name bin max_cycles
  out=$(SIM_MAX_CYCLES=$3 perl -e 'alarm 280; exec @ARGV' "$SIM" "$2" 2>&1); rc=$?
  if [ $rc -eq 0 ] && echo "$out" | grep -aq "Success"; then pass=$((pass+1)); else fail=$((fail+1)); echo "FAIL $1 (rc=$rc)"; fi
}
for t in rv$XLEN-fp rv$XLEN-fp-trap fp-kernels; do run $t "$W/t/$t/$t.bin" 10000000; done
cat > "$W/env/riscv_test.h" <<HDR
#include "$RT/isa/riscv_test.h"
#define RVTEST_RV${XLEN}UF
#undef RVTEST_CODE_BEGIN
#define RVTEST_CODE_BEGIN .section .text; .globl main; main: li t0, 0x2000; csrs mstatus, t0;
HDR
for suite in rv${XLEN}uf rv${XLEN}ud; do for s in "$RT/isa/$suite"/*.S; do
  t=$suite-$(basename "$s" .S); d=$W/$t; mkdir -p "$d"
  [ "$suite" = rv32ud ] && [ "$(basename "$s" .S)" = move ] && continue   # (no such test upstream: fmv.x.d / fmv.d.x are rv64 only)
  $L/clang $CT -ffreestanding -nostdlib -I"$W/env" -I"$RT/isa/macros/scalar" -c "$s" -o "$d/t.o" 2>"$d/cc.log" &&
  $LD -T "$W/env/link.lds" --gc-sections "$W/env/start.o" "$d/t.o" -o "$d/t.elf" 2>"$d/ld.log" && $L/llvm-objcopy -O binary "$d/t.elf" "$d/t.bin" || { echo "BUILD FAIL $t"; fail=$((fail+1)); continue; }
  run $t "$d/t.bin" 300000
done; done
echo "fp programs (difftest Success): pass=$pass fail=$fail"
# micro-benchmarks: cycles per iteration from two iteration counts
declare -a NAME=("4 dependent fadd.d" "8 independent fadd.d" "dependent fdiv.d" "4 dependent fmadd.d" "fld + fmadd.d + fsd" "fcvt.l.d + addi + fcvt.d.l")
for k in 0 1 2 3 4 5; do
  for n in 200 400; do
    d=$W/mb/k${k}_$n; mkdir -p "$d"
    $L/clang $CT -O1 -ffreestanding -nostdlib -DKIND=$k -DN=$n -c "$T/tests/fp/fpmicro.c" -o "$d/t.o" &&
    $LD -T "$W/env/link.lds" --gc-sections "$W/env/start.o" "$d/t.o" -o "$d/t.elf" && $L/llvm-objcopy -O binary "$d/t.elf" "$d/t.bin"
    eval "c_$n=$(SIM_MAX_CYCLES=2000000 "$SIM" "$d/t.bin" 2>&1 | grep -a -o 'sim-time(cycle)= *[0-9]*' | tail -1 | grep -o '[0-9]*$')"
  done
  echo "micro ${NAME[$k]}: $(( (c_400 - c_200) / 200 )) cycles per iteration"
done
[ $fail -eq 0 ]
