#!/usr/bin/env bash
# Build upstream baremetal cpu-tests with Homebrew LLVM (riscv64) instead of a gcc toolchain.
# Usage: build_tests.sh <simulator-repo> <outdir> [test...]   (default: all cpu-tests)
set -uo pipefail
SIMREPO=$1; OUT=$2; shift 2
L=${LLVM:-/opt/homebrew/opt/llvm/bin}
BM=$SIMREPO/baremetal; TD=$BM/test/cpu-tests; EX=${EXTRA_DIR:-$(cd "$(dirname "$0")/.." && pwd)/tests/extra}
ARCH=${ARCH:-rv64ima_zicsr_zifencei}
mkdir -p "$OUT"
sed -e 's/(wxa!ri)/(wxa)/' -e 's/^\( *\)\. = __stack_size;/\1. += __stack_size;/' -e 's/\(\.stack ORIGIN[^:]*\):/\1 (NOLOAD) :/' "$BM/link.lds" > "$OUT/link.lds.0"
python3 - "$OUT/link.lds.0" "$OUT/link.lds" <<'PY'
import sys
t=open(sys.argv[1]).read()
t=t.replace("  .data ","  .rodata : { *(.rodata .rodata.* .srodata .srodata.*) } >ram AT>ram\n  .data ",1)
t=t.replace("  .dma_buffer","  .bss (NOLOAD) : { *(.sbss .sbss.* .bss .bss.* COMMON) . = ALIGN(8); } >ram AT>ram\n\n  .dma_buffer",1)
open(sys.argv[2],"w").write(t)
PY
tests=("$@"); [ ${#tests[@]} -eq 0 ] && tests=($(cd "$TD" && ls *.c | sed 's/\.c$//'))
CF="--target=${TARGET:-riscv64} -march=$ARCH -mabi=${ABI:-lp64} -O2 -ffp-contract=off -mcmodel=${CMODEL:-medany} -mstrict-align -ffreestanding -fno-builtin -nostdlib -I$BM -I$BM/include"
# F/D tests (fd-*, tests/extra): the same flags with the F and D extensions and the double-float ABI
CFD=${CF/-march=$ARCH -mabi=${ABI:-lp64}/-march=${ARCH/ima/imafd} -mabi=${ABI:-lp64}d}
for t in "${tests[@]}"; do
  d=$OUT/$t; mkdir -p "$d"; objs=()
  D=$TD; [ -f "$EX/$t.c" ] && D=$EX
  srcs=("$BM/start.S" "$BM"/lib/*.c "$D/$t.c"); [ -f "$D/$t-helper.S" ] && srcs+=("$D/$t-helper.S")
  ok=1
  for s in "${srcs[@]}"; do
    o=$d/$(basename "$s").o
    cf=$CF; case $t in fd-*) cf=$CFD;; esac
    $L/clang $cf -c "$s" -o "$o" 2>"$d/cc.log" || { ok=0; break; }; objs+=("$o")
  done
  [ $ok = 1 ] && ${LLD:-/opt/homebrew/bin/ld.lld} -T "$OUT/link.lds" --gc-sections "${objs[@]}" -o "$d/$t.elf" 2>"$d/ld.log" \
    && $L/llvm-objcopy -O binary "$d/$t.elf" "$d/$t.bin" || ok=0
  echo "$t $([ $ok = 1 ] && echo OK || echo FAIL)"
done
