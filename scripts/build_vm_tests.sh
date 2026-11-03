#!/usr/bin/env bash
# Build the upstream baremetal vm-* (paging) tests with Homebrew LLVM.
# Usage: build_vm_tests.sh <simulator-repo> <outdir> [link.lds]
set -o pipefail
SIM=$1; OUT=$2; LDS=${3:-$(cd "$(dirname "$0")/.." && pwd)/build/tests/link.lds}
BM=$SIM/baremetal; L=${LLVM:-/opt/homebrew/opt/llvm/bin}; LLD=${LLD:-/opt/homebrew/bin/ld.lld}
mkdir -p "$OUT"
for d in vm-asid-test vm-pagefault-test vm-sfence-perm-test vm-sum-fetch-test; do
  src=$BM/$d; name=$(grep '^TARGET' $src/Makefile | sed 's/.*= *//')
  extra=(); [ "$d" = vm-asid-test ] && extra=("$BM/vm-pagefault-test/vm_helpers.S")
  CF="--target=riscv32 -march=rv32imab_zicsr_zifencei -mabi=ilp32 -O0 -mcmodel=medlow -mstrict-align -ffreestanding -fno-builtin -nostdlib -I$src -I$BM -I$BM/include -std=gnu99 -fgnu89-inline"
  o=$OUT/$d; mkdir -p $o; objs=(); ok=1
  for s in $BM/start.S $BM/lib/*.c $src/*.c $src/*.S "${extra[@]}"; do
    [ -f "$s" ] || continue
    x=$o/$(basename "$s").o
    # -std only valid for C
    case $s in *.S) F=${CF/-std=gnu99/};; *) F=$CF;; esac
    $L/clang $F -c "$s" -o "$x" 2>>"$o/cc.log" || { ok=0; echo "cc fail $s"; tail -3 "$o/cc.log"; break; }
    objs+=("$x")
  done
  [ $ok = 1 ] && $LLD -T "$LDS" --gc-sections "${objs[@]}" -o "$o/$name.elf" 2>"$o/ld.log" && $L/llvm-objcopy -O binary "$o/$name.elf" "$o/$name.bin" && echo "$d OK $(stat -f%z $o/$name.bin) bytes" || echo "$d FAIL"
done
