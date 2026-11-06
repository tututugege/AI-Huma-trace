#!/usr/bin/env bash
# pack.sh: stage the sources the cloud machines need into $OUT/stage/AI-Huma (small, a few MB), and list the checkpoints to push.
# Run on the Mac:   scripts/pf/cloud/pack.sh [out-dir]
set -euo pipefail
ROOT=${ROOT:-/Users/ruyaoh/personal/AI-Huma}
OUT=${1:-$HOME/personal/AI-Huma-data/cloud}
SPEC=${SPEC:-$HOME/personal/AI-Huma-data/spec-rv32imab}
ST=$OUT/stage/AI-Huma
rm -rf "$OUT/stage"; mkdir -p "$ST"

# trace-model sources (no build products, no history, no results)
rsync -a --exclude '.git' --exclude 'build' --exclude 'build_*' --exclude 'results' --exclude '*.o' --exclude '*.d' \
      --exclude 'third_party' --exclude '.claude' "$ROOT/trace-model/" "$ST/trace-model/"
# function-model: sources and tools only (the reference CPU library is rebuilt on Linux)
rsync -a --exclude '.git' --exclude 'build-spike' --exclude 'linux' --exclude 'SimPoint' --exclude 'docs' \
      --exclude '*.o' --exclude '*.d' --exclude 'a.out*' --exclude 'librefcpu.a' --exclude 'compile_commands.json' \
      "$ROOT/function-model/" "$ST/function-model/"
# the symlink trace-model/function-model -> ../function-model is kept by rsync -a
# DRAMsim3 (pinned submodule checkout of the DRAM-sim worktree)
mkdir -p "$ST/trace-model/third_party"
rsync -a --exclude '.git' --exclude 'build' "$ROOT/trace-model-dram/third_party/DRAMsim3/" "$ST/trace-model/third_party/DRAMsim3/"
# Linux x86_64 SoftFloat: the archive in git history before the macOS rebuild (commit 437930e)
git -C "$ROOT/function-model" show 437930e^:lib/softfloat/softfloat.a > "$ST/function-model/lib/softfloat/softfloat.a"
cp "$ST/function-model/lib/softfloat/softfloat.a" "$ST/trace-model/libs/softfloat.a"
# validation programs
mkdir -p "$ST/trace-model/validate" "$ST/rtl/programs/big"
cp "$ROOT/rtl/programs/big/dhrystone.bin" "$ROOT/rtl/programs/big/coremark.bin" "$ST/trace-model/validate/"
for t in memory-stress-small matmul_fpu prime; do cp "$ROOT/trace-model/build/tests/$t/$t.bin" "$ST/trace-model/validate/"; done

# checkpoint list (relative to $SPEC) for rsync --files-from
( cd "$ROOT/trace-model/scripts/pf"
  cat points.txt points_icode.txt points_guard.txt | awk '{print "ckpt/"$2"/"$3".gz"}' ) | sort -u > "$OUT/ckpt_list.txt"
# checkpoints recompressed with zstd -9 (about 31% of the gzip size); the cloud side converts them back to gzip
comp() { rel=$1; out="$OUT/zst/${rel%.gz}.zst"; [ -s "$out" ] && return 0; mkdir -p "$(dirname "$out")"
         gzip -dc "$SPEC/$rel" | zstd -9 --long=27 -T2 -q -f -o "$out.tmp" && mv "$out.tmp" "$out"; }
export -f comp; export OUT SPEC
xargs -P 4 -I{} bash -c 'comp {}' < "$OUT/ckpt_list.txt"
echo "zstd checkpoints: $(find "$OUT/zst" -name '*.zst' | wc -l) files, $(du -sh "$OUT/zst" | cut -f1)"
echo "staged: $(du -sh "$ST" | cut -f1) sources; $(wc -l < "$OUT/ckpt_list.txt") checkpoints:"
( cd "$SPEC" && xargs -I{} stat -f %z {} < "$OUT/ckpt_list.txt" | awk '{s+=$1} END {printf "  %.2f GB\n", s/1e9}' )
echo "out: $OUT"
