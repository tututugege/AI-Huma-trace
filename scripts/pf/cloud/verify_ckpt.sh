#!/usr/bin/env bash
# verify_ckpt.sh: run ON a cloud machine. Every checkpoint must pass gzip -t AND be larger than 1 MB (an empty gzip passes -t);
# broken ones are regenerated from the zstd copy under /root/spec/zst/ckpt/... and checked again.
cd /root/spec || exit 1
check() { gzip -t "$1" 2>/dev/null && [ "$(stat -c %s "$1")" -gt 1000000 ]; }
export -f check
bad=$(find ckpt -name '*.gz' | xargs -P 16 -I{} bash -c 'check {} || echo {}')
for f in $bad; do
  z=zst/${f%.gz}.zst
  if [ ! -s "$z" ]; then echo "MISSING source $z"; continue; fi
  rm -f "$f"; zstd -d --long=27 -q -c "$z" | gzip -1 > "$f.tmp" && mv -f "$f.tmp" "$f"
  if check "$f"; then echo "fixed $f"; else echo "STILL BAD $f"; fi
done
total=$(find ckpt -name '*.gz' | wc -l)
ok=$(find ckpt -name '*.gz' | xargs -P 16 -I{} bash -c 'check {} && echo 1' | wc -l)
echo "checkpoints ok: $ok / $total"
