#!/usr/bin/env bash

set -euo pipefail

target=axi-interconnect-kit/axi_interconnect/AXI_LLC.cpp
old='  io = {};'
new='  io = AXI_LLC_IO_t{};'

[[ -f $target ]] || {
  echo "missing AXI LLC source: $target" >&2
  exit 1
}

matches=$(grep -Fxc "$old" "$target" || true)
if [[ $matches != 1 ]]; then
  echo "expected exactly one GCC 13 compatibility target, found $matches" >&2
  exit 1
fi

sed -i "s/^  io = {};$/  io = AXI_LLC_IO_t{};/" "$target"
grep -Fqx "$new" "$target"
