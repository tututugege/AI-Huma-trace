#!/usr/bin/env bash
# EXP-011 fast-warmup validation: SPEC points (the EXP-010 set: coverage 0.25 = 30 points of 8 programs, or POINTS=<file> of
# checkpoint paths) with one configuration. Results in $SPEC/tm/fw_<tag>; per-point wall/user/sys seconds in <point>.time.
# usage: fw_eval.sh <tag> [ENV=..]...   env: FASTW=<n> (detailed warm-up; unset = exact 10M), SIM=<simulator>, JOBS (default 4)
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
export SPEC=${SPEC:-$HOME/personal/AI-Huma-data/spec-rv32imab}; export SIM=${SIM:-$HERE/../../build/simulator}
TAG=$1; shift
export OUT=$SPEC/tm/fw_$TAG; mkdir -p "$OUT"
for kv in "$@"; do export "$kv"; done
if [ -n "${POINTS:-}" ]; then cat "$POINTS"
else python3 -I "$HERE/select_points.py" "$SPEC" "${COV:-0.25}" 400.perlbench 403.gcc 429.mcf 456.hmmer 458.sjeng 445.gobmk 473.astar 464.h264ref; fi \
  | xargs -P "${JOBS:-4}" -n 1 "$HERE/replay_one.sh"
