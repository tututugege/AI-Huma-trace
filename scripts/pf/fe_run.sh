#!/usr/bin/env bash
# fe_run.sh <tag> <jobs> [VAR=val ...]   precise-warmup run of points_fe.lst with PF_ENABLE=1 PMU=1 and the given env -> $FE_RES/<tag>
T=$(cd "$(dirname "$0")/../.." && pwd); TAG=$1; JOBS=$2; shift 2
OUT=${FE_RES:-$T/../res}/$TAG
exec env SPEC=${SPEC:-/root/spec} POINTS=points_fe.lst PF_ENABLE=1 PMU=1 "$@" bash "$T/scripts/pf/run_eval.sh" "$T/build/simulator" "$OUT" all "$JOBS"
