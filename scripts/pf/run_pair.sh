#!/usr/bin/env bash
# run_pair.sh <sim> <outdir> <ckpt-name> <bench>
exec "$(dirname "$0")/run_point.sh" "$1" "$2" "$3" "$4"
