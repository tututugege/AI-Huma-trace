#!/usr/bin/env bash
# remote_sweep.sh <set> name:ENV=VAL,ENV=VAL ...   (runs ON a cloud machine; started by fleet.sh)
# Results go to /root/res/<name>/<bench>/<ckpt>.res; finished configs are listed in /root/res/DONE.txt.
T=/root/AI-Huma/trace-model
export SPEC=/root/spec
SET=$1; shift
for cfg in "$@"; do
  head=${cfg%%:*}; envs=${cfg#*:}
  [ "$envs" = "$cfg" ] && envs=""
  name=${head%%@*}; bdir=build; [ "$name" != "$head" ] && bdir=${head#*@}   # name@builddir:ENV picks another binary
  env $(echo $envs | tr ',' ' ') "$T/scripts/pf/run_eval.sh" "$T/$bdir/simulator" "/root/res/$name" "$SET" "${JOBS:-24}" > "/root/res/$name.log" 2>&1
  echo "done $name" >> /root/res/DONE.txt
done
