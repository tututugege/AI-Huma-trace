#!/usr/bin/env bash
# fleet.sh: drive the cloud machines from the Mac.  Hosts: scripts/pf/cloud/hosts.txt (one IP per line).
#   fleet.sh pack                 stage sources + checkpoint list (calls pack.sh)
#   fleet.sh push                 sources to every host; zstd checkpoints to the first host, then fanout over the private network
#   fleet.sh fanout               (re)copy the checkpoints from the first host to the others
#   fleet.sh pushall              every checkpoint of $SPEC/ckpt as is (gzip, ~62 GB) to every host, resumable
#   fleet.sh setup                install deps, build, validate cycles on every host (parallel; prints PASS/FAIL per program)
#   fleet.sh run <set> <cfg>...   start configs spread round-robin over the hosts, in the background on the hosts
#                                 <set> = train | hold | all ; <cfg> = name:PF_ALGO=2,PF_THROTTLE=1,...
#                                 env passthrough: POINTS=points_icode.txt  MEAS=4000000  JOBS=24
#   fleet.sh status               finished configs and running simulator processes per host
#   fleet.sh pull                 rsync the results back into $RC (default .../tm/pfexp_cloud; never the local pfexp)
#   fleet.sh wait <n>             block until <n> configs are finished on the hosts
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
HOSTS=$(grep -v '^#' "$HERE/hosts.txt" | grep -v '^$')
KEY=${KEY:-$HOME/Downloads/_mac.pem}
OUT=${OUT:-$HOME/personal/AI-Huma-data/cloud}
SPEC=${SPEC:-$HOME/personal/AI-Huma-data/spec-rv32imab}
R=${R:-$SPEC/tm/pfexp}
RC=${RC:-$SPEC/tm/pfexp_cloud}
SSH_OPTS="-i $KEY -o StrictHostKeyChecking=accept-new -o ConnectTimeout=20 -o ServerAliveInterval=30"
ssh_() { local h=$1; shift; ssh $SSH_OPTS "root@$h" "$@"; }
RSYNC_SSH="ssh $SSH_OPTS"

cmd=${1:-}; shift || true
case $cmd in
  pack) "$HERE/pack.sh" "$OUT" ;;
  push)
    [ -d "$OUT/zst" ] || "$HERE/pack.sh" "$OUT"
    for h in $HOSTS; do
      ( rsync -az -e "$RSYNC_SSH" "$OUT/stage/AI-Huma" "root@$h:/root/" && ssh_ "$h" 'mkdir -p /root/spec/zst' \
        && echo "[$h] sources done" || echo "[$h] sources FAILED" ) &
    done; wait
    first=$(echo $HOSTS | awk '{print $1}')
    echo "[$first] uploading compressed checkpoints ($(du -sh "$OUT/zst" | cut -f1)); the others get them over the private network"
    rsync -a --partial -e "$RSYNC_SSH" "$OUT/zst/" "root@$first:/root/spec/zst/" && echo "[$first] checkpoints uploaded" || { echo "upload FAILED"; exit 1; }
    "$0" fanout ;;
  ckptpush) # ckptpush <bench> <file.gz>: (re)upload one SPEC checkpoint to every host, resumable (a truncated copy is completed)
    b=${1:?benchmark dir}; f=${2:?file name}
    for h in $HOSTS; do
      rsync -a --partial --inplace -e "$RSYNC_SSH" "$SPEC/ckpt/$b/$f" "root@$h:/root/spec/ckpt/$b/$f" \
        && ssh_ "$h" "gzip -t /root/spec/ckpt/$b/$f && echo '[$h] $f ok'" || echo "[$h] $f FAILED"
    done ;;
  linuxpush) # the 13 Linux segment checkpoints (s01..s13, ~0.2 GB) to every host: /root/rtlroot/linux/ckpt
    L=${LINUX_CKPT:-$HOME/personal/AI-Huma-data/linux/ck_s01_13}
    for h in $HOSTS; do
      # one rsync per file, all in parallel (a single connection was limited to ~40 KB/s), resumable (--partial)
      ssh_ "$h" 'mkdir -p /root/rtlroot/linux/ckpt' && ls "$L"/*.gz | xargs -P 4 -I{} rsync -aL --partial -e "$RSYNC_SSH" {} "root@$h:/root/rtlroot/linux/ckpt/" \
        && echo "[$h] linux checkpoints done" || echo "[$h] linux checkpoints FAILED"
    done ;;
  rtlpush) # tracked files of ../rtl and ../function-model (git archive HEAD) to every host: /root/AI-Huma/rtl_new, function-model_new
    A=$(cd "$HERE/../../../.." && pwd); T=$(mktemp -d)
    git -C "$A/rtl" archive --format=tar.gz HEAD -o "$T/rtl.tgz"; git -C "$A/function-model" archive --format=tar.gz HEAD -o "$T/fm.tgz"
    for h in $HOSTS; do
      rsync -a -e "$RSYNC_SSH" "$T/rtl.tgz" "$T/fm.tgz" "root@$h:/root/" \
        && ssh_ "$h" 'cd /root/AI-Huma && rm -rf rtl_new function-model_new && mkdir rtl_new function-model_new && tar xzf /root/rtl.tgz -C rtl_new && tar xzf /root/fm.tgz -C function-model_new && ls rtl_new | wc -l && ls function-model_new | wc -l' \
        && ssh_ "$h" 'mkdir -p /root/rtlroot/rtl/build/lock_core_top' \
        && rsync -a -e "$RSYNC_SSH" "$A/rtl/build/lock_core_top/core_top_fields.h" "root@$h:/root/rtlroot/rtl/build/lock_core_top/" \
        && echo "[$h] rtl sources done" || echo "[$h] rtl sources FAILED"
    done ;;
  pushall)
    # every checkpoint of $SPEC/ckpt as it is (gzip, ~62 GB for the 580 SPEC points) to every host, one benchmark at a time in the
    # order of scripts/pf/bench_order.txt (longest simulation first, so the slow programs can start while the rest uploads);
    # files the host already has are kept, an interrupted transfer resumes
    for h in $HOSTS; do
      ( ssh_ "$h" 'mkdir -p /root/spec/ckpt' && for b in $(cat "$HERE/../bench_order.txt"); do
          rsync -a --partial --ignore-existing -e "$RSYNC_SSH" "$SPEC/ckpt/$b/" "root@$h:/root/spec/ckpt/$b/" || exit 1
          echo "[$h] $b uploaded $(date +%H:%M:%S)"; done \
        && echo "[$h] all checkpoints uploaded" || echo "[$h] upload FAILED (rerun: it resumes)" ) &
    done; wait ;;
  fanout)
    # first host -> the others over the VPC; the key never leaves this Mac: a throw-away ssh-agent is forwarded for this command only
    first=$(echo $HOSTS | awk '{print $1}')
    eval "$(ssh-agent -s)" > /dev/null; ssh-add -q "$KEY"
    for h in $HOSTS; do
      [ "$h" = "$first" ] && continue
      priv=$(ssh_ "$h" "hostname -I | cut -d' ' -f1")
      ( ssh -A $SSH_OPTS "root@$first" "rsync -a --partial -e 'ssh -o StrictHostKeyChecking=accept-new' /root/spec/zst/ root@$priv:/root/spec/zst/" \
        && echo "[$h] checkpoints copied from $first" || echo "[$h] fanout FAILED" ) &
    done; wait
    kill "$SSH_AGENT_PID" 2>/dev/null ;;
  setup)
    for h in $HOSTS; do
      ( ssh_ "$h" 'bash /root/AI-Huma/trace-model/scripts/pf/cloud/setup_cloud.sh' > "$OUT/setup_$h.log" 2>&1 \
        && echo "[$h] setup OK" || echo "[$h] setup FAILED (see $OUT/setup_$h.log)" ) &
    done; wait
    for h in $HOSTS; do echo "== $h"; grep -E "PASS|FAIL|\[setup\]" "$OUT/setup_$h.log" | tail -8; done ;;
  run)
    set_=${1:?set (train|hold|all|guard|icode)}; shift
    case $set_ in guard) POINTS=points_guard.txt;; icode) POINTS=points_icode.txt;; esac
    n=0; for h in $HOSTS; do n=$((n+1)); done
    i=0
    for h in $HOSTS; do
      mine=()
      j=0
      for c in "$@"; do if [ $((j % n)) -eq $i ]; then mine+=("$c"); fi; j=$((j+1)); done
      if [ ${#mine[@]} -gt 0 ]; then
        # one process per point, at most 24: more than one wave only wastes the hyper-threads
        pf_=${POINTS:-points.txt}
        if [ "$set_" = all ]; then np=$(wc -l < "$HERE/../$pf_"); else np=$(awk -v s="$set_" '$1==s' "$HERE/../$pf_" | wc -l); fi
        J=${JOBS:-$(( np > 24 ? 24 : np ))}
        fwd="POINTS=$pf_ JOBS=$J"; [ -n "${MEAS:-}" ] && fwd="$fwd MEAS=$MEAS WARM=${WARM:-10000000}"
        # shellcheck disable=SC2029
        ssh_ "$h" "cd /root; nohup env $fwd bash /root/AI-Huma/trace-model/scripts/pf/cloud/remote_sweep.sh $set_ ${mine[*]} > /root/sweep_\$(date +%s).log 2>&1 < /dev/null &" 
        echo "[$h] started: ${mine[*]}"
      fi
      i=$((i+1))
    done ;;
  status)
    for h in $HOSTS; do
      echo "== $h"
      ssh_ "$h" 'echo "spec: $(du -sm /root/spec 2>/dev/null | cut -f1) MB, private ip: $(hostname -I | cut -d" " -f1)"; cat /root/res/DONE.txt 2>/dev/null | tr "\n" ";"; echo; echo "simulators running: $(pgrep -fc "build/simulator --mode")"; for d in /root/res/*/; do [ -d "$d" ] && echo "  $(basename $d): $(find $d -name "*.res" | wc -l) done, $(cat $d/fail.txt 2>/dev/null | wc -l) failed"; done; echo "mem used: $(free -g | sed -n 2p | awk "{print \$3}") GB"' ;
    done ;;
  pull)
    mkdir -p "$RC"
    for h in $HOSTS; do
      rsync -a -e "$RSYNC_SSH" --exclude '*.ds3' --exclude 'DONE.txt' "root@$h:/root/res/" "$RC/" && echo "[$h] pulled into $RC" || echo "[$h] pull FAILED"
    done ;;
  wait)  # wait until <n> configs are finished in total (DONE.txt lines over all hosts), polling every 60 s
    n=${1:?number of configs}
    while :; do
      tot=0
      for h in $HOSTS; do c=$(ssh_ "$h" 'cat /root/res/DONE.txt 2>/dev/null | wc -l' 2>/dev/null || echo 0); tot=$((tot + ${c:-0})); done
      echo "finished configs: $tot / $n"
      [ "$tot" -ge "$n" ] && break
      sleep 60
    done ;;
  build)  # build <dir> [EXTRA_CXXFLAGS]: build a simulator variant into trace-model/<dir> on every host (current sources, x86-64-v3)
    bd=${1:?build dir name, e.g. build_m16}; ex=${2:-}
    "$0" all "cd /root/AI-Huma && sed -i 's/-march=native/-march=x86-64-v3/g; s/-mtune=native/-mtune=generic/g' trace-model/Makefile function-model/Makefile && cd trace-model && make -j16 BUILD_DIR=$bd EXTRA_CXXFLAGS='$ex' > $bd.log 2>&1; tail -2 $bd.log; ls -la $bd/simulator | awk '{print \$5, \$9}'" ;;
  verify) # every checkpoint must be a complete gzip (see verify_ckpt.sh); broken ones are regenerated from the zstd copy
    "$0" all "bash /root/AI-Huma/trace-model/scripts/pf/cloud/verify_ckpt.sh" ;;
  pgo)  # pgo <dir> [EXTRA_CXXFLAGS]: profile-guided build (pgo_build.sh) of the current sources on every host
    bd=${1:?build dir name, e.g. build_pgo}; ex=${2:-}
    "$0" all "bash /root/AI-Huma/trace-model/scripts/pf/cloud/pgo_build.sh $bd '$ex'" ;;
  unpack) # zstd -> gzip for checkpoints that are not unpacked yet (incremental)
    "$0" all "cd /root/spec && find zst -name '*.zst' | xargs -P 16 -I{} bash -c 'o=\${1#zst/}; o=\${o%.zst}.gz; [ -s \$o ] && exit 0; mkdir -p \$(dirname \$o); zstd -d --long=27 -q -c \$1 | gzip -1 > \$o.tmp && mv -f \$o.tmp \$o' _ {}; echo \"checkpoints: \$(find ckpt -name '*.gz' | wc -l)\"" ;;
  all)  # run one shell command on every host (read-only checks, tuning)
    for h in $HOSTS; do ( echo "== $h"; ssh_ "$h" "$1" 2>&1 ) & done; wait ;;
  *) sed -n 2,13p "$0"; exit 1 ;;
esac
