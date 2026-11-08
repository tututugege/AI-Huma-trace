#!/usr/bin/env bash
# dse_sync.sh: (Mac) copy this worktree's sources to the server's private tree /root/dse/AI-Huma/trace-model (never touches /root/AI-Huma used by other agents),
# link function-model / DRAMsim3 / softfloat from the shared tree. Run from the repo root:  bash scripts/dse/dse_sync.sh
set -euo pipefail
T=$(cd "$(dirname "$0")/../.." && pwd); F=$T/scripts/pf/cloud/fleet.sh
KEY=${KEY:-$HOME/Downloads/_mac.pem}; H=$(grep -v '^#' "$T/scripts/pf/cloud/hosts.txt" | head -1)
SSHO="-i $KEY -o StrictHostKeyChecking=accept-new -o ConnectTimeout=20"
bash "$F" all "mkdir -p /root/dse/AI-Huma/trace-model /root/dse/res" >/dev/null
rsync -a --delete -e "ssh $SSHO" --exclude .git --exclude 'build*' --exclude results --exclude libs --exclude third_party --exclude function-model --exclude '*.o' --exclude hosts.txt "$T/" "root@$H:/root/dse/AI-Huma/trace-model/"
bash "$F" all "cd /root/dse/AI-Huma && ln -sfn /root/AI-Huma/function-model function-model && cd trace-model && ln -sfn ../function-model function-model && ln -sfn /root/AI-Huma/trace-model/libs libs && ln -sfn /root/AI-Huma/trace-model/third_party third_party && sed -i 's/-march=native/-march=x86-64-v3/g; s/-mtune=native/-mtune=generic/g' Makefile && ls | head -30"
