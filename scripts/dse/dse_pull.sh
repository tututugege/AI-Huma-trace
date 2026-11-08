#!/usr/bin/env bash
# dse_pull.sh (Mac): rsync the DSE results back (events/hist/res only) into ~/personal/AI-Huma-data/dse/res
T=$(cd "$(dirname "$0")/../.." && pwd); KEY=${KEY:-$HOME/Downloads/_mac.pem}; H=$(grep -v '^#' "$T/scripts/pf/cloud/hosts.txt" | head -1)
D=${D:-$HOME/personal/AI-Huma-data/dse}; mkdir -p "$D/res"
rsync -a -e "ssh -i $KEY -o StrictHostKeyChecking=accept-new -o ConnectTimeout=20" --exclude '*.log' "root@$H:/root/dse/res/" "$D/res/" && echo "pulled into $D/res: $(find $D/res -name '*.res' | wc -l) points"
