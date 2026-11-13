#!/usr/bin/env bash
# Boot a Linux image on the trace-model and check the result: the login prompt must appear, with no kernel oops / panic / illegal instruction.
# (EXP-034..036 were merged with the gate alone and broke Linux through Bridge::find; run this before every merge to main.)
#   usage: scripts/linux_check.sh <simulator> <image.bin> [cycles=200000000] [log=/tmp/linux_check.log]
#   rv32: image from function-model/linux/mkimage.sh (login prompt "rv32 login:"), about 1.3e8 cycles; rv64: "rv64 login:" (about 1.5e8 cycles)
#   The image is built as described in function-model/linux/README.md; the simulator must be built with the matching XLEN / RVC.
set -uo pipefail
SIM=$1; IMG=$2; CYC=${3:-200000000}; LOG=${4:-/tmp/linux_check.log}
SIM_MAX_CYCLES=$CYC "$SIM" "$IMG" > "$LOG" 2>&1
rc=$?
login=$(grep -a -c -E 'rv(32|64) login:' "$LOG")
bad=$(grep -a -c -E 'NULL pointer|Kernel panic|Oops|illegal instruction|Unable to handle' "$LOG")
mm=$(grep -a -o 'mismatches rd=[0-9]* br=[0-9]* addr=[0-9]* target=[0-9]*' "$LOG" | tail -1)
echo "linux_check: rc=$rc login=$login oops/panic=$bad $mm (log $LOG)"
[ "$login" -ge 1 ] && [ "$bad" -eq 0 ]
