#!/usr/bin/env bash
# run_point.sh <sim> <outdir> <ckpt-name> <bench>   (env passes through, e.g. PF_DEGREE=4)
SIM=$1; OUT=$2; N=$3; B=$4
SPEC=${SPEC:-$HOME/personal/AI-Huma-data/spec-rv32imab}
C=$SPEC/ckpt/$B/$N.gz
STUBS=$SPEC/tm/pf_stubs/$B; mkdir -p "$STUBS" "$OUT/$B"
[ -f "$STUBS/$N.stub" ] || python3 -I "$(dirname "$0")/../../../function-model/tools/ckpt2stub.py" "$C" "$STUBS/$N.stub" > "$STUBS/$N.stub.log" 2>&1 || { echo "$B $N STUBFAIL" >> "$OUT/fail.txt"; exit 1; }
# quick-screen tier: WARM/MEAS (instructions) shorten warmup and measurement, the front-door restore needs WARM = checkpoint interval (10M); only MEAS can be shortened, e.g. MEAS=4000000
if [ -n "${MEAS:-}" ]; then ARGS="-w ${WARM:-10000000} -c $MEAS"
else case $N in *nowarmup*) ARGS="-w 0 -c 10000000";; *) ARGS="-c 10000000";; esac; fi
L=$OUT/$B/$N.log
PF_STATS=1 DS3_STATS=1 DS3_OUT=$OUT/$B/$N.ds3 "$SIM" --mode ckpt --restore-stub "$STUBS/$N.stub" $ARGS "$C" > "$L" 2>&1
rc=$?
clean() { sed 's/\x1b\[[0-9;]*m//g' "$L"; }
cyc=$(clean | grep -a "cycle       num:" | tail -1 | awk '{print $3}')
ins=$(clean | grep -a "instruction num:" | tail -1 | awk '{print $3}')
if [ $rc -eq 0 ] && [ -n "$cyc" ] && clean | grep -aq Success; then
  { echo "$N $ins $cyc"; clean | grep -a "^\[PF-STATS\]\|^\[DS3\]"; } > "$OUT/$B/$N.res"
  rm -rf "$OUT/$B/$N.ds3"
else echo "$B $N rc=$rc cyc=$cyc" >> "$OUT/fail.txt"; fi
