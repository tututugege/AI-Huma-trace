#!/usr/bin/env bash
# Usage: lsu_shadow.sh <simulator-repo> [gate-outdir]
# Builds the simulator with the frozen original LSU running in lockstep (LSU_SHADOW) and runs the gate on it:
# every LSU phase's outputs and a canonical state digest are compared; a mismatch exits with code 2.
SIM=$1; OUT=${2:-results/lsu_shadow}; T=$(cd "$(dirname "$0")/.." && pwd)
ln -sfn "$T/tests/lsu_ref" "$SIM/lsu_ref"; grep -q '^lsu_ref$' "$SIM/.git/info/exclude" 2>/dev/null || echo lsu_ref >> "$SIM/.git/info/exclude"
(cd "$SIM" && gmake BUILD_DIR=build_shadow LSU_SHADOW=1 LDFLAGS="-lz" -j8 2>&1 | grep -E "error|Linking")
DEADLINE=120 bash "$T/scripts/run_gate.sh" "$SIM/build_shadow/simulator" "$OUT"
LSU_SHADOW_DIRECTED=1 "$SIM/build_shadow/simulator" "$T/build/tests/add/add.bin" 2>&1 | grep -a LSU_DIRECTED   # directed state-injection scenarios
