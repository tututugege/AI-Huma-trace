#!/usr/bin/env bash
# pgo_build.sh <build-dir> [EXTRA_CXXFLAGS]   -- run ON a cloud machine (gcc, Linux).
# Profile-guided build of the simulator into trace-model/<build-dir>:
#   1. instrumented build (-fprofile-generate), 2. train on four SPEC points (5M cycles each, in parallel),
#   3. rebuild in the SAME directory with -fprofile-use (gcc keys the .gcda files by object path),
#   4. check that the cycle counts equal the plain build and time both side by side.
# The result is cycle-for-cycle identical to the plain build (PGO only changes code generation).
set -euo pipefail
BD=${1:?build dir}; EX=${2:-}
cd /root/AI-Huma/trace-model
sed -i 's/-march=native/-march=x86-64-v3/g; s/-mtune=native/-mtune=generic/g' Makefile ../function-model/Makefile
SPEC=/root/spec
rm -rf "$BD"
echo "[pgo] 1/4 instrumented build"
make -j16 BUILD_DIR="$BD" EXTRA_CXXFLAGS="$EX -fprofile-generate" > "$BD.gen.log" 2>&1 || { tail -20 "$BD.gen.log"; exit 1; }
echo "[pgo] 2/4 training"
train() { b=$1; n=$2; SIM_MAX_CYCLES=5000000 PF_ALGO=0 "$BD/simulator" --mode ckpt --restore-stub $SPEC/tm/pf_stubs/$b/$n.stub -c 10000000 $SPEC/ckpt/$b/$n.gz > /dev/null 2>&1 || true; }
train 429.mcf ckpt_sp60_target26087_warmup1 &
train 462.libquantum ckpt_sp1_target176981_warmup1 &
train 401.bzip2 ckpt_sp10_target134727_warmup1 &
train 483.xalancbmk ckpt_sp21_target28641_warmup1 &
wait
n=$(find "$BD" -name '*.gcda' | wc -l); echo "[pgo]    profile files: $n"
[ "$n" -gt 10 ] || { echo "no profile data written"; exit 1; }
echo "[pgo] 3/4 optimized build"
find "$BD" -name '*.o' -delete; rm -f "$BD/simulator"
make -j16 BUILD_DIR="$BD" EXTRA_CXXFLAGS="$EX -fprofile-use -fprofile-correction -Wno-missing-profile -Wno-coverage-mismatch" > "$BD.use.log" 2>&1 || { tail -20 "$BD.use.log"; exit 1; }
test -x "$BD/simulator"
echo "[pgo] 4/4 check: cycles must equal the plain build; then time both side by side"
fail=0
for t in dhrystone:162033 coremark:255335 memory-stress-small:347305; do
  p=${t%%:*}; e=${t#*:}
  got=$(PF_ALGO=0 SIM_MAX_CYCLES=5000000 "$BD/simulator" validate/$p.bin 2>&1 | sed 's/\x1b\[[0-9;]*m//g' | grep -a "cycle       num" | awk '{print $3}')
  if [ "$got" = "$e" ]; then echo "  PASS $p $got"; else echo "  FAIL $p got=$got expected=$e"; fail=1; fi
done
[ $fail -eq 0 ] || { echo "[pgo] cycle mismatch: do not use this binary"; exit 2; }
timeit() { /usr/bin/time -f "%U" -o "$2" env SIM_MAX_CYCLES=6000000 PF_ALGO=0 "$1/simulator" --mode ckpt --restore-stub $SPEC/tm/pf_stubs/429.mcf/ckpt_sp60_target26087_warmup1.stub -c 10000000 $SPEC/ckpt/429.mcf/ckpt_sp60_target26087_warmup1.gz > /dev/null 2>&1; }
timeit build /tmp/t_plain & timeit "$BD" /tmp/t_pgo & wait
echo "[pgo] mcf 6M cycles user time: plain $(cat /tmp/t_plain) s, PGO $(cat /tmp/t_pgo) s"
