#!/usr/bin/env bash
# setup_cloud.sh: run ON a cloud machine (Ubuntu 22.04 x86_64) as root, after `fleet.sh push`.
# Installs the build dependencies, rebuilds the reference CPU library and the simulator for x86-64-v3, and validates the cycle
# counts against the Mac results (they must be identical).
set -euo pipefail
cd /root/AI-Huma
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y -qq build-essential pkg-config zlib1g-dev rsync python3 zstd >/dev/null
echo "[setup] unpacking checkpoints (zstd -> gzip -1)"
( cd /root/spec && find zst -name '*.zst' | xargs -P 24 -I{} bash -c 'o=${1#zst/}; o=${o%.zst}.gz; mkdir -p $(dirname $o); zstd -d --long=27 -q -c $1 | gzip -1 > $o.tmp && mv -f $o.tmp $o' _ {} )
echo "[setup] checkpoints: $(find /root/spec/ckpt -name '*.gz' | wc -l)"
# portable code generation: the machines may be different CPU generations
sed -i 's/-march=native/-march=x86-64-v3/g; s/-mtune=native/-mtune=generic/g' trace-model/Makefile function-model/Makefile
echo "[setup] building function-model (librefcpu.a)"
make -C function-model librefcpu.a -j16 > function-model/build.log 2>&1 || { tail -30 function-model/build.log; exit 1; }
echo "[setup] building trace-model (with DRAMsim3)"
( cd trace-model && make -j32 > build.log 2>&1 ) || { tail -40 trace-model/build.log; exit 1; }
test -x trace-model/build/simulator
echo "[setup] validating cycle counts (PF_ENABLE=0 = no prefetch = the 2026-11-05 baseline; PF_ENABLE=1 = default; must equal the Mac numbers)"
cd trace-model
fail=0
check() {  # name expected [PF_ENABLE]
  got=$(PF_ENABLE=${3:-0} SIM_MAX_CYCLES=5000000 build/simulator validate/$1.bin 2>&1 | sed 's/\x1b\[[0-9;]*m//g' | grep -a "cycle       num" | awk '{print $3}')
  if [ "$got" = "$2" ]; then echo "  PASS $1 $got"; else echo "  FAIL $1 got=$got expected=$2"; fail=1; fi
}
check dhrystone 162033
check coremark 255335
check memory-stress-small 347305
check matmul_fpu 43523
check prime 12233
check dhrystone 161183 1
check coremark 256573 1
check memory-stress-small 334635 1
mkdir -p /root/res /root/spec
[ $fail -eq 0 ] && echo "[setup] OK" || { echo "[setup] VALIDATION FAILED: do not use the results of this machine"; exit 2; }
