#!/usr/bin/env bash

set -uo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)

SIMULATOR=${SIMULATOR:-$REPO_ROOT/build/simulator}
SHORT_BIN_DIR=${SHORT_BIN_DIR:-$REPO_ROOT/baremetal/test/build}
COREMARK_BIN=${COREMARK_BIN:-$REPO_ROOT/baremetal/coremark/build/coremark.bin}
DHRYSTONE_BIN=${DHRYSTONE_BIN:-$REPO_ROOT/baremetal/dhrystone/build/dhrystone.bin}
SPIFLASH_IMAGE=${SPIFLASH_IMAGE:-$REPO_ROOT/regression-images/spiflash.bin}
LIBQUANTUM_SDCARD=${LIBQUANTUM_SDCARD:-$REPO_ROOT/regression-images/462.libquantum-sdcard.img}
RESULT_ROOT=${RESULT_ROOT:-$REPO_ROOT/regression-results}

PIN_CPU=${PIN_CPU:-0}
SHORT_TIMEOUT_SECONDS=${SHORT_TIMEOUT_SECONDS:-180}
BENCH_TIMEOUT_SECONDS=${BENCH_TIMEOUT_SECONDS:-1800}
LINUX_TIMEOUT_SECONDS=${LINUX_TIMEOUT_SECONDS:-0}
LINUX_POLL_SECONDS=${LINUX_POLL_SECONDS:-30}
LINUX_MAX_COMMIT=${LINUX_MAX_COMMIT:-200000000000}
NO_DIFF=${NO_DIFF:-0}
RUN_LINUX=${RUN_LINUX:-1}

usage() {
  cat <<'EOF'
Usage: script/run_serial_regression.sh

Run the following regression serially on one logical CPU:
  1. all prebuilt short cpu-test binaries (memory-stress is not included)
  2. CoreMark
  3. Dhrystone
  4. SD-card Linux boot and SPEC2006 test 462.libquantum

Environment overrides:
  SIMULATOR              default: ./build/simulator
  SHORT_BIN_DIR          default: ./baremetal/test/build
  COREMARK_BIN           default: ./baremetal/coremark/build/coremark.bin
  DHRYSTONE_BIN          default: ./baremetal/dhrystone/build/dhrystone.bin
  SPIFLASH_IMAGE         default: ./regression-images/spiflash.bin
  LIBQUANTUM_SDCARD      default: ./regression-images/462.libquantum-sdcard.img
  RESULT_ROOT            default: ./regression-results
  PIN_CPU                default: 0
  SHORT_TIMEOUT_SECONDS  default: 180 per short test
  BENCH_TIMEOUT_SECONDS  default: 1800 per CoreMark/Dhrystone test
  LINUX_TIMEOUT_SECONDS  default: 0 (no wall-time limit)
  LINUX_POLL_SECONDS     default: 30
  LINUX_MAX_COMMIT       default: 200000000000
  NO_DIFF                0 keeps difftest enabled; 1 disables it
  RUN_LINUX              1 runs Linux + 462.libquantum; 0 skips it
EOF
}

die() {
  echo "error: $*" >&2
  exit 2
}

is_non_negative_integer() {
  [[ $1 =~ ^[0-9]+$ ]]
}

is_positive_integer() {
  [[ $1 =~ ^[1-9][0-9]*$ ]]
}

if [[ ${1-} == -h || ${1-} == --help ]]; then
  usage
  exit 0
fi
(( $# == 0 )) || die "unknown argument: $1"

[[ $PIN_CPU =~ ^[0-9]+$ ]] || die "PIN_CPU must be a non-negative integer"
is_positive_integer "$SHORT_TIMEOUT_SECONDS" || die "SHORT_TIMEOUT_SECONDS must be positive"
is_positive_integer "$BENCH_TIMEOUT_SECONDS" || die "BENCH_TIMEOUT_SECONDS must be positive"
is_non_negative_integer "$LINUX_TIMEOUT_SECONDS" || die "LINUX_TIMEOUT_SECONDS must be non-negative"
is_positive_integer "$LINUX_POLL_SECONDS" || die "LINUX_POLL_SECONDS must be positive"
is_positive_integer "$LINUX_MAX_COMMIT" || die "LINUX_MAX_COMMIT must be positive"
[[ $NO_DIFF =~ ^[01]$ ]] || die "NO_DIFF must be 0 or 1"
[[ $RUN_LINUX =~ ^[01]$ ]] || die "RUN_LINUX must be 0 or 1"

for command_name in taskset timeout flock sha256sum stdbuf; do
  command -v "$command_name" >/dev/null 2>&1 || die "missing command: $command_name"
done

[[ -x $SIMULATOR ]] || die "simulator not executable: $SIMULATOR"
[[ -d $SHORT_BIN_DIR ]] || die "short-test directory missing: $SHORT_BIN_DIR"
[[ -f $COREMARK_BIN ]] || die "CoreMark binary missing: $COREMARK_BIN"
[[ -f $DHRYSTONE_BIN ]] || die "Dhrystone binary missing: $DHRYSTONE_BIN"
[[ -f $SPIFLASH_IMAGE ]] || die "spiflash image missing: $SPIFLASH_IMAGE"
[[ -f $LIBQUANTUM_SDCARD ]] || die "462.libquantum SD-card image missing: $LIBQUANTUM_SDCARD"
taskset -c "$PIN_CPU" true >/dev/null 2>&1 || die "CPU $PIN_CPU is unavailable"

mapfile -t SHORT_BINS < <(find "$SHORT_BIN_DIR" -maxdepth 1 -type f -name '*.bin' | sort)
(( ${#SHORT_BINS[@]} > 0 )) || die "no short-test binaries found in $SHORT_BIN_DIR"

mkdir -p "$RESULT_ROOT"
RESULT_ROOT=$(cd -- "$RESULT_ROOT" && pwd)
RUN_DIR=$RESULT_ROOT/serial-$(date +%Y%m%d-%H%M%S)
mkdir -p "$RUN_DIR/short"

exec 9>"$RESULT_ROOT/.serial-regression.lock"
flock -n 9 || die "another serial regression is already running"

SUMMARY=$RUN_DIR/summary.tsv
printf 'test\tstatus\trc\telapsed_seconds\tlog\n' >"$SUMMARY"

DIFF_ARGS=()
if (( NO_DIFF == 1 )); then
  DIFF_ARGS+=(--no-diff)
fi

CURRENT_PID=""
cleanup() {
  if [[ -n $CURRENT_PID ]] && kill -0 "$CURRENT_PID" 2>/dev/null; then
    kill -INT "$CURRENT_PID" 2>/dev/null || true
    wait "$CURRENT_PID" 2>/dev/null || true
  fi
}
trap 'cleanup; exit 130' INT TERM

write_config() {
  {
    echo "started=$(date --iso-8601=seconds)"
    echo "host=$(hostname)"
    echo "simulator=$SIMULATOR"
    echo "simulator_sha256=$(sha256sum "$SIMULATOR" | awk '{print $1}')"
    echo "short_test_count=${#SHORT_BINS[@]}"
    echo "coremark_sha256=$(sha256sum "$COREMARK_BIN" | awk '{print $1}')"
    echo "dhrystone_sha256=$(sha256sum "$DHRYSTONE_BIN" | awk '{print $1}')"
    echo "spiflash_sha256=$(sha256sum "$SPIFLASH_IMAGE" | awk '{print $1}')"
    echo "libquantum_sdcard_sha256=$(sha256sum "$LIBQUANTUM_SDCARD" | awk '{print $1}')"
    echo "pin_cpu=$PIN_CPU"
    echo "difftest=$([[ $NO_DIFF == 1 ]] && echo disabled || echo enabled)"
    echo "short_timeout_seconds=$SHORT_TIMEOUT_SECONDS"
    echo "bench_timeout_seconds=$BENCH_TIMEOUT_SECONDS"
    echo "linux_timeout_seconds=$LINUX_TIMEOUT_SECONDS"
    echo "linux_max_commit=$LINUX_MAX_COMMIT"
    echo "run_linux=$RUN_LINUX"
  } >"$RUN_DIR/run_config.txt"
}

record_result() {
  local name=$1
  local status=$2
  local rc=$3
  local elapsed=$4
  local log=$5
  printf '%s\t%s\t%s\t%s\t%s\n' "$name" "$status" "$rc" "$elapsed" "$log" >>"$SUMMARY"
  printf '%-32s %s (%ss)\n' "$name" "$status" "$elapsed"
}

run_binary() {
  local name=$1
  local image=$2
  local timeout_seconds=$3
  local log=$4
  local required_marker=${5-}
  local started finished elapsed rc status

  started=$(date +%s)
  taskset -c "$PIN_CPU" timeout --foreground --signal=INT --kill-after=10s \
    "$timeout_seconds" "$SIMULATOR" "${DIFF_ARGS[@]}" "$image" >"$log" 2>&1
  rc=$?
  finished=$(date +%s)
  elapsed=$((finished - started))
  status=FAIL
  if (( rc == 0 )); then
    if [[ -z $required_marker ]] || grep -aFq "$required_marker" "$log"; then
      status=PASS
    fi
  fi
  record_result "$name" "$status" "$rc" "$elapsed" "$log"
  [[ $status == PASS ]]
}

run_linux_libquantum() {
  local name=linux-462.libquantum
  local log=$RUN_DIR/linux-462.libquantum.log
  local marker='@@SPEC_BENCH_DONE 462.libquantum 0@@'
  local started now elapsed rc status

  started=$(date +%s)
  taskset -c "$PIN_CPU" stdbuf -oL -eL "$SIMULATOR" "${DIFF_ARGS[@]}" \
    --flash-image "$SPIFLASH_IMAGE" \
    --sdcard-image "$LIBQUANTUM_SDCARD" \
    --max-commit "$LINUX_MAX_COMMIT" >"$log" 2>&1 &
  CURRENT_PID=$!
  status=FAIL
  rc=1

  while kill -0 "$CURRENT_PID" 2>/dev/null; do
    if grep -aFq "$marker" "$log" 2>/dev/null; then
      status=PASS
      rc=0
      kill -INT "$CURRENT_PID" 2>/dev/null || true
      wait "$CURRENT_PID" 2>/dev/null || true
      CURRENT_PID=""
      break
    fi
    if (( LINUX_TIMEOUT_SECONDS > 0 )); then
      now=$(date +%s)
      if (( now - started >= LINUX_TIMEOUT_SECONDS )); then
        rc=124
        kill -INT "$CURRENT_PID" 2>/dev/null || true
        wait "$CURRENT_PID" 2>/dev/null || true
        CURRENT_PID=""
        break
      fi
    fi
    sleep "$LINUX_POLL_SECONDS"
  done

  if [[ -n $CURRENT_PID ]]; then
    wait "$CURRENT_PID"
    rc=$?
    CURRENT_PID=""
    if grep -aFq "$marker" "$log" 2>/dev/null; then
      status=PASS
      rc=0
    fi
  fi

  now=$(date +%s)
  elapsed=$((now - started))
  record_result "$name" "$status" "$rc" "$elapsed" "$log"
  [[ $status == PASS ]]
}

write_config
echo "Results: $RUN_DIR"
echo "Simulator: $SIMULATOR"
echo "Difftest: $([[ $NO_DIFF == 1 ]] && echo disabled || echo enabled)"
echo "Pinned CPU: $PIN_CPU"
echo "Short tests: ${#SHORT_BINS[@]}"
echo "Linux + 462.libquantum: $([[ $RUN_LINUX == 1 ]] && echo enabled || echo skipped)"

failures=0
for binary in "${SHORT_BINS[@]}"; do
  test_name=$(basename "$binary" .bin)
  run_binary "short/$test_name" "$binary" "$SHORT_TIMEOUT_SECONDS" \
    "$RUN_DIR/short/$test_name.log" || ((failures += 1))
done

run_binary coremark "$COREMARK_BIN" "$BENCH_TIMEOUT_SECONDS" \
  "$RUN_DIR/coremark.log" 'Correct operation validated.' || ((failures += 1))
run_binary dhrystone "$DHRYSTONE_BIN" "$BENCH_TIMEOUT_SECONDS" \
  "$RUN_DIR/dhrystone.log" 'Dhrystones per Second:' || ((failures += 1))
if (( RUN_LINUX == 1 )); then
  run_linux_libquantum || ((failures += 1))
fi

total=$(( ${#SHORT_BINS[@]} + 2 + RUN_LINUX ))
passed=$((total - failures))
{
  echo "finished=$(date --iso-8601=seconds)"
  echo "total=$total"
  echo "pass=$passed"
  echo "fail=$failures"
} >"$RUN_DIR/final_status.txt"

echo "SUMMARY pass=$passed fail=$failures total=$total"
echo "RESULTS $RUN_DIR"
(( failures == 0 ))
