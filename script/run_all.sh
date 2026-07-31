#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

DEFAULT_SIMULATOR="./build/simulator"
SIMULATOR="${SIMULATOR:-${DEFAULT_SIMULATOR}}"
CKPT_ROOT="${CKPT_ROOT:-/share/personal/S/houruyao/simpoint_20260719/ckpts}"
SDCARD_IMAGE="${SDCARD_IMAGE-/share/personal/S/houruyao/simpoint_20260719/images/sdcard-ref.img}"
RESULT_DIR="${RESULT_DIR:-/nfs_global/S/houruyao/simpoint_20260719/results/simulator_main_10M_$(date +%Y%m%d_%H%M%S)}"

CKPT_WARMUP="${CKPT_WARMUP:-10000000}"
CKPT_MAX_COMMIT="${CKPT_MAX_COMMIT:-10000000}"
EXPECTED_CKPTS="${EXPECTED_CKPTS:-629}"
MAX_JOBS="${MAX_JOBS:-auto}"
PIN_CPUS="${PIN_CPUS:-}"
TIMEOUT_SECS="${TIMEOUT_SECS:-0}"
NO_DIFF="${NO_DIFF:-0}"
RESUME="${RESUME:-1}"
DRY_RUN="${DRY_RUN:-0}"

usage() {
  cat <<'EOF'
Usage:
  script/run_all.sh

Default workload:
  - replay all 629 SPEC2006 ref checkpoints from simpoint_20260719
  - use ./build/simulator and the ref SD-card image
  - run 10M O3 warmup instructions followed by 10M measured instructions
  - use one logical CPU from every physical core

Environment overrides:
  SIMULATOR        Simulator executable. Set an absolute path on a server.
  CKPT_ROOT        Root containing benchmark/checkpoint .gz files.
  SDCARD_IMAGE     SD-card backing image; set to an empty string to omit it.
  RESULT_DIR       Per-checkpoint logs, status files and summary output.
  CKPT_WARMUP      O3 warmup commits passed with -w. Default: 10000000.
  CKPT_MAX_COMMIT  Measured commits passed with -c. Default: 10000000.
  EXPECTED_CKPTS   Required checkpoint count. Default: 629; 0 disables.
  MAX_JOBS         Worker count, or auto for all physical cores. Default: auto.
  PIN_CPUS         Comma/space separated logical CPU IDs. Overrides discovery.
  TIMEOUT_SECS     Per-checkpoint timeout. Default: 0 (disabled).
  NO_DIFF          1 passes --no-diff; default 0 keeps difftest enabled.
  RESUME           1 skips checkpoints already marked PASS in RESULT_DIR.
  DRY_RUN          1 validates inputs and prints the launch plan only.

Examples:
  script/run_all.sh
  MAX_JOBS=36 script/run_all.sh
  RESULT_DIR=/path/to/results RESUME=1 script/run_all.sh
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

if [[ "${1-}" == "-h" || "${1-}" == "--help" ]]; then
  usage
  exit 0
fi

[[ "${CKPT_WARMUP}" =~ ^[0-9]+$ ]] || die "CKPT_WARMUP must be a non-negative integer"
[[ "${CKPT_MAX_COMMIT}" =~ ^[0-9]+$ ]] || die "CKPT_MAX_COMMIT must be a positive integer"
(( CKPT_MAX_COMMIT > 0 )) || die "CKPT_MAX_COMMIT must be > 0"
[[ "${EXPECTED_CKPTS}" =~ ^[0-9]+$ ]] || die "EXPECTED_CKPTS must be a non-negative integer"
[[ "${TIMEOUT_SECS}" =~ ^[0-9]+$ ]] || die "TIMEOUT_SECS must be a non-negative integer"
[[ "${NO_DIFF}" =~ ^[01]$ ]] || die "NO_DIFF must be 0 or 1"
[[ "${RESUME}" =~ ^[01]$ ]] || die "RESUME must be 0 or 1"
[[ "${DRY_RUN}" =~ ^[01]$ ]] || die "DRY_RUN must be 0 or 1"

command -v taskset >/dev/null 2>&1 || die "taskset is required"
command -v lscpu >/dev/null 2>&1 || die "lscpu is required"
[[ -x "${SIMULATOR}" ]] || die "simulator not found or not executable: ${SIMULATOR}"
[[ -d "${CKPT_ROOT}" ]] || die "checkpoint root not found: ${CKPT_ROOT}"
if [[ -n "${SDCARD_IMAGE}" ]]; then
  [[ -f "${SDCARD_IMAGE}" ]] || die "SD-card image not found: ${SDCARD_IMAGE}"
fi

mapfile -t ALL_CKPTS < <(find "${CKPT_ROOT}" -type f -name '*.gz' | sort)
TOTAL_TASKS="${#ALL_CKPTS[@]}"
(( TOTAL_TASKS > 0 )) || die "no .gz checkpoints found in ${CKPT_ROOT}"
if (( EXPECTED_CKPTS > 0 && TOTAL_TASKS != EXPECTED_CKPTS )); then
  die "expected ${EXPECTED_CKPTS} checkpoints, found ${TOTAL_TASKS}"
fi

discover_physical_cpus() {
  lscpu -p=CPU,CORE,SOCKET,ONLINE \
    | awk -F, '
        /^#/ { next }
        $4 != "Y" { next }
        {
          key = $3 ":" $2
          if (!(key in seen)) {
            seen[key] = 1
            printf "%s%s", (n++ == 0 ? "" : " "), $1
          }
        }
        END { if (n > 0) printf "\n" }
      '
}

if [[ -n "${PIN_CPUS}" ]]; then
  read -r -a AVAILABLE_CPUS <<<"$(tr ',:' '  ' <<<"${PIN_CPUS}")"
else
  read -r -a AVAILABLE_CPUS <<<"$(discover_physical_cpus)"
fi
(( ${#AVAILABLE_CPUS[@]} > 0 )) || die "no usable CPUs found"

if [[ "${MAX_JOBS}" == "auto" ]]; then
  JOBS="${#AVAILABLE_CPUS[@]}"
else
  [[ "${MAX_JOBS}" =~ ^[0-9]+$ ]] || die "MAX_JOBS must be auto or a positive integer"
  (( MAX_JOBS > 0 )) || die "MAX_JOBS must be > 0"
  JOBS="${MAX_JOBS}"
fi
(( JOBS <= ${#AVAILABLE_CPUS[@]} )) \
  || die "requested ${JOBS} workers, but only ${#AVAILABLE_CPUS[@]} selected physical-core CPUs are available"
if (( JOBS > TOTAL_TASKS )); then
  JOBS="${TOTAL_TASKS}"
fi
SELECTED_CPUS=("${AVAILABLE_CPUS[@]:0:JOBS}")

mkdir -p "${RESULT_DIR}"
for ckpt in "${ALL_CKPTS[@]}"; do
  mkdir -p "${RESULT_DIR}/$(basename "$(dirname "${ckpt}")")"
done

SIM_ARGS=("${SIMULATOR}" --mode ckpt -w "${CKPT_WARMUP}" -c "${CKPT_MAX_COMMIT}")
if [[ -n "${SDCARD_IMAGE}" ]]; then
  SIM_ARGS+=(--sdcard-image "${SDCARD_IMAGE}")
fi
if [[ "${NO_DIFF}" == "1" ]]; then
  SIM_ARGS+=(--no-diff)
fi

echo "=================================================="
echo "Start Time:       $(date)"
echo "Simulator:        ${SIMULATOR}"
echo "Checkpoint Root:  ${CKPT_ROOT}"
echo "SD Card:          ${SDCARD_IMAGE:-<none>}"
echo "Result Dir:       ${RESULT_DIR}"
echo "Checkpoints:      ${TOTAL_TASKS}"
echo "Warmup/Measure:   ${CKPT_WARMUP}/${CKPT_MAX_COMMIT} commits"
echo "Difftest:         $([[ "${NO_DIFF}" == "1" ]] && echo disabled || echo enabled)"
echo "Workers:          ${JOBS}"
echo "Pinned CPUs:      ${SELECTED_CPUS[*]}"
echo "CPU Policy:       one logical CPU per physical core; SMT siblings unused"
echo "Resume:           ${RESUME}"
echo "=================================================="

if [[ "${DRY_RUN}" == "1" ]]; then
  for ((worker = 0; worker < JOBS; worker++)); do
    count=$(((TOTAL_TASKS - 1 - worker) / JOBS + 1))
    echo "[DryRun] worker=${worker} cpu=${SELECTED_CPUS[$worker]} tasks=${count}"
  done
  exit 0
fi

START_TIME="$(date +%s)"
WORKER_PIDS=()
WORKER_RESULT_FILES=()

terminate_workers() {
  echo
  echo "Interrupted; stopping checkpoint workers..." >&2
  for pid in "${WORKER_PIDS[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  exit 130
}
trap terminate_workers INT TERM

run_worker() {
  local worker_id="$1"
  local cpu="$2"
  local result_file="${RESULT_DIR}/worker_${worker_id}.tsv"
  local failed=0
  : >"${result_file}"

  for ((task = worker_id; task < TOTAL_TASKS; task += JOBS)); do
    local ckpt="${ALL_CKPTS[$task]}"
    local bench base log_file status_file tmp_status
    local started finished elapsed rc status reason reported_commits reached_limit
    bench="$(basename "$(dirname "${ckpt}")")"
    base="$(basename "${ckpt}" .gz)"
    log_file="${RESULT_DIR}/${bench}/${base}.log"
    status_file="${RESULT_DIR}/${bench}/${base}.status"

    if [[ "${RESUME}" == "1" && -f "${status_file}" ]] \
      && grep -q '^PASS[[:space:]]' "${status_file}"; then
      printf "%s\t%s\tPASS\tresumed\t0\t0\t%s\t%s\n" \
        "${bench}" "${base}" "${cpu}" "${log_file}" >>"${result_file}"
      echo "[Skip] worker=${worker_id} cpu=${cpu} ${bench}/${base} already PASS"
      continue
    fi

    echo "[Start] worker=${worker_id} cpu=${cpu} ${bench}/${base}"
    started="$(date +%s)"
    if (( TIMEOUT_SECS > 0 )); then
      timeout --preserve-status "${TIMEOUT_SECS}" \
        taskset -c "${cpu}" "${SIM_ARGS[@]}" "${ckpt}" >"${log_file}" 2>&1
      rc=$?
    else
      taskset -c "${cpu}" "${SIM_ARGS[@]}" "${ckpt}" >"${log_file}" 2>&1
      rc=$?
    fi
    finished="$(date +%s)"
    elapsed=$((finished - started))

    status="FAIL"
    reason="sim_rc_${rc}"
    reported_commits="$(
      sed -nE \
        's/.*committed\(total\/load\/store\)=[[:space:]]*([0-9]+)[[:space:]]*\/.*/\1/p' \
        "${log_file}" | tail -n 1
    )"
    reached_limit=0
    if grep -Fq "Reached MAX_COMMIT_INST=${CKPT_MAX_COMMIT}" "${log_file}"; then
      reached_limit=1
    elif [[ "${reported_commits}" =~ ^[0-9]+$ ]] \
      && (( reported_commits >= CKPT_MAX_COMMIT )); then
      reached_limit=1
    fi

    if (( rc == 0 && reached_limit == 1 )); then
      status="PASS"
      reason="reached_${CKPT_MAX_COMMIT}_commits"
    elif (( TIMEOUT_SECS > 0 && rc == 124 )); then
      reason="timeout_before_${CKPT_MAX_COMMIT}_commits"
    elif grep -Eq 'Assertion failed|SIG[A-Z]+|received signal|terminate called|Error:' "${log_file}"; then
      reason="crash_or_error_before_${CKPT_MAX_COMMIT}_commits"
    fi

    tmp_status="${status_file}.tmp.$$.$worker_id"
    printf "%s rc=%s elapsed_seconds=%s reason=%s\n" \
      "${status}" "${rc}" "${elapsed}" "${reason}" >"${tmp_status}"
    mv "${tmp_status}" "${status_file}"
    printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" \
      "${bench}" "${base}" "${status}" "${reason}" "${rc}" \
      "${elapsed}" "${cpu}" "${log_file}" >>"${result_file}"
    echo "[${status}] worker=${worker_id} cpu=${cpu} ${bench}/${base} elapsed=${elapsed}s reason=${reason}"
    [[ "${status}" == "PASS" ]] || failed=1
  done

  return "${failed}"
}

for ((worker = 0; worker < JOBS; worker++)); do
  WORKER_RESULT_FILES+=("${RESULT_DIR}/worker_${worker}.tsv")
  run_worker "${worker}" "${SELECTED_CPUS[$worker]}" &
  WORKER_PIDS+=("$!")
done

FAILED=0
for pid in "${WORKER_PIDS[@]}"; do
  if ! wait "${pid}"; then
    FAILED=1
  fi
done

SUMMARY_TSV="${RESULT_DIR}/summary.tsv"
SUMMARY_TXT="${RESULT_DIR}/summary.txt"
printf "benchmark\tcheckpoint\tstatus\treason\trc\telapsed_seconds\tcpu\tlog\n" >"${SUMMARY_TSV}"
for result_file in "${WORKER_RESULT_FILES[@]}"; do
  [[ -f "${result_file}" ]] && cat "${result_file}" >>"${SUMMARY_TSV}"
done

PASS_COUNT="$(awk -F '\t' 'NR > 1 && $3 == "PASS" { n++ } END { print n + 0 }' "${SUMMARY_TSV}")"
FAIL_COUNT="$(awk -F '\t' 'NR > 1 && $3 == "FAIL" { n++ } END { print n + 0 }' "${SUMMARY_TSV}")"
END_TIME="$(date +%s)"
DURATION=$((END_TIME - START_TIME))

{
  echo "checkpoint replay summary"
  echo "end_time=$(date)"
  echo "simulator=${SIMULATOR}"
  echo "checkpoint_root=${CKPT_ROOT}"
  echo "sdcard_image=${SDCARD_IMAGE}"
  echo "warmup_commits=${CKPT_WARMUP}"
  echo "measure_commits=${CKPT_MAX_COMMIT}"
  echo "difftest=$([[ "${NO_DIFF}" == "1" ]] && echo disabled || echo enabled)"
  echo "workers=${JOBS}"
  echo "pinned_cpus=${SELECTED_CPUS[*]}"
  echo "total=${TOTAL_TASKS}"
  echo "pass=${PASS_COUNT}"
  echo "fail=${FAIL_COUNT}"
  echo "duration_seconds=${DURATION}"
} | tee "${SUMMARY_TXT}"

if (( FAIL_COUNT > 0 || PASS_COUNT != TOTAL_TASKS )); then
  FAILED=1
fi

exit "${FAILED}"
