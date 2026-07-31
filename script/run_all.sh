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

Scheduling and safety:
  - workers pull from a shared queue, so an idle CPU immediately takes the
    next checkpoint instead of waiting for a statically assigned worker
  - RESULT_DIR is locked against duplicate runners
  - run_config.txt prevents mixing results from different simulator binaries
    or warmup/measure/difftest configurations
  - progress.txt reports claimed/completed/pass/fail/running/pending live

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
command -v flock >/dev/null 2>&1 || die "flock is required"
command -v setsid >/dev/null 2>&1 || die "setsid is required"
command -v sha256sum >/dev/null 2>&1 || die "sha256sum is required"
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
  read -r -a CANDIDATE_CPUS <<<"$(tr ',:' '  ' <<<"${PIN_CPUS}")"
else
  read -r -a CANDIDATE_CPUS <<<"$(discover_physical_cpus)"
fi

AVAILABLE_CPUS=()
declare -A SEEN_CPUS=()
for cpu in "${CANDIDATE_CPUS[@]}"; do
  [[ "${cpu}" =~ ^[0-9]+$ ]] || die "invalid CPU ID: ${cpu}"
  [[ -z "${SEEN_CPUS[$cpu]+x}" ]] || die "duplicate CPU ID selected: ${cpu}"
  SEEN_CPUS["${cpu}"]=1
  if taskset -c "${cpu}" true >/dev/null 2>&1; then
    AVAILABLE_CPUS+=("${cpu}")
  elif [[ -n "${PIN_CPUS}" ]]; then
    die "CPU ${cpu} is not available in this process affinity mask"
  fi
done
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
echo "Scheduler:        dynamic shared queue; idle workers pull the next checkpoint"
echo "Resume:           ${RESUME}"
echo "=================================================="

if [[ "${DRY_RUN}" == "1" ]]; then
  for ((worker = 0; worker < JOBS; worker++)); do
    echo "[DryRun] worker=${worker} cpu=${SELECTED_CPUS[$worker]} scheduler=dynamic"
  done
  exit 0
fi

mkdir -p "${RESULT_DIR}"
for ckpt in "${ALL_CKPTS[@]}"; do
  mkdir -p "${RESULT_DIR}/$(basename "$(dirname "${ckpt}")")"
done

RUN_LOCK_FILE="${RESULT_DIR}/.run_all.lock"
exec {RUN_LOCK_FD}>"${RUN_LOCK_FILE}"
flock -n "${RUN_LOCK_FD}" \
  || die "another run_all.sh is already using RESULT_DIR=${RESULT_DIR}"
if [[ -f "${RESULT_DIR}/runner.pid" ]]; then
  previous_runner_pid="$(sed -n '1p' "${RESULT_DIR}/runner.pid")"
  if [[ "${previous_runner_pid}" =~ ^[0-9]+$ ]] \
    && kill -0 "${previous_runner_pid}" 2>/dev/null \
    && ps -p "${previous_runner_pid}" -o args= 2>/dev/null | grep -Fq 'run_all.sh'; then
    die "runner PID ${previous_runner_pid} is still active for RESULT_DIR=${RESULT_DIR}"
  fi
fi
printf "%s\n" "$$" >"${RESULT_DIR}/runner.pid"

SIMULATOR_REAL="$(readlink -f "${SIMULATOR}")"
CKPT_ROOT_REAL="$(readlink -f "${CKPT_ROOT}")"
if [[ -n "${SDCARD_IMAGE}" ]]; then
  SDCARD_IMAGE_REAL="$(readlink -f "${SDCARD_IMAGE}")"
else
  SDCARD_IMAGE_REAL=""
fi
RUN_CONFIG_FILE="${RESULT_DIR}/run_config.txt"
RUN_CONFIG_TMP="${RUN_CONFIG_FILE}.tmp.${BASHPID}"
{
  echo "format_version=2"
  echo "simulator=${SIMULATOR_REAL}"
  echo "simulator_sha256=$(sha256sum "${SIMULATOR_REAL}" | awk '{print $1}')"
  echo "checkpoint_root=${CKPT_ROOT_REAL}"
  echo "sdcard_image=${SDCARD_IMAGE_REAL}"
  echo "warmup_commits=${CKPT_WARMUP}"
  echo "measure_commits=${CKPT_MAX_COMMIT}"
  echo "difftest=$([[ "${NO_DIFF}" == "1" ]] && echo disabled || echo enabled)"
  echo "checkpoint_count=${TOTAL_TASKS}"
} >"${RUN_CONFIG_TMP}"
if [[ -f "${RUN_CONFIG_FILE}" ]] && ! cmp -s "${RUN_CONFIG_FILE}" "${RUN_CONFIG_TMP}"; then
  rm -f "${RUN_CONFIG_TMP}"
  die "RESULT_DIR contains results from a different simulator or run configuration: ${RESULT_DIR}"
fi
if [[ ! -f "${RUN_CONFIG_FILE}" ]]; then
  if find "${RESULT_DIR}" -type f -name '*.status' -print -quit | grep -q .; then
    echo "Warning: adopting legacy result directory without run_config.txt: ${RESULT_DIR}" >&2
  fi
  mv "${RUN_CONFIG_TMP}" "${RUN_CONFIG_FILE}"
else
  rm -f "${RUN_CONFIG_TMP}"
fi

START_TIME="$(date +%s)"
WORKER_PIDS=()
WORKER_RESULT_FILES=()
QUEUE_LOCK_FILE="${RESULT_DIR}/.queue.lock"
QUEUE_STATE_FILE="${RESULT_DIR}/.queue.state"
PROGRESS_FILE="${RESULT_DIR}/progress.txt"
printf "0 0 0 0 0\n" >"${QUEUE_STATE_FILE}"

write_progress_locked() {
  local next="$1"
  local completed="$2"
  local passed="$3"
  local failed="$4"
  local resumed="$5"
  local running=$((next - completed))
  local pending=$((TOTAL_TASKS - next))
  local tmp="${PROGRESS_FILE}.tmp.${BASHPID}"
  {
    echo "updated=$(date '+%F %T %Z')"
    echo "total=${TOTAL_TASKS}"
    echo "claimed=${next}"
    echo "completed=${completed}"
    echo "pass=${passed}"
    echo "fail=${failed}"
    echo "resumed=${resumed}"
    echo "running=${running}"
    echo "pending=${pending}"
  } >"${tmp}"
  mv "${tmp}" "${PROGRESS_FILE}"
}

write_progress_locked 0 0 0 0 0

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
  local queue_lock_fd
  local current_job_pgid=""
  : >"${result_file}"

  exec {queue_lock_fd}>"${QUEUE_LOCK_FILE}"

  stop_worker() {
    if [[ -n "${current_job_pgid}" ]]; then
      kill -TERM -- "-${current_job_pgid}" 2>/dev/null || true
      sleep 1
      kill -KILL -- "-${current_job_pgid}" 2>/dev/null || true
      wait "${current_job_pgid}" 2>/dev/null || true
    fi
    exit 130
  }
  trap stop_worker INT TERM

  claim_task() {
    local next completed passed fail_count resumed
    flock -x "${queue_lock_fd}"
    read -r next completed passed fail_count resumed <"${QUEUE_STATE_FILE}"
    if (( next >= TOTAL_TASKS )); then
      flock -u "${queue_lock_fd}"
      return 1
    fi
    task="${next}"
    next=$((next + 1))
    printf "%s %s %s %s %s\n" \
      "${next}" "${completed}" "${passed}" "${fail_count}" "${resumed}" \
      >"${QUEUE_STATE_FILE}"
    write_progress_locked "${next}" "${completed}" "${passed}" "${fail_count}" "${resumed}"
    claimed_count="${next}"
    flock -u "${queue_lock_fd}"
  }

  record_completion() {
    local completion_status="$1"
    local was_resumed="$2"
    local next completed passed fail_count resumed
    flock -x "${queue_lock_fd}"
    read -r next completed passed fail_count resumed <"${QUEUE_STATE_FILE}"
    completed=$((completed + 1))
    if [[ "${completion_status}" == "PASS" ]]; then
      passed=$((passed + 1))
    else
      fail_count=$((fail_count + 1))
    fi
    resumed=$((resumed + was_resumed))
    printf "%s %s %s %s %s\n" \
      "${next}" "${completed}" "${passed}" "${fail_count}" "${resumed}" \
      >"${QUEUE_STATE_FILE}"
    write_progress_locked "${next}" "${completed}" "${passed}" "${fail_count}" "${resumed}"
    progress_completed="${completed}"
    progress_passed="${passed}"
    progress_failed="${fail_count}"
    flock -u "${queue_lock_fd}"
  }

  while claim_task; do
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
      record_completion PASS 1
      echo "[Skip] worker=${worker_id} cpu=${cpu} ${bench}/${base} already PASS progress=${progress_completed}/${TOTAL_TASKS}"
      continue
    fi

    echo "[Start] worker=${worker_id} cpu=${cpu} queue=${claimed_count}/${TOTAL_TASKS} ${bench}/${base}"
    started="$(date +%s)"
    local -a launch_cmd
    if (( TIMEOUT_SECS > 0 )); then
      launch_cmd=(timeout "${TIMEOUT_SECS}" taskset -c "${cpu}" "${SIM_ARGS[@]}" "${ckpt}")
    else
      launch_cmd=(taskset -c "${cpu}" "${SIM_ARGS[@]}" "${ckpt}")
    fi
    setsid "${launch_cmd[@]}" >"${log_file}" 2>&1 &
    current_job_pgid="$!"
    wait "${current_job_pgid}"
    rc=$?
    current_job_pgid=""
    finished="$(date +%s)"
    elapsed=$((finished - started))

    status="FAIL"
    reason="sim_rc_${rc}"
    reported_commits="$(
      sed -nE \
        's/.*committed\(total\/load\/store\)=[[:space:]]*([0-9]+)[[:space:]]*\/.*/\1/p' \
        "${log_file}" | tail -n 1
    )"
    if [[ -z "${reported_commits}" ]]; then
      reported_commits="$(
        sed -nE 's/.*instruction num:[[:space:]]*([0-9]+).*/\1/p' \
          "${log_file}" | tail -n 1
      )"
    fi
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
    elif grep -Eq 'Assertion failed|SIG[A-Z]+|received signal|Program received signal|terminate called|Error:' "${log_file}"; then
      reason="crash_or_error_before_${CKPT_MAX_COMMIT}_commits"
    fi

    tmp_status="${status_file}.tmp.$$.$worker_id"
    printf "%s rc=%s elapsed_seconds=%s reason=%s\n" \
      "${status}" "${rc}" "${elapsed}" "${reason}" >"${tmp_status}"
    mv "${tmp_status}" "${status_file}"
    printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" \
      "${bench}" "${base}" "${status}" "${reason}" "${rc}" \
      "${elapsed}" "${cpu}" "${log_file}" >>"${result_file}"
    record_completion "${status}" 0
    echo "[${status}] worker=${worker_id} cpu=${cpu} ${bench}/${base} elapsed=${elapsed}s reason=${reason} progress=${progress_completed}/${TOTAL_TASKS} pass=${progress_passed} fail=${progress_failed}"
    [[ "${status}" == "PASS" ]] || failed=1
  done

  exec {queue_lock_fd}>&-
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
SUMMARY_TMP="${SUMMARY_TSV}.tmp.${BASHPID}"
printf "benchmark\tcheckpoint\tstatus\treason\trc\telapsed_seconds\tcpu\tlog\n" >"${SUMMARY_TMP}"
for result_file in "${WORKER_RESULT_FILES[@]}"; do
  [[ -f "${result_file}" ]] && cat "${result_file}" >>"${SUMMARY_TMP}"
done
{
  head -n 1 "${SUMMARY_TMP}"
  tail -n +2 "${SUMMARY_TMP}" | sort -t $'\t' -k1,1 -k2,2
} >"${SUMMARY_TSV}"
rm -f "${SUMMARY_TMP}" "${QUEUE_STATE_FILE}" "${QUEUE_LOCK_FILE}"

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
  echo "scheduler=dynamic_shared_queue"
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
