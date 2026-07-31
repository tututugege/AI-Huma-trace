#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

EDA04_SPECIAL_ROOT="/share/personal/S/houruyao/simpoint/special/qimeng1_20260724_mmio_v3"

DEFAULT_SIMULATOR="./build/simulator"
SIMULATOR="${SIMULATOR:-${DEFAULT_SIMULATOR}}"
SPECIAL_ROOT="${SPECIAL_ROOT:-${EDA04_SPECIAL_ROOT}}"
SDCARD_IMAGE="${SDCARD_IMAGE:-${SPECIAL_ROOT}/sdcard.img}"
RESULT_DIR="${RESULT_DIR:-./results_special_1mcommit_$(date +%Y%m%d_%H%M%S)}"
COMMIT_LIMIT="${COMMIT_LIMIT:-1000000}"
TIMEOUT_SECS="${TIMEOUT_SECS:-1800}"
MAX_JOBS="${MAX_JOBS:-4}"
EXPECTED_CKPTS="${EXPECTED_CKPTS:-12}"
PIN_CPUS="${PIN_CPUS:-}"
DRY_RUN="${DRY_RUN:-0}"

usage() {
  cat <<'EOF'
Usage:
  script/run_special_ckpts_1mcommit.sh

Environment overrides:
  SIMULATOR        Simulator binary. Default: ./build/simulator. Set this
                   explicitly when using a deployed build on a server.
  SPECIAL_ROOT     Directory containing special checkpoints. Default:
                   /share/personal/S/houruyao/simpoint/special/qimeng1_20260724_mmio_v3
  SDCARD_IMAGE     SD-card backing image. Default: SPECIAL_ROOT/sdcard.img
  RESULT_DIR       Output directory for per-checkpoint logs and summary.
                   Default: a timestamped directory under the repository root.
  COMMIT_LIMIT     Pass threshold in committed instructions. Default: 1000000
  TIMEOUT_SECS     Per-checkpoint timeout. Default: 1800; 0 disables timeout.
  MAX_JOBS         Parallel workers. Default: 4
  EXPECTED_CKPTS   Required number of checkpoints. Default: 12; 0 disables.
  PIN_CPUS         Comma/space separated logical CPU IDs to use.
                   If unset, the script selects one online logical CPU from each
                   physical core via lscpu, avoiding hyperthread siblings.
  DRY_RUN          1 = print launch plan without running the simulator

Pass rule:
  A checkpoint is PASS if the simulator exits successfully after
  --max-commit=COMMIT_LIMIT. Any assert/abort/non-zero exit is marked FAIL.
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

[[ "${COMMIT_LIMIT}" =~ ^[0-9]+$ ]] || die "COMMIT_LIMIT must be a non-negative integer, got: ${COMMIT_LIMIT}"
(( COMMIT_LIMIT > 0 )) || die "COMMIT_LIMIT must be > 0"
[[ "${TIMEOUT_SECS}" =~ ^[0-9]+$ ]] || die "TIMEOUT_SECS must be a non-negative integer"
[[ "${MAX_JOBS}" =~ ^[0-9]+$ ]] || die "MAX_JOBS must be a positive integer, got: ${MAX_JOBS}"
(( MAX_JOBS > 0 )) || die "MAX_JOBS must be > 0"
[[ "${EXPECTED_CKPTS}" =~ ^[0-9]+$ ]] || die "EXPECTED_CKPTS must be a non-negative integer"
[[ "${DRY_RUN}" =~ ^[01]$ ]] || die "DRY_RUN must be 0 or 1"

[[ -d "${SPECIAL_ROOT}" ]] || die "special checkpoint directory not found: ${SPECIAL_ROOT}"
command -v taskset >/dev/null 2>&1 || die "taskset is required"
command -v lscpu >/dev/null 2>&1 || die "lscpu is required"

[[ -x "${SIMULATOR}" ]] || die "simulator not found or not executable: ${SIMULATOR}"
[[ -f "${SDCARD_IMAGE}" ]] || die "SD-card image not found: ${SDCARD_IMAGE}"

mkdir -p "${RESULT_DIR}"

mapfile -t CKPTS < <(find "${SPECIAL_ROOT}" -maxdepth 1 -type f -name '*.gz' | sort)
(( ${#CKPTS[@]} > 0 )) || die "no .gz checkpoint files found in ${SPECIAL_ROOT}"
TOTAL_TASKS="${#CKPTS[@]}"
if (( EXPECTED_CKPTS > 0 && TOTAL_TASKS != EXPECTED_CKPTS )); then
  die "expected ${EXPECTED_CKPTS} checkpoints, found ${TOTAL_TASKS} in ${SPECIAL_ROOT}"
fi

discover_physical_cpus() {
  lscpu -p=CPU,CORE,SOCKET,ONLINE \
    | awk -F, -v max="${MAX_JOBS}" '
        /^#/ { next }
        $4 != "Y" { next }
        {
          key = $3 ":" $2
          if (!(key in seen)) {
            seen[key] = 1
            cpus[++n] = $1
            if (n == max) {
              exit
            }
          }
        }
        END {
          for (i = 1; i <= n; i++) {
            printf "%s%s", (i == 1 ? "" : " "), cpus[i]
          }
          if (n > 0) {
            printf "\n"
          }
        }
      '
}

if [[ -n "${PIN_CPUS}" ]]; then
  read -r -a SELECTED_CPUS <<<"$(tr ',:' '  ' <<<"${PIN_CPUS}")"
else
  read -r -a SELECTED_CPUS <<<"$(discover_physical_cpus)"
fi

(( ${#SELECTED_CPUS[@]} >= MAX_JOBS )) \
  || die "need ${MAX_JOBS} physical cores, got ${#SELECTED_CPUS[@]} CPUs: ${SELECTED_CPUS[*]:-<none>}"

SELECTED_CPUS=("${SELECTED_CPUS[@]:0:MAX_JOBS}")

SUMMARY_TSV="${RESULT_DIR}/summary.tsv"
SUMMARY_TXT="${RESULT_DIR}/summary.txt"
printf "checkpoint\tstatus\treason\tlog\n" >"${SUMMARY_TSV}"

echo "=================================================="
echo "Start Time:   $(date)"
echo "Simulator:    ${SIMULATOR}"
echo "SD Card:      ${SDCARD_IMAGE}"
echo "Special Root: ${SPECIAL_ROOT}"
echo "Result Dir:   ${RESULT_DIR}"
echo "Commit Limit: ${COMMIT_LIMIT}"
echo "Checkpoints:  ${TOTAL_TASKS}"
echo "Expected:     ${EXPECTED_CKPTS}"
echo "Timeout Secs: ${TIMEOUT_SECS}"
echo "Workers:      ${MAX_JOBS}"
echo "Pinned CPUs:  ${SELECTED_CPUS[*]}"
echo "=================================================="

if [[ "${DRY_RUN}" == "1" ]]; then
  for ((i = 0; i < TOTAL_TASKS; i++)); do
    cpu="${SELECTED_CPUS[$((i % MAX_JOBS))]}"
    ckpt_base="$(basename "${CKPTS[$i]}")"
    echo "[DryRun] CPU ${cpu} <- ${ckpt_base}"
  done
  exit 0
fi

START_TIME="$(date +%s)"
WORKER_PIDS=()
WORKER_RESULT_FILES=()

terminate_workers() {
  echo
  echo "Interrupted; stopping simulator workers..."
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

  for ((task = worker_id; task < TOTAL_TASKS; task += MAX_JOBS)); do
    local ckpt="${CKPTS[$task]}"
    local base log_file rc status reason

    base="$(basename "${ckpt}" .gz)"
    log_file="${RESULT_DIR}/${base}.log"

    echo "[Start] worker=${worker_id} cpu=${cpu} ckpt=${base}"

    local simulator_cmd=(
      "${SIMULATOR}"
      --mode ckpt
      -w 0
      -c "${COMMIT_LIMIT}"
      --sdcard-image "${SDCARD_IMAGE}"
      "${ckpt}"
    )

    set +e
    if (( TIMEOUT_SECS > 0 )); then
      timeout --preserve-status "${TIMEOUT_SECS}" \
        taskset -c "${cpu}" "${simulator_cmd[@]}" \
        >"${log_file}" 2>&1
      rc=$?
    else
      taskset -c "${cpu}" "${simulator_cmd[@]}" \
        >"${log_file}" 2>&1
      rc=$?
    fi
    set -e

    status="FAIL"
    reason="unknown"

    local reported_commits=""
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

    local reached_limit=0
    if grep -q "Reached MAX_COMMIT_INST=${COMMIT_LIMIT}" "${log_file}"; then
      reached_limit=1
    elif [[ "${reported_commits}" =~ ^[0-9]+$ ]] &&
      (( reported_commits >= COMMIT_LIMIT )); then
      reached_limit=1
    fi

    if [[ "${rc}" -eq 0 && "${reached_limit}" -eq 1 ]]; then
      status="PASS"
      reason="reached_${COMMIT_LIMIT}_commits"
    elif grep -Eq "Assertion failed|SIG[A-Z]+|received signal|Program received signal|terminate called" "${log_file}"; then
      reason="crash_or_assert_before_${COMMIT_LIMIT}_commits"
      failed=1
    elif (( TIMEOUT_SECS > 0 )) && [[ "${rc}" -eq 124 ]]; then
      reason="timeout_before_${COMMIT_LIMIT}_commits"
      failed=1
    elif grep -q "Success!!!!" "${log_file}"; then
      reason="exited_before_${COMMIT_LIMIT}_commits"
      failed=1
    else
      reason="sim_rc_${rc}"
      if [[ "${status}" != "PASS" ]]; then
        failed=1
      fi
    fi

    printf "%s\t%s\t%s\t%s\n" "${base}" "${status}" "${reason}" "${log_file}" >>"${result_file}"
    echo "[${status}] worker=${worker_id} cpu=${cpu} ckpt=${base} reason=${reason}"
  done

  return "${failed}"
}

for ((worker = 0; worker < MAX_JOBS; worker++)); do
  WORKER_RESULT_FILES+=("${RESULT_DIR}/worker_${worker}.tsv")
  run_worker "${worker}" "${SELECTED_CPUS[$worker]}" &
  WORKER_PIDS+=("$!")
done

FAILED=0
set +e
for pid in "${WORKER_PIDS[@]}"; do
  wait "${pid}"
  rc="$?"
  if (( rc != 0 )); then
    FAILED=1
  fi
done
set -e

pass_count=0
fail_count=0
total_count=0

for result_file in "${WORKER_RESULT_FILES[@]}"; do
  [[ -f "${result_file}" ]] || continue
  cat "${result_file}" >>"${SUMMARY_TSV}"
done

while IFS=$'\t' read -r checkpoint status reason log_path; do
  [[ "${checkpoint}" == "checkpoint" ]] && continue
  ((total_count+=1))
  if [[ "${status}" == "PASS" ]]; then
    ((pass_count+=1))
  else
    ((fail_count+=1))
  fi
done <"${SUMMARY_TSV}"

{
  echo "special checkpoint smoke summary"
  echo "end_time=$(date)"
  echo "simulator=${SIMULATOR}"
  echo "sdcard_image=${SDCARD_IMAGE}"
  echo "special_root=${SPECIAL_ROOT}"
  echo "result_dir=${RESULT_DIR}"
  echo "commit_limit=${COMMIT_LIMIT}"
  echo "workers=${MAX_JOBS}"
  echo "total=${total_count}"
  echo "pass=${pass_count}"
  echo "fail=${fail_count}"
  echo
  column -t -s $'\t' "${SUMMARY_TSV}"
} >"${SUMMARY_TXT}"

END_TIME="$(date +%s)"
DURATION="$((END_TIME - START_TIME))"

echo "=================================================="
echo "Finished: total=${total_count} pass=${pass_count} fail=${fail_count}"
echo "Duration: $((DURATION / 3600))h $(((DURATION % 3600) / 60))m $((DURATION % 60))s"
echo "Summary:  ${SUMMARY_TXT}"
echo "TSV:      ${SUMMARY_TSV}"
echo "=================================================="

if (( FAILED != 0 || fail_count > 0 )); then
  exit 1
fi
