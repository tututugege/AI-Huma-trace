#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

RESULTS_ROOT="${RESULTS_ROOT:-/nfs_global/S/houruyao/simpoint_20260719/results}"
WEIGHTS_DIR="${WEIGHTS_DIR:-/share/personal/S/houruyao/simpoint_20260719/simpoints}"
EXPECTED_CKPTS="${EXPECTED_CKPTS:-629}"
CPU_FREQ_GHZ="${2:-${CPU_FREQ_GHZ:-1.0}}"
ALLOW_INCOMPLETE="${ALLOW_INCOMPLETE:-0}"
DEBUG="${DEBUG:-0}"
CAL_SPEC="${CAL_SPEC:-${SCRIPT_DIR}/analysis/cal_spec.py}"

usage() {
  cat <<'EOF'
Usage:
  script/score_spec2006.sh [RESULT_DIR] [CPU_FREQ_GHZ]

With no RESULT_DIR, the newest completed run under RESULTS_ROOT is selected.

Environment overrides:
  RESULTS_ROOT      Root used for automatic result discovery.
  WEIGHTS_DIR       Directory containing the 12 SimPoint .weights files.
  EXPECTED_CKPTS    Expected number of checkpoint status files (default: 629).
  CPU_FREQ_GHZ      Estimated target CPU frequency in GHz (default: 1.0).
  ALLOW_INCOMPLETE  Set to 1 to calculate despite incomplete/failed checkpoints.
  DEBUG             Set to 1 for verbose cal_spec.py diagnostics.
  CAL_SPEC          Path to cal_spec.py.

Examples:
  script/score_spec2006.sh
  script/score_spec2006.sh /path/to/results
  script/score_spec2006.sh /path/to/results 1.5
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

find_latest_complete_result() {
  local candidate
  [[ -d "${RESULTS_ROOT}" ]] || return 1
  while IFS= read -r candidate; do
    [[ -s "${candidate}/summary.txt" ]] || continue
    grep -q "^total=${EXPECTED_CKPTS}$" "${candidate}/summary.txt" || continue
    grep -q "^pass=${EXPECTED_CKPTS}$" "${candidate}/summary.txt" || continue
    grep -q '^fail=0$' "${candidate}/summary.txt" || continue
    printf '%s\n' "${candidate}"
    return 0
  done < <(
    find "${RESULTS_ROOT}" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' \
      | sort -nr \
      | cut -d' ' -f2-
  )
  return 1
}

RESULT_DIR="${1:-${RESULT_DIR:-}}"
if [[ -z "${RESULT_DIR}" ]]; then
  RESULT_DIR="$(find_latest_complete_result)" \
    || die "no completed ${EXPECTED_CKPTS}-checkpoint result found under ${RESULTS_ROOT}"
  echo "Auto-selected result: ${RESULT_DIR}"
fi

[[ "${EXPECTED_CKPTS}" =~ ^[0-9]+$ ]] || die "EXPECTED_CKPTS must be an integer"
[[ "${ALLOW_INCOMPLETE}" =~ ^[01]$ ]] || die "ALLOW_INCOMPLETE must be 0 or 1"
[[ "${CPU_FREQ_GHZ}" =~ ^[0-9]+([.][0-9]+)?$ ]] \
  || die "CPU_FREQ_GHZ must be a positive number"
awk -v freq="${CPU_FREQ_GHZ}" 'BEGIN { exit !(freq > 0) }' \
  || die "CPU_FREQ_GHZ must be greater than zero"

[[ -d "${RESULT_DIR}" ]] || die "result directory not found: ${RESULT_DIR}"
[[ -d "${WEIGHTS_DIR}" ]] || die "weights directory not found: ${WEIGHTS_DIR}"
[[ -f "${CAL_SPEC}" ]] || die "cal_spec.py not found: ${CAL_SPEC}"

mapfile -t STATUS_FILES < <(find "${RESULT_DIR}" -type f -name '*.status' | sort)
STATUS_COUNT="${#STATUS_FILES[@]}"
PASS_COUNT=0
FAIL_COUNT=0
UNKNOWN_COUNT=0
for status_file in "${STATUS_FILES[@]}"; do
  read -r status _ <"${status_file}" || status=""
  case "${status}" in
    PASS) PASS_COUNT=$((PASS_COUNT + 1)) ;;
    FAIL) FAIL_COUNT=$((FAIL_COUNT + 1)) ;;
    *) UNKNOWN_COUNT=$((UNKNOWN_COUNT + 1)) ;;
  esac
done

WEIGHT_COUNT="$(find "${WEIGHTS_DIR}" -maxdepth 1 -type f -name '*.weights' | wc -l)"

echo "=================================================="
echo "SPEC2006 score input validation"
echo "Result Dir:       ${RESULT_DIR}"
echo "Weights Dir:      ${WEIGHTS_DIR}"
echo "Checkpoint State: total=${STATUS_COUNT} pass=${PASS_COUNT} fail=${FAIL_COUNT} unknown=${UNKNOWN_COUNT}"
echo "Weight Files:     ${WEIGHT_COUNT}"
echo "CPU Frequency:    ${CPU_FREQ_GHZ} GHz"
echo "=================================================="

if [[ "${ALLOW_INCOMPLETE}" != "1" ]]; then
  (( STATUS_COUNT == EXPECTED_CKPTS )) \
    || die "expected ${EXPECTED_CKPTS} checkpoint statuses, found ${STATUS_COUNT}"
  (( PASS_COUNT == EXPECTED_CKPTS && FAIL_COUNT == 0 && UNKNOWN_COUNT == 0 )) \
    || die "checkpoint results are not complete and clean"
  (( WEIGHT_COUNT == 12 )) || die "expected 12 weight files, found ${WEIGHT_COUNT}"
fi

LOG_ROOT_DIR="${RESULT_DIR}" \
WEIGHTS_DIR="${WEIGHTS_DIR}" \
CPU_FREQ_GHZ="${CPU_FREQ_GHZ}" \
DEBUG="${DEBUG}" \
python3 "${CAL_SPEC}"

PERF_REPORT="${RESULT_DIR}/perf_report.txt"
STATUS_REPORT="${RESULT_DIR}/log_status_report.txt"
SCORE_SUMMARY="${RESULT_DIR}/spec_score_summary.txt"

[[ -s "${PERF_REPORT}" ]] || die "performance report was not generated"
grep -q '^Final Estimated SPECint2006:' "${PERF_REPORT}" \
  || die "no final SPECint2006 score was produced; inspect ${STATUS_REPORT}"

awk -v freq="${CPU_FREQ_GHZ}" '
  BEGIN {
    printf "Estimated SPECint2006 at %s GHz\n", freq
    printf "%-16s %10s %10s %12s\n", "Benchmark", "IPC", "CPI", "SPEC Ratio"
  }
  /^Benchmark:/    { bench=$2 }
  /^Weighted IPC:/ { ipc=$3 }
  /^Weighted CPI:/ { cpi=$3 }
  /^SPEC Ratio:/ {
    printf "%-16s %10s %10s %12s\n", bench, ipc, cpi, $3
  }
  /^Final Estimated SPECint2006:/ {
    printf "\nFinal Estimated SPECint2006: %s\n", $4
  }
' "${PERF_REPORT}" | tee "${SCORE_SUMMARY}"

echo
echo "Full report:    ${PERF_REPORT}"
echo "Status report:  ${STATUS_REPORT}"
echo "Score summary:  ${SCORE_SUMMARY}"
