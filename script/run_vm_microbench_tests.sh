#!/usr/bin/env bash

set -uo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)

TIMEOUT_SECONDS=${TIMEOUT_SECONDS:-180}
LOG_ROOT=""

VM_TESTS=(
    vm-pagefault-test
    vm-sfence-perm-test
    vm-asid-test
    vm-sum-fetch-test
)

usage() {
    cat <<'EOF'
Usage: script/run_vm_microbench_tests.sh [options]

Run the privileged VM microbench tests with the already-built default
simulator.

Options:
  --timeout SECONDS    Per-test timeout (default: 180).
  --log-dir DIR        Store per-test logs in DIR instead of a temporary dir.
  --list               List selected tests without running them.
  -h, --help           Show this help.

Environment overrides:
  TIMEOUT_SECONDS      Same as --timeout.
EOF
}

die() {
    echo "error: $*" >&2
    exit 2
}

is_positive_integer() {
    [[ $1 =~ ^[1-9][0-9]*$ ]]
}

while (($# > 0)); do
    case $1 in
    --timeout)
        (($# >= 2)) || die "--timeout requires a value"
        TIMEOUT_SECONDS=$2
        shift 2
        ;;
    --log-dir)
        (($# >= 2)) || die "--log-dir requires a value"
        LOG_ROOT=$2
        shift 2
        ;;
    --list)
        printf '%s\n' "${VM_TESTS[@]}"
        exit 0
        ;;
    -h | --help)
        usage
        exit 0
        ;;
    *)
        die "unknown option: $1"
        ;;
    esac
done

is_positive_integer "$TIMEOUT_SECONDS" || die "timeout must be a positive integer"
[[ -x $REPO_ROOT/build/simulator ]] ||
    die "missing build/simulator; build the default simulator first"

if [[ -z $LOG_ROOT ]]; then
    LOG_ROOT=$(mktemp -d /tmp/qimeng-vm-microbench.XXXXXX)
else
    mkdir -p "$LOG_ROOT"
    LOG_ROOT=$(cd -- "$LOG_ROOT" && pwd)
fi

passed=0
failed=0

echo "[run] ${#VM_TESTS[@]} VM tests, timeout=${TIMEOUT_SECONDS}s, logs=$LOG_ROOT"
for test_name in "${VM_TESTS[@]}"; do
    test_dir="$REPO_ROOT/baremetal/$test_name"
    log_file="$LOG_ROOT/$test_name.log"
    if timeout "$TIMEOUT_SECONDS" make -C "$test_dir" >"$log_file" 2>&1; then
        ((passed += 1))
        printf 'PASS  %s\n' "$test_name"
    else
        rc=$?
        ((failed += 1))
        printf 'FAIL  %s (rc=%d, log=%s)\n' "$test_name" "$rc" "$log_file"
        tail -30 "$log_file"
    fi
done

echo "SUMMARY passed=$passed failed=$failed total=$((passed + failed))"
echo "LOGS $LOG_ROOT"

((failed == 0))
