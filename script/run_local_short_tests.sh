#!/usr/bin/env bash

set -uo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
TEST_DIR="$REPO_ROOT/baremetal/test"

TIMEOUT_SECONDS=${TIMEOUT_SECONDS:-180}
JOBS=${JOBS:-2}
INCLUDE_LONG=0
LIST_ONLY=0
LOG_ROOT=""

# Tests in this list are skipped by default because they take substantially
# longer than the rest of baremetal/test/cpu-tests.
LONG_TESTS=(memory-stress)

usage() {
    cat <<'EOF'
Usage: script/run_local_short_tests.sh [options]

Build the default simulator and run every local short cpu-test.

Options:
  --include-long       Also run tests classified as long.
  --timeout SECONDS    Per-test timeout (default: 180).
  --jobs N             Parallel jobs used for compilation (default: 2).
  --log-dir DIR        Store per-test logs in DIR instead of a temporary dir.
  --list               List selected tests without running them.
  -h, --help           Show this help.

Environment overrides:
  TIMEOUT_SECONDS      Same as --timeout.
  JOBS                 Same as --jobs.
EOF
}

die() {
    echo "error: $*" >&2
    exit 2
}

is_positive_integer() {
    [[ $1 =~ ^[1-9][0-9]*$ ]]
}

is_long_test() {
    local candidate=$1
    local long_test
    for long_test in "${LONG_TESTS[@]}"; do
        if [[ $candidate == "$long_test" ]]; then
            return 0
        fi
    done
    return 1
}

while (($# > 0)); do
    case $1 in
    --include-long)
        INCLUDE_LONG=1
        shift
        ;;
    --timeout)
        (($# >= 2)) || die "--timeout requires a value"
        TIMEOUT_SECONDS=$2
        shift 2
        ;;
    --jobs)
        (($# >= 2)) || die "--jobs requires a value"
        JOBS=$2
        shift 2
        ;;
    --log-dir)
        (($# >= 2)) || die "--log-dir requires a value"
        LOG_ROOT=$2
        shift 2
        ;;
    --list)
        LIST_ONLY=1
        shift
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
is_positive_integer "$JOBS" || die "jobs must be a positive integer"
[[ -d $TEST_DIR/cpu-tests ]] || die "missing test directory: $TEST_DIR/cpu-tests"

mapfile -t ALL_TESTS < <(
    find "$TEST_DIR/cpu-tests" -maxdepth 1 -type f -name '*.c' -printf '%f\n' |
        sed 's/\.c$//' |
        sort
)

SELECTED_TESTS=()
SKIPPED_TESTS=()
for test_name in "${ALL_TESTS[@]}"; do
    if is_long_test "$test_name" && ((INCLUDE_LONG == 0)); then
        SKIPPED_TESTS+=("$test_name")
    else
        SELECTED_TESTS+=("$test_name")
    fi
done

if ((LIST_ONLY)); then
    printf '%s\n' "${SELECTED_TESTS[@]}"
    if ((${#SKIPPED_TESTS[@]} > 0)); then
        echo "Skipped long tests: ${SKIPPED_TESTS[*]}" >&2
    fi
    exit 0
fi

if [[ -z $LOG_ROOT ]]; then
    LOG_ROOT=$(mktemp -d /tmp/qimeng-local-short-tests.XXXXXX)
else
    mkdir -p "$LOG_ROOT"
    LOG_ROOT=$(cd -- "$LOG_ROOT" && pwd)
fi

echo "[build] GenSimpoint reference library"
make -C "$REPO_ROOT/GenSimpoint" -j"$JOBS" librefcpu.a

echo "[build] default simulator"
make -C "$REPO_ROOT" -j"$JOBS" default

export PROFILE=default
passed=0
failed=0

echo "[run] ${#SELECTED_TESTS[@]} tests, timeout=${TIMEOUT_SECONDS}s, logs=$LOG_ROOT"
cd "$TEST_DIR" || exit 2

for test_name in "${SELECTED_TESTS[@]}"; do
    log_file="$LOG_ROOT/$test_name.log"
    if timeout "$TIMEOUT_SECONDS" make -s -f ../common.mk \
        COMMON_DIR=.. \
        TARGET="$test_name" \
        C_SRCS="cpu-tests/$test_name.c" \
        >"$log_file" 2>&1; then
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
if ((${#SKIPPED_TESTS[@]} > 0)); then
    echo "SKIPPED long tests: ${SKIPPED_TESTS[*]}"
fi
echo "LOGS $LOG_ROOT"

((failed == 0))
