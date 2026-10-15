"""Run x0 regressions with per-commit difftest; retain logs for every case."""
import argparse
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=Path('../../build/simulator'))
    parser.add_argument('--build-dir', type=Path, default=Path('build'))
    parser.add_argument('--expect-bug', action='store_true',
                        help='require the original ADD mismatch in dependent-x0')
    args = parser.parse_args()
    binary = args.binary.resolve()
    build = args.build_dir.resolve()
    logs = Path(tempfile.mkdtemp(prefix='run-', dir=build))
    passed = True
    for case in ('dependent-x0', 'dependent-x31', 'dependent-zero', 'lr-x0'):
        work = logs / case
        work.mkdir()
        log_path = work / 'console.log'
        command = [str(binary), '--mode', 'run', '--max-commit', '1000',
                   str(build / (case + '.bin'))]
        timed_out = False
        with log_path.open('w') as log:
            try:
                result = subprocess.run(command, cwd=work, stdout=log,
                                        stderr=subprocess.STDOUT, timeout=120)
            except subprocess.TimeoutExpired:
                timed_out = True
        text = log_path.read_text(errors='replace')
        expected_bug = args.expect_bug and case == 'dependent-x0'
        if timed_out:
            ok = False
        elif expected_bug:
            ok = result.returncode != 0 and all(token in text for token in (
                'Difftest: error', 'Commit PC: 0x8000004c', '00000158'))
        else:
            ok = (result.returncode == 0 and 'Difftest: error' not in text
                  and '[RefCPU]' in text
                  and 'ebreak signal received. code = 0x0' in text)
        passed &= ok
        label = 'EXPECTED_BUG' if ok and expected_bug else 'PASS' if ok else 'FAIL'
        print(f'{case}: {label}', flush=True)
    print(f'Logs: {logs}')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
