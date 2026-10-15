# x0 forwarding regressions

These standalone programs run at `0x80000000` without the C startup code.
The load cases preserve the instruction order of the PRF P0 handoff reproducer.
Run with the complete simulator and difftest enabled: intermediate results are
checked at commit even when later instructions overwrite the same register.

| Test | Checks |
| --- | --- |
| `dependent-x0` | Nonzero load to x0 must not contaminate either source operand; original failing ADD is at `0x8000004c`. |
| `dependent-x31` | Control: redirect the discarded load to x31, retaining the arithmetic. |
| `dependent-zero` | Control: both loads read the adjacent zero byte. |
| `lr-x0` | LR-to-x0 still establishes a reservation, allowing a subsequent SC to store successfully. |

Build with LLVM Clang, LLD and llvm-objcopy, then run an already-built simulator:

```sh
make -C baremetal/x0-bypass-test run
```

On a Homebrew installation where LLVM tools are not on PATH:

```sh
make -C baremetal/x0-bypass-test run \
  CLANG=/opt/homebrew/opt/llvm/bin/clang \
  LD_LLD=/opt/homebrew/opt/lld/bin/ld.lld \
  OBJCOPY=/opt/homebrew/opt/llvm/bin/llvm-objcopy
```

Override `SIMULATOR=/absolute/path/to/simulator` to select another binary.
The runner saves logs under `build/run-*`, requires a clean EBREAK with code 0,
and returns nonzero for mismatches, failed checks or timeouts. It does not
rebuild the simulator or overwrite `baremetal/memory`.

To check a simulator with the original bug, after building the images:

```sh
cd baremetal/x0-bypass-test
python3 run.py --binary /absolute/path/to/old/simulator --expect-bug
```

The `--expect-bug` flag changes only the expected result of `dependent-x0`;
all other cases still require success. Different microarchitecture configurations can
change the overlap and need not reproduce the same failure. These tests do not
cover load-x0 page faults, MMIO side effects or RTL.

## Observed results

Using native macOS ARM64 builds of default-profile commit `65dbff4`:
`dependent-x0` reproduces the ADD mismatch before the P0 fix; all four cases
pass after the P0 fix. The x31 and zero-value controls and LR-to-x0 case also
pass before the fix.
