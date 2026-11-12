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
| `sc-x0` | Successful SC-to-x0 still stores; failed SC-to-x0 does not store or contaminate x0 consumers. |
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

Direct host-side tests of `RealLsu::comb_stlf()` (the legacy LSU, `back-end/Lsu/RealLsu.cpp`, used with `LSU_PIPE=0`; the default staged LSU `RealLsuPipe.cpp` is covered only by the whole-simulator cases above). **Stale:** this target no longer builds (the Makefile names the removed `RealLsuPerf.cpp`, and `stlf-unit.cpp` uses `LsuState` members that no longer exist); it needs porting to the current `RealLsu.h` before use:

```sh
make -C baremetal/x0-bypass-test host-test
```

These explicitly cover pending SC retry, successful/failed resolution on retry,
skipping a failed SC to reach an older pending store, falling back to cache when
only failed stores remain, and ordinary youngest-store forwarding priority.
No RISC-V compiler is needed for this target. Initialize the repository's
submodules first, as for building the simulator.

To check a simulator with the original bug, after building the images:

```sh
cd baremetal/x0-bypass-test
python3 run.py --binary /absolute/path/to/old/simulator --expect-bug
```

The `--expect-bug` flag changes only the expected result of `dependent-x0`;
all other cases still require success. Different microarchitecture configurations can
change the overlap and need not reproduce the same failure. The SC case is a
semantic regression, not a claim of deterministic SC bypass failure on old
versions. These tests do not cover load-x0 page faults, MMIO side effects or RTL.

## Observed results

On default-profile commit `65dbff4`, using native macOS ARM64 builds:

| Case | Before P0 fix | P0 fix only | P0 and SC forwarding fixes |
| --- | --- | --- | --- |
| `dependent-x0` | Expected ADD mismatch at `0x8000004c` | PASS | PASS |
| `dependent-x31` | PASS | PASS | PASS |
| `dependent-zero` | PASS | PASS | PASS |
| `lr-x0` | PASS | PASS | PASS |
| `sc-x0` | FAIL | FAIL | PASS |

`sc-x0` exposed a separate issue: the load at `0x80000040` read
`0x77` after a failed SC, while the reference expects the earlier `0x55`.
Diagnostic logging confirmed forwarding from an SC with `sc_pass=0` and
`suppress_write=1`; no `0x77` cache write was issued. The fix skips suppressed
stores while continuing the older-store search, and retries overlapping loads
until an undecided SC resolves. The host test fails on the old LSU and passes
on the fixed implementation. No fences are inserted to hide the mismatch.
