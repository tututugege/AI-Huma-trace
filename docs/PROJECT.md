# AI-Huma / trace-model: project record

AI-Huma is the whole project (AI + human working together; also a nod to Rem's magic). It has three parts:
- `function-model` (`~/personal/AI-Huma/function-model`): the functional RISC-V core, tututugege/GenSimpoint plus our
  commits; built as `librefcpu.a`. Used here as the submodule `GenSimpoint/`.
- `trace-model` (this repo): the cycle-identical timing model, fed with facts by the function model.
- `rtl` (`~/personal/tututugege-rtl`, to be moved to `~/personal/AI-Huma/rtl` once its agents finish): the SystemVerilog
  translation of the modules, checked cycle by cycle against the trace model.
The sections below describe trace-model.

Trace-driven timing model distilled from tututugege/simulator (RTL-style, execution-driven, cycle-accurate OoO RISC-V).
Goal: cycle-identical to the original, much smaller and faster, later paired with generated RTL.
Every change must keep identical cycle counts; cycle equality is the correctness gate.

## Origin and history
- Forked from tututugege/simulator at 2f31e8d (`upstream` remote). `main` here is the former `trace-distill` branch.
- `axi-interconnect-kit/` (from dyh1807/axi-interconnect-kit, local commits up to 382cb6f) is vendored as a plain
  directory. `GenSimpoint/` is a submodule of the function-model repo (URL is a local path until it is published).
- The pre-merge patch-queue repo (patches 0001..0061, step docs) is `../tututugege-trace`; its docs are in `docs/`,
  its tests/scripts in `tests/` and `scripts/`.
- The original model stays available as a baseline binary (`~/personal/llm4trace/third_party/tututugege/baseline-sim/src`, 2f31e8d + exact kit patches); the
  baseline cycle counts are in `results/baseline/baseline.csv`.

## Architecture in one paragraph
The model no longer computes values (ALU/MUL/DIV/FP, PRF, L1D data, LLC data table are gone). Branch direction/target,
addresses, store data and CSR write data come from facts supplied by the trace bridge (`diff/TraceBridge.*`), a
functional core (librefcpu, GenSimpoint) that follows the model's dispatch stream including wrong paths. Functional
memory is written at ROB commit from facts; instruction/PTE/LLC read data come from pmem.

## Gate (run after every change)
All scripts take the repo root as `<simulator-repo>`, e.g. `scripts/run_gate.sh build/simulator results/gate`.
1. `gmake LDFLAGS="-lz" -j8` (macOS: no -lstdc++fs).
1b. `git submodule update --init`, then `gmake -C GenSimpoint librefcpu.a LDFLAGS="-lz"` first on a fresh checkout (build artifact, not tracked).
2. `scripts/build_tests.sh . build/tests` plus `scripts/build_tests.sh . build/tests memory-stress-small smc-addi smc-div`
   (Homebrew LLVM, riscv32; the extras are not built by default) and `scripts/build_vm_tests.sh . build/vm` (4 paging tests).
3. `scripts/run_gate.sh build/simulator results/gate`: 61 tests, pass = cycles equal to baseline.csv.
4. Big programs (binaries built with clang + stub libc headers, kept outside the repo in /tmp/big):
   dhrystone 3,911,612; coremark 1/3/10 iterations 127,757 / 323,962 / 976,652; memory-stress 1,541,195;
   smc-div 8,996; smc-addi 8,866.
5. Small-LLC check (`scripts/llc_small_check.sh $PWD <abs path of baseline-sim/src>` (absolute paths required)): the default 8 MiB LLC never evicts in small tests.
   `load-store` shows "base= FAIL" because the original aborts with "Misaligned Load" (pre-existing).

## Verification harnesses
Frozen copies of the original modules live in `tests/*_ref`; the live modules were rewritten and proven against them.
- Lockstep shadows (build with `<MOD>_SHADOW=1`, `BUILD_DIR=build_<x>`): Lsu, Rob, Dispatch, Exu, Isu, Idu, Ren, Csr,
  PreIduQueue; `scripts/lsu_shadow.sh` is the template. A mismatch exits with code 2.
- Random differentials vs the frozen original: `bpu_diff`, `csr_diff`, `idu_diff`, `pre_diff`, `ddr_diff`;
  closed-loop `llc_diff` (`scripts/llc_diff.sh`) and `ic_diff` (`scripts/ic_diff.sh`) for the LLC and interconnect.
- Lesson: tests miss many trap/S-mode/FTQ-full paths, so pair a lockstep with a random-stimulus harness.
- Coverage-only pruning was rejected (it deleted the DDR write path once); prefer rewrite plus differential test.

## Status (2026-11-03)
- Size: ~84k lines originally -> ~43k; back-end modules, BPU, LLC, interconnect, front-end `front_comb_calc` and
  `rv_simu_mmu_v2.cpp` rewritten or tidied.
- Speed (single run, noisy machine): dhrystone 3.91M cycles in 16.5 s = ~237 k cycles/s (original ~186 k);
  coremark10 976k cycles in 7.0 s = ~140 k cycles/s.
- Speed levers done: ring-buffer FIFOs, front-end whole-struct copies removed, LLC comb() memo plus quiescence
  shortcut, Ren checkpoint dirty-row mask, hold-register live flags, IssueQueue grants_live.
- Not touched: MemSubSystem peripherals (PeripheralModel.h, MemPtwBlock.h; no test coverage), checkpoint restore
  (compiled only), two Ren paths no test reaches (commit + mispredict same cycle, exception with dest_en).
- Remaining small speed items, each < 3%: Exu::comb_fu2exu_collect, Dispatch::is_preg_woken, Ren::comb_rename,
  tb::Bridge::find, Exu::seq / SimCpu::cycle struct copies.
- Next: consolidate the idle/live flags into one small abstraction; interface freeze for RTL pairing (event + fact
  protocol; only docs so far); a larger gate (the tests are tiny).

## Pitfalls
- Timing is layout sensitive; one timing run per change is enough (compare with the known number), cycles decide.
- zsh does not word-split unquoted variables: use bash scripts.
- BSD sed has no `\|`; `git commit` needs an explicit identity in this environment.
- Long runs go in the background with `SIM_MAX_CYCLES` set.
