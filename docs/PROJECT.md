# AI-Huma / trace-model: project record

AI-Huma is the whole project (AI + human working together; also a nod to Rem's magic). It has three parts:
- `function-model` (`~/personal/AI-Huma/function-model`): the functional RISC-V core;
  built as `librefcpu.a`. Used here through the symlink `function-model/` -> `../function-model` (the sibling repository in the AI-Huma layout).
- `trace-model` (this repo): the project's timing model and exploration platform, fed with facts by the function model.
- `rtl` (`~/personal/AI-Huma/rtl`, moved there from `~/personal/tututugege-rtl` on 2026-11-04): the SystemVerilog
  implementation; its configuration package is generated from this repo's `include/config.h`, and on a merge to main the RTL must
  match this model's cycle counts program by program.
Project-wide rules (branches, merge procedure, records, TODO) live in the AI-Huma repository: `docs/WORKFLOW.md`, `docs/GUIDE.md`.
The sections below describe trace-model.

Two phases. (1) Base, done 2026-11-04: a trace-driven timing model distilled from tututugege/simulator (RTL-style,
execution-driven, cycle-accurate OoO RISC-V), cycle-identical to the original, about half the code and faster (plan and
step records: `docs/history/`). (2) Our own optimization, since then: architecture changes are explored here (exploration
records EXP-xxx in the AI-Huma repository, `docs/EXPLORATION.md`) and, when merged, carried to the RTL.
Correctness: difftest against the function model (facts, commit checks). Gate: cycle counts equal to `results/baseline/baseline.csv`.
The baseline started as the original model's numbers and is only changed by an intentional configuration change merged to main
(2026-11-05: batch-3 area configuration, EXP-005); every other change must keep identical cycle counts.

## Configuration knobs (include/config.h)
Sizes are `CFG_*` macros, overridable with `-D` (`gmake EXTRA_CXXFLAGS="-D..." BUILD_DIR=build_area/<name>`); the RTL package
generator (`rtl/scripts/gen_cfg.sh`) uses the same defaults and accepts the same overrides.
- widths and windows: `CFG_BACKEND_FETCH_WIDTH` (8), `CFG_MAX_BR_NUM` (32), `CFG_ROB_NUM` (128), `CFG_PRF_NUM` (128), `CFG_IBUF_SIZE` (64);
- issue queues: `CFG_INT_IQ_SIZE` 64, `CFG_LD_IQ_SIZE` 64, `CFG_STA_IQ_SIZE` 64, `CFG_STD_IQ_SIZE` 16, `CFG_BR_IQ_SIZE` 32 (also exported as `IQ_*_SIZE`);
- issue ports: `CFG_PLAIN_ALU_PORTS` 2 (after the four ALU+MUL/DIV/FP/CSR ports), `CFG_LD_PORTS` 4, `CFG_STA_PORTS` 3, `CFG_STD_PORTS` 3, `CFG_BR_PORTS` 2;
- front-end FIFOs (`front-end/config/frontend_feature_config.h`): `FRONT2BACK_FIFO_SIZE` 8, `PTAB_SIZE`, `INSTRUCTION_FIFO_SIZE`, `FETCH_ADDR_FIFO_SIZE` 16.

## Origin and history
- Forked from tututugege/simulator at 2f31e8d (`upstream` remote). `main` here is the former `trace-distill` branch.
- `axi-interconnect-kit/` (from dyh1807/axi-interconnect-kit, local commits up to 382cb6f) is vendored as a plain
  directory. `function-model/` is a symlink to `../function-model` (the function-model repo checked out next to this one; it was a git submodule until 2026-11-04, which pinned an older function-model).
- The pre-merge patch-queue repo (patches 0001..0061, step docs) is `../tututugege-trace`; its docs are in `docs/`,
  its tests/scripts in `tests/` and `scripts/`.
- The original model stays available as a baseline binary (`~/personal/llm4trace/third_party/tututugege/baseline-sim/src`, 2f31e8d + exact kit patches); the
  baseline cycle counts are in `results/baseline/baseline.csv`.

## Architecture in one paragraph
The model no longer computes values (ALU/MUL/DIV/FP, PRF, L1D data, LLC data table are gone). Branch direction/target,
addresses, store data and CSR write data come from facts supplied by the trace bridge (`diff/TraceBridge.*`), a
functional core (librefcpu, function-model) that follows the model's dispatch stream including wrong paths. Functional
memory is written at ROB commit from facts; instruction/PTE/LLC read data come from pmem.

## Gate (run after every change)
All scripts take the repo root as `<simulator-repo>`, e.g. `scripts/run_gate.sh build/simulator results/gate`.
1. `gmake LDFLAGS="-lz" -j8` (macOS: no -lstdc++fs). DRAMsim3 is on by default (EXP-006): it is the pinned git submodule `third_party/DRAMsim3` (`git submodule update --init third_party/DRAMsim3` after cloning), compiled by the Makefile into `build_dramsim3/libdramsim3.a` (`gmake dramsim3`; no cmake, no shared library). `gmake DS3=0` builds the old fixed-latency SimDDR. Core frequency: `gmake CPU_FREQ_MHZ=...` (default 1000) or the `CPU_FREQ_MHZ` environment variable at run time. After changing flags use a clean `BUILD_DIR`.
1b. `gmake -C function-model librefcpu.a LDFLAGS="-lz"` first on a fresh checkout (builds `../function-model`; build artifact, not tracked). Needs the function-model repository next to this one.
2. `scripts/build_tests.sh . build/tests` plus `scripts/build_tests.sh . build/tests memory-stress-small smc-addi smc-div`
   (Homebrew LLVM, riscv32; the extras are not built by default) and `scripts/build_vm_tests.sh . build/vm` (4 paging tests).
3. `scripts/run_gate.sh build/simulator results/gate`: 62 tests, pass = cycles equal to baseline.csv (regenerated 2026-11-06 for move elimination, EXP-011: 7 programs changed).
4. Big programs (binaries built with clang + stub libc headers, kept outside the repo in /tmp/big):
   dhrystone 64 runs 162,033; coremark 2 iterations 252,400 (shortened builds fixed as the standard from 2026-11-04, binaries and sources in AI-Huma/rtl/programs/big);
   memory-stress 2,425,220; smc-div 11,348; smc-addi 11,218; memory-stress-small 347,305.
   Numbers are for the EXP-006 baseline of 2026-11-05: no AXI subsystem divider (`FREQ_DIV` 1), DRAMsim3 DDR4-3200 behind SimDDR with 60 ns of fixed non-core latency (SOC 20 + CDC 12 + CTL 15 + PHY 13 from config.h; `DS3_EXTRA_NS` overrides), 1 GHz core.
   Before move elimination (EXP-011, `CFG_MOVE_ELIM` 1 from 2026-11-06): dhrystone 162,033; coremark 255,335; memory-stress 2,428,470; smc-div 11,348; smc-addi 11,218; memory-stress-small 347,305.
   Before (batch-3 configuration, divider 16, fixed 60-tick DDR latency, 500 MHz): 150,427 / 233,515 / 1,541,195 / 8,996 / 8,866 / 189,128. (First version of the new baseline, 32 ns extra latency: 158,756 / 250,658 / 2,425,029 / 11,096 / 10,966 / 343,975.)
5. Small-LLC check (`scripts/llc_small_check.sh $PWD <abs path of baseline-sim/src>` (absolute paths required)): the default 8 MiB LLC never evicts in small tests.
   `load-store` shows "base= FAIL" because the original aborts with "Misaligned Load" (pre-existing); `load-store-aligned` (tests/extra, aligned byte loads instead of the unaligned `lw`) replaces it in the gate.

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

## Move elimination (EXP-011; `CFG_MOVE_ELIM`, `CFG_MOVE_ELIM_REFS` in config.h)
- `addi rd, rs, 0` (rd, rs real registers) is renamed away in Ren: rd takes rs's physical register, no free-list slot, no uop,
  the ROB entry is complete at dispatch. A register is freed when no arch_RAT entry names it (`arch_cnt`, counted at commit,
  so no counter is ever rolled back on a flush). Recovery is unchanged: an eliminated move allocates nothing, so the free-list
  head checkpoint and `free_head_commit` (which skips eliminated moves) stay consistent.
- Bound on the counters (`Ren::elim_refs_ok`): a move is eliminated only if its source comes straight from the speculative RAT
  (no older instruction of the group writes it) and (spec RAT entries naming it) + (older instructions of the group with a
  destination) + 1 <= `CFG_MOVE_ELIM_REFS` (default 7 = 3-bit counters). The architectural state after a move commits equals the
  speculative RAT right after its rename, so the architectural count never exceeds the bound.
- `ME_CHECK=1` (environment, run time): every cycle, arch_cnt equals the number of arch_RAT entries naming each register and no
  physical register leaks; prints the maximum number of references seen.
- Not handled: `add rd, rs, x0` (the toolchain emits no compressed `c.mv`). A commit with a destination in the same cycle
  as a mispredict asserts in ME mode (the baseline free-list sanity check already treats it as unreachable; never hit in the gate,
  the big programs, Linux or SPEC quick).
