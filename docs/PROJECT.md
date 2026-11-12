# AI-Huma / trace-model: project record

AI-Huma is the whole project (AI + human working together; also a nod to Rem's magic). It has three parts:
- `function-model` (`~/personal/AI-Huma/function-model`): the functional RISC-V core (rv32imab, rv64imab since 2026-11-12; `XLEN` is a compile-time parameter);
  built as `librefcpu.a`. Used here through the symlink `function-model/` -> `../function-model` (the sibling repository in the AI-Huma layout).
- `trace-model` (this repo): the project's timing model and exploration platform, fed with facts by the function model.
- `rtl` (`~/personal/AI-Huma/rtl`): the SystemVerilog implementation; its configuration package is generated from this repo's
  `include/config.h` (`rtl/scripts/gen_cfg.sh`), and on a merge to main the RTL must match this model's cycle counts program by program.
Project-wide rules (branches, merge procedure, records, TODO) live in the AI-Huma repository: `docs/WORKFLOW.md`, `docs/GUIDE.md`;
the per-merge performance/verification table is `docs/MAIN_RECORDS.md` there. Entry point of this repo: [../README.md](../README.md);
all run-time environment variables: [ENV_VARS.md](ENV_VARS.md).

Two phases. (1) Base, done 2026-11-04: a trace-driven timing model distilled from tututugege/simulator (RTL-style,
execution-driven, cycle-accurate OoO RISC-V), cycle-identical to the original, about half the code and faster (plan and
step records: `docs/history/`). (2) Our own optimization, since then: architecture changes are explored here (exploration
records EXP-xxx in the AI-Huma repository, `docs/EXPLORATION.md`) and, when merged, carried to the RTL.
Correctness: difftest against the function model (facts, commit checks). Gate: cycle counts equal to `results/baseline/baseline.csv`,
which is only changed by an intentional configuration change merged to main (every other change must keep identical cycle counts).

## Architecture in one paragraph
The model no longer computes values (ALU/MUL/DIV/FP, PRF, L1D data, LLC data table are gone). Branch direction/target,
addresses, store data and CSR write data come from facts supplied by the trace bridge (`diff/TraceBridge.*`), a
functional core (librefcpu, function-model) that follows the model's dispatch stream including wrong paths. Functional
memory is written at ROB commit from facts; instruction/PTE/LLC read data come from pmem.

Current microarchitecture (defaults of `include/config.h`; the RTL package takes the same values):
- Front end (`front-end/`): 8-wide fetch, pipelined BPU (S0 uBTB, S1, S_D; TAGE/ITT/L1 predictors, update queue; `BPU_PIPE=3`, source comment in
  `front-end/BPU/BPU_pipe.h`, RTL description `rtl/docs/analysis/bpu-pipe-rtl.md`), fetch-address FIFO / PTAB / instruction FIFO of 16 entries, pre-IDU buffer,
  32 KiB 8-way I-cache (64 B lines), redirect fast paths `CFG_FE_RFAST` 15 (EXP-019/023; `CFG_FE_BFAST` 0).
- Back end (`back-end/`): 8-wide decode / rename / dispatch / commit, move elimination (below), `CFG_PRF_NUM` 128, `CFG_ROB_NUM` 128 as a compact ROB
  (`CFG_ROB_COMPACT` 1, EXP-028), `CFG_MAX_BR_NUM` 32, issue queues INT 64 / LD 64 / STA 64 / STD 16 / BR 32 with enqueue widths 7/6/4/4/5 (EXP-017).
- Issue ports: 6 integer ALU ports (4 of them with MUL/DIV/FP/CSR capabilities, `CFG_PLAIN_ALU_PORTS` 2 plain), `CFG_BR_PORTS` 2, `CFG_LD_PORTS` 4, `CFG_STA_PORTS` 3, `CFG_STD_PORTS` 3.
  Multiplier 2 cycles (3 for XLEN=64), divider 16 (32).
- LSU (`back-end/Lsu/RealLsuPipe.cpp`, [LSU_PIPE.md](LSU_PIPE.md)): staged load/store pipeline (`CFG_LSU_PIPE` 1; `LSU_PIPE=0` selects the legacy LSU `RealLsu.cpp`, kept only
  for the regression baselines), LDQ/STQ 64 entries, memory-dependence speculation (`CFG_LSU_MEMSPEC` 1) with the value path `CFG_LSU_MS_DATA` 1 (wait-table predictor
  `CFG_LSU_MDP_BITS` 6), 4 replay registers, committed-store drain through a 4-entry write buffer (`CFG_LSU_PIPE_WBUF` 4, EXP-024).
- Memory system (`MemSubSystem/`, `axi-interconnect-kit/`): 64 KiB 4-way L1D (64 B lines, 8 MSHRs) with the L1D prefetcher h1 (`PF_ENABLE` 1), shared LLC
  `CONFIG_AXI_LLC_SIZE_BYTES` 4 MiB 16-way, 8 MSHRs, lookup latency 3, DRAMsim3 DDR4-3200 (EXP-006) behind SimDDR plus 60 ns of fixed non-core latency, 1 GHz core,
  no AXI subsystem divider (`CONFIG_AXI_SUBSYSTEM_FREQ_DIV` 1). Cacheable: 0x40000000-0x4fffffff and 0x80000000-0xffffffff, everything else is MMIO.
- PMU / TMA (`back-end/Pmu*.cpp`, `PMU=1`, EXP-013; the AI-Huma repository's `docs/PMU.md`).

## Configuration knobs (include/config.h)
Sizes are `CFG_*` macros, overridable with `-D` (`gmake EXTRA_CXXFLAGS="-D..." BUILD_DIR=build_area/<name>`); the RTL package
generator (`rtl/scripts/gen_cfg.sh`) uses the same defaults and accepts the same overrides. Most have a run-time environment
override, see [ENV_VARS.md](ENV_VARS.md). `XLEN` (32 default, 64: `gmake XLEN=64`) is set by the Makefile and exported as `XLEN_BITS`.
- widths and windows: `CFG_BACKEND_FETCH_WIDTH` (8), `CFG_MAX_BR_NUM` (32), `CFG_ROB_NUM` (128), `CFG_PRF_NUM` (128), `CFG_IBUF_SIZE` (64);
- ROB organisation: `CFG_ROB_COMPACT` (1; compile-time only) - 0: one dispatch group per 8-entry ROB line, a whole line commits; 1: compact ROB (EXP-028): sequential
  allocation over the same 8 banks x 16 lines, the longest completed prefix from the head commits (up to 8), cut by registered BPU commit credits
  (BPU update-queue / uBTB train-queue occupancy minus last cycle's commits); flushes, fences, WFI and interrupts commit alone at the head;
- issue queues: `CFG_INT_IQ_SIZE` 64, `CFG_LD_IQ_SIZE` 64, `CFG_STA_IQ_SIZE` 64, `CFG_STD_IQ_SIZE` 16, `CFG_BR_IQ_SIZE` 32 (also exported as `IQ_*_SIZE`);
  enqueue widths `CFG_INT_IQ_ENQ_WIDTH` 7, `CFG_LD_IQ_ENQ_WIDTH` 6, `CFG_STA_IQ_ENQ_WIDTH` 4, `CFG_STD_IQ_ENQ_WIDTH` 4, `CFG_BR_IQ_ENQ_WIDTH` 5;
- issue ports: `CFG_PLAIN_ALU_PORTS` 2 (after the four ALU+MUL/DIV/FP/CSR ports), `CFG_LD_PORTS` 4, `CFG_STA_PORTS` 3, `CFG_STD_PORTS` 3, `CFG_BR_PORTS` 2;
- queues: `CFG_LDQ_SIZE` 64, `CONFIG_STQ_SIZE` 64; D-cache `CFG_DCACHE_MSHR` 8;
- move elimination: `CFG_MOVE_ELIM` 1, `CFG_MOVE_ELIM_REFS` 7 (below);
- LSU: `CFG_LSU_PIPE` 1, `CFG_LSU_PIPE_CUT` 127, `CFG_LSU_PIPE_L2` 115, `CFG_LSU_MEMSPEC` 1, `CFG_LSU_MS_DATA` 1, `CFG_LSU_MDP_BITS` 6, `CFG_LSU_MDP_CLR` 2000,
  `CFG_LSU_MDP_MODE` 0, `CFG_LSU_MDP_HW` 2, `CFG_LSU_MS_SLOTS` 0, `CFG_LSU_PIPE_WBUF` 4, `_WBMAX` 2, `_WBINS` 4, `_WBSCAN` 4, `_DRAIN` 2, `_DCAND` 4, `_RP` 4;
- front end: `CFG_FE_RFAST` 15, `CFG_FE_BFAST` 0; FIFOs in `front-end/config/frontend_feature_config.h`: `FRONT2BACK_FIFO_SIZE`, `PTAB_SIZE` 16, `INSTRUCTION_FIFO_SIZE` 16, `FETCH_ADDR_FIFO_SIZE` 16;
- memory: `CONFIG_CPU_FREQ_MHZ` 1000 (`gmake CPU_FREQ_MHZ=...`), `CONFIG_AXI_LLC_*`, `CONFIG_SIM_DDR_LATENCY` 60 (fixed non-core latency in ns: SOC 20 + CDC 12 + CTL 15 + PHY 13; `DS3_EXTRA_NS` overrides at run time).

## Origin and history
- Forked from tututugege/simulator at 2f31e8d (`baseline.lock`). `main` here is the former `trace-distill` branch.
- `axi-interconnect-kit/` (from dyh1807/axi-interconnect-kit, local commits up to 382cb6f) is vendored as a plain
  directory. `function-model/` is a symlink to `../function-model` (the function-model repo checked out next to this one; it was a git submodule until 2026-11-04).
- The pre-merge patch-queue repo (patches 0001..0061, step docs) is `../tututugege-trace` (if present); its docs are in `docs/history/`,
  its tests/scripts in `tests/` and `scripts/`.
- The original model stays available as a baseline binary (`~/personal/llm4trace/third_party/tututugege/baseline-sim/src`, 2f31e8d + exact kit patches); the
  original's cycle counts of 2026-11-04 are no longer the target (the baseline was regenerated at each configuration change merged to main, see `docs/MAIN_RECORDS.md`).

## rv64 (EXP-029, merged 2026-11-12)
`XLEN` is a compile-time parameter of function-model, trace-model and the RTL package (`XLEN_BITS`, default 32; the default behaviour and all cycle counts are unchanged).
`XLEN=64` covers rv64imab: register-wide PCs and addresses, Sv39 page walk / TLB, 64-bit CSRs, 8-byte accesses, OP-IMM-32 / OP-32, multiplier 3 and divider 32 cycles (`*W` divides 16);
`addiw rd, rs, 0` is not a move (no elimination). Build: see [../README.md](../README.md). Verified through the RTL gate (cpu-tests, Linux to `rv64 login:`; `docs/MAIN_RECORDS.md`, 2026-11-12);
there is no rv64 variant of `scripts/run_gate.sh` / `baseline.csv` here yet. Not done: C, the SPEC rv64 flow, Sv39 in the FastWarm replay.

## Gate (run after every change)
All scripts take the repo root as `<simulator-repo>`, e.g. `scripts/run_gate.sh build/simulator results/gate`.
1. `gmake LDFLAGS="-lz" -j4` (macOS: no -lstdc++fs, hence the `LDFLAGS` override). DRAMsim3 is on by default (EXP-006): it is the pinned git submodule `third_party/DRAMsim3` (`git submodule update --init third_party/DRAMsim3` after cloning), compiled by the Makefile into `build_dramsim3/libdramsim3.a` (`gmake dramsim3`; no cmake, no shared library). `gmake DS3=0` builds the old fixed-latency SimDDR. Core frequency: `gmake CPU_FREQ_MHZ=...` (default 1000) or the `CPU_FREQ_MHZ` environment variable at run time. After changing flags use a clean `BUILD_DIR`.
1b. `gmake -C function-model librefcpu.a LDFLAGS="-lz"` first on a fresh checkout (builds `../function-model`; build artifact, not tracked). Needs the function-model repository next to this one.
2. `scripts/build_tests.sh . build/tests` plus `scripts/build_tests.sh . build/tests memory-stress-small smc-addi smc-div pf-stress`
   (Homebrew LLVM, riscv32; the extras are not built by default) and `scripts/build_vm_tests.sh . build/vm` (4 paging tests).
3. `scripts/run_gate.sh build/simulator results/gate`: 64 tests (the cpu-tests, the 4 paging tests and the extras of `tests/extra`; `memory-stress`, `load-store` and `smc-*` are excluded by the script, see its `grep -v`; `baseline.csv` also lists them), pass = cycles equal to `results/baseline/baseline.csv`
   (regenerated 2026-11-11 with the compact ROB (EXP-028) as the default, before that 2026-11-09 with the write buffer (EXP-024); `baseline_nopf.csv` is the prefetcher-off variant of the same baseline: `PF_ENABLE=0` reproduces it, it may be behind main).
   All end with difftest Success, 0 TB_CHECK mismatches. `LSU_PIPE_NOWD=1` is not needed for the gate (the baseline is the model's own cycle count); it is needed for any cycle comparison with the RTL.
   **Old-design regression** (old LSU + old BPU, the main of 2026-11-06): `LSU_PIPE=0 BPU_PIPE=0 BPO_HIST=0 BPO_HSYM=2 BPA_REPAIR=0 BPA_ITTH= BPO_PDCKPT=0 BPO_L1_ENTRIES=0 BPO_BANKBLK=0 BPU_ICALL=0 BASELINE=results/baseline/legacy_lsu_baseline.csv scripts/run_gate.sh ...`
   reproduced `legacy_lsu_baseline.csv` (prefetcher off: `legacy_lsu_baseline_nopf.csv`) in the 2026-11-07 main; since EXP-028 (target cache, statistical corrector and TAGE parameter sweeps removed) and EXP-027 (uncached windows removed) this string no longer reproduces those baselines exactly. Use it for difftest only (`scripts/run_gate_on.sh <sim> <out> . [jobs]` checks Success only).
4. Big programs (sources and binaries in `AI-Huma/rtl/programs/big`; clang + stub libc headers; shortened builds fixed as the standard from 2026-11-04). **Current defaults (2026-11-12, compact ROB EXP-028 on; `docs/MAIN_RECORDS.md`):**
   dhrystone 64 runs 116,774; coremark 2 iterations 284,902; memory-stress 2,308,184; smc-div 10,687; smc-addi 10,176; memory-stress-small 325,458; pf-stress 621,069; itt-dispatch 222,439.
   Line ROB (`-DCFG_ROB_COMPACT=0`): dhrystone 114,685; coremark 283,240; memory-stress 2,358,268; smc-div 10,875; smc-addi 10,315; memory-stress-small 329,424; pf-stress 630,916; itt-dispatch 222,445
   (the baseline.csv before EXP-028; `software-interrupt-vectored` 2254 -> 2356 from the removed uncached windows).
   Older configurations are listed in `docs/MAIN_RECORDS.md` (one row per merge); their switches (`LSU_PIPE_WBUF=0 LSU_PIPE_DRAIN=3` = before EXP-024, `LSU_MEMSPEC=0 FE_RFAST=0` = before EXP-019, `LSU_PIPE=0` = legacy LSU) still work but their numbers are no longer reproduced exactly.
   The cycle numbers assume the EXP-006 memory system: DRAMsim3 DDR4-3200 plus 60 ns of fixed non-core latency, 1 GHz core.
5. Small-LLC check (`scripts/llc_small_check.sh $PWD <abs path of baseline-sim/src>` (absolute paths required)): the default 4 MiB LLC never evicts in small tests, so the script builds both simulators with a 64 KiB LLC and compares cycles on programs that do evict.
   (It compares against the original model, so it only agrees while the trace-model runs the configuration the original had; use it as a harness for the DDR write / victim write-back path, not as a gate.)
   `load-store` shows "base= FAIL" because the original aborts with "Misaligned Load" (pre-existing); `load-store-aligned` (tests/extra, aligned byte loads instead of the unaligned `lw`) replaces it in the gate.

## Verification harnesses
Frozen copies of the original modules live in `tests/*_ref`; the live modules were rewritten and proven against them.
- Lockstep shadows (build with `<MOD>_SHADOW=1`, `BUILD_DIR=build_<x>`): Lsu, Rob, Dispatch, Exu, Isu, Idu, Ren, Csr,
  PreIduQueue; `scripts/lsu_shadow.sh` is the template. A mismatch exits with code 2. These compare against the original (legacy) modules; the staged LSU and the compact ROB have no frozen reference.
- Random differentials vs the frozen original: `bpu_diff`, `csr_diff`, `idu_diff`, `pre_diff`, `ddr_diff`;
  closed-loop `llc_diff` (`scripts/llc_diff.sh`) and `ic_diff` (`scripts/ic_diff.sh`) for the LLC and interconnect.
- Against the function model and the RTL: `TB_CHECK=1` (value comparison, `scripts/check_run.sh`), `TB_TRACE_COMMIT=1` with the RTL's `SOC_TB_COMMIT`, `LSU_PIPE_EVT=1` with `SOC_TB_LSUEVT` (see [ENV_VARS.md](ENV_VARS.md)).
- Lesson: tests miss many trap/S-mode/FTQ-full paths, so pair a lockstep with a random-stimulus harness.
- Coverage-only pruning was rejected (it deleted the DDR write path once); prefer rewrite plus differential test.

## Status
The 2026-11-03 distillation numbers (84k -> 43k lines, speed levers) are in `docs/history/`. Current state: see `docs/MAIN_RECORDS.md` (latest rows) and the TODO / EXPLORATION in the AI-Huma repository.
Not touched since the distillation: MemSubSystem peripherals (PeripheralModel.h, MemPtwBlock.h; no test coverage), checkpoint restore of the old format.

## Pitfalls
- Timing is layout sensitive; one timing run per change is enough (compare with the known number), cycles decide.
- zsh does not word-split unquoted variables: use bash scripts.
- BSD sed has no `\|`; `git commit` needs an explicit identity in this environment.
- Long runs go in the background with `SIM_MAX_CYCLES` set.
- Cycle comparison against the RTL needs `LSU_PIPE_NOWD=1`.
- Local simulation batches: at most 4 parallel jobs (each simulator holds up to ~3 GB RSS).

## Move elimination (EXP-012; `CFG_MOVE_ELIM`, `CFG_MOVE_ELIM_REFS` in config.h)
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
- rv64: `addiw rd, rs, 0` is a sign-extension, not a move, and is not eliminated.
- Not handled: `add rd, rs, x0` (the toolchain emits no compressed `c.mv`). A commit with a destination in the same cycle
  as a mispredict asserts in ME mode (the baseline free-list sanity check already treats it as unreachable; never hit in the gate,
  the big programs, Linux or SPEC quick).

## F/D floating point (EXP-030; default on, rv32 and rv64; Zfinx removed)
F/D is the default build (`FP=1`; the function-model's `librefcpu.a` must be built with the same `FP`: `RefCpuState` carries `fpr[32]` and `fcsr`). `FP=0` is a core without any floating point (no Zfinx either: the old `UOP_FP` / `FuKind::FPU` path and the tests `zfinx*`, `matmul_fpu` are gone; `tests/extra/fd-*` are their F/D versions). Integer code has identical cycles with and without F/D.
rv32: FLEN is 64 too; `fld` / `fsd` are 8-byte accesses (the LSU width helpers return 8 for funct3 3 with F/D), `fcvt.l/lu` and `fmv.x.d/fmv.d.x` do not exist (the function model traps them, so the model does too), `mstatus.SD` is bit 31.
- **Registers and tags.** Separate FP physical register file (`CFG_FPRF_NUM` 96, 32 architectural), FP RAT / free list / per-branch RAT checkpoints in `Ren`. In the
  model the physical-register tags are one number space (integer `0..PRF_NUM-1`, FP `FP_TAG_BASE..`; `TAG_NUM`, `PRF_IDX_WIDTH` follow), so issue-queue entries, the
  dispatch busy table, the wake-up buses and the memory-speculation taint table are unchanged; the RTL has two register files, a domain bit and per-domain wake buses.
  `UopS` has `dest_fp` / `src{1,2,3}_fp`, `src3_en/areg/preg` (fused multiply-add), `fp_cls`. `InstType::FP` was renamed `FPOP` (the name is the build macro now).
- **Issue.** New `IQ_FP` (`CFG_FP_IQ_SIZE` 32, enqueue width 4) with a third wake-up matrix; three FP issue ports (FMA+MISC, FMA+MISC, FDIV+MISC; each unit is its own FU
  so a long divide does not block the others) and `I2F` on integer ports 2 and 3. Units: `UOP_FMA` (fadd/fsub/fmul/fmadd family, pipelined), `UOP_FMISC` (fsgnj,
  fmin/fmax, fcmp, fclass, fcvt inside FP, FP -> integer moves/conversions, pipelined), `UOP_FDIV` (fdiv/fsqrt, iterative, latency by S/D), `UOP_I2F` (integer -> FP).
  `CFG_FP_*_LAT` are **dependent-issue intervals** (4 = a dependent fadd.d issues 4 cycles after the previous one; the delayed wake-up costs one cycle on top of the unit
  latency like MUL/DIV, so the model latency is CFG - 1): FMA 4, MISC 3, I2F 3, fdiv S/D 12/20, fsqrt S/D 14/24 (starting values, not calibrated).
- **Memory.** `flw/fld` are loads (LD queue, FP destination tag), `fsw/fsd` are STA (integer base) + STD whose data source is an FP tag (all STD ports can take it for now;
  the plan was port 0 only, open for the RTL). Nothing in the LSU changed.
- **Traps and status.** What traps (mstatus.FS = Off, reserved rounding mode ...) is the function model's verdict: an executed FP-class uop whose fact says "trapped, cause 2"
  becomes the illegal-instruction exception (`Fu::fp_trap`); statically reserved encodings trap in decode. mstatus.FS = Dirty / SD and misa F/D are modelled in `Csr`
  (a committed instruction whose fact says it turned FS to Dirty sets it at the end of the cycle). `csrr`/`csrw` of fflags / frm / fcsr are still NOPs with no
  serialization (the model computes no values, so a dynamic rounding mode has no effect; fcsr writes dirty FS through the same fact).
- **Interface.** `Fact` has `rd_is_fp`, `fs_dirty`, `trap_cause`; the bridge decodes opcodes 0x07 / 0x27 (address, size, store data from `fpr`), undoes FP register and fcsr writes,
  `FastWarm` replays FP loads/stores, checkpoint format 5 (`CPU_state.fpr`, fcsr after it) is read by `BackTopLoad.cpp`.
- **Checks.** `scripts/fp_check.sh <sim> <workdir>`: the function model's directed FP tests, trap/status test, compiler-generated kernels and the 23 official riscv-tests
  `rv64uf`/`rv64ud` all run to the end (difftest Success, every instruction committed); `tests/fp/fpmicro.c` micro-benchmarks show the configured latencies (4 dependent fadd.d: 16 cycles,
  8 independent: 4 cycles = both FMA pipes, dependent fdiv.d 20, fcvt.l.d + addi + fcvt.d.l 7). Small `CFG_FPRF_NUM=36`, `CFG_FP_IQ_SIZE=8`, `CFG_FP_FMA_LAT=8` runs without deadlock
  (the FP free-list invariant is asserted every cycle). `TB_TRACE_PATCH=1` prints every CSR read the model patches into the function model (0 for the FP tests).
- **Not done.** Serialized fcsr/frm accesses, FP-domain move elimination, FS-off trap coverage beyond the directed test, RTL, calibrated latencies, an FP workload (PolyBench / SPECfp) for the knobs, `MsData` taint of FP consumers (not modelled).

### FP register-file port study (EXP-032, branch `exp/fp-wb`)
`FP_WB_PORTS=<n>` (env, 0 = unlimited, default): at most n FP-register writes per cycle from the FP units (FMA, MISC with an FP destination, FDIV, I2F). The write cycle is known at issue
(`issue + latency`), so `Isu::comb_issue` reserves it in a busy table (`back-end/include/FpStats.h`, like XiangShan's WbFuBusyTable; an `IssueQueue::grant_gate` hook) and a uop that finds the cycle full stays in the
issue queue (reservations of squashed uops are not released, a small pessimism). FP loads write through the LSU and are not limited, only counted. Exit statistics `[FP_PORTS]`: FU writes per cycle, FP load write-backs per cycle,
FP reads of the FP queue's issued uops per cycle, FP store-data reads per cycle. `scripts/fp_bench.sh <sim> <work> [ports...]` builds the six kernels of `tests/fp/fpbench.c` and sweeps the limit.

