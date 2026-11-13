# trace-model

The timing model and architecture-exploration platform of the AI-Huma project (function-model + trace-model + rtl).
It is a cycle-accurate, RTL-style out-of-order RISC-V core model (8-wide front end / decode / commit, 128-entry ROB, staged LSU,
pipelined BPU, L1I/L1D, shared LLC, DRAMsim3 DDR). Ideas are measured here first (cycles, IPC, queue/port usage, TMA through the PMU)
and then carried to the RTL; on a merge to `main` the RTL must reproduce this model's cycle counts program by program.

The model does not compute values. Branch direction/target, addresses, store data and CSR write data come from facts supplied by the
trace bridge (`diff/TraceBridge.*`), a functional core (the sibling `function-model` repository, built as `librefcpu`;
`function-model/` here is a symlink to `../function-model`) that follows the model's dispatch stream including wrong paths.
Every change must keep the gate's cycle counts (`results/baseline/baseline.csv`) unless it is an intentional configuration change
merged to main. Project-wide rules (branches, merge procedure, `MAIN_RECORDS.md`, `EXPLORATION.md`, TODO) live in the AI-Huma
repository that contains the three parts (`docs/WORKFLOW.md`, `docs/GUIDE.md`).

ISA: rv32imafd by default (F/D since 2026-11-12, EXP-030; Zfinx was removed); rv64imafd with `-DXLEN=64` (EXP-029); `FP=0` builds a core without floating point. Not done: C, the SPEC rv64 flow,
Sv39 in the FastWarm replay.

## Documents

| document | content |
|---|---|
| [docs/PROJECT.md](docs/PROJECT.md) | architecture, configuration knobs (`include/config.h`), the gate, verification harnesses, pitfalls |
| [docs/ENV_VARS.md](docs/ENV_VARS.md) | all run-time environment variables with defaults (**use `LSU_PIPE_NOWD=1` when comparing cycles with the RTL**) |
| [docs/LSU_PIPE.md](docs/LSU_PIPE.md), [docs/LSU_PIPE_IPC.md](docs/LSU_PIPE_IPC.md) | staged LSU: specification, approximations, results, IPC study |
| [FRONTEND_NOTES.md](FRONTEND_NOTES.md), [INTEG_NOTES.md](INTEG_NOTES.md), [MS_DATA_NOTES.md](MS_DATA_NOTES.md) | records of the front-end fast paths, the memspec + front-end integration and the memspec value path (historical, defaults since merged) |
| [docs/history/](docs/history/) | the 2026-11-04 distillation (P0-P3); historical, may be outdated |
| [front-end/](front-end/), [axi-interconnect-kit/](axi-interconnect-kit/) | front-end coding standards, the vendored AXI/LLC/DDR kit |

## Build

Needs a C++20 compiler, zlib, and the `function-model` repository next to this one (`function-model` symlink). DRAMsim3 is the git
submodule `third_party/DRAMsim3` (`git submodule update --init third_party/DRAMsim3`); the Makefile builds it into
`build_dramsim3/libdramsim3.a` (`gmake DS3=0` uses the old fixed-latency SimDDR instead).

rv32 (default):

```bash
gmake -C function-model librefcpu.a LDFLAGS="-lz"     # builds ../function-model (build artifact, not tracked)
gmake -j4 LDFLAGS="-lz"                               # build/simulator; macOS: -lstdc++fs does not exist, so override LDFLAGS
```

rv64: the register width is a compile-time parameter of both repositories, object files are not tagged with it, so build the rv64
`librefcpu.a` in a separate copy or after a clean of the function-model checkout, and use a separate build directory here:

```bash
gmake -C <function-model copy> XLEN=64 librefcpu.a
gmake -j4 XLEN=64 BUILD_DIR=build64 REFCPU_LIB=<function-model copy>/librefcpu.a LDFLAGS="-lz"
```

After changing compile flags use a clean `BUILD_DIR`. Configuration macros (`CFG_*`, see `include/config.h`) are overridden with
`gmake EXTRA_CXXFLAGS="-DCFG_...=..." BUILD_DIR=build_x`. Core frequency: `gmake CPU_FREQ_MHZ=...` or the `CPU_FREQ_MHZ` environment variable.

## Run

```bash
./build/simulator path/to/program.bin                  # raw binary loaded at 0x80000000, prints sim-time(cycle)= ... at the end
PMU=1 ./build/simulator prog.bin                       # TMA / event statistics, writes pmu_*.csv in the current directory
LSU_PIPE_NOWD=1 ./build/simulator prog.bin             # cycles comparable with the RTL
SIM_MAX_CYCLES=1000000 ./build/simulator prog.bin      # stop (exit code 3) after N cycles; use it for anything that may hang
./build/simulator --mode ckpt checkpoint/mcf/ckpt_sp1.gz   # SimPoint checkpoint (also --mode fast / ref; ./build/simulator -h)
```

All variables: [docs/ENV_VARS.md](docs/ENV_VARS.md).

## Gate (run after every change)

```bash
scripts/build_tests.sh . build/tests                   # Homebrew LLVM, riscv32; extras: ... build/tests memory-stress-small smc-addi smc-div pf-stress
scripts/build_vm_tests.sh . build/vm                   # 4 paging tests
bash scripts/run_gate.sh build/simulator results/gate  # parallel; pass = cycles equal to results/baseline/baseline.csv
```

`scripts/run_gate.sh` takes the simulator, an output directory and an optional job count (default 8; use at most 4 locally) and
exits non-zero on any failure; `BASELINE=<csv>` selects another baseline. `scripts/run_gate_on.sh <sim> <out> . [jobs]` checks difftest Success only (for configurations that change timing), `scripts/check_run.sh`
runs programs with `TB_CHECK=1`, `scripts/gate.sh` runs a list of binaries into a csv, `scripts/compare.sh` compares two result
directories. Details (old-design regression switches, big-program cycle counts, small-LLC check, lockstep shadows and differential tests):
[docs/PROJECT.md](docs/PROJECT.md).

### C extension (`RVC=1`, EXP-038)

`RVC=1` makes the front end fetch 2-byte positions (16 slots per 32-byte block) and accept 16-bit compressed instructions; `RVC=0` (default) is unchanged
(gate 64/64). It needs a `librefcpu.a` built from `function-model` with the same `XLEN`, `FP` and `RVC` (`make XLEN=32 FP=1 RVC=1 librefcpu.a`).

```bash
make BUILD_DIR=build_rvc XLEN=32 RVC=1 REFCPU_LIB=<librefcpu built with RVC=1>
scripts/build_rvc_tests.sh                                           # build_c/tests (C-compiled cpu-tests), build_c/gen (function-model's generated C stress tests)
bash scripts/run_gate_c.sh build_rvc/simulator results/gate_c        # cycles equal to results/baseline/baseline_c.csv; FULL=1 adds the long generated tests
```

Design (XiangShan-style, see `docs/EXPLORATION.md` EXP-038 in the top-level repository): per-slot predecode with RVC expansion, instruction starts from two
chains (`start_a` / `start_b`) selected by the "last half" carry of the previous group, valid-slot masks with gaps, `is_rvc` per instruction to the back end.
`RVC_DBG=1` / `BPU_OVR_DBG=1` print predecode-flush / uBTB-override diagnostics.

## Relation to the other repositories

- `function-model`: the functional core `librefcpu` (rv32/rv64, XLEN compile-time), the source of the facts.
- `rtl`: the SystemVerilog implementation. Its configuration package is generated from `include/config.h` (`rtl/scripts/gen_cfg.sh`
  with the same defaults and overrides); the RTL `fm_gate.sh` compares its cycles with the model's (with `LSU_PIPE_NOWD=1`).
- AI-Huma (parent repository): `docs/MAIN_RECORDS.md` (performance and verification per merge to main), `docs/EXPLORATION.md` (EXP-xxx).

## Origin

Forked from tututugege/simulator at 2f31e8d (`baseline.lock`), distilled into a trace-driven model that was cycle-identical to the
original on 2026-11-04 (`docs/history/`); since then the model is our own design. The upstream README, the design documents of the
original (execution-driven) modules, the upstream CI and review workflows, the SPEC scoring scripts and the vendored riscv-tests
copy were removed on 2026-11-04; they remain in the git history.
