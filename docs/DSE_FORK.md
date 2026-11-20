# Fast back-end DSE: run-time caps + fork after warm-up (EXP-047)

Branch `exp/dse-fork` (based on `exp/wb-llc` 8fd0ae3).

## What it does

A DSE run used to compile one binary per configuration (`-DCFG_*`) and repeat the whole detailed warm-up for each one.
Now one binary covers every size up to its compiled sizes, and one warm-up serves every configuration:

1. **Run-time caps** (`include/DseCap.h`, `g_dse`): ROB, integer / FP physical registers, branch tags, LDQ, STQ, L1D
   MSHR, and the INT / MEM / STD / BR / FP issue queues. A cap only changes the "full" / "free entries" check of
   allocation; the arrays, ring indices and bit widths keep the compiled size. Default = compiled size, so without
   `DSE_CONFIGS` nothing changes.
2. **Fork** (`driver/DseFork.cpp`): with `DSE_CONFIGS` set, the parent runs the warm-up up to
   `warm-up target - DSE_SETTLE` committed instructions **with the caps of the first configuration** (the baseline), then
   forks one child per configuration. Each child sets its caps,
   runs the remaining `DSE_SETTLE` warm-up instructions (the queues drain down to the new caps) and then the normal
   measurement. The parent waits for the children and exits.

| Variable | Meaning |
|---|---|
| `DSE_CONFIGS=<file>` | one configuration per line: `<name> key=value ...`; keys `rob prf fprf br ldq stq mshr iq_int iq_mem iq_std iq_br iq_fp`; a missing key = the binary's compiled size |
| `DSE_OUT=<dir>` | each child runs in `<dir>/<name>/` (its relative output files such as `PMU_OUT` land there), log `<dir>/<name>/sim.log` |
| `DSE_SETTLE=<n>` | instructions between the fork and the end of the warm-up (default 200000) |
| `DSE_JOBS=<n>` | children running at the same time (default: all) |

A cap above the compiled size is an error; for larger sizes, build with larger `CFG_*` (the `sim64_dsebig` binary:
ROB 256, PRF 256, FPRF 192, branch tags 64, MSHR 16, INT IQ 64, MEM IQ 96, STD IQ 32, BR IQ 48, FP IQ 64; LDQ / STQ stay 64,
the LSU limit).

## Checks (2026-11-19)

- Unmodified `exp/wb-llc` HEAD and this branch without `DSE_CONFIGS`: dhrystone checkpoint 148937 cycles in both.
- A cap is the same as compiling that size (cycle-identical):
  - the big binary capped down to the default sizes = the default binary (dhrystone 148937 = 148937);
  - the big binary capped to ROB 64 / PRF 80 / LDQ 16 / STQ 16 / MSHR 4 / INT IQ 16 / MEM IQ 24 / branch tags 12 = a binary
    compiled with those sizes (dhrystone 138772 = 138772; Linux checkpoint 1.5M instructions: 1218577 = 1218577).
- A fork child without caps = a plain run of the same binary (dhrystone 160885 = 160885).
- A fork in the middle of a run: the baseline child = a plain run of the default binary (Linux checkpoint, fork at
  700k, 1196979 = 1196979). The first version warmed up uncapped (with the big binary's maxima) and was 0.019 % off on
  mcf sp44 (35501885 vs 35495239): the caches / predictors had been warmed by a different core; hence the warm-up now runs
  with the first configuration.
- Static binaries for eda must not be built with `-march=native` on iprc-jump (AMD Zen 5): they die with SIGILL on
  eda-04 (Intel Cascade Lake). Build trace-model and librefcpu with `-march=x86-64-v3`.
- SPEC (eda): see EXPLORATION.md EXP-047.

## Limits

- The L1D MSHR cap limits the allocation only; the LLC MSHR (`CONFIG_AXI_LLC_MSHR_NUM`) has no cap: shrinking it under
  in-flight requests would hide them from its scan loops.
- Issue/dispatch/commit widths, port counts and latencies are not capped (they need recompilation).
- Each child is a full process (about 1.8 GB RSS on an rv64 SPEC point, before copy-on-write sharing is counted).

## Scripts (`scripts/dse/`)

- `fork_point.sh <run> <bench> <ckpt>` (eda): one SimPoint, all configurations of `tm/dse/<run>/cfg.txt`.
- `fork_cfgs.py`: the one-parameter-at-a-time sweep around the default as a `DSE_CONFIGS` file.
- `fork_report.py <run dir> <jobs> [--bench]`: weighted CPI per benchmark, IPC geomean change against `base`.

## EXP-048 prefetch switches on this branch (all default off: without them the cycles are unchanged)

Full record: AI-Huma `docs/EXPLORATION.md` EXP-048. Full SPEC06 int (386 points, 10M + 10M): Berti in virtual addresses with
DTLB-port translation + LLC BOP + L1D / LLC MSHR 16 = **+6.53 %** against main's default (h1, MSHR 8 / 8).

| Variable | Meaning |
|---|---|
| `PF_KIND=1` | L1D Best-Offset prefetcher in place of h1 (`MemSubSystem/include/Bop.h`, parameters `PF_BOP_*`) |
| `PF_KIND=2` | L1D Berti in place of h1 (ported from EXP-043; history 8 sets x `PF_BERTI_HW` (64) ways, delta table 16 PCs x 16 deltas, `PF_BERTI_ALLOC` (1)) |
| `PF_KIND=3` | h1 and Berti together (one FIFO push per cycle: h1 burst > Berti > h1 LLC next-line) |
| `LLC_BOP=1` | Best-Offset prefetcher in the LLC (`LLC_BOP_RES` MSHRs kept for demand, `LLC_BOP_INF` prefetches in flight) |
| `PF_BERTI_VA=1` | Berti trains / predicts on virtual lines (the load's vaddr travels in `LoadReq.vaddr`); targets may leave the page |
| `PF_BERTI_XPORT=1` | cross-page targets are translated on real DTLB ports: queued, sent on load ports nobody uses that cycle; a miss starts the walk (translation prefetch); faulting prefetch walks are dropped (`TlbMmu::set_pf_page`) |
| `PF_BERTI_WALK=1` | (superseded by XPORT) free side-effect-free lookup (`PF_BERTI_XLATE` per cycle) + walk on a miss |
| `PF_BERTI_RAMP=n`, `PF_BERTI_FAR`, `PF_BERTI_APPEND`, `PF_BERTI_RFILT`, `PF_BERTI_LATEFIX` | experiments of the Berti diagnosis (see EXP-048; none kept) |
| `PF_STATS=1` | diagnostics: `[PF-PROBE]`, `[PF-PCMISS]`, `[PF-PAGEPOS]`, `[PF-XPORT]`, `[BERTI*]` (measurement window) |
| `DS3_STATS=1` | DRAMsim3 statistics of the measurement window (`DS3_OUT`, relative = per forked child) |

`CONFIG_AXI_LLC_MSHR_NUM` (LLC MSHRs) is compile-time; the run-time `mshr` cap is the L1D MSHR only.

## Running on the eda cluster (`scripts/dse/eda/`)

- `jbuild_dse.sh <src> <tag> "<EXTRA_CXXFLAGS>"` (on iprc-jump): static binary with `-march=x86-64-v3` for trace-model and
  librefcpu (a `-march=native` build on the jump host dies with SIGILL on eda-04) -> nfs `tm/bin/sim64_<tag>`.
- `mkjobs.sh` (weighted job list), `worker.sh <run>` (takes points from `tm/dse/<run>/jobs.txt` by an atomic mkdir claim, runs
  `fork_point.sh`), `full_launch.sh <node> <workers> <run>...` (the full-SPEC launch of EXP-048).
- Reports: `fork_report.py <run dir> <jobs>`, `variant_report.py <dse dir> <ref run> <run>...` (run on an eda node: the login
  node has no python3).
