# trace-model simplification (exp/tm-simplify, from exp/drop-rv32-code 75c8141)

Goal: much less code, somewhat faster, fewer cross-module combinational calls; cycles identical at every commit.

## Rule for every commit
* `scripts/run_gate.sh` (RVC=0, 62 programs) and `scripts/run_gate_c.sh` (RVC=1, 67 programs) identical to the base.
* Larger steps also: rv64gc Linux boot (`scripts/linux_check.sh`, log compared with the base run) and the speed set
  (`build_c/gen/{rvc-alu,rvc-mem,fd-rvc-mem}`).

## Base (75c8141)
| | value |
| --- | --- |
| model C/C++ lines | 37,269 |
| rvc-alu / rvc-mem / fd-rvc-mem cycles/s | 271.9k / 206.1k / 218.6k |

## Plan
1. Freeze the remaining experiment switches (BPU `BPA_*`/`BPO_*`/`BPU_*`, other `getenv` knobs); debug prints behind one mechanism.
2. Delete what the frozen configuration never reaches (AXI kit paths, duplicate device models, compatibility layers).
3. Write rules (`Reg<T>`, `Wire<T>`) on a pilot module; then a declared comb order instead of hand-written call chains.

## Result of the first night (2026-11-16, 21 commits after 5f1b82b)

| | base 75c8141 | now |
| --- | --- | --- |
| model C/C++ lines | 37,269 | 32,712 (-12.2%) |
| environment variables read | 77 (ENV_VARS.md) | 12 |
| rvc-alu / rvc-mem / fd-rvc-mem wall time | 14.7 / 15.4 / 14.2 s | 12.2 / 12.7 / 11.7 s (1.2x) |
| back-end comb order hazards (scripts/comb/comb_graph.py) | 1 real (+2 false positives) | 0 |
| gates run_gate.sh / run_gate_c.sh | 62 / 67 | 62 / 67 identical after every commit |

What was removed, by kind:
* frozen experiment switches (BPU 15, LSU memory speculation / value path, front-end fast paths, FP write ports, DRAMsim3 run-time
  overrides) and the compile-time variants nobody builds any more (FP=0, line ROB, no move elimination, no LLC, fixed-latency SimDDR);
* statistics and debug output that no script reads ([BPU_*], [PD_CHK], [RVC_FE], [LSU_PIPE_STATS], [MS_DATA], [TB_CHECK], [FP*], ...),
  the TB_CHECK value compare and its positive / negative controls, the RTL alignment fingerprints (FPR), compiled-out logs;
* dead code: 370+ unused constants / macros / types, ~60 functions nothing calls, the SD card controller model, the TAGE-SC-L / loop
  predictor metadata that still travelled front end -> FTQ -> back end -> BPU;
* duplication: the instruction FIFO / PTAB / front-to-back FIFO are one template.
Kept on purpose: PMU / TMA, FE_PAIR, FE_IPF, PF_ENABLE (nopf gate), LSU_PIPE_NOWD (RTL cycle comparisons), DS3_OUT (parallel runs).

Speed: the BPU's per-stage register no longer carries the 2.7 KB array-read buffer (it is one register next to S2), the statistics
counters are gone.

Comb order: scripts/comb/comb_graph.py extracts the read / write sets of the 69 back-end comb phases over the Wires (field and
sub-field level, local aliases, same-class helper calls) and reports a phase that reads a wire a later phase writes (it then sees
the previous cycle's value: a hidden register). The one real case (PreIduQueue reading the ROB flush of the previous cycle) is an
explicit register now; `comb_graph.py --check` is part of the per-commit check. Numbers: 69 phases (LSU 19, ROB 8, ISU 7, ...),
44 module-to-module wire edges.

Not done yet (next steps): the icache stack (ICacheTop / icache_module / icache.cpp / GenericTable, 2.9k lines of glue layers for
one cache), the AXI interconnect's remaining generality (2k lines), the two UART models (PeripheralModel and the AXI kit's), the
front end / memory subsystem in the comb checker, and the table-driven scheduler (declared reads / writes per phase, generated order).

## Log
See `git log 75c8141..` (one commit per step, each with its gate result).
