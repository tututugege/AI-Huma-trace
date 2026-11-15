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

## Log
(filled in per commit)
