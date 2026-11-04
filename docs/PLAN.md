# Distillation plan: execution-driven model -> trace-driven timing model

Goal: a standalone timing model, cycle-identical to the execution-driven baseline on the gate set, much faster.
Later: generated RTL + this model, so the event/fact interface must not depend on the old model's internals.

## Gate
- 56 tests (scripts/build_tests.sh, LLVM riscv32), results/baseline/baseline.csv, total 311,725 cycles, 13 of them interrupt/exception/CSR/fence.i.
- Pass = every test's cycle count equal to baseline (no pipeline-detail check).
- Excluded: memory-stress (long), load-store (misaligned UB under clang), matmul_soft/mersenne/string (need libc).
- Weakness: tests are tiny (most <1k cycles). Add 2-3 longer programs later (timer-mmio is the longest, 206k).
- Every pruning step needs a negative control: perturb a deleted value -> cycles must stay equal;
  perturb a kept fact (branch outcome, address) -> cycles must change. Otherwise the gate may be vacuous.

## P0 Profile the baseline (before deleting anything)
Where does host time go: FPU/ALU values, PRF/DCache data arrays, PTW/MMU, AXI/DDR, BPU, comb/seq call overhead?
Rank by share so we delete the biggest first and know the ceiling. Use `sample` (needs get-task-allow) and an
instruction-retired counter; do not trust wall-clock alone (layout-sensitive).

P0 result (docs/history/P0_PROFILE.md): ~68% of host time is memmove in AXI_LLC::comb/AXI_Interconnect::seq (whole-state copy);
value logic (FPU/ALU) is ~0%. So the first speed step is P3a below, before the bridge.

## P3a (moved first) LLC/AXI state: no whole-state copy, drop line data payloads; gate 56/56 equal.

## P1 Bridge in check mode (nothing injected)
- A functional core (librefcpu) follows the model's fetched PCs, wrong paths included:
  forced jump when the decoded pc differs from the core's next pc; checkpoint/undo log per instruction;
  mispredict -> rewind to first instruction younger than the branch; flush -> rewind to oldest in-flight;
  speculative stores in a store buffer, published at commit; values the core cannot know (timer, MMIO) patched.
- Identity: a uid per fetched instruction, carried in the instruction record.
- Check: for every executed instruction (wrong path too) compare the core's fact with the model's own value:
  result, branch taken/target, load/store vaddr/paddr/size, exception cause, div/FP latency inputs.
- Done when 56/56 have 0 mismatches and cycles unchanged.
- Open: librefcpu's API (get/set state, undo). If it lacks rewind, add it in GenSimpoint (it is the user's code).

## P2 Inject facts
Per-instruction Fact record (from the bridge, keyed by uid) replaces computation at the point of use:
branch resolve (taken/target -> mispred), AGU (vaddr, paddr, size), exception/page-fault, CSR read value
class, mul/div/FP latency, MMIO read. Keep timing structures (ROB, IQ, rename, LSU queues, BPU, caches).
Done when 56/56 equal with Facts driving everything and the old value path still present but unread.

## P3 Delete (largest first, one step per gate run)
1. FPU (fadd.h 12k + fmul.h 9.7k lines): latency only.
2. ALU/mul/div result datapath, PRF data arrays, bypass data.
3. CSR file values, store data, STQ data, DCache/ICache data arrays (keep tags, MSHR, replacement).
4. TlbMmu/PTW: translations come as facts; keep walk timing (PTE addresses from the bridge).
5. Bit-level: field widths via struct edit; find dead bits by taint, validate with the gate.
Order is by dependency: comb order is already fixed in the model, so a field change is a compile-checked edit.

## P4 Speed
Profile again. Expect the structure (comb/seq phases, 22 issue ports, wire<N> structs with memset, AXI/DDR
kit) to dominate once values are gone; restructure the hot clusters (frontend, ROB+LSU, IQ) to plain C++,
cycle count held by the gate.

## P5 Freeze the interface for RTL pairing
Event + Fact protocol (decode fire, dispatch, commit, mispredict, flush, fact fields) documented in docs/;
the same bridge must work from an RTL tap. Gate source switches from the old model to RTL.

## Risks
- Interrupt/timer timing is cycle-based: random IRQ delay (seed 1, 8..64 cycles) must be reproduced exactly.
- Wrong-path loads pollute caches: their addresses need the bridge's wrong-path values (P1).
- Gate is small: negative controls are mandatory.
- Speed-up is unknown until P0: if values are a small share of host time, the win comes from P4, not P3.

## Status table (updated at the end of P3)
| step | state |
|---|---|
| P0 profile | done (docs/history/P0_PROFILE.md) |
| P3a LLC copy, P3b IQ/PreIdu copies | done, exact (patches 0001, 0002) |
| P1 bridge (check mode) | done (0003) |
| P2 fact injection + commit-time events in the bridge | done (0004, 0005) |
| P3 delete values | done: FU compute+FPU (0006), PRF (0007), cache data (0008), unified Uop (0009), wide payloads (0010) |
| P4 speed (idle-tick skip, front end, back end) | not started |
| P5 interface freeze for RTL | not started |
