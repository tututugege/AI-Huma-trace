# P2a: facts drive the model's timing-relevant decisions (TB_INJECT=1)

The model's own value computation is still there; these points no longer read it when a fact exists:
| point | fact | n over 56 tests |
|---|---|---|
| BruUnit branch direction | br_taken | 61,200 |
| BruUnit jal/jalr target | next_pc | 42,641 |
| AguUnit address (load/STA) | vaddr | 107,846 |
| SduUnit store data (non-atomic) | rs2_value | 46,030 |
| Exu CSR write data | csr_wdata | 323 |
Overrides differing from the model's own computed value: 0 (facts == computation); fact misses: 0.
The model's latencies are not data dependent (MUL/DIV fixed latency), so no latency fact is needed.

Bridge additions found necessary:
- Boot ROM: the core boots from word-backed MMIO at 0x0..0x100000; the bridge's core is seeded from pmem and may fetch there.
- Control flow of not-executed instructions (blocked fetch, undone trap) is evaluated from the bridge registers
  (branch compare, jal/jalr target), because a wrong-path branch's mispredict also drives recovery.

Checks:
1. Gate with facts driving (TB_INJECT=1): 56/56 cycle counts equal to baseline.
2. Positive control (facts perturbed, --no-diff): flipping every 50th branch fact: prime 11,760 -> 2,923,
   quick-sort 2,779 -> 2,103; address perturbation (every 20th): quick-sort 2,779 -> 2,784 (prime unchanged: 10 loads).
3. Poison control (TB_POISON=1 TB_INJECT=1 --no-diff): ALU/MUL/DIV/FP results and non-MMIO load data of the model
   are XORed with 0xA5C3A5C3. Cycle counts stay equal to baseline on 45 of 56 tests (results/poison_all.txt),
   i.e. the model's timing does not read those values. The 11 others (csr-alias-boundary, fence-i-self-modify,
   illegal-instruction-trap, interrupt-vectored, software-interrupt-vectored, supervisor-external-interrupt-vectored,
   supervisor-illegal-delegation, supervisor-trap-delegation, timer-interrupt-mask, trap-boundary, trap-precise-store)
   hang: after a flush the bridge takes its state from the difftest reference, which does not step under --no-diff.

## Next (P2b)
Commit-time execution inside the bridge (traps, mret/sret, CSR side effects, interrupts with the cause from the model)
so the difftest reference is no longer a state source; then the 11 tests join the poison control.
Then P3 can delete the value datapath.

# P2b: commit-time events inside the bridge (no difftest reference as a state source)

- Exceptions (page fault, illegal, ecall, mret/sret are all `exception()` in the core): when an instruction traps at
  speculative execution the bridge keeps the state it reached (`trap_after`), undoes it so the stream continues,
  and at the commit-flush of that instruction (`commit_flush`) drops everything younger and takes `trap_after`.
  The trap is a deterministic function of instruction + architectural state, so it is a per-instruction fact.
- Interrupts: asynchronous. The model decides when (timer compare / pending bits, a function of cycle); at the commit
  boundary the tap (SimContext::run_difftest_interrupt) passes cause, target privilege and pending bits;
  the bridge drops every in-flight entry and calls refcpu_take_forced_interrupt. The interrupt cycle does not commit
  the head instruction (interrupt_nop), so the commit tap is skipped there.
- TB_RESYNC_REF=1 selects the old P1b behaviour (state from the difftest reference) for comparison.

Results (56 tests): check mode 0 mismatches in rd/branch/address/target (169,538 / 61,200 / 103,293 / 103,841),
cycles equal; plain gate and TB_INJECT gate 56/56 cycle equal; poison control (TB_POISON, --no-diff) 56/56 cycles
equal (was 45/56 in P2a), 18 s wall with 8 jobs.
Negative control (TB_NEG_NO_TRAP_STATE: do not take trap_after): the mismatch counters stay 0 because a wrong CSR
value read in the handler is auto-patched from the model; the signal is the csr_read patch count
(illegal-instruction-trap 0 -> 3, trap-boundary 0 -> 8, supervisor-trap-delegation 0 -> 7).
Gate rule derived from it: csr_read patches must be 0 except for mip/time reads (the only legitimately unknowable CSRs;
2 in timer-interrupt-mask).
