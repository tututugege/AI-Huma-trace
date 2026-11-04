# P1: trace bridge (functional core following the model's dispatch stream)

## Status
- P1a DONE: src/TraceBridge.{h,cpp} + tests/bridge_selftest.cpp. Uses only the existing librefcpu API (no GenSimpoint change).
- P1b DONE: taps in the model + check mode (TB_CHECK=1), patches/0003-p1b-bridge-check-mode.patch.

## Design
- Identity: `uop.dbg.inst_idx` (assigned in Idu decode, carried through the uops to the ROB) is the uid.
  Caveat to verify at tap time: the counter is a static incremented per decode evaluation; it must not skip/duplicate
  for dispatched instructions.
- dispatch(uid, pc, inst) in dispatch order = fetch order of instructions that enter the ROB. If pc != core's next pc
  the core is forced to jump (wrong path and nested wrong paths follow the model's stream).
- Undo log per instruction (not the ~270 B RefCpuState): pc, privilege, reservation, rd old value, the 8 CSRs a trap
  writes (mstatus sstatus mepc mcause mtval sepc scause stval), the whole CSR file only for SYSTEM instructions,
  and the old memory word of a store/AMO. Rewind applies the logs youngest first on one state copy.
- Blocked (not executed speculatively, state kept, pc+4): pc outside RAM, ebreak (refcpu would end the run),
  misaligned load/store (refcpu exit(-1)), load/store outside RAM (MMIO or illegal: no speculative device access).
- A speculative trap is undone at once (state + store) and the stream continues at pc+4; the committed-path trap
  is to be executed at commit (P1b).
- Facts per instruction: next pc, rd/value, branch taken, load/store vaddr/size/store data, blocked/trapped flags.

## Checks (P1a)
- Self-test: plain run (no wrong path) vs run with random wrong paths (1-24 instructions from random text pcs,
  garbage pcs, stack region), rewind to the last right-path uid, commit lag 16.
  Must match per instruction: next pc, rd value, br_taken, vaddr, store data, blocked; and final regs, CSRs, pc,
  memory hash. Result: 56/56 built tests at 50% wrong-path injection (up to 300k instructions).
- Wrong-path traffic seen, e.g. prime at 100%: 86,647 dispatched, 6,371 forced jumps, 23,972 speculative traps,
  7,505 pc-out-of-RAM, 664 ebreak, 130 misaligned, 1,334 out-of-RAM accesses.
- Negative controls (compile flags): no rd undo -> mismatches; no store undo -> mismatches.
- Limits: no translation (satp) handling yet (loads/stores use vaddr as paddr), no MMIO/timer/interrupt facts,
  CSR instructions executed speculatively only restore the CSR file (device side effects are not modelled),
  AMO/LR/SC reservation semantics only via the saved reservation.

## Next (P1b)
1. Tap Dispatch (dis_fire -> bridge.dispatch with dbg.inst_idx, dbg.pc, dbg.instruction).
2. Tap Exu mispredict (uid of the mispredicting uop) and ROB flush -> rewind_after; ROB commit -> commit().
3. Compare at execution: ExuRobUop.result vs fact.rd_value, br_taken, AGU address vs fact.vaddr.
4. Commit-time exceptions/interrupts/CSR side effects: execute for real at commit.

## P1b result (check mode, nothing injected)
Taps (all no-ops unless TB_CHECK is set; with it unset the 56-test gate is unchanged: cycle counts equal, logs identical):
- dispatch: Rob::comb_fire (uid = dbg.inst_idx; also records rob_idx -> uid, because the LSU path loses dbg.inst_idx).
- mispredict: Rob::comb_branch -> rewind_after(branch uid). commit: BackTop commit loop -> commit(uid).
- flush: BackTop (rob_bcast->flush) -> at the next dispatch the bridge core takes the difftest reference's state
  (the reference executed the flushing instruction, trap or CSR side effects included).
- exec: end of Exu::comb_fu2exu_collect, compares what goes to the ROB: rd value (add/mul/div/csr/fp/load), branch
  direction, next pc of jal/jalr/branch (diag_val), load/store address.
Patches (values the core cannot know): MMIO load result (blocked MemOutOfRam) and CSR reads whose value differs
(mip). The bridge rewinds younger entries, writes rd, replays them. 56 tests: 35 mmio_load + 2 csr_read patches.

Results over the 56 gate tests (scripts/check_run.sh, results/check2):
- compared rd 169,370; branch dir 61,200; addr 103,293; target 103,785.
- mismatches 0 / 0 / 0 / 0; nofact 0; uid conflicts 0; commit_front_miss 0.
- cycles identical to baseline in check mode (it only observes).
- Before patching: 200 rd + 19 br + 34 addr + 19 target mismatches, all in the 4 interrupt/timer tests
  (timer MMIO read and mip read feed later values).
- Negative control (TB_NEG_NO_REWIND=1: ignore mispredict rewinds): prime 3,866 rd + 112 br mismatches,
  quick-sort 1,889 rd + 512 br + 527 addr mismatches.
Limits: no translation (satp) handling; store data and unaligned/AMO/LR-SC facts are not compared yet;
commit-time exceptions rely on the difftest reference state; wrong-path MMIO-looking loads are patched with the
model's value (needed to keep wrong-path addresses equal), misaligned loads are skipped.
