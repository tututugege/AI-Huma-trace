# Next state in seq, ids instead of payloads, lookups instead of scans (exp/global-sched, after u5)

Changes after the unified module form (UNIFIED_MODULES.md). Cycles are identical at every step (both gates, FPR,
TB / LSU traces, the Linux checkpoint, FE_PAIR / FE_IPF; `vbin.sh` in AI-Huma-data/tm-simplify). dhrystone:
7.85 s on main a127371 -> 6.3 s (same machine, alternating runs).

## 1. No d: the next state is written at seq

Before: `struct Regs {...} q_, d;` comb read `q` and wrote `d`, seq copied `d` to `q` (with dirty tracking for the large
parts). Now most modules keep one register set: comb computes its outputs and a few named wires from the registers and
its inputs; `seq()` updates the registers in place from the wires and this cycle's inputs. Where seq needs an old value
after it has started writing, it takes a local snapshot (`const Regs q = q_;` or a single array, as in Ren's RAT restore).

| module | comb (outputs and wires) | seq (register update) |
| --- | --- | --- |
| Rob | commit outputs, `CommitWires cw_` (n, prev_ctl, prev_blk, stall_cycle), redirect | commit, completion, branch recovery, enqueue, flush |
| Idu | decode outputs, `idu_consume` | branch-tag bookkeeping |
| Ren | handshake: fired slots, ready, commit count, interrupt report | RAT, free lists, checkpoints, restore, stage register |
| PreIduQueue | accepted block as wires (`accept_*`) | FTQ allocation, instruction buffer, pops |
| Dispatch | allocation and dispatch outputs | busy table, stage register |
| Isu / IssueQueue | picks, wake-up bus (`set_wakeup`) | picked entries leave, wake-ups, enqueue, flush, branch clear, ready bits |
| Exu | execute outputs | the old `comb_pipeline` body |
| MemRouteBlock | `wait_walk_`, `req_owner_`, `req_addr_` | applied |
| PeripheralAxi, Back2Front, Front2Back | outputs | next state (`b2f.comb`, `f2b.comb_hold` are gone) |
| BpuPipe | outputs, the array transaction `Txn` | see 4 |

Kept with q / d (their d is a real same-cycle wire, or the module is a stage-register file read by several phases):
RealDcache, WriteBuffer, MSHR, MemPtwBlock, the LSU (`pcur_` / `pnxt`), FrontControl (`front_state.d`).

## 2. The CSR file is CPU-wide state, not a module

`SimCpu::csr` (include/Csr.h, Csr.cpp) holds the CSR registers and the privilege. Every module reads it through
`csr_state()` (const): the status view `status` (mstatus, sstatus, satp, privilege) for the TLBs, `interrupt_req()` for the
ROB, `read(idx)` for the EXU, `trap_out()` (the trap / xRET target, mode and cause of this cycle's commit) for the ROB's
redirect and the interrupt report. Only SimCpu writes it: `csr.seq()` at the edge (after the back end, before the devices'
tick) and the checkpoint restore (`SimCpu::install_arch_state`). Its edge inputs: `RobCsrIO` (the interrupt response, the
committing CSR instruction's write from its uop and the value of its fact, FS-dirty) and the interrupt lines
(`SimCpu::csr_interrupt_inject_io`, driven by the peripheral model). Gone: BackTop's / the ROB's / the EXU's CSR pointers,
the `csr_status` ports of the LSU, the front end, the I-cache and Back2Front, `ExeCsrIO` / `TopCsrIO` and the
execute-time write latch (a shadow compare of the latch against the commit-time write: 0 differences on both gates, the
Linux checkpoint and the 13 Linux windows).

## 3. Uops carry an id

`uop_pool[id]` (`UopS`) holds the static and write-once fields of an instruction: decode results, `dest_preg`,
`rob_idx` / `rob_flag`, `stq_idx` / `stq_flag`, `ldq_idx`, `expect_mask`, the TMA metadata. A `Uop` keeps the id and the
per-copy dynamic fields (busy bits, `br_mask`, `dest_en` = "this copy writes its destination", result, op). Pool fields are
read as `u->field`, dynamic ones as `u.field`. The LSU's LDQ / STQ entries and its write-backs carry the id. id 0 is the
shared null entry; a write to it is reported at exit (`[UOP_POOL] the shared null entry was written`): pool fields are
written only for valid slots.

## 4. Per-cycle copies and scans replaced

| where | before | now |
| --- | --- | --- |
| front entry FIFOs | read data and requests held entry copies | pointers to entries (`FifoReq<Entry>`); only the head output and the pushed entry are copied at the edge |
| BPU history push | TAGE config / ring mask per folded history | read once per push, each table's leaving bit once |
| BPU registers | double-buffered `State` (12.6 KB), ~5 KB copied per cycle | one `State` written in place by comb (nothing reads it later in the cycle: `decide()` runs before, the arrays still change at seq) |
| BPU queues | PTAB checkpoints (64) and uBTB train events shifted on pop | rings (`pq_h`, `tq_h`) |
| BPU `Txn` | 3.3 KB zeroed per cycle | `Txn::clear()`: only the enables and counters (every record is read under its enable) |
| LSU watchdog | a counter per LDQ / STQ entry, incremented every cycle | the send cycle `wd_t`; the queues are scanned only at the earliest deadline `wd_next_` |
| trace bridge `find(uid)` | binary search over the deque (+ linear scan on a miss) | direct-mapped `slot_[uid & 4095]`, the search only on a miss |
| wake-up bus membership | Dispatch / Isu scanned every channel per source | `IssAwakeIO::woke_at[preg] == epoch` |
| IQ grants | whole grant structs cleared | the valid bits of the queue's own ports |

The watchdog, the bridge table and the wake stamps were first run next to the old code with an assert per cycle (both
gates and the Linux checkpoint), then the old code was removed.

## 5. A modelling fix that came along

LSU_MS_DATA (`ms_load_value`): a done store younger than the base store (the store whose `m_old` is the starting value)
is applied to the load's value: its data is in the cache but not in the base store's snapshot. Unchanged on both gates,
TB and the Linux checkpoint; the Linux window s03 (`linux_segments_cmp.sh`) now matches the RTL (PT / UP fingerprints
differed from bin 2545 before).

## Not done

* Phase merges inside a module (fa / inst / f2b ports, I-cache, rename, dispatch, LSU translate + requests).
* d of the D-cache / MSHR / write buffer / LSU (genuine same-cycle wires: they would need an explicit wire per use).
* The BPU's raw array data (2.7 KB per S1 block) is still registered whole; it must be read at S1.
