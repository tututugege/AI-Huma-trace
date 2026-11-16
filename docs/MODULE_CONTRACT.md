# Module contract (pilot: Rename, exp/module-contract)

Goal: every module has the same shape, so its behaviour across a clock edge, its interface and its comb order can be read
from the header and checked by tools; cycles stay identical to the RTL-aligned model.

## The contract

1. **Registers in one struct, q and d.** `struct Regs { ... } q_, d;` Comb reads the registers through
   `const Regs &q = q_;` and writes only `d` (a register write in comb does not compile). `d == q` at the start of every
   cycle; `seq()` copies `d` to `q` (large, rarely changing parts only when dirty).
2. **Declared phase interface.** In the module header, per comb phase, the Wires it reads and writes, and the outputs the
   module alone drives:
   ```
   //   CONTRACT comb_rename  in: -                                               out: ren2dis
   //   CONTRACT comb_fire    in: dec2ren dis2ren dec_bcast rob_bcast rob_commit  out: ren2dec
   //   CONTRACT owns: ren2dis ren2dec
   ```
   `scripts/comb/comb_graph.py --check` compares the declarations with the code (scanned reads / writes) and fails on a
   mismatch or on another module writing an owned output.
3. (not in the pilot) A runtime check in debug builds that a signal read in a cycle was written in that cycle, which
   would catch signals written only in some cycles (the PMU's stale `g_pipe_txn`). `wire<N>` is a plain integer alias
   (`AutoType<N>`) used everywhere and there are 77 `memset`s over structs of wires, so a stamped `wire<N>` is a model-wide
   change; options: a stamped wire type only in the ports of migrated modules, or a port-level stamp (writer phase / cycle
   per port struct) set by the owner and checked by the readers.

## Pilot result (Rename)

* ee0199d: the ~20 register / `_1` pairs became `Regs q_, d`; init builds `q` and sets `d = q`; seq is `q = d`. Found by
  the const `q`: `arch_cnt` (move elimination reference counts) was written in comb (`handshake`) directly, i.e. a
  register updated outside seq; it goes through `d` now. Ren.cpp / Ren.h -21 lines.
* 188af66: the declared interface of the two phases; the checker verifies it (a mutation dropping an input is reported).
* Gates RVC=0 62/62, RVC=1 67/67 identical, FPR identical; speed unchanged (Rename is ~3% of the run time).

Rollout order if adopted: the small modules first (Dispatch, Idu, PreIduQueue, Csr, Exu), then Rob and Isu, LSU last;
once most modules declare their phases, `BackTop::comb()` can be generated from the declarations (comb_order.py).

## Rollout to the small modules (1515840, e5015d3, this commit)

Dispatch, Idu, PreIduQueue, Csr and Exu (its load write-back wake register) have `Regs q_, d` and a declared, checked
interface; 14 phases of six modules are declared, 0 mismatches. The const `q` and "seq is a pure copy" found four more
places where a register changed outside the edge or seq held logic:
* PreIduQueue `flush_q` was set in seq from a wire (`rob_bcast.flush`): comb_fire sets `d.flush_q`;
* Csr applied the FS / SD dirty update to mstatus in seq: done at the end of `comb_interrupt_inject`, the CSR's last
  phase;
* Csr `csr_write` used the register `csr_wdata` as a scratch variable (CSRRS / CSRRC): a local;
* (Rename, before) `arch_cnt` written in comb: through `d`.
Not yet under the contract: Exu's FU state (`units`, updated in place in comb), Rob, Isu, the LSU. (BackTop setting
`csr->fs_dirty_req` directly is the port `top2csr.fs_dirty` now.) Gates RVC=0 62/62, RVC=1 67/67 identical, FPR
identical at every commit.

Exu FU state: a q / d split with a copy at seq costs 8-13% of the run time (31 units, each with 3 ExuInst and a ring of
them; rvc-alu 11.29 -> 12.79 s) for no change in behaviour. Every unit's state is read before it is written in a
cycle, so the in-place update is the d-only form of a register (the q copy elided); a debug check of read-before-write
would make that explicit without the cost.

## ROB, ISU, LSU (90f5b9c, aaf593c, 962679f, 8e68d43)

All eight back-end modules are under the contract now: 22 of the 23 phases of BackTop::comb() are declared and match the
code (BackTop's own comb_outputs is the one left). Found on the way:
* ROB: the deadlock watchdog counter was updated in comb (through d now); BackTop::comb_outputs patched the ROB's output
  rob_commit after the ROB drove it, a write nothing read afterwards (removed; the ownership check found it);
* ISU: a global flush cleared the wake-pipeline registers themselves in comb (clears d only now); the issue queues'
  entries / count / wakeup matrices are Regs q_, d (in-class functions read const q; constructor and seq write q_);
* LSU: it already had cur / nxt and pcur / pnxt; making cur_ / pcur_ private and const in comb compiled unchanged.
Remaining exceptions: Exu's FU state (in-place, documented above), the issue queues' ready-bit cache recomputed in seq
(a derived value of the new q), `out` / `in` (BackTop's output / input structs) shared by several writers.
Gates RVC=0 62/62, RVC=1 67/67 identical, FPR identical at every commit; rv64gc Linux console identical up to b6bd4c4.

## BackTop::comb() generated from the contracts

All 23 phases are declared (BackTop's own comb_outputs too; BackTop's shared in / out structs are named one level deeper,
e.g. `out.lsu2dcache`, so the LSU and BackTop each own their part). `scripts/comb/gen_comb_order.py` derives the order:
a module's phases in header order, a writer before every reader of its outputs in another module, the trace-bridge
users in `TB_ORDER` (fact lookups and BackTop's commit / flush: side effects outside the Wires; comb_graph.py checks
the scanned users equal that list). Direct reads of another module's registers (`ftq->ftq_pc`, `csr->read`,
`idu->q()`) need no order: q does not change during comb. Ties follow a fixed module priority, not the old code, so the
generated order is a different (valid) order from the hand-made one: gates RVC=0 62/62, RVC=1 67/67, FPR,
TB_TRACE_COMMIT and LSU_PIPE_EVT are identical with it, i.e. the declarations hold everything the order depends on.
The block in BackTop::comb() is between BEGIN / END GENERATED markers; `comb_graph.py --check` fails when it differs from
the declarations (a hand swap of two lines is reported). To add or change a phase: edit the module and its CONTRACT
line, run `scripts/comb/gen_comb_order.py --write`.

## SimCpu level: the tops export their phases

The same idea one level up, for hierarchical development (front end, back end, memory subsystem, AXI as separately
developed tops inside SimCpu). Each top lists its SimCpu-level comb phases above its class with the ports they use;
SimCpu (include/SimCpu.h) holds the port table, i.e. the wires between the tops, as paths in SimCpu:
```
//   PORT dcache2lsu = back.in.dcache2lsu
//   EXPORT comb  in: csr_status back_in fetch_group dcache2lsu  out: back2front lsu2dcache mmio_req dtlb_walk   (BackTop.h)
```
Kinds: `in` (read after this cycle's writer), `pre` (read before it: the value of the clock edge or of the default
driver), `out` (one writer per port), `default` (driven first every cycle, the other writers override it: an
always_comb default; SimCpu's back_in_from_regs and axi_comb_begin). `scripts/comb/gen_top_order.py` generates the
comb block of `SimCpu::cycle()` from them: a top's phases in header order, a default driver first, a writer before its
`in` readers, a `pre` reader before the writer, the trace-bridge users (the I-cache fetch, the memory route block's PTW /
probe data, the back end's commit, all reading the functional memory that the commit updates) in that order; ties by a
fixed top priority. Its check (part of `comb_graph.py --check`) compares the exports with the scanned top domain: every
field of a port a phase reads or writes is declared, a read inside a port another phase writes names that port, every
declared port is used, a field two tops share is part of a port, one writer per port. Mutations (an input dropped, a
`pre` dropped, two generated lines swapped) are reported.

A cycle between two tops shows up as a cycle in the exports; the fix is to split a phase where the RTL has two
independent cones, as the memory subsystem's comb_outputs (responses, before the back end) / comb_inputs (requests,
after it) and the AXI's comb_outputs / comb_inputs do. A cycle that cannot be split is a combinational loop in the
design.

Changes the exports needed (cycles identical):
* the MMIO response went from the memory subsystem into `back.in.peripheral_resp` after the back end had read it, and
  SimCpu copied it from there into its hold register: PeripheralAxi now writes the hold register's d directly (its
  request / response pointers are in its `in` / `out`, so the scan sees them), the copy phase is gone;
* `back.out.fire` was a pointer into the pre-decode queue's wires: a value driven by BackTop::comb_outputs;
* the I-cache cleared the interconnect's `accepted` / `accepted_id` on reset, which the interconnect overwrites every
  cycle and never reads: removed (one writer per port);
* the comment that `resp.ready` is read in the interconnect's comb was wrong: it is sampled at the clock edge.
The generated order differs from the hand-made one (the front end's next-cycle inputs and the front-to-back hold run
before the memory subsystem's requests); gates RVC=0 62/62, RVC=1 67/67 and FPR are identical.

Known: the AXI ports' fields are scanned without the master index, so the I-cache's and the memory subsystem's parts of
the read port array are two port names over the same fields (aliases; the one-writer rule is per name). The interconnect
reads the memory-side masters' requests in comb_outputs only as their default (cleared) values (`pre`), part of the AXI
kit's internal registers listed in docs/REG_WIRE.md. TB_ORDER at this level is declared, not scanned.

## Memory subsystem under the contract

The memory blocks (D-cache, MSHR, write buffer, route block, PTW, MMIO) already had cur / nxt register pairs.
* q is read-only in comb: each block's `cur` is a private `cur_` behind a const reference (`s1s2_cur` / `bank_retry_cur`
  in the D-cache), written only in seq / init; it compiled unchanged. The MSHR and the write buffer update nxt in place
  and never read cur (d == q at the start of the cycle, as Exu's FU state).
* seq is a pure copy: the PTW block applied the TLBs' walk flushes in seq (`apply_seq_inputs`); they are set by the
  TLBs in front.comb / back.comb, before the memory subsystem's requests, so ptw_feedback passes them in and comb_finish
  clears the client's walk state in nxt (after the outputs, which show it from the next cycle on, as before).
* one writer per signal: drive_dcache_response set `dirty` on the MSHR's own fill request (`mshr_dcache_io_.fill_req`)
  when a waiting store completed into the arriving fill; it is the wire `fill_store_dirty_` now, ORed in by D-cache
  stage 1 (the only reader). Its writes through local reference aliases are direct member writes (the scan sees them).

The 11 phases are declared in MemSubsystem.h in MemSubsystem's signal names (the wires MemSubsystem::init binds to the
blocks' in / out pointers): `CONTRACT <block>.<method>` for a block's comb method, `CONTRACT <function> (<block>)` for a
MemSubsystem function driving a block (the D-cache stages, the PTW request / feedback). `scripts/comb/gen_mem_order.py`
checks them against the mem-domain scan (exact read / write sets; a block's own state is private and not declared;
one writer per signal) and generates both bodies. The comb_outputs / comb_inputs split is derived, not declared: a phase
from which the back end's input dcache2lsu depends goes before the back end, a phase that depends on a back-end /
front-end request (lsu2dcache, the I-cache probe, the MMIO request, the walk ports) after it; one that is both is a
loop through the back end and is reported. The generated order equals the hand-made one. `gen_mem_order.py --emit`
prints the CONTRACT lines of the scanned code (a starting point when a phase is added). Mutations (an input dropped, a
response phase depending on lsu2dcache, two generated lines swapped) are reported.

Known: the walk-port adapters (`dtlb_walk_port_inst` / `itlb_walk_port_inst`) are declared as whole objects: the PTW
request reads the TLBs' requests from them, the feedback clears them after use.

## Front end under the contract

The front end already had the q / d form: the registers are read through `rd` (FrontRegs and the FIFOs' snapshots, taken
at the edge), every update is a request in `req` applied by seq. What was left:
* the BPU pipeline's transaction and decision of the cycle lived in file statics written by comb (`g_pipe_txn`,
  `g_pipe_dec`, `g_pipe_txn_valid`): the transaction is BpuPipe's own d now (`comb_cycle()` builds it, `seq()` commits
  it), the decision is a FrontComb wire (`pipe_dec`);
* the BPU ran either inside read_fifos (when the PTAB is bypassed) or after it (`bpu_done`): it reads nothing the FIFO
  reads produce, so the phases are control (flush / refetch, the pipeline's decision) -> run_bpu -> read_fifos; the
  PTAB's write port and the head fix-ups (FE_PAIR slot cut, RVC=1 carry) are part of read_fifos, so each FIFO head has
  one writer; the predecode check is check_group;
* the I-cache's invalidate step K wrote over the fetch step's outputs (`ic_out`): it writes `ic_k`, finish reads the
  outputs of the last step;
* writes through local reference aliases (`pd`, `txn`, `state`) are direct; the FE_IPF prefetch candidates are their own
  request (`req.ipf`) instead of a second writer of `req.front_state`; the PMU's "last refetch" cycle, a function-local
  static updated in comb, is a front-end register (`FrontRegs::pmu_last_refetch`).

Checked beyond the gates: FE_IPF=1 identical on all 67 RVC programs; FE_PAIR=1 identical (fact, which never finishes with
FE_PAIR even on the base, has the same commit stream over 2M cycles). FE_PAIR caught one slip on the way: the
multi-block slot masks must be taken after the FE_PAIR cut and before the RVC=1 head fix (they are part of fix_heads).

The 9 phases are declared above struct FrontComb (front_top.cpp); `scripts/comb/gen_front_order.py` checks them against
the front-domain scan (exact read / write sets) and generates front_comb_calc(). A FIFO's state (its _in struct, its
read data and its request) is declared once with its ports in order (`CONTRACT fifo <name>: ... ports: read_fifos
write_...`): the only signals with two writers. The generated order equals the hand-made one. Mutations (an input
dropped, two generated lines swapped, a phase made to read a later phase's output) are reported.

With this, every comb schedule of the CPU is generated from declarations: SimCpu::cycle() (the tops' EXPORTs),
BackTop::comb(), MemSubsystem::comb_outputs / comb_inputs, front_comb_calc(). Not covered: the AXI kit's internals, the
I-cache module's own steps (icache_top.cpp, written per the RTL), Exu's FU state (in place).
