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

## One form for every module

* Registers: `Regs q_` (private, written only by seq / init), `Regs d` (comb writes it; d == q at the start of every
  cycle), read in comb through a local `const Regs &q = q_;`, `q()` for other modules. The memory blocks had
  `cur` / `nxt` (and the D-cache two pairs, `s1s2_*` / `bank_retry_*`): all are `Regs q_, d` now (`using Regs =
  <state struct>`; the D-cache's Regs holds both groups). The MSHR and the write buffer still update d in place
  (their comb reads d, which equals q until written); PeripheralAxi's seq lost a redundant `nxt = cur`. The front
  end's own registers were already `rd.q` / `req.front_state.d`.
* Contracts: in the header of the code they describe, in the names that code uses. A back-end module: the Wires it
  reads / writes. A memory block: its own ports (`in.axi`, `out.mshr2dcache`); gen_mem_order.py maps them through the
  bindings of MemSubsystem::init. MemSubsystem's own comb functions (the D-cache stages, the PTW request / feedback):
  in MemSubsystem.h, in its signal names, with the block they drive and the tie-break priority. The front end's phases:
  above struct FrontComb. A top's SimCpu-level phases: EXPORT lines above the top's class, ports from include/SimCpu.h.
* Generators: gen_comb_order.py (BackTop::comb), gen_mem_order.py, gen_front_order.py, gen_top_order.py share
  scripts/comb/contract_lib.py (the scan, the topological order with a fixed tie-break); `comb_graph.py --check` runs
  all four checks. `--emit` (mem, front) prints the lines of the scanned code; `--write` rewrites the generated block.
The memory order changed in one place (the write buffer's AXI requests now before the MSHR's: the priority list puts
the write buffer first; they drive different masters and both depend on stage 2 only).

## Front end as objects (exp/front-obj)

The front end kept its state at file scope: the four FIFOs (`g_inst_fifo`, `g_ptab`, `g_f2b_fifo`,
`g_fetch_addr_fifo_model`) behind free functions (`instruction_FIFO_read`, ...), the I-cache top (`icache`, `g_tags`,
`g_valid`, `g_lookup`, `g_bind`, `g_edge`, four `extern` port pointers, a fill-hook function pointer, a function-local
static rotation), `g_regs`, `bpu_pipe`, `front_ctx`, the FE_IPF statics. Now:
* `FrontCore` (front-end/front_core.h), owned by FrontTop (`core_`), holds all of it: `FrontRegs q_`, `bpu_pipe`,
  `fetch_addr_fifo`, `inst_fifo`, `ptab`, `f2b_fifo`, `icache`, `ipf`, `ctx`; one cycle is `seq_read` / `comb` (the
  generated FrontComb phases) / `seq_write`.
* `ICacheTop` (front-end/icache/icache_top.h): the I-cache top as a class; FrontTop binds its ports with `set_ports`
  (they take effect at its next step, as before) instead of the extern globals; the prefetcher's fill report is a
  member callback.
* The FIFO classes are in front-end/fifo/front_fifo.h (the wrappers are gone); the predecode checker, which has no
  state, is one comb function `predecode_checker_comb(in, out)` (its seq_read / seq_write were copies / empty).
* FastWarm reaches the BPU / I-cache / ITLB through `cpu.front.warm_*`.
FrontComb keeps references to the modules (`bpu_pipe`, `fetch_addr_fifo`, ...): the scan resolves their method calls
through these types (a method's non-const parameters are its outputs), so the contracts see the same signals; the
FIFO / I-cache methods' pure outputs are now seen as writes only (read_fifos' declared reads shrank accordingly).

## Back-end modules with their own ports (exp/back-ports)

The back-end modules used to reach every signal through one shared struct (`Wires`, `wires->X`): by type any module
could read or write any signal; only the contracts and the scan restricted it. Now each module has
```
struct In  { const DecRenIO *dec2ren; ... } in;    // what it reads: const pointers, bound by BackTop::init
struct Out { RenDisIO ren2dis; RenDecIO ren2dec; } out;   // what it drives: its own members
```
* A module cannot write an input (const) and cannot reach another module's output except through an input bound to
  it: one writer per signal holds by construction. All bindings are in BackTop::init (`dis->in.ren2dis =
  &rename->out.ren2dis;`), which reads as the back end's data path.
* The back end's interface stays in BackTop (`BackPorts`, back-end/include/BackPorts.h, formerly Wires): `in`, `out`
  (SimCpu / front end / memory subsystem), `csr_status`, `csr_interrupt_inject_io`, and BackTop's own `top2csr`. The
  modules that drive part of it have output pointers bound to it (the CSR: `csr_status`; the LSU: `out.lsu2dcache`,
  `out.peripheral_req`); a module that reads it has an input (`in.top_in` = BackTop's `in`).
* The pre-decode queue reads Idu's branch latch register through `in.idu_br_latch` (bound to `&idu->q().br_latch`).
* Found on the way: the CSR writes `csr_status` (BackTop's comb_csr_status calls it), so it is the CSR's output, not
  an input of it.
The contracts keep the signal names (a port is named after its signal); the scan rewrites the port form into the
signal form before reading (comb_graph.py `as_wires`), so the checks are unchanged. Done in two compile-checked passes:
inputs first (a module's reads become `in.X->`), then the outputs moved into the modules.

## Front-end IO, memset, environment, file-scope state, LSU files (exp/front-io)

* `FrontTopIn` is grouped: `ctrl` (reset, itlb_flush, fence_i, fifo_read_enable), `redirect` (valid, address, the
  mispredicted branch's rf_*), `train[COMMIT_WIDTH]` (one BpuTrain per committing instruction, instead of eleven
  parallel arrays).
* The three entry FIFOs' inputs are `FifoIn<Entry>`: the control of the cycle and `wdata`, the entry itself (its
  fields were repeated under other names; load() is a copy now). The fetch-address FIFO keeps its own input.
* No memset on struct objects (docs/STYLE.md); a clear is now a pure write to the scan (the D-cache's declared reads
  shrank, the order is unchanged).
* All environment variables in `SimEnv` (include/env.h).
* File-scope state moved into its modules: Isu's write-back busy tables, PreIduQueue's per-cycle accepted entries,
  the LSU's memory-dependence predictor (`MemDepPred`), the D-cache arrays (`DcacheArrays` in RealDcache).
* The staged LSU is one file per phase: RealLsuPipe.cpp (the clock edge and the phases' composition),
  LsuPipeOutputs.cpp, LsuPipeWriteback.cpp, LsuPipeTranslate.cpp, LsuPipeRequests.cpp, with the shared helpers in
  LsuPipeInternal.h. BPU_pipe.h (already in banner sections around one class) and IO.h (a list of IO structs) stay.

## Trace bridge: no state copies, entries pooled (exp/bridge-fast; function-model exp/step-access)

dhrystone profile: the bridge's dispatch read the function model's whole state twice per instruction
(refcpu_get_state, about 0.7 KB, for a few fields) and allocated an Entry per instruction; with the commit that
freed it, about 8% of the run time. Now dispatch reads single fields (refcpu_gpr / fpr / csr / pc / privilege /
reservation / store_info), a forced jump / a blocked instruction sets the pc with refcpu_set_pc (the same effect as
the get / set round trip, the visible mip / sip latched), only the trap path still takes the whole state; entries
come from a free list. The bridge's share went to about 4%. The Makefile takes the API header from REFCPU_INC (the
same checkout as REFCPU_LIB; default the function-model symlink).
Gates RVC=0 62/62, RVC=1 67/67 identical, FPR identical, TB_TRACE_COMMIT identical.

## One instruction decoder (exp/bridge-fast)

IDU and the trace bridge decoded each instruction separately (IDU into a uop, the bridge again to find which
function-model fields an instruction writes and whether it loads or stores). Both now call `decode_inst(inst)`
(bridge/InstDecode.cpp, a pure function returning `DecodedInst`): the operand fields, enables, type, immediate, CSR
legality, and the function-model view (writes_fpr, touches_fcsr, writes_xreg, mem_load / mem_store, is_lr).
Idu::decode copies it into the uop and keeps only the timing rules (AMO dest, x0 dest, TMA flags) and branch tags
(alloc_tag is now an Idu member, not file scope). The function model still executes at ROB dispatch (its undo log
and sim time), so cycles do not change.
Gates RVC=0 62/62, RVC=1 67/67 identical, FPR (4 programs) identical, TB_TRACE_COMMIT and LSU_PIPE_EVT identical
with the previous binary.

## SimCpu cleanup (exp/simcpu)

rv_simu_mmu_v2.cpp (the original simulator's name for the SimCpu implementation) is SimCpu.cpp, in the form of the
other tops:
* No `#define private public` around AXI_Interconnect.h (it was for kit revisions without comb_inputs_internal_tick();
  everything SimCpu uses is public). Unused includes and the oracle-mode hold register removed.
* The program's exit code is a0 of the function model (`tbhook::arch_gpr(10)`); `BackTop::get_reg` returned 0, so a
  non-zero exit code was never reported.
* AxiSubsystem (include/AxiSubsystem.h): the interconnect, DDR, MMIO bus and UART with comb_begin / comb_outputs /
  comb_inputs / seq, in place of free functions over SimCpu and a field-by-field copy outside a class. SimCpu's
  axi_comb_* phases call it; the LLC lookup table (in MemSubsystem) is passed in and its table out read in seq.
  comb_top.py inlines AxiSubsystem's phases like FrontTop's (members -> axi.*); the scan is the same as before but for
  the path names.
* SimCpu's registers are `Regs q_, d` (the held fetch group with its valid bit, the MMIO response) with
  `regs_seq() { q_ = d; }`; the `live` flags are gone (copying the whole Regs each cycle costs nothing measurable on
  dhrystone; copying only a valid group was ~3% slower, a code-layout effect). The I-cache port registers stay as they
  are (loaded at the start of the cycle, REG_WIRE.md).
* Side doors: `back.number_PC` was written by SimCpu every cycle but read only right after a restore set it; it is
  `restored_pc` now, set only by install_arch_state. `restore_pc` (a switch from fast mode) wrote back.out and
  front.in directly; FrontTop::restart_at / BackTop::restart_at do it now, in the same order.
* Kept: the commit hooks (Ren -> SimContext -> SimCpu::commit_sync -> the peripheral model) are the environment's
  action at commit, as in the RTL testbench (rtl/docs/modules/soc_top.md): a committed store to a modeled device acts
  in the same back.comb, where csr.comb_interrupt_inject reads the interrupt line. A port read after back.comb would
  delay it a cycle.
Gates RVC=0 62/62, RVC=1 67/67 identical, FPR / TB_TRACE_COMMIT / LSU_PIPE_EVT identical; the restore path: a
function-model special checkpoint of dhrystone (instruction 300000) run for 400000 instructions, log identical.

## Checkpoints read by the function model (exp/simcpu; function-model exp/step-access 55affa0)

The trace model parsed the checkpoint file itself (driver/BackTopLoad.cpp: header, CPU_state layout, sparse memory,
MMIO ranges), a second copy of the function model's restore_checkpoint. Now the function model reads it
(`refcpu_restore_checkpoint`; it got the page-sparse reader and accepts a smaller RAM image, as the trace model did)
and the trace model only does what is its own: the timing model's architectural registers (`BackTop::install_arch_state`:
privilege, CSRs, LR/SC reservation, the refetch from the pc), the device registers as stored into the model's IO
backing (`refcpu_get_io_backing_word`), the restore program of a front-door restore, the warm-up. The loaders are
driver functions (driver/Load.cpp: load_image, restore_checkpoint, restore_checkpoint_front_door, restore_from_ref),
not BackTop methods; BackTop's ckpt_interval_inst_count is gone (the restore returns the interval).
Checks: a function-model special checkpoint of dhrystone (back door 400k instructions; front door with the restore
program) and of rv64 Linux at 40M instructions (back door 3M instructions, 2,126,672 cycles): output identical to
the previous binary but for the restore messages. Gates RVC=0 62/62, RVC=1 67/67. Not checked: SPEC checkpoints
(server). Found: tools/ckpt2stub.py gives pc 0x0 for these format-5 checkpoints (the restore program jumps to 0).

## Global schedule, step 1: one dependence graph of all levels (exp/global-sched)

`scripts/comb/global_graph.py` (printed by `comb_graph.py`, section `== global`): the rows of SimCpu::cycle() with each
summarised sub-object call (back.comb, front.comb_core, mem_subsystem.comb_outputs / comb_inputs) replaced by that
domain's own phases as SimCpu paths (comb_top.py keeps them in TOP_DETAIL), 203 phases. From the current order it
derives the edges every schedule must keep: data (read-after-write, write-after-read, write-after-write), the order of
phases inside one object (a back-end module, the front core, one memory block, one AXI object: their state is not a
signal in the scans) and global side effects as pseudo-signals: the trace bridge (tbhook::, the functional memory:
front.run_icache / write_instruction_fifo / finish, mem_route_block.comb_response, the back end's TB_ORDER) and each PMU
event bus (g_pmu_front: front -> back.pmu_sample; g_pmu_back: rename / idu -> pmu_sample). It prints the current runs per
top module (14), a greedy schedule's runs (13), every cross-module data edge (161) and the side-effect users. No code
changed; the schedule is still generated per level.
