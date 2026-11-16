# Reg / Wire refactor (exp/tm-reg-wire, from main fb471da)

Goal: every value that lives across a clock edge is an explicit register (written in seq, or `d` in comb copied to `q`
in seq), every combinational signal is written before it is read in the cycle, and the comb order is derived from
declared read / write sets instead of hand-written call chains. Cycles identical at every commit (run_gate 62,
run_gate_c 67, rv64gc Linux console, FPR fingerprints).

Steps:
1. Safety net: `scripts/comb/comb_graph.py` over the back end, front end and memory subsystem; remove what it finds.
2. Write rules on one pilot module (`Reg<T>` / `Wire<T>`), documented here.
3. Table-driven comb order: each phase declares its reads / writes, the order is generated and checked.
4. Module by module: LSU, ISU, front end, memory subsystem.

## Step 1: the checker (8d804aa)

`python3 scripts/comb/comb_graph.py [--check]` (part of the per-commit check) reports, per domain, a phase that reads
a signal which a later phase of the same cycle writes.

| domain | phases | signals | rule |
| --- | --- | --- | --- |
| back | 69 (`BackTop::comb()`) | the `Wires` fields (sub-field level) | the Wires live across cycles: such a read is a hidden register |
| front | 8 (`front_comb_calc()`) | FrontComb members + the file-scope state of `front_top.cpp` | FrontComb is fresh every call: a read before the write sees the initial value ("staged read", 71, reported only); the file-scope state is persistent |
| mem | 29 (statements of `MemSubsystem::comb_outputs()` / `comb_inputs()`) | MemSubsystem members; a block's `in.F` / `out.F` pointers through the bindings in `MemSubsystem::init()` | members live across cycles |
| top | 201 (`SimCpu::cycle()` up to the first seq call, inlined) | paths from SimCpu (`back.out.lsu2dcache`, `axi_interconnect.read_ports[MASTER_ICACHE].req.ready`, ...) | a read with an earlier write in the same cycle is staged (34, reported); without one it sees last cycle's value: a hazard |

Call arguments: a non-const reference / pointer parameter counts as written (write-only when the callee assigns it
before any other use), `obj.method()` is resolved through the declared type of `obj`; an unresolved callee counts as
a read and is listed as a note.

Found and removed in step 1 (all cycles identical):
* DCache stage 1 -> stage 2: the MSHR / write-buffer lookups (`find_req`, `bypass_req`, `merge_req`), the tag-array
  reads and the fill read that stage 1 issues in cycle t were used at the start of t+1 straight from the live signals.
  They are the register `DcacheS1Reg dcache_s1_q_` now (copied in `dcache_domain_seq`); MSHR / WB read it through
  `in.s1_q`, `Dcache_Read` takes it as its argument.
* MSHR / WB `comb_inputs_axi` read and cleared their own `axi_out` of the previous cycle; dead (`comb_outputs_axi`
  clears it unconditionally before anything reads it), removed.

## Step 1b: the top level (6c6701a)

`scripts/comb/comb_top.py`. SimCpu methods and the free functions `cycle()` calls are inlined statement by statement
(parameters, `auto &` aliases, `for` bodies, `if` / `else` branches; phases in different branches of one `if` are
exclusive); a `*_seq` method is a clock edge and is skipped. A call into a sub-object is one phase with a summary:

| call | summary |
| --- | --- |
| `back.comb()` | the back-end rows (a Wires field F is `back.F`), plus the DTLB walk port |
| `front.comb_bpu()` | the front rows (`in.F` / `out.F` -> `front.in.F` / `front.out.F`), the I-cache ports (icache_top.cpp, the bindings parsed from `SimCpu::init()`), the ITLB walk port |
| `mem_subsystem.comb_outputs()` / `comb_inputs()` | the mem rows of that function; pointer members through `SimCpu::init()` (`lsu2dcache` -> `back.out.lsu2dcache`, ...) |
| `axi_interconnect.*`, `axi_ddr.*`, `axi_mmio.*` | a scan of the class's member function closure |
| a `PtwWalkPort` | `<port>.in` (request / consume: written by the TLB, read by `comb_inputs`), `<port>.out` (response) |

Found and changed (cycles identical, FPR identical):
* the I-cache read the ICACHE read port's outputs and the probe response `icache_resp`, which are written after the
  front end in the cycle (`axi_interconnect.comb_outputs()`, `mem_subsystem.comb_outputs()`): last cycle's values.
  They are the registers `SimCpu::icache_axi_q` / `icache_resp_q` now, loaded in `icache_port_regs_seq()` at the
  start of the cycle; the I-cache drives its requests on the live port and reads the outputs from the registers.
* the walk-port outputs were set in `MemSubsystem::comb_inputs()` and again in `MemSubsystem::seq()` before any TLB
  reads them: the comb copies were dead and are removed; the walk-port outputs are registers loaded at seq.
* `front2back_hold`: the clear of d moved from the start of the comb to seq (after d -> q).
* `front_comb_before_back()`: the save / restore of `front.in.reset` removed (it is 0 whenever `cycle()` runs).

Known, listed in `TOP_KNOWN`: the AXI interconnect reads its own last-cycle downstream AR / AW / W valid and the
write ports' `req.ready` (registers inside the AXI kit, kept in the shared port structs).

Mutation checks: the TLB outputs set again in `comb_inputs`, the I-cache bound to the live probe response, the MSHR
on the live `find_req`: each is reported.

Not covered yet:
* the seq order (a seq function that reads another module's state after that module's seq ran);
* the inside of a block (the cur / nxt state of RealDcache, MSHR, WB, the I-cache, the BPU pipe), the AXI interconnect
  and the LLC;
* conditional writes: the check is about order only. A signal written by an earlier phase only in some cycles keeps
  its old value in the others. Known case: `finish()` (PMU stall cause) reads `g_pipe_txn.pmu_l1_ovr` /
  `pmu_ptab_stall`, which `run_bpu()` writes only when the BPU runs; in a stall cycle the PMU sees the previous
  cycle's flags. PMU attribution only, not cycles; not changed. Finding these needs the `Wire<T>` type (step 2):
  a debug build can check that each wire is written in the cycle before it is read.

## Step 2: fewer comb phases (trace-driven reductions)

The values already come from the trace (`fu::compute` takes the AGU address and the store data from the fact, the PRF
reads no operands), so what is left to remove is structure: same-cycle request / response wire pairs, stages that are
only a one-cycle delay, phases split in two, the cache-line data path. Plan (each item one commit, cycles identical):

| | change | back-end phases |
| --- | --- | --- |
| A | FTQ PC read ports -> a direct read of the FTQ array | 69 -> 65 (f13f6d2) |
| B | the PRF module folded into the EXU | 65 -> 63 |
| C | LSU: the DTLB lookup inside the LSU, ldq / stq phase pairs merged | 63 -> 58 |
| D | CSR read: a direct read of the CSR file | 58 -> 56 |
| E | comb_begin into each module's first phase (comb_pipeline stays in comb) | 56 -> 52 |
| F | the cache-line data path (AXI resp.data / wdata, DCache / LLC / DDR line data) | (top level) |
| G | the DCache response buffering (capture / forward / publish) | (mem) |

A (f13f6d2): `PreIduQueue::ftq_pc(idx, off)` reads the registered `ftq_lookup_entries`; `Prf::comb_read` (ALU pc
operand, BRU) and `Rob::comb_commit` (the head's redirect pc) call it through `const PreIduQueue *ftq`. Removed:
`ftq_prf_pc_req / resp`, `ftq_rob_pc_req / resp` and the phases `Prf::comb_req_ftq`, `Rob::comb_ftq_pc_req`,
`PreIduQueue::comb_ftq_lookup_prf / rob`. Module-to-module wire edges 44 -> 41.

B: the PRF module had no register file and two unrelated jobs: `comb_read` copied `iss2prf` to `prf2exe` in the same
cycle (plus the FTQ pc), and `inst_r` was a one-cycle register of all 21 `exe2prf` entries used only for the late load
wakeup. Now `Exu::comb_pipeline` reads `iss2prf` directly (and fills the pc through the FTQ read port), and the EXU
keeps a write-back register of just what the wakeup uses (`LoadWake`: valid, preg, br_mask; filled in
`Exu::comb_pipeline`, read by `Exu::comb_load_wake` at the old `Prf::comb_awake` position, copied in `Exu::seq`).
Removed: `Prf.cpp` / `Prf.h`, `prf2exe` (`PrfExeIO`). Module-to-module wire edges 41 -> 36.

C: `lsu.comb_tlb_out` -> `top.comb_lsu_mmu` (DTLB) -> `lsu.comb_tlb_in_ldq` -> `lsu.comb_tlb_in_stq` was a same-cycle
request / response pair across the BackTop boundary; it is one phase `RealLsu::comb_tlb()` now, which calls the DTLB
directly (`TlbMmu *dtlb`; a fence flushes it, a flush / mispredict cancels the pending walk). `lsu2mmu_io` /
`mmu2lsu_io` left the Wires and are LSU members (`tlb_req_` / `tlb_resp_`, still printed by `LSU_PIPE_EVT`, whose
output is byte-identical). The adjacent ldq / stq halves are one phase each: `comb_dcache2lsu()`, `comb_lsu2dcache()`
(the halves stay as private helpers). LSU phases 19 -> 15.

D: `exu.comb_to_csr` -> `csr.comb_csr_read` -> `exu.comb_exec` were three adjacent steps of one CSR read. The CSR
file is real model state (counters, interrupts), so the read stays, but `Csr::read(idx)` reads the registered
`CSR_RegFile` directly (nothing writes it in comb) and `comb_exec` starts with the request (`to_csr`, which still
drives `exe2csr` for the write at commit). Removed: `csr2exe` (`CsrExeIO`), `Csr::comb_csr_read`, the phase
`exu.comb_to_csr`.

E: each `comb_begin` (default outputs, working copies `x_1 = x`) is now `begin_cycle()`, called at the top of the
module's first phase of the cycle (`PreIduQueue::comb_accept_front`, `Exu::comb_load_wake`, `Isu::comb_ready`,
`Rob::comb_ready`); the checker showed no phase between the old and the new position reads what it writes, and the
gates are identical. `BackTop::comb_begin_all` and the empty `clear_stage_ios` are gone. The planned move of
`comb_pipeline` into seq was dropped: it computes the next value of the pipeline registers (`d = f(wires)`), i.e.
their input logic, which belongs to comb; seq stays a pure copy.
