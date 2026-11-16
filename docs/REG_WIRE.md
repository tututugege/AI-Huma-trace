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
| mem | 32 (statements of `MemSubsystem::comb_outputs()` / `comb_inputs()`) | MemSubsystem members; a block's `in.F` / `out.F` pointers through the bindings in `MemSubsystem::init()` | members live across cycles |

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

Not covered yet:
* the top level (`SimCpu::cycle()`): the order front end -> AXI phase 1 -> memory outputs -> back end -> memory inputs
  -> AXI phase 2 -> back-to-front, and the signals between the domains (`front.in/out`, `back.in/out`, the AXI ports);
* the inside of a block (the cur / nxt state of RealDcache, MSHR, WB, the I-cache, the BPU pipe), the AXI interconnect
  and the LLC;
* conditional writes: the check is about order only. A signal written by an earlier phase only in some cycles keeps
  its old value in the others. Known case: `finish()` (PMU stall cause) reads `g_pipe_txn.pmu_l1_ovr` /
  `pmu_ptab_stall`, which `run_bpu()` writes only when the BPU runs; in a stall cycle the PMU sees the previous
  cycle's flags. PMU attribution only, not cycles; not changed. Finding these needs the `Wire<T>` type (step 2):
  a debug build can check that each wire is written in the cycle before it is read.
