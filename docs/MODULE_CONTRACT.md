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
Not yet under the contract: Exu's FU state (`units`, updated in place in comb), BackTop setting `csr->fs_dirty_req`
directly (a cross-module write; should be a port), Rob, Isu, the LSU. Gates RVC=0 62/62, RVC=1 67/67 identical, FPR
identical at every commit.
