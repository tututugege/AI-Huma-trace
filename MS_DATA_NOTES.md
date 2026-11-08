# EXP-020: value path of the memory-speculation window (trace-model branch exp/ms-data, from exp/memspec-refs 40ed5bc)

Goal: model cycles = RTL cycles with `LSU_MEMSPEC=1`, without touching the microarchitecture or the RTL. The model is oracle driven (values are facts
of the trace), the RTL executes real data: a load that passed an older store with unknown address reads the stale byte, its consumers compute with
it, a dependent branch can resolve the wrong way. This branch adds that value path behind `LSU_MS_DATA` (default 0: every cycle count unchanged).

## 1. Diagnosis (Step 0): what the dhrystone / vm_pagefault differences are

Evidence: `dhrystone` RTL 115,896 vs model 115,830; `vm_pagefault_test` RTL 19,271 vs model 19,270; everything else of the 68 programs is equal.

* The first dhrystone difference (+1 at the commit of the violating load at pc 0x3ac, cycle 3565) is the `bne a0,x0` at pc 0x800003f8 after
  `lbu a0,55(sp)` that read the stale byte 0 (true 0x44): the RTL branch resolves not-taken on 0, the prediction was taken -> an RTL-only mispredict.
* The later "wandering" of the per-commit delta (2894 delta events, jumps of +11 / -12 cycles, +93 at 50% of the run, +67 at the end) is **the same
  event repeated**: with the value path switched on the model finds exactly **27 flips in dhrystone, all of the same static branch (pc 0x800003f8,
  `bne a0,x0` on a stale `lbu`, chain length 1)**, at model cycles 3551, 6181, 7619, 10278, 70123 ... 114539. Applying them as RTL-only mispredict redirects
  makes the per-commit cycle series identical to the RTL over the whole run (291,710 commits compared, 0 delta events; before: 2894 events). The
  cumulative +66 cycles are 27 flips with an average cost of 2.4 cycles (a flip costs 1..2 cycles at the violating load and a perturbed
  refetch / BPU state afterwards, hence the large transient deltas).
* vm_pagefault_test: one flip of the same branch (cycle 15628, `bne` on the stale `lbu` at pc 0x800003f4) = +1; per-commit identical afterwards (37,239 commits).
* No other class of difference was found: no model/RTL mismatch in the speculative window itself (SPD / release / violation / second write-back events
  were already equal), no stale-address, store-data or exception effect needed for the equality (see section 4 for how often those happen).

## 2. What the model does now (LSU_MS_DATA, `back-end/include/MsData.h`, hooks in `RealLsuPipe.cpp` / `Fu.h` / `TraceBridge`)

Switch: `CFG_LSU_MS_DATA` / env `LSU_MS_DATA` (needs `LSU_MEMSPEC=1`). `1` = branches + store data; otherwise a bit mask: 1 branches resolve on stale
values (RTL-only mispredict redirect), 2 stale addresses of loads/stores are used, 4 stale store data goes into the STQ, 8 self checks (debug).
`LSU_MS_DTRACE=1` prints every stale load / flip. `LSU_PIPE_STATS=1` prints the `[MS_DATA]` lines.

1. **Stale value of a load** (`ms_load_value`, evaluated at the first, deferred write-back of a speculative load):
   * the memory image = the word before the oldest not-yet-drained STQ store that writes the word (`Fact::mem_old`, new; the cache content), or the core RAM
     when no STQ store writes it;
   * then the older STQ stores that were *visible* at the load's last L1 CAM are applied in age order, the youngest wins (what STLF does). Not visible =
     the unknown-address stores the CAM passed (`PLdq::skip`, bitmask recorded by `stq_cam`); stores that were known at the CAM are visible, with the data they carry
     (tainted store data included);
   * sub-word / partial overlap is exact (byte lanes), the word is extracted with the load's func3.
   * Self check (mode bit 8): the same function with every store visible must give the fact value for every non-speculative load and every second pass:
     **3.4M loads checked on perlbench sp3, 0 mismatches; dhrystone / vm tests 0**. A speculative load whose value differs from the truth always has `vio`
     set by its second write-back (`stale_without_vio=0` everywhere), i.e. the violation CAM and the value logic agree.
2. **Taint**: a table over physical registers (value, chain depth). Set at the first write-back of a stale load, cleared at its second write-back
   and at every ROB flush, overwritten by any clean producer. A consumer is evaluated in `Fu::compute` (the stage where the facts are applied) with
   its real ISA semantics (`MsData.h` RV32IMAB evaluator: base, M, Zba/Zbb/Zbc/Zbs; self check on all 95k ALU/branch uops of dhrystone: 0 bad) on the
   tainted inputs, the other source comes from the fact (`Fact::rs1_value`, new). A result equal to the true value drops the taint (masked).
3. **Branch** on tainted data: direction / jalr target recomputed; the usual `mispred` / `br_taken` / `diag_val` are set from the stale outcome and the
   prediction (a flip that agrees with the prediction suppresses the redirect, a flip that disagrees causes one to the stale target); the existing redirect path does
   the rest (bridge rewind, forced jump, BPU repair). Everything dies at the commit flush of the violating load.
4. **Store data** tainted -> the STQ data (forwarding to later loads uses it). **Addresses** (bit 2): a load / store whose base register is stale uses
   the stale address (TLB, cache, STQ CAM, STA broadcast); a stale-address load returns the memory value at that address and stays tainted;
   a stale-address store is not an overlay source for loads of the true address.
5. Statistics (`LSU_PIPE_STATS=1`): spec_wb, stale (vio), silent (vio), stale_without_vio, consumers, masked, unmodelled, branches / flips / flip to
   mispredict / flip to correct / jalr flips, tainted ld_addr / st_addr / st_data, chain length histogram of differing results and of branch flips.

## 3. Validation

| check | result |
| --- | --- |
| default (`LSU_MS_DATA=0`) vs `scripts/ref_cycles_memspec.txt`, 68 programs | 68/68 identical |
| `LSU_MS_DATA=1` and `=7` on the 68 programs | 66 unchanged, `dhrystone` 115,896 and `vm_pagefault_test` 19,271 = the RTL values; `vm_asid_test` 41,097 = RTL |
| per-commit cycles, whole program (dhrystone 291,710 commits, vm_pagefault 37,239) | 0 delta events (before: 2894 / 1) |

A first version tested `src2_is_imm`, which is true by default for branches and stores, and so missed the second source of `beq`; `vm_asid_test` (41,096) caught it.

## 4. 30 SPEC points

`PF_ENABLE=1 PMU=1 FASTWARM=1000000 LSU_MEMSPEC=1 LSU_MS_DATA=1` (mode 5), `points_p30_local.txt`, results `~/personal/AI-Huma-data/memspec/msdata1`.
Geomean IPC vs the oracle MEMSPEC=1 run (`final`): **-0.085%**; speedup over MEMSPEC=0 (`base`): **+5.30% -> +5.21%**, i.e. the oracle model overstates the gain by 0.09 point.
Per point: range -0.93% (gobmk sp23, perlbench sp5 -0.92%) .. +0.35% (mcf sp16); 14 of 30 points within +-0.05%. Some points get faster (a stale branch resolves earlier / state perturbation).
Channel counts over the 30 points (13.3M speculative first write-backs): stale values 12,234 (0.09%; the rest are silent: stale == true), consumers 35,397
(6,074 masked), branches on tainted data 6,807 of which 1,190 flip (1,175 add an RTL-only mispredict, 15 remove a true one) + 42 jalr target flips; chain length of
differing results 1/2/3/4/5+ = 11,203 / 1,336 / 698 / 167 / 39; branch flips by chain length 1 / 2 = 994 / 238 (none longer);
tainted store data 4,666, tainted store address 9,276, tainted load address 706 (these three are counted; store data is applied, addresses are not in mode 5);
unmodelled consumers 0. 474 stale loads reached their second write-back without `vio`: they got tainted store data by forwarding (a wrong-path effect, not a missed violation).
Not run: mode 7 (stale addresses applied) on the 30 points; it equals mode 5 on the 68 programs.

## Not modelled
* Loads / stores with a stale address: the cache / TLB / STQ effect is applied (bit 2) but there is no validation against the RTL for these cases (no program of the 68 reaches
  one that matters); the page fault / MMIO behaviour of a garbage address is whatever the model's LSU does with it.
* Consumers that are CSR, FP, AMO (counted as `unmodelled`: none seen).
* Stale values of wrong-path loads are computed with the function model state of the wrong path.
* The exact cycle of the read of the source operand (taint is read at the FU entry, one stage after the register read): a violating load whose second write-back falls
  in between could be seen one cycle early / late; no instance found in the 68 programs.
