# EXP-LSUTIM: where to put one more pipeline stage in the LSU (timing study)

Branch `exp/tlb-ports` (trace-model) on top of the shared-AGU / two-INT-queue design (docs/TLB_PORTS.md, `CFG_INT_CLUSTERS=2`).
RTL: worktree `rtl-agu`, branch `exp/agu-share`. Depth numbers are the yosys generic-gate proxy (`scripts/synth_depth.sh`,
`SYNTH_DEPTH=1`, longest topological path, flip-flops excluded): relative, not timing.

## 1. Where the time goes now

`back_top` (whole back end, 4.67 M cells): **234 levels**. The path, with the level at which each point is reached:

| levels | segment | what it is |
| --- | --- | --- |
| 0 - 35 | ROB commit -> trap -> `rob_flush` | the global flush, 35 levels from the ROB commit ports |
| 35 - 90 | `sa_acc` / `ag_acc` -> `free_l` -> `rp_ok` chain -> `lsu2mmu_req_vaddr` | the LSU accepts a new AGU request only if it is alive (`!flush`, LDQ / STQ entry matches); the replay registers then take the free load slots and DTLB ports (a chain over 4 replay registers) |
| 90 - 139 | DTLB (`tlb_mmu_core`, after EXP-042) | 49 levels |
| 139 - 167 | `mmu2lsu_resp` -> `tl_s` -> STQ `paddr8` -> `sta_info` | the STA translation updates the STQ entry and the broadcast |
| 167 - 234 | `u_wk.inf` -> `conf` -> `o` -> LDQ `ew` | wake-up of the loads parked on the store (STLF confirm against the updated store): 67 levels |

`lsu` alone (AGU and DTLB response are free inputs): 164 levels, 1.67 M cells, same shape (acceptance -> replay allocation -> STQ update -> wake confirm).
The AGU itself (issue-slot register -> operand read / bypass -> add) was ~64 levels in the earlier whole-core path and comes before the DTLB request when the
path starts at an AGU result instead of at the flush.

Important: the longest path does **not** start at the AGU but at the ROB flush, through the *acceptance check* of the new requests. A register
between the AGU and the LSU alone would not shorten it.

## 2. Candidate cuts, measured in the model (16 memory points, 4 M instructions, IPC relative to the same design without the cut)

Switches (all default 0, default cycles unchanged: 67/67 equal to `ref_cycles.txt`; the combined config runs all 67 programs, +1.05 % cycles geomean on them):

| switch | cut | IPC (16 pts, geomean) | worst point |
| --- | --- | --- | --- |
| `CFG_LSU_FREEPORT_RAW=1` | the replay DTLB-port allocation looks at the *valid* AGU requests, not at their acceptance | **1.0000** (cycles equal on all 16) | 1.000 |
| `CFG_LSU_STAEV_REG=1` | the STA / STD events that wake loads parked on a store are registered (wake one cycle later) | 0.9927 | 401.bzip2.3 0.935 |
| `CFG_LSU_AGU_REG=1` | a register between the AGU and L1 (DTLB + CAM): every load / STA reaches the LSU one cycle later | 0.9760 | 401.bzip2.3 0.937 |
| AGU_REG + STAEV_REG | both | 0.9725 | 401.bzip2.3 0.893 |
| FREEPORT_RAW + STAEV_REG | | 0.9927 | |

FREEPORT_RAW changes the model only for the port mask (slots are still counted by accepted requests); the RTL version has to apply the same rule to the
load-slot count and the compress of the new loads (`nnew`, `pre[]`, `sr_t`), otherwise the acceptance still feeds the replay chain. The clean form is
**slot k = AGU port k, no compress**: new load of AGU k in slot k, a replay takes the j-th free slot / port. That is the intended next step; it is not
implemented in RTL and not yet in the model (the model switch above only covers the DTLB port mask, which is enough for the IPC question: the cycles do not
change).

## 3. Recommendation (what to cut, in this order)

1. **FREEPORT_RAW (no IPC cost)**: removes the 0 - 35 flush segment and the acceptance from the replay allocation. The path then starts at the AGU result:
   AGU ~64 + vaddr mux ~8 + DTLB 49 -> TLB result ~121, STA update -> `sta_info` ~149, wake confirm -> ~216.
2. **STAEV_REG (-0.7 %)**: register the STQ address / data events before the LDQ wake-up confirm. Removes the 67-level segment: the longest path becomes
   AGU + DTLB + STQ update ~150.
3. **AGU_REG (-2.4 %)** only if the target is < 100: it splits [AGU 64] | [mux + DTLB + STQ update ~85]. Together with 1 and 2 the LSU chain is ~85 - 90 levels
   at a total IPC cost of about -2.7 % (measured together: 0.9725).

Not chosen: registering the DTLB response (cost like AGU_REG for loads because the CAM + DCache request move one cycle later, and the 0 - 90 segment would still be
there without FREEPORT_RAW).

## 4. Not done / caveats

* The RTL of the three cuts is not written. The numbers in section 3 are the sum of segments read from one synthesis path, not a resynthesis: the next-longest
  paths (other LSU chains, ISU 109 levels, EXU) are unknown until a cut is in.
* The IPC numbers are the model's, 16 points (the same points as docs/TLB_PORTS.md), the AGU_REG model also delays STA translation by one cycle.
* `back_top` synthesis needs `scripts/synth_module.sh back_top 26000 5400` (the `synth_depth.sh` memory limit of 12 GB kills it).

## 5. Result of the zero-cost cut (`CFG_LSU_SLOT_FIXED`, RTL `lp_l1`, rtl-agu 'lp_l1: L1 slot k is AGU port k')

L1 slot k is AGU port k (no compress by acceptance); the replays take the free slots and DTLB ports of the AGUs without a valid request. Model: cycles equal on
all 67 programs (reference `scripts/ref_cycles_memiq6_nr_int2.txt` unchanged); RTL `fm_gate` 67/67 with the same cycles, `lint_chip` 0.

Synthesis (same flow as section 1): `lsu` 164 -> 157 levels, `back_top` **234 -> 223** levels. Only -11: the flush segment (35) and the acceptance chain are gone,
but the path now starts at the EXU and is longer there than estimated:

| levels | segment |
| --- | --- |
| 0 - 96 | EXU issue-slot register -> operand read / bypass -> AGU add -> `exe2lsu_ldu_req_uop_result` (**96**, not the 64 assumed in section 1) |
| 96 - 127 | vaddr mux (5) + DTLB (~26 on this path) |
| 127 - 154 | `mmu2lsu_resp` -> `tl_s` -> STQ `paddr8` -> `sta_info` |
| 154 - 223 | LDQ wake-up confirm (`inf` -> `conf` -> `o` -> `ew`), 69 levels |

Consequences for the remaining cuts: `CFG_LSU_STAEV_REG` (-0.7 %) takes the 69-level wake segment off this path (-> ~154); after that the AGU add itself (96) is the longest
segment, so `CFG_LSU_AGU_REG` (-2.4 %) only splits [96] | [58 + the rest] and cannot go below ~96 unless the EXU stage (operand read / bypass / add) is cut as well.
