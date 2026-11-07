# LSU_PIPE: staged load/store unit prototype (trace-model; experiment branch exp/lsu-pipe, merged into main as lsu-pipe-main, EXP-014)

Purpose: the C++ model is the prototype of the RTL. `back-end/Lsu/RealLsu.cpp` (legacy) is queue-centric; its RTL translation
(`rtl/docs/modules/lsu.md`) is one combinational chain per cycle. This document (1) records the analysis of the legacy
design (Step 0), (2) specifies the staged replacement implemented in `back-end/Lsu/RealLsuPipe.cpp` /
`include/RealLsuPipe.h`, and (3) lists the model approximations. The RTL is to be written from sections 2-4.

Switch: `CFG_LSU_PIPE` (config.h, default 1 in main since 2026-11-07; env `LSU_PIPE=0` selects the legacy LSU for regression; cut masks `CFG_LSU_PIPE_CUT` 127 (was 63 before section 11: + bit 64 FM) and `CFG_LSU_PIPE_L2` 115, env `LSU_PIPE_CUT` / `LSU_PIPE_L2`; since section 11 also `CFG_LSU_PIPE_DRAIN` 3 / `CFG_LSU_PIPE_DCAND` 4 / `CFG_LSU_PIPE_RP` 4, env `LSU_PIPE_DRAIN` / `LSU_PIPE_DCAND` / `LSU_PIPE_RP`). The text below was written while the switch was off by default; "default OFF / cycle-identical to main" statements describe the experiment branch. Other env knobs (all runtime, defaults in brackets): `LSU_PIPE_RP` replay registers 1..4 [4 since section 11; was 1..2 [2]],
`LSU_PIPE_GATE` DCache read enable qualified by the partial CAM [1], `LSU_PIPE_BOUNCE=n` aging bounce of a new load by a replay register
that waited n cycles [0 = off], `LSU_PIPE_STATS=1` prints the counters at exit, `LSU_PIPE_DUMP=<cycle>` debug dump.
Port counts come from `config.h` (`CFG_LD_PORTS` 4, `CFG_STA_PORTS` 3, `CFG_STD_PORTS` 3, LDQ 64, STQ 64).

---------------------------------------------------------------------------------------------------------------------

## 1. Step 0: analysis of the legacy LSU (RealLsu.cpp, LD 4 / STA 3 / STD 3, LDQ 64, STQ 64)

### 1.1 Per-cycle stage order (BackTop::comb; every `comb_*` works on the same `nxt`, so a later stage sees earlier writes)

```
comb_cal / comb_lsu2dis / comb_lsu2rob     cycle-start views (counts over 64 LDQ / STQ entries)
comb_stq_commit      B  ROB commit -> WaitCommit -> Committed (8 commit ports in order)
comb_mmio_in         C  MMIO response -> finish
comb_dcache2lsu_ldq  D  DCache load response (HIT -> finish, else replay + countdown), MSHR-fill wake (window 20), finish push (window 20)
comb_dcache2lsu_stq  E  DCache store response, MSHR-fill wake (window = committed count <= 64)
comb_lsu2exe         F  write-back: mmu_done window (15) -> STA/SC ports, finish window (20) -> load ports
 -- EXU executes, produces exe2lsu --
comb_exe2lsu         G  STD (4 ports), STA, LOAD -> LDQ/STQ entry + push wait_mmu / mmu_done
comb_tlb_out         H  wait_mmu_ldq window 20 -> first 4 WaitTlb -> 4 MMU requests; wait_mmu_stq window 15 -> first 3
 -- MMU lookup (combinational, same cycle) --
comb_tlb_in_ldq/stq  I  4+3 responses (HIT/FAULT/MISS) + window scan: WaitSTLF -> stlf_queue push (<=4), WaitFinish -> finish
comb_stlf            J  STLF window 20 with `break` chain -> wait_dcache / finish / MMIO unit / retry requeue
comb_dis2lsu, comb_flush   alloc (8 LDQ / 8 STQ), flush / mispredict / clear_mask over all queues
comb_lsu2dcache_ldq  M  wait_dcache window 20 -> first 4 ReadyToIssue (replay countdown) -> 4 DCache requests
comb_lsu2dcache_stq  N  store window 15, pairwise older-conflict check, <=3 DCache store requests
comb_mmio_out / comb_check   MMIO request; retire Done stores (window 20)
```
So a load goes AGU request (G) -> wait_mmu window (H) -> TLB -> (I) -> stlf window (J) -> wait_dcache window (M) -> DCache request in
ONE cycle. The response (D) and the write-back (F) happen in the next cycle: AGU->write-back = 2 cycles, load-to-use 4
(`lat_ptr` 4.0), STLF 5 (`lat_stlf` 5.0).

### 1.2 Queues and windows

| structure | entries | per-cycle structure |
| --- | --- | --- |
| wait_mmu_ldq, stlf_queue, finish, wait_dcache_ldq (rings of {valid, idx[6]} + head/count; wait_dcache also req_gen[25], replay_wait[3]) | 64 each | window 20 (LD x 5), <= 4 consumers, holes occupy slots until they reach the head, `trim` pops head holes |
| wait_mmu_stq, mmu_done_stq (rings) | 64 each | window 15 (STA x 5) |
| LDQ entries | 64 | 14-state FSM, vaddr/paddr/result, stq snapshot (idx+flag), br_mask |
| STQ entries | 64 | 14-state FSM, vaddr/paddr/data, replay_wait |
| uncached unit, LR/SC unit | 1 each | |

### 1.3 CAM / priority structures (entries x compare width)

| structure | size | note |
| --- | --- | --- |
| STLF check | 20 loads x up-to-64 STQ scan, per pair a 32-bit address range compare (store covers / disjoint / partial: 2 x 32-bit magnitude compares on addr+width) | scan goes from the youngest older store down with `break`; per load a 64-step priority chain; the 20 loads themselves form a second `break` chain (resource counters wait_dcache / finish / uncached) |
| store window older-conflict | 15 x 14/2 = 105 pairs of CheckAddr (30-bit word compare + 4-bit strobe overlap) | `blocked` is a chain along the window |
| MSHR-fill wake (loads) | 20 x 26-bit line compare + "first 4" prefix count for MSHR_FULL | |
| MSHR-fill wake (stores) | up to 64 x 26-bit line compare | |
| window selects | wait_mmu_ldq 4-of-20, wait_mmu_stq 3-of-15, wait_dcache 4-of-20 (with replay countdown), finish 4-of-20, mmu_done 3-of-15 | prefix counts (`issued < N`) |
| response match | 4 DCache load responses by `req_id` into wait_dcache slot (+25-bit generation compare); 4 MMU responses by {wait_idx, entry_idx} | |
| MMIO / commit | store commit: 8 ports in order against `stq_commit`; retire: first run of Done stores in a 20 window | |
| allocation | lowest-8-empty of 64 LDQ entries; STQ tail + rank | |
| flush | 64 LDQ br_mask compares, STQ first-match scan, 6 ring `drop_dead` scans, clear_mask on both queues | |

### 1.4 Longest combinational dependency chain

`exe2lsu push (G)` -> `wait_mmu window select 4-of-20 (H)` -> `DTLB CAM + permission (MMU)` -> `response match + state write (I)` ->
`window scan of wait_mmu pushing stlf_queue (I)` -> `STLF window of 20 loads, each a 64-step older-store break chain, then
resource counters (J)` -> `wait_dcache push` -> `wait_dcache window 4-of-20 with replay countdown (M)` -> `DCache request address`.
The J stage is the dominant cut: a 20-wide `break` chain over loads multiplied by a 64-wide chain over stores, built from 32-bit
range compares. The pure queue bookkeeping (rings, holes, trim, window prefix counts, `finish`, `wait_dcache` generation tags)
adds a long tail of prefix logic around it. The next biggest is the store window pairwise check.

What was cut: all six rings and their windows (replaced by per-entry state + 1-2 replay ports), the STLF window (replaced by one
partial-address CAM per load port), the load-side window scans, the store-side window scans (replaced by oldest-first picks), the
combined AGU->DCache-request chain (cut by the L1/L2 register).

---------------------------------------------------------------------------------------------------------------------

## 2. Staged design (the specification for RTL)

### 2.1 Conventions

* `pcur` = registers, `pnxt` = next values; a phase reads registers and per-cycle wires (`pw`) and writes `pnxt` / `pw`; `seq()` copies.
* Widths: every field is declared in `RealLsuPipe.h` (`wire<N>`). Widths of indices: LDQ/STQ 6, STQ pointers 6+flag, counters 7.
* Port limits (checked at run time, abort when exceeded in a cycle): AGU load ports `LD`=4, STA ports `STA`=3, STD ports 3,
  replay ports `RP` 2, STA replay register 1, store drain ports `DRAIN_PORTS` = min(2, STA), STQ retire ports 4, STA write-back
  `STA`, load write-back `LD`, SC write-back shares load ports. No loops over data-dependent counts: all selects are
  find-first / youngest-first priority over a fixed 64-bit vector.
* Time naming: `t` = cycle in which the AGU result of a load/STA is visible (exe2lsu). The EXU AGU is the stage before L1.

### 2.2 Load pipeline (per load port p, 4 ports)

```
 AGU result (cycle t)           replay register RP[r]
        \                        /
         +--------- L1 ---------+     cycle t:  DTLB lookup (skipped when paddr is known)
                                              + partial STLF CAM over the STQ (page offset [11:0] + size, STA bypass)
                                              + DCache read request (enable = TLB hit && !MMIO && no CAM match)
               |  L1/L2 register PL12[p] {v, ldq, epoch, tlb, mmio, cand_v, cand}
         +--------- L2 ---------+     cycle t+1: DCache response (tag/data select) || STLF confirm (full paddr) -> hit/forward select
                                              -> write-back request on load port p   (the EXU/PRF write-back register is L3, wake-up L4)
```
Mapping to the requested names: L1 = AGU+DTLB, L2 = DCache tag/data read + STLF compare, L3 = hit/forward select,
L4 = write-back + wakeup. The model keeps the existing DCache interface (request in cycle t is the S1 array read, response in
t+1 is S2 tag compare + select), so the cache read of "L2" overlaps the model's L1 and the select of "L3" shares the cycle
with the L2 compare; the write-back is driven in the same cycle t+1 and the PRF write-back register / wake-up are the
remaining stages. This is exactly the observable timing of the legacy design (L1D hit: AGU t -> write-back t+1).

#### L1 (cycle t, combinational: `comb_exe2lsu_p`, `comb_tlb_out_p`, `comb_tlb_in_ldq_p`, `comb_stlf_p`, `comb_lsu2dcache_ldq_p`)

* Input per port: a new load (LDQ idx, vaddr from AGU; entry written: vaddr, func3, dest, stq snapshot {idx,flag}, LR flag,
  state PIPE, paddr invalid) or, when fewer than 4 new loads arrived, an RP register (replay). Ports are filled in order: new
  loads first, then replays.
* DTLB: request {valid, vaddr} only when the entry has no translated paddr. Response HIT -> `pa_v`, paddr, `mmio` (address map)
  written to the LDQ entry; PAGE_FAULT -> `pf`, result = vaddr, LR reservation cleared; MISS or no response -> `tlb=1`.
* Partial CAM (`stq_cam`): for each of the 64 STQ entries j (64 parallel comparators):
  `older[j] = rank(j) < dist(head tag -> load snapshot tag)` (rank = (j-head) mod 64, 2N tag ring, distance clipped to `stq_count`);
  `cand[j] = older[j] && !suppress[j] && (!va_v[j] || overlap12(load, store))` where `overlap12` compares the 12-bit page offsets with
  sizes (ranges [off, off+w) on 13 bits: not disjoint = cover or partial). Entries whose STA arrives in this very cycle bypass
  their vaddr/size into the compare (4x3 comparators against the STA ports). For a replay with a known paddr (`full`) a store with
  `pa_v` is compared on the full 32-bit physical address instead. Result: the youngest candidate (`max rank`): `cand_v`, `cand` (6 bits).
  Registers only (pcur STQ); same-cycle STD data is not bypassed (L2 re-reads it).
* DCache read enable = `tlb hit && !mmio && !cand_v` (the 12-bit CAM is ready with the TLB result). A request carries
  `req_id = {epoch[4], ldq[6]}`; `epoch` (per LDQ entry, never reset, incremented at allocation and at every launch) is the stale
  response filter that replaces the 25-bit `req_gen` and the wait_dcache slot table. Variant `LSU_PIPE_GATE=0` reads the cache
  regardless of the CAM (the response of a forwarded load is ignored).
* MMIO loads: `L_MMW` state; each cycle the (single) entry that is the ROB's oldest unfinished instruction, whose stq snapshot
  equals the STQ head (no older store left), is sent when the uncached unit is free (`comb_stlf_p`). MMIO loads never go to the DCache.
* Writes PL12[p].

#### L2 (cycle t+1, `comb_dcache2lsu_ldq_p`; reads PL12, pcur STQ, DCache responses)

Per port p with PL12[p].v and LDQ entry still `L_PIPE` with the same epoch (else dropped):

| condition | action |
| --- | --- |
| tlb == page fault | direct write-back with `page_fault_load` (result = vaddr) |
| tlb == miss | park: cause TLB, wait 0 |
| mmio | state MMW |
| !cand_v | state DCW (DCache response of this cycle, same port, is handled below) |
| cand_v | STLF confirm of the candidate store j (registered STQ entry, **full physical address**): store gone/suppressed or disjoint (false partial match, other page) -> RDY (replay, the full compare then excludes it); address missing (no va or no pa and offsets not provably disjoint) -> park ADDR(j); partial overlap -> park HOLD(j); cover and data missing -> park DATA(j); cover, data present, not an undecided SC -> forward: direct write-back on port p with the STQ data |

Then for every DCache load response (port p, `req_id`): matches an entry in `DCW` with the same epoch, else ignored. HIT ->
direct write-back on port p (if port p already carries an L2 forward, state WBP instead; not reachable with the DCache's retry
rules); CONFLICT / MSHR_HIT / MSHR_FULL -> park DC with wait 0 / 4 / 4 (`LSU_PEPLAY_WAIT_CYCLES`), `miss` flag for TMA.
Late responses of DCache-internal bank-conflict retries arrive on the original port p in a cycle in which L2[p] is empty
(DCache retries only when the LSU did not send on that port), so they never collide with an L2 write-back.

#### Write-back (`comb_lsu2exe_p`)

Load port p: 1. direct L2/response completion of its own pipeline; 2. SC results (free ports, ascending); 3. `WBP` entries
(lowest LDQ index first; forwards released from a data wait, MMIO data). The LDQ entry is freed at write-back; LR sets the
reservation here (address = paddr). STA completions: oldest-first picks over `wbp` (3 ports), SC through a free load port.

### 2.3 Replay (replaces wait_mmu_ldq / mmu_done / finish / stlf / wait_dcache rings)

A load that cannot complete is *parked in its LDQ entry* (state RPW + cause + blk_idx + wait counter); it is never queued.

| cause | wake condition | on wake |
| --- | --- | --- |
| TLB | counter (0) | RDY |
| ADDR(j) | STA event on STQ entry j (arrival, translation done) or j retired | confirm against j at once (same decision as L2): disjoint -> RDY; partial -> HOLD; no data -> DATA; forward -> WBP; still unknown -> stay |
| HOLD(j) | j retired or SC decision on j | RDY |
| DATA(j) | STD write into j | WBP (forwarded straight to write-back; the match was already decided at L2) |
| DC | counter 4 -> 0, or MSHR fill: MSHR_HIT && line match, or MSHR_FULL (first `LD` entries) | RDY |

STQ events are 64-bit vectors built in the cycle (`ev_sta`, `ev_data`, `ev_ret`, `ev_sc`) and applied by one wake pass at the end of
the cycle, which also sees loads parked in the same cycle (no lost wake-up); events of the L1 cycle are covered by the L2 re-read of
the registered STQ entry. RDY entries: the replay ports (RP=2) each hold a register `PRp {v, ldq}` filled by a find-first over the
64-bit RDY vector (including this cycle's wake-ups) and drained into an L1 slot of the next cycle that has no new AGU load
(re-enters L1; paddr known -> DTLB skipped). Replays have no priority over new loads (the issue queue cannot be stalled).

### 2.4 Store side

STQ entry bits: `va_v, pa_v, data_v, pf, mmio, lrsc, sc_pass, suppress, trp, wbp, wbd, cmt, iss, mmw, done` + vaddr/paddr/data/func3/dest/rob/br_mask/replay type+wait.
Pointers: head+flag, count, ccount (committed, not retired), commit pointer.

* STD (3 ports): write data, `data_v`; event `ev_data`; `wbp` when paddr known.
* STA = AGU + DTLB (S1, same cycle t): `va_v`, vaddr, func3, SC flag written at arrival (the partial address is visible to the CAM bypass
  immediately); HIT -> `pa_v`, paddr, `mmio` (and `wbp` if data present); PAGE_FAULT -> `pf`, `wbp` (completes without data); MISS ->
  `trp` and the single STA replay register (find-first oldest) re-enters on a free STA port. Event `ev_sta`.
* STA completion to the ROB (`sta_wb_req`, 3 ports, oldest first) sets `wbd`; the ROB commit then sets `cmt` (in order, against the commit pointer).
  Flushed or failed-SC stores become `suppress+done`.
* Memory-order check at STA arrival: CAM over the 64 LDQ entries, partial address: a younger load (store rank < load's older-count)
  in `DCW`/`WBP` (already read the cache / forwarded) with overlapping page offset is a violation -> replay (`RDY`) and a counter.
  With the default (non-speculative) load policy no load passes an unknown-address older store, so the counter must stay 0. A
  flush-from-load channel to the ROB does not exist in the model; see approximations.
* Drain (`comb_lsu2dcache_stq_p`): over the committed, not done, not in flight stores oldest-first (rotate by head), at most
  `DRAIN_PORTS`=2 per cycle. A store is blocked while an older committed, not done store overlaps its bytes (CAM over the STQ:
  30-bit word + 4-bit strobe; this replaces the 105 pairwise compares by a 2-port x 64 CAM); non-overlapping stores of the same line
  may go in the same cycle (counted as `merge_pair`). A store answered this cycle is not re-sent in the same cycle. MMIO store:
  only the STQ head, uncached unit free. `suppress` stores become done without a DCache access. DCache response HIT -> `done`;
  otherwise wait (0 / 4 cycles, MSHR fill wake as for loads) and re-send.
* Retire: up to 4 `done` stores from the head per cycle; event `ev_ret`.

### 2.5 Flush / mispredict / clear_mask

`flush`: every LDQ entry -> EMPTY, STQ keeps committed entries (count = ccount), replay registers of dead entries cleared, a committed MMIO
store on the bus survives. `mispred`: LDQ entries with `br_mask & mask` -> EMPTY; STQ truncated at the first entry hitting the mask;
uncached unit, LR reservation as legacy; replay registers of dead entries cleared. `clear_mask`: bits cleared in every live LDQ/STQ
entry and the reservation. L1 results of a killed load are dropped when the L1/L2 register is written (entry no longer `PIPE`);
stale DCache responses die on the epoch check. PL12 is rewritten every cycle.

### 2.6 Preserved behaviour

MMIO/uncached loads and stores at the head only; LR/SC reservation (LR sets it at write-back, SC reads the registered value, any SC
and any page fault clears it); AMO = LOAD + STA + STD micro-ops unchanged; page-fault reporting through `page_fault_load/store`;
DCache / MSHR / prefetch / MMU interfaces unchanged (no change in `MemSubSystem/`); `get_stq_entry` (peripheral model commit hook)
reads the pipe STQ.

---------------------------------------------------------------------------------------------------------------------

## 3. Model approximations / differences to the legacy LSU

1. L2/L3/L4 collapse: the existing DCache interface fixes request -> response at one cycle, so tag compare, forward select and write-back request share cycle t+1.
2. The DCache is read in L1 only when the 12-bit CAM finds no conflicting older store (`GATE=1`); with `GATE=0` the read is unconditional and wasted reads cost bank slots (measured).
3. Bank-conflict retries stay hidden in the DCache (as before); a CONFLICT response replays through RDY -> RP -> L1 (one cycle later than legacy, and only on a port with no new load).
4. LDQ allocation counts empty entries only (legacy also reserved the holes of `wait_mmu_ldq`); STQ free = 64 - count (legacy also subtracted the allocated-but-not-yet-in-queue entries). Slightly more capacity.
5. Loads wait for older stores with unknown address (as legacy); the memory-order CAM therefore never fires. A page-faulting older store now blocks only overlapping loads (legacy: all loads).
6. STQ retire 4 per cycle (legacy: window 20); STA/SC write-back picks are oldest first, load write-back picks lowest LDQ index.
7. MMIO load issue is evaluated one cycle later than legacy (L2 state MMW).
8. `epoch` replaces the 25-bit request generation; per-event wake vectors are modelled as 64-bit masks (a CAM against <= 3+3+2+1 broadcast ports in RTL).
9. Replay re-entry has no priority over new AGU loads; a replay waits for a free load slot.
10. The wake-time STLF confirm (ADDR -> forward) needs the blocking store's fields broadcast with the STA event (address, size, data valid, SC state).
11. Perf-only fields: `miss` for the TMA mask. Statistics counters (`LpStats`) are not hardware.
12. Data values are not modelled (functional oracle): forwarding is decided on addresses/sizes/data-valid only.

---------------------------------------------------------------------------------------------------------------------

## 4. Results

### 4.1 Switch off / on
* Switch off: dhrystone 162,033, coremark 255,335, memory-stress-small 347,305 (identical to main); gate 62/62 identical cycles
  (`run_gate.sh`; the only FAIL is `pf-stress`, a test added to the shared `build/tests` later, absent from baseline.csv; it passes, 612,878 cycles).
* Switch on (LD4): all 62 gate tests end in difftest Success (pf-stress too: 627,240 cycles); LD3 build: same. dhrystone 170,275 (-5.1%),
  coremark 254,711 (+0.2%), memory-stress-small 353,027 (-1.6%), all Success.
* ubench (off and on identical): lat_ptr 4.0, lat_stlf 5.0, ld_unk_addr 8.0, ld_viol 7.142, stlf_partial 9.0, bank_same 0.5;
  bank_diff 0.75 (off) vs 0.923 (on: CONFLICT replays need a free L1 slot). New tests: `tests/ubench/{stlf_partial,ld_unk_addr,ld_viol,bank_same,bank_diff}.c`.
* Counters (dhrystone, early build): about 90% of the waits on an unknown older store address turn out disjoint (39.6k disjoint vs 4k
  overlap), i.e. a dependence predictor / speculation would pay; order-violation CAM fired 0 times (non-speculative policy, expected).
  DCache response watchdog fired 32 load / 5 store times on perlbench sp3 (see 4.3).

### 4.2 SPEC 30 points (IPC ratio = baseline cycles / new cycles; geomean of points)
LD4, switch on vs `res_dram60` (tag `lp_d4_on3`, 30/30 Success): **ALL 0.9831**.
perlbench 0.911, gcc 0.921 (points 0.9997 / 0.9990 / **0.781**), mcf 1.068 (0.961..1.309), gobmk 0.973, hmmer 0.937, sjeng 0.975, h264ref 0.966, astar 0.979.
Per-point list: see the output of `scripts`-style comparison (baseline `.ok` vs `tim_lp_d4_on3`).
LD3: switch off (`lp_d3_off`) vs `res_dram60` 0.9797 (LD3 itself costs 2%); switch on (`lp_d3_on`) vs LD3 off: **ALL 0.9967**
(perlbench 0.934, gcc 0.920 (0.775..1.005), mcf 1.114, gobmk 0.974, hmmer 0.926, sjeng 0.977, h264ref 0.977, astar 0.992), 30/30 Success.
Per-step IPC was NOT measured: the four steps were implemented together behind one switch (the staged pipeline cannot run with the old
STLF window); ablation knobs are provided instead (`LSU_PIPE_RP`, `LSU_PIPE_GATE`, `LSU_PIPE_BOUNCE`). Early dhrystone ablations:
ungated DCache read 184.9k vs 172.9k cycles (wasted bank slots), replay-register fill one cycle earlier -6.7% -> -5.1% with round-robin.

### 4.3 Bugs found by the SPEC runs and their fixes
1. Livelock / ROB deadlock (sjeng sp14/sp23, mcf sp2/sp61): lowest-index find-first on the RDY vector let two loads that kept missing the DTLB
   (MMU `OTHER_WALK_ACTIVE`, a walk of another page) hold both replay ports; the walk owners starved, the MMU only advances a walk on a request
   of the walking page. Fix: find-first rotated by a round-robin pointer `rp_ptr`.
2. Deadlock (perlbench sp3): committed stores stayed `iss` forever and loads in `DCW`: DCache responses never arrived (consistent with the
   DCache bank-retry latch overflow; REFUTED, see below). Workaround: a 6-bit response watchdog per in-flight store and per DCW load
   (63 cycles -> replay). Root cause (traced with LSU_PIPE_NOWD=1, perlbench sp3 ldq=1, cycle 18543): a load on port 1 loses the bank to port 0
   (same 64 B line, one grant per bank), its DCache stage 2 swallows the CONFLICT response and latches a *hidden retry*. The hidden retry is only
   issued on a cycle when the LSU has no request on that port and the port's response register is empty (`!lsu_req.valid && !retry_blocked`).
   The pipe LSU keeps one new/replayed load on port 1 every cycle, and each one is itself answered with a CONFLICT response, so the port is never
   idle: the hidden retry starves for 60+ cycles (0 hidden issues in the window), the load stays in DCW with no response coming. It is a
   starvation, not a lost response; no latch overflow fired (the check was made live with NDEBUG off). Whether the legacy LSU can hit it is not
   tested (its replay wait is longer, so ports go idle, but nothing guarantees it). Candidate fixes, none implemented: (a) the pipe LSU does not
   launch on a port whose DCache hidden retry is pending (1 bit per port, registered); (b) the hidden retry wins over a new LSU request and that
   request gets the CONFLICT response; (c) drop the hidden retry and answer CONFLICT immediately.
3. `get_stq_entry` (peripheral/timer commit hook) must read the pipe STQ (interrupt tests hung without it).

4. Hidden-retry starvation fix (item 2), two layers, both switchable with LSU_PIPE and default on:
   * `LSU_PIPE_RBLK`: DCache gives the LSU one registered bit per load / store port (`DcacheLsuIO.load_retry_busy / store_retry_busy`,
     = a hidden retry is latched). A load at L1 does not launch on such a port: it parks like a bank conflict (`dcblk` in the L1/L2 register,
     short replay wait); the store drain uses only the free ports. The bit is registered state, no new combinational path.
   * `LSU_PIPE_RPRIO` (DCache, only when LSU_PIPE is set): bank arbitration grants the ports that issue a hidden retry this cycle first,
     for loads and stores, instead of fixed port-0-first priority.
   * Remaining long store waits (> 63 cycles) are a fill that holds the bank (the fill write waits for the write buffer); fills have
     priority by design, not changed.
   * Result: the watchdog is no longer needed. perlbench sp3: 37 -> 0 long load waits, mcf sp60: 15,783 -> 3 (watchdog off).
     30 points with `LSU_PIPE_NOWD=1`, LD4 (`tim_lp_d4_rp`): 30/30 Success, watchdog fires 0 times, IPC ratio vs `res_dram60` **1.003**
     (before: 0.983; vs the watchdog build `d4_on3`: 1.020). Per program vs baseline: perlbench 0.918, gcc 0.943, mcf 1.090, gobmk 0.979,
     hmmer 1.015, sjeng 0.974, h264ref 0.982, astar 0.973; worst point gcc sp42 0.855 (was 0.781). Gate: switch off 62/62 identical cycles,
     switch on 62 difftest Success (pf-stress: no baseline entry, as before). MemSubSystem now has additive changes (`IO.h`,
     `RealDcache.*`, `MemSubsystem.cpp`); with the switch off nothing changes.

### 4.4 Not verified / open
* gcc sp42 loses 22% (0.78) in both LD4 and LD3; cause not analysed (candidates: replay bandwidth with no priority over new loads, CONFLICT replays, store drain 2 ports).
* The 6-bit watchdog is still in the code (default on, `LSU_PIPE_NOWD=1` turns it off); with the fix it never fires on the 30 points.
* Memory-order speculation and the flush-from-load channel to the ROB do not exist; the violation CAM is implemented but only a detector.
* No timing synthesis; compare widths/entries in section 1.3 are structural estimates. Stats counters, TMA `miss` and debug dumps are model-only.
* LD 1/2 ports, RP=1, GATE=0, BOUNCE were not run on the SPEC set.

---------------------------------------------------------------------------------------------------------------------

## 5. Changes made for the RTL (rtl-lsupipe worktree, docs/analysis/lsu-pipe.md section 6)

* M4: `comb_lsu2dcache_stq_p` rewritten in the RTL-aligned form (all eligible stores count their replay countdown every cycle; the head MMIO
  store takes the uncached unit independently of the ports; the DRAIN_PORTS oldest ready non-MMIO stores are the candidates, candidate m uses the
  m-th store port without a hidden retry and is dropped for the cycle when an older committed, not done store overlaps it; the unreachable
  `suppress` branch is replaced by an abort). Cycle counts differ slightly from the previous drain (e.g. dhrystone 169,280 (LSU_PIPE=1, build_m4) vs 169,419 with the previous drain).
* Debug aid `LSU_PIPE_EVT=1` (+ `LSU_PIPE_STQ=<idx>`): per-cycle LSU boundary events on stderr, same format as the RTL tb `SOC_TB_LSUEVT=1`.
* The order-violation CAM stays in the model but is not implemented in the RTL (counter was 0 on every program tried).
* Reference cycles for the RTL: `LSU_PIPE=1 LSU_PIPE_NOWD=1`, simulator built with `BUILD_DIR=build_m4`; the RTL needs `MOVE_ELIM = 0`
  because this model branch predates EXP-012.

---------------------------------------------------------------------------------------------------------------------

## 6. Critical-path cuts (switch `LSU_PIPE_CUT`, bit mask, default 0 = the model of section 5)

Goal: stage boundaries are registers, no stage reads what another stage wrote in the same cycle (see rtl-lsupipe docs/analysis/lsu-pipe.md
section 9 for the RTL, the synthesis depths and the reasoning). Only the cuts that change the cycle count are model switches; the ones that
are equivalent (RTL-only) are listed there. Build dir used: `build_cut` (build_m4 / build_rp untouched).

| bit | name | what changes | where |
| --- | --- | --- | --- |
| 1 | B | wake-up STLF confirm uses the registered STQ window (`pcur` head/count) minus `ev_ret` instead of the window after the same-cycle count chain | `comb_check_p` |
| 2 | R | STQ retire reads registered state only (entry `done`, `ccount`, `count` as at the cycle start): a store that becomes done this cycle retires one cycle later | `comb_check_p` |
| 4 | D | the replay registers pick only from RDY bits registered at the cycle start (an entry woken this cycle is picked one cycle later) | `comb_check_p` |
| 8 | F | write-back of waiting loads (WBP) picks only from WBP bits registered at the cycle start | `comb_lsu2exe_p` |
| 16 | G | a store woken by an MSHR-full fill is seen by the drain one cycle later (`pw.st_fullwake`, `pw.st_rwpre`) | `comb_dcache2lsu_stq_p`, `comb_lsu2dcache_stq_p` |
| 32 | H | the store drain only sees stores that were already committed at the cycle start | `comb_lsu2dcache_stq_p` |

Also added (always on, no behaviour change): an `abort()` if a mispredict cut would go below `stq_ccount` (invariant the RTL uses: a committed store is
never cut by a mispredict). It never fired on the 14 SPEC points or the 66 programs.

IPC cost, 14 SPEC points (COV=0.12 of 8 programs, `scripts/spec/timing_eval.sh`, `LSU_PIPE=1 LSU_PIPE_NOWD=1`, jobs 4; tags `tim_cut<mask>`;
helper scripts in `~/personal/AI-Huma-data/rtl-lsupipe/{spec14.sh,cmp14.py}`). Ratio = geomean over the 14 points of cycles(off)/cycles(on),
off = mask 0 (weighted-IPC geomean of the 14 points: 1.856). **Only 14 points, not the 30-point set.**

| mask | cuts | IPC ratio vs 0 | worst point | 
| --- | --- | --- | --- |
| 1 | B | 1.0000 | 1.0000 |
| 2 | R | 0.9959 | 0.9630 (h264ref sp17) |
| 4 | D | 0.9861 | 0.9453 (perlbench sp5) |
| 8 | F | 0.9959 | 0.9487 (h264ref sp17) |
| 16 | G | 1.0000 | 0.9999 |
| 32 | H | 0.9980 | 0.9893 (h264ref sp17) |
| 7 | B+R+D | 0.9828 | 0.9501 |
| 31 | B+R+D+F+G | 0.9784 | 0.9135 (h264ref sp17) |
| **63** | **B+R+D+F+G+H** | **0.9787** | 0.9058 (h264ref sp17) |

Gate: mask 63 on the 66 programs of `scripts/ref_cycles.txt`: all end in difftest Success (0 NOSUCCESS); mask 0 reproduces `ref_cycles_lsupipe_m4.txt` exactly
(66/66 equal cycles). Mask 63 cycle geomean over the 66 programs: 0.9951 of mask 0 (worst: dhrystone 169,280 -> 182,763 = -7.4%, vm-asid-test -6.8%, vm-pagefault-test -6.7%; 11 programs lose more than 1%).
The 62-item `run_gate.sh` (cycle comparison against baseline.csv) was not run for these masks; only difftest Success.
Reference cycles for mask 63: `rtl-lsupipe/scripts/ref_cycles_lsupipe_cut.txt` (simulator `build_cut`, `LSU_PIPE=1 LSU_PIPE_NOWD=1 LSU_PIPE_CUT=63`).

---------------------------------------------------------------------------------------------------------------------

## 7. L2 cuts (switch `LSU_PIPE_L2`, bit mask, default 0 = the model of section 6)

Goal: after the section-6 cuts the longest synthesised path (156 generic-gate levels, rtl-lsupipe docs/analysis/lsu-pipe.md section 9) sits inside one L2
cycle: `pl12 -> L2 decision (68) -> rs_ok -> direct write-back -> write-back port allocation -> LR/SC reservation / LDQ state`. This section records the
load-to-use accounting of a hit, the model switches that cut that cycle, and their IPC cost. Depth numbers are generic gate levels (yosys + ABC, no library),
**not nanoseconds; no calibration "N levels = 1 GHz" exists** (nothing in this tree can produce one), so everything below is a relative comparison.
Build dir used: `build_l2` (SPEC runs, frozen) / `build_l2d` (development); `build_cut`, `build_m4`, `build_rp` untouched.

### 7.1 Per-cycle account of an L1D hit (before any L2 cut; `LSU_PIPE=1 LSU_PIPE_CUT=63`)

Measured with the timeline trace `LSU_PIPE_TL=1` (stderr lines `[TL] t=<cycle> ...`: ISSUE / EXEDONE / AGU/L1 / L2 / LSU_WB / WAKE) on the pointer chase
`tests/ubench/lat_ptr.c` (every load's address comes from the previous load; steady state 4.000 cycles per load). `t` = the cycle in which the AGU result is
visible to the LSU (`exe2lsu`), i.e. the execute cycle of the load's AGU uop.

| cycle | event | where in the model |
| --- | --- | --- |
| t-1 | load issued: select in the IQ, PRF read | `Isu::comb_issue`, `Prf::comb_read` |
| t | AGU executes; **L1**: DTLB lookup, partial STLF CAM, DCache S1 read request; PL12 written | `Exu::comb_exec` -> `comb_exe2lsu_p` -> `comb_tlb_*_p` -> `comb_stlf_p` -> `comb_lsu2dcache_ldq_p` |
| t+1 | **L2**: DCache response (tag compare, data select) / STLF confirm; direct completion -> **write-back request on load port p** (`lsu2exe.wb_req`); LDQ entry freed; EXU hands it to the PRF write-back register and the ROB (`exu2rob`) | `comb_dcache2lsu_ldq_p`, `comb_lsu2exe_p`, `Exu::lsu_writeback` |
| t+2 | the PRF write-back register `inst_r` holds the load: **wake-up broadcast** (`Prf::comb_awake`, "slow wake-up": only from the write-back register; `Isu::comb_awake` excludes loads from the fast wake-up); IQ ready bits are set | `Prf::comb_awake`, `Isu::comb_awake`, `IssueQueue::comb_wakeup` |
| t+3 | consumer issued (select + PRF read) | `Isu::comb_issue` |
| t+4 | consumer executes (EXEDONE) = **load-to-use 4** | |

The wake at cycle W lets the consumer issue at W+1 (the IQ's ready bits are refreshed in `seq`; the select of cycle W has already run). Also visible:
the data of the load is in the PRF write-back register at t+2, two cycles before the consumer executes (a fast/speculative wake-up could use that slack).
Excerpt (consumer = next pointer load): `t=1145 AGU/L1`, `1146 L2 + LSU_WB`, `1147 WAKE`, `1148 ISSUE (consumer)`, `1149 EXEDONE + AGU/L1 of the consumer`.

### 7.2 Switches (`LSU_PIPE_L2`, all default off; with 0 the cycle counts equal the section-6 model exactly)

| bit | name | what changes | load-to-use (hit) |
| --- | --- | --- | --- |
| 1 | a | **L3 register**: the direct completion decided at L2 (forward, page fault, DCache hit) is registered per port (`PL3`, LDQ state `L_WB3`); the write-back port allocation, the LDQ free and the LR reservation run one cycle later from registers (`comb_lsu2exe_p` step 1). Wake-up still from the PRF write-back register | 5 |
| 2 | a2 | (needs 1) the consumers are woken from the L3 register in the cycle of the write-back request (`LsuAwakeIO.wake[LD+p]`); the PRF write-back register no longer wakes that load (`Uop.early_woken`). Not speculative: the decision is registered | 4 |
| 4 | oracle | upper bound of 8: wake-up at L2 only for the loads that really hit (no wrong wake-ups, no cancel); not implementable | 3 |
| 8 | d | **speculative wake-up**: at L2 the consumers of every load whose L1 result is "TLB hit, not MMIO, no CAM candidate, no hidden-retry block" are woken from the L1/L2 register (`LsuAwakeIO.wake[p]`); if the DCache does not answer HIT in that cycle, the L2 stage registers a cancel (`PL12`-side `PSf`) and in the next cycle, before the select, the issue queues and the dispatch busy table put the woken entries back to waiting (`IssueQueue::cancel_wake`, `Dispatch::comb_cancel`); the load wakes them again through the normal path later | 3 (hit) |
| 16 | F | a load that gets MSHR_FULL in this very cycle is not woken by this cycle's MSHR fill (it waits for its counter); only loads parked at the start of the cycle compete for the first `LD` MSHR-FULL wake-ups. Cuts the chain L2 decision -> MSHR-FULL pick (4 chained first-set) -> LDQ state | - |
| 32 | M | an MMIO load that becomes MMW at L2 in this cycle is issued to the bus one cycle later (the issue looks at registered MMW only). Cuts L2 decision -> MMIO pick -> uncached unit | - |
| 64 | S | at most one SC completion per cycle (the SC pick becomes one find-first instead of `LD` chained ones). Cuts SC pick -> `sc_go` -> STQ view -> wake-up confirm | - |
| 128 | Y | (with 8) no speculative wake-up for loads coming from a replay register. **Negative result**, see 7.3 | - |

Wake-up channels: the issue-queue wake bus gets `2*LD` extra channels after the config.h ones (`WAKE_CH_ALL = MAX_WAKEUP_PORTS + 2*LD`, `Isu::comb_awake`);
`MAX_WAKEUP_PORTS` in `config.h` is unchanged (the RTL configuration package is generated from it).

Cancel semantics (bit 8): the wake at cycle t+1 (end of cycle) lets a consumer issue at t+2 at the earliest; the L2 result of cycle t+1 is registered, so the
cancel is applied at the start of t+2, **before the select**, and no consumer can have issued: nothing has to be squashed or replayed *after* issue.
This is the ideal case; in hardware the cancel then sits in front of the select (registered cancel vector against the source tags of the entries, ANDed into the ready
bits), which is a new path of the select cycle (not synthesised here). A load whose consumers are cancelled wakes them through the slow path at write-back; the
"replay" of the consumers is therefore just the later wake-up. If the cancel arrived one cycle later than the select, issued consumers would have to be killed and
re-issued (IQ entries kept until the load is confirmed): **not modelled**.

### 7.3 IPC (14 SPEC points, `LSU_PIPE=1 LSU_PIPE_NOWD=1 LSU_PIPE_CUT=63`, COV=0.12, geomean of cycles(`L2=0`)/cycles(mask); tags `tim_l2_<mask>`)

Only 14 points, not the 30-point set; single run each (the model is deterministic). Baseline `LSU_PIPE_L2=0` reproduces the section-6 mask-63 run point for point (ratio 1.0000, all 14 points).

| mask | what | load-to-use (hit) | IPC ratio | worst point |
| --- | --- | --- | --- | --- |
| 1 | a (L3 register, slow wake-up) | 5 | **0.9699** | 0.9266 (astar sp21) |
| 3 | a + a2 (wake from the L3 register) | 4 | **1.0023** | 0.9963 (sjeng sp18); best 1.0432 |
| 4 | oracle, no L3 | 3 | 1.0295 | 1.0008 (gcc sp42) |
| 8 | d, no L3 (speculative wake-up, ideal cancel) | 3 | 1.0295 | 1.0008 |
| 5 | a + oracle | 3 | 1.0242 | 1.0008 (mcf sp61) |
| 9 | a + d | 3 | 1.0242 (identical to 5) | 1.0008 |
| 137 | a + d + Y (no speculative wake-up for replayed loads) | 3 for first-pass loads | 1.0005 | 0.9814 (gobmk sp23) |
| 115 | a + a2 + F + M + S | 4 | **1.0018** | 0.9918 (h264ref sp17) |
| 123 | 115 + d | 3 | **1.0290** | 1.0022 (h264ref sp17); best 1.0770 |

Reading: (1) an L3 register on the old slow wake-up costs 3.0% (the whole +1 cycle of load-to-use); (2) **waking from the L3 register in the cycle of the write-back request (a2) hides it completely
and without speculation** (the decision is registered there, so the wake-up needs no prediction), 1.0023; F, M, S (the three depth cuts of 7.2) cost about 0.05% together;
(3) the speculative wake-up at L2 then gives a further 2.9% (load-to-use 3 instead of 4); the real cancel equals the oracle (mask 9 = 5, 8 = 4) because the model cancels **before the select** at no cost, so the number is an upper bound.
The estimate asked for ("hit rate x 1 cycle") is therefore not needed: the full implementation was cheap enough in the model.
(4) Y: the speculative wake-up of replayed loads carries most of the gain: SPEC pooled, 147M speculative wake-ups of which 61.7M succeed, but only 60.7M of them are first-pass loads (40.7M succeed).

Speculative wake-up counters (`LSU_PIPE_STATS=1`, summed over the 14 points, mask 123): predictions 146.8M, correct 61.5M (41.9%), **cancelled 85.3M (58.1%)**, issue-queue entries put back to waiting 96.5M (1.13 per cancel);
72.9% of the L3 write-backs had been woken speculatively. First-pass loads only (mask 137): 33% cancelled. dhrystone (mask 9): 11.6% cancelled, coremark (123): 14%.
The prediction has no cache information (tag hit is unknown at L1), so misses and the many replays of waiting loads are mispredicted; in hardware a hit/miss predictor would cut this, **not modelled**.

The 66 gate programs (cycles, same switches, `rtl-lsupipe/scripts/ref_cycles_lsupipe_l2_{115,123}.txt`, all end in difftest Success): geomean of cycles(`L2=0`)/cycles(mask): 1.0009 (115, worst `max` 0.974), 1.0052 (123, worst `pascal` 0.968);
dhrystone 182,763 -> 167,681 (115) / 176,946 (123), coremark 256,761 -> 259,117 / 238,666. The dhrystone change under 115 (-8%) is much larger than any 14-point SPEC change; dhrystone is sensitive to timing (the section-6 cuts had cost it 7.4%), **cause not analysed**.

### 7.4 What the L2 cuts F, M, S change in the model (all cycle-changing, depth-motivated; RTL-only equivalent cuts are in rtl-lsupipe docs/analysis/lsu-pipe.md section 10)

* F (16): `comb_check_p`, RC_DC wake-up: `pcur.ldq[i].st == L_RPW` is required for the MSHR-FULL wake-up (and for counting against the first `LD`).
* M (32): `comb_stlf_p`, MMIO issue: `pcur.ldq[i].st == L_MMW` is required.
* S (64): `comb_lsu2exe_p` step 2: at most one SC per cycle (`sc_n`).
* Y (128): `comb_dcache2lsu_ldq_p`: `PL12.rpl` (slot came from a replay register) suppresses the speculative wake-up.

### 7.5 Debug aids and bookkeeping

`LSU_PIPE_TL=1`: timeline trace (`[TL]` lines: ISSUE / EXEDONE / AGU/L1 / L2 / LSU_WB / WAKE) on stderr (also in `Isu.cpp`, `Prf.cpp`, `Exu.cpp`, `util.h`); no effect on cycles.
`Uop.early_woken`, `LsuAwakeIO` (`Wires::lsu_awake`), `IssueQueue::cancel_wake`, `Isu::comb_cancel`, `Dispatch::comb_cancel`, `RealLsu::comb_spec_cancel` (called first in `BackTop::comb_stage_decode_and_resources`).
Reproduce: `LSU_PIPE=1 LSU_PIPE_NOWD=1 LSU_PIPE_CUT=63 LSU_PIPE_L2=<mask> build_l2d/simulator <program>`; SPEC: `~/personal/AI-Huma-data/rtl-lsupipe/spec14l2.sh l2_<mask> LSU_PIPE_L2=<mask>` then `cmp14.py l2_0 l2_<mask>`.
Build note: build a model variant in its own clean directory (`gmake DS3_DIR=<trace-model-me>/third_party/DRAMsim3 BUILD_DIR=build_l2d LDFLAGS=-lz`): copying an existing build directory (`cp -a build_x build_y`) keeps the dependency files of the
old directory, headers then do not trigger recompiles and a changed struct layout gave a ROB deadlock once (found and fixed by a clean build).

### 7.6 Approximations / not verified

* Ideal cancel (see 7.2): the cancel is applied before the select of the cycle after the wake-up; hardware needs a registered cancel vector in front of the IQ select (RTL implements exactly this, not synthesised on its own), or issued consumers would have to be killed and replayed (not modelled).
* The wake-up of a load that was woken at L2 and completes at L3 is not repeated at the PRF write-back register (`early_woken`); the RTL repeats it harmlessly.
* The data of a speculatively woken consumer is a bypass from the L3 register to the PRF read stage; the model has no data path (the RTL needed it, found by the per-commit check).
* Load-to-use 3 relies on the prediction being checked in L2 (DCache response of the same cycle); nothing changes in the DCache.
* 14 points only; no Linux boot; the 62-item `run_gate.sh` cycle comparison was not run for these masks (only difftest Success on the 66 programs).

---------------------------------------------------------------------------------------------------------------------

## 9. After merging main (2026-11-07, branch exp/lsu-pipe, first stage of the merge into main: staged LSU WITHOUT the speculative wake-up)

Scope: `LSU_PIPE=1 LSU_PIPE_CUT=63 LSU_PIPE_L2=115` (rblk, rprio on; L2 bit d = speculative wake-up is NOT part of this stage, RTL `LP_L2=1907`).
Merged into the branch: main at `ac16e92` (move elimination EXP-012 default on, `--fast-warmup` EXP-011, L1D prefetcher EXP-009 default on with
`pf-stress`, regenerated baseline, fleet scripts). The work in progress was first committed in three pieces (`844dc41` model, `a3cb2b1` ubench,
`7490c06` this document), then `git merge main` (merge commit `7e7df6f`).

### 9.1 Conflicts and how they were resolved

* **Textual conflicts: none.** The overlapping files (`MemSubSystem/RealDcache.cpp`, `MemSubsystem.cpp`, `back-end/Dispatch.cpp`, `Isu.cpp`, `IO.h`,
  `Prf.cpp`, `RealLsu.*`) merged automatically. Semantic overlaps that were read through by hand:
  * Prefetcher probe vs `RPRIO` / `RBLK` (RealDcache stage 1): the probe loop runs after the demand loops and only takes a load port whose slot is
    not valid (`slot.valid` is set by a demand request or by a hidden retry), so the retry-first port order of `RPRIO` is untouched and a retry is never
    displaced by a probe. `load_retry_busy` (RBLK) is still published by `publish_pending_dcache_response`. No change was needed.
  * Move elimination vs the L2 wake-up hooks (`Dispatch.cpp` busy table cancel, `Isu.cpp` / `IssueQueue.h` cancel_wake): disjoint code paths; `ME_CHECK=1`
    on the 63 gate programs with the staged LSU: success, no assertion text (the check prints nothing when it holds; not confirmed that it is armed).
  * `LoadReq.replay` (used by the prefetcher as a training filter): the staged LSU already sets it (`e.rtype != HIT`), no change.
* **Build:** the worktree has no initialised DRAMsim3 submodule (empty `third_party/DRAMsim3`); built with
  `gmake LDFLAGS="-lz" BUILD_DIR=build_mg DS3_DIR=<main checkout>/third_party/DRAMsim3 -j3` (library `build_dramsim3/libdramsim3.a` reused).

### 9.2 Bug found and fixed: Linux boot ROB deadlock (not caused by the merge)

The staged LSU never booted Linux before ("no Linux boot" above). The first Linux runs deadlocked (`ROB deadlock`, 50,000 cycles without commit):
config (b) PF off at 66.6 M cycles, config (c) PF on at 52.3 M, and the pre-merge commit `7490c06` (built separately, move elimination off, no prefetcher)
at 52.8 M, so it is a defect of the staged LSU, found only now by a long program. `LSU_PIPE_NOWD=1` or not: no difference.

Root cause (dumps with `LSU_PIPE_DUMP`, MMU trace with the new `LSU_MMU_DBG=<cycle>` / `LSU_MMU_DBG2=<cycle>`): the DTLB page walk is shared. A request
for another page only gets RETRY (`OTHER_WALK_ACTIVE`) until the *owner* of the walk asks again. The walk owner was a store (STQ[9], page 0x9574c) whose
translation is retried through the STA replay register `srp`, which always took the **oldest** store with `trp`. An older store of another page (STQ[8]) was
retried every cycle, failed every time, set `trp` again and was taken again: the younger owner never asked, the walk never finished, the ROB head
(a load of a third page) waited forever (same family as bug 1 of section 4.3, which was fixed for the load replay registers with `rp_ptr`).
Fix: the `srp` pick is round-robin over the STQ index (`srp_ptr`, `RealLsuPipe.cpp` end of `comb_flush_p`/wake-up pass, `RealLsuPipe.h`). A first attempt
(oldest first, but another store goes first after 4 takes of the same one) fixed the first deadlock and hit a second one at 84.4 M (owner further down the
age order); round-robin does not. Effect on the gate and the big programs: none (gate cycles of (b) and (c) identical before and after; memory-stress (b) 2,415,952 -> 2,415,948).
The RTL has the same fix (`rtl-lsupipe/rtl/backend/lsu/lp_rp.sv`, `srp_ptr_q`), docs/analysis/lsu-pipe.md section 12.

Debug instrumentation left on the branch for the later clean-up: `LSU_MMU_DBG`, `LSU_MMU_DBG2` (`BackTop.cpp`), `TlbMmu::dbg_walk_*` accessors, plus the earlier ones.

### 9.3 Validation of the merged model (binary `build_mg`, simulations with `SIM_MAX_CYCLES`, at most 3 jobs)

| Configuration | Check | Result |
| --- | --- | --- |
| (a) `LSU_PIPE` unset (all switches off), prefetcher on (main default) | `scripts/run_gate.sh`: cycles equal `results/baseline/baseline.csv` | **63/63 equal** (62 + pf-stress), all difftest Success, 0 mismatches |
| (a) | big programs (main standard) | dhrystone 161,183; coremark 251,535; memory-stress 2,380,486; smc-div 11,348; smc-addi 11,218; memory-stress-small 334,635; pf-stress 619,963: **all equal to PROJECT.md** |
| (a) with `PF_ENABLE=0` | big programs | 162,033 / 252,400 / 2,425,220 / 11,348 / 11,218 / 347,305, pf-stress 613,485: equal to `baseline_nopf.csv` numbers in PROJECT.md (the gate itself was not run against `baseline_nopf.csv`) |
| (a) with `ME_CHECK=1` | big programs (first 3) | cycles unchanged |
| (b) `LSU_PIPE=1 CUT=63 L2=115`, `PF_ENABLE=0`, with and without `LSU_PIPE_NOWD=1` | gate 63 programs | **63/63 difftest Success**, 0 mismatches, identical cycles with and without NOWD |
| (c) same, prefetcher on, with and without `LSU_PIPE_NOWD=1` | gate 63 programs incl. pf-stress | **63/63 difftest Success**, no deadlock; pf-stress 623,428 (legacy LSU 619,963; (b) 610,886) |
| (c) with `ME_CHECK=1` | gate | 63/63 Success, cycles identical to without |
| (b) | big programs | coremark 254,460; dhrystone 167,681; memory-stress 2,415,948; memory-stress-small 345,364; pf-stress 610,886; smc-addi 11,406; smc-div 11,536 |
| (c) | big programs | coremark 251,224; dhrystone 166,296; memory-stress 2,361,664; memory-stress-small 330,652; pf-stress 623,428; smc-addi 11,406; smc-div 11,536 (same with and without NOWD) |
| (b), (c) | Linux from reset to `rv32 login:` (`~/personal/AI-Huma-data/linux/linux.bin`, `SIM_MAX_CYCLES=400000000`, NOWD) | **booted** after the fix in 9.2 (login between 100 M and 110 M cycles by the 10 M progress marks; exact cycle not recorded); before the fix: deadlock. (a) also booted (reference run) |
| (c) fast warm-up | one SPEC checkpoint (bzip2 sp10), `--fast-warmup -w 100000/200000 -c 300000`, (a) and (c) | runs to the commit limit, 0 TB_CHECK mismatches (no cycle comparison made) |

Prefetcher x LSU_PIPE interaction: **no deadlock and no mismatch found** in (c) (gate, pf-stress, big programs, Linux; NOWD on and off).
The model watchdog (`LSU_PIPE_NOWD` unset) never changed a cycle count: with NOWD the `[WD] ... stuck issued` lines still appear for stores waiting more than
63 cycles (crc32, memory-stress-small, Linux), these are long fill waits that complete on their own, cycles are identical with and without the watchdog.
Not run: `LSU_PIPE_RPRIO=0` / `LSU_PIPE_RBLK=0` with the prefetcher, other CUT/L2 masks with the prefetcher, SPEC points, small-LLC check, random differential tests.
Model default stays OFF (env switches); the new defaults for main (and the new `baseline.csv`) are a later stage.

## 10. Merge into main (EXP-014, 2026-11-07)

Branch `lsu-pipe-main` (two commits: code with the switch off, then the new defaults). Content: `RealLsuPipe.*`, the hooks (Dispatch busy-table cancel, IssueQueue/Isu wake channels, PRF wake, DCache RPRIO order and retry-busy outputs), L2 cuts mask 115, the STA replay register round-robin (9.2).
Left on the experiment branch `exp/lsu-pipe` only: `LSU_MMU_DBG` / `LSU_MMU_DBG2`, `TlbMmu::dbg_walk_*`, `LP_TL` timeline prints, `LSU_PIPE_ROBTRACE`, `LSU_PIPE_TRACE`, `tests/ubench`, `scripts/ubench.sh`, `scripts/spec/timing_eval.sh`.
Speculative wake-up (L2 bit 8 = d, with 128 = Y and 4 = oracle) is in the code and OFF (`LSU_PIPE_L2` 115 has none of them); it is entangled with the wake channels (`WAKE_CH_ALL`, `lsu_awake.cancel`) so it stays, default off.
New defaults: gate 63/63 difftest Success, new `results/baseline/baseline.csv`; `LSU_PIPE=0` reproduces the old baselines (`legacy_lsu_baseline*.csv`). Big programs: see PROJECT.md gate item 4.
Model-only: the LDQ/STQ watchdog (63 cycles stuck, off with `LSU_PIPE_NOWD=1`) is not in the RTL; it never fired in the gate programs (memory-stress differs by 4 cycles with and without it).

### 10.1 SPEC, 30 points (COV 0.25 of 8 programs, exact warm-up, prefetcher on, default watchdog; 2026-11-07)

Legacy LSU (`LSU_PIPE=0`, tag `tim_lp_legacy`) vs the new default (`LSU_PIPE=1 CUT=63 L2=115`, tag `tim_lp_new`); same simulator binary (`build_lm2`), checkpoints, `scripts/spec/replay_one.sh`, results in `~/personal/AI-Huma-data/spec-rv32imab/tm/tim_lp_*`. IPC change = (ins/cycles new) / (ins/cycles legacy) - 1. All 60 runs end with difftest Success, 0 mismatches, no crash or deadlock.

| benchmark | point | cycles legacy | cycles new | IPC change |
|---|---|---|---|---|
| 400.perlbench | sp3_target157064 | 4539718 | 5194211 | -12.60% |
| 400.perlbench | sp5_target244698 | 3183317 | 3583152 | -11.16% |
| 403.gcc | sp10_target688 | 3063677 | 3123430 | -1.91% |
| 403.gcc | sp16_target55459 | 3056633 | 3095347 | -1.25% |
| 403.gcc | sp42_target76255 | 6836930 | 8040623 | -14.97% |
| 429.mcf | sp16_target5282 | 28375784 | 27732030 | +2.32% |
| 429.mcf | sp2_target1739 | 11957844 | 11465784 | +4.29% |
| 429.mcf | sp32_target19545 | 26020048 | 25482844 | +2.11% |
| 429.mcf | sp51_target22531 | 8044742 | 8142136 | -1.20% |
| 429.mcf | sp60_target26087 | 28755916 | 27086611 | +6.16% |
| 429.mcf | sp61_target1970 | 10503394 | 10089667 | +4.10% |
| 429.mcf | sp70_target9981 | 5826180 | 5949409 | -2.07% |
| 429.mcf | sp71_target13273 | 25193187 | 24710093 | +1.96% |
| 445.gobmk | sp12_target148300 | 5897175 | 6136926 | -3.91% |
| 445.gobmk | sp23_target156257 | 4448938 | 4801291 | -7.34% |
| 445.gobmk | sp24_target141343 | 6137983 | 6356125 | -3.43% |
| 445.gobmk | sp3_target20898 | 6458558 | 6668217 | -3.14% |
| 456.hmmer | sp11_target56401 | 1938232 | 2124336 | -8.76% |
| 456.hmmer | sp26_target5428 | 1829368 | 2017864 | -9.34% |
| 456.hmmer | sp2_target25058 | 1881386 | 2053888 | -8.40% |
| 456.hmmer | sp5_target75553 | 1769166 | 1925294 | -8.11% |
| 458.sjeng | sp14_target208610 | 5601522 | 5833461 | -3.98% |
| 458.sjeng | sp17_target162647 | 5714502 | 5939891 | -3.79% |
| 458.sjeng | sp18_target41872 | 5567531 | 5887791 | -5.44% |
| 458.sjeng | sp23_target208006 | 5561249 | 5818308 | -4.42% |
| 464.h264ref | sp17_target116357 | 4232476 | 5040538 | -16.03% |
| 464.h264ref | sp9_target161085 | 9503091 | 11492588 | -17.31% |
| 473.astar | sp13_target7396 | 5823074 | 6314760 | -7.79% |
| 473.astar | sp17_target36471 | 11266074 | 11399819 | -1.17% |
| 473.astar | sp21_target9925 | 6271530 | 6749367 | -7.08% |

Per benchmark (geomean of the points): perlbench -11.88% (2 points), gcc -6.26% (3), mcf +2.18% (8), gobmk -4.47% (4), hmmer -8.65% (4), sjeng -4.41% (4), h264ref -16.67% (2), astar -5.39% (3).
**All 30 points: geomean -4.98%, arithmetic mean -4.79%, min -17.31% (h264ref sp9), max +6.16% (mcf sp60).** Only mcf (memory-bound) gains; the integer programs lose 4-17% IPC (the staged LSU adds load-to-use latency; the speculative wake-up that would win part of it back is not in this stage).


### 10.2 Checks on the main-ready branch (2026-11-07)

Linux from reset to `rv32 login:` (prefetcher on, default config, `SIM_MAX_CYCLES=400000000`): reached before 110 M cycles (no deadlock, watchdog on; log `~/personal/AI-Huma-data/linux/lsupipe_main/model_boot.log`). Small-LLC check (`llc_small_check.sh`, 64 KiB LLC) with `LSU_PIPE=0`: 5/5 equal to main; with the new default the 5 programs end with Success and 0 mismatches (memory-stress-small 621,167 vs 632,880 legacy).

---------------------------------------------------------------------------------------------------------------------

## 11. IPC fixes (branch lsu-pipe-fix, 2026-11-07): 3 drain ports with skip-blocked, 4 replay registers, MMIO exempt from the F cut (FM)

Analysis: `docs/LSU_PIPE_IPC.md` (root causes of the -4.98% against the legacy LSU). This section records what was implemented and measured.

### 11.1 Changes (defaults)
| parameter | before | now | env | RTL |
| --- | --- | --- | --- | --- |
| `CFG_LSU_PIPE_DRAIN` committed stores sent to the DCache per cycle | 2 | **3** | `LSU_PIPE_DRAIN` | `lp_pkg::NDR` |
| `CFG_LSU_PIPE_DCAND` drain candidates (CAM ports) = first N ready non-MMIO committed stores; a candidate blocked by an older overlapping store does **not** take a port, the m-th unblocked candidate takes the m-th free port | first-version drain (0): candidate m on port m, blocked candidate keeps the port idle | **4** (0 = first-version drain) | `LSU_PIPE_DCAND` | `lp_pkg::NDC` |
| `CFG_LSU_PIPE_RP` load replay registers | 2 | **4** (`RP_MAX` 4) | `LSU_PIPE_RP` | `lp_pkg::NRP` |
| `CFG_LSU_PIPE_CUT` | 63 (B R D F G H) | **127** (+ FM, bit 64; F stays on) | `LSU_PIPE_CUT` | `LP_CUT` 262143 (bit 17 = FM) |
`LSU_PIPE_DRAIN=2 LSU_PIPE_DCAND=0 LSU_PIPE_RP=2 LSU_PIPE_CUT=63` reproduces the previous default (gate 63/63 equal to the old baseline.csv before it was regenerated). The 4 candidates of 64 are enough: 2 or 3 candidates and 3 or 4 gave the same cycles on the 12 points tried (`z_n_c3/c4`, `z_n_d3c4` in the study); more than 4 candidates is not needed. Drain = 3 ports needs the third STA port (`STA`=3).
CUT bit 64 (new, FM; RTL `LP_CUT` bit 17): with the F cut the MMIO load whose response arrives in this cycle may still be written back in this cycle (see 11.3). F itself stays on: removing it makes the RTL `lp_core` 36 levels deeper (131 -> 167 levels, rtl docs 15), although it would give +0.1 point more IPC (CUT 55 below).

### 11.2 SPEC 30 points (same set / parameters as 10.1: exact warm-up, 10M warm-up + 10M measured, prefetcher on, `LSU_PIPE_STATS=1`; geomean of cycles(legacy)/cycles(config) - 1, tags in `/root/lsufix/res` on the server)
| program (points) | current (63, DRAIN 2 first-version, RP 2) | + drain fix (3 ports, 4 cand, skip) | + RP 4 | + F off (CUT 55, not taken: +36 levels) | F off + L2=123 (speculative wake-up d, info) | **F kept + MMIO exempt (CUT 127, new default)** |
| --- | --- | --- | --- | --- | --- | --- |
| 400.perlbench | -11.9% | -12.5% | -9.2% | -10.5% | -5.6% | **-9.1%** |
| 403.gcc | -6.3% | -2.2% | -2.1% | -2.1% | -2.0% | -2.1% |
| 429.mcf | +2.2% | +2.4% | +3.3% | +3.4% | +4.5% | **+3.5%** |
| 445.gobmk | -4.5% | -4.2% | -3.7% | -3.5% | +0.5% | **-3.7%** |
| 456.hmmer | -8.7% | +1.8% | +1.9% | +2.1% | +3.1% | **+2.1%** |
| 458.sjeng | -4.4% | -4.5% | -3.9% | -3.8% | +2.4% | **-3.7%** |
| 464.h264ref | -16.7% | -14.3% | -14.4% | -4.9% | -3.9% | **-4.9%** |
| 473.astar | -5.4% | -5.3% | -4.0% | -4.1% | +2.3% | **-4.0%** |
| **ALL 30** | **-4.98%** | **-2.94%** | **-2.18%** | -1.52% | +1.34% | **-1.39%** |
(The program rows are geomeans over the points of that program in the 30-point set of 10.1; the per-point tables are in the study data.) Default (last column) is -1.39%, i.e. the F-off column (-1.52%) is only 0.13 points lower. Drain fix alone +2.0 points (hmmer -8.7 -> +1.8, gcc sp42 -15.0 -> -6.0, h264 sp17), RP 4 +0.8, F off / FM +0.7-0.8, L2=123 speculative wake-up another +2.9 (hardware cost of the wake-up not measured here; the 58 percent cancel rate of 7.3 stays).
Remaining deficit to the legacy LSU: -1.52% (perlbench -10.5%, gobmk, sjeng, astar around -4%: cut D and the replay path, not investigated further).

### 11.3 Why the F cut cost h264ref 16 percent (found with `LSU_PIPE_STATS` occupancy counters)
h264ref sp9 executes 2.78 million MMIO loads in the 20 M simulated cycles (`mmio_ld`), almost 30 percent of its loads. MMIO loads run one at a time (the uncached unit) and a load only goes to the bus when it is the oldest unfinished instruction. With the F cut the result of the MMIO response goes through the registered WBP bit and is written back one cycle later, so the next MMIO load starts one cycle later: 2.6 M extra cycles in a 16.3 M cycle run (16.28 M -> 18.87 M, the same number as the WBP occupancy counter, 2.70 M entry-cycles against 2.8 K without F). F therefore costs exactly one cycle per MMIO load in a serial chain, not because of WBP port pressure. Cure options: (a) drop the F cut (-1.52%), but the RTL synthesis says F is worth 36 levels of lp_core (131 with F, 167 without; the study's estimate of about 6 levels was wrong), (b) keep F and give the MMIO response a same-cycle write-back (CUT bit 64 = FM, **default**): ALL -1.39%, h264ref -4.9%. RTL: `wbp_sel = wbp_r | wbp_m` with `wbp_m` = entry that goes MMR -> WBP in this cycle, an OR in front of the registered select.
Caveat on the benchmark: 2.78 M MMIO loads in a SPEC checkpoint is not typical SPEC behaviour (checkpoint restore region mapped as MMIO?); it is the same in the legacy run.

### 11.4 Checks (model, default build `build/simulator`)
| check | result |
| --- | --- |
| gate (`run_gate.sh`, 63 tests) after `baseline.csv` / `baseline_nopf.csv` regeneration | 63/63 PASS, all difftest Success, 0 mismatches (both prefetcher on and `PF_ENABLE=0`) |
| old defaults by env (`LSU_PIPE_CUT=63 LSU_PIPE_DRAIN=2 LSU_PIPE_DCAND=0 LSU_PIPE_RP=2`) before regeneration | 63/63 equal to the previous baseline.csv |
| memory-stress, memory-stress-small, smc-addi, smc-div, load-store-aligned, pf-stress, each with and without `LSU_PIPE_NOWD=1` | all Success, 0 mismatches, cycles identical with and without the watchdog (2,356,073 / 330,579 / 11,406 / 11,536 / 1,958 / 613,402) |
| 12 SPEC points with the most watchdog events, `LSU_PIPE_NOWD=1` | all finish (no deadlock), cycles within +-0.2% of the watchdog run (the stuck loads are wrong-path loads whose DCache response is lost) |
| Linux boot to `rv32 login:` (NOWD, `SIM_MAX_CYCLES=400000000`) | CUT 55 build: login reached (log line `rv32 login:` at the 100 M mark); final CUT 127 build: see below |
Side finding: with 4 replay registers the load watchdog ("load stuck in DCW", a DCache request that never answers, 63 cycles) fires much more often: 4,267 events in the 30 runs (22 runs affected) against 403 with 2 registers and 142 in the (6) current-default runs checked; the affected loads are on mispredicted paths (all NOWD runs complete), root cause in the DCache request path not analysed. The RTL has no watchdog, so a cycle difference against the default model is possible in short programs when a wrong-path load is released by the watchdog (seen with the intermediate CUT 55 build on sub-longlong: 1905 against 1906 without the watchdog and in the RTL; with the final defaults all 63 tests are equal with and without NOWD); the RTL reference tables are generated with `LSU_PIPE_NOWD=1`. The NOWD run of the 12 SPEC points above used CUT 55.
