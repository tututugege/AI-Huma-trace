# P3: deleting the value datapath

Order is by evidence (P0 profile + poison control), not by size. Every step: 56/56 cycle counts equal to baseline,
fact misses 0, csr_read patches only for mip/time (2 in timer-interrupt-mask). With facts mandatory the model runs
without difftest (main.cpp sets disable_difftest when the bridge is on); check mode (TB_CHECK, P1b) is the pre-P3
tool and is meaningless once values are deleted.

## P3.1 (patch 0006): ALU / MUL / DIV / FP compute, FPU softfloat, generated FADD/FMUL
- Fu.h 498 -> 255 lines, FPU.h 380 -> 59 (latency by operation type only), fadd.h (12,409) and fmul.h (9,673) removed.
- MUL/DIV latencies are fixed; FPU latency depends only on the operation type: no latency facts needed.
- Speed: ~0 (92.1 G -> 92.0 G host instructions on timer-mmio) as P0 predicted; the bridge cost is now included.

## P3.2 (patch 0007): PRF register file and operand fields
- Removed reg_file/reg_file_1, read_operand_with_bypass, PrfExeUop/MicroOp src1_rdata/src2_rdata, the PRF write port.
  The write-back stage `inst_r` stays (it drives load wake-up and bypass timing).
- AGU, BRU, SDU and CSR write data now read only the fact (fallback 0 for killed instructions, whose result is unobservable).
- ckpt restore of registers and get_reg() no longer exist (values live in the bridge).
- Speed: ~0 again (PRF is 0.1% of host time in the profile).

## Exact optimisation found on the way (in patch 0007)
MemSubsystem::AxiLlcTableRuntime::make_write_req allocated and zeroed a row payload (and a chunk-enable vector) for each of
the four LLC tables on every one of the 16 AXI ticks per cycle even when the write was disabled; DynamicStorage::write_chunks
ignores a disabled request. Returning before the allocation: timer-mmio 92.0 G -> 78.8 G host instructions (-14%), cycles equal.

## Where the time is now (timer-mmio, 2,260 samples, was 8,200 at baseline)
front end 27.7% (icache_comb_calc, BPU), BackTop::comb 24.4% (IssueQueue/Isu ~94, TlbMmu 60), AXI+LLC ~32%
(Interconnect comb_outputs 12.4%, LLC::comb 10%), dcache 6.1%. Top of stack: memmove 16% (copy_regs_live prefix,
PreIduQueue::seq, IssueQueue assign), memset 5%.
Speed so far: 11.9 k -> ~42 k cycles/s (timer-mmio 206,120 cycles in ~4.9 s CPU).

## Not yet done (cache data)
Design for L1D/LLC data deletion: stores write pmem at ROB commit from a fact (word address + word after the step),
SimDDR stops writing pmem, LLC/ifetch/PTW read data from pmem; then the L1D/MSHR/WB/LLC payload members can go.
Expected speed effect small (the payload copies are already skipped for idle slots); value is code/state reduction.

# P3.3: cache data (patches 0008, 0008b)

Design: functional memory (pmem) is the only place that holds values; every other path carries tags/timing only.
- Stores update pmem at ROB commit from a fact (word address + the word after the step; AMOs included, a failed SC
  sets no store). SimDDR no longer writes pmem (axi_kit_ddr_functional_writes = false).
- LLC: the 4 MB data table is gone; a lookup synthesizes the set's data row from pmem using the meta/valid tables;
  every read response (hit, refill, forwarded) is built from pmem (extract_line_response).
- L1D: the data array is gone (DcacheConfig.cpp). A victim snapshot that may be written back is read from pmem.
- MemRouteBlock: instruction data and page-table entries that came back through the L1D read port now come from pmem
  (found by marking the L1D read data with 0xCAFEF00D and seeing it arrive as an instruction word).
- Results: 56/56 cycle counts equal to baseline; timer-mmio unchanged at 78.4 G host instructions (the payload members
  of the AXI/LLC/MSHR/write-buffer structs are still there and still copied).

Controls
- Discriminating tests added (tests/extra): smc-div / smc-addi (self-modifying code alternating `addi` and `div`).
  Original model: 8,996 / 8,866 cycles. Trace build: 8,996 / 8,866. With the commit-time memory write disabled
  (TB_NEG_NO_MEMWRITE): smc-div 8,867 (stale instruction), smc-addi unchanged. Before this step that control did not
  differ because the data caches still carried the values.
- The original model is kept as a baseline binary: third_party/tututugege/baseline-sim (worktree of 2f31e8d with the exact
  kit changes of P3a); it reproduces baseline.csv and gives the cycles of any new test.

# Regression limits (scripts/run_gate.sh)
A hang used to cost one external timeout per test. Now: SIM_MAX_CYCLES (main.cpp, exit code 3) = max(3000, 4 x baseline
cycles) per test, 8 jobs in parallel, one hard deadline of 20 s as a last resort. The 56-test gate takes ~24 s.

## Known limits of the cache-data deletion
- LLC mode "mapped scratchpad" (the LLC as RAM at 0x30000000) keeps data only in the LLC table: not supported.
- A/D bits written by the page-table walker are not mirrored into pmem.
- Payload members (WideReadData/WideWriteData in AXI structs, LLC queues, MSHR/WB lines) are still present: P3.4.

# P3.4a: one instruction record for every stage (patch 0009)

Before: 19 per-stage structs (InstInfo, MicroOp, DecRenInst, RenDisInst, DispatchInst, DisRobInst, DisIssUop, IqStoredUop,
IssPrfUop, PrfExeUop, ExuInst, ExePrfWbUop, ExuRobUop, ExeLsuReqUop, LsuExeRespUop, RobStoredInst, RobCommitInst, ...)
with ~17 field-by-field converters. Together they had 49 distinct fields; only `op` (UopType vs encoded wire) and
`type` (InstType vs encoded wire) were declared differently.
Now: `struct Uop` (back-end/include/types.h, 112 bytes, 49 fields); every stage struct is `using X = Uop;`,
the converters are identity members, op/type are enums everywhere (encode/decode are identity).
- Duplicate helper overloads (is_store/is_load/is_exception/is_flush_inst/is_page_fault/cmp_inst_age existed up to 6 times,
  for InstInfo, MicroOp and four IO structs) are one each. The MicroOp (op-based) variants had no callers.
- RobCommitExtra (src1_areg, imm) is filled directly in Rob.cpp.
- Code: 9 files, +60 / -1031 lines. 56/56 cycle counts equal to baseline; smc-div/smc-addi equal to the original model.
- Speed: timer-mmio 78.4 G -> 79.6 G host instructions (+1.6%): each double-buffered copy moves the full 112 bytes.
  Accepted for the simplicity; field trimming below can win it back.
Not done: shrinking fields that are only values (result/diag_val/ftq_next_pc/imm are still needed: MMIO store data and
load results, addresses, targets), and the AXI/LLC/MSHR/write-buffer payload members.

# P3.4b: AXI/LLC wide data payloads (patch 0010)
- WideReadData_t: 256 B -> 64 B (one cache line, 16 words); WideWriteData_t: 64 B -> 4 B (one word). operator[] clamps the
  index (junk alias past the kept words), the members are private so every direct `.words[]` access had to be reviewed
  (20 sites in AXI_Interconnect.cpp / AXI_LLC.cpp, now operator[]).
- WideWriteStrb_t is NOT a payload and stays at 64 B: the byte enables decide full-line vs partial-line writes in the LLC
  (read-for-ownership), i.e. they drive timing.
- First attempt kept one word for reads: every test hung (rc=3 from SIM_MAX_CYCLES within 32 s, dispatch=flush=3333):
  the CPU boots from the boot ROM at 0x0, an MMIO region, and its instruction words arrive as the line of an MMIO read
  response. One cache line covers it.
- 56/56 cycle counts equal to baseline (gate 17 s); smc-div/smc-addi equal to the original model.
- Speed: timer-mmio 79.6 G -> 76.0 G host instructions (-4.6%), ~4.8 s CPU, ~43 k cycles/s.
- Remaining payload-like members: AXI_LLC_Bytes_t vectors (table write payloads, built and dropped), DCACHE_WORD_NUM
  arrays in MSHR / write-buffer / FILLWrite / FillIn. The profile says the AXI/LLC share (~33%) is mostly logic
  (comb_outputs, LLC::comb, quiescence checks) run 16 times per cycle, not payload copies.

# P3.5: id-indexed uops (patch 0011)
Pipeline registers and IO ports no longer carry the whole instruction record. A `Uop` is now `id` + the fields that change
while the uop is in flight (56 B, was 112 B); everything written when the uop is created lives in a pool, `UopS` (64 B per
entry, `uop_pool[id]`), read as `u->imm`, `u->pc`, `u->type`, `u->dbg` ... (23 fields: pc, imm, ftq_*, dest/src areg,
src preg, old_dest_preg, enables/flags, func3/func7, br_id, csr_idx, is_atomic, type, dbg).
Why not "id only": the model is comb/seq double buffered (`x` committed copy, `x_1` next copy). If every field lived in one
shared table, `entry` and `entry_1` would alias, and a field changed in the next-state copy (br_mask, src busy bits,
completion masks, results, page-fault bits, ftq_is_last, is_mmio, ...) would be visible to readers of the current state.
Those stay in the entry; they are the dynamic part.
- Ids: the front end allocates a block of 4 pool entries when an instruction enters the instruction buffer (decode re-runs
  every cycle for a stalled instruction, so it must not allocate); the instruction owns [id], its dispatched uops use
  [id+1+k] (k <= 2: STA/STD, JAL/JALR, AMO), filled in Dispatch from the instruction's entry. Allocation is deterministic,
  so re-evaluating a comb stage allocates nothing. Ring of 65,535 entries (about 16 k instructions); an instruction that lives
  longer than that would be overwritten (not seen: the deadlock detector fires at 50,000 cycles).
- Entry 0 is a shared null entry for default-constructed handles (LSU write-back uops); a destructor aborts if it was written.
- Bug found by bisecting (all fields dynamic passed; fields moved back in groups): Rename wrote old_dest_preg / src pregs for
  every slot including invalid ones whose `inst_r[i]` still held the handle of an instruction already in the ROB, so the
  write hit a live entry (it used to hit a throw-away copy). Fix: write only valid slots. Stages that compute for all slots and
  write static fields through a stale handle are the thing to watch for.
- The rewrite was compiler driven: the static fields are reached only via `->`, so every access to a moved field was a compile
  error whose location was rewritten by script (/tmp scripts, not kept).
- Result: 56/56 cycle counts equal to baseline; timer-mmio 76.0 G -> 74.0 G host instructions (-2.6%), ~45 k cycles/s.
  The gain is small, as in the earlier RSD experiment; the structure now makes the pipeline easier to reason about
  (create-once vs mutable) and is the base for further trimming of the mutable part.

# Cleanup 1: profile infrastructure and statistics (patch 0012)
- Removed host_profile.{h,cpp}, frontend_stats.{h,cpp} and the 47 FRONTEND_HOST_PROFILE_SCOPE uses.
- PerfCount.h 1,404 -> 33 lines: only cycle / commit counters and the `sim-time(cycle)=` line the gate reads. 372 counter uses
  were removed compile-driven (delete the statement at each "no member named" error; multi-line ones and the stat helper
  functions by hand): MemSubsystem::sync_llc_perf, the L1D COUNT_* macros, the LSU count_*/sample_* helpers
  (RealLsuPerf.cpp keeps the behavioural part: the dcache-response timeout replay), the TMA blocks of Dispatch/Ren/Idu/PreIdu.
- Things that look like statistics but are behaviour: CONFIG_LSU_DRESP_TIMEOUT_CYCLES replay in RealLsuPerf.cpp. Kept.
- 56/56 cycle counts equal to baseline, smc-div/smc-addi equal; model source 60.9 k -> 58.1 k lines; timer-mmio 74.0 G -> 72.7 G
  host instructions (-1.8%).
Next on the list (see the discussion): pin the default front-end/back-end profile (drop small/medium/large and the dead #if
branches), then rewrite the ICache as tags + miss/refill timing only (its data array is dead since instruction words come from
pmem; ~5.5 k lines of bool-array kernel + glue).

# Cleanup 2: single configuration (patch 0013)
- Kept only the default profile: removed include/config.h.{small,medium,large,default} and
  front-end/config/frontend_feature_config.h.{small,medium,large,default}; the active headers are include/config.h and
  front-end/config/frontend_feature_config.h (they were byte-identical to the default files). Makefile lost PROFILE and the
  default/small/medium/large targets.
- The four profiles differed in more than numbers: sizes (widths, ROB/PRF, queues, predictor tables, latencies), the issue-port
  table (23 ports in default, 11/13/16 in the others), the TAGE sub-components (ENABLE_TAGE_SC_LITE / SC_L / SC_PATH /
  LOOP_PRED are 0 in default, 1 in the others) and CONFIG_ORACLE_STEADY_FETCH_WIDTH (defined only in default).
- Dead #if branches were pruned with `unifdef -m` (-D for the 136 integer macros defined in the config headers and not redefined
  elsewhere, -U for macros referenced in conditions but never defined, e.g. LOG_*_ENABLE, BSD_CONFIG, SRAM_DELAY_ENABLE,
  FRONTEND_ENABLE_*_BYPASS, CONFIG_LSU_DRESP_TIMEOUT_CYCLES). Conditions not fully decided by those symbols are untouched;
  debug macros of this project (IQ_SELFCHECK, TB_NEG_*) are kept. 27 files, -1,390 lines (TAGE_top.h 1,893 -> 1,519,
  front_top.cpp 2,095 -> 1,711).
- Correction: CONFIG_LSU_DRESP_TIMEOUT_CYCLES is not defined anywhere, so the dcache-response timeout replay in RealLsuPerf.cpp
  was dead code (it was kept in cleanup 1 for caution and is now gone).
- 56/56 cycle counts equal, smc-div/smc-addi equal, host instructions unchanged (72.7 G): dead code did not execute.
- Model source now 56.7 k lines (original 83.9 k, -32%).

## ICache S1: remove generated kernel and pack/unpack layers

- Deleted ICacheCompactKernel.h (2373 lines), ICacheLookupMetaPackKernel.h, ICacheTopTrueGlueKernel.h and the bool-array pack/unpack glue; `comb_core` / `eval_true_glue` (native path) are used directly.
- front-end/icache: ~7.9k -> 3.3k lines.
- Gate 56/56 equal to baseline; smc-div 8,996, smc-addi 8,866 (unchanged). Patch: patches/0014-icache-s1.patch.
- Next: S2 drop the ICache data arrays (fetch words from pmem), S3 simplify comb_core.

## ICache S2: drop the data table

- icache.cpp: removed the DataTable (way x 64 B per set) and its read/write/reset paths and the focus-dump helpers. On a tag hit the line words are read from pmem at (tag << 12 | index << offset).
- Gate 56/56 equal to baseline; smc-div 8,996, smc-addi 8,866. Patch: patches/0015-icache-s2.patch.
- Next: S3 simplify comb_core; refill still carries line words inside icache_module (finish_refill / table_write.data) and can be pruned next.

## ICache S3: refill carries no line data

- finish_refill reads the line from pmem (single-word uncached responses still use the AXI word); removed `table_write.data`, `dcache_probe_data_r`, `DcacheProbeStep::line` and the empty copy loops. The D-cache probe keeps only its timing (done/hit/word counter).
- front-end/icache: 3,150 lines. Gate 56/56, smc-div 8,996, smc-addi 8,866. Patch: patches/0016-icache-s3.patch.
- Left: `mem_resp_data_r` / `mem_resp_data_w` still hold a full line (needed only for the uncached word); can shrink to one word. comb_core control flow (state machine, perf_state) not yet simplified.

## AXI idle-tick skip

- The 15 extra AXI-subsystem ticks per core cycle (FREQ_DIV 16, plus the L1D fast domain) are cut short: after tick 2, if interconnect+LLC, DDR, MMIO bus and the L1D fast domain (RealDcache stage regs, MSHR, write buffer, response meta) are all quiescent, the rest are fixed points and `break`. Predicates: `idle_quiescent()` in AXI_Interconnect/SimDDR/MMIO_Bus/MSHR/WriteBuffer/RealDcache, `dcache_domain_quiescent()` in MemSubsystem.
- Exactness: gate 56/56, smc-div 8,996, smc-addi 8,866. `AXI_IDLE_VERIFY=1` runs all ticks anyway and checks that quiescence persists and the boundary state is unchanged: 249,766 checks over the gate, 0 mismatches. (Verify checks quiescence + CPU-visible boundary only, not the full internal state; the gate is the full check.)
- Speed (timer-mmio, 206,120 cycles): 4.06 s (all ticks) -> 2.89 s, i.e. ~51k -> ~71k cycles/s, 1.4x. Patches: 0017-axi-idle-skip.patch, 0017b-kit-idle-quiescent.patch.

## BPU copy/clear cleanup

- bpu_comb_calc no longer builds a BpuCombOut (10.6 KB zeroed + two 10 KB copies); the core writes OutputPayload/UpdateRequest directly, and front_top passes `bpu_seq_txn_req.req` in place.
- BpuNlpCombIn held a by-value ReadData (13 KB copy per cycle); it now holds const references. Unused BpuCombIn/BpuCombOut removed.
- GHR/FH history update is done in place (`tage_hist_update_inplace`, memmove shift) instead of 6 copies of the 256-byte GHR per branch.
- Result: gate 56/56, smc 8,996 / 8,866 unchanged. timer-mmio user time (min of 5 alternating runs): 2.673 s -> 2.554 s (-4.5%); host instructions 38.26 G -> 37.27 G. Smaller than the profile share suggested: BPU memset/memmove fell to ~7% of samples; the rest of the BPU cost is real predictor logic. Patch: 0018-bpu-copies.patch.
- Next largest copy: PreIduQueue::seq (~6% of samples), IssueQueue vector assign (~3%), SimCpu::cycle copies (~2%).

## PreIduQueue: per-slot sync instead of whole-queue copy

- seq() used to copy the whole 128-entry instruction buffer (~3 KB) and all 64 FTQ entries + train-meta (~tens of KB) whenever anything changed. Now `InstructionBuffer::sync_from` copies only slots touched by push/pop (a clear touches all), and the FTQ copies only the slot allocated this cycle (`ftq_alloc_mask`) plus the 64-byte valid array. Entry payloads only change at allocation, so this is exact.
- Gate 56/56, smc 8,996 / 8,866. timer-mmio user time (min of 5 alternating): 2.560 s -> 2.414 s (-5.7%). Not run: an IQ_SELFCHECK build (the committed==working memcmp). Patch: 0019-preidu-sync.patch.
- Next copy targets: IssueQueue vector `assign` (~3% of samples), SimCpu::cycle copies (~2%), front2back_FIFO_seq_write, Ren::seq.

## IssueQueue: per-slot entry sync

- seq() assigned the whole `std::vector<IqStoredEntry>` whenever any slot changed. Writes to the working copy now mark the slot (`mark_slot`: wakeup, enqueue, issue/flush removal, br_mask clear); seq copies only marked slots (flush_all copies all).
- Gate 56/56, smc 8,996 / 8,866. IQ_SELFCHECK build (committed==working memcmp for IssueQueue and PreIduQueue every cycle) also 56/56, no aborts. timer-mmio user time 2.414 s -> 2.343 s (-3%). Patch: 0020-iq-slot-sync.patch.
- Cumulative since the AXI skip (2.89 s wall / 2.658 s user at the profile): 2.343 s user.
- Remaining copy targets: SimCpu::cycle copies, front2back_FIFO_seq_write, Ren::seq, BPU comb_bpu/seq_read memsets, AXI copy_regs_live.

## Front-end prune (round 1 of "write it compactly, only function + cycles")

- unifdef with the effective macro values removed dead branches (2-ahead/NLP, TAGE-SC-L/loop/path, ideal ICache, SRAM latency model).
- Removed the falcon/front stats counters (the summaries were already stubs), the DEBUG_LOG* calls, SIM_DEBUG_PRINT blocks and the focus-dump functions, the never-instantiated SimpleICacheTop and BitWriter/BitReader, and unused train_IO.h aliases/structs.
- Front end 13,678 -> 12,644 lines. Gate 56/56, smc 8,996 / 8,866 after each step. Patch (3 commits): 0021-frontend-prune.patch.
- Estimate check: my "-2-3k" guess was optimistic; the dead code was smaller than assumed. The remaining size is live structure: BPU 5.5k (algorithm + comb wrappers), front_top 1.4k, ICache ~3k, per-stage IO structs.

## BPU wrappers (round 2 of the compact rewrite)

- The 2-ahead / NLP predictor was compiled out (ENABLE_2AHEAD undefined) but its state, tables, comb function and the saved_2ahead / mini_flush plumbing were still there: removed from BPU (state, ReadData/UpdateRequest fields, bpu_nlp_comb), and the always-false mini-flush / 2-ahead outputs removed through front_top, front_IO, PTAB (dummy entries) and train_IO. Exact because every one of those values was constant (false / 0) in the active build.
- History and update-queue comb wrappers inlined: `bpu_hist_comb(inp, rd, type_out, req)` and `bpu_queue_comb(inp, rd, req, out)` now read the input/snapshot and write `req` directly (generated by a script from the pack/unpack lines), removing 4 wrapper structs and ~150 lines of memcpy/field copies.
- BPU.h 2000 -> 1439 lines, front end 12,644 -> 12,095. Gate 56/56, smc 8,996 / 8,866 at every step. timer-mmio user time 2.328 -> 2.291 s (-1.6%, small but in the right direction). Patch (3 commits): 0022-bpu-wrappers.patch.
- Left in BPU: PreReadReq / PostReadReq / SubmoduleBind / PredictMain wrappers (their outputs fan out to ReadData and UpdateRequest, so the mapping is not 1:1 and needs manual merging), the per-bank TAGE/BTB comb/seq wrappers in TAGE_top.h/BTB_top.h.

## BPU wrappers, remaining ones

- Merged `bpu_pre_read_req_comb` + `bpu_data_seq_read` into `bpu_data_read(inp, rd)` (address/RAS/queue read stage writes rd directly); inlined the post-read, submodule-bind and predict-main wrappers: the functions now take `inp`/`rd` plus the real intermediate values (type_out, tage_out, btb_out) and write `out`/`req` directly. 8 more wrapper structs and their pack/unpack code removed (hist/queue were done in round 2). Input mapping was derived mechanically from the old pack code; outputs of predict-main map `out.out.X -> out.X`, everything else -> `req`.
- BPU.h 1439 -> 1239 lines; front end 12,095 -> 11,886. Gate 56/56 and smc 8,996 / 8,866 at each step. Patch (3 commits): 0023-bpu-wrappers2.patch.
- Left in BPU: TAGE_top.h (1.5k) / BTB_top.h (1.4k) / TypePredictor.h (0.4k) carry the same In/Out/CombResult wrapper style and are the predictor algorithms themselves.

## front_top glue

- The BPU input used to be copied four times (front_top_in -> BPU_in seed -> control-in -> BPU_in -> BPU_TOP::InputPayload); it is now built once. The global-control, read-enable and read-stage-input glue functions and their six In/Out structs were inlined as plain boolean expressions. front_top.cpp 1,318 -> 1,137 lines, front end 11,886 -> 11,643. Gate 56/56, smc 8,996 / 8,866. Patch: 0024-front-top-glue.patch.
- Not done, and why: ptab_write / checker_input / front2back_write / output comb functions copy the *prediction metadata bundle* (pcpn, altpcpn, tage_idx/tag[4], sc_*, loop_*) field by field between near-identical structs (BPU output, PTAB_in/out, front2back_in/out, front_top_out, FTQ entries in the back end). Inlining them only saves ~60 lines. The real win is to define that bundle once (`PredMeta`) and let the FIFOs store/copy it as a unit, which touches the PTAB/front2back FIFO modules and the back-end interface (one-step-at-a-time with the gate).

## PredMeta: the prediction metadata is one struct

- `PredMeta` (wire_types.h) holds the 14 per-lane fields that travel with a fetch group (alt_pred, altpcpn, pcpn, tage_idx/tag[TN_MAX], sc_used/pred/sum/idx, loop_used/hit/pred/idx/tag) plus `mask()` for the hardware widths. It replaces the field-by-field declarations in PTAB_in/PTAB_out/PTAB_entry, front2back_FIFO_in/out/entry and BPU::OutputPayload (renamed out_* -> meta.*), and every copy loop in PTAB.cpp, front2bank_FIFO.cpp and front_top.cpp is now `dst.meta = src.meta` (+ `mask()` where the old code masked; masking is idempotent). Also removed the dead BPU_in / BPU_out structs and the lane-width static_asserts.
- Front end 11,643 -> 11,379 lines. Gate 56/56, smc 8,996 / 8,866. timer-mmio user time 2.309 -> 2.204 s (-4.5%, struct copies replace per-field loops). Patch: 0025-predmeta.patch.
- Not unified: the BACKEND_FETCH_WIDTH-wide front_top_out (flat arrays, copied once in front_output_comb), the COMMIT_WIDTH-wide update path (front_top_in, BPU InputPayload, QueueEntry) and the back-end FTQ train-meta; same field list, different width/direction, would need a width template.

## Paging tests: the bridge was not translation-aware (found and fixed)

- New gate members: the four upstream paging tests (vm-asid / vm-pagefault / vm-sfence-perm / vm-sum-fetch), built by scripts/build_vm_tests.sh into build/vm/ (clang, -fgnu89-inline); baseline cycles 68,737 / 29,412 / 38,695 / 2,694 (baseline model: all Success). run_gate.sh now runs 60 tests.
- Before the fix the trace model FAILED three of them and hung the fourth (3 of 4 "translated store did not update backing physical word", rc=3). Cause: TraceBridge applied its RAM-range checks and the store undo address to the *virtual* address (and to the virtual pc), so every access through a mapping outside the identity range was treated as "blocked" and never executed, and undo/redo used the wrong word.
- Fix: new core API `refcpu_translate_data(ctx, vaddr, type, &paddr)` (no side effects, same decision as exec.cpp: identity when satp is bare or the effective privilege is M, otherwise va2pa; type 0 = fetch ignores MPRV). The bridge uses the physical address for in_ram / misalign / undo, and treats a page fault as "execute and let the core raise it" instead of blocked. The fact's `vaddr` stays virtual because the model's own TLB/PTW translates it.
- After: all four paging tests Success with cycle counts identical to the baseline; the 56 original tests, smc-div/addi (8,996 / 8,866), dhrystone (3,911,612) and coremark x3 (323,962) unchanged. Patches: 0027-bridge-translation.patch, 0027b-refcpu-translate.patch.
- Consequence for the TLB/PTW rewrite plan: the gate now covers translation-on paths (page walk, ASID, sfence, SUM, permission faults), so the native rewrite of TlbMmuKernel / PtwCompactCoreKernel can be validated.

## TLB / PTW bit-vector kernels removed

- TLB: TlbMmu already had native `core_comb()` / `top_comb()`; the runtime went through `encode_*_bits -> tlb_kernel_*_io_generator -> decode_*_bits` (TlbMmuKernel.h, 1,547 lines, plus ~300 lines of codec in TlbMmu.cpp and the width constants in TlbMmu.h). Verified native vs kernel with a temporary TLB_NATIVE_VERIFY compare over all 60 gate tests (9,440 non-trivial calls incl. 2,684 in the four paging tests, 0 mismatches), then switched `run_top_via_io_generator` to `top_comb(in, entries)` and deleted the kernel, codec and constants. TlbMmu.cpp 1,551 -> 1,080, TlbMmu.h 410 -> 307, TlbMmuKernel.h deleted: -2.1k lines total. The bare-mode fast path and its verify code went with it (the native path is cheap enough: timer-mmio user time 2.226 -> 2.061 s vs the profile binary).
- PTW: `PtwCompactCoreKernel.h` (1,157 lines) and the packed encode/decode + generator wrappers in MemPtwBlock.h were not reachable from the runtime (no callers outside the header); removed. MemPtwBlock.h 1,236 -> 674: -1.7k lines total.
- After: gate 60/60 (56 original + 4 paging tests), smc 8,996 / 8,866, dhrystone 3,911,612, coremark x3 323,962 all identical to the baseline. Patches: 0028-tlb-native.patch, 0029-ptw-kernel-removed.patch.

## Top level and back-end IO.h

- rv_simu_mmu_v2.cpp 1,365 -> 1,051: removed the model-side difftest hooks (dead since P3: no register file, enable_difftest always off in trace mode; the interrupt hook that feeds tbhook::on_interrupt is kept as `run_interrupt_hook`), the config banner, the unused internal-tick helpers, and the empty TMA/stat branches of commit_sync. main.cpp / harness use of the reference core is unchanged.
- back-end/include/IO.h 1,025 -> 711: 44 IO structs had constructors that only zeroed every field with loops; replaced by `= {}` member initializers (same zero state). Gate 60/60, smc 8,996 / 8,866, dhrystone 3,911,612, coremark x3 323,962; timer-mmio user time unchanged (2.073 s). Patches: 0030-top-cleanup.patch, 0031-io-initializers.patch. 8 IO structs with non-trivial constructors were left as they are.

## Back-end: PredMeta by width

- `PredMetaT<N>` (wire_types.h; `PredMeta = PredMetaT<FETCH_WIDTH>`) with `assign_prefix()` for narrower-to-wider copies is now used by front_top_out, FrontPreIO and the FTQ train meta (BACKEND_FETCH_WIDTH = 16): the front-to-back glue in rv_simu, PreIduQueue's FTQ write and the front_output_comb lane copies became struct copies (about -150 lines). Gate 60/60, smc, dhrystone, coremark identical; timer-mmio 2.035 s. Patch: 0032-predmeta-width.patch.
- Survey of the rest of the back end (script: structs with >= 8 fields, pairs sharing >= 8 field names): the only remaining overlaps are the COMMIT_WIDTH commit-side bundle (front_top_in vs FtqCommitInfoResp, same 14 names but per-lane scalar vs lane arrays; ~50 lines) and StqEntry/LdqEntry (9 names). The back-end bulk is genuine logic written in RTL style (Dispatch 714, Rob 761, RealLsu 1,529, Lsu total 5.6k, Exu 1.8k), not duplicated declarations.

## Exu: table-driven FUs

- AbstractFU / SingleCycleFU / FixedLatencyFU / IterativeFU plus AluUnit, MulUnit, DivUnit, AguUnit, SduUnit, BruUnit, CsrUnit and FPUSoftfloat (impl_compute was a no-op for ALU/MUL/DIV/CSR/FPU) are one class `Fu` with a `FuKind` (ALU, MUL, DIV, AGU, SDU, BRU, CSR, FPU) and three timing modes: SINGLE (one cycle), PIPE (fixed-latency pipeline: MUL), ITER (one at a time, latency by op: DIV, FPU). The only compute left is what the facts replace: AGU address, SDU store data, and the BRU taken/target check against the FTQ prediction. `Exu::init` builds the units from a (kind, op-mask) table instead of nine copy-pasted blocks.
- Exu directory 1,804 -> 1,514 lines (AbstractFU.h, FPU.h deleted, Fu.h 167 -> 220 incl. everything). Gate 60/60, smc 8,996 / 8,866, dhrystone 3,911,612, coremark x1/3/10 identical; timer-mmio user time 2.228 -> 2.020 s (the vtable calls are gone). Patch: 0033-exu-fu-table.patch.
- What remains in Exu: Csr (586 lines: trap entry/return, delegation, vectored traps, interrupts; real state that decides redirect pcs and the TLB's satp/mstatus view) and Exu.cpp (~600 lines: issue-port / FU orchestration, writeback collection, branch arbitration).

## Lsu round: stats/debug leftovers and dead values

- Step 1 (patch 0034): RealLsuPerf.{h,cpp} removed (mem-latency/wait-start statistics were empty shells; the 8 tiny replay setters were inlined; `load_miss_mask` kept as a static helper; perf fields and the *_PERF_FIELDS macros removed). All dump_debug functions removed (RealLsu 160 lines, TlbMmu, MemSubsystem/RealDcache/MSHR/WriteBuffer/MemRouteBlock/MemPtwBlock, front_dump_debug_state, icache_dump_debug_state), plus the deadlock/diff trace infra (DeadlockDebug, DeadlockReplayTrace.h, DiffMemTrace.h: no producers existed) and the TLB trace ring (record_trace, trace_watch_vaddr, ITLB focus prints). The ROB deadlock Assert stays. -1.15k lines. Gate 60/60, big programs identical.
- Step 2 (patch 0035): the LSU value path was already dead: STLF `extract_data(stq data)`, dcache load-response data, `align_store_data` for the dcache store port, `LoadResp.data`, and the two helpers are gone; STQ `data` stays because MMIO stores (peripheral model) and the MMIO load patch use it, `LdqEntry.result` stays for page-fault tval and MMIO. Cycles unchanged on gate 60/60, smc, dhrystone, coremark x1/3/10; TB_POISON run still passes.
- Not done (next candidate): the dcache line-data chain is still alive but dead in effect: StoreReq.data -> dcache store slots / bank-retry data -> MSHR merged_store_data + fill_write.data merges (apply_strobe) -> WriteBuffer entry data -> AXI write wdata. Nothing reads these values any more (cache data arrays and LLC data were removed in P3), so they can go in one compile-guided sweep.
- Model total after this round: 43,437 lines.

## dcache line-data chain removed

- Removed every carrier of cache-line / store values below the LSU: StoreReq.data, dcache store slots and bank-retry data, MergeReq/MSHRReq data+strb, FillReqMerge, MSHR merged_store_data/strb and axi_resp_hold_data and the MSHR fill data (`MSHR_FILLReq.data`, `FILLWrite.data`, `MshrAxiIn.resp_data`), DirtyInfo/FillIn data_snap (the pmem victim-line reads), DcacheLineReadResp.data, BypassResp.data, WriteBufferEntry.data and WbAxiOut.req_wdata, the DcacheRespMeta data/strb, `apply_strobe`, `write_dcache_line`'s data parameter, and the rv_simu copies of R data into the MSHR and of WB data into the AXI write port. What stays: tags/valid/dirty/PLRU, store strobes (byte-overlap check), `merged_store_dirty`, merge/bypass valid + busy handshakes, and the peripheral (MMIO) write data.
- Exactness: gate 60/60 (56 + 4 paging), smc 8,996 / 8,866, dhrystone 3,911,612, coremark x1/3/10 identical; memory-stress (not in the gate, ~1.5M cycles) 1,541,195 cycles = baseline model's 1,541,195 (run once on both for this change). Model total 43,293 lines. Patch: 0036-line-data-chain.patch.

## memory-stress-small joins the gate (61 tests)

- tests/extra/memory-stress-small.c: the upstream memory-stress with ROUNDS 16 -> 2 (same 3 x 64 KiB arrays, so the same L1D conflict / dirty-writeback / MSHR pressure), expected values recomputed on the host. Baseline model and trace model both take 189,128 cycles (1,081,439 instructions, 147,460 loads, 114,688 stores); trace model 3 s. The gate pattern now excludes only the big `memory-stress/` directory. baseline.csv gets the new row; gate = 61 tests, all pass.
- Controls: the gate compares cycles, so the test is only as sensitive as its timing. TB_PERTURB_ADDR=3 (every 3rd load/store address has bit 12 flipped) gives 189,167 cycles instead of 189,128, i.e. the dcache/memory timing is observed (every 50th is too weak: unchanged). TB_NEG_NO_MEMWRITE does NOT fail this test: data values come from the functional core, and the pmem writes at commit only feed the model's own fetch/PTW reads, so a broken store path would show up as a cycle change, not as a wrong checksum.

## The model no longer has a memory of its own for code, page tables or line data

- New `tbhook::mem_read(paddr)` (Bridge::read_word): the functional word as the core sees it (RAM via refcpu_load_word, boot ROM via the core's io words). Instruction words (ICache glue, MemRouteBlock icache response) and page-table entries (MemRouteBlock PTW events) are read from the core's memory instead of pmem. Gate 61/61 including smc-div/addi (8,996 / 8,866) and the paging tests: no cycle changed, so the earlier worry (the core memory is "ahead" of commit) does not show up in these tests; it can only matter for a PTE/instruction store that is in flight while it is read, which is architecturally unordered without sfence.vma / fence.i.
- The commit-time `pmem_write` from facts is gone; nothing reads RAM from pmem any more. AXI/LLC read data (`extract_line_response`), the LLC data row and the ICache line words (`rd_data`, `data_resp_line`, `mem_resp_data*`, `lookup_hit_data_w`) were zero-filled and removed; the ICache module now reports `ifu_resp_paddr` and the glue builds the fetch group from `mem_read`.
- Checks: gate 61/61, smc 8,996 / 8,866, dhrystone 3,911,612, coremark x1/3/10 identical, memory-stress 1,541,195 identical; timer-mmio user time 2.02 s. Model 43,235 lines. Patches: 0037-core-memory-reads.patch (2 commits).
- What still uses pmem: the program loader (image -> pmem -> copied into the core at start, `refcpu_sync_ram_from_dut`), the peripheral / MMIO device registers (they live in pmem's IO window), the boot-ROM stub and UART init writes in BackTop. SD-card DMA writes land in pmem only (the difftest sync that used to forward them was already off in trace mode): a gap for workloads that read the DMA'd data, none of the gate tests do.

## One RAM: pmem is the core's memory

- `pmem_init()` creates the core (`refcpu_init`) and `p_memory` / the SDRAM pointer are the core's own arrays; `pmem_read/write/memcpy` on RAM addresses therefore act on the core memory directly. The loader (`cpu.back.load_image` -> `pmem_memcpy_to_ram`) and SD-card DMA writes (`pmem_write`) land in the core, the staging copy (`refcpu_sync_ram_from_dut` of the first 1 MiB) is gone, and `tbhook::ensure()` takes the core from `pmem_core_ctx()`. Only the IO window (device registers) is still model-side state (`g_io_words`); the boot ROM words are seeded into the core's io words as before. This also closes the DMA gap noted in the previous step.
- RUN mode no longer creates the difftest reference core when injection is on (`difftest_init_run_state` skipped): it was another full RAM + SDRAM that was never used.
- Effect: resident memory of a tiny test 4.8 GB -> 0.54 GB, startup of `add` 0.24 s -> 0.04 s; timer-mmio user time 2.01 -> 1.90 s. Gate 61/61, smc 8,996 / 8,866, dhrystone 3,911,612, coremark x1/3/10 and memory-stress 1,541,195 unchanged. Patch: 0038-single-ram.patch.
- Not touched: CKPT / REF_ONLY / fast-forward modes still use the difftest reference core as before (they were not exercised by the gate).

## Functional-memory leftovers removed

- SimDDR no longer reads or writes pmem (`do_memory_read/write` and `axi_kit_ddr_functional_writes` deleted; its read data is zero), `PeripheralModel` / `MemSubsystem` lost the `memory` RAM pointer (it was only a null guard), and the AXI read payload is one word (`kWideReadKeptWords` 16 -> 1: only the MMIO / peripheral read value travels; the boot-ROM words now come from `tbhook::mem_read`). The write payload was already one word. Gate 61/61, smc, dhrystone, coremark x1/3/10, memory-stress unchanged. Patch: 0039-memory-leftovers.patch.
- What is left that looks like memory but is not RAM: the device registers (IO window, `g_io_words`, used by PeripheralModel / the MMIO bus legacy backing line), STQ `data` / LDQ `result` (MMIO store data and the MMIO load patch), and the LLC's per-line byte plumbing (`AXI_LLC_Bytes_t` rows, `refill_data`, `apply write to line`: all carrying zeros). The last one is woven through AXI_LLC.cpp (3.3k lines) and goes away with the LLC rewrite, not before.

## AXI/LLC line-data plumbing removed (patch 0042)

LLC line-byte helpers, table data payload assignments, `victim_data` / `victim_wb_data_r` carriers removed
(MMIO bypass 1-word data kept). Timing signals (table enable/write/index/way) kept. Kit axi_interconnect
now ~18.1k lines incl. headers (cpp+h). Gate 61/61; smc-div 8,996, smc-addi 8,866, dhrystone 3,911,612,
coremark10 976,652, memory-stress 1,541,195 — all equal to baseline.

## AXI/LLC/DDR coverage-driven pruning (patch 0043)

Method: build with `-fprofile-instr-generate -fcoverage-mapping` (BUILD_DIR=build_cov), run the 66 gate/vm/dhrystone/coremark3
programs, `llvm-cov export` to JSON, then (1) delete zero-count functions that nothing references (iterating on
undeclared/undefined-symbol errors), (2) empty zero-count `{...}` blocks (region start `{`, end `}`), (3) re-run coverage,
delete the functions orphaned by (2), (4) remove `if (pure-cond) {}` leftovers (conditions with calls kept: a side-effecting
`if` removal broke all 61 tests). Result: AXI_Interconnect.cpp 3406->~2.3k, AXI_LLC.cpp 2588->~1.5k, SimDDR 
~416, route_shape.h 757->364; kit sources+headers ~5.9k lines (was ~10.9k). Gate 61/61, smc/dhrystone/coremark10/memory-stress
equal to baseline; timer-mmio 1.83 s user (unchanged). Caveat: pruning is only as safe as test coverage; features not
exercised by the 66 programs (UART MMIO path, prefetch, invalidate, mapped window) are gone/empty.

## BPU rewrite + differential test (patch 0044)

Coverage-based pruning was rejected (it only says "not exercised by these programs"), so the BPU was rewritten
and proven equivalent with a differential test instead.
- Analysis: TAGE/BTB/TypePredictor were single-cycle in practice. The S_STAGE2 states, SRAM-delay PRNG, input/pipeline
  latches, tage_done/btb_done, SC/loop predictor (outputs hard-wired 0), path history and the 256-entry GHR copies were
  dead for observable behaviour. Quirks kept on purpose: BTB gets the bank-local pc (so its fallback target is bank_pc+4),
  TAGE/BTB update from the queue head while the type predictor updates from the commit stream, the update queue drops
  (one push per bank per cycle, update_queue_full is always 0), reset leaves TAGE/BTB arrays untouched.
- New front-end/BPU/BPU.h (615 lines, namespace bpuf): Tage/Btb/TypePred (predict/update -> write-set, commit), Hist
  (128-bit GHR + folded histories), Ras, Bpu::comb() (pure, returns Txn) / commit() / reset(). BPU dir 4,754 -> 764 lines.
- tests/bpu_ref = frozen old RTL-style BPU_TOP; tests/bpu_diff.cpp drives both with identical structured random stimulus
  (synthetic branch pool, realistic metadata replay from earlier predictions, rare reset/refetch) and memcmp's every output
  every cycle; scripts/bpu_diff.sh <sim> [cycles] [seeds]. 30M cycles over 6 configs (pool 100..30000 pcs, span 1k..64k words): 0 mismatches.
- Harness validity: 18 deliberate mutations of the new code; 16 caught, 2 survivors are equivalent/unobservable (type conf only
  feeds an unused output; an 8-bit BHT value >>18 vs >>19 is 0).
- Gate 61/61; smc/dhrystone/coremark10/memory-stress equal to baseline; timer-mmio 1.83 -> 1.35 s user (-26%).

## Lsu rewrite + lockstep verification (patch 0045)

RealLsu.cpp/.h 1,617 -> 971 lines, same externally visible behaviour (all phases, same call order from BackTop).
- Six hand-written index queues (wait_mmu_ldq/stq, mmu_done_stq, finish, stlf_queue, wait_dcache_ldq) plus ~14
  enqueue/dequeue/trim helpers became one `Ring<E,N>` template (push/pop/trim/clear/window). Holes (consumed entries behind a
  valid head) are kept on purpose: they occupy window slots and change timing. Unused load states, FenceUnit, byte_mask,
  stq_wait, alloc_* wrappers, diagnostic-only code removed; enum values kept stable.
- Verification is a lockstep shadow, not coverage: with `LSU_SHADOW=1` the build links tests/lsu_ref (frozen original LSU,
  renamed RealLsuRef) and BackTop's `LSU(phase)` macro runs every phase on both with the same inputs, comparing the phase's
  output struct and a canonical state digest (all LDQ/STQ fields, every queue incl. ring head/count, uncached/LR-SC units,
  req_gen). scripts/lsu_shadow.sh builds it and runs the gate. Old-vs-old gave 61/61 (no false positives); new-vs-old: gate 61/61,
  dhrystone, coremark3, memory-stress(+small), smc-*: no mismatch, every phase digest equal.
- Mutation check of the verifier (21 deliberate bugs in the new code): 17 caught (incl. vm paging tests for TLB retry), 1 equivalent
  (`x-- > 1` == `--x > 0`), 3 survived because no program reaches the path (MSHR_FULL wakeups beyond LSU_LDU_COUNT=4, SC held behind an
  overlapping load, MMIO store surviving a flush). Bare-metal tests built for them never got there (probe: at most 4 MSHR_FULL
  waiters; SC-hold and flush-with-MMIO-on-bus counters stayed 0), so LsuShadow.cpp has directed state-injection scenarios
  (LSU_SHADOW_DIRECTED=1): identical hand-built states go into the new and the reference LSU, one phase runs on both, digests
  must match. All 5 scenarios match and all 3 surviving mutants are now caught (20 of 21 caught, 1 equivalent).
- Gate 61/61 and dhrystone/coremark10/memory-stress/smc equal to baseline; speed unchanged (A/B timer-mmio 1.34 vs 1.34 s).

## Back-end speed round (patch 0046)

Profile (dhrystone, `sample`, ~10k samples, self time): back-end ~48% (IssueQueue 9.4, Dispatch 6.9, Exu 6.6, Rob 6.1, Isu 5.2, Lsu 3.4, Fu 3.0 ...),
memmove+memset ~13% spread over many callers (SimCpu::cycle, front_comb_calc, FIFO seq, Ren).
- Rob: comb_begin/seq copied the whole 128-entry array twice per cycle; now a dirty bitmap (touch() at every entry_1 write site,
  touch_all() on flush) copies back only written slots. Rob 6.1% -> 1.6% of samples; timer-mmio 1.33 -> 1.27 s, coremark3 2.64 -> 2.56 s.
- IssueQueue: ready bitmap refreshed for dirty slots in seq(); schedule() iterates set bits instead of scanning slots (same order). ~0.5% faster.
- Gate 61/61 and dhrystone/coremark10/memory-stress/smc equal to baseline after each change.
- What is left is diffuse: IssueQueue::comb_issue (3.9%), AXI copy_regs_live (3.5%+1%, LLC regs bulk copy several times per tick),
  Exu::comb_begin (2.6%, per-FU slot copy), Dispatch::is_preg_woken (2%), per-cycle struct copies of 1-5 KB (FrontUpdateRequest 4.6 KB,
  FrontPreIO 0.9 KB, LsuState 8.6 KB cur=nxt) - each worth 1-2%.
- LLC (round 2 of the speed work): `io.table_out = {}` in AXI_LLC::comb freed/allocated 8 vectors per call; replaced by clear() that keeps
  the storage (same state as `= {}`), and MemSubsystem no longer fills a 1 KiB zero data row per LLC lookup (only `data_valid` is read).
  timer-mmio 1.26 -> 1.24 s, coremark3 2.56 -> 2.49 s. Cumulative over the back-end speed work: timer-mmio 1.33 -> 1.24 s (-7%),
  coremark3 2.64 -> 2.49 s (-6%). Not done: Dispatch::is_preg_woken wake bitmap (needs the exact comb ordering of prf/iss wake inputs),
  Exu::comb_begin per-FU copies, per-cycle 1-5 KB front/back struct copies (each <=1-2%).

## comb_begin copies removed (patch 0047)

Every `X_1 = X` copy in Idu/Ren/Dispatch/Isu/Prf/Exu/Csr::comb_begin was redundant: seq() copies X_1 back into X, so the two are equal at the next
begin. back-end/include/BeginCheck.h turns each copy into `BEGIN_EQ(X_1, X)`, which is a no-op normally and a memcmp+abort with
`-DBEGIN_SELFCHECK`. The self-check build ran gate 61/61, dhrystone, coremark10, memory-stress, smc and AXI_IDLE_VERIFY with no violation, so
the copies are gone. Speed effect ~0 (timer-mmio 1.24 -> 1.23 s): these structs are small; it is a simplification, not a speed-up.
Not touched: PreIduQueue (ftq heads), the per-FU `units[i]->comb_begin()` and the output-default clears in Exu::comb_begin (those are needed).

## Correction: coverage-driven emptying reverted (kit commits c707481, 237bdf6, 652645e)

The 0043 round emptied every block no test executed. The 66 programs never evict a dirty line from the 8 MiB LLC, so this also gutted the
DDR write path (AW/W/B handling), victim write-back handling and more. With a 64 KiB LLC (`-DCONFIG_AXI_LLC_SIZE_BYTES=65536`) the emptied
kit never finishes memory-stress-small (hang), while the reverted kit gives 189,639 cycles = the original simulator built the same way
(also equal on dcache-conflict-writeback 868 and dcache-mshr-pressure 193). Gate/big-program numbers were identical before and after the
revert, which is exactly why the gate could not see it. What stays from 0043: deletion of functions that nothing references
(2bbb1a3) and the allocator-free `table_out` reset. Lesson: an unexecuted branch is not a dead branch. scripts/llc_small_check.sh builds both
simulators with the small LLC and compares cycles; the AXI rewrite will be verified against this full kit, not the emptied one.

## SimDDR rewrite (patch 0048; verified against the full kit, not the emptied one)

SimDDR.cpp+.h 771 -> 257 lines, identical behaviour. The write data FIFO keeps only a beat count (no addr/data), WriteTransaction/ReadTransaction lost their
unused address fields, the three Read arbitration branches that all choose Read are one, w_active/w_current (redundant with the queues) are gone.
- Standalone differential test (tests/ddr_diff.cpp, scripts/ddr_diff.sh): random AXI masters (protocol-correct W bursts, bounded outstanding writes, read
  pause windows so writes win the backend) drive the new SimDDR and the frozen original (tests/ddr_ref); every output and a state digest compared each cycle.
  11 configurations x 1.5M cycles (hundreds of thousands of read and write beats, 10^5 B responses): 0 mismatches. 8 deliberate mutations (drain keep, turnaround,
  B aging, WREADY gating, retire condition, round-robin, ARREADY, drain-enter watermark): all caught.
- In-simulator lockstep (`gmake DDR_SHADOW=1`, tests/ddr_ref/DdrShadow.*): gate 61/61 and small-LLC runs match, no mismatch. The integrated system alone could not reach several DDR paths
  (outstanding 32, write watermarks, turnaround), so the standalone test is the real coverage.
- Gate 61/61, big programs equal to baseline; scripts/llc_small_check.sh (64 KiB LLC, real DDR writes) equal to the original simulator.
- Speed: timer-mmio 1.23 -> 1.29 s after the revert of the emptying (restored branches run again); the DDR itself is a tiny share.

## Back-end tidy, round 1 (patch 0049, cycle-exact, no behaviour change)

- Removed the identity `Uop` converters (`from_*`, `to_*`) and the 14 stage-specific aliases (`DecRenInst`, `DisIssUop`, `ExuRobUop`, ...): every stage
  passes the same `Uop`. The frozen Lsu reference was updated mechanically; the lockstep gate still passes.
- `BackTop::comb()` is now a short list of seven named stages plus the fire/recover stage, each a small method with a comment saying what it
  produces; the call order is unchanged and is the dependency order.
- Gate 61/61 and dhrystone/coremark10/memory-stress/smc equal to baseline.

## Back-end tidy, round 2: Wires (patch 0050, cycle-exact)

All inter-module signals of the back-end (46 IO objects plus the top-level in/out) now live in one `struct Wires` (back-end/include/Wires.h); `BackTop` derives from it
and every module reads/writes them through `wires->name.field`. Gone: the ~100 `module->in.x = &x` / `->out.x = &x` lines in BackTop::init, ten per-module
pointer-only `In`/`Out` structs and their members; accesses were rewritten by script (`in.x->f` -> `wires->x.f`), which the compiler checked; the one wire owned by a
module (Idu's branch latch) is a pointer in Wires. The Lsu lockstep reference and the directed scenarios were adapted (they build a Wires of their own); lockstep gate
61/61 and the 5 directed scenarios still match. Back-end 11.0k -> 10.9k lines (the saving is less in lines than in indirection). Gate 61/61, dhrystone/coremark10/
memory-stress/smc equal to baseline.

## Rob rewrite + lockstep (patch 0051)

Rob.cpp 761 -> 364 lines, same phases, same members (entry/entry_1/pointers, dirty-slot sync), same behaviour.
- Commit arbitration restructured: head-line scan merged into one loop, the always-true `group_commit_mask`/`head_line_empty` conditions dropped (the mask equals
  the valid bits), single-commit decision in three short steps, the special-instruction broadcast as one if/else chain, "advance head if the line drained" as a lambda
  shared by the group and single paths; comb_complete uses port ranges instead of per-port loops; ~110 lines of debug dumps (DUP-CPLT, deadlock, DBG_PRINTF) became Asserts
  with short messages.
- Verification: `gmake ROB_SHADOW=1` links tests/rob_ref (frozen original Rob, trace-bridge calls disabled so it does not report twice) and BackTop's `ROB(phase)` macro runs
  every phase on both, each with its own Wires (inputs copied from the real ones before each phase, outputs compared, plus a digest of the pointers and every valid entry).
  Old-vs-old 61/61 (rob_commit is compared only in comb_commit because later stages patch diag_val in place). New-vs-old: gate 61/61, dhrystone, coremark3,
  memory-stress(+small), smc: no mismatch.
- Mutation check (14 deliberate bugs): 10 caught by the programs, 2 more (flush_pipe accumulation, STA store-fault clearing the load fault) by directed scenarios
  (`ROB_SHADOW_DIRECTED=1`), 1 is unobservable here (`front_stall` is constant 0 because the BPU update queue never reports full) and 1 is caught by the gate itself (EBREAK exit);
  not exercised by any program or scenario: a fence at the head while a committed store is still draining (fence gating).
- Gate 61/61, dhrystone/coremark10/memory-stress/smc equal to baseline.

## Dispatch rewrite + lockstep (patch 0052)

Dispatch.cpp 712 -> 372 lines, same phases/members/behaviour.
- `decompose_inst` (about 240 lines of per-instruction case blocks) is a small table: each uop is {queue, op, overrides for src1/src2/dest enable, imm=4, atomic}; the one data-dependent
  case (store data of a non-SWAP AMO reads the load's destination) is patched after the loop. comb_fire lost the dead "block reason" bookkeeping (never read); the
  CSR/atomic serialization check is one condition; the roll-back loop and port-owner loops are shorter.
- Verification: `gmake DIS_SHADOW=1` links tests/dis_ref (frozen original) and BackTop's `DIS(phase)` macro runs every phase on both with separate Wires; outputs (dis2ren/iss/rob/lsu),
  a digest of the registered and working state, and the shared uop-pool slots the real Dispatch wrote for issued uops (the reference would overwrite them) are compared.
  Old-vs-old 61/61; new-vs-old: gate 61/61, dhrystone, coremark3, memory-stress(+small), smc: no mismatch.
- Mutation check (15 deliberate bugs): 10 caught by the programs, 2 more by directed scenarios (`DIS_SHADOW_DIRECTED=1`: an older slot writing x0 must not keep a source busy; a full issue
  queue stops the group), 3 are equivalent (LDQ limit `<=` is shadowed by the port-valid check, the JALR jump's src1 is already enabled, the LSU br_mask is already cleared in comb_alloc).
- Gate 61/61, dhrystone/coremark10/memory-stress/smc equal to baseline.

## Exu rewrite + lockstep (patch 0053)

Exu.cpp 577 -> 306 lines (and ExuIn/ExuOut pointer structs gone: the FU request/response buses are plain members), same phases and behaviour.
- The FU specs table, `comb_begin` defaults, per-port ready masks, CSR request extraction, pipeline slot fill, the four execution sub-stages and the result collection are the
  same logic with the redundant scaffolding removed: `fu_global_idx` (= loop index), per-stage asserts that restated invariants, the winner-pointer bookkeeping, repeated
  flush/kill conditions (one `dead` flag), two copy-pasted LSU write-back blocks (one lambda), the mispredict arbitration as one condition.
- Verification: `gmake EXU_SHADOW=1` links tests/exu_ref (frozen original with its own FUs; its trace-bridge exec observation is disabled so it does not report twice) and BackTop's
  `EXU(phase)` macro runs every phase on both with separate Wires; the six output buses (exe2csr/prf/iss/lsu, exu2id, exu2rob) and a digest of the FU input slots and the FU request/response
  buses are compared. Old-vs-old 61/61; new-vs-old: gate 61/61, dhrystone, coremark3, memory-stress(+small), smc: no mismatch.
- Mutation check (15 deliberate bugs): 12 caught; 3 survive: `is_br_killed` with `== br_mask` (equivalent while the broadcast mask is one-hot), dropping `|| fu_ready` from the
  port ready mask, and a squashed branch still driving recovery (needs a branch completing in the cycle it is flushed). Those paths were transcribed line by line, not dynamically proven.
- Gate 61/61, dhrystone/coremark10/memory-stress/smc equal to baseline.

## Isu rewrite + lockstep (patch 0054)

Isu.cpp 525 -> 247 lines, same phases/members/behaviour.
- The three delayed-wakeup structures (MUL pipeline, DIV and FP slots) are handled through shared helpers: `clear_wake_state()`, a `for_each_wake_slot_1` visitor (used by mispredict kill and
  branch-bit clearing), one `age` lambda for DIV/FP countdown and one `put`/`free_slot` pair for allocation; the wakeup channels of `comb_awake` are driven through one `drive` lambda.
  The slot placement stays outside `Assert` (it must run in NDEBUG builds).
- Verification: `gmake ISU_SHADOW=1` links tests/isu_ref (frozen original with its own issue queues) and BackTop's `ISU(phase)` macro runs every phase on both with separate Wires; iss2dis/iss2prf/iss_awake
  and a digest of every queue (committed and working entries, counts) plus the wakeup state are compared. Old-vs-old 61/61; new-vs-old: gate 61/61, dhrystone, coremark3, memory-stress(+small), smc: no mismatch.
- Mutation check (15 deliberate bugs): 8 caught by the programs, 2 more by directed scenarios (`ISU_SHADOW_DIRECTED=1`: a flush clears pending DIV wakeups; a DIV without destination starts none), 5 survive and look
  equivalent in this configuration (MUL pipeline depth is 1, so the shift and last-stage mutants change nothing; STA ports are not ALU ports; Dispatch already applies wakeups to entering uops).
- Gate 61/61, dhrystone/coremark10/memory-stress/smc equal to baseline.

## Idu rewrite + lockstep (patch 0055)
Idu.cpp 505 -> 302 lines: `decode()` uses one `set(dest,s1,s2,type)` helper per opcode and a CSR known-list function; comb_decode/comb_fire compacted. Reference in tests/idu_ref (IduRef + IduShadow, `IDU_SHADOW=1`), compares dec2ren, dec_bcast, idu_consume, decoded uop-pool slots and a digest of now_br_mask / br_mask_cp / tag_vec / br_latch.
- Verified: gate 61/61 under lockstep with no mismatch; smc-div, smc-addi, dhrystone, coremark3, memory-stress(-small) lockstep clean; normal binary: coremark10 976652, dhrystone 3911612, memory-stress 1541195, smc 8866/8996.
- Mutation check (16 mutants): lockstep catches 9 (tag clear, flush reset, mispred free, checkpoint clear, LR src2, DIV/MUL, FENCE.I, bcast mask, LUI src1). The 6 decode survivors (sip in CSR list, time/timeh illegal, AMO rd=0 -> areg 32, fp unary 0x70, is_ret imm, sfence rd) are instruction forms no test program contains; `tests/idu_diff.cpp` compares `decode()` new vs ref over all opcode x funct3 x funct7 x register patterns, all 4096 CSR numbers and 20M random words: 0 mismatches, and it catches all 6 mutants.
- Tag-exhaustion stall: unreached with 8 tags/cycle, so both copies were patched to allocate at most 2 tags/cycle (stalls then occur, quick-sort etc.); lockstep clean. The `page_fault_inst=false` mutant is equivalent (u = {} already clears it).

## Ren rewrite + lockstep (patch 0056)
Ren.cpp 400 -> 220 lines: single-loop free-list allocation with in-order stall, one `lookup()` lambda for src1/src2/old-dest with same-group bypass (replaces nine per-slot arrays), commit statistics in `count_commit()`, memcpy checkpoints, compact comb_pipeline. Reference in tests/ren_ref (RenRef with the commit side effects - perf counters, commit/interrupt hooks, logging - removed so they run once; RenShadow). Compares ren2dis (valid, uop, shared uop-pool slots), ren2dec and the full state (inst_r, RATs, checkpoints, free list, pointers) per phase, working and committed.
- Verified: gate 61/61 under lockstep, smc/dhrystone/coremark3/memory-stress(-small) lockstep clean; normal binary cycle counts unchanged (coremark10 976652, dhrystone 3911612, memory-stress 1541195, smc 8866/8996).
- Mutation check (19 mutants): 13 caught. Survivors: `stall` after failed alloc and bypass-from-x0 (equivalent: the decoder clears dest_en for x0), checkpoint read from `_1` copy (equivalent: a tag is never re-checkpointed in the cycle it is used), clear_mask in the ready path (redundant: Idu already cleared it), and two paths no test reaches (probe-verified over all 66 tests + big programs): reclaim dropped when a commit coincides with a mispredict (`free_tail_1 = free_tail`), and an exception instruction with dest_en (old dest not freed). Both kept statement-for-statement from the original.

## Csr rewrite + lockstep (patch 0057)
Csr.cpp 420 -> 222 lines: interrupt evaluation with two small lambdas (to_m / to_s), table-driven interrupt injection, named delegation flags and a `trap_target` lambda in comb_exception, one `write_pair` helper for the mie/mip/mstatus aliases. Reference in tests/csr_ref (CsrRef without the satp trace record, CsrShadow: outputs csr_status/csr2exe/csr2rob/csr2front and full state per phase; the machine state is adopted once at the first phase because it is also loaded from outside).
- Verified: gate 61/61 under lockstep, smc/dhrystone/coremark3/memory-stress lockstep clean, normal cycle counts unchanged.
- The tests reach few trap/S-mode paths, so `tests/csr_diff.cpp` drives new and ref with random machine state (dense-bit words, all privilege levels) and random events (ecall, faults, illegal, mret/sret, interrupt responses and injection, CSR writes of every kind) for 20M cycles: 0 mismatches. 27 of 28 mutants are caught by it (the survivor reads `_1` in comb_csr_read, which equals the committed value at that point: equivalent).

## PreIduQueue rewrite + lockstep (patch 0058)
PreIduQueue.cpp 351 -> 245 lines: empty `if` bodies and unused ctx hooks removed, FTQ helpers compacted, shared PC-lookup path, commit-info metadata copy via a local reference, `ftq_alloc() < 0` branch (provably dead: ready already required a free slot) turned into an Assert. Reference in tests/pre_ref (PreIduQueueRef whose uop allocation is a stub, PreShadow as `friend` of the live class for the private state): compares pre_issue (uop ids masked), pre2front, FTQ PC/commit-info responses, instruction buffer and FTQ state per phase.
- Verified: gate 61/61 under lockstep, big programs lockstep clean, normal cycle counts unchanged.
- 26 mutants on the lockstep subset: 6 survived (page_fault_inst, next_pc, sc_idx offset, rob lookup entry_valid, FTQ-full ready, metadata cursor invalidation) because the programs never produce those situations. `tests/pre_diff.cpp` drives live + reference with random blocks, consumption, flush/mispredict, commits and lookups (millions of cycles, FTQ/buffer full and empty regimes): clean, and it catches all six. Two further mutants are equivalent (head-pointer no-op, the dead alloc branch).

## BackTop tidy (patch 0059)
BackTop.cpp 850 -> 366 lines. Image/flash/SD loading and checkpoint restore moved to BackTopLoad.cpp (422 lines, unchanged logic); the nine `<MOD>_SHADOW` macro blocks moved to include/ShadowHooks.h; `restore_from_ref` and `restore_checkpoint` share `install_arch_state()`; the diagnostic IO clear and the output/redirect/commit-group code are `clear_stage_ios()` and `comb_outputs()`. BackTop is glue, so there is no lockstep for it: verified by cycle equality (gate 61/61, coremark10/dhrystone/memory-stress/smc unchanged) and by a build with all nine shadows enabled at once, 61/61 with no mismatch (DEADLINE=300, it is slow). Checkpoint restore itself is not exercised by the gate (compiled and unchanged apart from the shared helper).

## LLC rewrite + random differential (patch 0060, kit repo commit 67a6b81)
AXI_LLC.cpp 2538 -> 1467 lines (header private section reorganised). The three giant functions are split by case: `try_complete_lookup` -> `complete_direct_lookup / complete_hit_lookup / complete_write_miss / complete_read_miss`; the write path -> `on_mem_write_resp`, `try_commit_write_after_victim`; the refill commit shares `externalize_read_victim` / `release_mshr`. Repeated blocks became helpers: `start_lookup`, `end_lookup`, `retry_lookup`, `write_line_tables`, `put_meta / put_repl / begin_write`, `promote_read_response`, `arm_victim_writeback`, `any_read_victim`. Removed: the debug-focus macros and trace hooks (all compiled out), empty perf-count functions, empty `if` bodies, and checks made redundant by `pick_new_read_master` / `pick_new_write_master` (same register set, same conditions).
Behavioural note: `end_lookup` clears all lookup flags, the original cleared a subset in some paths; the flags are only read while `lookup_valid_r` and every `start_lookup` rewrites all of them, so nothing observable changes (the differential masks them for an invalid lookup).
- The LLC is a standalone state machine (upstream ports, memory port, table commands/answers), so verification is a closed-loop differential, `tests/llc_diff.cpp` (`scripts/llc_diff.sh`): the live class and the frozen original (`tests/llc_ref`) each run the same random simulation (per-master sticky requests with bypass / direct-mapped / full-line and partial writes, random ready/latency memory model with write-response codes, table model, invalidate-all (also in drain windows) and invalidate-line, sequential streams that train the prefetcher, one-MSHR configs, 1-4 ways, tiny sets so conflicts and dirty evictions are constant). Every cycle all ext_out fields, every table command and a hash of the live register state must be identical. Harness reach on the original source: ~91% of the lines.
- Result: 12M cycles over 6 seeds, plus 6M with `CONFIG_AXI_LLC_DCACHE_READ_MISS_NOALLOC=1` (the no-allocate path): 0 mismatches. Mutation check on the new code (38 mutants): 32 caught; 6 survive and are equivalent or unreachable by protocol (memory answering an invalid MSHR index; second queue promote and the read-queue term of the quiet check, which the queue invariant makes redundant; table-busy guard of the post-victim commit, never busy at that point; same-line skip in the hit-write victim loop, excluded by the lookup's MSHR check; the by-address owner fallback for a victim write-back, kept verbatim from the original).
- Simulator: gate 61/61, coremark10 976652, dhrystone 3911612, memory-stress 1541195, smc 8866/8996 unchanged; small-LLC check (64 KiB, dirty evictions): memory-stress-small 189639, dcache-conflict-writeback 868, dcache-mshr-pressure 193, memory-boundary 323 identical to the baseline.

## Interconnect tidy + random differential (patch 0061, kit repo)
AXI_Interconnect.cpp 3373 -> 2771 lines. This is a protocol bridge with a lot of coupled hand-written state, so it was tidied in verified steps rather than rewritten wholesale: (1) debug-focus macros, trace hooks, no-op `if` bodies, the read timeout monitor (it only touched its own counters) and eleven write-only members (`ar_*_c`, `aw_port_c`, `r_current_master`, `req_drop_warned`; the simulator's `read_req_drop_warned` snapshot went with it) were removed; (2) `seq()` (900 lines) is split into `seq_read_ar / seq_llc_dispatch / seq_read_data / seq_llc_write / seq_direct_write / seq_finish` with the pre-`seq` samples in `SeqPrev`; the two identical tails became `seq_finish`; (3) shared helpers: `to_llc_read_req / to_llc_write_req`, `start_downstream_write` (two copies of the current-transaction + AW-latch setup), `issue_master_read` (three copies of the AR meta setup), `first_free_id`, `mode2_ddr_aligned`; (4) compact forms of the port accessors, same-line hazards, external hazards, quiescence predicates, id-conflict check and pending-queue finders. One redundant term (`!request_in_mapped_window` inside the mode-2 aligned flag, already excluded by `request_uses_ddr_port`) was dropped.
- Verification: closed-loop differential `tests/ic_diff.cpp` (`scripts/ic_diff.sh`): live and the frozen original (`tests/ic_ref`) run the same random simulation - four read and two write masters with sticky requests (DDR, direct-mapped window, MMIO incl. unsupported sizes, bypass, partial/full writes), a randomised AXI slave per downstream port (random ready/latency, multi-beat bursts, per-port memory mirror), LLC tables, invalidate-all/line and mid-run mode changes (0/1/2, which trigger the reconfiguration flush). Every cycle all upstream and downstream outputs, the LLC table commands, accepted/ready pulses, active mode and `idle_quiescent()` must be identical. About 90% line reach on the original source. Result: 9M cycles over 6 seeds, 0 mismatches.
- Mutation check on the tidied code (22 mutants, 600k cycles each, then 6 more seeds for survivors): 15 caught immediately; survivors are redundant or unreachable here (per-master pending-read cap of 32 never reached, AW id exclusion of a latched AW already covered by the active write, `w_current` same-granule check covered by `w_pending` outside the LLC path, line-invalidate gating against a same-line write stage, non-LLC quiescence terms during mode changes); an AW-latch-size mutant that the first mutation script put in the wrong place was re-run correctly and caught at cycle 166.
- Simulator: gate 61/61, coremark10 976652, dhrystone 3911612, memory-stress 1541195, smc 8866/8996, small-LLC check identical to the baseline.
