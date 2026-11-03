# P0: where the baseline's host time goes (measured)

Binary: upstream 2f31e8d, default profile, -O3, macOS arm64, sampled with `sample` (1 ms), get-task-allow signed copy.
Workloads: timer-mmio (206,120 cycles, 17.3 s; sampled 10 s after 3 s) and matmul_fpu (42,253 cycles; 2 s).
Raw samples: results/baseline/p0_*.sample.txt.

## Headline
- timer-mmio: 202.3 G host instructions retired for 206,120 cycles = ~981k instructions per simulated cycle; RSS 4.8 GB.
- Startup is ~0.3 s (wfi-boundary: 151 cycles, 0.34 s), so the cost is per cycle, not startup.

## Top of stack (timer-mmio, ~8.2k samples)
| share | function |
|---|---|
| ~68% | `_platform_memmove` (5567 samples) |
| 2.6% | `_platform_memset` |
| ~1% each | AXI_LLC::comb, prepare_llc_inputs, llc_path_quiescent, icache_compact_kernel, IssueQueue::comb_begin ... |

memmove by caller: AXI_LLC::comb() 3498, AXI_Interconnect::seq() 1564 (together 91% of memmove);
the rest is IssueQueue vector assigns (109), PreIduQueue (154), others (<50 each).

## Inclusive time by subsystem (children of SimCpu::cycle, timer-mmio)
| subsystem | samples | share |
|---|---|---|
| shared AXI interconnect + LLC + DDR (comb_outputs, LLC::comb, seq, llc_seq, comb_inputs, ...) | ~6,400 | ~78% |
| backend (BackTop comb+seq) | ~970 | ~12% |
| frontend (front comb+seq) | ~615 | ~7.5% |
| dcache fast path | ~220 | ~2.7% |

matmul_fpu gives the same split (memmove 71% of top-of-stack; same callers). FPU/softfloat functions: 0 samples.

## Cause
`AXI_LLC::comb()` starts with `io.ext_out = {}; io.table_out = {}; io.reg_write = io.regs;` and `AXI_Interconnect::seq()`
copies the registered state back. AXI_LLC_Regs_t holds wide-data queue arrays (read-response queues, write queues,
victim write-back queue) and the table requests carry byte vectors, so each cycle copies/zeroes tens of KB (not measured
exactly) for state that rarely changes. Same pattern as the RSD "array old-value memcpy in tick".

## Consequences for the plan
1. Deleting value logic (FPU, ALU, PRF data) removes ~0% of host time here: P3 steps 1-2 are not the speed lever.
2. The first speed step is the LLC/AXI state: stop whole-state copy (in-place update / dirty-only), drop data payloads
   (timing needs tags, valid, repl, MSHR, queue occupancy and addresses, not line data). Exactness is held by the gate.
3. After that the remaining ~25%: backend 12%, frontend 7.5%, IssueQueue/PreIduQueue vector copies.
4. Caveats: two workloads, one sample window each; sampling attributes inlined code to the caller; the LLC share on
   workloads with fewer memory accesses may differ. Re-measure after each step with instructions retired.

## Profile after ICache S1-S3 + AXI idle skip (timer-mmio, 2 s of `sample`, 1,178 samples; one run)

Inclusive, children of SimCpu::cycle: front_comb_before_back 38.4%, BackTop::comb 38.2%, BackTop::seq 15.3%, AXI comb_outputs 7.0% (LLC::comb 5.6% inside), AXI seq 3.1%, comb_inputs 2.2%, front_seq_write 3.3%.
Top of stack: `_platform_memmove` 303 (26%), `_platform_memset` 116 (10%), copy_regs_live, front_comb_calc 52, IssueQueue::comb_begin 40.

Front end (415 samples under front_comb_calc): BPU_TOP::bpu_comb_calc 245 (20.8% of total; bpu_core_comb_calc 146, memset 30, bpu_submodule_seq_read 27, memmove 18), bpu_seq_read 32, icache_comb_calc 20 (1.7%), PTAB/FIFOs ~20. The BPU is now the largest single block; the ICache is small.
Back end: Isu::comb_issue 89 (7.6%), TlbMmu::translate_lsu_ports 50 (4.2%), Isu::comb_begin 40, Exu::comb_begin/comb_exec/fu2exu_collect 32/30/24, Rob and Dispatch ~10 each.
memmove callers: PreIduQueue::seq 88 (7.5% of total), IssueQueue vector `assign` 41, front_comb_calc 31, SimCpu::cycle 24, TAGE/BTB/BPU copies ~40. memset callers: BPU (bpu_comb_calc 30, bpu_seq_read 27, comb_bpu 23, core 14).

Takeaways: (1) whole-struct copy/clear (memmove+memset ~36% top-of-stack) is again the main cost, now spread over BPU payload structs, PreIduQueue::seq and IssueQueue assigns; (2) BPU ~30% inclusive; (3) AXI+LLC fell from ~78% to ~15%.

## Profile after the front-end rewrite rounds (timer-mmio, 2 s sample, 1,313 samples; one run)

Inclusive (children of SimCpu::cycle): BackTop::comb 50.1%, front_comb_before_back 30.5%, AXI comb_outputs 7.3% (LLC::comb 4.3%), BackTop::seq 4.6%, front_seq_write 3.7%, AXI comb_inputs/seq 2.7/2.6%.
Back end: Isu::comb_issue 114 (IssueQueue::comb_issue incl. schedule), TlbMmu::translate_lsu_ports 73 (all in the bit-vector TLB kernel round trip), Isu::comb_begin 71 (IssueQueue::comb_begin), Exu::comb_exec/begin/collect 37/33/25, Dispatch::comb_dispatch 25, Isu::comb_enq 24, Rob::seq 21.
Front end: BPU core 151 (11.5%), bpu_submodule_seq_read 35, icache 28, bpu_seq_read 27 (memset).
Top of stack: memmove 173 (13%), memset 93 (7%).

Done from this profile (patch 0026): IssueQueue scheduling used three heap vectors per call per IQ per cycle and zeroed the whole enqueue/grant payloads in comb_begin -> scratch buffers + clear only `valid` (-3.3%); TLB top: when no valid port needs translation (M-mode or satp bare) and no flush/cancel, skip the bit-vector kernel (encode -> generator -> decode) and return the identity result directly (-6.4%). The fast path was checked against the kernel with TLB_FAST_VERIFY=1: 456,429 checks over the gate, 0 mismatches (field-wise compare). Note: the 56-test gate has no paging test, so the translation-on paths are unchanged code, not newly validated.
Combined: timer-mmio user time 2.271 -> 2.063 s (-9%), ~100k cycles/s.
