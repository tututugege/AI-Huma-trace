# Front end as modules (exp/global-sched, fm1..fm6)

> **Historical record.** The current rules and tools are in [SCHEDULING.md](SCHEDULING.md); this file records how they came about (tools and the q / d register pairs named here may no longer exist).

The front end is written like the back end: modules with `struct In` (const pointers, named after the wires they read)
and `struct Out` (the wires they drive), phases as methods, ports bound once. FrontComb (the per-cycle working struct
whose members were the wires and whose methods were the phases) and FrontUpdateRequest are gone.

| module (header) | RTL | phases | Out |
| --- | --- | --- | --- |
| FrontControl (front_ctrl.h) | front_ctrl.sv | control, enables, finish | global_reset global_refetch pipe_dec pipe_head_ok the read enables ptab_can_write ptab_bypass front_state (out: FrontTop::out) |
| BpuPipe (BPU/BPU_pipe.h) | bpu.sv | comb_front | bpu_output bpu_can_run bpu_seq_txn |
| FetchAddrPort / InstPort / PtabPort / F2bPort (front_fifo_ports.h) | the four FIFOs | comb_read, comb_write | each FIFO's in / read data / head / edge request; InstPort also the fixed head (multi, grp_*, rvc_*, carry_low, grp_ptab) |
| ICacheTop (icache/icache_top.h) | icache | comb_fetch, comb_invalidate | ic_in ic_out ic_k |
| FrontChecker (front_checker.h) | predecode_checker.sv | comb | checker_out predecode_flush flush_address pd_next lh_* |

* FrontCore's constructor binds every module input to the output of the module that drives it; `bind()` (from
  FrontTop::init) adds the front end's inputs, registers and output and the context. Nothing is rebuilt per cycle.
* Each FrontCore phase is one module call (`void FrontCore::control() { ctrl.control(); }`); SimCpu::cycle() calls them
  through FrontTop in the generated order; FrontCore::comb() (init / step_bpu) runs the same order.
* FrontCore::seq applies the modules' edge requests (`fa_port.out.fetch_addr_fifo_req`, ..., `bpu_pipe.out.bpu_seq_txn`,
  `ctrl.out.front_state`).
* A module's first phase of the cycle clears its outputs' valid / enable bits; a writer that sets valid drives the
  whole entry.

## Contracts and tooling

The phase contracts are above class FrontCore (front_core.h), in wire names (the modules' port names), with the three
FIFO groups (`CONTRACT fifo`). The front scan (comb_domains.front_domain) takes the wires from the modules' Out fields
and scans each module phase's own body with `in.X` / `out.X` / `*in.X` / `in.X->` read as the wire X, a FIFO port's
`fifo->` as its storage. gen_front_order.py --emit prints the scanned lines, --write regenerates FrontCore::comb(),
--check compares; gen_top_order.py places the front phases in the global order.

## Verified

Every step (fm1..fm6) identical on: gates RVC=0 62/62, RVC=1 67/67; FPR x4; TB_TRACE_COMMIT and LSU_PIPE_EVT md5; the
Linux checkpoint (3M commits); FE_PAIR=1 / FE_IPF=1 on the RVC gate (fact excluded).
