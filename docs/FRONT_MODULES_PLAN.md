# Front end as modules (plan, exp/global-sched)

Goal: the front end written like the back end. Each module has its comb phases as methods, `struct In` (const
pointers) / `struct Out` (what it drives), CONTRACT lines above its class, ports bound in FrontCore::init; SimCpu::cycle()
calls the phases (gen_top_order.py). FrontComb (today: the cycle's working state, its members are the wires, its
methods the phases) goes away.

## Modules (as the RTL: rtl/rtl/frontend)

| module | phases (today's FrontComb code) | Out (today's FrontComb members) |
| --- | --- | --- |
| FrontCtrl (front_ctrl.sv) | control; enables (read_fifos: read enables, FIFO in-control fields, PTAB bypass); heads (fix_heads); finish | global_reset global_refetch pipe_dec pipe_head_ok fifo_read_enable f2b_read_enable multi grp_in_range grp_seg_first rvc_n rvc_straddle carry_low ic_k out req.front_state |
| BpuPipe (bpu.sv) | run_bpu | bpu_output bpu_can_run req.bpu_seq_txn (bpu_pipe.txn) |
| FetchAddrFifo | read port (read_fifos); write (write_fetch_addr) | fetch_addr_head fetch_addr_in fetch_addr_rd req.fetch_addr_fifo_req req.ipf |
| InstFifo | read port; write (write_instruction_fifo: predecode, RVC carry, FE_PAIR second block) | fifo_head fifo_in fifo_rd req.fifo_req |
| Ptab | read port; write | ptab_head ptab_in ptab_rd req.ptab_req |
| F2bFifo | read port; write (write_front2back) | f2b_head f2b_in f2b_rd f2b_bypass use_f2b_bypass req.front2back_fifo_req |
| ICacheTop | run_icache | ic_in ic_out |
| PredecodeChecker (predecode_checker.sv) | check_group | checker_out predecode_flush flush_address pd_next lh_new_* lh_updated |

## Steps (each: identical gates RVC=0/1, FPR x4, TB/LSU prints, FE_PAIR / FE_IPF, Linux checkpoint; commit)

1. Tooling: the front domain scanned like the back end (comb_graph back machinery parameterised: wires = FrontComb's
   members while it exists, module In/Out rewritten to them), front contracts in the back form
   (`CONTRACT <phase> in: <wires> out: <wires>`), gen_top_order reads them like the back end's.
2. One module at a time, smallest first: PredecodeChecker, ICacheTop, F2bFifo write, FetchAddrFifo write, BpuPipe,
   InstFifo write, the read ports + FrontCtrl. A moved module's outputs move from FrontComb into its Out; FrontComb
   keeps a reference to them until no FrontComb phase reads them.
3. FrontComb removed; FrontCore::comb (init) calls the module phases; docs.
