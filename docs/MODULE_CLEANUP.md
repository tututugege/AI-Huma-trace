# Module clean-up: TLB, CSR, PhysMemory, Dispatch, Rename, ROB, LSU (exp/global-sched, after NEXT_STATE.md)

Ten commits on top of main 83af731. Cycles identical at every step (both gates, FPR, TB / LSU traces, the Linux checkpoint,
FE_PAIR / FE_IPF: `vbin.sh`; the 13 Linux windows against the RTL: 13/13, fingerprints byte-identical to the previous
step). Source .cpp/.h: 30070 -> 29096 lines.

## TLB (back-end/Lsu/TlbMmu.*, MemSubSystem/include/PtwWalkPort.h)

* Owned by their users: `RealLsu::dtlb`, `ICacheTop::itlb` are members (no pointer, no lazy creation, no null checks);
  their edge is in the owner's seq. `TlbMmu` has no SimContext.
* One `comb(flush, cancel, req[], resp[], n)` and `seq()`: a flush (table cleared at the edge), a cancel of the walk (a
  response that has arrived is still taken), then the request ports in order. Requests `{valid, vaddr, type}`, responses
  `{valid, result (OK / FAULT / RETRY), paddr}`. Gone: the lookup / core / top layers and their eight structs, the five
  entry points (translate, translate_lsu_ports, flush, cancel_pending_walk, run_control_via_top), the walk / refill shadows,
  the entry / wait index echo, the result conversions. `peek_translate` (FE_IPF) shares the lookup and the permission check.
  Kept as before: the DTLB takes an arrived walk response before translating, the ITLB does not; the refill of a walk is
  visible to the later requests of the same cycle.
* Walk port: the TLB writes `PtwWalkPortCombIn` (request, response taken, flush; the walker reads and clears it) and reads
  the walker's client outputs latched at the edge (`MemPtwBlock::dtlb_q / itlb_q`). The virtual `PtwWalkPort`, its
  MemSubsystem adapter, `PtwWalkPortSeqIn` and `sync_ptw_port_outputs` are gone.

## CSR file (Csr.cpp, include/Csr.h)

* `SimCpu::csr`, read everywhere through `csr_state()`; only SimCpu writes it (`csr.seq()` after the back end, before the
  devices; `install_arch_state`). The interrupt lines are a SimCpu wire. The csr_status ports (LSU, front end, I-cache,
  Back2Front, the LSU's TLB request) and the ROB's / EXU's CSR pointers are gone.
* The CSR write happens at commit from the ROB (`RobCsrIO`: the committing CSR instruction's CSR, command and the value of
  its fact; the interrupt response; FS-dirty). The execute-time latch, `ExeCsrIO` and `TopCsrIO` are gone (a shadow compare
  of the latch against the commit-time write: 0 differences on both gates, the Linux checkpoint, the 13 windows).

## PhysMemory (removed)

The reference core (whose RAM is the model's memory) is created / released / cleared by driver/RefCore; the device
registers are `IoBacking` (`io_read / io_write`: the MMIO window's words, used by the peripheral model, the AXI MMIO path
and UART, the loaders, the core sync); the memory-map predicates are in mmio_address_map.h.

## Dispatch

One pass in slot order: busy bits, then up to the first slot that does not fire: its uops, whether they fit, whether it
fires; only fired slots place their uops (nothing is taken back). `out.dis2rob.uop` is the slot's working copy. `DisLsuIO`
carries the pool id. The wake-up check uses only the wake bus (it carries the load write-backs).

## Rename

`RenDomain<NA, NP>` (RAT, checkpoints, free-list ring) instantiated for the integer and the FP registers; the integer
domain's move-elimination reference count stays in Ren. The phase wires are members. The commit statistics and the
interrupt report moved to the ROB.

## ROB

One scan of the head line per cycle (flush-type instruction, oldest incomplete; also used by the PMU and the head
broadcast). Commit uops are copied for committing slots only. `comb_redirect` = `redirect()` + `report_commit()`.

## LSU

* LSU_MS_DATA state beside the queues (`MsLdq / MsStq`, cleared at allocation; `ms_capture_stores`,
  `ms_load_written_back`).
* LDQ entries read the load's ROB entry, destination, funct3 and pc from its uop's pool entry (`PLdq::uop()`); STQ
  entries keep their copies (a committed store outlives its pool entry's guarantee).
* Long steps split and named: `stq_retire / watchdog / print_events / wake_loads / fill_replay_registers`,
  `l2_decide / dcache_load_responses / l3_register`, `drain_countdown / mmio_store_issue / wb_drain`.
* RealLsuPipe.h opens with how to read pcur / pnxt / pw and a table of the names.

## Not done

* LSU: a declared pcur / pnxt read per step with a debug-build check; removing pnxt (the steps' same-cycle dependences would
  each become a named wire).
