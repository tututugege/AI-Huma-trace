// SimCpu: the tops (front end, back end, memory subsystem, AXI) wired together, and one cycle of the whole core.
#include "SimCpu.h"
#include "Csr.h"
#include "PhysMemory.h"
#include "RealLsu.h"
#include "config.h"
#include "trace_hook.h"
#include "util.h"
#include <cstdint>

// Environment actions at commit (not ports of a module; the RTL testbench does the same, rtl/docs/modules/soc_top.md):
// Ren calls them through SimContext while it commits. A committed store to a modeled device (CLINT timer / timecmp,
// ...) takes effect in the peripheral model at once: its interrupt line is read by csr.comb_interrupt_inject later in
// the same back.comb. An interrupt taken at commit is reported to the trace bridge.
void SimCpu::commit_sync(InstInfo *inst, int commit_slot) {
  BackTop *back = &this->back;
  Assert(commit_slot >= 0 && commit_slot < COMMIT_WIDTH &&
         "SimCpu::commit_sync: invalid commit slot");
  if (inst->tma.mem_commit_is_store && !inst->page_fault_store) {
    StqEntry e = back->lsu->get_stq_entry(inst->stq_idx, inst->stq_flag);
    const bool store_suppressed = static_cast<bool>(e.suppress_write) &&
                                  e.rob_idx == inst->rob_idx &&
                                  e.rob_flag == inst->rob_flag;
    if (!store_suppressed && e.paddr_valid && e.data_valid) {
      mem_subsystem.on_commit_store(e.paddr, e.data, e.func3);
    }
  }
}

void SimContext::run_commit_inst(InstEntry *inst_entry, int commit_slot) {
  Assert(cpu != nullptr && "SimContext::run_commit_inst: cpu is null");
  Assert(inst_entry != nullptr &&
         "SimContext::run_commit_inst: inst_entry is null");
  Assert(inst_entry->valid &&
         "SimContext::run_commit_inst: inst_entry is not valid");
  cpu->commit_sync(&inst_entry->uop, commit_slot);
}

void SimContext::run_interrupt_hook() {
  Assert(cpu != nullptr && "SimContext::run_interrupt_hook: cpu is null");
  const uint8_t tp = static_cast<uint8_t>(cpu->back.csr->d.privilege);
  const xlen_t c = tp == RISCV_MODE_M ? cpu->back.csr->d.CSR_RegFile[csr_mcause]
                                        : cpu->back.csr->d.CSR_RegFile[csr_scause];
  tbhook::on_interrupt(c, tp, cpu->back.csr->d.CSR_RegFile[csr_mip]);
}

// reset
void SimCpu::init() {
  // 1. the top-level context
  ctx.cpu = this;

  // 2. the module objects (and their submodules)
  back.init();

  // 3. the wires between the modules
  mem_subsystem.csr_interrupt_inject = &back.csr_interrupt_inject_io;
  mem_subsystem.peripheral_req = &back.out.peripheral_req;
  mem_subsystem.peripheral_resp = &d.peripheral_resp;   // the MMIO response enters the back end through a register
  mem_subsystem.set_ptw_coherent_source(back.lsu.get());

  front.in.csr_status = &back.csr_status;
  front.ctx = &ctx;

  back.set_lsu_ptw_walk_port(mem_subsystem.dtlb_walk_port);

  mem_subsystem.lsu2dcache = &back.out.lsu2dcache;
  mem_subsystem.dcache2lsu = &back.in.dcache2lsu;

  mem_subsystem.icache_req = &icache_req;
  mem_subsystem.dcache_axi_r = &axi.interconnect.read_ports[axi_interconnect::MASTER_DCACHE_R];
  mem_subsystem.dcache_axi_w = &axi.interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W];
  mem_subsystem.extra_axi_r = &axi.interconnect.read_ports[axi_interconnect::MASTER_EXTRA_R];
  mem_subsystem.extra_axi_w = &axi.interconnect.write_ports[axi_interconnect::MASTER_EXTRA_W];
  mem_subsystem.icache_resp = &icache_resp;

  front.icache_ptw_walk_port = mem_subsystem.itlb_walk_port;
  front.icache_mem_req_port = &icache_req;
  front.icache_mem_resp_port = &icache_resp_q;   // the I-cache reads its memory responses through the registers
  // the I-cache's read port of the shared interconnect (the D-cache, PTW and peripherals use the same interconnect)
  front.icache_mem_read_port =
      &axi.interconnect.read_ports[axi_interconnect::MASTER_ICACHE];
  front.icache_mem_read_q = &icache_axi_q;

  // 4. reset of every module; the memory subsystem first (PTW / D-cache / write buffer state)
  mem_subsystem.init();
  axi.init();
  // front.init() steps the BPU once and touches the I-cache's AXI port, so the AXI side is reset first
  icache_port_regs_seq();
  front.init();
  q_ = {};
  d = {};
}

void SimCpu::sync_mmio_devices_from_backing() {
  axi.sync_uart_from_backing();
  mem_subsystem.sync_mmio_devices_from_backing();
}

// force the front-end PC (switch from fast mode)
void SimCpu::restore_pc(pc_t pc) {
  // each top restarts at pc (the front end refetches, the back end shows a redirect); a held fetch group is dropped
  front.restart_at(pc);
  q_.front2back_hold_valid = false;
  d.front2back_hold_valid = false;
  back.restart_at(pc);
}

// The clock edge of the I-cache port registers (at the start of the cycle: nothing between the end of the previous
// cycle and here changes the port outputs or icache_resp).
void SimCpu::icache_port_regs_seq() {
  icache_axi_q = axi.interconnect.read_ports[axi_interconnect::MASTER_ICACHE];
  icache_resp_q = icache_resp;
}

// The back end's inputs that come from SimCpu's registers: everything cleared, then the held fetch group (the back end
// stalled last cycle) and the held peripheral response.
void SimCpu::back_in_from_regs() {
  back.clear_comb_inputs();
  const auto &q = q_;
  if (q.front2back_hold_valid) static_cast<FrontPreIO &>(back.in) = q.front2back_hold;
  back.in.peripheral_resp = q.peripheral_resp;
}

// AXI: the master-side request fields are driven anew every cycle (resp.ready stays: it is read before the masters
// drive it again).
void SimCpu::axi_comb_begin() { axi.comb_begin(); }
// AXI phase 1: slave outputs -> interconnect outputs (req.ready / accepted / resp on the master ports).
void SimCpu::axi_comb_outputs() {
  mem_subsystem.llc_comb_outputs();
  axi.comb_outputs(mem_subsystem.llc_lookup_in());
}
// AXI phase 2: master requests -> interconnect -> slave inputs.
void SimCpu::axi_comb_inputs() { axi.comb_inputs(); }

void SimCpu::cycle() {
  // clock edge at the start of the cycle: the I-cache port registers, the cycle counter
  icache_port_regs_seq();
  ctx.perf.cycle++;

  // comb: the tops' exported phases (include/SimCpu.h), in the order the exports give
  // BEGIN GENERATED by scripts/comb/gen_top_order.py from the EXPORT declarations: do not edit
  back_in_from_regs();
  back.comb_csr_status();
  axi_comb_begin();
  front.comb(&back.csr_status, back.in);
  axi_comb_outputs();
  mem_subsystem.comb_outputs();
  back.comb();
  comb_front2back_hold();
  front.comb_from_back(back);
  mem_subsystem.comb_inputs();
  axi_comb_inputs();
  // END GENERATED

  front.seq_core();
  front.seq_back2front();
  regs_seq();

  back.seq();
  mem_subsystem.seq();
  mem_subsystem.llc_seq(axi.interconnect.get_llc_table_out());
  axi.seq();
  ctx.perf.perf_maybe_capture_simtime_snapshot();

  if (ctx.exit_reason != ExitReason::NONE) {
    printf("Simulation Exited with Reason: %d\n", (int)ctx.exit_reason);
    return;
  }
}

// The fetch-group hold register: when the back end stalls (and does not redirect), the group it did not take
// (the fired slots invalid) is back.in of the next cycle.
void SimCpu::comb_front2back_hold() {
  d.front2back_hold_valid = false;
  if (back.out.mispred || back.out.flush || !back.out.stall) {
    return;
  }

  d.front2back_hold_valid = true;
  d.front2back_hold = static_cast<const FrontPreIO &>(back.in);
  for (int i = 0; i < BACKEND_FETCH_WIDTH; ++i) {
    if (back.out.fire[i]) {
      d.front2back_hold.valid[i] = false;
    }
  }
}

void SimCpu::regs_seq() { q_ = d; }

