// SimCpu: the tops (front end, back end, memory subsystem, AXI) wired together, and one cycle of the whole core.
#include "SimCpu.h"
#include "front_core.h"   // the front end's modules (SimCpu::cycle() calls their phases)
#include "Csr.h"
#include "RealLsu.h"
#include "config.h"
#include "trace_hook.h"
#include "util.h"
#include <cstdint>

// An interrupt taken at commit is reported to the trace bridge (Ren calls it through SimContext while it commits).
void SimContext::run_interrupt_hook() {
  Assert(cpu != nullptr && "SimContext::run_interrupt_hook: cpu is null");
  const Csr::TrapOut t = cpu->csr.trap_out();   // the trap of this cycle
  tbhook::on_interrupt(t.cause, t.privilege, cpu->csr.q().CSR_RegFile[csr_mip]);
}

// reset
namespace { const Csr *g_csr = nullptr; }
const Csr &csr_state() { return *g_csr; }

SimCpu::SimCpu() : back(&this->ctx), mem_subsystem(&this->ctx) { g_csr = &csr; }

void SimCpu::install_arch_state(const CPU_state &state, uint8_t privilege) {
  csr.q_mut().privilege = privilege;
  for (int i = 0; i < CSR_NUM; i++) csr.q_mut().CSR_RegFile[i] = state.csr[i];
  csr.refresh();
  back.install_arch_state(state);
}

void SimCpu::init() {
  // 1. the top-level context
  ctx.cpu = this;

  // 2. the module objects (and their submodules)
  back.init();

  // 3. the wires between the modules
  csr.in.csr_interrupt_inject_io = &csr_interrupt_inject_io;
  csr.in.rob2csr = &back.rob->out.rob2csr;
  csr.in.rob_bcast = &back.rob->out.rob_bcast;
  csr.init();
  mem_subsystem.csr_interrupt_inject = &csr_interrupt_inject_io;
  mem_subsystem.peripheral_req = &back.out.peripheral_req;
  mem_subsystem.commit_store = &back.out.commit_store;
  mem_subsystem.peripheral_resp = &back.lsu->mmio_resp_d;   // the MMIO response enters the back end through a register
  mem_subsystem.set_ptw_coherent_source(back.lsu.get());

  front.ctx = &ctx;


  mem_subsystem.lsu2dcache = &back.out.lsu2dcache;
  mem_subsystem.dcache2lsu = &back.in.dcache2lsu;

  mem_subsystem.icache_req = &icache_req;
  mem_subsystem.dcache_axi_r = &axi.interconnect.read_ports[axi_interconnect::MASTER_DCACHE_R];
  mem_subsystem.dcache_axi_w = &axi.interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W];
  mem_subsystem.extra_axi_r = &axi.interconnect.read_ports[axi_interconnect::MASTER_EXTRA_R];
  mem_subsystem.extra_axi_w = &axi.interconnect.write_ports[axi_interconnect::MASTER_EXTRA_W];
  mem_subsystem.icache_resp = &icache_resp;

  // the TLBs' page-table walk ports (the memory subsystem's walker)
  back.lsu->in.dtlb_walk = &mem_subsystem.ptw_block.dtlb_q;
  back.lsu->out.dtlb_walk = &mem_subsystem.dtlb_walk_req;
  back.lsu->dtlb.walk_resp = back.lsu->in.dtlb_walk;
  back.lsu->dtlb.walk_req = back.lsu->out.dtlb_walk;
  front.core->icache.in.itlb_walk = &mem_subsystem.ptw_block.itlb_q;
  front.core->icache.out.itlb_walk = &mem_subsystem.itlb_walk_req;
  // the I-cache's read port of the shared interconnect and its D-cache probe (it reads the responses through registers)
  front.core->icache.out.axi = &axi.interconnect.read_ports[axi_interconnect::MASTER_ICACHE];
  front.core->icache.in.axi_q = &icache_axi_q;
  front.core->icache.out.probe_req = &icache_req;
  front.core->icache.in.probe_resp = &icache_resp_q;
  // the masters' ports, as the AXI subsystem sees them
  axi.out.read_ports = axi.interconnect.read_ports;
  axi.out.write_ports = axi.interconnect.write_ports;

  // 4. reset of every module; the memory subsystem first (PTW / D-cache / write buffer state)
  mem_subsystem.init();
  axi.llc = &mem_subsystem;
  axi.init();
  // front.init() steps the BPU once and touches the I-cache's AXI port, so the AXI side is reset first
  icache_port_regs_seq();
  front.init();
  f2b.in.front_out = &front.out;
  f2b.in.can_read = &front.can_read;
  f2b.in.back_out = &back.out;
  f2b.out.to_back = &back.in;
  f2b.init();
  b2f.in.back_out = &back.out;
  b2f.out.front_in = &front.in;
  b2f.out.can_read = &front.can_read;
  b2f.init();
}

void SimCpu::sync_mmio_devices_from_backing() {
  axi.sync_uart_from_backing();
  mem_subsystem.sync_mmio_devices_from_backing();
}

// force the front-end PC (switch from fast mode)
void SimCpu::restore_pc(pc_t pc) {
  // each top restarts at pc (the front end refetches, the back end shows a redirect); a held fetch group is dropped
  front.restart_at(pc);
  b2f.restart_at(pc);
  f2b.restart();
  back.restart_at(pc);
  csr.refresh();
}

// The clock edge of the I-cache port registers (at the start of the cycle: nothing between the end of the previous
// cycle and here changes the port outputs or icache_resp).
void SimCpu::icache_port_regs_seq() {
  icache_axi_q = axi.interconnect.read_ports[axi_interconnect::MASTER_ICACHE];
  icache_resp_q = icache_resp;
}

void SimCpu::cycle() {
  // clock edge at the start of the cycle: the I-cache port registers, the cycle counter
  icache_port_regs_seq();
  ctx.perf.cycle++;

  // comb: the tops' exported phases (include/SimCpu.h), in the order the exports give
  // BEGIN GENERATED by scripts/comb/gen_top_order.py from the module contracts: do not edit
  back.rename->comb_rename();
  axi.comb_begin();
  b2f.comb_front();
  front.core->ctrl.control();
  front.core->bpu_pipe.comb_front();
  front.core->ctrl.enables();
  front.core->fa_port.comb_read();
  front.core->ptab_port.comb_read();
  front.core->inst_port.comb_read();
  front.core->f2b_port.comb_read();
  front.core->fa_port.comb_write();
  front.core->icache.comb_fetch();
  front.core->inst_port.comb_write();
  front.core->checker.comb();
  front.core->f2b_port.comb_write();
  front.core->icache.comb_invalidate();
  front.core->ctrl.finish();
  axi.comb_outputs();
  f2b.comb();
  back.pre->comb_accept_front();
  back.idu->comb_decode();
  mem_subsystem.wb_.comb_axi_to_dcache();
  mem_subsystem.mshr_.comb_axi_to_dcache();
  mem_subsystem.dcache_.comb_stage2();
  mem_subsystem.mem_route_block.comb_response();
  back.lsu->comb_outputs();
  back.rob->comb_commit();
  back.pre->comb_commit_info();
  back.lsu->comb_writeback();
  back.dis->comb_alloc();
  back.exu->comb_exec();
  back.lsu->comb_translate();
  back.rob->comb_redirect();
  back.isu->comb_issue();
  back.dis->comb_dispatch();
  back.rename->comb_fire();
  back.idu->comb_fire();
  back.lsu->comb_requests();
  back.lsu->comb_commit_store();
  mem_subsystem.ptw_block.comb_request();
  mem_subsystem.mem_route_block.comb_request();
  mem_subsystem.dcache_.comb_stage1();
  mem_subsystem.wb_.comb_dcache_to_axi();
  mem_subsystem.mshr_.comb_dcache_to_axi();
  mem_subsystem.ptw_block.comb_feedback();
  mem_subsystem.peripheral_axi_.comb_commit_store();
  mem_subsystem.peripheral_axi_.comb();
  axi.comb_inputs();
  // END GENERATED
  back.pmu_sample();   // the cycle's PMU sample (after all comb phases)

  front.seq_core();
  b2f.seq();
  f2b.seq();

  back.seq();
  csr.seq();   // the commit's trap / xRET, CSR write and FS state, the interrupt lines (before the devices' tick)
  mem_subsystem.seq();
  axi.seq();
  ctx.perf.perf_maybe_capture_simtime_snapshot();

  if (ctx.exit_reason != ExitReason::NONE) {
    printf("Simulation Exited with Reason: %d\n", (int)ctx.exit_reason);
    return;
  }
}

