#include "trace_hook.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <queue>
#include <vector>

// Parent-side compatibility for axi-interconnect-kit revisions that do not
// expose comb_inputs_internal_tick(). The adapter below snapshots the explicit
// CPU-boundary ready/hold registers while extra AXI ticks advance only the
// internal AXI/LLC/DDR state.
#define private public
#include "AXI_Interconnect.h"
#undef private

#include "BackTop.h"
#include "Csr.h"
#include "PhysMemory.h"
#include "RealLsu.h"
#include "SimCpu.h"
#include "config.h"
#include "RefCore.h"
#include "front_IO.h"
#include "front_module.h"
#include "BPU/BPU.h"
#include "util.h"

namespace {
void sync_uart_registers_from_backing(mmio::UART16550_Device &uart) {
  // The AXI UART model is TX-only: register 0 is a consumed THR and must stay
  // empty after restore. Restore the remaining software-visible registers
  // without replaying a transmit write.
  for (uint32_t reg_off = 1; reg_off < 8; ++reg_off) {
    const uint32_t word_addr = UART_ADDR_BASE + (reg_off & ~0x3u);
    const uint32_t shift = (reg_off & 0x3u) * 8u;
    const uint8_t value =
        static_cast<uint8_t>((pmem_read(word_addr) >> shift) & 0xffu);
    uart.write(UART_ADDR_BASE + reg_off, &value, 1u, 1u);
  }
}

template <typename InterconnectT>
void clear_axi_master_inputs(InterconnectT &interconnect) {
  for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; i++) {
    auto &port = interconnect.read_ports[i];
    port.req.valid = false;
    port.req.addr = 0;
    port.req.total_size = 0;
    port.req.id = 0;
    port.req.bypass = false;
    // resp.ready is not cleared: each master drives it, the interconnect samples it at the clock edge.
  }
  for (int i = 0; i < axi_interconnect::NUM_WRITE_MASTERS; i++) {
    auto &port = interconnect.write_ports[i];
    port.req.valid = false;
    port.req.addr = 0;
    port.req.wdata.clear();
    port.req.wstrb = 0;
    port.req.total_size = 0;
    port.req.id = 0;
    port.req.bypass = false;
  }
}

void connect_axi_slave_outputs(sim_ddr::SimDDR_IO_t &master_io,
                               const sim_ddr::SimDDR_IO_t &slave_io) {
  master_io.ar.arready = slave_io.ar.arready;
  master_io.r.rvalid = slave_io.r.rvalid;
  master_io.r.rid = slave_io.r.rid;
  master_io.r.rdata = slave_io.r.rdata;
  master_io.r.rresp = slave_io.r.rresp;
  master_io.r.rlast = slave_io.r.rlast;
  master_io.aw.awready = slave_io.aw.awready;
  master_io.w.wready = slave_io.w.wready;
  master_io.b.bvalid = slave_io.b.bvalid;
  master_io.b.bid = slave_io.b.bid;
  master_io.b.bresp = slave_io.b.bresp;
}

void connect_axi_master_outputs(const sim_ddr::SimDDR_IO_t &master_io,
                                sim_ddr::SimDDR_IO_t &slave_io) {
  slave_io.ar.arvalid = master_io.ar.arvalid;
  slave_io.ar.arid = master_io.ar.arid;
  slave_io.ar.araddr = master_io.ar.araddr;
  slave_io.ar.arlen = master_io.ar.arlen;
  slave_io.ar.arsize = master_io.ar.arsize;
  slave_io.ar.arburst = master_io.ar.arburst;
  slave_io.r.rready = master_io.r.rready;

  slave_io.aw.awvalid = master_io.aw.awvalid;
  slave_io.aw.awid = master_io.aw.awid;
  slave_io.aw.awaddr = master_io.aw.awaddr;
  slave_io.aw.awlen = master_io.aw.awlen;
  slave_io.aw.awsize = master_io.aw.awsize;
  slave_io.aw.awburst = master_io.aw.awburst;
  slave_io.w.wvalid = master_io.w.wvalid;
  slave_io.w.wdata = master_io.w.wdata;
  slave_io.w.wstrb = master_io.w.wstrb;
  slave_io.w.wlast = master_io.w.wlast;
  slave_io.b.bready = master_io.b.bready;
}

axi_interconnect::AXI_LLCConfig make_default_llc_config() {
  axi_interconnect::AXI_LLCConfig llc_cfg;
  llc_cfg.enable = (CONFIG_AXI_LLC_ENABLE != 0);
  llc_cfg.size_bytes = CONFIG_AXI_LLC_SIZE_BYTES;
  llc_cfg.ways = CONFIG_AXI_LLC_WAYS;
  llc_cfg.mshr_num = CONFIG_AXI_LLC_MSHR_NUM;
  llc_cfg.lookup_latency = CONFIG_AXI_LLC_LOOKUP_LATENCY;
  return llc_cfg;
}

void axi_subsystem_comb_outputs(SimCpu &cpu) {
  cpu.mem_subsystem.llc_comb_outputs();
  cpu.axi_interconnect.set_llc_lookup_in(cpu.mem_subsystem.llc_lookup_in());
  cpu.axi_ddr.comb_outputs();
  cpu.axi_mmio.comb_outputs();
  connect_axi_slave_outputs(cpu.axi_interconnect.axi_ddr_io, cpu.axi_ddr.io);
  connect_axi_slave_outputs(cpu.axi_interconnect.axi_mmio_io, cpu.axi_mmio.io);
  cpu.axi_interconnect.comb_outputs();
}

void axi_subsystem_comb_inputs(SimCpu &cpu) {
  cpu.axi_interconnect.comb_inputs();
  connect_axi_master_outputs(cpu.axi_interconnect.axi_ddr_io, cpu.axi_ddr.io);
  connect_axi_master_outputs(cpu.axi_interconnect.axi_mmio_io, cpu.axi_mmio.io);
  cpu.axi_ddr.comb_inputs();
  cpu.axi_mmio.comb_inputs();
}

void axi_subsystem_seq(SimCpu &cpu) {
  cpu.mem_subsystem.llc_seq(cpu.axi_interconnect.get_llc_table_out());
  cpu.axi_interconnect.seq();
  cpu.axi_ddr.seq();
  cpu.axi_mmio.seq();
}

} // namespace

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
  const auto llc_cfg = make_default_llc_config();

  // 1. the top-level context
  ctx.cpu = this;

  // 2. the module objects (and their submodules)
  back.init();

  // 3. the wires between the modules
  mem_subsystem.csr_interrupt_inject = &back.csr_interrupt_inject_io;
  mem_subsystem.peripheral_req = &back.out.peripheral_req;
  mem_subsystem.peripheral_resp = &peripheral_resp_hold_d;   // the MMIO response enters the back end through a register
  mem_subsystem.set_ptw_coherent_source(back.lsu.get());

  front.in.csr_status = &back.csr_status;
  front.ctx = &ctx;

  back.set_lsu_ptw_walk_port(mem_subsystem.dtlb_walk_port);

  mem_subsystem.lsu2dcache = &back.out.lsu2dcache;
  mem_subsystem.dcache2lsu = &back.in.dcache2lsu;

  mem_subsystem.icache_req = &icache_req;
  mem_subsystem.dcache_axi_r = &axi_interconnect.read_ports[axi_interconnect::MASTER_DCACHE_R];
  mem_subsystem.dcache_axi_w = &axi_interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W];
  mem_subsystem.extra_axi_r = &axi_interconnect.read_ports[axi_interconnect::MASTER_EXTRA_R];
  mem_subsystem.extra_axi_w = &axi_interconnect.write_ports[axi_interconnect::MASTER_EXTRA_W];
  mem_subsystem.icache_resp = &icache_resp;

  front.icache_ptw_walk_port = mem_subsystem.itlb_walk_port;
  front.icache_mem_req_port = &icache_req;
  front.icache_mem_resp_port = &icache_resp_q;   // the I-cache reads its memory responses through the registers
  // Keep a single active SoC memory topology. DCache/PTW/peripheral already use
  // the top-level interconnect; frontend icache should use the same shared path
  // whenever the real BPU frontend is enabled. Oracle mode does not step the
  // icache model, so it simply leaves the port disconnected instead of falling
  // back to MemSubsystem's legacy private AXI runtime.
  front.icache_mem_read_port =
      &axi_interconnect.read_ports[axi_interconnect::MASTER_ICACHE];
  front.icache_mem_read_q = &icache_axi_q;

  // 4. reset of every module; the memory subsystem first (PTW / D-cache / write buffer state)
  mem_subsystem.init();
  axi_interconnect.set_llc_config(llc_cfg);
  axi_interconnect.init();
  axi_ddr.init();
  axi_mmio.init();
  axi_mmio.add_device(UART_ADDR_BASE, UART_MMIO_SIZE, &axi_uart);
  // In shared-LLC mode, front.init()/front.step_bpu() directly touches the
  // top-level icache AXI port, so the shared interconnect/MMIO/DDRx must be
  // reset before frontend init. Keeping the order uniform is harmless in the
  // LLC-off path.
  icache_port_regs_seq();
  front.init();
  oracle_pending_valid = false;
  oracle_pending_out = {};
  front2back_hold_q = {};
  front2back_hold_d = {};
  front2back_hold_q_live = front2back_hold_d_live = false;
  peripheral_resp_hold_q = {};
  peripheral_resp_hold_d = {};
}

void SimCpu::sync_mmio_devices_from_backing() {
  sync_uart_registers_from_backing(axi_uart);
  mem_subsystem.sync_mmio_devices_from_backing();
}

// force the front-end PC (switch from fast mode)
void SimCpu::restore_pc(pc_t pc) {
  front.in.ctrl.reset = false;
  front.in.ctrl.fifo_read_enable = false;
  // an explicit refetch for the front end and the back end's redirect outputs: the fetch restarts at pc even if
  // the caller set no flush / mispred
  front.in.redirect.valid = true;
  front.in.redirect.address = pc;
  front.force_back2front_refetch(pc);
  front2back_hold_q = {};
  front2back_hold_d = {};
  front2back_hold_q_live = front2back_hold_d_live = false;
  back.out.flush = true;
  back.out.mispred = true;
  back.out.redirect_pc = pc;

  // refresh the CSR status (satp, privilege) so the MMU mode is right
  back.comb_csr_status();
}

// The clock edge of the I-cache port registers (at the start of the cycle: nothing between the end of the previous
// cycle and here changes the port outputs or icache_resp).
void SimCpu::icache_port_regs_seq() {
  icache_axi_q = axi_interconnect.read_ports[axi_interconnect::MASTER_ICACHE];
  icache_resp_q = icache_resp;
}

// The back end's inputs that come from SimCpu's registers: everything cleared, then the held fetch group (the back end
// stalled last cycle) and the held peripheral response.
void SimCpu::back_in_from_regs() {
  back.clear_comb_inputs();
  if (front2back_hold_q_live) static_cast<FrontPreIO &>(back.in) = front2back_hold_q;
  back.in.peripheral_resp = peripheral_resp_hold_q;
}

// AXI: the master-side request fields are driven anew every cycle (resp.ready stays: it is read before the masters
// drive it again).
void SimCpu::axi_comb_begin() { clear_axi_master_inputs(axi_interconnect); }
// AXI phase 1: slave outputs -> interconnect outputs (req.ready / accepted / resp on the master ports).
void SimCpu::axi_comb_outputs() { axi_subsystem_comb_outputs(*this); }
// AXI phase 2: master requests -> interconnect -> slave inputs.
void SimCpu::axi_comb_inputs() { axi_subsystem_comb_inputs(*this); }

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
  front2back_hold_seq();
  peripheral_resp_hold_seq();

  back.seq();
  mem_subsystem.seq();
  axi_subsystem_seq(*this);
  ctx.perf.perf_maybe_capture_simtime_snapshot();

  if (ctx.exit_reason != ExitReason::NONE) {
    printf("Simulation Exited with Reason: %d\n", (int)ctx.exit_reason);
    return;
  }

  if (back.out.mispred || back.out.flush) {
    back.number_PC = back.out.redirect_pc;
  }
}

// The hold register: d = front2back_hold_d (written only when the back end stalls, otherwise left all-zero), q =
// front2back_hold_q. The live flags let the cycle skip copying zeros; seq clears d after the copy.
void SimCpu::comb_front2back_hold() {
  if (back.out.mispred || back.out.flush || !back.out.stall) {
    return;
  }

  front2back_hold_d_live = true;
  front2back_hold_d = static_cast<const FrontPreIO &>(back.in);
  for (int i = 0; i < BACKEND_FETCH_WIDTH; ++i) {
    if (back.out.fire[i]) {
      front2back_hold_d.valid[i] = false;
    }
  }
}

void SimCpu::front2back_hold_seq() {
  if (front2back_hold_q_live || front2back_hold_d_live) front2back_hold_q = front2back_hold_d;
  front2back_hold_q_live = front2back_hold_d_live;
  if (front2back_hold_d_live) {
    front2back_hold_d = {};
    front2back_hold_d_live = false;
  }
}

void SimCpu::peripheral_resp_hold_seq() {
  peripheral_resp_hold_q = peripheral_resp_hold_d;
}


