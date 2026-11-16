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
    // resp.ready is consumed during axi_interconnect.comb_outputs() before the
    // top-level bridges re-drive master inputs later in the cycle. Clearing it
    // here would make a held LLC response observe permanent backpressure.
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

// 复位逻辑
void SimCpu::init() {
  const auto llc_cfg = make_default_llc_config();

  // 第一阶段：绑定顶层上下文
  ctx.cpu = this;

  // 第二阶段：构建模块对象（生成内部子模块实例）
  back.init();

  // 第三阶段：集中完成跨模块连线
  mem_subsystem.csr_interrupt_inject = &back.csr_interrupt_inject_io;
  mem_subsystem.peripheral_req = &back.out.peripheral_req;
  mem_subsystem.peripheral_resp = &back.in.peripheral_resp;
  mem_subsystem.set_ptw_coherent_source(back.lsu);

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

  // 第四阶段：统一执行各模块复位逻辑
  // 先初始化内存子系统，确保 PTW / DCache / WB 等内部状态已经完成复位。
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
  front2back_hold = {};
  front2back_hold_1 = {};
  front2back_hold_live = front2back_hold_1_live = false;
  peripheral_resp_hold = {};
  peripheral_resp_hold_1 = {};
}

void SimCpu::sync_mmio_devices_from_backing() {
  sync_uart_registers_from_backing(axi_uart);
  mem_subsystem.sync_mmio_devices_from_backing();
}

// 强制重置前端 PC (用于 FAST 模式切换)
void SimCpu::restore_pc(pc_t pc) {
  front.in.reset = false;
  front.in.FIFO_read_enable = false;
  // 显式给前端一个重取指请求，并同步后端重定向输出，
  // 保证即使调用方未额外设置 flush/mispred，也会从目标 PC 重新开始。
  front.in.refetch = true;
  front.in.refetch_address = pc;
  front.force_back2front_refetch(pc);
  front2back_hold = {};
  front2back_hold_1 = {};
  front2back_hold_live = front2back_hold_1_live = false;
  back.out.flush = true;
  back.out.mispred = true;
  back.out.redirect_pc = pc;

  // 刷新 CSR 状态输出 (SATP, Privilege) 以确保 MMU 模式正确
  back.comb_csr_status();
}

// The clock edge of the I-cache port registers (at the start of the cycle: nothing between the end of the previous
// cycle and here changes the port outputs or icache_resp).
void SimCpu::icache_port_regs_seq() {
  icache_axi_q = axi_interconnect.read_ports[axi_interconnect::MASTER_ICACHE];
  icache_resp_q = icache_resp;
}

void SimCpu::cycle() {
  icache_port_regs_seq();
  ctx.perf.cycle++;
  // BackTop inputs are combinational wires. Re-drive the only intentionally
  // retained payload from an explicit top-level register before producers run.
  back.clear_comb_inputs();
  if (front2back_hold_live) static_cast<FrontPreIO &>(back.in) = front2back_hold;
  back.in.peripheral_resp = peripheral_resp_hold;
  // 统一在此处刷新 CSR 状态，供本拍 front/back 组合逻辑共同使用。
  back.comb_csr_status();

  clear_axi_master_inputs(axi_interconnect);

  front.comb(&back.csr_status, back.in);   // the front end, and the fetch group for the back end

  // AXI phase 1: slave outputs -> interconnect outputs (req.ready / accepted / resp on the master ports, which the
  // MSHR, the write buffer and the MMIO path read directly in the memory subsystem's comb).
  axi_subsystem_comb_outputs(*this);

  mem_subsystem.comb_outputs();

  back.comb();

  mem_subsystem.comb_inputs();
  peripheral_resp_hold_comb();

  // AXI phase-2: master requests -> interconnect -> slave inputs.
  axi_subsystem_comb_inputs(*this);

  front.comb_from_back(back);              // the front end's next-cycle inputs from the back end's commit / redirect
  front2back_hold_comb();

  front.seq_write_bpu();
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

// The hold register: d = front2back_hold_1 (written only when the back end stalls, otherwise left all-zero), q =
// front2back_hold. The live flags let the cycle skip copying zeros; seq clears d after the copy.
void SimCpu::front2back_hold_comb() {
  if (back.out.mispred || back.out.flush || !back.out.stall) {
    return;
  }

  front2back_hold_1_live = true;
  front2back_hold_1 = static_cast<const FrontPreIO &>(back.in);
  for (int i = 0; i < BACKEND_FETCH_WIDTH; ++i) {
    if (back.out.fire[i]) {
      front2back_hold_1.valid[i] = false;
    }
  }
}

void SimCpu::front2back_hold_seq() {
  if (front2back_hold_live || front2back_hold_1_live) front2back_hold = front2back_hold_1;
  front2back_hold_live = front2back_hold_1_live;
  if (front2back_hold_1_live) {
    front2back_hold_1 = {};
    front2back_hold_1_live = false;
  }
}

void SimCpu::peripheral_resp_hold_comb() {
  peripheral_resp_hold_1 = back.in.peripheral_resp;
}

void SimCpu::peripheral_resp_hold_seq() {
  peripheral_resp_hold = peripheral_resp_hold_1;
}


