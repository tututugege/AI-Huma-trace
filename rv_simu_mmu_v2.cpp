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
#include "diff.h"
#include "front_IO.h"
#include "front_module.h"
#include "BPU/BPU.h"
#include "util.h"

#ifndef CONFIG_DCACHE_FAST_DOMAIN
#define CONFIG_DCACHE_FAST_DOMAIN 16
#endif

namespace {
constexpr bool kDcacheFastDomainEnabled = CONFIG_DCACHE_FAST_DOMAIN != 0;

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

bool is_dcache_read_master(int master) {
  return kDcacheFastDomainEnabled &&
         master == axi_interconnect::MASTER_DCACHE_R;
}

bool is_dcache_write_master(int master) {
  return kDcacheFastDomainEnabled &&
         master == axi_interconnect::MASTER_DCACHE_W;
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

struct AxiInternalTickBoundaryState {
  bool read_req_ready_r[axi_interconnect::NUM_READ_MASTERS] = {};
  axi_interconnect::ReadReqHoldLatch
      read_req_hold[axi_interconnect::NUM_READ_MASTERS] = {};
  bool write_req_ready_r[axi_interconnect::NUM_WRITE_MASTERS] = {};
};

template <typename InterconnectT>
void capture_axi_internal_tick_boundary_state(
    InterconnectT &interconnect, AxiInternalTickBoundaryState &state) {
  for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; ++i) {
    state.read_req_ready_r[i] = interconnect.req_ready_r[i];
    state.read_req_hold[i] = interconnect.read_req_hold[i];
  }
  for (int i = 0; i < axi_interconnect::NUM_WRITE_MASTERS; ++i) {
    state.write_req_ready_r[i] = interconnect.w_req_ready_r[i];
  }
}

template <typename InterconnectT>
void restore_axi_internal_tick_boundary_state_except_dcache(
    InterconnectT &interconnect, const AxiInternalTickBoundaryState &state) {
  for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; ++i) {
    if (is_dcache_read_master(i)) {
      continue;
    }
    interconnect.req_ready_r[i] = state.read_req_ready_r[i];
    interconnect.read_req_hold[i] = state.read_req_hold[i];
  }
  for (int i = 0; i < axi_interconnect::NUM_WRITE_MASTERS; ++i) {
    if (is_dcache_write_master(i)) {
      continue;
    }
    interconnect.w_req_ready_r[i] = state.write_req_ready_r[i];
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

// L1D (MSHR refill / write-back) side of the AXI <-> MemSubsystem handshake.
void bridge_dcache_axi_to_mem_subsystem(SimCpu &cpu) {
  const auto &rport = cpu.axi_interconnect.read_ports[axi_interconnect::MASTER_DCACHE_R];
  cpu.mem_subsystem.mshr_axi_in.req_ready = rport.req.ready;
  cpu.mem_subsystem.mshr_axi_in.req_accepted = rport.req.accepted;
  cpu.mem_subsystem.mshr_axi_in.req_accepted_id = rport.req.accepted_id;
  cpu.mem_subsystem.mshr_axi_in.resp_valid = rport.resp.valid;
  cpu.mem_subsystem.mshr_axi_in.resp_id = rport.resp.id;
  const auto &wport = cpu.axi_interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W];
  cpu.mem_subsystem.wb_axi_in.req_ready = wport.req.ready;
  cpu.mem_subsystem.wb_axi_in.req_accepted = wport.req.accepted;
  cpu.mem_subsystem.wb_axi_in.resp_valid = wport.resp.valid;
}

void bridge_axi_to_mem_subsystem(SimCpu &cpu) {
  bridge_dcache_axi_to_mem_subsystem(cpu);

  const auto &peri_rport = cpu.axi_interconnect.read_ports[axi_interconnect::MASTER_EXTRA_R];
  cpu.mem_subsystem.peripheral_axi_read_in.req_ready = peri_rport.req.ready;
  cpu.mem_subsystem.peripheral_axi_read_in.req_accepted = peri_rport.req.accepted;
  cpu.mem_subsystem.peripheral_axi_read_in.resp_valid = peri_rport.resp.valid;
  cpu.mem_subsystem.peripheral_axi_read_in.resp_id = peri_rport.resp.id;
  for (int i = 0; i < DCACHE_WORD_NUM && i < axi_interconnect::MAX_READ_TRANSACTION_WORDS; i++) {
    cpu.mem_subsystem.peripheral_axi_read_in.resp_data[i] = peri_rport.resp.data[i];
  }

  const auto &peri_wport = cpu.axi_interconnect.write_ports[axi_interconnect::MASTER_EXTRA_W];
  cpu.mem_subsystem.peripheral_axi_write_in.req_ready = peri_wport.req.ready;
  cpu.mem_subsystem.peripheral_axi_write_in.req_accepted = peri_wport.req.accepted;
  cpu.mem_subsystem.peripheral_axi_write_in.resp_valid = peri_wport.resp.valid;
  cpu.mem_subsystem.peripheral_axi_write_in.resp_id = peri_wport.resp.id;
}

void bridge_dcache_mem_subsystem_to_axi(SimCpu &cpu) {
  auto &rport = cpu.axi_interconnect.read_ports[axi_interconnect::MASTER_DCACHE_R];
  rport.req.valid = cpu.mem_subsystem.mshr_axi_out.req_valid;
  rport.req.addr = cpu.mem_subsystem.mshr_axi_out.req_addr;
  rport.req.total_size = cpu.mem_subsystem.mshr_axi_out.req_total_size;
  rport.req.id = cpu.mem_subsystem.mshr_axi_out.req_id;
  rport.req.bypass = false;
  rport.resp.ready = cpu.mem_subsystem.mshr_axi_out.resp_ready;

  auto &wport = cpu.axi_interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W];
  wport.req.valid = cpu.mem_subsystem.wb_axi_out.req_valid;
  wport.req.addr = cpu.mem_subsystem.wb_axi_out.req_addr;
  wport.req.total_size = cpu.mem_subsystem.wb_axi_out.req_total_size;
  wport.req.id = cpu.mem_subsystem.wb_axi_out.req_id;
  wport.req.wstrb = cpu.mem_subsystem.wb_axi_out.req_wstrb;
  wport.req.bypass = false;
  for (int i = 0; i < axi_interconnect::CACHELINE_WORDS; i++) wport.req.wdata[i] = 0;
  wport.resp.ready = cpu.mem_subsystem.wb_axi_out.resp_ready;
}

void bridge_mem_subsystem_to_axi(SimCpu &cpu) {
  bridge_dcache_mem_subsystem_to_axi(cpu);

  auto &peri_rport = cpu.axi_interconnect.read_ports[axi_interconnect::MASTER_EXTRA_R];
  peri_rport.req.valid = cpu.mem_subsystem.peripheral_axi_read_out.req_valid;
  peri_rport.req.addr = cpu.mem_subsystem.peripheral_axi_read_out.req_addr;
  peri_rport.req.total_size = cpu.mem_subsystem.peripheral_axi_read_out.req_total_size;
  peri_rport.req.id = cpu.mem_subsystem.peripheral_axi_read_out.req_id;
  peri_rport.req.bypass = true;
  peri_rport.resp.ready = cpu.mem_subsystem.peripheral_axi_read_out.resp_ready;

  auto &peri_wport = cpu.axi_interconnect.write_ports[axi_interconnect::MASTER_EXTRA_W];
  peri_wport.req.valid = cpu.mem_subsystem.peripheral_axi_write_out.req_valid;
  peri_wport.req.addr = cpu.mem_subsystem.peripheral_axi_write_out.req_addr;
  peri_wport.req.total_size = cpu.mem_subsystem.peripheral_axi_write_out.req_total_size;
  peri_wport.req.id = cpu.mem_subsystem.peripheral_axi_write_out.req_id;
  peri_wport.req.wstrb = cpu.mem_subsystem.peripheral_axi_write_out.req_wstrb;
  peri_wport.req.bypass = true;
  for (int i = 0; i < axi_interconnect::CACHELINE_WORDS; i++) peri_wport.req.wdata[i] = 0;
  for (int i = 0; i < DCACHE_WORD_NUM && i < axi_interconnect::CACHELINE_WORDS; i++) {
    peri_wport.req.wdata[i] = cpu.mem_subsystem.peripheral_axi_write_out.req_wdata[i];
  }
  peri_wport.resp.ready = cpu.mem_subsystem.peripheral_axi_write_out.resp_ready;
}

struct AxiCpuBoundaryState {
  bool read_req_ready[axi_interconnect::NUM_READ_MASTERS] = {};
  bool read_req_accepted[axi_interconnect::NUM_READ_MASTERS] = {};
  uint8_t read_req_accepted_id[axi_interconnect::NUM_READ_MASTERS] = {};
  bool write_req_ready[axi_interconnect::NUM_WRITE_MASTERS] = {};
  bool write_req_accepted[axi_interconnect::NUM_WRITE_MASTERS] = {};
  uint8_t write_req_accepted_id[axi_interconnect::NUM_WRITE_MASTERS] = {};
  bool read_resp_valid[axi_interconnect::NUM_READ_MASTERS] = {};
  uint8_t read_resp_id[axi_interconnect::NUM_READ_MASTERS] = {};
  bool read_resp_ready[axi_interconnect::NUM_READ_MASTERS] = {};
  bool write_resp_valid[axi_interconnect::NUM_WRITE_MASTERS] = {};
  uint8_t write_resp_id[axi_interconnect::NUM_WRITE_MASTERS] = {};
  bool write_resp_ready[axi_interconnect::NUM_WRITE_MASTERS] = {};
};

void capture_axi_cpu_boundary_state(SimCpu &cpu, AxiCpuBoundaryState &state) {
  for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; ++i) {
    state.read_req_ready[i] = cpu.axi_interconnect.read_ports[i].req.ready;
    state.read_req_accepted[i] = cpu.axi_interconnect.read_req_accepted[i];
    state.read_req_accepted_id[i] =
        cpu.axi_interconnect.read_req_accepted_id[i];
    state.read_resp_valid[i] = cpu.axi_interconnect.read_ports[i].resp.valid;
    state.read_resp_id[i] =
        static_cast<uint8_t>(cpu.axi_interconnect.read_ports[i].resp.id);
    state.read_resp_ready[i] = cpu.axi_interconnect.read_ports[i].resp.ready;
  }
  for (int i = 0; i < axi_interconnect::NUM_WRITE_MASTERS; ++i) {
    state.write_req_ready[i] = cpu.axi_interconnect.write_ports[i].req.ready;
    state.write_req_accepted[i] = cpu.axi_interconnect.write_req_accepted[i];
    state.write_req_accepted_id[i] =
        static_cast<uint8_t>(cpu.axi_interconnect.write_ports[i].req.id);
    state.write_resp_valid[i] = cpu.axi_interconnect.write_ports[i].resp.valid;
    state.write_resp_id[i] =
        static_cast<uint8_t>(cpu.axi_interconnect.write_ports[i].resp.id);
    state.write_resp_ready[i] = cpu.axi_interconnect.write_ports[i].resp.ready;
  }
}

void accumulate_axi_cpu_boundary_state(SimCpu &cpu,
                                       AxiCpuBoundaryState &state) {
  for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; ++i) {
    if (is_dcache_read_master(i)) {
      continue;
    }
    state.read_req_ready[i] =
        state.read_req_ready[i] || cpu.axi_interconnect.read_ports[i].req.ready;
    if (cpu.axi_interconnect.read_req_accepted[i]) {
      state.read_req_accepted[i] = true;
      state.read_req_accepted_id[i] =
          cpu.axi_interconnect.read_req_accepted_id[i];
    }
  }
  for (int i = 0; i < axi_interconnect::NUM_WRITE_MASTERS; ++i) {
    if (is_dcache_write_master(i)) {
      continue;
    }
    state.write_req_ready[i] = state.write_req_ready[i] ||
                               cpu.axi_interconnect.write_ports[i].req.ready;
    if (cpu.axi_interconnect.write_req_accepted[i]) {
      state.write_req_accepted[i] = true;
      state.write_req_accepted_id[i] =
          static_cast<uint8_t>(cpu.axi_interconnect.write_ports[i].req.id);
    }
  }
}

void restore_axi_cpu_boundary_state(SimCpu &cpu,
                                    const AxiCpuBoundaryState &state) {
  for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; ++i) {
    if (is_dcache_read_master(i)) {
      continue;
    }
    cpu.axi_interconnect.read_req_accepted[i] = state.read_req_accepted[i];
    cpu.axi_interconnect.read_req_accepted_id[i] =
        state.read_req_accepted_id[i];
    cpu.axi_interconnect.read_ports[i].req.ready = state.read_req_ready[i];
    cpu.axi_interconnect.read_ports[i].req.accepted =
        state.read_req_accepted[i];
    cpu.axi_interconnect.read_ports[i].req.accepted_id =
        state.read_req_accepted_id[i];
    cpu.axi_interconnect.read_ports[i].resp.ready = state.read_resp_ready[i];
    const bool read_resp_valid = cpu.axi_interconnect.read_ports[i].resp.valid;
    const uint8_t read_resp_id =
        static_cast<uint8_t>(cpu.axi_interconnect.read_ports[i].resp.id);
    const bool read_resp_from_extra_tick =
        read_resp_valid &&
        !(state.read_resp_valid[i] && state.read_resp_id[i] == read_resp_id);
    const bool read_resp_matches_new_accept =
        state.read_req_accepted[i] && read_resp_valid &&
        read_resp_id == state.read_req_accepted_id[i];
    if (read_resp_matches_new_accept) {
      cpu.axi_interconnect.read_ports[i].resp.valid = false;
    }
    if (read_resp_from_extra_tick || read_resp_matches_new_accept) {
      cpu.axi_interconnect.read_ports[i].resp.ready = false;
    }
  }
  for (int i = 0; i < axi_interconnect::NUM_WRITE_MASTERS; ++i) {
    if (is_dcache_write_master(i)) {
      continue;
    }
    cpu.axi_interconnect.write_req_accepted[i] = state.write_req_accepted[i];
    cpu.axi_interconnect.write_ports[i].req.ready = state.write_req_ready[i];
    cpu.axi_interconnect.write_ports[i].req.accepted =
        state.write_req_accepted[i];
    cpu.axi_interconnect.write_ports[i].resp.ready = state.write_resp_ready[i];
    const bool write_resp_valid =
        cpu.axi_interconnect.write_ports[i].resp.valid;
    const uint8_t write_resp_id =
        static_cast<uint8_t>(cpu.axi_interconnect.write_ports[i].resp.id);
    const bool write_resp_from_extra_tick =
        write_resp_valid &&
        !(state.write_resp_valid[i] && state.write_resp_id[i] == write_resp_id);
    const bool write_resp_matches_new_accept =
        state.write_req_accepted[i] && write_resp_valid &&
        write_resp_id == state.write_req_accepted_id[i];
    if (write_resp_matches_new_accept) {
      cpu.axi_interconnect.write_ports[i].resp.valid = false;
    }
    if (write_resp_from_extra_tick || write_resp_matches_new_accept) {
      cpu.axi_interconnect.write_ports[i].resp.ready = false;
    }
  }
}

void mask_axi_cpu_boundary_for_internal_tick(SimCpu &cpu) {
  for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; ++i) {
    if (is_dcache_read_master(i)) {
      continue;
    }
    auto &port = cpu.axi_interconnect.read_ports[i];
    port.req.valid = false;
    port.req.addr = 0;
    port.req.total_size = 0;
    port.req.id = 0;
    port.req.bypass = false;
    port.resp.ready = false;
  }
  for (int i = 0; i < axi_interconnect::NUM_WRITE_MASTERS; ++i) {
    if (is_dcache_write_master(i)) {
      continue;
    }
    auto &port = cpu.axi_interconnect.write_ports[i];
    port.req.valid = false;
    port.req.addr = 0;
    port.req.wdata.clear();
    port.req.wstrb = 0;
    port.req.total_size = 0;
    port.req.id = 0;
    port.req.bypass = false;
    port.resp.ready = false;
  }
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

void axi_subsystem_comb_inputs_dcache_fast_tick(SimCpu &cpu) {
  AxiInternalTickBoundaryState boundary_state{};
  capture_axi_internal_tick_boundary_state(cpu.axi_interconnect,
                                           boundary_state);
  cpu.axi_interconnect.comb_inputs();
  restore_axi_internal_tick_boundary_state_except_dcache(cpu.axi_interconnect,
                                                        boundary_state);
  connect_axi_master_outputs(cpu.axi_interconnect.axi_ddr_io, cpu.axi_ddr.io);
  connect_axi_master_outputs(cpu.axi_interconnect.axi_mmio_io, cpu.axi_mmio.io);
  cpu.axi_ddr.comb_inputs();
  cpu.axi_mmio.comb_inputs();
}

void axi_subsystem_seq(SimCpu &cpu) {
  cpu.mem_subsystem.llc_seq(cpu.axi_interconnect.get_llc_table_out(),
                            cpu.axi_interconnect.get_llc_perf_counters());
  cpu.axi_interconnect.seq();
  cpu.axi_ddr.seq();
  cpu.axi_mmio.seq();
}

bool axi_subsystem_quiescent(SimCpu &cpu) {
  return cpu.axi_interconnect.idle_quiescent() &&
         cpu.axi_ddr.idle_quiescent() && cpu.axi_mmio.idle_quiescent() &&
         (!kDcacheFastDomainEnabled ||
          cpu.mem_subsystem.dcache_domain_quiescent());
}

void run_extra_axi_subsystem_ticks(SimCpu &cpu) {
  if (CONFIG_AXI_SUBSYSTEM_FREQ_DIV <= 1u) {
    return;
  }

  AxiCpuBoundaryState cpu_boundary_state{};
  capture_axi_cpu_boundary_state(cpu, cpu_boundary_state);
  for (uint32_t tick = 1; tick < CONFIG_AXI_SUBSYSTEM_FREQ_DIV; ++tick) {
    // Once a tick has left the whole AXI/LLC/DDR/MMIO subsystem and the L1D
    // fast domain quiescent, the remaining ticks are fixed points: their
    // boundary contribution is already accumulated.
    if (tick >= 2 && axi_subsystem_quiescent(cpu)) {
#ifdef SIM_DDR_DS3
      cpu.axi_ddr.ds3_advance(CONFIG_AXI_SUBSYSTEM_FREQ_DIV - tick);  // keep DRAM time (refresh) running
#endif
      break;
    }
    mask_axi_cpu_boundary_for_internal_tick(cpu);
    axi_subsystem_comb_outputs(cpu);
    if (kDcacheFastDomainEnabled) {
      bridge_dcache_axi_to_mem_subsystem(cpu);
      cpu.mem_subsystem.dcache_fast_comb(false);
      bridge_dcache_mem_subsystem_to_axi(cpu);
    }
    axi_subsystem_comb_inputs_dcache_fast_tick(cpu);
    if (kDcacheFastDomainEnabled) cpu.mem_subsystem.dcache_fast_seq(false);
    axi_subsystem_seq(cpu);
    accumulate_axi_cpu_boundary_state(cpu, cpu_boundary_state);
  }
  restore_axi_cpu_boundary_state(cpu, cpu_boundary_state);
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
      uint32_t dma_paddr = 0;
      size_t dma_size = 0;
      while (mem_subsystem.consume_dma_write(dma_paddr, dma_size)) {
      }
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
  if (tbhook::enabled) {
    const uint8_t tp = static_cast<uint8_t>(cpu->back.csr->privilege_1);
    const uint32_t c = tp == RISCV_MODE_M ? cpu->back.csr->CSR_RegFile_1[csr_mcause]
                                          : cpu->back.csr->CSR_RegFile_1[csr_scause];
    tbhook::on_interrupt(c, tp, cpu->back.csr->CSR_RegFile_1[csr_mip]);
  }
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
  back.set_lsu_ptw_mem_port(mem_subsystem.dtlb_ptw_port);

  mem_subsystem.lsu2dcache = &back.out.lsu2dcache;
  mem_subsystem.dcache2lsu = &back.in.dcache2lsu;

  mem_subsystem.icache_req = &icache_req;
  mem_subsystem.icache_resp = &icache_resp;

  front.icache_ptw_walk_port = mem_subsystem.itlb_walk_port;
  front.icache_ptw_mem_port = mem_subsystem.itlb_ptw_port;
  front.icache_mem_req_port = &icache_req;
  front.icache_mem_resp_port = &icache_resp;
  // Keep a single active SoC memory topology. DCache/PTW/peripheral already use
  // the top-level interconnect; frontend icache should use the same shared path
  // whenever the real BPU frontend is enabled. Oracle mode does not step the
  // icache model, so it simply leaves the port disconnected instead of falling
  // back to MemSubsystem's legacy private AXI runtime.
  front.icache_mem_read_port =
      &axi_interconnect.read_ports[axi_interconnect::MASTER_ICACHE];

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
  front.init();
  oracle_pending_valid = false;
  oracle_pending_out = {};
  front2back_hold = {};
  front2back_hold_1 = {};
  front2back_hold_live = front2back_hold_1_live = false;
  peripheral_resp_hold = {};
  peripheral_resp_hold_1 = {};
}

void SimCpu::reinit_frontend_after_restore() {
  // FAST/CKPT switch starts O3 from a mid-execution architectural snapshot.
  // Reinitialize frontend-local state so BPU/icache/predecode static latches do
  // not carry stale reset-era contents into the restored control flow.
  clear_axi_master_inputs(axi_interconnect);
  front.init();
  oracle_pending_valid = false;
  oracle_pending_out = {};
  front2back_hold = {};
  front2back_hold_1 = {};
  front2back_hold_live = front2back_hold_1_live = false;
}

void SimCpu::sync_mmio_devices_from_backing() {
  sync_uart_registers_from_backing(axi_uart);
  mem_subsystem.sync_mmio_devices_from_backing();
}

// 强制重置前端 PC (用于 FAST 模式切换)
void SimCpu::restore_pc(uint32_t pc) {
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

void SimCpu::cycle() {
  ctx.perf.cycle++;
  // BackTop inputs are combinational wires. Re-drive the only intentionally
  // retained payload from an explicit top-level register before producers run.
  back.clear_comb_inputs();
  if (front2back_hold_live) static_cast<FrontPreIO &>(back.in) = front2back_hold;
  back.in.peripheral_resp = peripheral_resp_hold;
  // 统一在此处刷新 CSR 状态，供本拍 front/back 组合逻辑共同使用。
  back.comb_csr_status();

  clear_axi_master_inputs(axi_interconnect);

  front_comb_before_back();

  // AXI phase-1: slave outputs -> interconnect outputs.
  // Interconnect outputs (req.ready/resp.valid) are bridged into MemSubsystem
  // before backend comb, so DCache responses from the previous DCache stage can
  // be visible to LSU in this CPU cycle.
  axi_subsystem_comb_outputs(*this);
  bridge_axi_to_mem_subsystem(*this);

  mem_subsystem.comb_outputs();

  back.comb();

  mem_subsystem.comb_inputs();
  peripheral_resp_hold_comb();

  bridge_mem_subsystem_to_axi(*this);

  // AXI phase-2: master requests -> interconnect -> slave inputs.
  axi_subsystem_comb_inputs(*this);

  back2front_comb();
  front2back_hold_comb();

  front.seq_write_bpu();
  front.seq_back2front();
  front2back_hold_seq();
  peripheral_resp_hold_seq();

  back.seq();
  mem_subsystem.seq();
  axi_subsystem_seq(*this);
  run_extra_axi_subsystem_ticks(*this);
  ctx.perf.perf_maybe_capture_simtime_snapshot();

  if (ctx.exit_reason != ExitReason::NONE) {
    printf("Simulation Exited with Reason: %d\n", (int)ctx.exit_reason);
    return;
  }

  if (back.out.mispred || back.out.flush) {
    back.number_PC = back.out.redirect_pc;
  }
}

// The hold registers stay all-zero in the common (no stall) case; the live flags let the cycle skip copying zeros.
void SimCpu::front2back_hold_comb() {
  if (front2back_hold_1_live) {
    front2back_hold_1 = {};
    front2back_hold_1_live = false;
  }
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
}

void SimCpu::peripheral_resp_hold_comb() {
  peripheral_resp_hold_1 = back.in.peripheral_resp;
}

void SimCpu::peripheral_resp_hold_seq() {
  peripheral_resp_hold = peripheral_resp_hold_1;
}

void SimCpu::front_comb_before_back() {
  const bool registered_refetch = static_cast<bool>(front.back2front.refetch);
  const bool frontend_can_read = !front.back2front_stall || registered_refetch;

  const auto reset = front.in.reset;
  front.in = front.back2front;
  front.in.reset = reset;
  front.in.csr_status = &back.csr_status;

  if (frontend_can_read) {
    front.in.FIFO_read_enable = true;

    front.seq_read_bpu();
    front.comb_bpu();

    bool no_taken = true;
    for (int j = 0; j < BACKEND_FETCH_WIDTH; j++) {
      back.in.valid[j] =
          no_taken && front.out.FIFO_valid && front.out.inst_valid[j];
      back.in.pc[j] = front.out.pc[j];
      back.in.predict_next_fetch_address[j] =
          front.out.predict_next_fetch_address;
      back.in.page_fault_inst[j] = front.out.page_fault_inst[j];
      back.in.inst[j] = front.out.instructions[j];

      back.in.predict_dir[j] = front.out.predict_dir[j];
      if (back.in.valid[j] && front.out.predict_dir[j])
        no_taken = false;
    }
    back.in.meta = front.out.meta;
    back.in.front_stall = static_cast<bool>(front.out.commit_stall);
  } else {
    front.in.FIFO_read_enable = false;
    front.in.refetch = false;
    front.seq_read_bpu();
    front.comb_bpu();
    back.in.front_stall = static_cast<bool>(front.out.commit_stall);
  }
}

bool SimCpu::ready_to_exit() const {
  Assert(false && "SimCpu::ready_to_exit: not implemented");
  return false;
  // if (back.lsu->has_committed_store_pending()) {
  //   return false;
  // }

  // const auto &peri = mem_subsystem.get_peripheral_axi().cur;
  // if (peri.busy || peri.req_accepted || peri.resp_valid) {
  //   return false;
  // }

  // return true;
}
void SimCpu::back2front_comb() {
  front.back2front_1 = front.back2front;
  front.back2front_stall_1 = back.out.stall;
  front.back2front_1.FIFO_read_enable = false;
  front.back2front_1.csr_status = &back.csr_status;
  front.back2front_1.refetch = back.out.mispred || back.out.flush;
  front.back2front_1.rf_valid = back.out.rf_valid && !back.out.flush;
  front.back2front_1.rf_blk = back.out.rf_blk;
  front.back2front_1.rf_slot = back.out.rf_slot;
  front.back2front_1.rf_type = back.out.rf_type;
  front.back2front_1.rf_pc = back.out.rf_pc;
  front.back2front_1.itlb_flush = back.out.itlb_flush;
  front.back2front_1.fence_i = back.out.fence_i;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    InstInfo *inst = &back.out.commit_entry[i].uop;
    const auto &commit_extra = back.rob->out.commit_extra[i];
    const auto &ftq_info = back.ftq_commit_info.resp[i];
    front.back2front_1.back2front_valid[i] = back.out.commit_entry[i].valid;
    for (int j = 0; j < 4; j++) {
      front.back2front_1.tage_tag[i][j] = 0;
    }
    front.back2front_1.sc_used[i] = false;
    front.back2front_1.sc_pred[i] = false;
    front.back2front_1.sc_sum[i] = 0;
    for (int t = 0; t < BPU_SCL_META_NTABLE; ++t) {
      front.back2front_1.sc_idx[i][t] = 0;
    }
    front.back2front_1.loop_used[i] = false;
    front.back2front_1.loop_hit[i] = false;
    front.back2front_1.loop_pred[i] = false;
    front.back2front_1.loop_idx[i] = 0;
    front.back2front_1.loop_tag[i] = 0;

    if (front.back2front_1.back2front_valid[i]) {
      front.back2front_1.predict_dir[i] = ftq_info.pred_taken;
      front.back2front_1.predict_base_pc[i] = ftq_info.pc;
      front.back2front_1.actual_dir[i] =
          ((*inst)->type == JAL || (*inst)->type == JALR) ? true : inst->br_taken;
      front.back2front_1.actual_target[i] =
          (is_branch((*inst)->type) || (*inst)->type == JAL || (*inst)->type == JALR)
              ? back.out.commit_entry[i].uop.diag_val
              : ftq_info.pc + 4;
      int br_type = BR_NONCTL;
      if (is_branch((*inst)->type)) {
        br_type = BR_DIRECT;
      }
      if ((*inst)->type == JAL) {
        br_type = BR_JAL;
      }
      if ((*inst)->type == JAL && inst->dest_en && (*inst)->dest_areg == 1) {
        br_type = BR_CALL;
      } else if ((*inst)->type == JALR) {
        if (commit_extra.src1_areg == 1 && (*inst)->dest_areg == 0 &&
            commit_extra.imm == 0)
          br_type = BR_RET;
        else if (bpuf::cfg_icall() && (*inst)->dest_areg == 1)
          br_type = BR_ICALL;
        else
          br_type = BR_IDIRECT;
      }

      front.back2front_1.actual_br_type[i] = br_type;
      front.back2front_1.alt_pred[i] = ftq_info.alt_pred;
      front.back2front_1.altpcpn[i] = ftq_info.altpcpn;
      front.back2front_1.pcpn[i] = ftq_info.pcpn;
      front.back2front_1.sc_used[i] = ftq_info.sc_used;
      front.back2front_1.sc_pred[i] = ftq_info.sc_pred;
      front.back2front_1.sc_sum[i] = ftq_info.sc_sum;
      for (int t = 0; t < BPU_SCL_META_NTABLE; ++t) {
        front.back2front_1.sc_idx[i][t] = ftq_info.sc_idx[t];
      }
      front.back2front_1.loop_used[i] = ftq_info.loop_used;
      front.back2front_1.loop_hit[i] = ftq_info.loop_hit;
      front.back2front_1.loop_pred[i] = ftq_info.loop_pred;
      front.back2front_1.loop_idx[i] = ftq_info.loop_idx;
      front.back2front_1.loop_tag[i] = ftq_info.loop_tag;
      for (int j = 0; j < 4; j++) { // TN_MAX = 4 (分支预测相关索引)
        front.back2front_1.tage_idx[i][j] = ftq_info.tage_idx[j];
        front.back2front_1.tage_tag[i][j] = ftq_info.tage_tag[j];
      }
    }
  }

  if (back.out.mispred || back.out.flush) {
    front.back2front_1.refetch_address = back.out.redirect_pc;
  }
}
