// The AXI side of the SoC (include/AxiSubsystem.h).
#include "AxiSubsystem.h"
#include "MemSubsystem.h"
#include "PhysMemory.h"
#include "config.h"

namespace {
axi_interconnect::AXI_LLCConfig default_llc_config() {
  axi_interconnect::AXI_LLCConfig llc_cfg;
  llc_cfg.enable = (CONFIG_AXI_LLC_ENABLE != 0);
  llc_cfg.size_bytes = CONFIG_AXI_LLC_SIZE_BYTES;
  llc_cfg.ways = CONFIG_AXI_LLC_WAYS;
  llc_cfg.mshr_num = CONFIG_AXI_LLC_MSHR_NUM;
  llc_cfg.lookup_latency = CONFIG_AXI_LLC_LOOKUP_LATENCY;
  return llc_cfg;
}
} // namespace

void AxiSubsystem::init() {
  interconnect.set_llc_config(default_llc_config());
  interconnect.init();
  ddr.init();
  mmio.init();
  mmio.add_device(UART_ADDR_BASE, UART_MMIO_SIZE, &uart);
}

void AxiSubsystem::sync_uart_from_backing() {
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

void AxiSubsystem::comb_begin() {
  // only valid: the other request fields are read under valid (a master that requests drives them all)
  for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; i++) out.read_ports[i].req.valid = false;
  for (int i = 0; i < axi_interconnect::NUM_WRITE_MASTERS; i++) out.write_ports[i].req.valid = false;
}

void AxiSubsystem::comb_outputs() {
  llc->llc_comb_outputs();
  interconnect.set_llc_lookup_in(llc->llc_lookup_in());
  ddr.comb_outputs();
  mmio.comb_outputs();
  connect_slave_outputs(interconnect.axi_ddr_io, ddr.io);
  connect_slave_outputs(interconnect.axi_mmio_io, mmio.io);
  interconnect.comb_outputs();
}

void AxiSubsystem::comb_inputs() {
  interconnect.comb_inputs();
  connect_master_outputs(interconnect.axi_ddr_io, ddr.io);
  connect_master_outputs(interconnect.axi_mmio_io, mmio.io);
  ddr.comb_inputs();
  mmio.comb_inputs();
}

void AxiSubsystem::seq() {
  llc->llc_seq(interconnect.get_llc_table_out());
  interconnect.seq();
  ddr.seq();
  mmio.seq();
}

void AxiSubsystem::connect_slave_outputs(sim_ddr::SimDDR_IO_t &master_io, const sim_ddr::SimDDR_IO_t &slave_io) {
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

void AxiSubsystem::connect_master_outputs(const sim_ddr::SimDDR_IO_t &master_io, sim_ddr::SimDDR_IO_t &slave_io) {
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
