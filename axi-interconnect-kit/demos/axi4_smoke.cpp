#include "AXI_Interconnect.h"
#include "SimDDR.h"
#include <cstdio>

// Demo scope:
// - Build a minimal AXI4 read path: Master(ICACHE) -> Interconnect -> Router -> SimDDR.
// - Drive one read request and verify end-to-end handshake completion.
// - This is a smoke test, not a full stress test.

namespace {
constexpr uint32_t kDemoMemWords = 0x100000;
uint32_t g_demo_mem[kDemoMemWords] = {};
} // namespace

uint32_t *p_memory = g_demo_mem;
long long sim_time = 0;

namespace {

void wire_ddr_to_interconnect(axi_interconnect::AXI_Interconnect &interconnect,
                              sim_ddr::SimDDR &ddr) {
  interconnect.axi_io.ar.arready = ddr.io.ar.arready;
  interconnect.axi_io.r.rvalid = ddr.io.r.rvalid;
  interconnect.axi_io.r.rid = ddr.io.r.rid;
  interconnect.axi_io.r.rdata = ddr.io.r.rdata;
  interconnect.axi_io.r.rlast = ddr.io.r.rlast;
  interconnect.axi_io.r.rresp = ddr.io.r.rresp;
  interconnect.axi_io.aw.awready = ddr.io.aw.awready;
  interconnect.axi_io.w.wready = ddr.io.w.wready;
  interconnect.axi_io.b.bvalid = ddr.io.b.bvalid;
  interconnect.axi_io.b.bid = ddr.io.b.bid;
  interconnect.axi_io.b.bresp = ddr.io.b.bresp;
}

void wire_interconnect_to_ddr(axi_interconnect::AXI_Interconnect &interconnect,
                              sim_ddr::SimDDR &ddr) {
  ddr.io.ar.arvalid = interconnect.axi_io.ar.arvalid;
  ddr.io.ar.araddr = interconnect.axi_io.ar.araddr;
  ddr.io.ar.arid = interconnect.axi_io.ar.arid;
  ddr.io.ar.arlen = interconnect.axi_io.ar.arlen;
  ddr.io.ar.arsize = interconnect.axi_io.ar.arsize;
  ddr.io.ar.arburst = interconnect.axi_io.ar.arburst;

  ddr.io.aw.awvalid = interconnect.axi_io.aw.awvalid;
  ddr.io.aw.awaddr = interconnect.axi_io.aw.awaddr;
  ddr.io.aw.awid = interconnect.axi_io.aw.awid;
  ddr.io.aw.awlen = interconnect.axi_io.aw.awlen;
  ddr.io.aw.awsize = interconnect.axi_io.aw.awsize;
  ddr.io.aw.awburst = interconnect.axi_io.aw.awburst;

  ddr.io.w.wvalid = interconnect.axi_io.w.wvalid;
  ddr.io.w.wdata = interconnect.axi_io.w.wdata;
  ddr.io.w.wstrb = interconnect.axi_io.w.wstrb;
  ddr.io.w.wlast = interconnect.axi_io.w.wlast;

  ddr.io.r.rready = interconnect.axi_io.r.rready;
  ddr.io.b.bready = interconnect.axi_io.b.bready;
}

void clear_master_inputs(axi_interconnect::AXI_Interconnect &interconnect) {
  for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; i++) {
    interconnect.read_ports[i].req.valid = false;
    interconnect.read_ports[i].resp.ready = false;
  }
  for (int i = 0; i < axi_interconnect::NUM_WRITE_MASTERS; i++) {
    interconnect.write_ports[i].req.valid = false;
    interconnect.write_ports[i].resp.ready = false;
  }
}

} // namespace

int main() {
  axi_interconnect::AXI_Interconnect interconnect;
  sim_ddr::SimDDR ddr;
  interconnect.init();
  ddr.init();

  auto &port = interconnect.read_ports[axi_interconnect::MASTER_ICACHE];
  bool accepted = false;
  bool ar_issued = false;
  bool responded = false;

  // Step-by-step timeline:
  // Step 0: reset upstream master inputs for this cycle
  // Step 1: (until AR is observed) drive one ICACHE read request
  // Step 2: keep response channel ready so return data can be consumed
  // Step 3: evaluate DDR comb outputs first (provides ready/valid back to fabric)
  // Step 4: wire DDR -> interconnect AXI signals
  // Step 5: evaluate interconnect comb (outputs then inputs)
  // Step 6: wire interconnect -> DDR AXI signals
  // Step 7: evaluate DDR comb inputs with latest routed signals
  // Step 8: sample handshake/result conditions
  // Step 9: advance sequential state for both blocks
  for (int cycle = 0; cycle < 4000 && !responded; cycle++) {
    // Step 0
    clear_master_inputs(interconnect);

    // Step 1
    if (!ar_issued) {
      port.req.valid = true;
      port.req.addr = 0x1000;
      port.req.total_size = 63;
      port.req.id = 1;
    }

    // Step 2
    port.resp.ready = true;

    // Steps 3-7
    ddr.comb_outputs();
    wire_ddr_to_interconnect(interconnect, ddr);
    interconnect.comb_outputs();
    interconnect.comb_inputs();
    wire_interconnect_to_ddr(interconnect, ddr);
    ddr.comb_inputs();

    // Step 8
    accepted = accepted || (port.req.valid && port.req.ready);
    ar_issued = ar_issued || (interconnect.axi_io.ar.arvalid && interconnect.axi_io.ar.arready);
    responded = port.resp.valid;

    // Step 9
    ddr.seq();
    interconnect.seq();
  }

  // Pass criteria:
  // 1) upstream request was accepted by interconnect
  // 2) AXI AR transaction was issued
  // 3) upstream read response returned
  if (!accepted || !ar_issued || !responded) {
    std::printf("AXI4 smoke demo failed: accepted=%d ar_issued=%d responded=%d\n",
                accepted ? 1 : 0, ar_issued ? 1 : 0, responded ? 1 : 0);
    return 1;
  }

  std::printf("AXI4 smoke demo passed\n");
  return 0;
}
