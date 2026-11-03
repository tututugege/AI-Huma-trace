/**
 * @file axi_interconnect_aw_latch_test.cpp
 * @brief Focused AW-latch coverage for AXI_Interconnect.
 */

#include <cstdio>

#define private public
#include "AXI_Interconnect.h"
#undef private
#include "SimDDR.h"

uint32_t *p_memory = nullptr;
long long sim_time = 0;

namespace {

void clear_upstream_inputs(axi_interconnect::AXI_Interconnect &interconnect) {
  for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; i++) {
    interconnect.read_ports[i].req.valid = false;
    interconnect.read_ports[i].req.addr = 0;
    interconnect.read_ports[i].req.total_size = 0;
    interconnect.read_ports[i].req.id = 0;
    interconnect.read_ports[i].req.bypass = false;
    interconnect.read_ports[i].resp.ready = false;
  }
  for (int i = 0; i < axi_interconnect::NUM_WRITE_MASTERS; ++i) {
    interconnect.write_ports[i].req.valid = false;
    interconnect.write_ports[i].req.addr = 0;
    interconnect.write_ports[i].req.wdata.clear();
    interconnect.write_ports[i].req.wstrb.clear();
    interconnect.write_ports[i].req.total_size = 0;
    interconnect.write_ports[i].req.id = 0;
    interconnect.write_ports[i].req.bypass = false;
    interconnect.write_ports[i].resp.ready = false;
  }
}

bool test_aw_latching_ready_first() {
  std::printf("=== Test: AW latching (mocked awready) ===\n");

  axi_interconnect::AXI_Interconnect interconnect;
  interconnect.init();

  axi_interconnect::WideData256_t wdata;
  wdata.clear();
  wdata[0] = 0xDEADBEEFu;

  const uint32_t req_addr = 0x3000;
  const uint8_t req_id = 1;

  bool req_valid = true;
  bool saw_ready = false;
  bool have_latched_id = false;
  uint8_t latched_awid = 0;

  for (int cyc = 0; cyc < 8; cyc++) {
    clear_upstream_inputs(interconnect);

    interconnect.axi_io.aw.awready = (cyc >= 5);
    interconnect.axi_io.w.wready = true;
    interconnect.axi_io.b.bvalid = false;
    interconnect.axi_io.b.bid = 0;
    interconnect.axi_io.b.bresp = sim_ddr::AXI_RESP_OKAY;
    interconnect.axi_io.ar.arready = true;
    interconnect.axi_io.r.rvalid = false;
    interconnect.axi_io.r.rid = 0;
    interconnect.axi_io.r.rdata = 0;
    interconnect.axi_io.r.rlast = false;
    interconnect.axi_io.r.rresp = sim_ddr::AXI_RESP_OKAY;

    interconnect.comb_outputs();
    const bool req_ready = interconnect.write_port.req.ready;

    if (req_valid) {
      interconnect.write_port.req.valid = true;
      interconnect.write_port.req.addr = req_addr;
      interconnect.write_port.req.wdata = wdata;
      interconnect.write_port.req.wstrb = 0xFULL;
      interconnect.write_port.req.total_size = 3;
      interconnect.write_port.req.id = req_id;
      interconnect.write_port.req.bypass = false;
    }

    if (req_valid && req_ready) {
      saw_ready = true;
      req_valid = false;
    }

    interconnect.comb_inputs();

    if (saw_ready && cyc >= 2 && cyc < 5 && !req_valid) {
      if (!interconnect.axi_io.aw.awvalid) {
        std::printf("FAIL: awvalid dropped under backpressure\n");
        return false;
      }
      if (interconnect.axi_io.aw.awaddr != req_addr ||
          interconnect.axi_io.aw.awlen != 0) {
        std::printf("FAIL: latched AW changed addr=0x%x len=%u\n",
                    interconnect.axi_io.aw.awaddr,
                    interconnect.axi_io.aw.awlen);
        return false;
      }
      if (!have_latched_id) {
        latched_awid = interconnect.axi_io.aw.awid;
        have_latched_id = true;
      } else if (interconnect.axi_io.aw.awid != latched_awid) {
        std::printf("FAIL: latched AW id changed exp=0x%x got=0x%x\n",
                    latched_awid, interconnect.axi_io.aw.awid);
        return false;
      }
    }

    interconnect.seq();
    sim_time++;
  }

  if (!saw_ready) {
    std::printf("FAIL: upstream write req never saw ready\n");
    return false;
  }
  if (!have_latched_id) {
    std::printf("FAIL: never observed latched AW under backpressure\n");
    return false;
  }

  std::printf("PASS\n");
  return true;
}

} // namespace

int main() {
  return test_aw_latching_ready_first() ? 0 : 1;
}
