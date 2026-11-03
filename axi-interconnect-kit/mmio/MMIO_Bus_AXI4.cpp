/**
 * @file MMIO_Bus_AXI4.cpp
 * @brief AXI4 MMIO bus (single outstanding read/write stream, 32-bit data).
 */

#include "MMIO_Bus_AXI4.h"
#include <algorithm>
#include <cstring>

namespace mmio {

void MMIO_Bus_AXI4::init() {
  regions.clear();
  r_pending = {};
  w_pending = {};
  w_resp = {};

  io.aw.awready = false;
  io.aw.awvalid = false;
  io.aw.awid = 0;
  io.aw.awaddr = 0;
  io.aw.awlen = 0;
  io.aw.awsize = sim_ddr::AXI_SIZE_CODE;
  io.aw.awburst = sim_ddr::AXI_BURST_INCR;

  io.w.wready = false;
  io.w.wvalid = false;
  io.w.wdata = 0;
  io.w.wstrb = 0;
  io.w.wlast = false;

  io.b.bvalid = false;
  io.b.bid = 0;
  io.b.bresp = sim_ddr::AXI_RESP_OKAY;

  io.ar.arready = false;
  io.ar.arvalid = false;
  io.ar.arid = 0;
  io.ar.araddr = 0;
  io.ar.arlen = 0;
  io.ar.arsize = sim_ddr::AXI_SIZE_CODE;
  io.ar.arburst = sim_ddr::AXI_BURST_INCR;

  io.r.rvalid = false;
  io.r.rid = 0;
  io.r.rdata = 0;
  io.r.rresp = sim_ddr::AXI_RESP_OKAY;
  io.r.rlast = false;
}

void MMIO_Bus_AXI4::add_device(uint32_t base, uint32_t size, MMIO_Device *dev) {
  regions.push_back({base, size, dev});
}

MMIO_Device *MMIO_Bus_AXI4::find_device(uint32_t addr, uint32_t &base) const {
  for (const auto &r : regions) {
    if (addr >= r.base && addr < (r.base + r.size)) {
      base = r.base;
      return r.dev;
    }
  }
  base = 0;
  return nullptr;
}

uint8_t MMIO_Bus_AXI4::beat_bytes(uint8_t size) {
  uint32_t bytes = (size < 8) ? (1u << size) : 4u;
  if (bytes == 0) {
    return 1;
  }
  return static_cast<uint8_t>(std::min<uint32_t>(bytes, sim_ddr::AXI_DATA_BYTES));
}

uint32_t MMIO_Bus_AXI4::beat_addr(uint32_t base_addr, uint8_t burst,
                                  uint8_t size, uint8_t beat_idx) {
  if (burst == sim_ddr::AXI_BURST_FIXED) {
    return base_addr;
  }
  return base_addr + (static_cast<uint32_t>(beat_idx) << size);
}

sim_ddr::axi_data_t MMIO_Bus_AXI4::load_beat(const uint8_t *p, uint8_t nbytes) {
  sim_ddr::axi_data_t value{};
  for (uint8_t i = 0; i < nbytes; ++i) {
    axi_compat::set_byte(value, i, p[i]);
  }
  return value;
}

void MMIO_Bus_AXI4::store_beat(uint8_t *p, sim_ddr::axi_data_t v, uint8_t nbytes) {
  for (uint8_t i = 0; i < nbytes; ++i) {
    p[i] = axi_compat::get_byte(v, i);
  }
}

void MMIO_Bus_AXI4::build_read_beat() {
  if (!r_pending.active || r_pending.beat_valid) {
    return;
  }

  uint32_t addr = beat_addr(r_pending.addr, r_pending.burst, r_pending.size,
                            r_pending.beat_idx);
  uint32_t base = 0;
  MMIO_Device *dev = find_device(addr, base);

  uint8_t bytes[sim_ddr::AXI_DATA_BYTES] = {0};
  uint8_t nbytes = beat_bytes(r_pending.size);
  if (dev) {
    dev->read(addr, bytes, nbytes);
    r_pending.beat_resp = sim_ddr::AXI_RESP_OKAY;
  } else {
    r_pending.beat_resp = sim_ddr::AXI_RESP_DECERR;
  }

  r_pending.beat_data = load_beat(bytes, nbytes);
  r_pending.beat_valid = true;
}

void MMIO_Bus_AXI4::comb_outputs() {
  io.ar.arready = false;
  io.aw.awready = false;
  io.w.wready = false;
  io.r.rvalid = false;
  io.r.rid = 0;
  io.r.rdata = 0;
  io.r.rresp = sim_ddr::AXI_RESP_OKAY;
  io.r.rlast = false;
  io.b.bvalid = false;
  io.b.bid = 0;
  io.b.bresp = sim_ddr::AXI_RESP_OKAY;

  // Single outstanding read stream.
  if (!r_pending.active) {
    io.ar.arready = true;
  }

  // Single outstanding write stream/response.
  if (!w_pending.active && !w_resp.active) {
    io.aw.awready = true;
  }
  if (w_pending.active) {
    io.w.wready = true;
  }

  if (r_pending.active && r_pending.beat_valid) {
    io.r.rvalid = true;
    io.r.rid = r_pending.id;
    io.r.rdata = r_pending.beat_data;
    io.r.rresp = r_pending.beat_resp;
    io.r.rlast = (r_pending.beat_idx == r_pending.len);
  }

  if (w_resp.active && w_resp.latency_cnt >= MMIO_LATENCY) {
    io.b.bvalid = true;
    io.b.bid = w_resp.id;
    io.b.bresp = w_resp.resp;
  }
}

void MMIO_Bus_AXI4::comb_inputs() {
  // no-op
}

void MMIO_Bus_AXI4::seq() {
  for (auto &r : regions) {
    if (r.dev != nullptr) {
      r.dev->tick();
    }
  }

  // Accept AR
  if (io.ar.arvalid && io.ar.arready) {
    r_pending = {};
    r_pending.active = true;
    r_pending.id = io.ar.arid;
    r_pending.addr = io.ar.araddr;
    r_pending.len = io.ar.arlen;
    r_pending.size = io.ar.arsize;
    r_pending.burst = io.ar.arburst;
    r_pending.beat_idx = 0;
    r_pending.latency_cnt = 0;
    r_pending.beat_valid = false;
    r_pending.beat_data = 0;
    r_pending.beat_resp = sim_ddr::AXI_RESP_OKAY;
  }

  // Read latency progression and beat build.
  if (r_pending.active && !r_pending.beat_valid) {
    if (r_pending.latency_cnt < MMIO_LATENCY) {
      r_pending.latency_cnt++;
    }
    if (r_pending.latency_cnt >= MMIO_LATENCY) {
      build_read_beat();
    }
  }

  // Consume R beat
  if (io.r.rvalid && io.r.rready && r_pending.active && r_pending.beat_valid) {
    if (io.r.rlast) {
      r_pending.active = false;
      r_pending.beat_valid = false;
    } else {
      r_pending.beat_idx++;
      r_pending.latency_cnt = 0;
      r_pending.beat_valid = false;
      r_pending.beat_data = 0;
      r_pending.beat_resp = sim_ddr::AXI_RESP_OKAY;
    }
  }

  // Accept AW
  if (io.aw.awvalid && io.aw.awready) {
    w_pending = {};
    w_pending.active = true;
    w_pending.id = io.aw.awid;
    w_pending.addr = io.aw.awaddr;
    w_pending.len = io.aw.awlen;
    w_pending.size = io.aw.awsize;
    w_pending.burst = io.aw.awburst;
    w_pending.beat_idx = 0;
    w_pending.resp = sim_ddr::AXI_RESP_OKAY;
  }

  // Accept W beat
  if (io.w.wvalid && io.w.wready && w_pending.active) {
    uint32_t addr = beat_addr(w_pending.addr, w_pending.burst, w_pending.size,
                              w_pending.beat_idx);
    uint32_t base = 0;
    MMIO_Device *dev = find_device(addr, base);

    uint8_t nbytes = beat_bytes(w_pending.size);
    uint8_t in_bytes[sim_ddr::AXI_DATA_BYTES] = {0};
    store_beat(in_bytes, io.w.wdata, nbytes);
    uint32_t local_wstrb = 0;
    for (uint8_t i = 0; i < nbytes && i < 32u; ++i) {
      if (axi_compat::test_bit(io.w.wstrb, i)) {
        local_wstrb |= (1u << i);
      }
    }

    if (dev != nullptr) {
      dev->write(addr, in_bytes, nbytes, local_wstrb);
    } else {
      w_pending.resp = sim_ddr::AXI_RESP_DECERR;
    }

    bool last_beat = (w_pending.beat_idx == w_pending.len);
    w_pending.beat_idx++;
    if (io.w.wlast || last_beat) {
      w_pending.active = false;
      w_resp.active = true;
      w_resp.id = w_pending.id;
      w_resp.resp = w_pending.resp;
      w_resp.latency_cnt = 0;
    }
  }

  if (w_resp.active && w_resp.latency_cnt < MMIO_LATENCY) {
    w_resp.latency_cnt++;
  }

  // Consume write response
  if (io.b.bvalid && io.b.bready && w_resp.active) {
    w_resp.active = false;
  }
}

} // namespace mmio
