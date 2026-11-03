/**
 * @file axi_interconnect_test.cpp
 * @brief Test harness for AXI_Interconnect layer
 */

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <queue>
#include <vector>

#define private public
#include "AXI_Interconnect.h"
#undef private
#include "SimDDR.h"

uint32_t *p_memory = nullptr;
long long sim_time = 0;
constexpr uint32_t TEST_MEM_SIZE_WORDS = 0x100000;

// ============================================================================
// Helpers
// ============================================================================

struct ArEvent {
  uint32_t addr;
  uint8_t id;
  uint8_t len;
};

struct AwEvent {
  uint32_t addr;
  uint8_t id;
  uint8_t len;
};

struct WEvent {
  uint32_t data;
  uint8_t strb;
  bool last;
};

struct TestEnv {
  axi_interconnect::AXI_Interconnect interconnect;
  sim_ddr::SimDDR ddr;
  std::vector<ArEvent> ar_events;
  std::vector<AwEvent> aw_events;
  std::vector<WEvent> w_events;

  void clear_events() {
    ar_events.clear();
    aw_events.clear();
    w_events.clear();
  }
};

uint8_t calc_burst_len(uint8_t total_size) {
  uint16_t bytes = static_cast<uint16_t>(total_size) + 1u;
  uint16_t beats = (bytes + 3u) / 4u;
  return beats > 0 ? static_cast<uint8_t>(beats - 1u) : 0;
}

uint8_t calc_total_beats(uint8_t total_size) { return calc_burst_len(total_size) + 1; }

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

void cycle_outputs(TestEnv &env) {
  clear_upstream_inputs(env.interconnect);

  // Phase 1: DDR outputs → interconnect inputs → interconnect outputs
  env.ddr.comb_outputs();
  env.interconnect.axi_io.ar.arready = env.ddr.io.ar.arready;
  env.interconnect.axi_io.r.rvalid = env.ddr.io.r.rvalid;
  env.interconnect.axi_io.r.rid = env.ddr.io.r.rid;
  env.interconnect.axi_io.r.rdata = env.ddr.io.r.rdata;
  env.interconnect.axi_io.r.rlast = env.ddr.io.r.rlast;
  env.interconnect.axi_io.r.rresp = env.ddr.io.r.rresp;
  env.interconnect.axi_io.aw.awready = env.ddr.io.aw.awready;
  env.interconnect.axi_io.w.wready = env.ddr.io.w.wready;
  env.interconnect.axi_io.b.bvalid = env.ddr.io.b.bvalid;
  env.interconnect.axi_io.b.bid = env.ddr.io.b.bid;
  env.interconnect.axi_io.b.bresp = env.ddr.io.b.bresp;

  env.interconnect.comb_outputs();
}

void cycle_inputs(TestEnv &env) {
  // Phase 2: interconnect inputs → DDR inputs
  env.interconnect.comb_inputs();

  // Record DDR-side handshakes for protocol/beat-splitting checks
  if (env.interconnect.axi_io.ar.arvalid && env.interconnect.axi_io.ar.arready) {
    env.ar_events.push_back(
        {.addr = env.interconnect.axi_io.ar.araddr,
         .id = static_cast<uint8_t>(env.interconnect.axi_io.ar.arid),
         .len = static_cast<uint8_t>(env.interconnect.axi_io.ar.arlen)});
  }
  if (env.interconnect.axi_io.aw.awvalid && env.interconnect.axi_io.aw.awready) {
    env.aw_events.push_back(
        {.addr = env.interconnect.axi_io.aw.awaddr,
         .id = static_cast<uint8_t>(env.interconnect.axi_io.aw.awid),
         .len = static_cast<uint8_t>(env.interconnect.axi_io.aw.awlen)});
  }
  if (env.interconnect.axi_io.w.wvalid && env.interconnect.axi_io.w.wready) {
    env.w_events.push_back(
        {.data = env.interconnect.axi_io.w.wdata,
         .strb = static_cast<uint8_t>(env.interconnect.axi_io.w.wstrb),
         .last = env.interconnect.axi_io.w.wlast});
  }

  env.ddr.io.ar.arvalid = env.interconnect.axi_io.ar.arvalid;
  env.ddr.io.ar.araddr = env.interconnect.axi_io.ar.araddr;
  env.ddr.io.ar.arid = env.interconnect.axi_io.ar.arid;
  env.ddr.io.ar.arlen = env.interconnect.axi_io.ar.arlen;
  env.ddr.io.ar.arsize = env.interconnect.axi_io.ar.arsize;
  env.ddr.io.ar.arburst = env.interconnect.axi_io.ar.arburst;

  env.ddr.io.aw.awvalid = env.interconnect.axi_io.aw.awvalid;
  env.ddr.io.aw.awaddr = env.interconnect.axi_io.aw.awaddr;
  env.ddr.io.aw.awid = env.interconnect.axi_io.aw.awid;
  env.ddr.io.aw.awlen = env.interconnect.axi_io.aw.awlen;
  env.ddr.io.aw.awsize = env.interconnect.axi_io.aw.awsize;
  env.ddr.io.aw.awburst = env.interconnect.axi_io.aw.awburst;

  env.ddr.io.w.wvalid = env.interconnect.axi_io.w.wvalid;
  env.ddr.io.w.wdata = env.interconnect.axi_io.w.wdata;
  env.ddr.io.w.wstrb = env.interconnect.axi_io.w.wstrb;
  env.ddr.io.w.wlast = env.interconnect.axi_io.w.wlast;

  env.ddr.io.r.rready = env.interconnect.axi_io.r.rready;
  env.ddr.io.b.bready = env.interconnect.axi_io.b.bready;

  env.ddr.comb_inputs();
  env.ddr.seq();
  env.interconnect.seq();
  sim_time++;
}

// ============================================================================
// Tests
// ============================================================================

bool test_ar_latching_ready_first() {
  printf("=== Test 1: AR latching (mocked arready) ===\n");

  axi_interconnect::AXI_Interconnect interconnect;
  interconnect.init();

  // Drive a request and force arready low; interconnect should latch AR and
  // keep arvalid asserted even if req.valid drops.
  bool req_valid = true;
  bool saw_ready = false;

  for (int cyc = 0; cyc < 8; cyc++) {
    clear_upstream_inputs(interconnect);

    // Mock AXI slave (no R channel activity)
    interconnect.axi_io.ar.arready = (cyc >= 5); // backpressure for first 5 cycles
    interconnect.axi_io.r.rvalid = false;
    interconnect.axi_io.r.rid = 0;
    interconnect.axi_io.r.rdata = 0;
    interconnect.axi_io.r.rlast = false;
    interconnect.axi_io.r.rresp = sim_ddr::AXI_RESP_OKAY;
    interconnect.axi_io.aw.awready = true;
    interconnect.axi_io.w.wready = true;
    interconnect.axi_io.b.bvalid = false;
    interconnect.axi_io.b.bid = 0;
    interconnect.axi_io.b.bresp = sim_ddr::AXI_RESP_OKAY;

    interconnect.comb_outputs();
    bool req_ready = interconnect.read_ports[0].req.ready;

    // Keep valid asserted until we see ready (ready-first handshake)
    if (req_valid) {
      interconnect.read_ports[0].req.valid = true;
      interconnect.read_ports[0].req.addr = 0x2000;
      interconnect.read_ports[0].req.total_size = 3;
      interconnect.read_ports[0].req.id = 1;
    }

    if (req_valid && req_ready) {
      saw_ready = true;
      req_valid = false; // drop after handshake
    }

    interconnect.comb_inputs();

    if (saw_ready && cyc >= 2 && cyc < 5 && !req_valid) {
      // After req.valid drops, arvalid must stay asserted while arready=0.
      if (!interconnect.axi_io.ar.arvalid) {
        printf("FAIL: arvalid dropped under backpressure\n");
        return false;
      }
    }

    interconnect.seq();
    sim_time++;
  }

  if (!saw_ready) {
    printf("FAIL: never observed req.ready high\n");
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_read_multi_master_var_sizes(TestEnv &env) {
  printf("=== Test 2: Multi-master reads (sizes/ids) ===\n");

  env.clear_events();
  env.interconnect.init();
  env.ddr.init();

  // master 0: 4B, master 1: 8B, master 2: 32B, master 3: 16B
  struct Req {
    uint8_t master;
    uint32_t addr;
    uint8_t total_size;
    uint8_t id;
  };
  Req reqs[axi_interconnect::NUM_READ_MASTERS] = {
      {.master = 0, .addr = 0x1000, .total_size = 3, .id = 0},
      {.master = 1, .addr = 0x2000, .total_size = 7, .id = 1},
      {.master = 2, .addr = 0x3000, .total_size = 31, .id = 2},
      {.master = 3, .addr = 0x3400, .total_size = 15, .id = 3},
  };

  for (const auto &r : reqs) {
    uint8_t beats = calc_total_beats(r.total_size);
    for (int b = 0; b < beats; b++) {
      p_memory[(r.addr >> 2) + b] = (r.master << 28) | (b << 16) | 0x1234;
    }
  }

  bool issued[axi_interconnect::NUM_READ_MASTERS] = {};
  auto mark_issued_from_ar_events = [&]() {
    for (const auto &e : env.ar_events) {
      for (const auto &r : reqs) {
        if (!issued[r.master] && e.addr == r.addr &&
            e.len == calc_burst_len(r.total_size)) {
          issued[r.master] = true;
          break;
        }
      }
    }
  };
  int issue_timeout = 200;
  while (issue_timeout-- > 0) {
    mark_issued_from_ar_events();
    bool all = true;
    for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; i++) {
      all &= issued[i];
    }
    if (all)
      break;

    cycle_outputs(env);
    for (const auto &r : reqs) {
      if (issued[r.master])
        continue;
      env.interconnect.read_ports[r.master].req.valid = true;
      env.interconnect.read_ports[r.master].req.addr = r.addr;
      env.interconnect.read_ports[r.master].req.total_size = r.total_size;
      env.interconnect.read_ports[r.master].req.id = r.id;
    }

    for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; i++) {
      env.interconnect.read_ports[i].resp.ready = false;
    }

    cycle_inputs(env);
    mark_issued_from_ar_events();
  }

  for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; i++) {
    if (!issued[i]) {
      printf("FAIL: master %d req not accepted\n", i);
      return false;
    }
  }

  if (env.ar_events.size() != axi_interconnect::NUM_READ_MASTERS) {
    printf("FAIL: expected %d AR handshakes, got %zu\n",
           axi_interconnect::NUM_READ_MASTERS, env.ar_events.size());
    return false;
  }

  // Validate AR fields. AXI read IDs are allocator-generated and not encoded
  // with master/orig ID, so match requests by address/len.
  bool ar_seen[axi_interconnect::NUM_READ_MASTERS] = {};
  for (const auto &e : env.ar_events) {
    bool found = false;
    uint8_t matched_master = 0;
    for (const auto &r : reqs) {
      uint8_t exp_len = calc_burst_len(r.total_size);
      if (e.addr == r.addr && e.len == exp_len) {
        found = true;
        matched_master = r.master;
        break;
      }
    }
    if (!found) {
      printf("FAIL: unexpected AR event addr=0x%x len=%u id=%u\n", e.addr,
             e.len, e.id);
      return false;
    }
    ar_seen[matched_master] = true;
  }
  for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; i++) {
    if (!ar_seen[i]) {
      printf("FAIL: missing AR event for master %d\n", i);
      return false;
    }
  }

  // Wait for all responses (may arrive out-of-order)
  bool done[axi_interconnect::NUM_READ_MASTERS] = {};
  int resp_timeout = sim_ddr::SIM_DDR_LATENCY * 40;
  while (resp_timeout-- > 0) {
    bool all = true;
    for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; i++) {
      all &= done[i];
    }
    if (all)
      break;

    cycle_outputs(env);

    for (const auto &r : reqs) {
      if (done[r.master])
        continue;
      if (!env.interconnect.read_ports[r.master].resp.valid)
        continue;

      uint8_t exp_beats = calc_total_beats(r.total_size);
      if (env.interconnect.read_ports[r.master].resp.id != r.id) {
        printf("FAIL: master %u resp.id mismatch exp=%u got=%u\n", r.master,
               r.id, env.interconnect.read_ports[r.master].resp.id);
        return false;
      }

      for (int b = 0; b < exp_beats; b++) {
        uint32_t exp = p_memory[(r.addr >> 2) + b];
        if (env.interconnect.read_ports[r.master].resp.data[b] != exp) {
          printf("FAIL: master %u beat %d exp=0x%08x got=0x%08x\n", r.master, b,
                 exp, env.interconnect.read_ports[r.master].resp.data[b]);
          return false;
        }
      }
      for (int b = exp_beats; b < axi_interconnect::MAX_READ_TRANSACTION_WORDS;
           b++) {
        if (env.interconnect.read_ports[r.master].resp.data[b] != 0) {
          printf("FAIL: master %u beat %d expected 0 padding\n", r.master, b);
          return false;
        }
      }

      env.interconnect.read_ports[r.master].resp.ready = true;
      done[r.master] = true;
    }

    cycle_inputs(env);
  }

  for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; i++) {
    if (!done[i]) {
      printf("FAIL: master %d resp timeout\n", i);
      return false;
    }
  }

  printf("PASS\n");
  return true;
}

bool test_write_burst_split_and_backpressure(TestEnv &env) {
  printf("=== Test 3: Write burst splitting + resp backpressure ===\n");

  env.clear_events();
  env.interconnect.init();
  env.ddr.init();

  // Case A: full cache-line write (64B = 16 beats on 32-bit AXI4 data bus)
  uint32_t base_addr = 0x8000;
  axi_interconnect::WideData256_t wdata;
  for (int i = 0; i < axi_interconnect::CACHELINE_WORDS; i++) {
    wdata[i] = 0xA0A00000u | static_cast<uint32_t>(i);
    p_memory[(base_addr >> 2) + i] = 0x0;
  }

  uint64_t wstrb = 0xFFFFFFFFFFFFFFFFull;
  uint8_t total_size = 63;
  uint8_t req_id = 2;

  bool issued = false;
  int issue_timeout = 200;
  while (!issued && issue_timeout-- > 0) {
    cycle_outputs(env);
    bool ready_snapshot = env.interconnect.write_port.req.ready;

    env.interconnect.write_port.req.valid = true;
    env.interconnect.write_port.req.addr = base_addr;
    env.interconnect.write_port.req.wdata = wdata;
    env.interconnect.write_port.req.wstrb = wstrb;
    env.interconnect.write_port.req.total_size = total_size;
    env.interconnect.write_port.req.id = req_id;

    cycle_inputs(env);
    if (ready_snapshot) {
      issued = true;
    }
  }
  if (!issued) {
    printf("FAIL: write req not accepted\n");
    return false;
  }

  // Wait for write resp.valid, but keep resp.ready low for a few cycles first.
  bool saw_resp = false;
  int resp_timeout = sim_ddr::SIM_DDR_LATENCY * 40;
  while (!saw_resp && resp_timeout-- > 0) {
    cycle_outputs(env);
    if (env.interconnect.write_port.resp.valid) {
      saw_resp = true;
      break;
    }
    cycle_inputs(env);
  }
  if (!saw_resp) {
    printf("FAIL: write resp timeout\n");
    return false;
  }

  // Backpressure upstream for 5 cycles; resp.valid must remain asserted. The
  // interconnect may keep downstream BREADY high when its response buffer has
  // space, but it must not drop or rewrite the visible upstream payload.
  for (int i = 0; i < 5; i++) {
    cycle_outputs(env);
    if (!env.interconnect.write_port.resp.valid) {
      printf("FAIL: resp.valid dropped under backpressure\n");
      return false;
    }
    if (env.interconnect.write_port.req.ready) {
      printf("FAIL: write req.ready should stay low while resp pending\n");
      return false;
    }
    cycle_inputs(env);
  }

  // Now accept the response.
  cycle_outputs(env);
  if (!env.interconnect.write_port.resp.valid) {
    printf("FAIL: resp.valid unexpectedly low before accept\n");
    return false;
  }
  if (env.interconnect.write_port.resp.id != req_id ||
      env.interconnect.write_port.resp.resp != sim_ddr::AXI_RESP_OKAY) {
    printf("FAIL: write resp mismatch id=%u resp=%u\n",
           env.interconnect.write_port.resp.id, env.interconnect.write_port.resp.resp);
    return false;
  }
  env.interconnect.write_port.resp.ready = true;
  cycle_inputs(env);

  // Next cycle: resp.valid should be cleared.
  cycle_outputs(env);
  if (env.interconnect.write_port.resp.valid) {
    printf("FAIL: resp.valid not cleared after handshake\n");
    return false;
  }
  cycle_inputs(env);

  // Verify memory contents
  for (int i = 0; i < axi_interconnect::CACHELINE_WORDS; i++) {
    uint32_t got = p_memory[(base_addr >> 2) + i];
    if (got != wdata[i]) {
      printf("FAIL: mem[%d] exp=0x%08x got=0x%08x\n", i, wdata[i], got);
      return false;
    }
  }

  // Validate AW/W splitting
  if (env.aw_events.size() != 1) {
    printf("FAIL: expected 1 AW handshake, got %zu\n", env.aw_events.size());
    return false;
  }
  const uint8_t exp_awid = 0;
  if (env.aw_events[0].addr != base_addr || env.aw_events[0].len != 15 ||
      env.aw_events[0].id != exp_awid) {
    printf("FAIL: AW mismatch addr=0x%x len=%u id=0x%x\n", env.aw_events[0].addr,
           env.aw_events[0].len, env.aw_events[0].id);
    return false;
  }
  if (env.w_events.size() != axi_interconnect::CACHELINE_WORDS) {
    printf("FAIL: expected %u W handshakes, got %zu\n",
           static_cast<unsigned>(axi_interconnect::CACHELINE_WORDS),
           env.w_events.size());
    return false;
  }
  for (int i = 0; i < axi_interconnect::CACHELINE_WORDS; i++) {
    if (env.w_events[i].data != wdata[i] || env.w_events[i].strb != 0xF ||
        env.w_events[i].last != (i == (axi_interconnect::CACHELINE_WORDS - 1))) {
      printf("FAIL: W[%d] mismatch data=0x%08x strb=0x%x last=%d\n", i,
             env.w_events[i].data, env.w_events[i].strb, env.w_events[i].last);
      return false;
    }
  }

  // Case B: partial strobe single-beat write
  env.clear_events();
  env.interconnect.init();
  env.ddr.init();

  uint32_t paddr = 0x9000;
  p_memory[paddr >> 2] = 0x11223344;
  axi_interconnect::WideData256_t pwdata;
  pwdata.clear();
  pwdata[0] = 0xAABBCCDD;

  issued = false;
  issue_timeout = 200;
  while (!issued && issue_timeout-- > 0) {
    cycle_outputs(env);
    bool ready_snapshot = env.interconnect.write_port.req.ready;

    env.interconnect.write_port.req.valid = true;
    env.interconnect.write_port.req.addr = paddr;
    env.interconnect.write_port.req.wdata = pwdata;
    env.interconnect.write_port.req.wstrb = 0x5; // byte0 + byte2
    env.interconnect.write_port.req.total_size = 3;
    env.interconnect.write_port.req.id = 1;
    env.interconnect.write_port.resp.ready = true;

    cycle_inputs(env);
    if (ready_snapshot) {
      issued = true;
    }
  }
  if (!issued) {
    printf("FAIL: partial write req not accepted\n");
    return false;
  }

  // Wait for resp to be consumed
  resp_timeout = sim_ddr::SIM_DDR_LATENCY * 40;
  bool consumed = false;
  while (resp_timeout-- > 0) {
    cycle_outputs(env);
    env.interconnect.write_port.resp.ready = true;
    bool done = env.interconnect.write_port.resp.valid;
    cycle_inputs(env);
    if (done) {
      consumed = true;
      break;
    }
  }
  if (!consumed) {
    printf("FAIL: partial write resp timeout\n");
    return false;
  }
  uint32_t expected = 0x11BB33DD;
  if (p_memory[paddr >> 2] != expected) {
    printf("FAIL: partial strobe exp=0x%08x got=0x%08x\n", expected,
           p_memory[paddr >> 2]);
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_read_write_parallel(TestEnv &env) {
  printf("=== Test 4: Read/write parallel ===\n");

  env.clear_events();
  env.interconnect.init();
  env.ddr.init();

  uint32_t r_addr = 0x10000;
  uint32_t w_addr = 0x11000;
  p_memory[r_addr >> 2] = 0xDEADBEEF;
  p_memory[w_addr >> 2] = 0;

  axi_interconnect::WideData256_t wdata;
  wdata.clear();
  wdata[0] = 0xCAFEBABE;

  bool r_issued = false;
  bool w_issued = false;
  int issue_timeout = 300;
  while (issue_timeout-- > 0) {
    if (r_issued && w_issued)
      break;

    cycle_outputs(env);
    bool r_ready_snapshot = env.interconnect.read_ports[0].req.ready;
    bool w_ready_snapshot = env.interconnect.write_port.req.ready;

    if (!r_issued) {
      env.interconnect.read_ports[0].req.valid = true;
      env.interconnect.read_ports[0].req.addr = r_addr;
      env.interconnect.read_ports[0].req.total_size = 3;
      env.interconnect.read_ports[0].req.id = 3;
    }
    if (!w_issued) {
      env.interconnect.write_port.req.valid = true;
      env.interconnect.write_port.req.addr = w_addr;
      env.interconnect.write_port.req.wdata = wdata;
      env.interconnect.write_port.req.wstrb = 0xF;
      env.interconnect.write_port.req.total_size = 3;
      env.interconnect.write_port.req.id = 0;
    }

    env.interconnect.read_ports[0].resp.ready = true;
    env.interconnect.write_port.resp.ready = true;
    cycle_inputs(env);

    if (!r_issued && r_ready_snapshot)
      r_issued = true;
    if (!w_issued && w_ready_snapshot)
      w_issued = true;
  }
  if (!r_issued || !w_issued) {
    printf("FAIL: parallel issue timeout r=%d w=%d\n", r_issued, w_issued);
    return false;
  }

  bool r_done = false;
  bool w_done = false;
  int timeout = sim_ddr::SIM_DDR_LATENCY * 60;
  while (timeout-- > 0) {
    if (r_done && w_done)
      break;

    cycle_outputs(env);
    if (!r_done && env.interconnect.read_ports[0].resp.valid) {
      if (env.interconnect.read_ports[0].resp.data[0] != p_memory[r_addr >> 2]) {
        printf("FAIL: read data mismatch\n");
        return false;
      }
      env.interconnect.read_ports[0].resp.ready = true;
      r_done = true;
    }
    if (!w_done && env.interconnect.write_port.resp.valid) {
      env.interconnect.write_port.resp.ready = true;
      w_done = true;
    }
    cycle_inputs(env);
  }
  if (!r_done || !w_done) {
    printf("FAIL: parallel completion timeout r=%d w=%d\n", r_done, w_done);
    return false;
  }

  if (p_memory[w_addr >> 2] != wdata[0]) {
    printf("FAIL: write memory mismatch exp=0x%08x got=0x%08x\n", wdata[0],
           p_memory[w_addr >> 2]);
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_read_resp_backpressure(TestEnv &env) {
  printf("=== Test 5: Read resp backpressure ===\n");

  env.clear_events();
  env.interconnect.init();
  env.ddr.init();

  uint8_t master = axi_interconnect::MASTER_ICACHE;
  uint32_t addr = 0x12000;
  uint8_t total_size = 31;
  uint8_t id = 3;

  uint8_t beats = calc_total_beats(total_size);
  for (int b = 0; b < beats; b++) {
    p_memory[(addr >> 2) + b] = 0xFACE0000u | static_cast<uint32_t>(b);
  }

  // Issue request (ready-first)
  bool issued = false;
  int issue_timeout = 200;
  while (!issued && issue_timeout-- > 0) {
    cycle_outputs(env);
    bool ready_snapshot = env.interconnect.read_ports[master].req.ready;

    env.interconnect.read_ports[master].req.valid = true;
    env.interconnect.read_ports[master].req.addr = addr;
    env.interconnect.read_ports[master].req.total_size = total_size;
    env.interconnect.read_ports[master].req.id = id;

    cycle_inputs(env);
    if (ready_snapshot) {
      issued = true;
    }
  }
  if (!issued) {
    printf("FAIL: read req not accepted\n");
    return false;
  }

  // Wait for response
  axi_interconnect::WideReadData_t resp_data_snapshot;
  resp_data_snapshot.clear();
  bool saw_resp = false;
  int timeout = sim_ddr::SIM_DDR_LATENCY * 40;
  while (!saw_resp && timeout-- > 0) {
    cycle_outputs(env);
    if (env.interconnect.read_ports[master].resp.valid) {
      if (env.interconnect.read_ports[master].resp.id != id) {
        printf("FAIL: resp.id mismatch exp=%u got=%u\n", id,
               env.interconnect.read_ports[master].resp.id);
        return false;
      }
      resp_data_snapshot = env.interconnect.read_ports[master].resp.data;
      saw_resp = true;
    }
    cycle_inputs(env);
  }
  if (!saw_resp) {
    printf("FAIL: read resp timeout\n");
    return false;
  }

  // Backpressure for 5 cycles: resp.valid/data must remain stable
  for (int i = 0; i < 5; i++) {
    cycle_outputs(env);
    if (!env.interconnect.read_ports[master].resp.valid) {
      printf("FAIL: resp.valid dropped under backpressure\n");
      return false;
    }
    if (env.interconnect.read_ports[master].resp.id != id) {
      printf("FAIL: resp.id changed under backpressure\n");
      return false;
    }
    for (int b = 0; b < axi_interconnect::MAX_READ_TRANSACTION_WORDS; b++) {
      if (env.interconnect.read_ports[master].resp.data[b] != resp_data_snapshot[b]) {
        printf("FAIL: resp.data changed under backpressure\n");
        return false;
      }
    }
    cycle_inputs(env);
  }

  // Consume the response
  cycle_outputs(env);
  if (!env.interconnect.read_ports[master].resp.valid) {
    printf("FAIL: resp.valid unexpectedly low before consume\n");
    return false;
  }
  env.interconnect.read_ports[master].resp.ready = true;
  cycle_inputs(env);

  cycle_outputs(env);
  if (env.interconnect.read_ports[master].resp.valid) {
    printf("FAIL: resp.valid not cleared after handshake\n");
    return false;
  }
  cycle_inputs(env);

  printf("PASS\n");
  return true;
}

bool test_large_read_transactions(TestEnv &env) {
  printf("=== Test 6: Large read transactions (64B/256B) ===\n");

  struct Req {
    uint32_t addr;
    uint8_t total_size;
    uint8_t id;
    uint8_t exp_len;
  };

  const Req reqs[] = {
      {.addr = 0x14000, .total_size = 63, .id = 1, .exp_len = 15},
      {.addr = 0x18000, .total_size = 255, .id = 2, .exp_len = 63},
  };

  for (const auto &r : reqs) {
    env.clear_events();
    env.interconnect.init();
    env.ddr.init();

    uint8_t beats = calc_total_beats(r.total_size);
    for (int b = 0; b < beats; ++b) {
      p_memory[(r.addr >> 2) + b] = 0xAC000000u | (r.total_size << 8) | b;
    }

    bool issued = false;
    int issue_timeout = 200;
    while (!issued && issue_timeout-- > 0) {
      cycle_outputs(env);
      bool ready_snapshot = env.interconnect.read_ports[0].req.ready;

      env.interconnect.read_ports[0].req.valid = true;
      env.interconnect.read_ports[0].req.addr = r.addr;
      env.interconnect.read_ports[0].req.total_size = r.total_size;
      env.interconnect.read_ports[0].req.id = r.id;
      env.interconnect.read_ports[0].resp.ready = true;

      cycle_inputs(env);
      if (ready_snapshot) {
        issued = true;
      }
    }
    if (!issued) {
      printf("FAIL: large read req not accepted size=%u\n", r.total_size);
      return false;
    }

    if (env.ar_events.size() != 1) {
      printf("FAIL: expected 1 AR handshake, got %zu for size=%u\n",
             env.ar_events.size(), r.total_size);
      return false;
    }
    if (env.ar_events[0].addr != r.addr || env.ar_events[0].len != r.exp_len) {
      printf("FAIL: AR mismatch addr=0x%x len=%u for size=%u\n",
             env.ar_events[0].addr, env.ar_events[0].len, r.total_size);
      return false;
    }

    bool done = false;
    int resp_timeout = sim_ddr::SIM_DDR_LATENCY * 120;
    while (!done && resp_timeout-- > 0) {
      cycle_outputs(env);
      auto &resp = env.interconnect.read_ports[0].resp;
      if (resp.valid) {
        if (resp.id != r.id) {
          printf("FAIL: resp.id mismatch exp=%u got=%u\n", r.id, resp.id);
          return false;
        }
        for (int b = 0; b < beats; ++b) {
          uint32_t exp = p_memory[(r.addr >> 2) + b];
          if (resp.data[b] != exp) {
            printf("FAIL: size=%u beat=%d exp=0x%08x got=0x%08x\n", r.total_size,
                   b, exp, resp.data[b]);
            return false;
          }
        }
        resp.ready = true;
        done = true;
      }
      cycle_inputs(env);
    }
    if (!done) {
      printf("FAIL: large read resp timeout size=%u\n", r.total_size);
      return false;
    }
  }

  printf("PASS\n");
  return true;
}

bool test_same_master_multi_outstanding(TestEnv &env) {
  printf("=== Test 7: Same-master multi-outstanding reads ===\n");

  env.clear_events();
  env.interconnect.init();
  env.ddr.init();

  struct Req {
    uint32_t addr;
    uint8_t total_size;
    uint8_t id;
  };
  const Req reqs[2] = {
      {.addr = 0x1C000, .total_size = 3, .id = 1},
      {.addr = 0x1C040, .total_size = 3, .id = 2},
  };
  constexpr uint8_t master = axi_interconnect::MASTER_ICACHE;

  for (int i = 0; i < 2; ++i) {
    p_memory[reqs[i].addr >> 2] = 0xA5000000u | (i << 8) | reqs[i].id;
  }

  bool issued[2] = {false, false};
  bool second_issued_before_first_consumed = false;
  bool first_resp_seen = false;
  int issue_timeout = sim_ddr::SIM_DDR_LATENCY * 40;
  while ((!issued[0] || !issued[1]) && issue_timeout-- > 0) {
    cycle_outputs(env);
    bool ready_snapshot = env.interconnect.read_ports[master].req.ready;
    if (env.interconnect.read_ports[master].resp.valid) {
      first_resp_seen = true;
    }

    int idx = issued[0] ? 1 : 0;
    env.interconnect.read_ports[master].req.valid = true;
    env.interconnect.read_ports[master].req.addr = reqs[idx].addr;
    env.interconnect.read_ports[master].req.total_size = reqs[idx].total_size;
    env.interconnect.read_ports[master].req.id = reqs[idx].id;

    cycle_inputs(env);

    if (ready_snapshot) {
      issued[idx] = true;
      if (idx == 1 && !first_resp_seen) {
        second_issued_before_first_consumed = true;
      }
    }
  }

  if (!issued[0] || !issued[1]) {
    printf("FAIL: same-master issue timeout first=%d second=%d\n", issued[0],
           issued[1]);
    return false;
  }
  if (!second_issued_before_first_consumed) {
    printf("FAIL: second request was not accepted before first response path\n");
    return false;
  }
  if (env.ar_events.size() < 2) {
    printf("FAIL: expected 2 AR handshakes, got %zu\n", env.ar_events.size());
    return false;
  }

  bool resp_done[2] = {false, false};
  int resp_timeout = sim_ddr::SIM_DDR_LATENCY * 80;
  while ((!resp_done[0] || !resp_done[1]) && resp_timeout-- > 0) {
    cycle_outputs(env);
    auto &resp = env.interconnect.read_ports[master].resp;
    if (resp.valid) {
      int idx = -1;
      if (resp.id == reqs[0].id) {
        idx = 0;
      } else if (resp.id == reqs[1].id) {
        idx = 1;
      } else {
        printf("FAIL: unexpected resp.id=%u\n", resp.id);
        return false;
      }
      uint32_t expected = p_memory[reqs[idx].addr >> 2];
      if (resp.data[0] != expected) {
        printf("FAIL: resp data mismatch id=%u exp=0x%08x got=0x%08x\n",
               resp.id, expected, resp.data[0]);
        return false;
      }
      resp.ready = true;
      resp_done[idx] = true;
    }
    cycle_inputs(env);
  }

  if (!resp_done[0] || !resp_done[1]) {
    printf("FAIL: same-master resp timeout first=%d second=%d\n", resp_done[0],
           resp_done[1]);
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_llc_mem_issue_priority() {
  printf("=== Test 8: LLC downstream AR wins over new upstream req ===\n");

  axi_interconnect::AXI_Interconnect interconnect;
  axi_interconnect::AXI_LLCConfig cfg;
  cfg.enable = true;
  cfg.size_bytes = 512;
  cfg.line_bytes = 64;
  cfg.ways = 2;
  cfg.mshr_num = 2;
  interconnect.set_llc_config(cfg);
  interconnect.init();

  clear_upstream_inputs(interconnect);
  interconnect.llc.io.ext_out.mem.read_req_valid = true;
  interconnect.llc.io.ext_out.mem.read_req_addr = 0x4000;
  interconnect.llc.io.ext_out.mem.read_req_size = 63;
  interconnect.llc.io.ext_out.mem.read_req_id = 2;

  interconnect.read_ports[axi_interconnect::MASTER_ICACHE].req.valid = true;
  interconnect.read_ports[axi_interconnect::MASTER_ICACHE].req.addr = 0x80005380;
  interconnect.read_ports[axi_interconnect::MASTER_ICACHE].req.total_size = 63;
  interconnect.read_ports[axi_interconnect::MASTER_ICACHE].req.id = 1;

  interconnect.comb_inputs();
  if (!interconnect.axi_io.ar.arvalid ||
      interconnect.axi_io.ar.araddr != 0x4000) {
    printf("FAIL: LLC downstream AR was not prioritized addr=0x%08x\n",
           interconnect.axi_io.ar.araddr);
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_llc_latched_ar_not_ready_upstream() {
  printf("=== Test 9: Latched LLC AR must not ack upstream ===\n");

  axi_interconnect::AXI_Interconnect interconnect;
  axi_interconnect::AXI_LLCConfig cfg;
  cfg.enable = true;
  cfg.size_bytes = 512;
  cfg.line_bytes = 64;
  cfg.ways = 2;
  cfg.mshr_num = 2;
  interconnect.set_llc_config(cfg);
  interconnect.init();

  clear_upstream_inputs(interconnect);
  interconnect.ar_latched.valid = true;
  interconnect.ar_latched.to_llc = true;
  interconnect.ar_latched.master_id = axi_interconnect::MASTER_ICACHE;
  interconnect.ar_latched.addr = 0x80005380;
  interconnect.req_ready_r[axi_interconnect::MASTER_ICACHE] = false;

  interconnect.comb_outputs();
  if (interconnect.read_ports[axi_interconnect::MASTER_ICACHE].req.ready) {
    printf("FAIL: latched LLC AR leaked ready to upstream\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_llc_upstream_slot_blocks_ready() {
  printf("=== Test 10: Occupied LLC upstream slot blocks new ready ===\n");

  axi_interconnect::AXI_Interconnect interconnect;
  axi_interconnect::AXI_LLCConfig cfg;
  cfg.enable = true;
  cfg.size_bytes = 512;
  cfg.line_bytes = 64;
  cfg.ways = 2;
  cfg.mshr_num = 2;
  interconnect.set_llc_config(cfg);
  interconnect.init();

  clear_upstream_inputs(interconnect);
  interconnect.req_ready_r[axi_interconnect::MASTER_ICACHE] = true;
  interconnect.llc_upstream_req[axi_interconnect::MASTER_ICACHE].valid = true;
  interconnect.llc_upstream_req[axi_interconnect::MASTER_ICACHE].addr = 0x80001000;
  interconnect.llc_upstream_req[axi_interconnect::MASTER_ICACHE].total_size = 63;
  interconnect.llc_upstream_req[axi_interconnect::MASTER_ICACHE].id = 2;

  interconnect.comb_outputs();
  if (interconnect.read_ports[axi_interconnect::MASTER_ICACHE].req.ready) {
    printf("FAIL: occupied LLC upstream slot leaked ready\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_llc_write_resp_holds_until_ready() {
  printf("=== Test 11: LLC write resp holds payload until ready ===\n");

  axi_interconnect::AXI_Interconnect interconnect;
  axi_interconnect::AXI_LLCConfig cfg;
  cfg.enable = true;
  cfg.size_bytes = 512;
  cfg.line_bytes = 64;
  cfg.ways = 2;
  cfg.mshr_num = 2;
  interconnect.set_llc_config(cfg);
  interconnect.init();

  interconnect.llc.io.regs.write_resp_valid_r[axi_interconnect::MASTER_DCACHE_W] = true;
  interconnect.llc.io.regs.write_resp_id_r[axi_interconnect::MASTER_DCACHE_W] = 5;
  interconnect.llc.io.regs.write_resp_code_r[axi_interconnect::MASTER_DCACHE_W] =
      sim_ddr::AXI_RESP_OKAY;

  bool saw_resp = false;
  for (int cyc = 0; cyc < 4; ++cyc) {
    clear_upstream_inputs(interconnect);
    interconnect.comb_outputs();
    auto &resp = interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W].resp;
    if (resp.valid) {
      saw_resp = true;
      break;
    }
    interconnect.comb_inputs();
    interconnect.seq();
  }
  if (!saw_resp) {
    printf("FAIL: LLC write resp did not become visible through bridge\n");
    return false;
  }

  for (int cyc = 0; cyc < 4; ++cyc) {
    clear_upstream_inputs(interconnect);
    interconnect.comb_outputs();
    auto &resp = interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W].resp;
    if (!resp.valid || resp.id != 5 || resp.resp != sim_ddr::AXI_RESP_OKAY) {
      printf("FAIL: LLC write resp changed under backpressure cyc=%d valid=%d id=%u resp=%u\n",
             cyc, static_cast<int>(resp.valid), resp.id, resp.resp);
      return false;
    }
    interconnect.comb_inputs();
    interconnect.seq();
  }

  clear_upstream_inputs(interconnect);
  interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W].resp.ready = true;
  interconnect.comb_outputs();
  if (!interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W].resp.valid) {
    printf("FAIL: LLC write resp missing before handshake\n");
    return false;
  }
  interconnect.seq();

  clear_upstream_inputs(interconnect);
  interconnect.comb_outputs();
  if (interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W].resp.valid) {
    printf("FAIL: LLC write resp not cleared after handshake\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_multi_write_outstanding(TestEnv &env) {
  printf("=== Test 12: multiple write outstanding contexts ===\n");

  env.clear_events();
  env.interconnect.init();
  env.ddr.init();

  const uint32_t addr0 = 0x1A000;
  const uint32_t addr1 = 0x1A040;
  axi_interconnect::WideData256_t data0;
  axi_interconnect::WideData256_t data1;
  data0.clear();
  data1.clear();
  data0[0] = 0xABCDEF01;
  data1[0] = 0x12345678;

  bool first_issued = false;
  int timeout = 200;
  while (!first_issued && timeout-- > 0) {
    cycle_outputs(env);
    bool ready_snapshot =
        env.interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W].req.ready;

    env.interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W].req.valid = true;
    env.interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W].req.addr = addr0;
    env.interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W].req.wdata = data0;
    env.interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W].req.wstrb = 0xF;
    env.interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W].req.total_size = 3;
    env.interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W].req.id = 1;

    cycle_inputs(env);
    if (ready_snapshot) {
      first_issued = true;
    }
  }
  if (!first_issued) {
    printf("FAIL: first write was not accepted\n");
    return false;
  }

  bool second_issued = false;
  timeout = 200;
  while (!second_issued && timeout-- > 0) {
    cycle_outputs(env);
    if (env.interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W].resp.valid) {
      printf("FAIL: first response became visible before second issue\n");
      return false;
    }
    bool ready_snapshot =
        env.interconnect.write_ports[axi_interconnect::MASTER_UNCORE_LSU_W].req.ready;

    env.interconnect.write_ports[axi_interconnect::MASTER_UNCORE_LSU_W].req.valid = true;
    env.interconnect.write_ports[axi_interconnect::MASTER_UNCORE_LSU_W].req.addr = addr1;
    env.interconnect.write_ports[axi_interconnect::MASTER_UNCORE_LSU_W].req.wdata = data1;
    env.interconnect.write_ports[axi_interconnect::MASTER_UNCORE_LSU_W].req.wstrb = 0xF;
    env.interconnect.write_ports[axi_interconnect::MASTER_UNCORE_LSU_W].req.total_size = 3;
    env.interconnect.write_ports[axi_interconnect::MASTER_UNCORE_LSU_W].req.id = 2;

    cycle_inputs(env);
    if (ready_snapshot) {
      second_issued = true;
    }
  }
  if (!second_issued) {
    printf("FAIL: second write was not accepted before first response\n");
    return false;
  }

  bool first_done = false;
  bool second_done = false;
  timeout = sim_ddr::SIM_DDR_LATENCY * 40;
  while ((!first_done || !second_done) && timeout-- > 0) {
    cycle_outputs(env);
    auto &resp0 = env.interconnect.write_ports[axi_interconnect::MASTER_DCACHE_W].resp;
    if (!first_done && resp0.valid) {
      if (resp0.id != 1 || resp0.resp != sim_ddr::AXI_RESP_OKAY) {
        printf("FAIL: first write resp mismatch id=%u resp=%u\n",
               resp0.id, resp0.resp);
        return false;
      }
      resp0.ready = true;
      first_done = true;
    }
    auto &resp1 =
        env.interconnect.write_ports[axi_interconnect::MASTER_UNCORE_LSU_W].resp;
    if (!second_done && resp1.valid) {
      if (resp1.id != 2 || resp1.resp != sim_ddr::AXI_RESP_OKAY) {
        printf("FAIL: second write resp mismatch id=%u resp=%u\n", resp1.id,
               resp1.resp);
        return false;
      }
      resp1.ready = true;
      second_done = true;
    }
    cycle_inputs(env);
  }
  if (!first_done || !second_done) {
    printf("FAIL: multi-write response timeout first=%d second=%d\n",
           static_cast<int>(first_done), static_cast<int>(second_done));
    return false;
  }

  if (env.aw_events.size() < 2) {
    printf("FAIL: expected at least two AW handshakes, got %zu\n",
           env.aw_events.size());
    return false;
  }

  if (p_memory[addr0 >> 2] != data0[0] || p_memory[addr1 >> 2] != data1[0]) {
    printf("FAIL: multi-write memory mismatch got0=0x%08x got1=0x%08x\n",
           p_memory[addr0 >> 2], p_memory[addr1 >> 2]);
    return false;
  }

  printf("PASS\n");
  return true;
}

int main() {
  printf("====================================\n");
  printf("AXI-Interconnect Test Suite\n");
  printf("====================================\n\n");

  p_memory = new uint32_t[TEST_MEM_SIZE_WORDS];
  memset(p_memory, 0, TEST_MEM_SIZE_WORDS * sizeof(uint32_t));

  int passed = 0, failed = 0;

  if (test_ar_latching_ready_first())
    passed++;
  else
    failed++;

  TestEnv env;

  if (test_read_multi_master_var_sizes(env))
    passed++;
  else
    failed++;

  if (test_write_burst_split_and_backpressure(env))
    passed++;
  else
    failed++;

  if (test_read_write_parallel(env))
    passed++;
  else
    failed++;

  if (test_read_resp_backpressure(env))
    passed++;
  else
    failed++;

  if (test_large_read_transactions(env))
    passed++;
  else
    failed++;

  if (test_same_master_multi_outstanding(env))
    passed++;
  else
    failed++;

  if (test_llc_mem_issue_priority())
    passed++;
  else
    failed++;

  if (test_llc_latched_ar_not_ready_upstream())
    passed++;
  else
    failed++;

  if (test_llc_upstream_slot_blocks_ready())
    passed++;
  else
    failed++;

  if (test_llc_write_resp_holds_until_ready())
    passed++;
  else
    failed++;

  if (test_multi_write_outstanding(env))
    passed++;
  else
    failed++;

  printf("\n====================================\n");
  printf("Results: %d passed, %d failed\n", passed, failed);
  printf("====================================\n");

  delete[] p_memory;
  return failed == 0 ? 0 : 1;
}
