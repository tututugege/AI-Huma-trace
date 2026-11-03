#include "SimDDR.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

uint32_t *p_memory = nullptr;
long long sim_time = 0;

namespace {

#ifndef SIM_DDR_WRITE_DRAIN_TEST_QUEUE_DEPTH
#error "SIM_DDR_WRITE_DRAIN_TEST_QUEUE_DEPTH must be defined"
#endif

#ifndef SIM_DDR_WRITE_DRAIN_TEST_ACCEPT_GAP
#error "SIM_DDR_WRITE_DRAIN_TEST_ACCEPT_GAP must be defined"
#endif

#ifndef SIM_DDR_WRITE_DRAIN_TEST_DATA_FIFO_DEPTH
#error "SIM_DDR_WRITE_DRAIN_TEST_DATA_FIFO_DEPTH must be defined"
#endif

#ifndef SIM_DDR_WRITE_DRAIN_TEST_DRAIN_GAP
#error "SIM_DDR_WRITE_DRAIN_TEST_DRAIN_GAP must be defined"
#endif

#ifndef SIM_DDR_WRITE_DRAIN_TEST_HIGH_WM
#error "SIM_DDR_WRITE_DRAIN_TEST_HIGH_WM must be defined"
#endif

#ifndef SIM_DDR_WRITE_DRAIN_TEST_LOW_WM
#error "SIM_DDR_WRITE_DRAIN_TEST_LOW_WM must be defined"
#endif

static_assert(sim_ddr::SIM_DDR_WRITE_QUEUE_DEPTH ==
                  SIM_DDR_WRITE_DRAIN_TEST_QUEUE_DEPTH,
              "test binary must use the intended write queue depth override");
static_assert(sim_ddr::SIM_DDR_WRITE_ACCEPT_GAP ==
                  SIM_DDR_WRITE_DRAIN_TEST_ACCEPT_GAP,
              "test binary must use the intended write accept gap override");
static_assert(sim_ddr::SIM_DDR_WRITE_DATA_FIFO_DEPTH ==
                  SIM_DDR_WRITE_DRAIN_TEST_DATA_FIFO_DEPTH,
              "test binary must use the intended write data fifo depth override");
static_assert(sim_ddr::SIM_DDR_WRITE_DRAIN_GAP ==
                  SIM_DDR_WRITE_DRAIN_TEST_DRAIN_GAP,
              "test binary must use the intended write drain gap override");
static_assert(sim_ddr::SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK ==
                  SIM_DDR_WRITE_DRAIN_TEST_HIGH_WM,
              "test binary must use the intended write drain high watermark override");
static_assert(sim_ddr::SIM_DDR_WRITE_DRAIN_LOW_WATERMARK ==
                  SIM_DDR_WRITE_DRAIN_TEST_LOW_WM,
              "test binary must use the intended write drain low watermark override");

constexpr uint32_t kTestMemWords = 0x100000;
constexpr int kHandshakeTimeout = 80;
constexpr int kResponseTimeout =
    static_cast<int>(sim_ddr::SIM_DDR_WRITE_RESP_LATENCY) + 80;

void advance_seq(sim_ddr::SimDDR &ddr) {
  ddr.seq();
  ++sim_time;
}

void sim_cycle(sim_ddr::SimDDR &ddr) {
  ddr.comb();
  advance_seq(ddr);
}

void clear_master_signals(sim_ddr::SimDDR &ddr) {
  ddr.io.aw.awvalid = false;
  ddr.io.aw.awid = 0;
  ddr.io.aw.awaddr = 0;
  ddr.io.aw.awlen = 0;
  ddr.io.aw.awsize = sim_ddr::AXI_SIZE_CODE;
  ddr.io.aw.awburst = sim_ddr::AXI_BURST_INCR;

  ddr.io.w.wvalid = false;
  ddr.io.w.wdata = 0;
  ddr.io.w.wstrb = 0;
  ddr.io.w.wlast = false;

  ddr.io.b.bready = true;

  ddr.io.ar.arvalid = false;
  ddr.io.ar.arid = 0;
  ddr.io.ar.araddr = 0;
  ddr.io.ar.arlen = 0;
  ddr.io.ar.arsize = sim_ddr::AXI_SIZE_CODE;
  ddr.io.ar.arburst = sim_ddr::AXI_BURST_INCR;

  ddr.io.r.rready = true;
}

sim_ddr::axi_strb_t make_full_strobe() {
  sim_ddr::axi_strb_t strobe{};
  strobe = 0;
  for (uint8_t byte = 0; byte < sim_ddr::AXI_DATA_BYTES; ++byte) {
    axi_compat::set_bit(strobe, byte, true);
  }
  return strobe;
}

sim_ddr::axi_data_t make_pattern(uint32_t seed) {
  sim_ddr::axi_data_t data{};
  data = 0;
  for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
    axi_compat::set_u32(data, word, seed + word);
  }
  return data;
}

bool wait_aw_ready(sim_ddr::SimDDR &ddr) {
  int timeout = kHandshakeTimeout;
  while (timeout-- > 0) {
    ddr.comb();
    if (ddr.io.aw.awready) {
      return true;
    }
    advance_seq(ddr);
  }
  ddr.comb();
  return ddr.io.aw.awready;
}

bool wait_w_ready(sim_ddr::SimDDR &ddr) {
  int timeout = kHandshakeTimeout;
  while (timeout-- > 0) {
    ddr.comb();
    if (ddr.io.w.wready) {
      return true;
    }
    advance_seq(ddr);
  }
  ddr.comb();
  return ddr.io.w.wready;
}

int wait_b_visible(sim_ddr::SimDDR &ddr) {
  int cycles = 0;
  while (!ddr.io.b.bvalid && cycles < kResponseTimeout) {
    sim_cycle(ddr);
    ++cycles;
  }
  return ddr.io.b.bvalid ? cycles : -1;
}

bool issue_aw(sim_ddr::SimDDR &ddr, uint8_t id, uint32_t addr, uint8_t len = 0) {
  ddr.io.aw.awvalid = true;
  ddr.io.aw.awid = id;
  ddr.io.aw.awaddr = addr;
  ddr.io.aw.awlen = len;
  ddr.io.aw.awsize = sim_ddr::AXI_SIZE_CODE;
  ddr.io.aw.awburst = sim_ddr::AXI_BURST_INCR;
  if (!wait_aw_ready(ddr)) {
    std::printf("FAIL: AW handshake timeout for id=%u\n",
                static_cast<unsigned>(id));
    return false;
  }
  advance_seq(ddr);
  ddr.io.aw.awvalid = false;
  return true;
}

bool issue_w_beat_expect_immediate_ready(sim_ddr::SimDDR &ddr,
                                         sim_ddr::axi_data_t data,
                                         sim_ddr::axi_strb_t strobe, bool last) {
  ddr.io.w.wvalid = true;
  ddr.io.w.wdata = data;
  ddr.io.w.wstrb = strobe;
  ddr.io.w.wlast = last;
  ddr.comb();
  if (!ddr.io.w.wready) {
    std::printf(
        "FAIL: WREADY unexpectedly low for immediate beat at sim_time=%lld\n",
        sim_time);
    ddr.print_state();
    ddr.io.w.wvalid = false;
    ddr.io.w.wlast = false;
    return false;
  }
  advance_seq(ddr);
  ddr.io.w.wvalid = false;
  ddr.io.w.wlast = false;
  return true;
}

bool issue_w_beat(sim_ddr::SimDDR &ddr, sim_ddr::axi_data_t data,
                  sim_ddr::axi_strb_t strobe, bool last) {
  ddr.io.w.wvalid = true;
  ddr.io.w.wdata = data;
  ddr.io.w.wstrb = strobe;
  ddr.io.w.wlast = last;
  if (!wait_w_ready(ddr)) {
    std::printf("FAIL: W handshake timeout\n");
    ddr.io.w.wvalid = false;
    ddr.io.w.wlast = false;
    return false;
  }
  advance_seq(ddr);
  ddr.io.w.wvalid = false;
  ddr.io.w.wlast = false;
  return true;
}

bool expect_memory_pattern(uint32_t addr, uint32_t seed) {
  for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
    const uint32_t got = p_memory[(addr >> 2) + word];
    const uint32_t expected = seed + word;
    if (got != expected) {
      std::printf("FAIL: memory[%u] at 0x%08x exp=0x%08x got=0x%08x\n", word,
                  addr, expected, got);
      return false;
    }
  }
  return true;
}

bool expect_memory_zero(uint32_t addr) {
  for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
    if (p_memory[(addr >> 2) + word] != 0) {
      std::printf("FAIL: memory at 0x%08x changed before expected\n", addr);
      return false;
    }
  }
  return true;
}

bool expect_memory_patterns(uint32_t base_addr, uint32_t first_seed,
                            uint32_t beat_count) {
  for (uint32_t beat = 0; beat < beat_count; ++beat) {
    const uint32_t beat_addr = base_addr + beat * sim_ddr::AXI_DATA_BYTES;
    if (!expect_memory_pattern(beat_addr, first_seed + beat * 0x100u)) {
      return false;
    }
  }
  return true;
}

bool test_wready_stays_low_through_a_drain_batch(sim_ddr::SimDDR &ddr) {
  std::printf("=== Test 1: WREADY stays low through a drain batch ===\n");

  constexpr uint32_t kAddr = 0x2000;
  constexpr uint32_t kBeatCount = 6;
  const sim_ddr::axi_strb_t full_strobe = make_full_strobe();

  clear_master_signals(ddr);
  if (!issue_aw(ddr, 0x1, kAddr, static_cast<uint8_t>(kBeatCount - 1))) {
    return false;
  }

  for (uint32_t beat = 0; beat < 4; ++beat) {
    if (!issue_w_beat_expect_immediate_ready(
            ddr, make_pattern(0x10000000u + beat * 0x100u), full_strobe,
            false)) {
      return false;
    }
  }

  ddr.io.w.wvalid = true;
  ddr.io.w.wdata = make_pattern(0x10000000u + 4u * 0x100u);
  ddr.io.w.wstrb = full_strobe;
  ddr.io.w.wlast = false;

  int stalled_cycles = 0;
  while (stalled_cycles < kHandshakeTimeout) {
    ddr.comb();
    if (ddr.io.w.wready) {
      break;
    }
    if (!expect_memory_zero(kAddr + 4u * sim_ddr::AXI_DATA_BYTES)) {
      ddr.io.w.wvalid = false;
      return false;
    }
    ++stalled_cycles;
    advance_seq(ddr);
  }

  if (stalled_cycles < 2) {
    std::printf("FAIL: expected multi-cycle drain-mode stall, got %d cycles\n",
                stalled_cycles);
    ddr.print_state();
    ddr.io.w.wvalid = false;
    return false;
  }
  if (!ddr.io.w.wready) {
    std::printf("FAIL: WREADY did not reopen after the drain batch\n");
    ddr.io.w.wvalid = false;
    return false;
  }

  advance_seq(ddr); // Accept beat 4
  ddr.io.w.wvalid = false;
  ddr.io.w.wlast = false;

  if (!issue_w_beat(ddr, make_pattern(0x10000000u + 5u * 0x100u), full_strobe,
                    true)) {
    return false;
  }

  const int b_cycles = wait_b_visible(ddr);
  if (b_cycles < 0) {
    std::printf("FAIL: B did not become visible for the drained burst\n");
    return false;
  }
  if (!expect_memory_patterns(kAddr, 0x10000000u, kBeatCount)) {
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_short_completed_burst_drains_without_hitting_high_watermark(
    sim_ddr::SimDDR &ddr) {
  std::printf(
      "=== Test 2: short completed burst drains without hitting high watermark ===\n");

  constexpr uint32_t kAddr = 0x6000;
  const sim_ddr::axi_strb_t full_strobe = make_full_strobe();

  clear_master_signals(ddr);
  if (!issue_aw(ddr, 0x2, kAddr, 1)) {
    return false;
  }

  if (!issue_w_beat_expect_immediate_ready(ddr, make_pattern(0x20000000u),
                                           full_strobe, false)) {
    return false;
  }
  if (!issue_w_beat_expect_immediate_ready(ddr, make_pattern(0x20000100u),
                                           full_strobe, true)) {
    return false;
  }

  ddr.comb();
  if (ddr.io.b.bvalid) {
    std::printf("FAIL: B became visible before the buffered tail had drained\n");
    return false;
  }

  const int b_cycles = wait_b_visible(ddr);
  if (b_cycles < 0) {
    std::printf("FAIL: completed short burst never produced B\n");
    return false;
  }
  if (!expect_memory_patterns(kAddr, 0x20000000u, 2)) {
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_aw_channel_can_progress_while_w_is_draining(sim_ddr::SimDDR &ddr) {
  std::printf("=== Test 3: AW can progress while W is blocked by drain mode ===\n");

  constexpr uint32_t kAddr0 = 0xA000;
  constexpr uint32_t kAddr1 = 0xC000;
  const sim_ddr::axi_strb_t full_strobe = make_full_strobe();

  clear_master_signals(ddr);
  if (!issue_aw(ddr, 0x4, kAddr0, 5)) {
    return false;
  }

  for (uint32_t beat = 0; beat < 4; ++beat) {
    if (!issue_w_beat_expect_immediate_ready(
            ddr, make_pattern(0x30000000u + beat * 0x100u), full_strobe,
            false)) {
      return false;
    }
  }

  ddr.io.w.wvalid = true;
  ddr.io.w.wdata = make_pattern(0x30000000u + 4u * 0x100u);
  ddr.io.w.wstrb = full_strobe;
  ddr.io.w.wlast = false;

  ddr.io.aw.awvalid = true;
  ddr.io.aw.awid = 0x5;
  ddr.io.aw.awaddr = kAddr1;
  ddr.io.aw.awlen = 0;
  ddr.io.aw.awsize = sim_ddr::AXI_SIZE_CODE;
  ddr.io.aw.awburst = sim_ddr::AXI_BURST_INCR;

  ddr.comb();
  if (ddr.io.w.wready) {
    std::printf("FAIL: expected WREADY low while the controller is draining\n");
    ddr.io.w.wvalid = false;
    ddr.io.aw.awvalid = false;
    return false;
  }
  if (!ddr.io.aw.awready) {
    std::printf("FAIL: AWREADY unexpectedly low while only W is backpressured\n");
    ddr.io.w.wvalid = false;
    ddr.io.aw.awvalid = false;
    return false;
  }

  advance_seq(ddr); // Handshake AW only; W remains stalled
  ddr.io.aw.awvalid = false;

  if (!wait_w_ready(ddr)) {
    std::printf("FAIL: WREADY never reopened for the stalled beat\n");
    ddr.io.w.wvalid = false;
    return false;
  }
  advance_seq(ddr); // Accept beat 4
  ddr.io.w.wvalid = false;

  if (!issue_w_beat(ddr, make_pattern(0x30000000u + 5u * 0x100u), full_strobe,
                    true)) {
    return false;
  }

  const int first_b_cycles = wait_b_visible(ddr);
  if (first_b_cycles < 0 || ddr.io.b.bid != 0x4) {
    std::printf("FAIL: first B did not correspond to the head write transaction\n");
    return false;
  }
  sim_cycle(ddr);

  if (!issue_w_beat(ddr, make_pattern(0x40000000u), full_strobe, true)) {
    return false;
  }

  const int second_b_cycles = wait_b_visible(ddr);
  if (second_b_cycles < 0 || ddr.io.b.bid != 0x5) {
    std::printf("FAIL: queued AW did not complete after the drain batch\n");
    return false;
  }

  if (!expect_memory_patterns(kAddr0, 0x30000000u, 6) ||
      !expect_memory_pattern(kAddr1, 0x40000000u)) {
    return false;
  }

  std::printf("PASS\n");
  return true;
}

} // namespace

int main() {
  p_memory = new uint32_t[kTestMemWords];
  std::memset(p_memory, 0, kTestMemWords * sizeof(uint32_t));

  sim_ddr::SimDDR ddr;
  ddr.init();

  int passed = 0;
  int failed = 0;

  if (test_wready_stays_low_through_a_drain_batch(ddr)) {
    ++passed;
  } else {
    ++failed;
  }

  ddr.init();
  sim_time = 0;
  std::memset(p_memory, 0, kTestMemWords * sizeof(uint32_t));
  if (test_short_completed_burst_drains_without_hitting_high_watermark(ddr)) {
    ++passed;
  } else {
    ++failed;
  }

  ddr.init();
  sim_time = 0;
  std::memset(p_memory, 0, kTestMemWords * sizeof(uint32_t));
  if (test_aw_channel_can_progress_while_w_is_draining(ddr)) {
    ++passed;
  } else {
    ++failed;
  }

  std::printf("\n====================================\n");
  std::printf("Write drain-mode tests: %d passed, %d failed\n", passed, failed);
  std::printf("====================================\n");

  delete[] p_memory;
  return failed == 0 ? 0 : 1;
}
