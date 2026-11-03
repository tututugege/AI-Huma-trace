#include "SimDDR.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

uint32_t *p_memory = nullptr;
long long sim_time = 0;

namespace {

#ifndef SIM_DDR_WRITE_QUEUE_TEST_QUEUE_DEPTH
#error "SIM_DDR_WRITE_QUEUE_TEST_QUEUE_DEPTH must be defined"
#endif

#ifndef SIM_DDR_WRITE_QUEUE_TEST_ACCEPT_GAP
#error "SIM_DDR_WRITE_QUEUE_TEST_ACCEPT_GAP must be defined"
#endif

#ifndef SIM_DDR_WRITE_QUEUE_TEST_DATA_FIFO_DEPTH
#error "SIM_DDR_WRITE_QUEUE_TEST_DATA_FIFO_DEPTH must be defined"
#endif

#ifndef SIM_DDR_WRITE_QUEUE_TEST_DRAIN_GAP
#error "SIM_DDR_WRITE_QUEUE_TEST_DRAIN_GAP must be defined"
#endif

#ifndef SIM_DDR_WRITE_QUEUE_TEST_DRAIN_HIGH_WATERMARK
#error "SIM_DDR_WRITE_QUEUE_TEST_DRAIN_HIGH_WATERMARK must be defined"
#endif

#ifndef SIM_DDR_WRITE_QUEUE_TEST_DRAIN_LOW_WATERMARK
#error "SIM_DDR_WRITE_QUEUE_TEST_DRAIN_LOW_WATERMARK must be defined"
#endif

static_assert(sim_ddr::SIM_DDR_WRITE_QUEUE_DEPTH ==
                  SIM_DDR_WRITE_QUEUE_TEST_QUEUE_DEPTH,
              "test binary must use the intended write queue depth override");
static_assert(sim_ddr::SIM_DDR_WRITE_ACCEPT_GAP ==
                  SIM_DDR_WRITE_QUEUE_TEST_ACCEPT_GAP,
              "test binary must use the intended write accept gap override");
static_assert(sim_ddr::SIM_DDR_WRITE_DATA_FIFO_DEPTH ==
                  SIM_DDR_WRITE_QUEUE_TEST_DATA_FIFO_DEPTH,
              "test binary must use the intended write data fifo depth override");
static_assert(sim_ddr::SIM_DDR_WRITE_DRAIN_GAP ==
                  SIM_DDR_WRITE_QUEUE_TEST_DRAIN_GAP,
              "test binary must use the intended write drain gap override");
static_assert(
    sim_ddr::SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK ==
        SIM_DDR_WRITE_QUEUE_TEST_DRAIN_HIGH_WATERMARK,
    "test binary must use the intended write drain high watermark override");
static_assert(
    sim_ddr::SIM_DDR_WRITE_DRAIN_LOW_WATERMARK ==
        SIM_DDR_WRITE_QUEUE_TEST_DRAIN_LOW_WATERMARK,
    "test binary must use the intended write drain low watermark override");

constexpr uint32_t kTestMemWords = 0x100000;
constexpr int kHandshakeTimeout = 40;
constexpr int kResponseTimeout =
    static_cast<int>(sim_ddr::SIM_DDR_WRITE_RESP_LATENCY) + 32;

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

bool issue_w_beat(sim_ddr::SimDDR &ddr, sim_ddr::axi_data_t data,
                  sim_ddr::axi_strb_t strobe, bool last) {
  ddr.io.w.wvalid = true;
  ddr.io.w.wdata = data;
  ddr.io.w.wstrb = strobe;
  ddr.io.w.wlast = last;
  if (!wait_w_ready(ddr)) {
    std::printf("FAIL: W handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.w.wvalid = false;
  ddr.io.w.wlast = false;
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
    std::printf("FAIL: WREADY unexpectedly low for a back-to-back beat at sim_time=%lld\n",
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

bool expect_memory_zero(uint32_t addr) {
  for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
    if (p_memory[(addr >> 2) + word] != 0) {
      std::printf("FAIL: memory at 0x%08x changed before expected\n", addr);
      return false;
    }
  }
  return true;
}

bool test_aw_queue_full_backpressure_and_retry(sim_ddr::SimDDR &ddr) {
  std::printf("=== Test 1: AW queue full backpressure and retry ===\n");

  constexpr uint32_t kAddr0 = 0x4000;
  constexpr uint32_t kAddr1 = 0x5000;
  constexpr uint32_t kAddr2 = 0x6000;

  clear_master_signals(ddr);

  if (!issue_aw(ddr, 1, kAddr0) || !issue_aw(ddr, 2, kAddr1)) {
    return false;
  }

  ddr.comb();
  if (ddr.io.aw.awready) {
    std::printf("FAIL: AWREADY stayed high when write queue should be full\n");
    return false;
  }

  if (!issue_w_beat(ddr, make_pattern(0x10000000u), make_full_strobe(), true)) {
    return false;
  }

  if (!wait_aw_ready(ddr)) {
    std::printf("FAIL: AWREADY did not reopen after one queue slot freed\n");
    return false;
  }

  if (!issue_aw(ddr, 3, kAddr2)) {
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_w_fifo_backpressure_is_bursty_not_uniform(sim_ddr::SimDDR &ddr) {
  std::printf("=== Test 2: W backpressure comes from FIFO fill, not every beat ===\n");

  constexpr uint32_t kAddr = 0x8000;
  constexpr uint32_t kBeatCount = 6;
  const sim_ddr::axi_strb_t full_strobe = make_full_strobe();

  clear_master_signals(ddr);

  if (!issue_aw(ddr, 7, kAddr, static_cast<uint8_t>(kBeatCount - 1))) {
    return false;
  }

  uint32_t first_stalled_beat = kBeatCount;
  for (uint32_t beat = 0; beat < kBeatCount; ++beat) {
    ddr.io.w.wvalid = true;
    ddr.io.w.wdata = make_pattern(0x20000000u + beat * 0x100u);
    ddr.io.w.wstrb = full_strobe;
    ddr.io.w.wlast = (beat + 1u == kBeatCount);
    ddr.comb();
    if (!ddr.io.w.wready) {
      first_stalled_beat = beat;
      break;
    }
    advance_seq(ddr);
    ddr.io.w.wvalid = false;
    ddr.io.w.wlast = false;
  }

  if (first_stalled_beat == kBeatCount) {
    std::printf("FAIL: expected the write FIFO to apply backpressure\n");
    return false;
  }
  if (first_stalled_beat < 2u) {
    std::printf("FAIL: backpressure arrived too early; got only %u immediate beats\n",
                first_stalled_beat);
    return false;
  }

  const uint32_t stalled_beat_addr =
      kAddr + first_stalled_beat * sim_ddr::AXI_DATA_BYTES;
  if (!expect_memory_zero(stalled_beat_addr)) {
    return false;
  }

  ddr.io.w.wvalid = true;
  ddr.io.w.wdata = make_pattern(0x20000000u + first_stalled_beat * 0x100u);
  ddr.io.w.wstrb = full_strobe;
  ddr.io.w.wlast = (first_stalled_beat + 1u == kBeatCount);
  if (!wait_w_ready(ddr)) {
    std::printf("FAIL: WREADY did not reopen after backend drain freed FIFO credit\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.w.wvalid = false;
  ddr.io.w.wlast = false;

  for (uint32_t beat = first_stalled_beat + 1u; beat < kBeatCount; ++beat) {
    if (!issue_w_beat(ddr, make_pattern(0x20000000u + beat * 0x100u),
                      full_strobe, beat + 1u == kBeatCount)) {
      return false;
    }
  }

  const int b_cycles = wait_b_visible(ddr);
  if (b_cycles <= static_cast<int>(sim_ddr::SIM_DDR_WRITE_RESP_LATENCY) + 1) {
    std::printf(
        "FAIL: B became visible too early; expected extra delay from buffered writes, got %d cycles\n",
        b_cycles);
    return false;
  }
  if (!expect_memory_patterns(kAddr, 0x20000000u, kBeatCount)) {
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_multi_aw_single_w_stream_axi4_contract(sim_ddr::SimDDR &ddr) {
  std::printf("=== Test 3: multi-AW still drains a single W stream ===\n");

  constexpr uint32_t kAddr0 = 0xA000;
  constexpr uint32_t kAddr1 = 0xB000;

  clear_master_signals(ddr);

  if (!issue_aw(ddr, 0xA, kAddr0) || !issue_aw(ddr, 0xB, kAddr1)) {
    return false;
  }

  if (!issue_w_beat(ddr, make_pattern(0x40000000u), make_full_strobe(), true)) {
    return false;
  }
  const int first_b_cycles = wait_b_visible(ddr);
  if (first_b_cycles < 0 || ddr.io.b.bid != 0xA) {
    std::printf("FAIL: first B did not belong to the head-of-line AW\n");
    return false;
  }
  if (!expect_memory_zero(kAddr1)) {
    return false;
  }

  sim_cycle(ddr);
  if (!issue_w_beat(ddr, make_pattern(0x50000000u), make_full_strobe(), true)) {
    return false;
  }
  const int second_b_cycles = wait_b_visible(ddr);
  if (second_b_cycles < 0 || ddr.io.b.bid != 0xB) {
    std::printf("FAIL: second B did not route back to the second AW\n");
    return false;
  }

  if (!expect_memory_pattern(kAddr0, 0x40000000u) ||
      !expect_memory_pattern(kAddr1, 0x50000000u)) {
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_bready_hold_preserves_response_and_allows_aw_progress(
    sim_ddr::SimDDR &ddr) {
  std::printf("=== Test 4: B hold preserves response while AW can still queue ===\n");

  constexpr uint32_t kAddr0 = 0xC000;
  constexpr uint32_t kAddr1 = 0xD000;
  const sim_ddr::axi_strb_t full_strobe = make_full_strobe();

  clear_master_signals(ddr);

  if (!issue_aw(ddr, 0x1, kAddr0)) {
    return false;
  }
  if (!issue_w_beat(ddr, make_pattern(0x60000000u), full_strobe, true)) {
    return false;
  }

  ddr.io.b.bready = false;
  const int b_cycles = wait_b_visible(ddr);
  if (b_cycles < 0 || ddr.io.b.bid != 0x1) {
    std::printf("FAIL: first B did not become visible under bready=0\n");
    return false;
  }

  if (!issue_aw(ddr, 0x2, kAddr1)) {
    return false;
  }

  for (int i = 0; i < 3; ++i) {
    sim_cycle(ddr);
    if (!ddr.io.b.bvalid || ddr.io.b.bid != 0x1 ||
        ddr.io.b.bresp != sim_ddr::AXI_RESP_OKAY) {
      std::printf("FAIL: held B response changed while bready=0\n");
      return false;
    }
  }

  ddr.io.b.bready = true;
  sim_cycle(ddr);
  sim_cycle(ddr);
  if (ddr.io.b.bvalid) {
    std::printf("FAIL: B response did not clear after release\n");
    return false;
  }

  if (!issue_w_beat(ddr, make_pattern(0x70000000u), full_strobe, true)) {
    return false;
  }
  const int second_b_cycles = wait_b_visible(ddr);
  if (second_b_cycles < 0 || ddr.io.b.bid != 0x2) {
    std::printf("FAIL: queued AW did not complete after held B was released\n");
    return false;
  }

  if (!expect_memory_pattern(kAddr0, 0x60000000u) ||
      !expect_memory_pattern(kAddr1, 0x70000000u)) {
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

  if (test_aw_queue_full_backpressure_and_retry(ddr)) {
    ++passed;
  } else {
    ++failed;
  }

  ddr.init();
  sim_time = 0;
  std::memset(p_memory, 0, kTestMemWords * sizeof(uint32_t));
  if (test_w_fifo_backpressure_is_bursty_not_uniform(ddr)) {
    ++passed;
  } else {
    ++failed;
  }

  ddr.init();
  sim_time = 0;
  std::memset(p_memory, 0, kTestMemWords * sizeof(uint32_t));
  if (test_multi_aw_single_w_stream_axi4_contract(ddr)) {
    ++passed;
  } else {
    ++failed;
  }

  ddr.init();
  sim_time = 0;
  std::memset(p_memory, 0, kTestMemWords * sizeof(uint32_t));
  if (test_bready_hold_preserves_response_and_allows_aw_progress(ddr)) {
    ++passed;
  } else {
    ++failed;
  }

  std::printf("\n====================================\n");
  std::printf("Write queue tests: %d passed, %d failed\n", passed, failed);
  std::printf("====================================\n");

  delete[] p_memory;
  return failed == 0 ? 0 : 1;
}
