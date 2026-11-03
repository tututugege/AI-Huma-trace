#include "SimDDR.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

uint32_t *p_memory = nullptr;
long long sim_time = 0;

namespace {

#ifndef SIM_DDR_WRITE_RESP_TEST_LATENCY
#error "SIM_DDR_WRITE_RESP_TEST_LATENCY must be defined"
#endif

constexpr uint32_t kExpectedWriteRespLatency =
    SIM_DDR_WRITE_RESP_TEST_LATENCY;
static_assert(
    sim_ddr::SIM_DDR_WRITE_RESP_LATENCY == kExpectedWriteRespLatency,
    "test binary must use the intended write-response latency override");

constexpr uint32_t kTestMemWords = 0x100000;
constexpr int kHandshakeTimeout = 20;
constexpr int kMaxResponseCycles =
    static_cast<int>(sim_ddr::SIM_DDR_WRITE_RESP_LATENCY) + 8;

void sim_cycle(sim_ddr::SimDDR &ddr) {
  ddr.comb();
  ddr.seq();
  ++sim_time;
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

sim_ddr::axi_data_t make_test_pattern(uint32_t seed) {
  sim_ddr::axi_data_t data{};
  data = 0;
  for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
    axi_compat::set_u32(data, word, seed + word);
  }
  return data;
}

bool wait_aw_ready(sim_ddr::SimDDR &ddr) {
  int timeout = kHandshakeTimeout;
  while (!ddr.io.aw.awready && timeout-- > 0) {
    sim_cycle(ddr);
  }
  return ddr.io.aw.awready;
}

bool wait_w_ready(sim_ddr::SimDDR &ddr) {
  int timeout = kHandshakeTimeout;
  while (!ddr.io.w.wready && timeout-- > 0) {
    sim_cycle(ddr);
  }
  return ddr.io.w.wready;
}

int wait_for_b_visible(sim_ddr::SimDDR &ddr) {
  int visible_cycles = 0;
  while (!ddr.io.b.bvalid && visible_cycles < kMaxResponseCycles) {
    sim_cycle(ddr);
    ++visible_cycles;
  }
  return ddr.io.b.bvalid ? visible_cycles : -1;
}

bool issue_single_write(sim_ddr::SimDDR &ddr, uint8_t id, uint32_t addr,
                        sim_ddr::axi_data_t data,
                        sim_ddr::axi_strb_t strobe) {
  clear_master_signals(ddr);
  ddr.io.aw.awvalid = true;
  ddr.io.aw.awid = id;
  ddr.io.aw.awaddr = addr;
  ddr.io.aw.awlen = 0;
  ddr.io.aw.awsize = sim_ddr::AXI_SIZE_CODE;
  ddr.io.aw.awburst = sim_ddr::AXI_BURST_INCR;
  if (!wait_aw_ready(ddr)) {
    std::printf("FAIL: AW handshake timeout\n");
    return false;
  }
  sim_cycle(ddr);
  ddr.io.aw.awvalid = false;

  ddr.io.w.wvalid = true;
  ddr.io.w.wdata = data;
  ddr.io.w.wstrb = strobe;
  ddr.io.w.wlast = true;
  if (!wait_w_ready(ddr)) {
    std::printf("FAIL: W handshake timeout\n");
    return false;
  }
  sim_cycle(ddr);
  ddr.io.w.wvalid = false;
  ddr.io.w.wlast = false;
  return true;
}

bool check_memory_pattern(uint32_t addr, uint32_t seed) {
  for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
    const uint32_t got = p_memory[(addr >> 2) + word];
    const uint32_t expected = seed + word;
    if (got != expected) {
      std::printf("FAIL: memory[%u] exp=0x%08x got=0x%08x\n", word, expected,
                  got);
      return false;
    }
  }
  return true;
}

bool test_b_visible_after_expected_cycles(sim_ddr::SimDDR &ddr) {
  std::printf("=== Test 1: write response latency=%u ===\n",
              static_cast<unsigned>(sim_ddr::SIM_DDR_WRITE_RESP_LATENCY));

  constexpr uint8_t kId = 0x5;
  constexpr uint32_t kAddr = 0x4000;
  constexpr uint32_t kSeed = 0xA5A50000u;
  const sim_ddr::axi_data_t data = make_test_pattern(kSeed);
  const sim_ddr::axi_strb_t strobe = make_full_strobe();
  const int expected_visible_cycles =
      static_cast<int>(sim_ddr::SIM_DDR_WRITE_RESP_LATENCY) + 2;

  if (!issue_single_write(ddr, kId, kAddr, data, strobe)) {
    return false;
  }

  const int visible_cycles = wait_for_b_visible(ddr);
  if (visible_cycles < 0) {
    std::printf("FAIL: B response timeout\n");
    return false;
  }
  if (visible_cycles != expected_visible_cycles) {
    std::printf("FAIL: expected B visible after %d cycles, got %d\n",
                expected_visible_cycles, visible_cycles);
    return false;
  }
  if (ddr.io.b.bid != kId || ddr.io.b.bresp != sim_ddr::AXI_RESP_OKAY) {
    std::printf("FAIL: unexpected B response bid=0x%x bresp=%u\n", ddr.io.b.bid,
                static_cast<unsigned>(ddr.io.b.bresp));
    return false;
  }
  sim_cycle(ddr);
  sim_cycle(ddr);
  if (ddr.io.b.bvalid) {
    std::printf("FAIL: B channel did not clear after handshake\n");
    return false;
  }
  if (!check_memory_pattern(kAddr, kSeed)) {
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_bready_stall_preserves_visible_response(sim_ddr::SimDDR &ddr) {
  std::printf("=== Test 2: bready stall after response becomes visible ===\n");

  constexpr uint8_t kId = 0x9;
  constexpr uint32_t kAddr = 0x5000;
  constexpr uint32_t kSeed = 0x5A5A0000u;
  const sim_ddr::axi_data_t data = make_test_pattern(kSeed);
  const sim_ddr::axi_strb_t strobe = make_full_strobe();
  const int expected_visible_cycles =
      static_cast<int>(sim_ddr::SIM_DDR_WRITE_RESP_LATENCY) + 2;

  if (!issue_single_write(ddr, kId, kAddr, data, strobe)) {
    return false;
  }

  ddr.io.b.bready = false;
  const int visible_cycles = wait_for_b_visible(ddr);
  if (visible_cycles < 0) {
    std::printf("FAIL: stalled B response timeout\n");
    return false;
  }
  if (visible_cycles != expected_visible_cycles) {
    std::printf("FAIL: stalled B expected after %d cycles, got %d\n",
                expected_visible_cycles, visible_cycles);
    return false;
  }

  for (int i = 0; i < 3; ++i) {
    sim_cycle(ddr);
    if (!ddr.io.b.bvalid) {
      std::printf("FAIL: B response dropped under bready=0\n");
      return false;
    }
    if (ddr.io.b.bid != kId || ddr.io.b.bresp != sim_ddr::AXI_RESP_OKAY) {
      std::printf("FAIL: stalled B response changed bid=0x%x bresp=%u\n",
                  ddr.io.b.bid, static_cast<unsigned>(ddr.io.b.bresp));
      return false;
    }
  }

  ddr.io.b.bready = true;
  sim_cycle(ddr);
  sim_cycle(ddr);
  if (ddr.io.b.bvalid) {
    std::printf("FAIL: stalled B response not consumed after handshake\n");
    return false;
  }
  if (!check_memory_pattern(kAddr, kSeed)) {
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

  if (test_b_visible_after_expected_cycles(ddr)) {
    ++passed;
  } else {
    ++failed;
  }

  ddr.init();
  sim_time = 0;
  std::memset(p_memory, 0, kTestMemWords * sizeof(uint32_t));
  if (test_bready_stall_preserves_visible_response(ddr)) {
    ++passed;
  } else {
    ++failed;
  }

  std::printf("\n====================================\n");
  std::printf("Write response latency tests: %d passed, %d failed\n", passed,
              failed);
  std::printf("====================================\n");

  delete[] p_memory;
  return failed == 0 ? 0 : 1;
}
