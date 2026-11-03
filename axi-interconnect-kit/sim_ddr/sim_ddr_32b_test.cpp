/**
 * @file sim_ddr_32b_test.cpp
 * @brief Focused 32B-beat coverage for SimDDR.
 */

#include "SimDDR.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

uint32_t *p_memory = nullptr;
long long sim_time = 0;

namespace {

constexpr uint32_t TEST_MEM_SIZE_WORDS = 0x100000;
constexpr int kHandshakeTimeout = 20;
constexpr int kDataTimeout = sim_ddr::SIM_DDR_LATENCY * 8 + 200;

void sim_cycle(sim_ddr::SimDDR &ddr) {
  ddr.comb();
  ddr.seq();
  ++sim_time;
}

void advance_seq(sim_ddr::SimDDR &ddr) {
  ddr.seq();
  ++sim_time;
}

sim_ddr::axi_strb_t make_full_strobe() {
  sim_ddr::axi_strb_t strobe{};
  strobe = 0;
  for (uint8_t byte = 0; byte < sim_ddr::AXI_DATA_BYTES; ++byte) {
    axi_compat::set_bit(strobe, byte, true);
  }
  return strobe;
}

sim_ddr::axi_data_t make_word_pattern(uint32_t base_word) {
  sim_ddr::axi_data_t data{};
  data = 0;
  for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
    axi_compat::set_u32(data, word, base_word + word);
  }
  return data;
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

bool wait_aw_ready(sim_ddr::SimDDR &ddr) {
  int timeout = kHandshakeTimeout;
  ddr.comb();
  while (!ddr.io.aw.awready && timeout-- > 0) {
    advance_seq(ddr);
    ddr.comb();
  }
  return ddr.io.aw.awready;
}

bool wait_w_ready(sim_ddr::SimDDR &ddr) {
  int timeout = kHandshakeTimeout;
  ddr.comb();
  while (!ddr.io.w.wready && timeout-- > 0) {
    advance_seq(ddr);
    ddr.comb();
  }
  return ddr.io.w.wready;
}

bool wait_b_valid(sim_ddr::SimDDR &ddr) {
  int timeout = kDataTimeout;
  ddr.comb();
  while (!ddr.io.b.bvalid && timeout-- > 0) {
    advance_seq(ddr);
    ddr.comb();
  }
  return ddr.io.b.bvalid;
}

bool wait_ar_ready(sim_ddr::SimDDR &ddr) {
  int timeout = kHandshakeTimeout;
  ddr.comb();
  while (!ddr.io.ar.arready && timeout-- > 0) {
    advance_seq(ddr);
    ddr.comb();
  }
  return ddr.io.ar.arready;
}

bool wait_r_valid(sim_ddr::SimDDR &ddr) {
  int timeout = kDataTimeout;
  ddr.comb();
  while (!ddr.io.r.rvalid && timeout-- > 0) {
    advance_seq(ddr);
    ddr.comb();
  }
  return ddr.io.r.rvalid;
}

bool test_single_32b_write_read(sim_ddr::SimDDR &ddr) {
  std::printf("=== Test 1: Single 32B beat write/read ===\n");

  const uint32_t addr = 0x2000;
  const sim_ddr::axi_data_t write_data = make_word_pattern(0xABC00000u);
  const sim_ddr::axi_strb_t full_strobe = make_full_strobe();

  clear_master_signals(ddr);

  ddr.io.aw.awvalid = true;
  ddr.io.aw.awaddr = addr;
  ddr.io.aw.awlen = 0;
  ddr.io.aw.awsize = sim_ddr::AXI_SIZE_CODE;
  if (!wait_aw_ready(ddr)) {
    std::printf("FAIL: AW handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.aw.awvalid = false;

  ddr.io.w.wvalid = true;
  ddr.io.w.wdata = write_data;
  ddr.io.w.wstrb = full_strobe;
  ddr.io.w.wlast = true;
  if (!wait_w_ready(ddr)) {
    std::printf("FAIL: W handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.w.wvalid = false;
  ddr.io.w.wlast = false;

  if (!wait_b_valid(ddr)) {
    std::printf("FAIL: B response timeout\n");
    return false;
  }
  if (ddr.io.b.bresp != sim_ddr::AXI_RESP_OKAY) {
    std::printf("FAIL: B response mismatch resp=%u\n",
                static_cast<unsigned>(ddr.io.b.bresp));
    return false;
  }
  sim_cycle(ddr);

  for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
    const uint32_t got = p_memory[(addr >> 2) + word];
    const uint32_t exp = 0xABC00000u + word;
    if (got != exp) {
      std::printf("FAIL: memory word[%u] exp=0x%08x got=0x%08x\n", word, exp,
                  got);
      return false;
    }
  }

  clear_master_signals(ddr);
  ddr.io.ar.arvalid = true;
  ddr.io.ar.araddr = addr;
  ddr.io.ar.arlen = 0;
  ddr.io.ar.arsize = sim_ddr::AXI_SIZE_CODE;
  if (!wait_ar_ready(ddr)) {
    std::printf("FAIL: AR handshake timeout\n");
    return false;
  }
  sim_cycle(ddr);
  ddr.io.ar.arvalid = false;

  if (!wait_r_valid(ddr)) {
    std::printf("FAIL: R response timeout\n");
    return false;
  }
  for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
    const uint32_t got = axi_compat::get_u32(ddr.io.r.rdata, word);
    const uint32_t exp = 0xABC00000u + word;
    if (got != exp) {
      std::printf("FAIL: read word[%u] exp=0x%08x got=0x%08x\n", word, exp,
                  got);
      return false;
    }
  }
  if (!ddr.io.r.rlast) {
    std::printf("FAIL: expected single-beat read to assert rlast\n");
    return false;
  }
  sim_cycle(ddr);

  std::printf("PASS\n");
  return true;
}

bool test_partial_32b_strobe(sim_ddr::SimDDR &ddr) {
  std::printf("=== Test 2: 32B beat partial strobe ===\n");

  const uint32_t addr = 0x3000;
  auto *byte_mem = reinterpret_cast<uint8_t *>(p_memory);
  std::memset(byte_mem + addr, 0x11, sim_ddr::AXI_DATA_BYTES);

  sim_ddr::axi_data_t write_data{};
  write_data = 0;
  sim_ddr::axi_strb_t wstrb{};
  wstrb = 0;
  for (uint8_t byte = 0; byte < sim_ddr::AXI_DATA_BYTES; ++byte) {
    axi_compat::set_byte(write_data, byte, static_cast<uint8_t>(0x80u + byte));
    if ((byte % 3u) != 1u) {
      axi_compat::set_bit(wstrb, byte, true);
    }
  }

  clear_master_signals(ddr);
  ddr.io.aw.awvalid = true;
  ddr.io.aw.awaddr = addr;
  ddr.io.aw.awlen = 0;
  ddr.io.aw.awsize = sim_ddr::AXI_SIZE_CODE;
  if (!wait_aw_ready(ddr)) {
    std::printf("FAIL: AW handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.aw.awvalid = false;

  ddr.io.w.wvalid = true;
  ddr.io.w.wdata = write_data;
  ddr.io.w.wstrb = wstrb;
  ddr.io.w.wlast = true;
  if (!wait_w_ready(ddr)) {
    std::printf("FAIL: W handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.w.wvalid = false;
  ddr.io.w.wlast = false;

  if (!wait_b_valid(ddr)) {
    std::printf("FAIL: B response timeout\n");
    return false;
  }
  sim_cycle(ddr);

  for (uint8_t byte = 0; byte < sim_ddr::AXI_DATA_BYTES; ++byte) {
    const uint8_t got = byte_mem[addr + byte];
    const uint8_t exp = ((byte % 3u) != 1u) ? static_cast<uint8_t>(0x80u + byte)
                                            : static_cast<uint8_t>(0x11u);
    if (got != exp) {
      std::printf("FAIL: byte[%u] exp=0x%02x got=0x%02x\n", byte, exp, got);
      return false;
    }
  }

  std::printf("PASS\n");
  return true;
}

} // namespace

int main() {
  static_assert(sim_ddr::AXI_DATA_BYTES == 32,
                "sim_ddr_32b_test must be built with 32B beats");

  p_memory = static_cast<uint32_t *>(
      std::calloc(TEST_MEM_SIZE_WORDS, sizeof(uint32_t)));
  if (p_memory == nullptr) {
    std::printf("FAIL: could not allocate test memory\n");
    return 1;
  }

  sim_ddr::SimDDR ddr;
  ddr.init();

  int passed = 0;
  int failed = 0;

  if (test_single_32b_write_read(ddr)) {
    ++passed;
  } else {
    ++failed;
  }

  if (test_partial_32b_strobe(ddr)) {
    ++passed;
  } else {
    ++failed;
  }

  std::printf("32B SimDDR results: %d passed, %d failed\n", passed, failed);
  std::free(p_memory);
  return failed == 0 ? 0 : 1;
}
