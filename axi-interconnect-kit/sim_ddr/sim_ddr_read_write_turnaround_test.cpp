#include "SimDDR.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

uint32_t *p_memory = nullptr;
long long sim_time = 0;

namespace {

#ifndef SIM_DDR_RW_TA_TEST_QUEUE_DEPTH
#error "SIM_DDR_RW_TA_TEST_QUEUE_DEPTH must be defined"
#endif

#ifndef SIM_DDR_RW_TA_TEST_DATA_FIFO_DEPTH
#error "SIM_DDR_RW_TA_TEST_DATA_FIFO_DEPTH must be defined"
#endif

#ifndef SIM_DDR_RW_TA_TEST_HIGH_WM
#error "SIM_DDR_RW_TA_TEST_HIGH_WM must be defined"
#endif

#ifndef SIM_DDR_RW_TA_TEST_LOW_WM
#error "SIM_DDR_RW_TA_TEST_LOW_WM must be defined"
#endif

#ifndef SIM_DDR_RW_TA_TEST_READ_LATENCY
#error "SIM_DDR_RW_TA_TEST_READ_LATENCY must be defined"
#endif

#ifndef SIM_DDR_RW_TA_TEST_R2W
#error "SIM_DDR_RW_TA_TEST_R2W must be defined"
#endif

#ifndef SIM_DDR_RW_TA_TEST_W2R
#error "SIM_DDR_RW_TA_TEST_W2R must be defined"
#endif

static_assert(sim_ddr::SIM_DDR_WRITE_QUEUE_DEPTH ==
                  SIM_DDR_RW_TA_TEST_QUEUE_DEPTH,
              "test binary must use the intended write queue depth override");
static_assert(sim_ddr::SIM_DDR_WRITE_DATA_FIFO_DEPTH ==
                  SIM_DDR_RW_TA_TEST_DATA_FIFO_DEPTH,
              "test binary must use the intended write data fifo depth override");
static_assert(sim_ddr::SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK ==
                  SIM_DDR_RW_TA_TEST_HIGH_WM,
              "test binary must use the intended high watermark override");
static_assert(sim_ddr::SIM_DDR_WRITE_DRAIN_LOW_WATERMARK ==
                  SIM_DDR_RW_TA_TEST_LOW_WM,
              "test binary must use the intended low watermark override");
static_assert(sim_ddr::SIM_DDR_LATENCY == SIM_DDR_RW_TA_TEST_READ_LATENCY,
              "test binary must use the intended read latency override");
static_assert(sim_ddr::SIM_DDR_READ_TO_WRITE_TURNAROUND ==
                  SIM_DDR_RW_TA_TEST_R2W,
              "test binary must use the intended read-to-write turnaround");
static_assert(sim_ddr::SIM_DDR_WRITE_TO_READ_TURNAROUND ==
                  SIM_DDR_RW_TA_TEST_W2R,
              "test binary must use the intended write-to-read turnaround");

constexpr uint32_t kTestMemWords = 0x100000;
constexpr int kHandshakeTimeout = 80;
constexpr int kWaitTimeout = 200;

void advance_seq(sim_ddr::SimDDR &ddr) {
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

sim_ddr::axi_data_t make_pattern(uint32_t seed) {
  sim_ddr::axi_data_t data{};
  data = 0;
  for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
    axi_compat::set_u32(data, word, seed + word);
  }
  return data;
}

void fill_memory_pattern(uint32_t addr, uint32_t seed) {
  for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
    p_memory[(addr >> 2) + word] = seed + word;
  }
}

bool expect_memory_pattern(uint32_t addr, uint32_t seed) {
  for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
    const uint32_t got = p_memory[(addr >> 2) + word];
    const uint32_t expected = seed + word;
    if (got != expected) {
      return false;
    }
  }
  return true;
}

bool expect_memory_zero(uint32_t addr) {
  for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
    if (p_memory[(addr >> 2) + word] != 0) {
      return false;
    }
  }
  return true;
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

bool issue_aw(sim_ddr::SimDDR &ddr, uint8_t id, uint32_t addr, uint8_t len) {
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

bool issue_w_beat_expect_ready(sim_ddr::SimDDR &ddr, sim_ddr::axi_data_t data,
                               sim_ddr::axi_strb_t strobe, bool last) {
  ddr.io.w.wvalid = true;
  ddr.io.w.wdata = data;
  ddr.io.w.wstrb = strobe;
  ddr.io.w.wlast = last;
  ddr.comb();
  if (!ddr.io.w.wready) {
    std::printf("FAIL: expected WREADY for beat at sim_time=%lld\n", sim_time);
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

bool issue_ar(sim_ddr::SimDDR &ddr, uint8_t id, uint32_t addr, uint8_t len) {
  ddr.io.ar.arvalid = true;
  ddr.io.ar.arid = id;
  ddr.io.ar.araddr = addr;
  ddr.io.ar.arlen = len;
  ddr.io.ar.arsize = sim_ddr::AXI_SIZE_CODE;
  ddr.io.ar.arburst = sim_ddr::AXI_BURST_INCR;
  int timeout = kHandshakeTimeout;
  while (timeout-- > 0) {
    ddr.comb();
    if (ddr.io.ar.arready) {
      break;
    }
    advance_seq(ddr);
  }
  ddr.comb();
  if (!ddr.io.ar.arready) {
    std::printf("FAIL: AR handshake timeout for id=%u\n",
                static_cast<unsigned>(id));
    ddr.io.ar.arvalid = false;
    return false;
  }
  advance_seq(ddr);
  ddr.io.ar.arvalid = false;
  return true;
}

bool test_write_to_read_turnaround_blocks_ready_read(sim_ddr::SimDDR &ddr) {
  std::printf("=== Test 1: write drain blocks ready read until turnaround clears ===\n");

  constexpr uint32_t kWriteAddr = 0x4000;
  constexpr uint32_t kReadAddr = 0x8000;
  constexpr uint32_t kReadSeed = 0xABC00000u;
  constexpr uint32_t kBeatCount = 4;
  const auto full_strobe = make_full_strobe();

  clear_master_signals(ddr);
  fill_memory_pattern(kReadAddr, kReadSeed);

  if (!issue_aw(ddr, 0x11, kWriteAddr, static_cast<uint8_t>(kBeatCount - 1))) {
    return false;
  }
  for (uint32_t beat = 0; beat < kBeatCount; ++beat) {
    if (!issue_w_beat_expect_ready(
            ddr, make_pattern(0x10000000u + beat * 0x100u), full_strobe,
            beat + 1u == kBeatCount)) {
      return false;
    }
  }

  if (!issue_ar(ddr, 0x21, kReadAddr, 0)) {
    return false;
  }

  const uint32_t last_write_addr =
      kWriteAddr + (kBeatCount - 1u) * sim_ddr::AXI_DATA_BYTES;
  int cycles_after_last_write = 0;
  bool saw_last_write_commit = false;

  for (int cyc = 0; cyc < kWaitTimeout; ++cyc) {
    ddr.comb();
    if (!saw_last_write_commit) {
      if (ddr.io.r.rvalid) {
        std::printf("FAIL: read became visible while write drain was still active\n");
        return false;
      }
      advance_seq(ddr);
      if (expect_memory_pattern(last_write_addr,
                                0x10000000u + (kBeatCount - 1u) * 0x100u)) {
        saw_last_write_commit = true;
        cycles_after_last_write = 0;
      }
      continue;
    }

    if (cycles_after_last_write < SIM_DDR_RW_TA_TEST_W2R) {
      if (ddr.io.r.rvalid) {
        std::printf(
            "FAIL: read became visible before write-to-read turnaround expired\n");
        return false;
      }
      advance_seq(ddr);
      ++cycles_after_last_write;
      continue;
    }

    if (ddr.io.r.rvalid) {
      if (ddr.io.r.rid != 0x21) {
        std::printf("FAIL: unexpected RID after turnaround got=%u\n",
                    static_cast<unsigned>(ddr.io.r.rid));
        return false;
      }
      for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
        const uint32_t got = axi_compat::get_u32(ddr.io.r.rdata, word);
        const uint32_t expected = kReadSeed + word;
        if (got != expected) {
          std::printf("FAIL: read data mismatch word=%u exp=0x%08x got=0x%08x\n",
                      word, expected, got);
          return false;
        }
      }
      advance_seq(ddr);
      std::printf("PASS\n");
      return true;
    }
    advance_seq(ddr);
    ++cycles_after_last_write;
  }

  std::printf("FAIL: read never became visible after write drain\n");
  return false;
}

bool test_read_to_write_turnaround_delays_first_write_commit(sim_ddr::SimDDR &ddr) {
  std::printf("=== Test 2: read burst delays first write commit by turnaround ===\n");

  constexpr uint32_t kReadAddr = 0xA000;
  constexpr uint32_t kWriteAddr = 0xC000;
  constexpr uint32_t kReadSeed = 0x12340000u;
  const auto full_strobe = make_full_strobe();

  clear_master_signals(ddr);
  fill_memory_pattern(kReadAddr, kReadSeed);
  ddr.io.r.rready = false;

  if (!issue_ar(ddr, 0x31, kReadAddr, 1)) {
    return false;
  }

  if (!issue_aw(ddr, 0x41, kWriteAddr, 0)) {
    return false;
  }
  if (!issue_w_beat_expect_ready(ddr, make_pattern(0x50000000u), full_strobe,
                                 true)) {
    return false;
  }
  ddr.io.r.rready = true;

  int reads_consumed = 0;
  int cycles_after_read_burst = 0;
  bool read_burst_finished = false;
  for (int cyc = 0; cyc < kWaitTimeout; ++cyc) {
    ddr.comb();
    if (!read_burst_finished) {
      if (!expect_memory_zero(kWriteAddr)) {
        std::printf("FAIL: write committed before the read burst completed\n");
        return false;
      }
      const bool beat_visible = ddr.io.r.rvalid;
      advance_seq(ddr);
      if (beat_visible) {
        reads_consumed++;
      }
      if (reads_consumed >= 2) {
        read_burst_finished = true;
        cycles_after_read_burst = 0;
      }
      continue;
    }

    if (cycles_after_read_burst < SIM_DDR_RW_TA_TEST_R2W) {
      if (!expect_memory_zero(kWriteAddr)) {
        std::printf(
            "FAIL: write committed before read-to-write turnaround expired\n");
        return false;
      }
      advance_seq(ddr);
      ++cycles_after_read_burst;
      continue;
    }

    if (expect_memory_pattern(kWriteAddr, 0x50000000u)) {
      std::printf("PASS\n");
      return true;
    }
    advance_seq(ddr);
    ++cycles_after_read_burst;
  }

  std::printf("FAIL: write never committed after read burst\n");
  return false;
}

} // namespace

int main() {
  p_memory = static_cast<uint32_t *>(std::calloc(kTestMemWords, sizeof(uint32_t)));
  if (p_memory == nullptr) {
    std::printf("FAIL: could not allocate test memory\n");
    return 1;
  }

  int failures = 0;

  {
    sim_time = 0;
    std::memset(p_memory, 0, kTestMemWords * sizeof(uint32_t));
    sim_ddr::SimDDR ddr;
    ddr.init();
    clear_master_signals(ddr);
    if (!test_write_to_read_turnaround_blocks_ready_read(ddr)) {
      failures++;
    }
  }

  {
    sim_time = 0;
    std::memset(p_memory, 0, kTestMemWords * sizeof(uint32_t));
    sim_ddr::SimDDR ddr;
    ddr.init();
    clear_master_signals(ddr);
    if (!test_read_to_write_turnaround_delays_first_write_commit(ddr)) {
      failures++;
    }
  }

  std::free(p_memory);
  p_memory = nullptr;

  if (failures == 0) {
    std::printf("\nALL TURNAROUND TESTS PASSED\n");
    return 0;
  }

  std::printf("\n%d TURNAROUND TEST(S) FAILED\n", failures);
  return 1;
}
