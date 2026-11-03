#include <cstdio>
#include <cstring>
#include <random>
#include <unordered_map>

#include "axi_test_axi4_llc_env.h"

uint32_t *p_memory = nullptr;
long long sim_time = 0;
constexpr uint32_t TEST_MEM_SIZE_WORDS = 0x100000;

namespace {

using namespace axi_interconnect;
using namespace axi_test;

AXI_LLCConfig make_mode2_llc_config() {
  AXI_LLCConfig cfg;
  cfg.enable = true;
  cfg.size_bytes = 8ull << 20;
  cfg.line_bytes = 64;
  cfg.ways = 16;
  cfg.mshr_num = 8;
  cfg.prefetch_enable = false;
  return cfg;
}

bool test_cross_master_write_then_read_latest() {
  std::printf("=== AXI4 LLC Integration Test 1: cross-master latest value ===\n");

  Axi4LlcTestEnv env;
  init_env(env);

  const uint32_t line_addr = 0x1800;
  const uint32_t read_addr = line_addr + 8;
  write_memory_line(line_addr, 0x1100);

  WideWriteData_t wdata;
  wdata.clear();
  for (uint32_t i = 0; i < 16; ++i) {
    wdata[i] = 0x2200 + i;
  }
  WideWriteStrb_t wstrb;
  wstrb.clear();
  for (uint32_t i = 0; i < 64; ++i) {
    wstrb.set(i, true);
  }

  if (!issue_write(env, MASTER_DCACHE_W, line_addr, wdata, wstrb, 63, 3, false)) {
    std::printf("FAIL: cacheable write not accepted\n");
    return false;
  }
  if (!wait_write_resp(env, MASTER_DCACHE_W, 3)) {
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_ICACHE, read_addr, 15, 4, false)) {
    std::printf("FAIL: post-write cacheable read not accepted\n");
    return false;
  }
  if (!wait_read_resp(env, MASTER_ICACHE, 4, 0x2202, 0x2203)) {
    return false;
  }
  if (!env.ar_events.empty()) {
    std::printf("FAIL: post-write cacheable read unexpectedly missed LLC ar_count=%zu\n",
                env.ar_events.size());
    return false;
  }
  if (env.interconnect.get_llc_perf_counters().read_hit == 0) {
    std::printf("FAIL: LLC did not record read hit after cacheable write\n");
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_bypass_read_does_not_allocate_line() {
  std::printf("=== AXI4 LLC Integration Test 2: bypass read does not allocate ===\n");

  Axi4LlcTestEnv env;
  init_env(env);

  const uint32_t line_addr = 0x1c00;
  const uint32_t read_addr = line_addr + 4;
  write_memory_line(line_addr, 0x3300);

  if (!issue_read(env, MASTER_UNCORE_LSU_R, read_addr, 15, 5, true)) {
    std::printf("FAIL: bypass read not accepted\n");
    return false;
  }
  if (!wait_read_resp(env, MASTER_UNCORE_LSU_R, 5, 0x3301, 0x3302)) {
    return false;
  }
  if (env.ar_events.size() != 1) {
    std::printf("FAIL: bypass read expected one DDR AR, got %zu\n",
                env.ar_events.size());
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_ICACHE, read_addr, 15, 6, false)) {
    std::printf("FAIL: first cacheable read not accepted\n");
    return false;
  }
  if (!wait_read_resp(env, MASTER_ICACHE, 6, 0x3301, 0x3302)) {
    return false;
  }
  if (env.ar_events.size() != 1) {
    std::printf("FAIL: first cacheable read should miss LLC once, got ar_count=%zu\n",
                env.ar_events.size());
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_DCACHE_R, read_addr, 15, 7, false)) {
    std::printf("FAIL: second cacheable read not accepted\n");
    return false;
  }
  if (!wait_read_resp(env, MASTER_DCACHE_R, 7, 0x3301, 0x3302)) {
    return false;
  }
  if (!env.ar_events.empty()) {
    std::printf("FAIL: second cacheable read should hit LLC without DDR AR, got %zu\n",
                env.ar_events.size());
    return false;
  }

  const auto &perf = env.interconnect.get_llc_perf_counters();
  if (perf.bypass_read == 0 || perf.read_miss == 0 || perf.read_hit == 0) {
    std::printf("FAIL: LLC perf counters unexpected bypass=%llu miss=%llu hit=%llu\n",
                static_cast<unsigned long long>(perf.bypass_read),
                static_cast<unsigned long long>(perf.read_miss),
                static_cast<unsigned long long>(perf.read_hit));
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_cacheable_and_bypass_parallel_disjoint() {
  std::printf("=== AXI4 LLC Integration Test 3: cacheable+bypass coexist ===\n");

  Axi4LlcTestEnv env;
  init_env(env);

  const uint32_t bypass_line = 0x2400;
  const uint32_t cache_line = 0x2480;
  const uint32_t bypass_addr = bypass_line + 4;
  const uint32_t cache_addr = cache_line + 8;
  write_memory_line(bypass_line, 0x4400);
  write_memory_line(cache_line, 0x5500);

  bool bypass_issued = false;
  bool cache_issued = false;
  int timeout = 200;
  while ((!bypass_issued || !cache_issued) && timeout-- > 0) {
    cycle_outputs(env);
    const bool bypass_ready =
        env.interconnect.read_ports[MASTER_UNCORE_LSU_R].req.ready;
    const bool cache_ready =
        env.interconnect.read_ports[MASTER_ICACHE].req.ready;

    if (!bypass_issued) {
      auto &rp = env.interconnect.read_ports[MASTER_UNCORE_LSU_R];
      rp.req.valid = true;
      rp.req.addr = bypass_addr;
      rp.req.total_size = 15;
      rp.req.id = 8;
      rp.req.bypass = true;
    }
    if (!cache_issued) {
      auto &rp = env.interconnect.read_ports[MASTER_ICACHE];
      rp.req.valid = true;
      rp.req.addr = cache_addr;
      rp.req.total_size = 15;
      rp.req.id = 9;
      rp.req.bypass = false;
    }

    cycle_inputs(env);
    bypass_issued = bypass_issued || bypass_ready;
    cache_issued = cache_issued || cache_ready;
  }
  if (!bypass_issued || !cache_issued) {
    std::printf("FAIL: parallel cacheable+bypass issue timeout bypass=%d cache=%d\n",
                static_cast<int>(bypass_issued), static_cast<int>(cache_issued));
    return false;
  }

  bool bypass_done = false;
  bool cache_done = false;
  timeout = sim_ddr::SIM_DDR_LATENCY * 100;
  while ((!bypass_done || !cache_done) && timeout-- > 0) {
    cycle_outputs(env);
    auto &bypass_resp = env.interconnect.read_ports[MASTER_UNCORE_LSU_R].resp;
    if (!bypass_done && bypass_resp.valid) {
      if (bypass_resp.id != 8 || bypass_resp.data[0] != 0x4401 ||
          bypass_resp.data[1] != 0x4402) {
        std::printf("FAIL: bypass parallel resp mismatch id=%u d0=0x%x d1=0x%x\n",
                    bypass_resp.id, bypass_resp.data[0], bypass_resp.data[1]);
        return false;
      }
      bypass_resp.ready = true;
      bypass_done = true;
    }
    auto &cache_resp = env.interconnect.read_ports[MASTER_ICACHE].resp;
    if (!cache_done && cache_resp.valid) {
      if (cache_resp.id != 9 || cache_resp.data[0] != 0x5502 ||
          cache_resp.data[1] != 0x5503) {
        std::printf("FAIL: cacheable parallel resp mismatch id=%u d0=0x%x d1=0x%x\n",
                    cache_resp.id, cache_resp.data[0], cache_resp.data[1]);
        return false;
      }
      cache_resp.ready = true;
      cache_done = true;
    }
    cycle_inputs(env);
  }
  if (!bypass_done || !cache_done) {
    std::printf("FAIL: parallel response timeout bypass=%d cache=%d\n",
                static_cast<int>(bypass_done), static_cast<int>(cache_done));
    return false;
  }

  if (env.ar_events.size() != 2) {
    std::printf("FAIL: expected two DDR ARs for bypass+cacheable miss, got %zu\n",
                env.ar_events.size());
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_DCACHE_R, cache_addr, 15, 10, false)) {
    std::printf("FAIL: cacheable follow-up read not accepted\n");
    return false;
  }
  if (!wait_read_resp(env, MASTER_DCACHE_R, 10, 0x5502, 0x5503)) {
    return false;
  }
  if (!env.ar_events.empty()) {
    std::printf("FAIL: cacheable follow-up read should hit LLC, got ar_count=%zu\n",
                env.ar_events.size());
    return false;
  }

  const auto &perf = env.interconnect.get_llc_perf_counters();
  if (perf.bypass_read == 0 || perf.read_miss == 0 || perf.read_hit == 0) {
    std::printf("FAIL: coexist perf counters unexpected bypass=%llu miss=%llu hit=%llu\n",
                static_cast<unsigned long long>(perf.bypass_read),
                static_cast<unsigned long long>(perf.read_miss),
                static_cast<unsigned long long>(perf.read_hit));
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_same_set_eviction_roundtrip_latest() {
  std::printf("=== AXI4 LLC Integration Test 4: same-set eviction preserves latest ===\n");

  Axi4LlcTestEnv env;
  AXI_LLCConfig cfg = make_small_llc_config();
  cfg.size_bytes = 256;
  cfg.line_bytes = 64;
  cfg.ways = 2;
  cfg.mshr_num = 2;
  init_env(env, cfg);

  const uint32_t line_a = 0x000;
  const uint32_t line_b = 0x080;
  const uint32_t line_c = 0x100;
  const uint32_t read_a = line_a + 8;
  const uint32_t read_b = line_b + 8;
  const uint32_t read_c = line_c + 8;
  write_memory_line(line_a, 0x1000);
  write_memory_line(line_b, 0x2000);
  write_memory_line(line_c, 0x3000);

  auto make_line = [](uint32_t base_word) {
    WideWriteData_t wdata;
    wdata.clear();
    for (uint32_t i = 0; i < 16; ++i) {
      wdata[i] = base_word + i;
    }
    return wdata;
  };

  WideWriteStrb_t full_strobe;
  full_strobe.clear();
  for (uint32_t i = 0; i < 64; ++i) {
    full_strobe.set(i, true);
  }

  if (!issue_write(env, MASTER_DCACHE_W, line_a, make_line(0xA000), full_strobe, 63, 1,
                   false) ||
      !wait_write_resp(env, MASTER_DCACHE_W, 1)) {
    std::printf("FAIL: write line A failed\n");
    return false;
  }
  if (!issue_write(env, MASTER_DCACHE_W, line_b, make_line(0xB000), full_strobe, 63, 2,
                   false) ||
      !wait_write_resp(env, MASTER_DCACHE_W, 2)) {
    std::printf("FAIL: write line B failed\n");
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_ICACHE, read_c, 15, 3, false) ||
      !wait_read_resp(env, MASTER_ICACHE, 3, 0x3002, 0x3003)) {
    std::printf("FAIL: read line C failed\n");
    return false;
  }
  if (env.ar_events.size() != 1) {
    std::printf("FAIL: line C should issue exactly one DDR AR, got %zu\n",
                env.ar_events.size());
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_DCACHE_R, read_a, 15, 4, false) ||
      !wait_read_resp(env, MASTER_DCACHE_R, 4, 0xA002, 0xA003)) {
    std::printf("FAIL: roundtrip read line A failed\n");
    return false;
  }
  if (read_mem_word(read_a) != 0xA002) {
    std::printf("FAIL: line A latest value did not reach memory got=0x%08x\n",
                read_mem_word(read_a));
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_ICACHE, read_b, 15, 5, false) ||
      !wait_read_resp(env, MASTER_ICACHE, 5, 0xB002, 0xB003)) {
    std::printf("FAIL: roundtrip read line B failed\n");
    return false;
  }
  if (read_mem_word(read_b) != 0xB002) {
    std::printf("FAIL: line B latest value did not reach memory got=0x%08x\n",
                read_mem_word(read_b));
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_same_set_valid_updates_preserve_neighbor_way() {
  std::printf("=== AXI4 LLC Integration Test 4a: same-set valid updates preserve neighbor way ===\n");

  Axi4LlcTestEnv env;
  AXI_LLCConfig cfg = make_small_llc_config();
  cfg.size_bytes = 256;
  cfg.line_bytes = 64;
  cfg.ways = 2;
  cfg.mshr_num = 2;
  init_env(env, cfg);

  const uint32_t line_a = 0x000;
  const uint32_t line_b = 0x080;
  const uint32_t read_a = line_a + 8;
  write_memory_line(line_a, 0x1000);
  write_memory_line(line_b, 0x2000);

  auto make_line = [](uint32_t base_word) {
    WideWriteData_t wdata;
    wdata.clear();
    for (uint32_t i = 0; i < 16; ++i) {
      wdata[i] = base_word + i;
    }
    return wdata;
  };

  WideWriteStrb_t full_strobe;
  full_strobe.clear();
  for (uint32_t i = 0; i < 64; ++i) {
    full_strobe.set(i, true);
  }

  if (!issue_write(env, MASTER_DCACHE_W, line_a, make_line(0xA000), full_strobe, 63,
                   0x31, false) ||
      !wait_write_resp(env, MASTER_DCACHE_W, 0x31)) {
    std::printf("FAIL: write line A failed\n");
    return false;
  }
  if (!issue_write(env, MASTER_DCACHE_W, line_b, make_line(0xB000), full_strobe, 63,
                   0x32, false) ||
      !wait_write_resp(env, MASTER_DCACHE_W, 0x32)) {
    std::printf("FAIL: write line B failed\n");
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_DCACHE_R, read_a, 15, 0x33, false) ||
      !wait_read_resp(env, MASTER_DCACHE_R, 0x33, 0xA002, 0xA003)) {
    std::printf("FAIL: read line A after same-set write failed\n");
    return false;
  }
  if (!env.ar_events.empty()) {
    std::printf("FAIL: line A should remain resident after line B valid update, got %zu DDR ARs\n",
                env.ar_events.size());
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_partial_cacheable_write_miss_refill_merge_and_dirty_eviction() {
  std::printf("=== AXI4 LLC Integration Test 4b: partial write miss refills then writes back dirty victim ===\n");

  Axi4LlcTestEnv env;
  AXI_LLCConfig cfg = make_small_llc_config();
  cfg.size_bytes = 256;
  cfg.line_bytes = 64;
  cfg.ways = 2;
  cfg.mshr_num = 2;
  init_env(env, cfg);

  const uint32_t line_a = 0x000;
  const uint32_t line_b = 0x080;
  const uint32_t line_c = 0x100;
  const uint32_t write_addr = line_a + 8;
  const uint32_t write_value = 0xDEADBEEF;

  write_memory_line(line_a, 0x1000);
  write_memory_line(line_b, 0x2000);
  write_memory_line(line_c, 0x3000);

  WideWriteData_t wdata;
  wdata.clear();
  wdata[0] = write_value;
  WideWriteStrb_t wstrb;
  wstrb.clear();
  for (uint32_t i = 0; i < 4; ++i) {
    wstrb.set(i, true);
  }

  if (!issue_write(env, MASTER_DCACHE_W, write_addr, wdata, wstrb, 3, 0x41, false)) {
    std::printf("FAIL: partial cacheable write miss not accepted\n");
    return false;
  }
  if (!wait_write_resp(env, MASTER_DCACHE_W, 0x41)) {
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_ICACHE, line_a, 15, 0x42, false)) {
    std::printf("FAIL: readback after partial write miss not accepted\n");
    return false;
  }
  if (!wait_read_resp(env, MASTER_ICACHE, 0x42, 0x1000, 0x1001)) {
    return false;
  }
  if (!env.ar_events.empty()) {
    std::printf("FAIL: resident line should serve readback without DDR AR, got %zu\n",
                env.ar_events.size());
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_DCACHE_R, line_b, 15, 0x43, false)) {
    std::printf("FAIL: same-set fill line_b not accepted\n");
    return false;
  }
  if (!wait_read_resp(env, MASTER_DCACHE_R, 0x43, 0x2000, 0x2001)) {
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_DCACHE_R, line_c, 15, 0x44, false)) {
    std::printf("FAIL: eviction fill line_c not accepted\n");
    return false;
  }
  if (!wait_read_resp(env, MASTER_DCACHE_R, 0x44, 0x3000, 0x3001)) {
    return false;
  }

  if (!wait_mem_word(env, line_a + 8, write_value)) {
    std::printf("FAIL: dirty victim writeback did not reach backing memory got=0x%x\n",
                read_mem_word(line_a + 8));
    return false;
  }
  if (read_mem_word(line_a + 0) != 0x1000 || read_mem_word(line_a + 4) != 0x1001 ||
      read_mem_word(line_a + 8) != write_value || read_mem_word(line_a + 12) != 0x1003) {
    std::printf("FAIL: dirty victim writeback corrupted backing memory w0=0x%x w1=0x%x w2=0x%x w3=0x%x\n",
                read_mem_word(line_a + 0), read_mem_word(line_a + 4),
                read_mem_word(line_a + 8), read_mem_word(line_a + 12));
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_bypass_read_sees_latest_after_cacheable_write() {
  std::printf("=== AXI4 LLC Integration Test 5: bypass read sees latest after cacheable write ===\n");

  Axi4LlcTestEnv env;
  init_env(env);

  const uint32_t line = 0x40;
  const uint32_t addr = line + 0x10;
  write_memory_line(line, 0x6400);

  WideWriteData_t wdata;
  wdata.clear();
  for (uint32_t i = 0; i < 16; ++i) {
    wdata[i] = 0x7700 + i;
  }
  WideWriteStrb_t wstrb;
  wstrb.clear();
  for (uint32_t i = 0; i < 64; ++i) {
    wstrb.set(i, true);
  }

  if (!issue_write(env, MASTER_DCACHE_W, line, wdata, wstrb, 63, 0x21, false) ||
      !wait_write_resp(env, MASTER_DCACHE_W, 0x21)) {
    std::printf("FAIL: cacheable write before bypass read failed\n");
    return false;
  }

  if (!issue_read(env, MASTER_UNCORE_LSU_R, addr, 15, 0x22, true) ||
      !wait_read_resp(env, MASTER_UNCORE_LSU_R, 0x22, 0x7704, 0x7705)) {
    std::printf("FAIL: bypass read did not see latest line contents\n");
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_bypass_write_hit_updates_resident_line() {
  std::printf("=== AXI4 LLC Integration Test 6: bypass write hit updates resident line ===\n");

  Axi4LlcTestEnv env;
  init_env(env);

  const uint32_t line = 0x400;
  const uint32_t addr = line + 0x10;
  const uint32_t write_value = 0x8A04DEAD;
  write_memory_line(line, 0x8100);

  if (!issue_read(env, MASTER_ICACHE, addr, 15, 0x31, false) ||
      !wait_read_resp(env, MASTER_ICACHE, 0x31, 0x8104, 0x8105)) {
    std::printf("FAIL: resident-line priming read failed\n");
    return false;
  }

  WideWriteData_t wdata;
  wdata.clear();
  wdata[0] = write_value;
  WideWriteStrb_t wstrb;
  wstrb.clear();
  for (uint32_t i = 0; i < 4; ++i) {
    wstrb.set(i, true);
  }

  if (!issue_write(env, MASTER_UNCORE_LSU_W, addr, wdata, wstrb, 3, 0x32, true) ||
      !wait_write_resp(env, MASTER_UNCORE_LSU_W, 0x32)) {
    std::printf("FAIL: bypass write-hit request failed\n");
    return false;
  }
  if (read_mem_word(addr) != write_value) {
    std::printf("FAIL: bypass write-hit did not update memory got=0x%08x\n",
                read_mem_word(addr));
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_DCACHE_R, addr, 15, 0x33, false) ||
      !wait_read_resp(env, MASTER_DCACHE_R, 0x33, write_value, 0x8105)) {
    std::printf("FAIL: cacheable read after bypass write-hit failed\n");
    return false;
  }
  if (!env.ar_events.empty()) {
    std::printf("FAIL: cacheable read after bypass write-hit should not issue DDR AR, got %zu\n",
                env.ar_events.size());
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_mode2_direct_mapped_local_window_stays_off_ddr() {
  std::printf("=== AXI4 LLC Integration Test 6c: mode2 mapped window stays local ===\n");

  Axi4LlcTestEnv env;
  env.interconnect.mode = 2;
  env.interconnect.llc_mapped_offset = 0x1000;
  init_env(env, make_mode2_llc_config());

  const uint32_t mapped_line = 0x1000 + 0x200;
  const uint32_t mapped_addr = mapped_line + 8;
  write_memory_line(mapped_line, 0x9000);

  env.ar_events.clear();
  if (!issue_read(env, MASTER_ICACHE, mapped_addr, 15, 0x34, false) ||
      !wait_read_resp(env, MASTER_ICACHE, 0x34, 0x0, 0x0)) {
    std::printf("FAIL: mode2 cold read did not return zeroed local contents\n");
    return false;
  }
  cycle_outputs(env);
  cycle_inputs(env);
  if (!env.ar_events.empty()) {
    std::printf("FAIL: mode2 cold read should not issue DDR AR, got %zu\n",
                env.ar_events.size());
    return false;
  }
  if (read_mem_word(mapped_addr) != 0x9002) {
    std::printf("FAIL: mode2 cold read should not touch backing memory got=0x%08x\n",
                read_mem_word(mapped_addr));
    return false;
  }

  WideWriteData_t wdata;
  wdata.clear();
  for (uint32_t i = 0; i < 16; ++i) {
    wdata[i] = 0xD000 + i;
  }
  WideWriteStrb_t wstrb;
  wstrb.clear();
  for (uint32_t i = 0; i < 64; ++i) {
    wstrb.set(i, true);
  }

  env.ar_events.clear();
  env.aw_events.clear();
  if (!issue_write(env, MASTER_DCACHE_W, mapped_line, wdata, wstrb, 63, 0x35,
                   false) ||
      !wait_write_resp(env, MASTER_DCACHE_W, 0x35)) {
    std::printf("FAIL: mode2 local write failed\n");
    return false;
  }
  cycle_outputs(env);
  cycle_inputs(env);
  if (!env.ar_events.empty() || !env.aw_events.empty()) {
    std::printf("FAIL: mode2 local write should not issue DDR traffic ar=%zu aw=%zu\n",
                env.ar_events.size(), env.aw_events.size());
    return false;
  }
  if (read_mem_word(mapped_line) != 0x9000 || read_mem_word(mapped_addr) != 0x9002) {
    std::printf("FAIL: mode2 local write should not modify backing memory w0=0x%08x w2=0x%08x\n",
                read_mem_word(mapped_line), read_mem_word(mapped_addr));
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_DCACHE_R, mapped_addr, 15, 0x36, false) ||
      !wait_read_resp(env, MASTER_DCACHE_R, 0x36, 0xD002, 0xD003)) {
    std::printf("FAIL: mode2 reread did not observe local write data\n");
    return false;
  }
  if (!env.ar_events.empty()) {
    std::printf("FAIL: mode2 reread should still stay off DDR, got %zu ARs\n",
                env.ar_events.size());
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_mode2_window_outside_still_uses_ddr() {
  std::printf("=== AXI4 LLC Integration Test 6d: mode2 outside-window narrow read uses aligned DDR beat ===\n");

  Axi4LlcTestEnv env;
  env.interconnect.mode = 2;
  env.interconnect.llc_mapped_offset = 0x1000;
  init_env(env, make_mode2_llc_config());

  const uint32_t outside_line = 0x400;
  const uint32_t outside_addr = outside_line + 8;
  write_memory_line(outside_line, 0xA200);

  env.ar_events.clear();
  if (!issue_read(env, MASTER_UNCORE_LSU_R, outside_addr, 3, 0x37, false) ||
      !wait_read_resp(env, MASTER_UNCORE_LSU_R, 0x37, 0xA202, 0xA203)) {
    std::printf("FAIL: mode2 outside-window read failed\n");
    return false;
  }
  if (env.ar_events.size() != 1) {
    std::printf("FAIL: mode2 outside-window read should issue one DDR AR, got %zu\n",
                env.ar_events.size());
    return false;
  }
  if (env.ar_events[0].addr != (outside_addr & ~0x1Fu) ||
      env.ar_events[0].len != 0) {
    std::printf("FAIL: mode2 outside-window read should issue one aligned 32B beat addr=0x%08x len=%u\n",
                env.ar_events[0].addr, static_cast<unsigned>(env.ar_events[0].len));
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_mode2_window_outside_narrow_write_uses_aligned_ddr_beat() {
  std::printf("=== AXI4 LLC Integration Test 6e: mode2 outside-window narrow write uses aligned DDR beat ===\n");

  Axi4LlcTestEnv env;
  env.interconnect.mode = 2;
  env.interconnect.llc_mapped_offset = 0x1000;
  init_env(env, make_mode2_llc_config());

  const uint32_t outside_line = 0x400;
  const uint32_t outside_addr = outside_line + 12;
  write_memory_line(outside_line, 0xB400);

  WideWriteData_t wdata;
  wdata.clear();
  wdata[0] = 0xCAFEBABE;
  WideWriteStrb_t wstrb;
  wstrb.clear();
  for (uint32_t byte = 0; byte < 4; ++byte) {
    wstrb.set(byte, true);
  }

  env.aw_events.clear();
  if (!issue_write(env, MASTER_UNCORE_LSU_W, outside_addr, wdata, wstrb, 3, 0x38,
                   false) ||
      !wait_write_resp(env, MASTER_UNCORE_LSU_W, 0x38)) {
    std::printf("FAIL: mode2 outside-window narrow write failed\n");
    return false;
  }
  if (env.aw_events.size() != 1) {
    std::printf("FAIL: mode2 outside-window narrow write should issue one DDR AW, got %zu\n",
                env.aw_events.size());
    return false;
  }
  if (env.aw_events[0].addr != (outside_addr & ~0x1Fu) ||
      env.aw_events[0].len != 0) {
    std::printf("FAIL: mode2 outside-window narrow write should issue one aligned 32B beat addr=0x%08x len=%u\n",
                env.aw_events[0].addr, static_cast<unsigned>(env.aw_events[0].len));
    return false;
  }
  if (read_mem_word(outside_line + 8) != 0xB402 ||
      read_mem_word(outside_addr) != 0xCAFEBABE) {
    std::printf("FAIL: mode2 outside-window narrow write data mismatch prev=0x%08x now=0x%08x\n",
                read_mem_word(outside_line + 8), read_mem_word(outside_addr));
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_mode2_window_outside_cross_beat_read_falls_back_to_line() {
  std::printf("=== AXI4 LLC Integration Test 6f: mode2 outside-window cross-beat read falls back to 64B line ===\n");

  Axi4LlcTestEnv env;
  env.interconnect.mode = 2;
  env.interconnect.llc_mapped_offset = 0x1000;
  init_env(env, make_mode2_llc_config());

  const uint32_t outside_line = 0x480;
  const uint32_t outside_addr = outside_line + 28;
  write_memory_line(outside_line, 0xC200);

  env.ar_events.clear();
  if (!issue_read(env, MASTER_UNCORE_LSU_R, outside_addr, 7, 0x39, false) ||
      !wait_read_resp(env, MASTER_UNCORE_LSU_R, 0x39, 0xC207, 0xC208)) {
    std::printf("FAIL: mode2 outside-window cross-beat read failed\n");
    return false;
  }
  if (env.ar_events.size() != 1) {
    std::printf("FAIL: mode2 outside-window cross-beat read should issue one DDR AR, got %zu\n",
                env.ar_events.size());
    return false;
  }
  if (env.ar_events[0].addr != outside_line || env.ar_events[0].len != 1) {
    std::printf("FAIL: mode2 outside-window cross-beat read should fall back to aligned 64B line addr=0x%08x len=%u\n",
                env.ar_events[0].addr, static_cast<unsigned>(env.ar_events[0].len));
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_mode2_window_outside_cross_beat_write_falls_back_to_line() {
  std::printf("=== AXI4 LLC Integration Test 6g: mode2 outside-window cross-beat write falls back to 64B line ===\n");

  Axi4LlcTestEnv env;
  env.interconnect.mode = 2;
  env.interconnect.llc_mapped_offset = 0x1000;
  init_env(env, make_mode2_llc_config());

  const uint32_t outside_line = 0x480;
  const uint32_t outside_addr = outside_line + 28;
  write_memory_line(outside_line, 0xD400);

  WideWriteData_t wdata;
  wdata.clear();
  wdata[0] = 0x11111111;
  wdata[1] = 0x22222222;
  WideWriteStrb_t wstrb;
  wstrb.clear();
  for (uint32_t byte = 0; byte < 8; ++byte) {
    wstrb.set(byte, true);
  }

  env.aw_events.clear();
  if (!issue_write(env, MASTER_UNCORE_LSU_W, outside_addr, wdata, wstrb, 7, 0x3A,
                   false) ||
      !wait_write_resp(env, MASTER_UNCORE_LSU_W, 0x3A)) {
    std::printf("FAIL: mode2 outside-window cross-beat write failed\n");
    return false;
  }
  if (env.aw_events.size() != 1) {
    std::printf("FAIL: mode2 outside-window cross-beat write should issue one DDR AW, got %zu\n",
                env.aw_events.size());
    return false;
  }
  if (env.aw_events[0].addr != outside_line || env.aw_events[0].len != 1) {
    std::printf("FAIL: mode2 outside-window cross-beat write should fall back to aligned 64B line addr=0x%08x len=%u\n",
                env.aw_events[0].addr, static_cast<unsigned>(env.aw_events[0].len));
    return false;
  }
  if (read_mem_word(outside_addr) != 0x11111111 ||
      read_mem_word(outside_line + 32) != 0x22222222) {
    std::printf("FAIL: mode2 outside-window cross-beat write data mismatch w7=0x%08x w8=0x%08x\n",
                read_mem_word(outside_addr), read_mem_word(outside_line + 32));
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_bypass_write_hit_preserves_dirty_on_eviction() {
  std::printf("=== AXI4 LLC Integration Test 6b: bypass write hit preserves dirty on eviction ===\n");

  Axi4LlcTestEnv env;
  AXI_LLCConfig cfg = make_small_llc_config();
  cfg.size_bytes = 256;
  cfg.line_bytes = 64;
  cfg.ways = 2;
  cfg.mshr_num = 2;
  init_env(env, cfg);

  const uint32_t line_a = 0x000;
  const uint32_t line_b = 0x080;
  const uint32_t line_c = 0x100;
  const uint32_t addr_a = line_a + 0x10;
  const uint32_t write_value = 0x8B04D00D;
  write_memory_line(line_a, 0x7100);
  write_memory_line(line_b, 0x7200);
  write_memory_line(line_c, 0x7300);

  const WideWriteStrb_t full_strobe = make_full_write_strobe();
  if (!issue_write(env, MASTER_DCACHE_W, line_a, make_line_write_data(0xA100), full_strobe,
                   63, 0x36, false) ||
      !wait_write_resp(env, MASTER_DCACHE_W, 0x36)) {
    std::printf("FAIL: cacheable write for line A failed\n");
    return false;
  }

  WideWriteData_t subline_wdata;
  subline_wdata.clear();
  subline_wdata[0] = write_value;
  WideWriteStrb_t subline_wstrb;
  subline_wstrb.clear();
  for (uint32_t i = 0; i < 4; ++i) {
    subline_wstrb.set(i, true);
  }
  if (!issue_write(env, MASTER_UNCORE_LSU_W, addr_a, subline_wdata, subline_wstrb, 3,
                   0x37, true) ||
      !wait_write_resp(env, MASTER_UNCORE_LSU_W, 0x37)) {
    std::printf("FAIL: bypass write-hit on dirty line A failed\n");
    return false;
  }

  if (!issue_write(env, MASTER_DCACHE_W, line_b, make_line_write_data(0xB200), full_strobe,
                   63, 0x38, false) ||
      !wait_write_resp(env, MASTER_DCACHE_W, 0x38)) {
    std::printf("FAIL: cacheable write for line B failed\n");
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_ICACHE, line_c + 8, 15, 0x39, false) ||
      !wait_read_resp(env, MASTER_ICACHE, 0x39, 0x7302, 0x7303)) {
    std::printf("FAIL: eviction-triggering read for line C failed\n");
    return false;
  }

  if (!wait_mem_word(env, line_a, 0xA100)) {
    std::printf("FAIL: evicted dirty line A writeback did not reach memory mem0=0x%08x\n",
                read_mem_word(line_a));
    return false;
  }
  if (read_mem_word(line_a) != 0xA100 || read_mem_word(addr_a) != write_value) {
    std::printf("FAIL: evicted dirty line A did not write back latest data mem0=0x%08x mem4=0x%08x\n",
                read_mem_word(line_a), read_mem_word(addr_a));
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_DCACHE_R, addr_a, 15, 0x3A, false) ||
      !wait_read_resp(env, MASTER_DCACHE_R, 0x3A, write_value, 0xA105)) {
    std::printf("FAIL: reread of evicted line A failed\n");
    return false;
  }
  if (env.ar_events.size() != 1) {
    std::printf("FAIL: reread of evicted line A should miss once, got %zu\n",
                env.ar_events.size());
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_invalidate_all_drops_stale_refill_install() {
  std::printf("=== AXI4 LLC Integration Test 7: invalidate_all drops stale refill install ===\n");

  Axi4LlcTestEnv env;
  init_env(env);

  const uint32_t line = 0x500;
  const uint32_t addr = line + 0x10;
  write_memory_line(line, 0x9100);

  if (!issue_read(env, MASTER_ICACHE, addr, 15, 0x41, false)) {
    std::printf("FAIL: first cacheable read not accepted\n");
    return false;
  }

  int timeout = sim_ddr::SIM_DDR_LATENCY * 20;
  while (env.ar_events.empty() && timeout-- > 0) {
    cycle_outputs(env);
    cycle_inputs(env);
  }
  if (env.ar_events.size() != 1) {
    std::printf("FAIL: expected one outstanding DDR AR before invalidate_all, got %zu\n",
                env.ar_events.size());
    return false;
  }

  env.interconnect.set_llc_invalidate_all(true);
  cycle_outputs(env);
  cycle_inputs(env);
  env.interconnect.set_llc_invalidate_all(false);

  if (!wait_read_resp(env, MASTER_ICACHE, 0x41, 0x9104, 0x9105)) {
    std::printf("FAIL: demand response after invalidate_all did not complete\n");
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_DCACHE_R, addr, 15, 0x42, false)) {
    std::printf("FAIL: second cacheable read not accepted\n");
    return false;
  }
  if (!wait_read_resp(env, MASTER_DCACHE_R, 0x42, 0x9104, 0x9105)) {
    std::printf("FAIL: second cacheable read after invalidate_all failed\n");
    return false;
  }
  if (env.ar_events.size() != 1) {
    std::printf("FAIL: second cacheable read should miss LLC once after stale refill drop, got %zu\n",
                env.ar_events.size());
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_invalidate_all_after_dirty_cacheable_write_stalls_and_preserves_latest() {
  std::printf("=== AXI4 LLC Integration Test 7b: invalidate_all stalls on dirty resident and preserves latest ===\n");

  Axi4LlcTestEnv env;
  init_env(env);

  const uint32_t line_addr = 0x2c00;
  write_memory_line(line_addr, 0x6600);

  const WideWriteData_t wdata = make_line_write_data(0x7700);
  const WideWriteStrb_t wstrb = make_full_write_strobe();
  if (!issue_write(env, MASTER_DCACHE_W, line_addr, wdata, wstrb, 63, 0x61, false)) {
    std::printf("FAIL: dirty cacheable write was not accepted\n");
    return false;
  }
  if (!wait_write_resp(env, MASTER_DCACHE_W, 0x61)) {
    return false;
  }
  if (read_mem_word(line_addr) != 0x6600) {
    std::printf("FAIL: backing memory should still hold old value before eviction\n");
    return false;
  }

  bool accepted = false;
  for (int i = 0; i < 4; ++i) {
    env.interconnect.set_llc_invalidate_all(true);
    cycle_outputs(env);
    accepted = accepted || env.interconnect.llc_invalidate_all_accepted();
    cycle_inputs(env);
  }
  env.interconnect.set_llc_invalidate_all(false);
  if (accepted) {
    std::printf("FAIL: invalidate_all should stall while dirty resident line exists\n");
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_ICACHE, line_addr, 15, 0x62, false)) {
    std::printf("FAIL: reread after stalled invalidate_all not accepted\n");
    return false;
  }
  if (!wait_read_resp(env, MASTER_ICACHE, 0x62, 0x7700, 0x7701)) {
    return false;
  }
  if (!env.ar_events.empty()) {
    std::printf("FAIL: dirty resident reread should still hit LLC, got ar_count=%zu\n",
                env.ar_events.size());
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_victim_writeback_maintenance_and_miss_interlock() {
  std::printf("=== AXI4 LLC Integration Test 8: victim writeback + maintenance + miss interlock ===\n");

  Axi4LlcTestEnv env;
  AXI_LLCConfig cfg = make_small_llc_config();
  cfg.size_bytes = 128;
  cfg.line_bytes = 64;
  cfg.ways = 1;
  cfg.mshr_num = 2;
  init_env(env, cfg);

  const uint32_t line_a = 0x000;
  const uint32_t line_b = 0x080;
  const uint32_t addr_a = line_a + 0x8;
  const uint32_t addr_b = line_b + 0x8;
  write_memory_line(line_a, 0x1000);
  write_memory_line(line_b, 0x2000);

  WideWriteData_t wdata_a;
  wdata_a.clear();
  for (uint32_t i = 0; i < 16; ++i) {
    wdata_a[i] = 0xA000 + i;
  }
  WideWriteStrb_t full_strobe;
  full_strobe.clear();
  for (uint32_t i = 0; i < 64; ++i) {
    full_strobe.set(i, true);
  }

  if (!issue_write(env, MASTER_DCACHE_W, line_a, wdata_a, full_strobe, 63, 0x51, false) ||
      !wait_write_resp(env, MASTER_DCACHE_W, 0x51)) {
    std::printf("FAIL: dirty line A setup write failed\n");
    return false;
  }

  if (!issue_read(env, MASTER_ICACHE, addr_b, 15, 0x52, false)) {
    std::printf("FAIL: demand miss on line B not accepted\n");
    return false;
  }

  int timeout = sim_ddr::SIM_DDR_LATENCY * 40;
  bool saw_ar_b = false;
  bool saw_aw_a = false;
  while (timeout-- > 0 && (!saw_ar_b || !saw_aw_a)) {
    cycle_outputs(env);
    saw_ar_b = saw_ar_b || env.interconnect.axi_io.ar.arvalid;
    saw_aw_a = saw_aw_a || env.interconnect.axi_io.aw.awvalid;
    cycle_inputs(env);
  }

  if (!saw_ar_b || !saw_aw_a) {
    std::printf("FAIL: did not observe both miss read and victim writeback traffic ar=%d aw=%d\n",
                static_cast<int>(saw_ar_b), static_cast<int>(saw_aw_a));
    return false;
  }

  env.interconnect.set_llc_invalidate_line(true, line_b);
  cycle_outputs(env);
  if (env.interconnect.llc_invalidate_line_accepted()) {
    std::printf("FAIL: line-B invalidate accepted while line-B miss still inflight\n");
    return false;
  }
  cycle_inputs(env);
  env.interconnect.set_llc_invalidate_line(false, 0);

  if (!wait_read_resp(env, MASTER_ICACHE, 0x52, 0x2002, 0x2003)) {
    std::printf("FAIL: demand miss on line B did not complete correctly\n");
    return false;
  }

  timeout = 100;
  bool invalidate_accepted = false;
  while (timeout-- > 0) {
    env.interconnect.set_llc_invalidate_line(true, line_b);
    cycle_outputs(env);
    if (env.interconnect.llc_invalidate_line_accepted()) {
      invalidate_accepted = true;
      cycle_inputs(env);
      env.interconnect.set_llc_invalidate_line(false, 0);
      break;
    }
    cycle_inputs(env);
    env.interconnect.set_llc_invalidate_line(false, 0);
  }
  if (!invalidate_accepted) {
    std::printf("FAIL: line-B invalidate was never accepted after miss completion\n");
    return false;
  }

  if (read_mem_word(addr_a) != 0xA002) {
    std::printf("FAIL: dirty victim writeback did not reach memory got=0x%08x\n",
                read_mem_word(addr_a));
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_DCACHE_R, addr_b, 15, 0x53, false) ||
      !wait_read_resp(env, MASTER_DCACHE_R, 0x53, 0x2002, 0x2003)) {
    std::printf("FAIL: post-maintenance line-B reread failed\n");
    return false;
  }
  if (env.ar_events.size() != 1) {
    std::printf("FAIL: post-maintenance line-B reread should miss LLC once, got %zu\n",
                env.ar_events.size());
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_DCACHE_R, addr_a, 15, 0x54, false) ||
      !wait_read_resp(env, MASTER_DCACHE_R, 0x54, 0xA002, 0xA003)) {
    std::printf("FAIL: line-A reread after victim writeback failed\n");
    return false;
  }
  if (env.ar_events.size() != 1) {
    std::printf("FAIL: line-A reread after eviction should miss once, got %zu\n",
                env.ar_events.size());
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_invalidate_all_waits_for_dirty_victim_writeback() {
  std::printf("=== AXI4 LLC Integration Test 8b: invalidate_all waits for dirty victim writeback ===\n");

  Axi4LlcTestEnv env;
  AXI_LLCConfig cfg = make_small_llc_config();
  cfg.size_bytes = 128;
  cfg.line_bytes = 64;
  cfg.ways = 1;
  cfg.mshr_num = 2;
  init_env(env, cfg);

  const uint32_t line_a = 0x000;
  const uint32_t line_b = 0x080;

  write_memory_line(line_a, 0x1000);
  write_memory_line(line_b, 0x2000);

  if (!issue_write(env, MASTER_DCACHE_W, line_a, make_line_write_data(0x3000),
                   make_full_write_strobe(), 63, 0x71, false)) {
    std::printf("FAIL: initial dirty line write not accepted\n");
    return false;
  }
  if (!wait_write_resp(env, MASTER_DCACHE_W, 0x71)) {
    return false;
  }

  if (!issue_read(env, MASTER_ICACHE, line_b, 15, 0x72, false)) {
    std::printf("FAIL: eviction-causing read not accepted\n");
    return false;
  }

  bool saw_read_resp = false;
  bool saw_invalidate_accept = false;
  int timeout = sim_ddr::SIM_DDR_LATENCY * 120;
  while (timeout-- > 0) {
    env.interconnect.set_llc_invalidate_all(true);
    cycle_outputs(env);
    if (env.interconnect.llc_invalidate_all_accepted()) {
      saw_invalidate_accept = true;
      if (read_mem_word(line_a) != 0x3000 || read_mem_word(line_a + 4) != 0x3001) {
        std::printf("FAIL: invalidate_all accepted before dirty victim writeback reached memory\n");
        return false;
      }
    }
    auto &resp = env.interconnect.read_ports[MASTER_ICACHE].resp;
    if (resp.valid) {
      if (resp.id != 0x72 || resp.data[0] != 0x2000 || resp.data[1] != 0x2001) {
        std::printf("FAIL: eviction-causing read resp mismatch id=%u d0=0x%x d1=0x%x\n",
                    resp.id, resp.data[0], resp.data[1]);
        return false;
      }
      resp.ready = true;
      saw_read_resp = true;
    }
    cycle_inputs(env);
    if (saw_read_resp && saw_invalidate_accept) {
      break;
    }
  }
  env.interconnect.set_llc_invalidate_all(false);
  if (!saw_read_resp || !saw_invalidate_accept) {
    std::printf("FAIL: dirty victim + invalidate_all interlock timed out read=%d accept=%d\n",
                static_cast<int>(saw_read_resp), static_cast<int>(saw_invalidate_accept));
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_ICACHE, line_b, 15, 0x73, false)) {
    std::printf("FAIL: reread after accepted invalidate_all not accepted\n");
    return false;
  }
  if (!wait_read_resp(env, MASTER_ICACHE, 0x73, 0x2000, 0x2001)) {
    return false;
  }
  if (env.ar_events.empty()) {
    std::printf("FAIL: reread after accepted invalidate_all should miss once\n");
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_invalidate_all_pending_blocks_new_upstream_requests_but_drains_captured_requests() {
  std::printf("=== AXI4 LLC Integration Test 8c: invalidate_all pending blocks new upstream requests but drains captured requests ===\n");

  Axi4LlcTestEnv env;
  AXI_LLCConfig cfg = make_small_llc_config();
  cfg.size_bytes = 128;
  cfg.line_bytes = 64;
  cfg.ways = 1;
  cfg.mshr_num = 2;
  init_env(env, cfg);

  const uint32_t line_a = 0x000;
  const uint32_t line_b = 0x080;
  const uint32_t line_c = 0x100;

  write_memory_line(line_a, 0x1000);
  write_memory_line(line_b, 0x2000);
  write_memory_line(line_c, 0x3000);

  if (!issue_write(env, MASTER_DCACHE_W, line_a, make_line_write_data(0x4000),
                   make_full_write_strobe(), 63, 0x74, false) ||
      !wait_write_resp(env, MASTER_DCACHE_W, 0x74)) {
    std::printf("FAIL: initial dirty line setup failed\n");
    return false;
  }

  if (!issue_read(env, MASTER_ICACHE, line_b, 15, 0x75, false)) {
    std::printf("FAIL: eviction-causing read not accepted\n");
    return false;
  }

  bool saw_aw_a = false;
  bool saw_ar_b = false;
  int timeout = sim_ddr::SIM_DDR_LATENCY * 40;
  while (timeout-- > 0 && (!saw_aw_a || !saw_ar_b)) {
    cycle_outputs(env);
    saw_aw_a = saw_aw_a || env.interconnect.axi_io.aw.awvalid;
    saw_ar_b = saw_ar_b || env.interconnect.axi_io.ar.arvalid;
    cycle_inputs(env);
  }
  if (!saw_aw_a || !saw_ar_b) {
    std::printf("FAIL: did not observe captured dirty-victim traffic before invalidate_all pending ar=%d aw=%d\n",
                static_cast<int>(saw_ar_b), static_cast<int>(saw_aw_a));
    return false;
  }

  bool late_read_ready_while_pending = false;
  bool invalidate_accept = false;
  timeout = sim_ddr::SIM_DDR_LATENCY * 120;
  while (timeout-- > 0) {
    env.interconnect.set_llc_invalidate_all(true);
    cycle_outputs(env);

    auto &late_rp = env.interconnect.read_ports[MASTER_DCACHE_R];
    late_rp.req.valid = true;
    late_rp.req.addr = line_c;
    late_rp.req.total_size = 15;
    late_rp.req.id = 0x76;
    late_rp.req.bypass = false;
    if (late_rp.req.ready) {
      late_read_ready_while_pending = true;
    }

    if (env.interconnect.llc_invalidate_all_accepted()) {
      invalidate_accept = true;
    }

    auto &resp = env.interconnect.read_ports[MASTER_ICACHE].resp;
    if (resp.valid) {
      if (resp.id != 0x75 || resp.data[0] != 0x2000 || resp.data[1] != 0x2001) {
        std::printf("FAIL: drain read resp mismatch id=%u d0=0x%x d1=0x%x\n",
                    resp.id, resp.data[0], resp.data[1]);
        return false;
      }
      resp.ready = true;
    }

    cycle_inputs(env);
    if (invalidate_accept) {
      break;
    }
  }
  env.interconnect.set_llc_invalidate_all(false);

  if (late_read_ready_while_pending) {
    std::printf("FAIL: new upstream request should be blocked while invalidate_all is pending\n");
    return false;
  }
  if (!invalidate_accept) {
    std::printf("FAIL: invalidate_all was never accepted after draining captured traffic\n");
    return false;
  }
  if (read_mem_word(line_a) != 0x4000 || read_mem_word(line_a + 4) != 0x4001) {
    std::printf("FAIL: dirty victim data did not drain to memory before invalidate_all accept\n");
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_public_api_invalidate_all_hold_until_accept_contract() {
  std::printf("=== AXI4 LLC Integration Test 8d: public API invalidate_all must be held until accepted ===\n");

  Axi4LlcTestEnv env;
  AXI_LLCConfig cfg = make_small_llc_config();
  cfg.size_bytes = 128;
  cfg.line_bytes = 64;
  cfg.ways = 1;
  cfg.mshr_num = 2;
  init_env(env, cfg);

  const uint32_t line_a = 0x000;
  const uint32_t line_b = 0x080;
  write_memory_line(line_a, 0x1000);
  write_memory_line(line_b, 0x2000);

  if (!issue_write(env, MASTER_DCACHE_W, line_a, make_line_write_data(0x5000),
                   make_full_write_strobe(), 63, 0x77, false) ||
      !wait_write_resp(env, MASTER_DCACHE_W, 0x77)) {
    std::printf("FAIL: initial dirty line setup failed\n");
    return false;
  }

  if (!issue_read(env, MASTER_ICACHE, line_b, 15, 0x78, false)) {
    std::printf("FAIL: drain read not accepted\n");
    return false;
  }

  bool accepted = false;
  bool saw_read_resp = false;
  int timeout = sim_ddr::SIM_DDR_LATENCY * 120;
  while (timeout-- > 0) {
    env.interconnect.set_llc_invalidate_all(true);
    cycle_outputs(env);
    accepted = accepted || env.interconnect.llc_invalidate_all_accepted();
    auto &resp = env.interconnect.read_ports[MASTER_ICACHE].resp;
    if (resp.valid) {
      if (resp.id != 0x78 || resp.data[0] != 0x2000 || resp.data[1] != 0x2001) {
        std::printf("FAIL: drain read response mismatch id=%u d0=0x%x d1=0x%x\n",
                    resp.id, resp.data[0], resp.data[1]);
        return false;
      }
      resp.ready = true;
      saw_read_resp = true;
    }
    cycle_inputs(env);
    if (accepted) {
      break;
    }
  }
  env.interconnect.set_llc_invalidate_all(false);

  if (!accepted) {
    std::printf("FAIL: invalidate_all was never accepted while caller held request\n");
    return false;
  }
  if (read_mem_word(line_a) != 0x5000 || read_mem_word(line_a + 4) != 0x5001) {
    std::printf("FAIL: dirty victim writeback not visible before invalidate_all acceptance\n");
    return false;
  }
  if (saw_read_resp) {
    std::printf("PASS\n");
    return true;
  }

  timeout = sim_ddr::SIM_DDR_LATENCY * 40;
  while (timeout-- > 0) {
    cycle_outputs(env);
    auto &resp = env.interconnect.read_ports[MASTER_ICACHE].resp;
    if (resp.valid) {
      if (resp.id != 0x78 || resp.data[0] != 0x2000 || resp.data[1] != 0x2001) {
        std::printf("FAIL: post-accept drain read response mismatch id=%u d0=0x%x d1=0x%x\n",
                    resp.id, resp.data[0], resp.data[1]);
        return false;
      }
      resp.ready = true;
      cycle_inputs(env);
      std::printf("PASS\n");
      return true;
    }
    cycle_inputs(env);
  }

  std::printf("FAIL: captured clean LLC-path work never drained to upstream response\n");
  return false;
}

bool test_bypass_write_miss_does_not_allocate_line() {
  std::printf("=== AXI4 LLC Integration Test 7: bypass write miss does not allocate ===\n");

  Axi4LlcTestEnv env;
  init_env(env);

  const uint32_t line = 0x480;
  const uint32_t addr = line + 0x10;
  const uint32_t write_value = 0x9B04BEEF;
  write_memory_line(line, 0x9100);

  WideWriteData_t wdata;
  wdata.clear();
  wdata[0] = write_value;
  WideWriteStrb_t wstrb;
  wstrb.clear();
  for (uint32_t i = 0; i < 4; ++i) {
    wstrb.set(i, true);
  }

  if (!issue_write(env, MASTER_UNCORE_LSU_W, addr, wdata, wstrb, 3, 0x34, true) ||
      !wait_write_resp(env, MASTER_UNCORE_LSU_W, 0x34)) {
    std::printf("FAIL: bypass write-miss request failed\n");
    return false;
  }
  if (read_mem_word(addr) != write_value) {
    std::printf("FAIL: bypass write-miss did not update memory got=0x%08x\n",
                read_mem_word(addr));
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_ICACHE, addr, 15, 0x35, false) ||
      !wait_read_resp(env, MASTER_ICACHE, 0x35, write_value, 0x9105)) {
    std::printf("FAIL: cacheable read after bypass write-miss failed\n");
    return false;
  }
  if (env.ar_events.size() != 1) {
    std::printf("FAIL: cacheable read after bypass write-miss should issue one DDR AR, got %zu\n",
                env.ar_events.size());
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_bypass_write_miss_preserves_same_set_residents() {
  std::printf("=== AXI4 LLC Integration Test 8: bypass write miss preserves same-set residents ===\n");

  Axi4LlcTestEnv env;
  AXI_LLCConfig cfg = make_small_llc_config();
  cfg.size_bytes = 512;
  cfg.line_bytes = 64;
  cfg.ways = 2;
  cfg.mshr_num = 2;
  init_env(env, cfg);

  const uint32_t line_a = 0x000;
  const uint32_t line_b = 0x100;
  const uint32_t line_c = 0x200;
  const uint32_t addr_a = line_a + 0x10;
  const uint32_t addr_b = line_b + 0x10;
  const uint32_t addr_c = line_c + 0x10;
  const uint32_t write_value = 0xCAFE0004;
  write_memory_line(line_a, 0xA100);
  write_memory_line(line_b, 0xB100);
  write_memory_line(line_c, 0xC100);

  if (!issue_read(env, MASTER_ICACHE, addr_a, 15, 0x41, false) ||
      !wait_read_resp(env, MASTER_ICACHE, 0x41, 0xA104, 0xA105)) {
    std::printf("FAIL: fill line A failed\n");
    return false;
  }
  if (!issue_read(env, MASTER_DCACHE_R, addr_b, 15, 0x42, false) ||
      !wait_read_resp(env, MASTER_DCACHE_R, 0x42, 0xB104, 0xB105)) {
    std::printf("FAIL: fill line B failed\n");
    return false;
  }

  WideWriteData_t wdata;
  wdata.clear();
  wdata[0] = write_value;
  WideWriteStrb_t wstrb;
  wstrb.clear();
  for (uint32_t i = 0; i < 4; ++i) {
    wstrb.set(i, true);
  }
  if (!issue_write(env, MASTER_UNCORE_LSU_W, addr_c, wdata, wstrb, 3, 0x43, true) ||
      !wait_write_resp(env, MASTER_UNCORE_LSU_W, 0x43)) {
    std::printf("FAIL: bypass write miss failed\n");
    return false;
  }
  if (read_mem_word(addr_c) != write_value) {
    std::printf("FAIL: bypass write miss did not update backing memory got=0x%08x\n",
                read_mem_word(addr_c));
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_ICACHE, addr_a, 15, 0x44, false) ||
      !wait_read_resp(env, MASTER_ICACHE, 0x44, 0xA104, 0xA105)) {
    std::printf("FAIL: resident line A lost after bypass miss\n");
    return false;
  }
  if (!issue_read(env, MASTER_DCACHE_R, addr_b, 15, 0x45, false) ||
      !wait_read_resp(env, MASTER_DCACHE_R, 0x45, 0xB104, 0xB105)) {
    std::printf("FAIL: resident line B lost after bypass miss\n");
    return false;
  }
  if (!env.ar_events.empty()) {
    std::printf("FAIL: resident lines should remain in LLC after bypass miss, got %zu ARs\n",
                env.ar_events.size());
    return false;
  }

  env.ar_events.clear();
  if (!issue_read(env, MASTER_DCACHE_R, addr_c, 15, 0x46, false) ||
      !wait_read_resp(env, MASTER_DCACHE_R, 0x46, write_value, 0xC105)) {
    std::printf("FAIL: cacheable read of bypass-miss line failed\n");
    return false;
  }
  if (env.ar_events.size() != 1) {
    std::printf("FAIL: bypass-miss line should still require one DDR AR, got %zu\n",
                env.ar_events.size());
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_seeded_small_depth_stress() {
  std::printf("=== AXI4 LLC Integration Test 9: seeded small-depth stress ===\n");

  Axi4LlcTestEnv env;
  AXI_LLCConfig cfg = make_small_llc_config();
  cfg.size_bytes = 256;
  cfg.line_bytes = 64;
  cfg.ways = 2;
  cfg.mshr_num = 2;
  init_env(env, cfg);

  const std::vector<uint32_t> lines = {0x000, 0x080, 0x100, 0x180, 0x040, 0x0C0};
  std::unordered_map<uint32_t, std::vector<uint32_t>> shadow;
  for (size_t idx = 0; idx < lines.size(); ++idx) {
    const uint32_t base = 0x6000 + static_cast<uint32_t>(idx) * 0x100;
    write_memory_line(lines[idx], base);
    shadow[lines[idx]].resize(16);
    for (uint32_t word = 0; word < 16; ++word) {
      shadow[lines[idx]][word] = base + word;
    }
  }

  WideWriteStrb_t full_strobe;
  full_strobe.clear();
  for (uint32_t i = 0; i < 64; ++i) {
    full_strobe.set(i, true);
  }

  std::mt19937 rng(0xC0DEC0DEu);
  for (uint32_t step = 0; step < 60; ++step) {
    const uint32_t line = lines[rng() % lines.size()];
    const uint32_t word_idx = static_cast<uint32_t>(rng() % 14);
    const uint32_t addr = line + word_idx * 4;
    const uint32_t op = rng() % 4;

    if (op == 0 || op == 1) {
      const uint8_t master = (op == 0) ? MASTER_ICACHE : MASTER_DCACHE_R;
      const uint8_t id = static_cast<uint8_t>((step & 0xF) + 1);
      if (!issue_read(env, master, addr, 15, id, false)) {
        std::printf("FAIL: stress cacheable read issue failed step=%u master=%u addr=0x%x\n",
                    step, master, addr);
        return false;
      }
      if (!wait_read_resp(env, master, id, shadow[line][word_idx],
                          shadow[line][word_idx + 1])) {
        std::printf("FAIL: stress cacheable read resp failed step=%u addr=0x%x\n",
                    step, addr);
        return false;
      }
    } else if (op == 2) {
      const uint8_t id = static_cast<uint8_t>((step & 0xF) + 1);
      if (!issue_read(env, MASTER_UNCORE_LSU_R, addr, 15, id, true)) {
        std::printf("FAIL: stress bypass read issue failed step=%u addr=0x%x\n",
                    step, addr);
        return false;
      }
      if (!wait_read_resp(env, MASTER_UNCORE_LSU_R, id, shadow[line][word_idx],
                          shadow[line][word_idx + 1])) {
        std::printf("FAIL: stress bypass read resp failed step=%u addr=0x%x\n",
                    step, addr);
        return false;
      }
    } else {
      const uint32_t base = 0x90000000u + step * 0x100u;
      WideWriteData_t wdata;
      wdata.clear();
      for (uint32_t word = 0; word < 16; ++word) {
        const uint32_t value = base + word;
        wdata[word] = value;
        shadow[line][word] = value;
      }
      const uint8_t id = static_cast<uint8_t>((step & 0xF) + 1);
      if (!issue_write(env, MASTER_DCACHE_W, line, wdata, full_strobe, 63, id, false)) {
        std::printf("FAIL: stress cacheable write issue failed step=%u line=0x%x\n",
                    step, line);
        return false;
      }
      if (!wait_write_resp(env, MASTER_DCACHE_W, id)) {
        std::printf("FAIL: stress cacheable write resp failed step=%u line=0x%x\n",
                    step, line);
        return false;
      }
    }
  }

  uint8_t final_id = 0x40;
  for (uint32_t line : lines) {
    if (!issue_read(env, MASTER_DCACHE_R, line, 15, final_id, false)) {
      std::printf("FAIL: final sweep read issue failed line=0x%x id=%u\n", line,
                  final_id);
      return false;
    }
    if (!wait_read_resp(env, MASTER_DCACHE_R, final_id, shadow[line][0],
                        shadow[line][1])) {
      std::printf("FAIL: final sweep read resp failed line=0x%x id=%u\n", line,
                  final_id);
      return false;
    }
    ++final_id;
  }

  const auto &perf = env.interconnect.get_llc_perf_counters();
  if (perf.read_access == 0 || perf.read_miss == 0 || perf.read_hit == 0 ||
      perf.bypass_read == 0) {
    std::printf("FAIL: seeded stress perf coverage too weak access=%llu miss=%llu hit=%llu bypass=%llu\n",
                static_cast<unsigned long long>(perf.read_access),
                static_cast<unsigned long long>(perf.read_miss),
                static_cast<unsigned long long>(perf.read_hit),
                static_cast<unsigned long long>(perf.bypass_read));
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_llc_same_master_multi_write_accepts_before_first_resp() {
  std::printf("=== AXI4 LLC Integration Test 10: LLC path queues same-master writes ===\n");

  Axi4LlcTestEnv env;
  init_env(env);

  const uint32_t line0 = 0xA000;
  const uint32_t line1 = 0xA040;
  write_memory_line(line0, 0x1000);
  write_memory_line(line1, 0x2000);

  auto make_line = [](uint32_t base_word) {
    WideWriteData_t wdata;
    wdata.clear();
    for (uint32_t i = 0; i < 16; ++i) {
      wdata[i] = base_word + i;
    }
    return wdata;
  };
  WideWriteData_t wdata0 = make_line(0x3300);
  WideWriteData_t wdata1 = make_line(0x4400);
  WideWriteStrb_t full_strobe;
  full_strobe.clear();
  for (uint32_t i = 0; i < 64; ++i) {
    full_strobe.set(i, true);
  }

  bool first_issued = false;
  int timeout = 200;
  while (!first_issued && timeout-- > 0) {
    cycle_outputs(env);
    const bool ready_snapshot = env.interconnect.write_ports[MASTER_DCACHE_W].req.ready;
    auto &req = env.interconnect.write_ports[MASTER_DCACHE_W].req;
    req.valid = true;
    req.addr = line0;
    req.wdata = wdata0;
    req.wstrb = full_strobe;
    req.total_size = 63;
    req.id = 0x51;
    req.bypass = false;
    cycle_inputs(env);
    if (ready_snapshot) {
      first_issued = true;
    }
  }
  if (!first_issued) {
    std::printf("FAIL: first LLC write was not accepted\n");
    return false;
  }

  bool second_issued = false;
  timeout = 200;
  while (!second_issued && timeout-- > 0) {
    cycle_outputs(env);
    if (env.interconnect.write_ports[MASTER_DCACHE_W].resp.valid) {
      std::printf("FAIL: first LLC write response became visible before second issue\n");
      return false;
    }
    const bool ready_snapshot = env.interconnect.write_ports[MASTER_DCACHE_W].req.ready;
    auto &req = env.interconnect.write_ports[MASTER_DCACHE_W].req;
    req.valid = true;
    req.addr = line1;
    req.wdata = wdata1;
    req.wstrb = full_strobe;
    req.total_size = 63;
    req.id = 0x52;
    req.bypass = false;
    cycle_inputs(env);
    if (ready_snapshot) {
      second_issued = true;
    }
  }
  if (!second_issued) {
    std::printf("FAIL: second LLC write was not accepted before first response\n");
    return false;
  }

  if (!wait_write_resp(env, MASTER_DCACHE_W, 0x51) ||
      !wait_write_resp(env, MASTER_DCACHE_W, 0x52)) {
    std::printf("FAIL: LLC queued write responses did not complete\n");
    return false;
  }

  if (!issue_read(env, MASTER_DCACHE_R, line0, 15, 0x61, false) ||
      !wait_read_resp(env, MASTER_DCACHE_R, 0x61, 0x3300, 0x3301)) {
    std::printf("FAIL: LLC queued first write not visible to readback\n");
    return false;
  }
  if (!issue_read(env, MASTER_DCACHE_R, line1, 15, 0x62, false) ||
      !wait_read_resp(env, MASTER_DCACHE_R, 0x62, 0x4400, 0x4401)) {
    std::printf("FAIL: LLC queued second write not visible to readback\n");
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_seeded_mixed_read_write_coherence_stress() {
  std::printf("=== AXI4 LLC Integration Test 11: seeded mixed read/write coherence stress ===\n");

  Axi4LlcTestEnv env;
  AXI_LLCConfig cfg = make_small_llc_config();
  cfg.size_bytes = 256;
  cfg.line_bytes = 64;
  cfg.ways = 2;
  cfg.mshr_num = 2;
  init_env(env, cfg);

  const std::vector<uint32_t> lines = {0x000, 0x040, 0x080, 0x0C0, 0x100, 0x140};
  std::unordered_map<uint32_t, std::vector<uint32_t>> shadow;
  for (size_t idx = 0; idx < lines.size(); ++idx) {
    const uint32_t base = 0x7000 + static_cast<uint32_t>(idx) * 0x100;
    write_memory_line(lines[idx], base);
    shadow[lines[idx]].resize(16);
    for (uint32_t word = 0; word < 16; ++word) {
      shadow[lines[idx]][word] = base + word;
    }
  }

  const WideWriteStrb_t full_strobe = make_full_write_strobe();
  std::mt19937 rng(0x13579BDFu);
  for (uint32_t step = 0; step < 96; ++step) {
    const uint32_t line = lines[rng() % lines.size()];
    const uint32_t word_idx = static_cast<uint32_t>(rng() % 8) + 4;
    const uint32_t addr = line + word_idx * 4;
    const uint32_t op = rng() % 4;
    const uint8_t id = static_cast<uint8_t>((step & 0x3F) + 1);

    if (op == 0) {
      const uint8_t master = ((step & 1u) == 0u) ? MASTER_ICACHE : MASTER_DCACHE_R;
      if (!issue_read(env, master, line, 15, id, false) ||
          !wait_read_resp(env, master, id, shadow[line][0], shadow[line][1])) {
        std::printf("FAIL: mixed stress cacheable read failed step=%u master=%u addr=0x%x\n",
                    step, master, line);
        return false;
      }
      continue;
    }

    if (op == 1) {
      if (!issue_read(env, MASTER_UNCORE_LSU_R, line + 8, 15, id, true) ||
          !wait_read_resp(env, MASTER_UNCORE_LSU_R, id, shadow[line][2],
                          shadow[line][3])) {
        std::printf("FAIL: mixed stress bypass read failed step=%u addr=0x%x\n",
                    step, line + 8);
        return false;
      }
      continue;
    }

    if (op == 2) {
      const uint32_t base = 0xA0000000u + step * 0x40u;
      const WideWriteData_t wdata = make_line_write_data(base);
      if (!issue_write(env, MASTER_DCACHE_W, line, wdata, full_strobe, 63, id, false) ||
          !wait_write_resp(env, MASTER_DCACHE_W, id)) {
        std::printf("FAIL: mixed stress cacheable write failed step=%u line=0x%x\n",
                    step, line);
        return false;
      }
      for (uint32_t word = 0; word < 16; ++word) {
        shadow[line][word] = base + word;
      }
      continue;
    }

    const uint32_t write_value = 0xC0000000u + step * 0x100u + word_idx;
    WideWriteData_t wdata;
    wdata.clear();
    wdata[0] = write_value;
    WideWriteStrb_t wstrb;
    wstrb.clear();
    for (uint32_t i = 0; i < 4; ++i) {
      wstrb.set(i, true);
    }
    if (!issue_write(env, MASTER_UNCORE_LSU_W, addr, wdata, wstrb, 3, id, true) ||
        !wait_write_resp(env, MASTER_UNCORE_LSU_W, id)) {
      std::printf("FAIL: mixed stress bypass write failed step=%u addr=0x%x\n",
                  step, addr);
      return false;
    }
    shadow[line][word_idx] = write_value;
    const uint8_t verify_id = static_cast<uint8_t>(0x80 + (step & 0x1F));
    if (!issue_read(env, MASTER_DCACHE_R, addr, 15, verify_id, false) ||
        !wait_read_resp(env, MASTER_DCACHE_R, verify_id, shadow[line][word_idx],
                        shadow[line][word_idx + 1])) {
      std::printf("FAIL: mixed stress bypass-write verification failed step=%u addr=0x%x\n",
                  step, addr);
      return false;
    }
  }

  uint8_t final_id = 0x70;
  for (uint32_t line : lines) {
    if (!issue_read(env, MASTER_DCACHE_R, line, 15, final_id, false) ||
        !wait_read_resp(env, MASTER_DCACHE_R, final_id, shadow[line][0],
                        shadow[line][1])) {
      std::printf("FAIL: mixed stress final cacheable sweep failed line=0x%x id=%u\n",
                  line, final_id);
      return false;
    }
    ++final_id;
    if (!issue_read(env, MASTER_UNCORE_LSU_R, line + 8, 15, final_id, true) ||
        !wait_read_resp(env, MASTER_UNCORE_LSU_R, final_id, shadow[line][2],
                        shadow[line][3])) {
      std::printf("FAIL: mixed stress final bypass sweep failed line=0x%x id=%u\n",
                  line, final_id);
      return false;
    }
    ++final_id;
  }

  const auto &perf = env.interconnect.get_llc_perf_counters();
  if (perf.read_hit == 0 || perf.read_miss == 0 || perf.bypass_read == 0) {
    std::printf("FAIL: mixed stress perf coverage too weak hit=%llu miss=%llu bypass=%llu\n",
                static_cast<unsigned long long>(perf.read_hit),
                static_cast<unsigned long long>(perf.read_miss),
                static_cast<unsigned long long>(perf.bypass_read));
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_seeded_invalidate_all_epoch_stress() {
  std::printf("=== AXI4 LLC Integration Test 12: seeded invalidate-all epoch stress ===\n");

  Axi4LlcTestEnv env;
  init_env(env);

  const std::vector<uint32_t> lines = {0x200, 0x240, 0x280, 0x2C0};
  std::unordered_map<uint32_t, std::vector<uint32_t>> shadow;
  for (size_t idx = 0; idx < lines.size(); ++idx) {
    const uint32_t base = 0x8800 + static_cast<uint32_t>(idx) * 0x80;
    write_memory_line(lines[idx], base);
    shadow[lines[idx]].resize(16);
    for (uint32_t word = 0; word < 16; ++word) {
      shadow[lines[idx]][word] = base + word;
    }
  }

  std::mt19937 rng(0x2468ACE0u);
  for (uint32_t step = 0; step < 24; ++step) {
    const uint32_t line = lines[rng() % lines.size()];
    const uint32_t word_idx = static_cast<uint32_t>(rng() % 14);
    const uint32_t addr = line + word_idx * 4;
    const uint8_t seed_id = static_cast<uint8_t>((step & 0x1F) + 1);

    WideWriteData_t wdata;
    wdata.clear();
    const uint32_t write_value = 0xD0000000u + step * 0x20u + word_idx;
    wdata[0] = write_value;
    WideWriteStrb_t wstrb;
    wstrb.clear();
    for (uint32_t i = 0; i < 4; ++i) {
      wstrb.set(i, true);
    }
    if (!issue_write(env, MASTER_UNCORE_LSU_W, addr, wdata, wstrb, 3, seed_id, true) ||
        !wait_write_resp(env, MASTER_UNCORE_LSU_W, seed_id)) {
      std::printf("FAIL: epoch stress bypass write failed step=%u addr=0x%x\n",
                  step, addr);
      return false;
    }
    shadow[line][word_idx] = write_value;

    env.interconnect.set_llc_invalidate_all(true);
    cycle_outputs(env);
    cycle_inputs(env);
    env.interconnect.set_llc_invalidate_all(false);

    env.ar_events.clear();
    const uint8_t miss_id = static_cast<uint8_t>(0x40 + step);
    if (!issue_read(env, MASTER_ICACHE, addr, 15, miss_id, false)) {
      std::printf("FAIL: epoch stress cacheable read issue failed step=%u addr=0x%x\n",
                  step, addr);
      return false;
    }

    int timeout = sim_ddr::SIM_DDR_LATENCY * 20;
    bool saw_ar = false;
    while (!saw_ar && timeout-- > 0) {
      cycle_outputs(env);
      saw_ar = env.interconnect.axi_io.ar.arvalid && env.interconnect.axi_io.ar.arready;
      cycle_inputs(env);
    }
    if (!saw_ar) {
      std::printf("FAIL: epoch stress did not observe DDR AR before invalidate_all step=%u\n",
                  step);
      return false;
    }

    env.interconnect.set_llc_invalidate_all(true);
    cycle_outputs(env);
    cycle_inputs(env);
    env.interconnect.set_llc_invalidate_all(false);

    if (!wait_read_resp(env, MASTER_ICACHE, miss_id, shadow[line][word_idx],
                        shadow[line][word_idx + 1])) {
      std::printf("FAIL: epoch stress demand response failed step=%u addr=0x%x\n",
                  step, addr);
      return false;
    }

    env.ar_events.clear();
    const uint8_t reread_id = static_cast<uint8_t>(0x80 + step);
    if (!issue_read(env, MASTER_DCACHE_R, addr, 15, reread_id, false) ||
        !wait_read_resp(env, MASTER_DCACHE_R, reread_id, shadow[line][word_idx],
                        shadow[line][word_idx + 1])) {
      std::printf("FAIL: epoch stress reread after invalidate_all failed step=%u addr=0x%x\n",
                  step, addr);
      return false;
    }
    if (env.ar_events.size() != 1) {
      std::printf("FAIL: epoch stress reread should re-miss once after invalidate_all, got %zu\n",
                  env.ar_events.size());
      return false;
    }
  }

  std::printf("PASS\n");
  return true;
}

bool test_invalidate_line_rejects_same_cycle_upstream_write_accept_ready_first() {
  std::printf("=== AXI4 LLC Integration Test 12b: invalidate_line rejects same-cycle upstream write accept ready-first ===\n");

  Axi4LlcTestEnv env;
  init_env(env);

  const uint32_t line_addr = 0x3c00;
  auto &wp = env.interconnect.write_ports[MASTER_DCACHE_W];
  const WideWriteData_t wdata = make_line_write_data(0x8800);
  const WideWriteStrb_t wstrb = make_full_write_strobe();

  cycle_outputs(env);
  wp.req.valid = true;
  wp.req.addr = line_addr;
  wp.req.wdata = wdata;
  wp.req.wstrb = wstrb;
  wp.req.total_size = 63;
  wp.req.id = 0x81;
  wp.req.bypass = false;
  cycle_inputs(env);

  clear_upstream_inputs(env.interconnect);
  env.interconnect.set_llc_invalidate_line(true, line_addr);
  wp.req.valid = true;
  wp.req.addr = line_addr;
  wp.req.wdata = wdata;
  wp.req.wstrb = wstrb;
  wp.req.total_size = 63;
  wp.req.id = 0x81;
  wp.req.bypass = false;
  apply_downstream_outputs(env);
  env.interconnect.comb_outputs();

  const bool write_ready = env.interconnect.write_ports[MASTER_DCACHE_W].req.ready;
  const bool invalidate_accepted = env.interconnect.llc_invalidate_line_accepted();
  commit_cycle_inputs(env);
  env.interconnect.set_llc_invalidate_line(false, 0);
  const bool write_accepted = env.interconnect.write_req_accepted[MASTER_DCACHE_W];

  if (invalidate_accepted) {
    std::printf("FAIL: invalidate_line must not be accepted together with same-line upstream write accept\n");
    return false;
  }
  if (write_accepted) {
    std::printf("FAIL: ready-first same-line write should be blocked while invalidate_line is pending\n");
    return false;
  }
  if (write_ready) {
    std::printf("FAIL: ready-first same-line write should not continue advertising ready in conflict cycle\n");
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_invalidate_line_rejects_same_cycle_upstream_write_capture() {
  std::printf("=== AXI4 LLC Integration Test 12c: invalidate_line rejects same-cycle upstream write capture ===\n");

  Axi4LlcTestEnv env;
  init_env(env);

  const uint32_t line_addr = 0x4000;
  auto &wp = env.interconnect.write_ports[MASTER_DCACHE_W];
  const WideWriteData_t wdata = make_line_write_data(0x9900);
  const WideWriteStrb_t wstrb = make_full_write_strobe();

  cycle_outputs(env);
  wp.req.valid = true;
  wp.req.addr = line_addr;
  wp.req.wdata = wdata;
  wp.req.wstrb = wstrb;
  wp.req.total_size = 63;
  wp.req.id = 0x82;
  wp.req.bypass = false;
  cycle_inputs(env);

  clear_upstream_inputs(env.interconnect);
  env.interconnect.set_llc_invalidate_line(true, line_addr);
  wp.req.valid = true;
  wp.req.addr = line_addr;
  wp.req.wdata = wdata;
  wp.req.wstrb = wstrb;
  wp.req.total_size = 63;
  wp.req.id = 0x82;
  wp.req.bypass = false;
  apply_downstream_outputs(env);
  env.interconnect.comb_outputs();
  commit_cycle_inputs(env);
  env.interconnect.set_llc_invalidate_line(false, 0);

  if (env.interconnect.write_req_accepted[MASTER_DCACHE_W]) {
    std::printf("FAIL: same-line write capture should be blocked while invalidate_line is pending\n");
    return false;
  }
  if (env.interconnect.llc_invalidate_line_accepted()) {
    std::printf("FAIL: invalidate_line must not be accepted when same-line write capture happens\n");
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_invalidate_line_rejects_same_cycle_upstream_bypass_write_accept() {
  std::printf("=== AXI4 LLC Integration Test 12d: invalidate_line rejects same-cycle upstream bypass write accept ===\n");

  Axi4LlcTestEnv env;
  init_env(env);

  const uint32_t line_addr = 0x4400;
  auto &wp = env.interconnect.write_ports[MASTER_UNCORE_LSU_W];
  WideWriteData_t wdata{};
  wdata.clear();
  wdata[0] = 0xA5A5A5A5;
  WideWriteStrb_t wstrb{};
  wstrb.clear();
  for (uint32_t i = 0; i < 4; ++i) {
    wstrb.set(i, true);
  }

  cycle_outputs(env);
  wp.req.valid = true;
  wp.req.addr = line_addr;
  wp.req.wdata = wdata;
  wp.req.wstrb = wstrb;
  wp.req.total_size = 3;
  wp.req.id = 0x83;
  wp.req.bypass = true;
  cycle_inputs(env);

  clear_upstream_inputs(env.interconnect);
  env.interconnect.set_llc_invalidate_line(true, line_addr);
  wp.req.valid = true;
  wp.req.addr = line_addr;
  wp.req.wdata = wdata;
  wp.req.wstrb = wstrb;
  wp.req.total_size = 3;
  wp.req.id = 0x83;
  wp.req.bypass = true;
  apply_downstream_outputs(env);
  env.interconnect.comb_outputs();

  const bool write_ready = env.interconnect.write_ports[MASTER_UNCORE_LSU_W].req.ready;
  const bool invalidate_accepted = env.interconnect.llc_invalidate_line_accepted();
  commit_cycle_inputs(env);
  env.interconnect.set_llc_invalidate_line(false, 0);
  const bool write_accepted = env.interconnect.write_req_accepted[MASTER_UNCORE_LSU_W];

  if (invalidate_accepted) {
    std::printf("FAIL: invalidate_line must not be accepted together with same-line upstream bypass write\n");
    return false;
  }
  if (write_accepted) {
    std::printf("FAIL: same-line upstream bypass write should be blocked while invalidate_line is pending\n");
    return false;
  }
  if (write_ready) {
    std::printf("FAIL: same-line upstream bypass write should not advertise ready in conflict cycle\n");
    return false;
  }

  std::printf("PASS\n");
  return true;
}

bool test_invalidate_line_rejects_active_same_line_bypass_write_hazard() {
  std::printf("=== AXI4 LLC Integration Test 12e: invalidate_line rejects active same-line bypass write hazard until response is visible ===\n");

  Axi4LlcTestEnv env;
  init_env(env);

  const uint32_t line_addr = 0x4480;
  const uint32_t addr = line_addr + 8;
  write_memory_line(line_addr, 0x1200);

  WideWriteData_t wdata{};
  wdata.clear();
  wdata[0] = 0xCAFEBABE;
  WideWriteStrb_t wstrb{};
  wstrb.clear();
  for (uint32_t i = 0; i < 4; ++i) {
    wstrb.set(i, true);
  }

  if (!issue_write(env, MASTER_UNCORE_LSU_W, addr, wdata, wstrb, 3, 0x84, true)) {
    std::printf("FAIL: bypass write not accepted before hazard test\n");
    return false;
  }

  int timeout = sim_ddr::SIM_DDR_LATENCY * 80;
  bool saw_write_resp = false;
  bool accepted_with_visible_resp = false;
  while (timeout-- > 0) {
    env.interconnect.set_llc_invalidate_line(true, line_addr);
    cycle_outputs(env);
    auto &wresp = env.interconnect.write_ports[MASTER_UNCORE_LSU_W].resp;
    const bool resp_visible = wresp.valid;
    const bool invalidate_accepted = env.interconnect.llc_invalidate_line_accepted();
    if (wresp.valid) {
      wresp.ready = true;
      saw_write_resp = true;
    }
    cycle_inputs(env);
    env.interconnect.set_llc_invalidate_line(false, 0);
    if (invalidate_accepted) {
      if (!resp_visible) {
        std::printf("FAIL: invalidate_line accepted before same-line bypass write response became visible\n");
        return false;
      }
      accepted_with_visible_resp = true;
      break;
    }
    if (saw_write_resp) {
      break;
    }
  }

  if (!saw_write_resp) {
    std::printf("FAIL: bypass write never completed while hazard test was running\n");
    return false;
  }

  if (accepted_with_visible_resp) {
    std::printf("PASS\n");
    return true;
  }

  bool accepted_after_drain = false;
  timeout = sim_ddr::SIM_DDR_LATENCY * 40;
  while (timeout-- > 0) {
    env.interconnect.set_llc_invalidate_line(true, line_addr);
    cycle_outputs(env);
    if (env.interconnect.llc_invalidate_line_accepted()) {
      accepted_after_drain = true;
      cycle_inputs(env);
      env.interconnect.set_llc_invalidate_line(false, 0);
      break;
    }
    cycle_inputs(env);
    env.interconnect.set_llc_invalidate_line(false, 0);
  }

  if (!accepted_after_drain) {
    std::printf("FAIL: invalidate_line never accepted after bypass write hazard drained\n");
    return false;
  }

  std::printf("PASS\n");
  return true;
}

} // namespace

int main() {
  p_memory = new uint32_t[TEST_MEM_SIZE_WORDS];
  std::memset(p_memory, 0, TEST_MEM_SIZE_WORDS * sizeof(uint32_t));

  int passed = 0;
  int failed = 0;

  if (test_cross_master_write_then_read_latest()) {
    passed++;
  } else {
    failed++;
  }

  if (test_bypass_read_does_not_allocate_line()) {
    passed++;
  } else {
    failed++;
  }

  if (test_cacheable_and_bypass_parallel_disjoint()) {
    passed++;
  } else {
    failed++;
  }

  if (test_same_set_eviction_roundtrip_latest()) {
    passed++;
  } else {
    failed++;
  }

  if (test_same_set_valid_updates_preserve_neighbor_way()) {
    passed++;
  } else {
    failed++;
  }

  if (test_partial_cacheable_write_miss_refill_merge_and_dirty_eviction()) {
    passed++;
  } else {
    failed++;
  }

  if (test_bypass_read_sees_latest_after_cacheable_write()) {
    passed++;
  } else {
    failed++;
  }

  if (test_bypass_write_hit_updates_resident_line()) {
    passed++;
  } else {
    failed++;
  }

  if (test_mode2_direct_mapped_local_window_stays_off_ddr()) {
    passed++;
  } else {
    failed++;
  }

  if (test_mode2_window_outside_still_uses_ddr()) {
    passed++;
  } else {
    failed++;
  }

  if (test_mode2_window_outside_narrow_write_uses_aligned_ddr_beat()) {
    passed++;
  } else {
    failed++;
  }

  if (test_mode2_window_outside_cross_beat_read_falls_back_to_line()) {
    passed++;
  } else {
    failed++;
  }

  if (test_mode2_window_outside_cross_beat_write_falls_back_to_line()) {
    passed++;
  } else {
    failed++;
  }

  if (test_bypass_write_hit_preserves_dirty_on_eviction()) {
    passed++;
  } else {
    failed++;
  }

  if (test_invalidate_all_drops_stale_refill_install()) {
    passed++;
  } else {
    failed++;
  }

  if (test_invalidate_all_after_dirty_cacheable_write_stalls_and_preserves_latest()) {
    passed++;
  } else {
    failed++;
  }

  if (test_victim_writeback_maintenance_and_miss_interlock()) {
    passed++;
  } else {
    failed++;
  }

  if (test_invalidate_all_waits_for_dirty_victim_writeback()) {
    passed++;
  } else {
    failed++;
  }

  if (test_invalidate_all_pending_blocks_new_upstream_requests_but_drains_captured_requests()) {
    passed++;
  } else {
    failed++;
  }

  if (test_public_api_invalidate_all_hold_until_accept_contract()) {
    passed++;
  } else {
    failed++;
  }

  if (test_bypass_write_miss_does_not_allocate_line()) {
    passed++;
  } else {
    failed++;
  }

  if (test_bypass_write_miss_preserves_same_set_residents()) {
    passed++;
  } else {
    failed++;
  }

  if (test_seeded_small_depth_stress()) {
    passed++;
  } else {
    failed++;
  }

  if (test_llc_same_master_multi_write_accepts_before_first_resp()) {
    passed++;
  } else {
    failed++;
  }

  if (test_seeded_mixed_read_write_coherence_stress()) {
    passed++;
  } else {
    failed++;
  }

  if (test_seeded_invalidate_all_epoch_stress()) {
    passed++;
  } else {
    failed++;
  }

  if (test_invalidate_line_rejects_same_cycle_upstream_write_accept_ready_first()) {
    passed++;
  } else {
    failed++;
  }

  if (test_invalidate_line_rejects_same_cycle_upstream_write_capture()) {
    passed++;
  } else {
    failed++;
  }

  if (test_invalidate_line_rejects_same_cycle_upstream_bypass_write_accept()) {
    passed++;
  } else {
    failed++;
  }

  if (test_invalidate_line_rejects_active_same_line_bypass_write_hazard()) {
    passed++;
  } else {
    failed++;
  }

  std::printf("AXI4 LLC integration results: %d passed, %d failed\n", passed, failed);
  delete[] p_memory;
  return failed == 0 ? 0 : 1;
}
