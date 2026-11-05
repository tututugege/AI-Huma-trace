#pragma once
/**
 * @file SimDDR_DS3.h
 * @brief DRAMsim3 backend for SimDDR (EXP-004, build with SIM_DDR_DS3=1).
 *
 * SimDDR keeps its AXI front end (queues, W FIFO, R burst drain). Only the DRAM timing comes from DRAMsim3: reads become
 * 64 B DRAM reads issued in order, a read is ready for data after all of its lines return plus a fixed extra latency
 * (SoC/CDC/PHY path that DRAMsim3 does not model); writes are posted, B is returned once all lines are accepted.
 * Env: CPU_FREQ_MHZ (core frequency, default CONFIG_CPU_FREQ_MHZ), DS3_CONFIG (ini), DS3_OUT (output dir), DS3_EXTRA_NS, DS3_STATS=1.
 */
#include "dramsim3.h"
#ifndef DS3_DEFAULT_CONFIG  // DS3_CFG_PATH: unquoted path, for build systems where quoting a -D string is painful
#define DS3_S2_(x) #x
#define DS3_S_(x) DS3_S2_(x)
#define DS3_DEFAULT_CONFIG DS3_S_(DS3_CFG_PATH)
#endif
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace sim_ddr {

struct Ds3 {
  struct Rd { uint64_t first; uint32_t lines, issued = 0, done = 0; uint64_t ar_cycle, done_cycle = 0; std::vector<uint64_t> pend; };
  struct Wr { uint64_t first; uint32_t lines, issued = 0; uint8_t id; };

  dramsim3::MemorySystem *ms;
  uint64_t cpu_fs, sub_fs, tck_fs, dram_next_fs = 0, now_fs = 0, cpu_cycle = 0, extra_cycles;
  uint32_t line_bytes;
  std::unordered_map<uint32_t, Rd> rd;  // by tag
  std::deque<uint32_t> rq;              // tags with lines still to issue
  std::deque<Wr> wq;
  std::vector<uint64_t> done_addrs;
  uint32_t next_tag = 1;
  // stats
  uint64_t n_rd = 0, lat_sum = 0, lat_max = 0, n_wr = 0, rd_stall = 0, wr_stall = 0;

  // one tick() = one SimDDR::seq() call = 1/div of a core cycle (CONFIG_AXI_SUBSYSTEM_FREQ_DIV)
  explicit Ds3(uint32_t cpu_mhz, uint32_t div, uint32_t default_extra_ns) {
    const char *cfg = std::getenv("DS3_CONFIG");
    const char *out = std::getenv("DS3_OUT");
    const char *ex = std::getenv("DS3_EXTRA_NS");
    std::string cfgs = cfg ? cfg : DS3_DEFAULT_CONFIG;
    std::string outs = out ? out : "ds3_out";
    std::filesystem::create_directories(outs);
    ms = dramsim3::GetMemorySystem(
        cfgs, outs, [this](uint64_t a) { done_addrs.push_back(a); }, [](uint64_t) {});
    line_bytes = uint32_t(ms->GetBurstLength() * ms->GetBusBits() / 8);
    tck_fs = uint64_t(ms->GetTCK() * 1e6 + 0.5);
    const char *fq = std::getenv("CPU_FREQ_MHZ");  // runtime override of CONFIG_CPU_FREQ_MHZ
    if (fq && std::strtoul(fq, nullptr, 10) > 0) cpu_mhz = uint32_t(std::strtoul(fq, nullptr, 10));
    cpu_fs = uint64_t(1e9 / cpu_mhz + 0.5);
    const uint64_t extra_ns = ex ? std::strtoull(ex, nullptr, 10) : default_extra_ns;
    sub_fs = cpu_fs / div;
    extra_cycles = (extra_ns * 1000000ull + sub_fs / 2) / sub_fs;
  }
  ~Ds3() {
    if (std::getenv("DS3_STATS"))
      std::fprintf(stderr,
                   "[DS3] reads %llu avg_lat %.1f max %llu ticks (%.1f ns avg), writes %llu, rd_stall %llu wr_stall %llu, "
                   "line %uB tCK %.3fns extra %llu ticks\n",
                   (unsigned long long)n_rd, n_rd ? double(lat_sum) / n_rd : 0.0, (unsigned long long)lat_max, n_rd ? double(lat_sum) / n_rd * sub_fs / 1e6 : 0.0,
                   (unsigned long long)n_wr, (unsigned long long)rd_stall, (unsigned long long)wr_stall, line_bytes,
                   ms->GetTCK(), (unsigned long long)extra_cycles);
    delete ms;
  }

  uint64_t line_of(uint64_t a) const { return a & ~uint64_t(line_bytes - 1); }
  uint32_t lines_for(uint64_t addr, uint32_t bytes) const {
    return uint32_t((line_of(addr + bytes - 1) - line_of(addr)) / line_bytes + 1);
  }

  uint32_t add_read(uint64_t addr, uint32_t bytes) {
    const uint32_t tag = next_tag++;
    rd[tag] = Rd{line_of(addr), lines_for(addr, bytes), 0, 0, cpu_cycle, 0, {}};
    rq.push_back(tag);
    order_.push_back(tag);
    return tag;
  }
  void add_write(uint64_t addr, uint32_t bytes, uint8_t id) {
    wq.push_back(Wr{line_of(addr), lines_for(addr, bytes), 0, id});
  }

  // advance DRAM time by one CPU cycle and route completions
  void tick() {
    cpu_cycle++;
    now_fs += sub_fs;
    while (dram_next_fs <= now_fs) { ms->ClockTick(); dram_next_fs += tck_fs; }
    for (uint64_t a : done_addrs) {
      for (uint32_t tag : order_) {
        Rd &r = rd[tag];
        auto p = std::find(r.pend.begin(), r.pend.end(), a);
        if (p == r.pend.end()) continue;
        r.pend.erase(p);
        if (++r.done == r.lines) r.done_cycle = cpu_cycle;
        break;
      }
    }
    done_addrs.clear();
  }
  // issue pending lines (reads first, in order)
  void pump(std::vector<uint8_t> &wr_done_ids) {
    while (!rq.empty()) {
      Rd &r = rd[rq.front()];
      bool blocked = false;
      while (r.issued < r.lines) {
        const uint64_t a = r.first + uint64_t(r.issued) * line_bytes;
        if (!ms->WillAcceptTransaction(a, false)) { blocked = true; rd_stall++; break; }
        ms->AddTransaction(a, false);
        r.pend.push_back(a);
        r.issued++;
      }
      if (blocked) break;
      rq.pop_front();
    }
    while (!wq.empty()) {
      Wr &w = wq.front();
      bool blocked = false;
      while (w.issued < w.lines) {
        const uint64_t a = w.first + uint64_t(w.issued) * line_bytes;
        if (!ms->WillAcceptTransaction(a, true)) { blocked = true; wr_stall++; break; }
        ms->AddTransaction(a, true);
        w.issued++;
      }
      if (blocked) break;
      wr_done_ids.push_back(w.id);
      n_wr++;
      wq.pop_front();
    }
  }
  bool read_ready(uint32_t tag) {
    auto it = rd.find(tag);
    if (it == rd.end()) return false;
    Rd &r = it->second;
    if (r.done != r.lines || cpu_cycle < r.done_cycle + extra_cycles) return false;
    return true;
  }
  void read_finish(uint32_t tag) {
    const Rd &r = rd[tag];
    const uint64_t lat = cpu_cycle - r.ar_cycle;
    n_rd++; lat_sum += lat; if (lat > lat_max) lat_max = lat;
  }
  void read_retire(uint32_t tag) {
    rd.erase(tag);
    for (auto it = order_.begin(); it != order_.end(); ++it) if (*it == tag) { order_.erase(it); break; }
  }

private:
  std::deque<uint32_t> order_;  // issued reads, oldest first (callbacks carry only the address)
};

} // namespace sim_ddr
