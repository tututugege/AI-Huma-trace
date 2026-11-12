// F/D register-file port statistics and the FP write-port limit (EXP-032), printed at exit.
//   FP_WB_PORTS=<n> (env; 0 = unlimited, the default): at most n FP-register writes of the pipelined / iterative FP units (FMA, MISC with an FP
//   destination, FDIV, I2F) per cycle.  The write cycle is known at issue (the unit latency), so the issue stage reserves it in a
//   busy table (like XiangShan's WbFuBusyTable) and a uop that finds the cycle full stays in the issue queue.  FP loads write
//   through the LSU and are only counted.
#pragma once
#include "config.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern long long sim_time;

struct FpStats {
  static constexpr int kRing = 64, kHist = 16;
  int ports = [] { const char *e = std::getenv("FP_WB_PORTS"); return e ? std::atoi(e) : 0; }();
  uint8_t busy[kRing] = {};
  uint64_t wb_hist[kHist] = {};       // FU writes to the FP register file per cycle
  uint64_t ld_hist[kHist] = {};       // FP load write-backs per cycle (cycles with at least one)
  uint64_t rd_hist[kHist] = {};       // FP register-file reads per cycle (issued uops of the FP queue and I2F-free; STD FP data counted separately)
  uint64_t std_hist[kHist] = {};      // FP store-data reads per cycle
  uint64_t gated = 0, reserved = 0;
  long long last_cycle = -1;

  void new_cycle(bool flush) {
    if (sim_time == last_cycle) return;
    last_cycle = sim_time;
    const int slot = static_cast<int>((sim_time - 1) & (kRing - 1));
    if (busy[slot]) wb_hist[busy[slot] < kHist ? busy[slot] : kHist - 1]++;
    busy[slot] = 0;
    if (flush) std::memset(busy, 0, sizeof busy);
  }
  bool try_reserve(int lat) {
    const int slot = static_cast<int>((sim_time + lat) & (kRing - 1));
    if (ports > 0 && busy[slot] >= ports) { gated++; return false; }
    busy[slot]++;
    reserved++;
    return true;
  }
  static void bump(uint64_t *h, int n) { if (n > 0) h[n < kHist ? n : kHist - 1]++; }
  ~FpStats() {
    if (!reserved && !gated) return;
    auto pr = [](const char *name, const uint64_t *h) {
      std::fprintf(stderr, "[FP_PORTS] %s (cycles with n = 1..): ", name);
      for (int i = 1; i < kHist; i++) if (h[i]) std::fprintf(stderr, "%d:%llu ", i, (unsigned long long)h[i]);
      std::fprintf(stderr, "\n");
    };
    std::fprintf(stderr, "[FP_PORTS] limit=%d reserved=%llu issue-gated=%llu\n", ports, (unsigned long long)reserved, (unsigned long long)gated);
    pr("FU writes per cycle", wb_hist);
    pr("FP load write-backs per cycle", ld_hist);
    pr("FP reads of issued FP-queue uops per cycle", rd_hist);
    pr("FP store-data reads per cycle", std_hist);
  }
};
inline FpStats g_fp_stats;
