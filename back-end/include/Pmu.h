#pragma once
// Performance monitoring unit of the model (EXP-013).
//
// Two layers, both off unless PMU=1 is set in the environment (they never change a cycle count):
//
//  1. Hardware-style event counters. Every cycle the model fills one event vector (PmuVec, the "event bus": one small
//     increment per event, the list is include/pmu_events.def) and adds it to 64-bit totals. This is exactly what the
//     RTL pmu block does with its event bus, so the totals of an RTL run and of this model are comparable event by
//     event. The top-down tree (level 1 + level 2) is computed from these counters only.
//
//  2. Model-only analysis, things a counter block could not hold: per-PC profile of who blocks commit, histograms
//     (commit per cycle, ROB/LDQ/STQ occupancy, outstanding misses), and a time series of all events every
//     PMU_INTERVAL cycles. Written as CSV for scripts/pmu/pmu_report.py.
//
// Environment: PMU=1 enable; PMU_OUT=<prefix> output prefix for the CSV files (default "pmu");
//              PMU_INTERVAL=<cycles> time-series interval (0/unset: off); PMU_PCPROF=0 turns the per-PC profile off.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>

enum PmuEvent : int {
#define PMU_EV(NAME, GROUP, WIDTH, DESC) PMU_##NAME,
#include "pmu_events.def"
#undef PMU_EV
  PMU_NUM
};

// One cycle of the event bus.
struct PmuVec {
  uint16_t v[PMU_NUM] = {};
  void add(PmuEvent e, unsigned n = 1) { v[e] = static_cast<uint16_t>(v[e] + n); }
};

// Per-cycle signals of the frontend that live outside BackTop (written by the ICache stage, read by the sampler).
struct PmuFrontSignals {
  bool icache_req_fire = false, icache_miss_issue = false, icache_miss_busy = false;
  bool itlb_access = false, itlb_miss = false;
};
inline PmuFrontSignals g_pmu_front;

// Model-only back-pressure flags (EXP-016): written by the comb stage of Ren / Idu, read by the sampler.
struct PmuBackSignals { bool ren_no_preg = false, idu_no_brtag = false; };
inline PmuBackSignals g_pmu_back;

// What blocks commit, per static instruction (model-only).
struct PmuPcStat {
  uint64_t commits = 0;
  uint64_t head_cycles = 0;       // cycles this instruction was the oldest incomplete one while fewer than COMMIT_WIDTH retired
  uint64_t head_miss_cycles = 0;  // ... of which it was a load waiting for an L1D miss
  uint64_t stall_slots = 0;       // dispatch slots lost (backend-stalled) while it was the oldest incomplete instruction
  uint64_t mispreds = 0;          // committed mispredicts
  uint8_t type = 0;
};

class Pmu {
public:
  bool enabled = false;
  uint64_t total[PMU_NUM] = {};
  PmuVec cur;
  bool recovering = false, recov_by_flush = false;

  // model-only layer
  bool pcprof = true;
  uint64_t interval = 0;
  std::string out_prefix = "pmu";
  std::unordered_map<uint32_t, PmuPcStat> pc_stat;
  std::vector<uint64_t> h_commit, h_rob, h_ldq, h_stq, h_mlp;  // histograms
  FILE *series = nullptr;
  uint64_t series_last[PMU_NUM] = {};
  uint64_t series_next = 0, ncycles = 0;
  uint64_t identity_violations = 0;

  void init_from_env();
  void reset();
  void accumulate();  // add `cur` to the totals, update the time series
  void report(uint64_t instructions);  // print the counters and the top-down tree, write the CSV files
  void close();

  static const char *name(int e);
  static const char *group(int e);
  static const char *desc(int e);
};
