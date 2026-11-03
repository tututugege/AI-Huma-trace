#pragma once
#include "config.h"
#include <cstdint>
#include <cstdio>

// Only the counters the simulation needs: the cycle and commit counters used by the main loop, the exit
// conditions and the checkpoint measurement window. The statistics suite (TMA, cache/LLC/PTW/branch counters,
// periodic snapshots) is gone; the gate reads the `sim-time(cycle)` line printed below.
class PerfCount {
public:
  bool perf_start = false;
  uint64_t cycle = 0;
  uint64_t cycle_base = 0;  // front-door restore: cycles before the measurement window (cycle itself is an absolute clock)
  uint64_t commit_num = 0;
  uint64_t commit_load_num = 0;
  uint64_t commit_store_num = 0;

  void perf_reset() {
    cycle = 0;
    commit_num = 0;
    commit_load_num = 0;
    commit_store_num = 0;
  }

  void perf_print() {
    const uint64_t cyc = cycle - cycle_base;
    printf("\033[38;5;34msim-time(cycle)= %ld, committed(total/load/store)= %ld / %ld / %ld\033[0m\n",
           (long)cyc, (long)commit_num, (long)commit_load_num, (long)commit_store_num);
    printf("\033[38;5;34minstruction num: %ld\033[0m\n", (long)commit_num);
    printf("\033[38;5;34mcycle       num: %ld\033[0m\n", (long)cyc);
    printf("\033[38;5;34mipc            : %f\033[0m\n", cyc ? (double)commit_num / cyc : 0.0);
  }

  void perf_maybe_capture_simtime_snapshot() {}
};
