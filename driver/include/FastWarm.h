// Fast warm-up replay (EXP-011): run the functional reference for N instructions while feeding the committed-path
// branch outcome, instruction-line and data-line streams straight into the detailed model's L1I/L1D/LLC tags, ITLB/DTLB
// and BPU, so a checkpoint needs only a short detailed warm-up. Streaming: nothing is recorded, memory stays small.
#pragma once
#include <cstdint>

class SimCpu;

struct FastWarmStats {
  uint64_t stub_steps = 0, steps = 0, branches = 0, ifetch_lines = 0, ifetch_miss = 0, loads = 0, stores = 0;
  uint64_t l1d_miss = 0, itlb_miss = 0, dtlb_miss = 0, llc_acc = 0, llc_miss = 0, skipped_exc = 0;
  bool ended_early = false;
};

// 1. skip_until_mret: execute the restore program (first MRET included, not recorded);
// 2. replay `steps` further reference steps with recording;
// 3. (the caller restores the DUT from the reference) -- then call fast_warm_install() to load the warmed structures.
bool fast_warm_replay(SimCpu &cpu, bool skip_until_mret, uint64_t steps, FastWarmStats &st);
void fast_warm_install(SimCpu &cpu);
