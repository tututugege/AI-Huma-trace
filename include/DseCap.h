#pragma once
// EXP-047 fast DSE: run-time capacity caps of the back-end queues, each <= its compiled size.  A cap only limits allocation
// (the "full" / "free entries" checks); the arrays, ring indices and bit widths keep the compiled size, so a capped queue
// behaves like a smaller queue as far as the occupancy goes.  The defaults are the compiled sizes: without DSE_CONFIGS
// nothing changes.  driver/DseFork.cpp sets them in the forked measurement children (docs/DSE_FORK.md).
#include "config.h"

struct DseCaps {
  int rob = ROB_NUM;              // ROB entries
  int prf = PRF_NUM;              // integer physical registers
  int fprf = FPRF_NUM;            // FP physical registers
  int br = MAX_BR_NUM;            // branch tags (tag 0 is "no branch", so br - 1 branches in flight)
  int ldq = LDQ_SIZE;             // LDQ entries
  int stq = STQ_SIZE;             // STQ entries
  int mshr = DCACHE_MSHR_ENTRIES; // L1D MSHR entries
  int iq_int = IQ_INT_SIZE;       // each INT queue (IQ_INT and IQ_INT1)
  int iq_mem = CFG_MEM_IQ_SIZE;
  int iq_std = CFG_STD_IQ_SIZE;
  int iq_br = CFG_BR_IQ_SIZE;
  int iq_fp = CFG_FP_IQ_SIZE;
};
inline DseCaps g_dse;

// cap of issue queue `id` (IQType), or `size` for a queue without a cap
inline int dse_iq_cap(int id, int size) {
  switch (id) {
  case 0: case 5: return g_dse.iq_int < size ? g_dse.iq_int : size;   // IQ_INT, IQ_INT1
  case 1: return g_dse.iq_mem < size ? g_dse.iq_mem : size;
  case 2: return g_dse.iq_std < size ? g_dse.iq_std : size;
  case 3: return g_dse.iq_br < size ? g_dse.iq_br : size;
  case 4: return g_dse.iq_fp < size ? g_dse.iq_fp : size;
  default: return size;
  }
}
inline int dse_free(int cap, int used) { return used >= cap ? 0 : cap - used; }

// The fork point: in a DSE run (DSE_CONFIGS set) the parent runs the detailed warm-up up to commit g_dse_fork_at
// (= warm-up target - DSE_SETTLE), then forks one child per configuration; each child sets its caps and runs the rest of the
// warm-up (the settle part, which drains the queues to the new caps) and the measurement as a normal run would.
inline bool g_dse_active = false;
inline bool g_dse_forked = false;
inline bool g_dse_fork_pending = false;
inline uint64_t g_dse_fork_at = 0;

// EXP-048: called when the measurement window starts (Pmu::reset), for model-side statistics outside the PMU (DRAMsim3)
inline void (*g_measure_reset_hook)() = nullptr;
