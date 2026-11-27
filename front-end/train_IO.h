#ifndef FRONTEND_TRAIN_IO_H
#define FRONTEND_TRAIN_IO_H

#include "front_module.h"
#include "predecode.h"
#include "predecode_checker.h"
#include "BPU/BPU.h"

// Canonical training IO declarations for top-level pure combinational functions.
// All fields must ultimately resolve to wireX_t aliases via the included headers.

// An entry FIFO's request of the cycle, applied at the edge: the pushed entry is the write port's data (its In's
// wdata, which holds still until seq), not a copy.
template <class Entry> struct FifoReq {
  wire<1> clear_fifo;
  wire<1> push_en;
  const Entry *push;
  wire<1> pop_en;
};
using InstructionCombOut = FifoReq<InstFifoEntry>;

struct PendingBpuSeqTxn {
  wire<1> valid = false;
  wire<1> reset = false;
};

// The front end's own registers (Unified FTQ-64 lives in PreIduQueue): q = the value of this cycle, comb builds d, seq copies d to q.
struct FrontRegs {
  wire<1> predecode_refetch = false;          // the predecode checker flushed last cycle: refetch from predecode_refetch_address
  fetch_addr_t predecode_refetch_address = 0;
  bpuf::PdInfo pd{};                          // the check of last cycle, reported to the BPU
  wire<1> fifo_full = false, fifo_empty = true;
  wire<1> lh_valid = false;     // RVC=1: the next group begins with the second half of an instruction that started in the last slot of the group before
  pc_t lh_pc = 0;               // ... that group starts at this pc
  uint16_t lh_data = 0;         // ... and the carried first halfword (C910-style carry)
  wire<1> pend_valid = false;   // FE_PAIR: the ICache request in flight and the second block that belongs to it
  fetch_addr_t pend_addr = 0;
  FetchPair pend_pair{};
  wire<1> fdip_s2_en = false, fdip_s2_clear = false;  // Cut B1: 1-cycle registered BPU S2 -> FDIP pipe
  pc_t fdip_s2_va = 0;
  uint64_t pmu_last_refetch = ~0ull;   // PMU: the cycle of the last refetch (front-end delivery attribution)
};

struct PendingFrontState {
  wire<1> valid = false;
  FrontRegs d;
};

struct FrontReadData {
  FrontRegs q;
  InstFifoReadData fifo_rd_snapshot{};
};

#endif
