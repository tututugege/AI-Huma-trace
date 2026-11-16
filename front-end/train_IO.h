#ifndef FRONTEND_TRAIN_IO_H
#define FRONTEND_TRAIN_IO_H

#include "front_module.h"
#include "predecode.h"
#include "predecode_checker.h"
#include "BPU/BPU.h"

// Canonical training IO declarations for top-level pure combinational functions.
// All fields must ultimately resolve to wireX_t aliases via the included headers.

struct FetchAddrCombOut {
  FetchAddrFifoOut out_regs;
  wire<1> clear_fifo;
  wire<1> push_en;
  fetch_addr_t push_data;
  FetchPair push_pair;
  wire<1> pop_en;
  uint8_t drop_tail_n;
};

struct InstructionCombOut {
  InstFifoOut out_regs;
  wire<1> clear_fifo;
  wire<1> push_en;
  InstFifoEntry push_entry;
  wire<1> pop_en;
};

struct PtabCombOut {
  PtabOut out_regs;
  wire<1> clear_fifo;
  wire<1> push_en;
  PtabEntry push_entry;
  wire<1> pop_en;
};

struct Front2BackCombOut {
  F2bFifoOut out_regs;
  wire<1> clear_fifo;
  wire<1> push_en;
  F2bFifoEntry push_entry;
  wire<1> pop_en;
};


struct PendingBpuSeqTxn {
  wire<1> valid = false;
  wire<1> reset = false;
};

// The front end's own registers (the FIFOs and the BPU keep theirs): q = the value of this cycle, the comb builds d, seq copies d to q.
struct FrontRegs {
  wire<1> predecode_refetch = false;          // the predecode checker flushed last cycle: refetch from predecode_refetch_address
  fetch_addr_t predecode_refetch_address = 0;
  bpuf::PdInfo pd{};                          // the check of last cycle, reported to the BPU
  wire<1> fetch_addr_fifo_full = false, fetch_addr_fifo_empty = true;   // FIFO occupancy as registered at the end of last cycle
  wire<1> fifo_full = false, fifo_empty = true;
  wire<1> ptab_empty = true;
  wire<1> f2b_full = false, f2b_empty = true;
  wire<1> lh_valid = false;     // RVC=1: the next group begins with the second half of an instruction that started in the last slot of the group before
  pc_t lh_pc = 0;               // ... that group starts at this pc
  uint16_t lh_data = 0;         // ... and the carried first halfword (C910-style carry)
  wire<1> pend_valid = false;   // FE_PAIR: the ICache request in flight and the second block that belongs to it
  fetch_addr_t pend_addr = 0;
  FetchPair pend_pair{};
  uint64_t pmu_last_refetch = ~0ull;   // PMU: the cycle of the last refetch (front-end delivery attribution)
};

struct PendingFrontState {
  wire<1> valid = false;
  FrontRegs d;
};

struct PendingIpf {   // FE_IPF: fetch addresses the BPU pushed this cycle (prefetch candidates, started at the edge)
  int ncand = 0;
  uint32_t cand[2] = {0, 0};
};

struct FrontReadData {
  FrontRegs q;
  FetchAddrFifoReadData fetch_addr_fifo_rd_snapshot{};
  InstFifoReadData fifo_rd_snapshot{};
  PtabReadData ptab_rd_snapshot{};
  F2bFifoReadData front2back_fifo_rd_snapshot{};
};

#endif
