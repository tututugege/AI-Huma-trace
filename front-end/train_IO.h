#ifndef FRONTEND_TRAIN_IO_H
#define FRONTEND_TRAIN_IO_H

#include "front_module.h"
#include "predecode.h"
#include "predecode_checker.h"
#include "BPU/BPU.h"

// Canonical training IO declarations for top-level pure combinational functions.
// All fields must ultimately resolve to wireX_t aliases via the included headers.

struct FetchAddrCombOut {
  fetch_address_FIFO_out out_regs;
  wire1_t clear_fifo;
  wire1_t push_en;
  fetch_addr_t push_data;
  FetchPair push_pair;
  wire1_t pop_en;
  uint8_t drop_tail_n;
};

struct InstructionCombOut {
  instruction_FIFO_out out_regs;
  wire1_t clear_fifo;
  wire1_t push_en;
  instruction_FIFO_entry push_entry;
  wire1_t pop_en;
};

struct PtabCombOut {
  PTAB_out out_regs;
  wire1_t clear_fifo;
  wire1_t push_en;
  PTAB_entry push_entry;
  wire1_t pop_en;
};

struct Front2BackCombOut {
  front2back_FIFO_out out_regs;
  wire1_t clear_fifo;
  wire1_t push_en;
  front2back_FIFO_entry push_entry;
  wire1_t pop_en;
};


struct PendingBpuSeqTxn {
  wire1_t valid = false;
  wire1_t reset = false;
};

// The front end's own registers (the FIFOs and the BPU keep theirs): q = the value of this cycle, the comb builds d, seq copies d to q.
struct FrontRegs {
  wire1_t predecode_refetch = false;          // the predecode checker flushed last cycle: refetch from predecode_refetch_address
  fetch_addr_t predecode_refetch_address = 0;
  bpuf::PdInfo pd{};                          // the check of last cycle, reported to the BPU
  wire1_t fetch_addr_fifo_full = false, fetch_addr_fifo_empty = true;   // FIFO occupancy as registered at the end of last cycle
  wire1_t fifo_full = false, fifo_empty = true;
  wire1_t ptab_empty = true;
  wire1_t f2b_full = false, f2b_empty = true;
  wire1_t lh_valid = false;     // RVC=1: the next group begins with the second half of an instruction that started in the last slot of the group before
  pc_t lh_pc = 0;               // ... that group starts at this pc
  uint16_t lh_data = 0;         // ... and the carried first halfword (C910-style carry)
  wire1_t pend_valid = false;   // FE_PAIR: the ICache request in flight and the second block that belongs to it
  fetch_addr_t pend_addr = 0;
  FetchPair pend_pair{};
  uint64_t pmu_last_refetch = ~0ull;   // PMU: the cycle of the last refetch (front-end delivery attribution)
};

struct PendingFrontState {
  wire1_t valid = false;
  FrontRegs d;
};

struct PendingIpf {   // FE_IPF: fetch addresses the BPU pushed this cycle (prefetch candidates, started at the edge)
  int ncand = 0;
  uint32_t cand[2] = {0, 0};
};

struct FrontReadData {
  FrontRegs q;
  fetch_address_FIFO_read_data fetch_addr_fifo_rd_snapshot{};
  instruction_FIFO_read_data fifo_rd_snapshot{};
  PTAB_read_data ptab_rd_snapshot{};
  front2back_FIFO_read_data front2back_fifo_rd_snapshot{};
};

struct FrontUpdateRequest {
  PendingBpuSeqTxn bpu_seq_txn;
  PendingFrontState front_state;
  PendingIpf ipf;
  FetchAddrCombOut fetch_addr_fifo_req{};
  InstructionCombOut fifo_req{};
  PtabCombOut ptab_req{};
  Front2BackCombOut front2back_fifo_req{};
};

#endif
