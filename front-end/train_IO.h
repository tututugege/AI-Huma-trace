#ifndef FRONTEND_TRAIN_IO_H
#define FRONTEND_TRAIN_IO_H

#include "front_module.h"
#include "predecode.h"
#include "predecode_checker.h"
#include "BPU/BPU.h"

// Canonical training IO declarations for top-level pure combinational functions.
// All fields must ultimately resolve to wireX_t aliases via the included headers.

struct FetchAddrCombIn {
  fetch_address_FIFO_in inp;
  fetch_address_FIFO_read_data rd;
};

struct FetchAddrCombOut {
  fetch_address_FIFO_out out_regs;
  wire1_t clear_fifo;
  wire1_t push_en;
  fetch_addr_t push_data;
  FetchPair push_pair;
  wire1_t pop_en;
  uint8_t drop_tail_n;
};

struct InstructionCombIn {
  instruction_FIFO_in inp;
  instruction_FIFO_read_data rd;
};

struct InstructionCombOut {
  instruction_FIFO_out out_regs;
  wire1_t clear_fifo;
  wire1_t push_en;
  instruction_FIFO_entry push_entry;
  wire1_t pop_en;
};

struct PtabCombIn {
  PTAB_in inp;
  PTAB_read_data rd;
};

struct PtabCombOut {
  PTAB_out out_regs;
  wire1_t clear_ptab;
  wire1_t push_write_en;
  PTAB_entry push_write_entry;
  wire1_t pop_en;
};

struct Front2BackCombIn {
  front2back_FIFO_in inp;
  front2back_FIFO_read_data rd;
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
  bpuf::InputPayload inp;
  bpuf::Bpu::Txn req;
};

struct PendingFrontState {
  wire1_t valid = false;
  wire32_t next_front_sim_time = 0;
  wire1_t next_predecode_refetch = false;
  bpuf::PdInfo next_pd{};
  fetch_addr_t next_predecode_refetch_address = 0;
  wire1_t next_fetch_addr_fifo_full = false;
  wire1_t next_fetch_addr_fifo_empty = true;
  wire1_t next_fifo_full = false;
  wire1_t next_fifo_empty = true;
  wire1_t next_ptab_full = false;
  wire1_t next_ptab_empty = true;
  wire1_t next_front2back_fifo_full = false;
  wire1_t next_front2back_fifo_empty = true;
  wire1_t next_pend_valid = false;   // FE_PAIR: the ICache request in flight and the second block that belongs to it
  fetch_addr_t next_pend_addr = 0;
  FetchPair next_pend_pair{};
  int ipf_ncand = 0;                 // FE_IPF: fetch addresses the BPU pushed this cycle (prefetch candidates)
  uint32_t ipf_cand[2] = {0, 0};
};

struct FrontReadData {
  wire1_t predecode_refetch_snapshot = false;
  bpuf::PdInfo pd_snapshot{};
  fetch_addr_t predecode_refetch_address_snapshot = 0;
  wire32_t front_sim_time_snapshot = 0;
  fetch_address_FIFO_read_data fetch_addr_fifo_rd_snapshot{};
  instruction_FIFO_read_data fifo_rd_snapshot{};
  PTAB_read_data ptab_rd_snapshot{};
  front2back_FIFO_read_data front2back_fifo_rd_snapshot{};
  wire1_t fetch_addr_fifo_full_latch_snapshot = false;
  wire1_t fetch_addr_fifo_empty_latch_snapshot = true;
  wire1_t fifo_full_latch_snapshot = false;
  wire1_t fifo_empty_latch_snapshot = true;
  wire1_t ptab_full_latch_snapshot = false;
  wire1_t ptab_empty_latch_snapshot = true;
  wire1_t front2back_fifo_full_latch_snapshot = false;
  wire1_t front2back_fifo_empty_latch_snapshot = true;
  wire1_t pend_valid_snapshot = false;
  fetch_addr_t pend_addr_snapshot = 0;
  FetchPair pend_pair_snapshot{};
};

struct FrontUpdateRequest {
  PendingBpuSeqTxn bpu_seq_txn;
  PendingFrontState front_state;
  FetchAddrCombOut fetch_addr_fifo_req{};
  InstructionCombOut fifo_req{};
  PtabCombOut ptab_req{};
  Front2BackCombOut front2back_fifo_req{};
};

#endif
