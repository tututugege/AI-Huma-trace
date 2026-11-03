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
  wire1_t pop_en;
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
  fetch_addr_t next_predecode_refetch_address = 0;
  wire1_t next_fetch_addr_fifo_full = false;
  wire1_t next_fetch_addr_fifo_empty = true;
  wire1_t next_fifo_full = false;
  wire1_t next_fifo_empty = true;
  wire1_t next_ptab_full = false;
  wire1_t next_ptab_empty = true;
  wire1_t next_front2back_fifo_full = false;
  wire1_t next_front2back_fifo_empty = true;
};

struct FrontReadData {
  wire1_t predecode_refetch_snapshot = false;
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
};

struct FrontUpdateRequest {
  PendingBpuSeqTxn bpu_seq_txn;
  PendingFrontState front_state;
  FetchAddrCombOut fetch_addr_fifo_req{};
  InstructionCombOut fifo_req{};
  PtabCombOut ptab_req{};
  Front2BackCombOut front2back_fifo_req{};
};















struct FrontPtabWriteCombIn {
  bpuf::OutputPayload bpu_output;
  wire1_t global_reset;
  wire1_t global_refetch;
  wire1_t ptab_can_write;
};

struct FrontPtabWriteCombOut {
  PTAB_in ptab_in;
};

struct FrontCheckerInputCombIn {
  instruction_FIFO_out fifo_out;
  PTAB_out ptab_out;
};

struct FrontCheckerInputCombOut {
  predecode_checker_in checker_in;
};

struct FrontFront2backWriteCombIn {
  instruction_FIFO_out fifo_out;
  PTAB_out ptab_out;
  predecode_checker_out checker_out;
  wire1_t use_front2back_output_bypass;
};

struct FrontFront2backWriteCombOut {
  front2back_FIFO_in front2back_fifo_in;
  front2back_FIFO_out bypass_front2back_fifo_out;
};

struct FrontOutputCombIn {
  front2back_FIFO_out saved_front2back_fifo_out;
  front2back_FIFO_out bypass_front2back_fifo_out;
  wire1_t use_front2back_output_bypass;
};

struct FrontOutputCombOut {
  front_top_out out;
};

#endif
