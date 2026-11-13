#pragma once

#include "FTQ.h"
#include "IO.h"
#include "InstructionBuffer.h"
#include "config.h"

class SimContext;

static_assert(FTQ_SIZE <= 64, "ftq_alloc_mask is a 64-bit slot mask");

class PreIduQueue {
public:
  explicit PreIduQueue(SimContext *ctx = nullptr) : ctx(ctx) {}
  struct Wires *wires = nullptr;  // signals shared with the other back-end modules

  void init();
  void comb_begin();
  void comb_accept_front();
  void comb_fire();
  void comb_ftq_lookup_prf();
  void comb_ftq_lookup_rob();
  void comb_commit_info();
  void seq();

  friend struct PreShadow;  // lockstep verification (PRE_SHADOW builds)
  // BPA_REPAIR: BPU block sequence number and slot pc of an FTQ entry (read by BackTop when a branch mispredict redirects the front end)
  uint16_t ftq_blk_id(uint32_t idx, uint32_t off = 0) const { return ftq_train_meta_fifo[idx].meta.blk_id[off]; }
  uint64_t ftq_slot_pc(uint32_t idx, uint32_t off) const { return ftq_lookup_entries[idx].slot_pc[off]; }
  bool ftq_slot_rvc(uint32_t idx, uint32_t off) const { return ftq_lookup_entries[idx].slot_rvc[off]; }
  const FTQEntry &dbg_entry(uint32_t idx) const { return ftq_lookup_entries[idx]; }
private:
  SimContext *ctx = nullptr;

  int ftq_alloc();
  void ftq_pop(int pop_cnt);
  void ftq_recover(int new_tail);
  void ftq_flush();

  InstructionBuffer ibuf;
  InstructionBuffer ibuf_1;
  // committed (x) == working (x_1) at comb_begin; seq copies only dirty parts.
  bool ibuf_dirty = false;
  bool ftq_dirty = false;
  uint64_t ftq_alloc_mask = 0; // slots allocated this cycle (FTQ_SIZE <= 64)

  FTQEntry ftq_lookup_entries[FTQ_SIZE];
  FTQEntry ftq_lookup_entries_1[FTQ_SIZE];
  FTQTrainMetaEntry ftq_train_meta_fifo[FTQ_SIZE];
  FTQTrainMetaEntry ftq_train_meta_fifo_1[FTQ_SIZE];
  wire<1> ftq_valid[FTQ_SIZE] = {};
  wire<1> ftq_valid_1[FTQ_SIZE] = {};
  reg<FTQ_IDX_WIDTH> ftq_head = 0;
  reg<FTQ_IDX_WIDTH> ftq_tail = 0;
  reg<bit_width_for_count(FTQ_SIZE + 1)> ftq_count = 0;
  wire<FTQ_IDX_WIDTH> ftq_head_1 = 0;
  wire<FTQ_IDX_WIDTH> ftq_tail_1 = 0;
  wire<bit_width_for_count(FTQ_SIZE + 1)> ftq_count_1 = 0;
};
