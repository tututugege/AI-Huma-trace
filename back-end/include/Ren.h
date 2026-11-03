#pragma once
#include "IO.h"
#include "config.h"

class BackTop;

class Ren {
public:
  Ren(SimContext *ctx) { this->ctx = ctx; }
  SimContext *ctx;
  struct Wires *wires = nullptr;  // signals shared with the other back-end modules

  void init();
  void comb_begin(); // 默认保持寄存器状态（*_1 <- *）
  void comb_rename(); // 重命名
  void comb_fire();
  void comb_alloc(); // 分配寄存器
  void comb_pipeline();
  void seq();
  void me_check() const;

  // register
  Uop inst_r[DECODE_WIDTH];
  reg<1> inst_valid[DECODE_WIDTH];
  reg<PRF_IDX_WIDTH> arch_RAT[ARF_NUM + 1];
  reg<PRF_IDX_WIDTH> spec_RAT[ARF_NUM + 1];
  reg<PRF_IDX_WIDTH> RAT_checkpoint[MAX_BR_NUM][ARF_NUM + 1];
  // move elimination: number of arch_RAT entries that name each physical register (a register is freed at 0)
  int arch_cnt[PRF_NUM];
  reg<PRF_IDX_WIDTH> free_list[PRF_NUM];
  reg<PRF_IDX_WIDTH> free_head; // speculative head
  reg<PRF_IDX_WIDTH> free_head_commit;
  reg<PRF_IDX_WIDTH> free_tail;
  reg<PRF_IDX_WIDTH> alloc_checkpoint_head[MAX_BR_NUM];

  Uop inst_r_1[DECODE_WIDTH];
  wire<1> inst_valid_1[DECODE_WIDTH];
  wire<PRF_IDX_WIDTH> arch_RAT_1[ARF_NUM + 1];
  wire<PRF_IDX_WIDTH> spec_RAT_1[ARF_NUM + 1];
  wire<PRF_IDX_WIDTH> RAT_checkpoint_1[MAX_BR_NUM][ARF_NUM + 1];
  static_assert(MAX_BR_NUM <= 64, "ckpt_dirty is a 64-bit row mask");
  uint64_t ckpt_dirty = 0;  // rows of RAT_checkpoint_1 written this cycle
  wire<PRF_IDX_WIDTH> free_list_1[PRF_NUM];
  wire<PRF_IDX_WIDTH> free_head_1;
  wire<PRF_IDX_WIDTH> free_head_commit_1;
  wire<PRF_IDX_WIDTH> free_tail_1;
  wire<PRF_IDX_WIDTH> alloc_checkpoint_head_1[MAX_BR_NUM];
};
