#pragma once
// Frozen copy of the original Rob (lockstep reference, ROB_SHADOW builds).
#include "IO.h"
#include "config.h"

struct RobCommitExtraRef {
  wire<AREG_IDX_WIDTH> src1_areg;
  wire<32> imm;

  RobCommitExtraRef() { std::memset(this, 0, sizeof(RobCommitExtraRef)); }
};

struct RobOutRef {
  RobCommitExtraRef commit_extra[COMMIT_WIDTH];
};

struct RobStoredEntryRef {
  wire<1> valid;
  Uop uop;
};

class RobRef {
public:
  RobRef(SimContext *ctx) { this->ctx = ctx; }
  SimContext *ctx;
  void init();
  void seq();
  uint64_t digest() const;
  void comb_begin(); // 默认保持寄存器状态（*_1 <- *）
  void comb_ready();
  void comb_ftq_pc_req();
  void comb_commit();
  void comb_complete();
  void comb_fire();
  void comb_branch();
  void comb_flush();

  struct Wires *wires = nullptr;  // signals shared with the other back-end modules
  RobOutRef out;

  // 状态
  RobStoredEntryRef entry[ROB_BANK_NUM][ROB_LINE_NUM];
  reg<clog2(ROB_LINE_NUM)> enq_ptr;
  reg<clog2(ROB_LINE_NUM)> deq_ptr;
  reg<1> enq_flag;
  reg<1> deq_flag;

  RobStoredEntryRef entry_1[ROB_BANK_NUM][ROB_LINE_NUM];
  // Slots of entry_1 written this cycle; seq() copies only these back (entry_1 == entry at the start of a cycle).
  static constexpr int kDirtyWords = (ROB_BANK_NUM * ROB_LINE_NUM + 63) / 64;
  uint64_t dirty_[kDirtyWords] = {};
  bool dirty_all_ = false;
  void touch(int bank, int line) {
    const int k = bank * ROB_LINE_NUM + line;
    dirty_[k >> 6] |= 1ull << (k & 63);
  }
  void touch_all() { dirty_all_ = true; }
  wire<clog2(ROB_LINE_NUM)> enq_ptr_1;
  wire<clog2(ROB_LINE_NUM)> deq_ptr_1;
  wire<1> enq_flag_1;
  wire<1> deq_flag_1;

private:
  int stall_cycle = 0;
  bool is_empty() { return (enq_ptr == deq_ptr) && (enq_flag == deq_flag); };
  bool is_full() { return (enq_ptr == deq_ptr) && (enq_flag != deq_flag); };
};
