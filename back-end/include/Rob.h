#pragma once
#include "IO.h"
#include "config.h"
#include <cstdlib>

struct RobCommitExtra {
  wire<AREG_IDX_WIDTH> src1_areg;
  wire<32> imm;

  RobCommitExtra() { std::memset(this, 0, sizeof(RobCommitExtra)); }
};

struct RobOut {
  RobCommitExtra commit_extra[COMMIT_WIDTH];
};

// Compact ROB (EXP-028, CFG_ROB_COMPACT=1): entries are allocated one after another (rob_idx = ring position,
// bank = rob_idx % 8, line = rob_idx / 8, same storage), a dispatch group no longer takes a whole line, commit takes the
// longest completed prefix from the head (max COMMIT_WIDTH, cut by the BPU commit credits), a
// mispredict pulls the tail back to the entry after the branch.
constexpr bool rob_compact() { return ROB_COMPACT != 0; }

struct RobStoredEntry {
  wire<1> valid;
  Uop uop;
};

class Rob {
public:
  Rob(SimContext *ctx) { this->ctx = ctx; }
  SimContext *ctx;
  void init();
  void seq();
  void comb_begin(); // 默认保持寄存器状态（*_1 <- *）
  void comb_ready();
  void comb_ftq_pc_req();
  void comb_commit();
  void comb_complete();
  void comb_fire();
  void comb_branch();
  void comb_flush();
  // PMU (EXP-013): the oldest incomplete instruction of the head line (the one that blocks commit), if any
  bool pmu_head_incomplete(Uop &out, int &occupied_entries) const;

  struct Wires *wires = nullptr;  // signals shared with the other back-end modules
  RobOut out;

  // 状态
  RobStoredEntry entry[ROB_BANK_NUM][ROB_LINE_NUM];
  reg<clog2(ROB_LINE_NUM)> enq_ptr;
  reg<clog2(ROB_LINE_NUM)> deq_ptr;
  reg<1> enq_flag;
  reg<1> deq_flag;

  RobStoredEntry entry_1[ROB_BANK_NUM][ROB_LINE_NUM];
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

  // compact mode state (head entry, number of valid entries, wrap flag of the head)
  int c_head = 0, c_count = 0; bool c_hflag = false;
  int c_head_1 = 0, c_count_1 = 0; bool c_hflag_1 = false;
  // control-flow instructions / fetch-block ends committed last cycle (still on their way to the BPU queues)
  int c_prev_ctl = 0, c_prev_blk = 0, c_prev_ctl_1 = 0, c_prev_blk_1 = 0;
  RobStoredEntry &cent(int idx) { return entry[idx % ROB_BANK_NUM][idx / ROB_BANK_NUM]; }
  RobStoredEntry &cent_1(int idx) { return entry_1[idx % ROB_BANK_NUM][idx / ROB_BANK_NUM]; }
  void ctouch(int idx) { touch(idx % ROB_BANK_NUM, idx / ROB_BANK_NUM); }
  void comb_commit_compact();
  void single_actions(const Uop &u);  // what a single-retired instruction triggers (flush, trap, CSR, fence, ...)

private:
  int stall_cycle = 0;
  bool is_empty() { return rob_compact() ? c_count == 0 : (enq_ptr == deq_ptr) && (enq_flag == deq_flag); };
  bool is_full() { return rob_compact() ? c_count == ROB_NUM : (enq_ptr == deq_ptr) && (enq_flag != deq_flag); };
};
