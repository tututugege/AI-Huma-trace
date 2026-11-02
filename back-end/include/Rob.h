#pragma once
#include "IO.h"
#include "config.h"

struct RobCommitExtra {
  wire<AREG_IDX_WIDTH> src1_areg;
  wire<32> imm;

  RobCommitExtra() { std::memset(this, 0, sizeof(RobCommitExtra)); }
};

struct RobOut {
  RobDisIO *rob2dis;
  RobCsrIO *rob2csr;
  RobCommitIO *rob_commit;
  RobBroadcastIO *rob_bcast;
  FtqRobPcReqIO *ftq_pc_req;
  RobCommitExtra commit_extra[COMMIT_WIDTH];
};

struct RobIn {
  DisRobIO *dis2rob;
  CsrRobIO *csr2rob;
  LsuRobIO *lsu2rob;
  DecBroadcastIO *dec_bcast;
  ExuRobIO *exu2rob;
  FtqRobPcRespIO *ftq_pc_resp;
  wire<1> *front_stall;
};

using RobStoredInst = Uop;

struct RobStoredEntry {
  wire<1> valid;
  RobStoredInst uop;
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

  RobIn in;
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

private:
  int stall_cycle = 0;
  bool is_empty() { return (enq_ptr == deq_ptr) && (enq_flag == deq_flag); };
  bool is_full() { return (enq_ptr == deq_ptr) && (enq_flag != deq_flag); };
};
