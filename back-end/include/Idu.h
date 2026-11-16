#pragma once
#include "IO.h"
#include "PreIduQueue.h"
#include "config.h"
class SimContext;

// Decode. Contract (checked by scripts/comb/comb_graph.py against the code): the Wires each phase reads (in)
// and writes (out), and the outputs only this module drives.
//   CONTRACT comb_decode  in: pre_issue                 out: dec2ren dec_bcast
//   CONTRACT comb_fire    in: exu2id ren2dec rob_bcast  out: idu_consume
//   CONTRACT owns: dec2ren dec_bcast idu_consume
class Idu {
public:
  // Registers: q (read in comb through const Regs &q, written only by seq) and d (their next value, written by comb;
  // d == q at the start of every cycle). seq: q = d.
  struct Regs {
    reg<BR_MASK_WIDTH> now_br_mask;
    reg<BR_MASK_WIDTH> br_mask_cp[MAX_BR_NUM];
    reg<1> tag_vec[MAX_BR_NUM];
    ExuIdIO br_latch;
  };
  Regs d;
  const Regs &q() const { return q_; }
  Regs &q_mut() { return q_; }   // checkpoint restore only
  void comb_decode();  // branch resolution (mispredict broadcast), then decode
  Idu(SimContext *ctx, int max_br = 1) {
    this->ctx = ctx;
    this->max_br_per_cycle = max_br;
  }
  SimContext *ctx;
  int max_br_per_cycle;
  struct Wires *wires = nullptr;  // signals shared with the other back-end modules
  void decode(Uop &uop, uint32_t inst);

  void init();
  void comb_fire();  // 发射握手与分支tag推进
  void seq();              // 时钟跳变，状态更新

  // 状态

  // 下一周期状态

private:
  // phases of one cycle, run in the order of the methods above
  void branch();
  void decode();

private:
  Regs q_;
};
