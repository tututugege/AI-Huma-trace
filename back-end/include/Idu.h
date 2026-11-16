#pragma once
#include "IO.h"
#include "PreIduQueue.h"
#include "config.h"
class SimContext;

// Decode. Contract (checked by scripts/comb/comb_graph.py against the code): the signals each phase reads (in)
// and writes (out), and the outputs only this module drives.
//   CONTRACT comb_decode  in: in.pre_issue  out: out.dec2ren out.dec_bcast
//   CONTRACT comb_fire  in: in.exu2id in.ren2dec in.rob_bcast  out: out.idu_consume
class Idu {
public:
  // Registers: read by the comb phases, updated only by seq (from the registers and this cycle's wires).
  struct Regs {
    reg<BR_MASK_WIDTH> now_br_mask;
    reg<BR_MASK_WIDTH> br_mask_cp[MAX_BR_NUM];
    reg<1> tag_vec[MAX_BR_NUM];
    ExuIdIO br_latch;
  };
  const Regs &q() const { return q_; }
  Regs &q_mut() { return q_; }   // checkpoint restore only
  void comb_decode();  // branch resolution (mispredict broadcast), then decode
  Idu(SimContext *ctx, int max_br = 1) {
    this->ctx = ctx;
    this->max_br_per_cycle = max_br;
  }
  SimContext *ctx;
  int max_br_per_cycle;
  // Outputs: the signals this module drives (it alone).
  struct Out {
    DecRenIO dec2ren;
    DecBroadcastIO dec_bcast;
    IduConsumeIO idu_consume;
  } out;
  // Inputs: the signals this module reads (const: it cannot write them), bound by BackTop::init.
  struct In {
    const ExuIdIO *exu2id = nullptr;
    const PreIssueIO *pre_issue = nullptr;
    const RenDecIO *ren2dec = nullptr;
    const RobBroadcastIO *rob_bcast = nullptr;
  } in;
  void decode(Uop &uop, uint32_t inst);

  void init();
  void comb_fire();  // handshake and branch tag advance
  void seq();              // clock edge

  // registers

  // next-cycle values

private:
  // phases of one cycle, run in the order of the methods above
  void branch();
  void decode();

private:
  wire<BR_TAG_WIDTH> alloc_tag[DECODE_WIDTH] = {};   // branch tags picked in comb_decode, committed in comb_fire
  Regs q_;
};
