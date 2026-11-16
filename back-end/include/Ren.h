#pragma once
#include "IO.h"
#include "config.h"

class BackTop;

// Rename. Contract (checked by scripts/comb/comb_graph.py against the code): the signals each phase reads (in) and writes
// (out), and the outputs only this module drives.
//   CONTRACT comb_rename  in: -  out: out.ren2dis
//   CONTRACT comb_fire  in: in.dec2ren in.dec_bcast in.dis2ren in.rob_bcast in.rob_commit  out: out.ren2dec
class Ren {
public:
  void comb_rename();  // allocate the destination registers and rename the group of q.inst_r -> ren2dis
  void comb_fire();    // dispatch handshake -> ren2dec
  Ren(SimContext *ctx) { this->ctx = ctx; }
  SimContext *ctx;
  // Outputs: the signals this module drives (it alone).
  struct Out {
    RenDisIO ren2dis;
    RenDecIO ren2dec;
  } out;
  // Inputs: the signals this module reads (const: it cannot write them), bound by BackTop::init.
  struct In {
    const DecRenIO *dec2ren = nullptr;
    const DecBroadcastIO *dec_bcast = nullptr;
    const DisRenIO *dis2ren = nullptr;
    const RobBroadcastIO *rob_bcast = nullptr;
    const RobCommitIO *rob_commit = nullptr;
  } in;

  void init();
  void seq();

  // Registers: read by the comb phases, updated only by seq (from the registers and this cycle's wires).
  struct Regs {
    Uop inst_r[DECODE_WIDTH];                                 // the group waiting for dispatch
    reg<1> inst_valid[DECODE_WIDTH];
    reg<PRF_IDX_WIDTH> arch_RAT[ARF_NUM + 1];
    reg<PRF_IDX_WIDTH> spec_RAT[ARF_NUM + 1];
    reg<PRF_IDX_WIDTH> RAT_checkpoint[MAX_BR_NUM][ARF_NUM + 1];
    int arch_cnt[PRF_NUM];   // move elimination: number of arch_RAT entries that name each physical register (freed at 0)
    reg<PRF_IDX_WIDTH> free_list[PRF_NUM];
    reg<PRF_IDX_WIDTH> free_head;   // speculative head
    reg<PRF_IDX_WIDTH> free_head_commit;
    reg<PRF_IDX_WIDTH> free_tail;
    reg<PRF_IDX_WIDTH> alloc_checkpoint_head[MAX_BR_NUM];
    // F/D: the FP rename state (32 architectural registers, no x0; tags FP_TAG_BASE + 0..FPRF_NUM-1)
    reg<PRF_IDX_WIDTH> fp_arch_RAT[FP_ARF_NUM];
    reg<PRF_IDX_WIDTH> fp_spec_RAT[FP_ARF_NUM];
    reg<PRF_IDX_WIDTH> fp_RAT_checkpoint[MAX_BR_NUM][FP_ARF_NUM];
    reg<PRF_IDX_WIDTH> fp_free_list[FPRF_NUM];
    reg<PRF_IDX_WIDTH> fp_free_head, fp_free_head_commit, fp_free_tail;
    reg<PRF_IDX_WIDTH> fp_alloc_checkpoint_head[MAX_BR_NUM];
  };
  bool elim_refs_ok(int i) const;

private:
  Regs q_;
  // phases of one cycle, run in the order of the methods above
  void alloc();
  void rename();
  void handshake();
};
