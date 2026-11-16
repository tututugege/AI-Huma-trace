#pragma once
#include "IO.h"
#include "config.h"

class BackTop;

// Rename. Contract (checked by scripts/comb/comb_graph.py against the code): the Wires each phase reads (in) and writes
// (out), and the outputs only this module drives.
//   CONTRACT comb_rename  in: -                                               out: ren2dis
//   CONTRACT comb_fire    in: dec2ren dis2ren dec_bcast rob_bcast rob_commit  out: ren2dec
//   CONTRACT owns: ren2dis ren2dec
class Ren {
public:
  void comb_rename();  // allocate the destination registers and rename the group of q.inst_r -> ren2dis
  void comb_fire();    // dispatch handshake: commit the rename, take the next group from decode (d.inst_r) -> ren2dec
  Ren(SimContext *ctx) { this->ctx = ctx; }
  SimContext *ctx;
  struct Wires *wires = nullptr;  // signals shared with the other back-end modules

  void init();
  void seq();

  // Registers: q (read in comb through a const reference, written only by seq) and d (their next value; comb writes
  // it, d == q at the start of every cycle). seq copies d to q.
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
  Regs d;
  static_assert(MAX_BR_NUM <= 64, "ckpt_dirty is a 64-bit row mask");
  uint64_t ckpt_dirty = 0, ckpt_dirty_fp = 0;   // checkpoint rows of d written this cycle (seq copies only those)
  bool cnt_dirty = false;                        // d.arch_cnt changed this cycle
  bool elim_refs_ok(int i) const;

private:
  Regs q_;
  // phases of one cycle, run in the order of the methods above
  void alloc();
  void rename();
  void handshake();
  void pipeline();
};
