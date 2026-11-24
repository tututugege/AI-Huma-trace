#pragma once
#include "IO.h"
#include "config.h"
#include <cstring>

class BackTop;

// One register domain of the renamer: the speculative and architectural RAT and the free
// list (a ring: the speculative head, the commit head, the tail). Integer: x0..x31 and the AMO
// scratch register 32, tags 0..PRF_NUM-1. FP: f0..f31, tags FP_TAG_BASE + 0..FPRF_NUM-1.
template <int NA, int NP> struct RenDomain {
  reg<PRF_IDX_WIDTH> arch[NA], spec[NA];
  reg<PRF_IDX_WIDTH> free_list[NP];
  reg<PRF_IDX_WIDTH> head, head_commit, tail;

  static int dist(int from, int to) { return to >= from ? to - from : to - from + NP; }
  int free_count() const { return dist(head, tail); }
  // identity map of the architectural registers onto tags base + 0..NA-1, the other tags free
  void reset(int base) {
    for (int i = 0; i < NA; i++) arch[i] = spec[i] = base + i;
    for (int i = NA; i < NP; i++) free_list[i - NA] = base + i;
    head = head_commit = 0;
    tail = NP - NA;
  }
  void take() { Assert(head != tail && "Ren: allocate when the free list is empty"); head = (head + 1) % NP; }
  void reclaim(int old) { free_list[tail] = old; tail = (tail + 1) % NP; }   // an old destination back to the free list
  void commit_alloc() { Assert(head_commit != head && "Ren: commit reclaim when no pending spec allocation"); head_commit = (head_commit + 1) % NP; }
  void restore_arch() { std::memcpy(spec, arch, sizeof(spec)); head = head_commit; }   // a flush
};

// Rename. Contract (checked by scripts/comb/comb_graph.py against the code): the signals each phase reads (in) and writes
// (out), and the outputs only this module drives.
//   CONTRACT comb_rename  in: -  out: out.ren2dis out.ren2rob
//   CONTRACT comb_fire  in: in.dec_bcast in.dis2ren in.rob_bcast  out: out.ren2dec
class Ren {
public:
  struct RenLogEntry {
    reg<1> dest_en = 0;
    reg<1> dest_fp = 0;
    reg<AREG_IDX_WIDTH> dest_areg = 0;
    reg<PRF_IDX_WIDTH> dest_preg = 0;
    reg<PRF_IDX_WIDTH> old_dest_preg = 0;
    reg<1> me_elim = 0;
  };
  struct RenCkptEntry {
    bool valid = false;
    int rob_idx = 0;
    reg<PRF_IDX_WIDTH> x_spec[ARF_NUM + 1] = {};
    reg<PRF_IDX_WIDTH> f_spec[FP_ARF_NUM] = {};
    reg<PRF_IDX_WIDTH> x_head = 0;
    reg<PRF_IDX_WIDTH> f_head = 0;
  };

  void comb_rename();  // allocate the destination registers and rename the group of q.inst_r -> ren2dis
  void comb_fire();    // dispatch handshake -> ren2dec
  Ren(SimContext *ctx) { this->ctx = ctx; }
  SimContext *ctx;
  // Outputs: the signals this module drives (it alone).
  struct Out {
    RenDisIO ren2dis;
    RenDecIO ren2dec;
    RenRobIO ren2rob;
  } out;
  // Inputs: the signals this module reads (const: it cannot write them), bound by BackTop::init.
  struct In {
    const DecRenIO *dec2ren = nullptr;
    const DecBroadcastIO *dec_bcast = nullptr;
    const DisRenIO *dis2ren = nullptr;
    const RobBroadcastIO *rob_bcast = nullptr;
    const RobCommitIO *rob_commit = nullptr;   // (seq: the committed destinations)
  } in;

  void init();
  void seq();

  // Registers: read by the comb phases, updated only by seq (from the registers and this cycle's wires).
  using IntDomain = RenDomain<ARF_NUM + 1, PRF_NUM>;
  using FpDomain = RenDomain<FP_ARF_NUM, FPRF_NUM>;
  struct Regs {
    Uop s1_inst[DECODE_WIDTH];  // Stage 1 pre-rename register (IDU -> S1 -> S2)
    reg<1> s1_valid[DECODE_WIDTH];
    Uop inst_r[DECODE_WIDTH];   // Stage 2 group waiting for dispatch
    reg<1> inst_valid[DECODE_WIDTH];
    IntDomain x;
    FpDomain f;
    int arch_cnt[PRF_NUM];      // move elimination: number of x.arch entries that name each integer tag (freed at 0)
    // Sparse checkpoint + local ROB-indexed rename log + 8-wide forward walk
    RenLogEntry ren_log[ROB_NUM];
    int ren_rob_head = 0;
    int ren_rob_tail = 0;
    int ren_log_used = 0;
    RenCkptEntry ckpt[REN_CKPT_NUM];
    int ckpt_head = 0;
    int ckpt_tail = 0;
    int ckpt_count = 0;
    bool walk_active = false;
    int walk_ptr = 0;
    int walk_rem = 0;
  };

private:
  Regs q_;
  // wires of this cycle (comb_rename / comb_fire -> seq)
  wire<PRF_IDX_WIDTH> alloc_reg_[DECODE_WIDTH];   // the destination tag taken from the free list (0: none)
  bool elim_[DECODE_WIDTH];                       // move elimination: renamed away, takes no free tag
  bool fire_[DECODE_WIDTH];                       // the slot went to dispatch
  bool s2_ready_ = false;                         // Stage 2 ready to accept next group from Stage 1
  bool elim_refs_ok(int i) const;
};
