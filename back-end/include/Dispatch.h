#pragma once
#include "IO.h"
#include "config.h"

class BackTop;
class Ren;

struct UopPacket {
  int iq_id;               // target IQ
  Uop uop; // in the issue wire format
};

// Dispatch. Contract (checked by scripts/comb/comb_graph.py against the code): the Wires each phase reads (in)
// and writes (out), and the outputs only this module drives.
//   CONTRACT comb_alloc     in: dec_bcast lsu2dis rob2dis                                        out: dis2lsu dis2rob
//   CONTRACT comb_dispatch  in: dec_bcast iss2dis iss_awake prf_awake ren2dis rob2dis rob_bcast  out: dis2iss dis2lsu dis2ren dis2rob
//   CONTRACT owns: dis2iss dis2lsu dis2ren dis2rob
class Dispatch {
private:
  // phases of one cycle, run in the order of the methods above
  void wake();
  void dispatch();
  void fire();
  void pipeline();

  Uop make_dis_iss_uop(const Uop &inst, int k) const;
  bool is_preg_woken(wire<PRF_IDX_WIDTH> preg) const;
  void apply_wakeup_to_uop(Uop &uop) const;
  int decompose_inst(const Uop &original_inst, UopPacket *out_uops);

  Uop inst_alloc[DECODE_WIDTH];

  // per instruction: dispatched (comb_dispatch -> comb_fire)
  bool dispatch_success_flags[DECODE_WIDTH];
  // PMU only (EXP-014): the issue queue that stopped this slot in comb_dispatch (-1: none) and whether it was its enqueue
  // width (true) or a lack of free entries (false)
  int iq_blk[DECODE_WIDTH];
  bool iq_blk_width[DECODE_WIDTH];

  // per port: the slot it was given, -1 if none this cycle
  int stq_port_owner[MAX_STQ_DISPATCH_WIDTH];
  int ldq_port_owner[MAX_LDQ_DISPATCH_WIDTH];

  struct DispatchCache {
    int count;                     // number of uops
    int iq_ids[MAX_UOPS_PER_INST]; // target IQ ids
  };

  // passed from comb_dispatch to comb_fire
  DispatchCache dispatch_cache[DECODE_WIDTH];

public:
  // Registers: q (read in comb through const Regs &q, written only by seq) and d (their next value, written by comb;
  // d == q at the start of every cycle). seq: q = d.
  struct Regs {
    Uop inst_r[DECODE_WIDTH];
    reg<1> inst_valid[DECODE_WIDTH];
    reg<1> busy_table[TAG_NUM];
  };
  Regs d;
  const Regs &q() const { return q_; }
  Regs &q_mut() { return q_; }   // checkpoint restore only
  void comb_dispatch();  // wakeups, dispatch to the issue queues / LSU / ROB, the handshake, the next pipeline register
  friend struct DisDigestAccess;  // lockstep test hook
  Dispatch(SimContext *ctx) { this->ctx = ctx; }
  SimContext *ctx;
  struct Wires *wires = nullptr;  // signals shared with the other back-end modules

  void init();
  void comb_alloc();
  void seq();
  // PMU (EXP-013): classify this cycle's DECODE_WIDTH dispatch slots into the event vector (read-only, after comb_fire)
  void pmu_slots(struct PmuVec &ev, bool recovering, bool recovering_by_flush, int &be_slots) const;

private:
  Regs q_;
};
