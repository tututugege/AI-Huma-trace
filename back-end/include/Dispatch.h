#pragma once
#include "IO.h"
#include "config.h"

class BackTop;
class Ren;

struct UopPacket {
  int kind;                // UopKind of the uop
  Uop uop; // in the issue wire format
};

// Dispatch. Contract (checked by scripts/comb/comb_graph.py against the code): the signals each phase reads (in)
// and writes (out), and the outputs only this module drives.
//   CONTRACT comb_alloc  in: in.lsu2dis in.rob2dis  out: out.dis2rob
//   CONTRACT comb_dispatch  in: in.dec_bcast in.iss2dis in.iss_awake in.lsu_awake in.rob2dis in.rob_bcast  out: out.dis2iss out.dis2lsu out.dis2ren out.dis2rob
class Dispatch {
private:
  void wake(int i);   // the busy bits of slot i (comb_dispatch)

  Uop make_dis_iss_uop(const Uop &inst, int k) const;
  void apply_wakeup_to_uop(Uop &uop) const;
  int decompose_inst(const Uop &original_inst, UopPacket *out_uops);

  // per slot (PMU): its uops fit the issue queues (evaluated up to the first slot that does not fire)
  bool fit_[DECODE_WIDTH];
  // PMU only (EXP-014): the issue queue that stopped this slot in comb_dispatch (-1: none) and whether it was its enqueue
  // width (true) or a lack of free entries (false)
  int iq_blk[DECODE_WIDTH];
  bool iq_blk_width[DECODE_WIDTH];

  // per port: the slot it was given, -1 if none this cycle
  int stq_port_owner[MAX_STQ_DISPATCH_WIDTH];
  int ldq_port_owner[MAX_LDQ_DISPATCH_WIDTH];


public:
  // Registers: read by the comb phases, updated only by seq (from the registers and this cycle's wires).
  struct Regs {
    Uop inst_r[DECODE_WIDTH];
    reg<1> inst_valid[DECODE_WIDTH];
    reg<1> busy_table[TAG_NUM];
  };
  const Regs &q() const { return q_; }
  Regs &q_mut() { return q_; }   // checkpoint restore only
  void comb_dispatch();  // busy bits, the fire decision per slot, the issue-queue / LSQ requests of the fired slots, the handshake
  friend struct DisDigestAccess;  // lockstep test hook
  Dispatch(SimContext *ctx) { this->ctx = ctx; }
  SimContext *ctx;
  // Outputs: the signals this module drives (it alone).
  struct Out {
    DisIssIO dis2iss;
    DisLsuIO dis2lsu;
    DisRenIO dis2ren;
    DisRobIO dis2rob;
  } out;
  // Inputs: the signals this module reads (const: it cannot write them), bound by BackTop::init.
  struct In {
    const DecBroadcastIO *dec_bcast = nullptr;
    const IssDisIO *iss2dis = nullptr;
    const IssAwakeIO *iss_awake = nullptr;
    const LsuAwakeIO *lsu_awake = nullptr;
    const LsuDisIO *lsu2dis = nullptr;
    const RenDisIO *ren2dis = nullptr;
    const RobDisIO *rob2dis = nullptr;
    const RobBroadcastIO *rob_bcast = nullptr;
  } in;

  void init();
  void comb_alloc();
  void seq();
  // PMU (EXP-013): classify this cycle's DECODE_WIDTH dispatch slots into the event vector (read-only, after comb_dispatch)
  void pmu_slots(struct PmuVec &ev, bool recovering, bool recovering_by_flush, int &be_slots) const;

private:
  Regs q_;
};
