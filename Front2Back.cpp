// Front end -> back end (include/Front2Back.h; the RTL's core_front2back.sv).
#include "Front2Back.h"
#include "config.h"
#include "util.h"

void Front2Back::init() {
  q_ = {};
}

void Front2Back::comb() {
  const auto &q = q_;
  auto &o = *out.to_back;
  const FrontTopOut &fo = *in.front_out;
  if (q.hold_valid) o = q.hold;   // the held group
  else for (int j = 0; j < BACKEND_FETCH_WIDTH; j++) o.valid[j] = false;   // (no group unless the front end delivers)
  if (*in.can_read) {
    for (int j = 0; j < BACKEND_FETCH_WIDTH; j++) {
      o.valid[j] = fo.FIFO_valid && fo.inst_valid[j];
      o.pc[j] = fo.pc[j];
      o.is_rvc[j] = fo.is_rvc[j];
      o.page_fault_inst[j] = fo.page_fault_inst[j];
      o.pf_inst_hi[j] = fo.pf_inst_hi[j];
      o.inst[j] = fo.instructions[j];
    }
    o.ftq_idx = fo.ftq_idx;
  }
  // Unified FTQ-64 frontend writes pass straight through every cycle (never held on ibuf stall)
  o.ftq_s0 = fo.ftq_s0;
  o.ftq_s2 = fo.ftq_s2;
  o.ftq_pd = fo.ftq_pd;
  o.front_stall = static_cast<bool>(fo.commit_stall);
  o.front_upd_free = fo.commit_upd_free;
  o.front_tq_free = fo.commit_tq_free;
}

// The clock edge: the hold register.
void Front2Back::seq() {
  Regs &d = q_;
  d.hold_valid = false;
  if (in.back_out->mispred || in.back_out->flush || !in.back_out->stall) {
    return;
  }
  d.hold_valid = true;
  d.hold = *out.to_back;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; ++i) {
    if (in.back_out->fire[i]) {
      d.hold.valid[i] = false;
    }
  }
}


void Front2Back::restart() {
  q_.hold_valid = false;
}
