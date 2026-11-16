// Front end -> back end (include/Front2Back.h; the RTL's core_front2back.sv).
#include "Front2Back.h"
#include "config.h"
#include "util.h"

void Front2Back::init() {
  q_ = {};
  d = {};
}

void Front2Back::comb() {
  const auto &q = q_;
  auto &o = *to_back;
  const auto &out = *front_out;
  if (q.hold_valid) o = q.hold;   // the held group
  else for (int j = 0; j < BACKEND_FETCH_WIDTH; j++) o.valid[j] = false;   // (no group unless the front end delivers)
  if (*can_read) {
    bool no_taken = true;
    for (int j = 0; j < BACKEND_FETCH_WIDTH; j++) {
      // FE_PAIR: a fetch group may continue with a second block after a predicted-taken branch (pc discontinuity)
      if (j > 0 && !no_taken && o.valid[j - 1] && out.inst_valid[j] && out.pc[j] != out.pc[j - 1] + ilen_of(out.is_rvc[j - 1]) && out.predict_dir[j - 1])
        no_taken = true;
      o.valid[j] = no_taken && out.FIFO_valid && out.inst_valid[j];
      o.pc[j] = out.pc[j];
      o.is_rvc[j] = out.is_rvc[j];
      o.predict_next_fetch_address[j] = out.predict_next_fetch_address;
      o.page_fault_inst[j] = out.page_fault_inst[j];
      o.inst[j] = out.instructions[j];
      o.predict_dir[j] = out.predict_dir[j];
      if (o.valid[j] && out.predict_dir[j]) no_taken = false;
    }
    o.meta = out.meta;
  }
  o.front_stall = static_cast<bool>(out.commit_stall);
  o.front_upd_free = out.commit_upd_free;
  o.front_tq_free = out.commit_tq_free;
}

void Front2Back::comb_hold() {
  d.hold_valid = false;
  if (back_out->mispred || back_out->flush || !back_out->stall) {
    return;
  }
  d.hold_valid = true;
  d.hold = *to_back;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; ++i) {
    if (back_out->fire[i]) {
      d.hold.valid[i] = false;
    }
  }
}

void Front2Back::seq() { q_ = d; }

void Front2Back::restart() {
  q_.hold_valid = false;
  d.hold_valid = false;
}
