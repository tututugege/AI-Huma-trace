// Back end -> front end register (include/Back2Front.h; the RTL's core_back2front.sv).
#include "Back2Front.h"
#include "config.h"
#include "util.h"

void Back2Front::init() { q_ = {}; }

void Back2Front::comb_front() {
  const bool registered_refetch = static_cast<bool>(q_.in.redirect.valid);
  const bool can_read = !q_.stall || registered_refetch;
  *out.can_read = can_read;
  FrontTopIn &fi = *out.front_in;
  fi = q_.in;   // (reset is 0 here: only FrontTop::init() runs the front end with reset)
  fi.ctrl.fifo_read_enable = can_read;
  if (!can_read) fi.redirect.valid = false;
}

// The front end's inputs of the next cycle: the redirect and the BPU training of the back end's commit.
void Back2Front::seq() {
  Back2FrontRegs &d = q_;   // updated in place (fields not refreshed keep their value)
  const auto &bo = *in.back_out;
  d.stall = bo.stall;
  d.in.ctrl.fifo_read_enable = false;
  d.in.redirect.valid = bo.mispred || bo.flush;
  d.in.redirect.rf_valid = bo.rf_valid && !bo.flush;
  d.in.redirect.rf_blk = bo.rf_blk;
  d.in.redirect.rf_slot = bo.rf_slot;
  d.in.redirect.rf_type = bo.rf_type;
  d.in.redirect.rf_pc = bo.rf_pc;
  d.in.redirect.rf_is_rvc = bo.rf_is_rvc;
  d.in.ctrl.itlb_flush = bo.itlb_flush;
  d.in.ctrl.fence_i = bo.fence_i;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    const InstInfo *inst = &bo.commit_entry[i].uop;
    const auto &commit_extra = bo.commit_extra[i];
    const auto &ftq_info = bo.commit_ftq[i];
    d.in.train[i].valid = bo.commit_entry[i].valid;
    for (int j = 0; j < 4; j++) {
      d.in.train[i].tage_tag[j] = 0;
    }

    if (d.in.train[i].valid) {
      d.in.train[i].pred_dir = ftq_info.pred_taken;
      d.in.train[i].base_pc = ftq_info.pc;
      d.in.train[i].is_rvc = ftq_info.is_rvc;
      d.in.train[i].actual_dir =
          ((*inst)->type == JAL || (*inst)->type == JALR) ? true : inst->br_taken;
      d.in.train[i].target =
          (is_branch((*inst)->type) || (*inst)->type == JAL || (*inst)->type == JALR)
              ? bo.commit_entry[i].uop.diag_val
              : ftq_info.pc + ilen_of(ftq_info.is_rvc);
      int br_type = BR_NONCTL;
      if (is_branch((*inst)->type)) {
        br_type = BR_DIRECT;
      }
      if ((*inst)->type == JAL) {
        br_type = BR_JAL;
      }
      if ((*inst)->type == JAL && inst->dest_en && (*inst)->dest_areg == 1) {
        br_type = BR_CALL;
      } else if ((*inst)->type == JALR) {
        if (commit_extra.src1_areg == 1 && (*inst)->dest_areg == 0 &&
            commit_extra.imm == 0)
          br_type = BR_RET;
        else if ((*inst)->dest_areg == 1)
          br_type = BR_ICALL;
        else
          br_type = BR_IDIRECT;
      }

      d.in.train[i].br_type = br_type;
      d.in.train[i].alt_pred = ftq_info.alt_pred;
      d.in.train[i].pcpn = ftq_info.pcpn;
      d.in.train[i].blk_id = ftq_info.blk_id;
      d.in.train[i].ftq_is_last = ftq_info.ftq_is_last;
      for (int j = 0; j < 4; j++) { // TN_MAX
        d.in.train[i].tage_idx[j] = ftq_info.tage_idx[j];
        d.in.train[i].tage_tag[j] = ftq_info.tage_tag[j];
      }
    }
  }

  if (bo.mispred || bo.flush) {
    d.in.redirect.address = bo.redirect_pc;
  }}

void Back2Front::restart_at(pc_t pc) {
  q_.in.redirect.valid = true;
  q_.in.redirect.address = pc;
  q_.stall = false;
}
