#include "FrontTop.h"
#include "BackTop.h"
#include "config.h"
#include "front_core.h"
#include "front_module.h"
#include "train_IO.h"
#include "util.h"
#include <cstring>
#include <memory>

FrontTop::FrontTop()
    : core_(std::make_unique<FrontCore>()), read_data_(std::make_unique<FrontReadData>()),
      update_req_(std::make_unique<FrontUpdateRequest>()) {}

// the I-cache's ports (SimCpu binds them on FrontTop; they take effect at the I-cache's next step)
void FrontTop::bind_ports() {
  ICacheTop::Ports p;
  p.ptw_walk = icache_ptw_walk_port;
  p.probe_req = icache_mem_req_port;
  p.probe_resp = icache_mem_resp_port;
  p.read = icache_mem_read_port;
  p.read_q = icache_mem_read_q;
  core_->icache.set_ports(p);
}

FrontTop::~FrontTop() = default;

void FrontTop::init() {
  CsrStatusIO *bound_csr = in.csr_status;
  bind_ports();
  core_->ctx = ctx;
  core_->icache.set_context(ctx);
  in = {};
  out = {};
  back2front = {};
  back2front_1 = {};
  in.csr_status = bound_csr;
  back2front.csr_status = bound_csr;
  back2front_1.csr_status = bound_csr;
  back2front_stall = false;
  back2front_stall_1 = false;
  Assert(ctx != nullptr && "FrontTop::init requires bound SimContext");
  Assert(in.csr_status != nullptr && "FrontTop::init requires bound csr_status");
  in.ctrl.fifo_read_enable = true;
  in.ctrl.reset = true;
  step_bpu();
  in.ctrl.reset = false;
}

void FrontTop::read_core_q() {
  bind_ports();
  core_->read_q(*read_data_);
}

void FrontTop::comb_core() {
  *update_req_ = {};
  core_->comb(in, *read_data_, out, *update_req_);
}

void FrontTop::seq_core() { core_->seq(in, *update_req_); }

void FrontTop::warm_commit(pc_t pc, br_type_t type, bool dir, target_addr_t target, bool rvc) {
  core_->bpu_pipe.warm_commit(pc, type, dir, target, rvc);
}
void FrontTop::warm_finish() { core_->bpu_pipe.warm_finish(); }
void FrontTop::warm_icache(const uint32_t tag[ICACHE_SET_NUM][ICACHE_WAY_NUM], const bool valid[ICACHE_SET_NUM][ICACHE_WAY_NUM]) {
  core_->icache.warm_install(tag, valid);
}
void FrontTop::warm_itlb(const void *entries, int count, int repl_ptr) { core_->icache.warm_install_itlb(entries, count, repl_ptr); }

void FrontTop::seq_back2front() {
  back2front = back2front_1;
  back2front_stall = back2front_stall_1;
}

void FrontTop::restart_at(pc_t pc) {
  in.ctrl.reset = false;
  in.ctrl.fifo_read_enable = false;
  in.redirect.valid = true;
  in.redirect.address = pc;
  back2front.redirect.valid = true;
  back2front.redirect.address = pc;
  back2front_1 = back2front;
  back2front_stall = false;
  back2front_stall_1 = false;
}

void FrontTop::step_bpu() {
  read_core_q();
  comb_core();
  seq_core();
}

// One front-end cycle from the registered back2front: when the back end does not stall (or a refetch is pending) the
// fetch-group FIFO is read (can_read; Front2Back then takes the group from out), then the BPU / FIFOs / I-cache comb.
void FrontTop::comb(CsrStatusIO *csr_status) {
  const bool registered_refetch = static_cast<bool>(back2front.redirect.valid);
  can_read = !back2front_stall || registered_refetch;

  in = back2front;   // (reset is 0 here: only init() runs the front end with reset)
  in.csr_status = csr_status;
  in.ctrl.fifo_read_enable = can_read;
  if (!can_read) in.redirect.valid = false;
  read_core_q();
  comb_core();
}

// The front end's inputs of the next cycle (back2front_1 / back2front_stall_1, registered by seq_back2front): the
// redirect and the BPU training of the back end's commit. Fields not refreshed keep their registered value.
void FrontTop::comb_from_back(BackTop &back) {
  back2front_1 = back2front;
  back2front_stall_1 = back.out.stall;
  back2front_1.ctrl.fifo_read_enable = false;
  back2front_1.csr_status = &back.csr_status;
  back2front_1.redirect.valid = back.out.mispred || back.out.flush;
  back2front_1.redirect.rf_valid = back.out.rf_valid && !back.out.flush;
  back2front_1.redirect.rf_blk = back.out.rf_blk;
  back2front_1.redirect.rf_slot = back.out.rf_slot;
  back2front_1.redirect.rf_type = back.out.rf_type;
  back2front_1.redirect.rf_pc = back.out.rf_pc;
  back2front_1.redirect.rf_is_rvc = back.out.rf_is_rvc;
  back2front_1.ctrl.itlb_flush = back.out.itlb_flush;
  back2front_1.ctrl.fence_i = back.out.fence_i;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    InstInfo *inst = &back.out.commit_entry[i].uop;
    const auto &commit_extra = back.out.commit_extra[i];
    const auto &ftq_info = back.out.commit_ftq[i];
    back2front_1.train[i].valid = back.out.commit_entry[i].valid;
    for (int j = 0; j < 4; j++) {
      back2front_1.train[i].tage_tag[j] = 0;
    }

    if (back2front_1.train[i].valid) {
      back2front_1.train[i].pred_dir = ftq_info.pred_taken;
      back2front_1.train[i].base_pc = ftq_info.pc;
      back2front_1.train[i].is_rvc = ftq_info.is_rvc;
      back2front_1.train[i].actual_dir =
          ((*inst)->type == JAL || (*inst)->type == JALR) ? true : inst->br_taken;
      back2front_1.train[i].target =
          (is_branch((*inst)->type) || (*inst)->type == JAL || (*inst)->type == JALR)
              ? back.out.commit_entry[i].uop.diag_val
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

      back2front_1.train[i].br_type = br_type;
      back2front_1.train[i].alt_pred = ftq_info.alt_pred;
      back2front_1.train[i].pcpn = ftq_info.pcpn;
      for (int j = 0; j < 4; j++) { // TN_MAX
        back2front_1.train[i].tage_idx[j] = ftq_info.tage_idx[j];
        back2front_1.train[i].tage_tag[j] = ftq_info.tage_tag[j];
      }
    }
  }

  if (back.out.mispred || back.out.flush) {
    back2front_1.redirect.address = back.out.redirect_pc;
  }}
