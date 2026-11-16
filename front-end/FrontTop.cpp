#include "FrontTop.h"
#include "BackTop.h"
#include "config.h"
#include "front_module.h"
#include "train_IO.h"
#include "util.h"
#include <cstring>
#include <memory>

namespace {
inline void sync_icache_ptw_ports(const FrontTop &front) {
  icache_ptw_walk_port = front.icache_ptw_walk_port;
  icache_mem_req_port = front.icache_mem_req_port;
  icache_mem_resp_port = front.icache_mem_resp_port;
  icache_mem_read_port = front.icache_mem_read_port;
  icache_mem_read_q = front.icache_mem_read_q;
}
} // namespace

FrontTop::FrontTop()
    : read_data_(std::make_unique<FrontReadData>()),
      update_req_(std::make_unique<FrontUpdateRequest>()) {}

FrontTop::~FrontTop() = default;

void FrontTop::init() {
  CsrStatusIO *bound_csr = in.csr_status;
  sync_icache_ptw_ports(*this);
  front_set_context(ctx);
  icache_set_context(ctx);
  std::memset(&in, 0, sizeof(in));
  std::memset(&out, 0, sizeof(out));
  std::memset(&back2front, 0, sizeof(back2front));
  std::memset(&back2front_1, 0, sizeof(back2front_1));
  in.csr_status = bound_csr;
  back2front.csr_status = bound_csr;
  back2front_1.csr_status = bound_csr;
  back2front_stall = false;
  back2front_stall_1 = false;
  Assert(ctx != nullptr && "FrontTop::init requires bound SimContext");
  Assert(in.csr_status != nullptr && "FrontTop::init requires bound csr_status");
  in.FIFO_read_enable = true;
  in.reset = true;
  step_bpu();
  in.reset = false;
}

void FrontTop::seq_read_bpu() {
  sync_icache_ptw_ports(*this);
  front_top_seq_read(in, *read_data_);
}

void FrontTop::comb_bpu() {
  sync_icache_ptw_ports(*this);
  *update_req_ = {};
  front_comb_calc(in, *read_data_, out, *update_req_);
}

void FrontTop::seq_write_bpu() {
  sync_icache_ptw_ports(*this);
  front_top_seq_write(in, *update_req_, in.reset);
}

void FrontTop::seq_back2front() {
  back2front = back2front_1;
  back2front_stall = back2front_stall_1;
}

void FrontTop::force_back2front_refetch(pc_t pc) {
  back2front.refetch = true;
  back2front.refetch_address = pc;
  back2front_1 = back2front;
  back2front_stall = false;
  back2front_stall_1 = false;
}

void FrontTop::step_bpu() {
  seq_read_bpu();
  comb_bpu();
  seq_write_bpu();
}

// One front-end cycle from the registered back2front: when the back end does not stall (or a refetch is pending) the
// fetch-group FIFO is read; then the BPU / FIFOs / I-cache comb. to_back: the whole fetch group when it was read (after a
// predicted-taken branch the rest of the group is invalid, unless FE_PAIR continues with the second block), the commit
// credits always.
void FrontTop::comb(CsrStatusIO *csr_status, FrontPreIO &to_back) {
  const bool registered_refetch = static_cast<bool>(back2front.refetch);
  const bool frontend_can_read = !back2front_stall || registered_refetch;

  in = back2front;   // (reset is 0 here: only init() runs the front end with reset)
  in.csr_status = csr_status;

  if (frontend_can_read) {
    in.FIFO_read_enable = true;

    seq_read_bpu();
    comb_bpu();

    bool no_taken = true;
    for (int j = 0; j < BACKEND_FETCH_WIDTH; j++) {
      // FE_PAIR: a fetch group may continue with a second block after a predicted-taken branch (pc discontinuity)
      if (j > 0 && !no_taken && to_back.valid[j - 1] && out.inst_valid[j] && out.pc[j] != out.pc[j - 1] + ilen_of(out.is_rvc[j - 1]) && out.predict_dir[j - 1])
        no_taken = true;
      to_back.valid[j] =
          no_taken && out.FIFO_valid && out.inst_valid[j];
      to_back.pc[j] = out.pc[j];
      to_back.is_rvc[j] = out.is_rvc[j];
      to_back.predict_next_fetch_address[j] =
          out.predict_next_fetch_address;
      to_back.page_fault_inst[j] = out.page_fault_inst[j];
      to_back.inst[j] = out.instructions[j];

      to_back.predict_dir[j] = out.predict_dir[j];
      if (to_back.valid[j] && out.predict_dir[j])
        no_taken = false;
    }
    to_back.meta = out.meta;
    to_back.front_stall = static_cast<bool>(out.commit_stall); to_back.front_upd_free = out.commit_upd_free; to_back.front_tq_free = out.commit_tq_free;
  } else {
    in.FIFO_read_enable = false;
    in.refetch = false;
    seq_read_bpu();
    comb_bpu();
    to_back.front_stall = static_cast<bool>(out.commit_stall); to_back.front_upd_free = out.commit_upd_free; to_back.front_tq_free = out.commit_tq_free;
  }}

// The front end's inputs of the next cycle (back2front_1 / back2front_stall_1, registered by seq_back2front): the
// redirect and the BPU training of the back end's commit. Fields not refreshed keep their registered value.
void FrontTop::comb_from_back(BackTop &back) {
  back2front_1 = back2front;
  back2front_stall_1 = back.out.stall;
  back2front_1.FIFO_read_enable = false;
  back2front_1.csr_status = &back.csr_status;
  back2front_1.refetch = back.out.mispred || back.out.flush;
  back2front_1.rf_valid = back.out.rf_valid && !back.out.flush;
  back2front_1.rf_blk = back.out.rf_blk;
  back2front_1.rf_slot = back.out.rf_slot;
  back2front_1.rf_type = back.out.rf_type;
  back2front_1.rf_pc = back.out.rf_pc;
  back2front_1.rf_is_rvc = back.out.rf_is_rvc;
  back2front_1.itlb_flush = back.out.itlb_flush;
  back2front_1.fence_i = back.out.fence_i;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    InstInfo *inst = &back.out.commit_entry[i].uop;
    const auto &commit_extra = back.rob->out.commit_extra[i];
    const auto &ftq_info = back.ftq_commit_info.resp[i];
    back2front_1.back2front_valid[i] = back.out.commit_entry[i].valid;
    for (int j = 0; j < 4; j++) {
      back2front_1.tage_tag[i][j] = 0;
    }

    if (back2front_1.back2front_valid[i]) {
      back2front_1.predict_dir[i] = ftq_info.pred_taken;
      back2front_1.predict_base_pc[i] = ftq_info.pc;
      back2front_1.update_is_rvc[i] = ftq_info.is_rvc;
      back2front_1.actual_dir[i] =
          ((*inst)->type == JAL || (*inst)->type == JALR) ? true : inst->br_taken;
      back2front_1.actual_target[i] =
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

      back2front_1.actual_br_type[i] = br_type;
      back2front_1.alt_pred[i] = ftq_info.alt_pred;
      back2front_1.pcpn[i] = ftq_info.pcpn;
      for (int j = 0; j < 4; j++) { // TN_MAX = 4 (分支预测相关索引)
        back2front_1.tage_idx[i][j] = ftq_info.tage_idx[j];
        back2front_1.tage_tag[i][j] = ftq_info.tage_tag[j];
      }
    }
  }

  if (back.out.mispred || back.out.flush) {
    back2front_1.refetch_address = back.out.redirect_pc;
  }}
