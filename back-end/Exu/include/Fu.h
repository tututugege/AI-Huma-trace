#pragma once
#include <cstdlib>
#include <cstdio>
#include "ExuTypes.h"
#include "IO.h"
#include "config.h"
#include "trace_hook.h"
#include "MsData.h"
#include "util.h"
#include <deque>

// Functional-unit timing model. The value datapath is gone (results are facts of the
// trace); what is left per unit is (a) when it can accept an instruction, (b) when its
// result completes, (c) what a flush / branch-mask clear does to the instructions in it.
//   SINGLE : one cycle, always ready unless its result is not consumed (ALU/AGU/SDU/BRU/CSR)
//   PIPE   : fully pipelined with a fixed latency (MUL)
//   ITER   : one instruction at a time, latency depends on the operation (DIV, FDIV)
enum class FuKind : uint8_t { ALU, MUL, DIV, AGU, SDU, BRU, CSR, FMA, FMISC, FDIV, I2F };   // FMA / FMISC / I2F: pipelined FP units (F/D); FDIV: iterative

class Fu {
public:
  struct FuInput {
    wire<1> en = 0;
    wire<1> consume = 0;
    wire<1> flush = 0;
    wire<BR_MASK_WIDTH> flush_mask = 0;
    wire<BR_MASK_WIDTH> clear_mask = 0;
    ExuInst inst;
  };

  struct FuOutput {
    wire<1> ready = 0;
    wire<1> complete = 0;
    ExuInst inst;
  };

  FuInput in;
  FuOutput out;
  const FuKind kind;
  const int issue_port_idx;

  Fu(FuKind k, int port_idx) : kind(k), issue_port_idx(port_idx) {
    out.ready = true;
    pipe_depth = pipe_latency();
  }

  void comb_begin() {
    switch (mode()) {
    case Mode::SINGLE:
      out.complete = false;
      break;
    case Mode::PIPE:
      pipeline_1 = pipeline;
      out.ready = pipeline.size() < pipe_depth;
      out.complete = pipe_done(pipeline);
      if (out.complete) out.inst = pipeline.front().uop;
      break;
    case Mode::ITER:
      cur_1 = cur;
      done_1 = done;
      busy_1 = busy;
      out.ready = !busy;
      out.complete = busy && done <= sim_time;
      if (out.complete) out.inst = cur.inst;
      break;
    }
  }

  void seq() {
    if (mode() == Mode::PIPE) {
      pipeline = pipeline_1;
    } else if (mode() == Mode::ITER) {
      cur = cur_1;
      done = done_1;
      busy = busy_1;
    }
  }

  // flush / branch-mask clear applied to the instructions held in the unit
  void comb_ctrl() {
    const wire<BR_MASK_WIDTH> all = ~wire<BR_MASK_WIDTH>(0);
    if (mode() == Mode::PIPE) {
      if (in.flush) {
        if (in.flush_mask == all) {
          pipeline_1.clear();
        } else {
          for (auto it = pipeline_1.begin(); it != pipeline_1.end();) {
            it = (it->uop.br_mask & in.flush_mask) ? pipeline_1.erase(it) : it + 1;
          }
        }
      }
      if (in.clear_mask) {
        for (auto &e : pipeline_1) e.uop.br_mask &= ~in.clear_mask;
      }
    } else if (mode() == Mode::ITER) {
      if (in.flush && busy_1 &&
          (in.flush_mask == all || (cur_1.inst.br_mask & in.flush_mask) != 0)) {
        busy_1 = false;
        done_1 = 0;
      }
      if (in.clear_mask && busy_1) cur_1.inst.br_mask &= ~in.clear_mask;
    }
  }

  void comb_issue() {
    switch (mode()) {
    case Mode::SINGLE:
      if (in.en) {
        ExuInst inst = in.inst;
        compute(inst);
        out.complete = true;
        out.inst = inst;
      }
      break;
    case Mode::PIPE:
      if (in.en && pipeline_1.size() < pipe_depth) {
        ExuInst inst = in.inst;
        compute(inst);
        pipeline_1.push_back({inst, sim_time + int(pipe_latency()) - 1});
      }
      out.complete = pipe_done(pipeline_1);
      if (out.complete) out.inst = pipeline_1.front().uop;
      break;
    case Mode::ITER:
      if (in.en && !busy_1) {
        ExuInst inst = in.inst;
        compute(inst);
        cur_1.inst = inst;
        done_1 = sim_time + iter_latency(inst);
        busy_1 = true;
      }
      out.complete = busy_1 && done_1 <= sim_time;
      if (out.complete) out.inst = cur_1.inst;
      break;
    }
  }

  void comb_consume() {
    switch (mode()) {
    case Mode::SINGLE:
      out.ready = !in.en || in.consume;
      break;
    case Mode::PIPE:
      if (in.consume && pipe_done(pipeline_1)) pipeline_1.pop_front();
      out.ready = pipeline_1.size() < pipe_depth;
      break;
    case Mode::ITER:
      if (in.consume && busy_1 && done_1 <= sim_time) {
        busy_1 = false;
        done_1 = 0;
      }
      out.ready = !busy_1;
      break;
    }
  }

private:
  enum class Mode : uint8_t { SINGLE, PIPE, ITER };
  struct PipeEntry {
    ExuInst uop;
    int64_t done_cycle;
  };
  struct Held {
    ExuInst inst;
  };

  // latency of a pipelined unit (it is also the depth: one new uop per cycle)
  size_t pipe_latency() const {
    switch (kind) {
    case FuKind::MUL: return MUL_MAX_LATENCY <= 0 ? 1 : MUL_MAX_LATENCY;
    case FuKind::FMA: return FP_FMA_LAT;
    case FuKind::FMISC: return FP_MISC_LAT;
    case FuKind::I2F: return FP_I2F_LAT;
    default: return 0;
    }
  }

  Mode mode() const {
    switch (kind) {
    case FuKind::MUL:
    case FuKind::FMA:
    case FuKind::FMISC:
    case FuKind::I2F: return Mode::PIPE;
    case FuKind::DIV:
    case FuKind::FDIV: return Mode::ITER;
    default: return Mode::SINGLE;
    }
  }

  static bool pipe_done(const std::deque<PipeEntry> &p) {
    return !p.empty() && p.front().done_cycle <= sim_time;
  }

  // Latency of an iterative unit: the divider takes DIV_MAX_LATENCY, the FP divide / square root unit its S / D latency.
  int iter_latency(const ExuInst &inst) const {
    if (kind == FuKind::DIV) return div_latency(inst->is_word);
    if (kind == FuKind::FDIV) {   // F/D divide / square root: funct7[1:0] = fmt (1 = D), funct5 0x0b = fsqrt
      const bool d = inst->func7 & 1, sq = (inst->func7 >> 2) == 0x0b;
      return sq ? (d ? FP_FSQRT_D_LAT : FP_FSQRT_S_LAT) : (d ? FP_FDIV_D_LAT : FP_FDIV_S_LAT);
    }
    return 1;   // (only DIV and FDIV are iterative)
  }

  // Facts of the trace stand in for the datapath: the AGU address, the store data and the
  // branch outcome (checked against the FTQ prediction) come from the trace bridge.
  void compute(ExuInst &inst) const {
    switch (kind) {
    case FuKind::AGU:
      inst.result = 0;
      if (const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx)) inst.result = f->vaddr;
      break;
    case FuKind::SDU:
      inst.result = 0;
      if (const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx)) inst.result = f->rs2_value;
      break;
    case FuKind::BRU:
      resolve_branch(inst);
      break;
    default:
      break;
    }
    if (FP) fp_trap(inst);
    if (msd::on) {
      msd::self_check(inst);
      msd::exec(0, inst);
    }
    if (tbhook::poison && (inst.op == UOP_ADD || inst.op == UOP_MUL ||
                           inst.op == UOP_DIV || inst.op == UOP_FMA || inst.op == UOP_FMISC || inst.op == UOP_FDIV || inst.op == UOP_I2F))
      inst.result ^= 0xA5C3A5C3u;
  }

  // F/D: the function model decides what traps (mstatus.FS = Off, reserved rounding mode, ...): an FP-class uop whose fact says
  // "trapped with an illegal instruction" becomes the illegal-instruction exception.  A page fault of flw/fld/fsw/fsd is the LSU's.
  static void fp_trap(ExuInst &inst) {
    const UopType op = inst.op;
    const bool fpq = op == UOP_FMA || op == UOP_FMISC || op == UOP_FDIV || op == UOP_I2F;
    const bool fpmem = (op == UOP_LOAD || op == UOP_STA || op == UOP_STD) && (inst->dest_fp || inst->src2_fp);
    const bool fpcsr = op == UOP_CSR && inst->csr_idx >= 1 && inst->csr_idx <= 3;   // fflags / frm / fcsr: illegal while FS = Off
    if (!fpq && !fpmem && !fpcsr) return;
    const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx);
    if (f && f->trapped && f->blocked == tb::Blocked::None && f->trap_cause == 2) inst.illegal_inst = true;
  }

  static void resolve_branch(ExuInst &inst) {
    const xlen_t inst_pc = inst->pc;
    xlen_t pc_br = inst_pc + static_cast<xlen_t>(static_cast<sxlen_t>(static_cast<int32_t>(inst->imm)));   // (imm is a 32-bit sign-extended immediate)
    bool br_taken = false;
    if (const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx)) {
      if (inst.op == UOP_BR) {
        br_taken = f->br_taken;
      } else if (inst.op == UOP_JUMP) {
        br_taken = true;
        pc_br = f->next_pc;  // jal/jalr target
      }
    } else if (inst.op == UOP_JUMP) {
      br_taken = true;  // killed instruction: no fact, outcome unobservable
    }
    const bool pred_taken = inst->ftq_pred_taken;
    const xlen_t pred_target = pred_taken ? static_cast<xlen_t>(inst->ftq_next_pc) : inst_pc + 4;
    inst.mispred = !((br_taken && pred_taken && pred_target == pc_br) ||
                     (!br_taken && !pred_taken));
    inst.br_taken = br_taken;
    inst.diag_val = br_taken ? pc_br : (inst_pc + 4);
  }

  // PIPE state
  size_t pipe_depth = 0;
  std::deque<PipeEntry> pipeline, pipeline_1;
  // ITER state
  Held cur, cur_1;
  int64_t done = 0, done_1 = 0;
  bool busy = false, busy_1 = false;
};
