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
//   ITER   : one instruction at a time, latency depends on the operation (DIV, FPU)
enum class FuKind : uint8_t { ALU, MUL, DIV, AGU, SDU, BRU, CSR, FPU };

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
    pipe_depth = (kind == FuKind::MUL) ? (MUL_MAX_LATENCY <= 0 ? 1 : MUL_MAX_LATENCY) : 0;
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
        pipeline_1.push_back({inst, sim_time + MUL_MAX_LATENCY - 1});
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

  Mode mode() const {
    switch (kind) {
    case FuKind::MUL: return Mode::PIPE;
    case FuKind::DIV:
    case FuKind::FPU: return Mode::ITER;
    default: return Mode::SINGLE;
    }
  }

  static bool pipe_done(const std::deque<PipeEntry> &p) {
    return !p.empty() && p.front().done_cycle <= sim_time;
  }

  // Latency of an iterative unit: the divider takes DIV_MAX_LATENCY, the FPU by operation.
  int iter_latency(const ExuInst &inst) const {
    if (kind == FuKind::DIV) return div_latency(inst->is_word);
    constexpr int FADD = 0b00000, FSUB = 0b00001, FMUL = 0b00010, FDIV = 0b00011;
    constexpr int FSQRT = 0x2C, FCVT_W_S = 0x60, FCVT_S_W = 0x68;
    constexpr int LAT_FADD = 5, LAT_FMUL = 3, LAT_FDIV = 10, LAT_FCVT = 3;
    if (inst.op != UOP_FP) return LAT_FADD;
    switch (inst->func7 >> 2) {
    case FMUL: return LAT_FMUL;
    case FDIV: return LAT_FDIV;
    case FADD:
    case FSUB: return LAT_FADD;
    default:
      if (inst->func7 == FCVT_W_S || inst->func7 == FCVT_S_W) return LAT_FCVT;
      if (inst->func7 == FSQRT) return LAT_FDIV;
      return LAT_FADD;
    }
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
    if (msd::on) {
      msd::self_check(inst);
      msd::exec(0, inst);
    }
    if (tbhook::poison && (inst.op == UOP_ADD || inst.op == UOP_MUL ||
                           inst.op == UOP_DIV || inst.op == UOP_FP))
      inst.result ^= 0xA5C3A5C3u;
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
    { static const bool dg = std::getenv("FE_DIAG") != nullptr; static uint64_t c[4]; static bool rg = false;
      if (dg) { if (!rg) { rg = true; std::atexit([] { std::printf("[FE_DIAG] BRU resolves with a fact: dir-mispredict(pred T, actual NT)=%llu (pred NT, actual T)=%llu target-only mispredict(both taken, wrong target)=%llu ok=%llu\n", (unsigned long long)c[0], (unsigned long long)c[1], (unsigned long long)c[2], (unsigned long long)c[3]); }); }
        if (tbhook::fact_by_rob(inst.rob_idx)) { if (br_taken && pred_taken && pred_target != pc_br) c[2]++; else if (pred_taken && !br_taken) c[0]++; else if (!pred_taken && br_taken) c[1]++; else c[3]++; } } }
    { static int n = std::getenv("FE_PCDBG") ? 30 : 0; static const uint32_t want = std::getenv("FE_PCDBG") ? uint32_t(std::strtoul(std::getenv("FE_PCDBG"), nullptr, 16)) : 0;
      if (n > 0 && inst_pc == want && inst.mispred && tbhook::fact_by_rob(inst.rob_idx)) { n--; std::printf("[PCDBG] pc=%x off=%d pred_taken=%d taken=%d next_pc=%x pc_br=%x mispred=%d\n", inst_pc, (int)inst->ftq_offset, (int)pred_taken, (int)br_taken, (unsigned)inst->ftq_next_pc, pc_br, (int)inst.mispred); } }
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
