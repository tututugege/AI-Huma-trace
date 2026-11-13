#pragma once
#include <cstdlib>
#include <cstdio>
#include "ExuTypes.h"
#include "IO.h"
#include "config.h"
#include "trace_hook.h"
#include "MsData.h"
#include "util.h"
#include <vector>

// Functional-unit timing model. The value datapath is gone (results are facts of the trace); what is left per unit is
// (a) when it can accept an instruction, (b) when its result completes, (c) what a flush / branch-mask clear does to the
// instructions in it. A unit is plain state (no object, no in/out copies); Exu walks only the units that hold something.
//   SINGLE : one cycle, completes in the cycle after issue (ALU/AGU/SDU/BRU/CSR)
//   PIPE   : fully pipelined, fixed latency: a ring of (uop, completion cycle)  (MUL, FMA, FMISC, I2F)
//   ITER   : one instruction at a time, latency depends on the operation: busy until `done`  (DIV, FDIV)
enum class FuKind : uint8_t { ALU, MUL, DIV, AGU, SDU, BRU, CSR, FMA, FMISC, FDIV, I2F };
enum class FuMode : uint8_t { SINGLE, PIPE, ITER };

inline FuMode fu_mode(FuKind k) {
  switch (k) {
  case FuKind::MUL: case FuKind::FMA: case FuKind::FMISC: case FuKind::I2F: return FuMode::PIPE;
  case FuKind::DIV: case FuKind::FDIV: return FuMode::ITER;
  default: return FuMode::SINGLE;
  }
}

// latency of a pipelined unit (it is also the depth: one new uop per cycle)
inline int fu_pipe_latency(FuKind k) {
  switch (k) {
  case FuKind::MUL: return MUL_MAX_LATENCY <= 0 ? 1 : MUL_MAX_LATENCY;
  case FuKind::FMA: return FP_FMA_LAT;
  case FuKind::FMISC: return FP_MISC_LAT;
  case FuKind::I2F: return FP_I2F_LAT;
  default: return 0;
  }
}

namespace fu {
inline void fp_trap(ExuInst &inst);
inline void resolve_branch(ExuInst &inst);

// Latency of an iterative unit: the divider takes DIV_MAX_LATENCY, the FP divide / square root unit its S / D latency.
inline int iter_latency(FuKind kind, const ExuInst &inst) {
  if (kind == FuKind::DIV) return div_latency(inst->is_word);
  if (kind == FuKind::FDIV) {   // F/D divide / square root: funct7[1:0] = fmt (1 = D), funct5 0x0b = fsqrt
    const bool d = inst->func7 & 1, sq = (inst->func7 >> 2) == 0x0b;
    return sq ? (d ? FP_FSQRT_D_LAT : FP_FSQRT_S_LAT) : (d ? FP_FDIV_D_LAT : FP_FDIV_S_LAT);
  }
  return 1;   // (only DIV and FDIV are iterative)
}

// Facts of the trace stand in for the datapath: the AGU address, the store data and the
// branch outcome (checked against the FTQ prediction) come from the trace bridge.
inline void compute(FuKind kind, ExuInst &inst) {
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
inline void fp_trap(ExuInst &inst) {
  const UopType op = inst.op;
  const bool fpq = op == UOP_FMA || op == UOP_FMISC || op == UOP_FDIV || op == UOP_I2F;
  const bool fpmem = (op == UOP_LOAD || op == UOP_STA || op == UOP_STD) && (inst->dest_fp || inst->src2_fp);
  const bool fpcsr = op == UOP_CSR && inst->csr_idx >= 1 && inst->csr_idx <= 3;   // fflags / frm / fcsr: illegal while FS = Off
  if (!fpq && !fpmem && !fpcsr) return;
  const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx);
  if (f && f->trapped && f->blocked == tb::Blocked::None && f->trap_cause == 2) inst.illegal_inst = true;
}

inline void resolve_branch(ExuInst &inst) {
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
  const xlen_t pred_target = pred_taken ? static_cast<xlen_t>(inst->ftq_next_pc) : inst_pc + ilen_of(inst->is_rvc);
  inst.mispred = !((br_taken && pred_taken && pred_target == pc_br) ||
                   (!br_taken && !pred_taken));
  inst.br_taken = br_taken;
  inst.diag_val = br_taken ? pc_br : (inst_pc + ilen_of(inst->is_rvc));
}

}  // namespace fu
