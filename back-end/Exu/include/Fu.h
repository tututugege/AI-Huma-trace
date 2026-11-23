#pragma once
#include <cstdlib>
#include <cstdio>
#include "ExuTypes.h"
#include "trace_win.h"
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

// Variable latency of the Radix-16 (dual Radix-4, 4 bits/cycle) early-out integer divider:
//   - divisor == 0 or |dividend| < |divisor|: 2 cycles (accept computes special result, next cycle wakes & blocks port 1 ALU, then wb)
//   - otherwise: diff = clz(|divisor|) - clz(|dividend|) in 0..63 (0..31 for *W), iters = (diff >> 2) + 1 in 1..16, latency = 1 + iters (2..17).
inline int div_radix16_latency(uint32_t rob_idx, uint32_t func3, bool word) {
  const tb::Fact *f = tbhook::fact_by_rob(rob_idx);
  if (!f) return 2;
  const bool is_signed = !(func3 & 1u);
  uint64_t a_mag, b_mag;
  if (word) {
    const int32_t as = static_cast<int32_t>(f->rs1_value);
    const int32_t bs = static_cast<int32_t>(f->rs2_value);
    const uint32_t au = static_cast<uint32_t>(f->rs1_value);
    const uint32_t bu = static_cast<uint32_t>(f->rs2_value);
    a_mag = is_signed ? static_cast<uint64_t>(as < 0 ? (0u - au) : au) : static_cast<uint64_t>(au);
    b_mag = is_signed ? static_cast<uint64_t>(bs < 0 ? (0u - bu) : bu) : static_cast<uint64_t>(bu);
  } else {
    const int64_t as = static_cast<int64_t>(f->rs1_value);
    const int64_t bs = static_cast<int64_t>(f->rs2_value);
    const uint64_t au = static_cast<uint64_t>(f->rs1_value);
    const uint64_t bu = static_cast<uint64_t>(f->rs2_value);
    a_mag = is_signed ? (as < 0 ? (0ull - au) : au) : au;
    b_mag = is_signed ? (bs < 0 ? (0ull - bu) : bu) : bu;
  }
  if (b_mag == 0 || a_mag < b_mag) return 2;
  const int diff = __builtin_clzll(b_mag) - __builtin_clzll(a_mag);
  return 1 + ((diff >> 2) + 1);
}

// Latency of an iterative unit: Radix-16 early-out divider (2..17 cycles), FP divide / square root unit its S / D latency.
inline int iter_latency(FuKind kind, const ExuInst &inst) {
  if (kind == FuKind::DIV) return div_radix16_latency(inst->rob_idx, inst->func3, inst->is_word);
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
    if (const tb::Fact *f = tbhook::fact_by_rob(inst->rob_idx)) inst.result = f->vaddr;
    { static const bool tr = std::getenv("MSD_TRACE") != nullptr;
      if (tr && trace_win::ok()) { const tb::Fact *f = tbhook::fact_by_rob(inst->rob_idx);
        std::fprintf(stderr, "[AGU] cyc=%lld rob=%d op=%d pc=%llx inst=%08x vaddr=%llx rs1=%llx src1=p%d live1=%d\n", (long long)sim_time, (int)inst->rob_idx, (int)inst.op, f ? (unsigned long long)f->pc : 0ull, f ? f->inst : 0u,
                     f ? (unsigned long long)f->vaddr : 0ull, f ? (unsigned long long)f->rs1_value : 0ull, (int)inst->src1_preg, (int)(inst->src1_en && msd::live(inst->src1_preg))); } }
    break;
  case FuKind::SDU:
    inst.result = 0;
    if (const tb::Fact *f = tbhook::fact_by_rob(inst->rob_idx)) inst.result = f->rs2_value;
    break;
  case FuKind::BRU:
    resolve_branch(inst);
    { static const bool tr = std::getenv("MSD_TRACE") != nullptr;
      if (tr && trace_win::ok()) { const tb::Fact *f = tbhook::fact_by_rob(inst->rob_idx);
        std::fprintf(stderr, "[BRU] cyc=%lld rob=%d pc=%llx mispred=%d taken=%d fact_pc=%llx fact_taken=%d rs1=%llx rs2=%llx src1=p%d src2=p%d\n", (long long)sim_time, (int)inst->rob_idx, (unsigned long long)inst->pc, (int)inst.mispred, (int)inst.br_taken,
                     f ? (unsigned long long)f->pc : 0ull, f ? (int)f->br_taken : -1, f ? (unsigned long long)f->rs1_value : 0ull, f ? (unsigned long long)f->rs2_value : 0ull, (int)inst->src1_preg, (int)inst->src2_preg); } }
    break;
  default:
    break;
  }
  fp_trap(inst);
  msd::exec(inst);
}

// F/D: the function model decides what traps (mstatus.FS = Off, reserved rounding mode, ...): an FP-class uop whose fact says
// "trapped with an illegal instruction" becomes the illegal-instruction exception.  A page fault of flw/fld/fsw/fsd is the LSU's.
inline void fp_trap(ExuInst &inst) {
  const UopType op = inst.op;
  const bool fpq = op == UOP_FMA || op == UOP_FMISC || op == UOP_FDIV || op == UOP_I2F;
  const bool fpmem = (op == UOP_LOAD || op == UOP_STA || op == UOP_STD) && (inst->dest_fp || inst->src2_fp);
  const bool fpcsr = op == UOP_CSR && inst->csr_idx >= 1 && inst->csr_idx <= 3;   // fflags / frm / fcsr: illegal while FS = Off
  if (!fpq && !fpmem && !fpcsr) return;
  const tb::Fact *f = tbhook::fact_by_rob(inst->rob_idx);
  if (f && f->trapped && f->blocked == tb::Blocked::None && f->trap_cause == 2) inst.illegal_inst = true;
}

inline void resolve_branch(ExuInst &inst) {
  const xlen_t inst_pc = inst->pc;
  xlen_t pc_br = inst_pc + static_cast<xlen_t>(static_cast<sxlen_t>(static_cast<int32_t>(inst->imm)));   // (imm is a 32-bit sign-extended immediate)
  bool br_taken = false;
  if (const tb::Fact *f = tbhook::fact_by_rob(inst->rob_idx)) {
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
