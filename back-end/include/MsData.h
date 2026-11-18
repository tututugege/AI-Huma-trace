// MsData.h -- value path of the memory-dependence speculative window (EXP-020 / EXP-040).
//
// The model is oracle driven: every value is a fact of the trace.  The RTL executes real data, so a load that passed an older store
// whose address was still unknown returns the STALE byte, its consumers compute with it, a dependent branch can resolve the wrong way, a
// dependent load/store can get a wrong address.  All of that dies at the commit-time flush of the violating load, but it changes timing
// before.  This header keeps (a) a taint table over physical registers (stale value + chain depth), (b) an RV32IMAB / RV64IM evaluator (xlen_t wide) for the
// consumers of a tainted register, (c) the statistics.  The stale value of a load is computed in RealLsuPipe.cpp (it needs the STQ).
// Taint exists from the first (deferred) write-back of a stale load to its second write-back, and is dropped at every ROB flush.
#pragma once
#include "config.h"
#include "trace_win.h"
#include "trace_hook.h"
#include "types.h"
#include <algorithm>
#include <limits>
#include <type_traits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace msd {

// Fixed configuration (the LSU_MS_DATA mask sweep was removed): branches resolve on stale values (RTL-only mispredict redirect),
// loads / stores with a stale base register use the stale address, stale store data goes into the STQ.
struct TV {
  bool t = false;
  uint8_t depth = 0;  // 0 = the stale load itself
  xlen_t v = 0;
  long long until = 0;  // the second write-back of the load put the true value on the bus: consumers that issued up to this cycle still read the stale one
  // the previous stale value of a register that a write-back replaced by another stale value: consumers executing up to cycle `pwhen` (the write-back cycle + 1)
  // still read it (the RTL reads the register file in the grant cycle, a write-back of that very cycle is not visible; found in the Linux window w505)
  bool pt = false;
  uint8_t pdepth = 0;
  xlen_t pv = 0;
  long long pwhen = 0;
};
inline TV taint[TAG_NUM];
inline bool prev_on() { static const bool on = std::getenv("MSD_NOPREV") == nullptr; return on; }   // MSD_NOPREV=1: A/B switch, a replaced stale value is not kept for one more cycle
// the true value reaches the register file one cycle after the write-back bus: a consumer executing in the next cycle still has the stale operand (RTL, found at the Linux cycle 1129708)
inline void release(uint32_t preg) {
  if (preg == 0 || preg >= (uint32_t)TAG_NUM) return;
  if (taint[preg].t) taint[preg].until = (long long)sim_time + 1;
  else taint[preg] = TV{};
}
// what a consumer executing now reads from the register: the previous stale value in the cycle after its replacement
inline TV view(uint32_t preg) {
  const TV &c = taint[preg];
  if (c.pt && (long long)sim_time <= c.pwhen) { TV o; o.t = true; o.v = c.pv; o.depth = c.pdepth; return o; }
  return c;
}
inline bool live(uint32_t preg) { const TV c = view(preg); return c.t && (c.until == 0 || (long long)sim_time <= c.until); }


inline void clear_all() {
  for (auto &t : taint) t = TV{};
}

// ---------------------------------------------------------------------------------------------------------------
// RV32IMAB integer evaluator (stateless): result of instruction word `in` at `pc` with source values a (rs1) and b (rs2).
// Returns false for anything it does not model (CSR, FP, AMO, system).
// ---------------------------------------------------------------------------------------------------------------
using X = xlen_t;
using SX = std::make_signed_t<X>;
constexpr int XL = XLEN;
inline uint32_t mbits(uint32_t v, int hi, int lo) { return (v >> lo) & ((1u << (hi - lo + 1)) - 1); }
inline int32_t msext(uint32_t v, int n) { return (int32_t)(v << (32 - n)) >> (32 - n); }
inline X xs(int32_t v) { return (X)(SX)v; }                  // sign extension of a 32-bit value to XLEN
inline X mimmI(uint32_t i) { return xs(msext(mbits(i, 31, 20), 12)); }
inline X mimmS(uint32_t i) { return xs(msext((mbits(i, 31, 25) << 5) | mbits(i, 11, 7), 12)); }
inline X rolx(X x, X s) { s &= XL - 1; return s ? (x << s) | (x >> (XL - s)) : x; }
inline X rorx(X x, X s) { s &= XL - 1; return s ? (x >> s) | (x << (XL - s)) : x; }
inline X clzx(X a) { return a == 0 ? XL : (XL == 32 ? __builtin_clz((uint32_t)a) : __builtin_clzll((uint64_t)a)); }
inline X ctzx(X a) { return a == 0 ? XL : (XL == 32 ? __builtin_ctz((uint32_t)a) : __builtin_ctzll((uint64_t)a)); }
inline X popx(X a) { return XL == 32 ? __builtin_popcount((uint32_t)a) : __builtin_popcountll((uint64_t)a); }
inline X bswapx(X a) { return XL == 32 ? (X)__builtin_bswap32((uint32_t)a) : (X)__builtin_bswap64((uint64_t)a); }
inline uint32_t clmul(uint32_t a, uint32_t b, int mode) {  // rv32 only: 0 clmul, 1 clmulr, 2 clmulh
  uint64_t r = 0;
  for (int i = 0; i < 32; i++)
    if (b >> i & 1) r ^= (uint64_t)a << i;
  return mode == 0 ? (uint32_t)r : mode == 2 ? (uint32_t)(r >> 32) : (uint32_t)(r >> 31);
}

// funct7 of the shift-immediate group: rv32 compares the 7 bits, rv64 the upper 6 (shamt[5] is bit 25)
#define MS_K(c) (XL == 32 ? (uint32_t)(c) : (uint32_t)(c) >> 1)

inline bool eval_alu(uint32_t in, X pc, X a, X b, X &r) {
  const uint32_t op = in & 0x7f, f3 = mbits(in, 14, 12), f7 = mbits(in, 31, 25), rs2f = mbits(in, 24, 20);
  const uint32_t fx = XL == 32 ? f7 : f7 >> 1;   // funct7 / funct6 of the shift-immediate group
  if (op == 0x37) { r = xs((int32_t)(in & 0xfffff000u)); return true; }
  if (op == 0x17) { r = pc + xs((int32_t)(in & 0xfffff000u)); return true; }
  if (op == 0x13) {  // OP-IMM
    const X imm = mimmI(in);
    const uint32_t sh = XL == 32 ? rs2f : mbits(in, 25, 20);
    switch (f3) {
    case 0: r = a + imm; return true;
    case 2: r = (SX)a < (SX)imm; return true;
    case 3: r = a < imm; return true;
    case 4: r = a ^ imm; return true;
    case 6: r = a | imm; return true;
    case 7: r = a & imm; return true;
    case 1:
      if (fx == 0x00) { r = a << sh; return true; }
      if (fx == MS_K(0x14)) { r = a | ((X)1 << sh); return true; }   // bseti
      if (fx == MS_K(0x24)) { r = a & ~((X)1 << sh); return true; }  // bclri
      if (fx == MS_K(0x34)) { r = a ^ ((X)1 << sh); return true; }   // binvi
      if (fx == MS_K(0x30)) {
        if (sh == 0) { r = clzx(a); return true; }
        if (sh == 1) { r = ctzx(a); return true; }
        if (sh == 2) { r = popx(a); return true; }
        if (sh == 4) { r = (X)(SX)(int8_t)(a & 0xff); return true; }
        if (sh == 5) { r = (X)(SX)(int16_t)(a & 0xffff); return true; }
      }
      return false;
    case 5:
      if (fx == 0x00) { r = a >> sh; return true; }
      if (fx == MS_K(0x20)) { r = (X)((SX)a >> sh); return true; }
      if (fx == MS_K(0x30)) { r = rorx(a, sh); return true; }
      if (fx == MS_K(0x24)) { r = (a >> sh) & 1; return true; }  // bexti
      if (in >> 20 == 0x287) {                                    // orc.b
        r = 0;
        for (int i = 0; i < XL / 8; i++) if ((a >> (8 * i)) & 0xff) r |= (X)0xff << (8 * i);
        return true;
      }
      if (in >> 20 == (XL == 32 ? 0x698u : 0x6b8u)) { r = bswapx(a); return true; }  // rev8
      return false;
    }
    return false;
  }
  if (XL == 64 && op == 0x1b) {  // OP-IMM-32
    const uint32_t sh = rs2f, a32 = (uint32_t)a;
    switch (f3) {
    case 0: r = xs((int32_t)(a32 + (uint32_t)mimmI(in))); return true;
    case 1: if (f7 == 0x00) { r = xs((int32_t)(a32 << sh)); return true; } return false;
    case 5:
      if (f7 == 0x00) { r = xs((int32_t)(a32 >> sh)); return true; }
      if (f7 == 0x20) { r = xs((int32_t)a32 >> sh); return true; }
      return false;
    }
    return false;
  }
  if (XL == 64 && op == 0x3b) {  // OP-32
    const uint32_t a32 = (uint32_t)a, b32 = (uint32_t)b;
    switch (f7) {
    case 0x00:
      switch (f3) {
      case 0: r = xs((int32_t)(a32 + b32)); return true;
      case 1: r = xs((int32_t)(a32 << (b32 & 31))); return true;
      case 5: r = xs((int32_t)(a32 >> (b32 & 31))); return true;
      }
      return false;
    case 0x20:
      switch (f3) {
      case 0: r = xs((int32_t)(a32 - b32)); return true;
      case 5: r = xs((int32_t)a32 >> (b32 & 31)); return true;
      }
      return false;
    case 0x01:
      switch (f3) {
      case 0: r = xs((int32_t)(a32 * b32)); return true;
      case 4:
        if (b32 == 0) r = ~(X)0;
        else if ((int32_t)a32 == INT32_MIN && (int32_t)b32 == -1) r = xs((int32_t)a32);
        else r = xs((int32_t)a32 / (int32_t)b32);
        return true;
      case 5: r = b32 == 0 ? ~(X)0 : xs((int32_t)(a32 / b32)); return true;
      case 6:
        if (b32 == 0) r = xs((int32_t)a32);
        else if ((int32_t)a32 == INT32_MIN && (int32_t)b32 == -1) r = 0;
        else r = xs((int32_t)a32 % (int32_t)b32);
        return true;
      case 7: r = b32 == 0 ? xs((int32_t)a32) : xs((int32_t)(a32 % b32)); return true;
      }
      return false;
    }
    return false;
  }
  if (op == 0x33) {  // OP
    const X sm = XL - 1;
    switch (f7) {
    case 0x00:
      switch (f3) {
      case 0: r = a + b; return true;
      case 1: r = a << (b & sm); return true;
      case 2: r = (SX)a < (SX)b; return true;
      case 3: r = a < b; return true;
      case 4: r = a ^ b; return true;
      case 5: r = a >> (b & sm); return true;
      case 6: r = a | b; return true;
      case 7: r = a & b; return true;
      }
      return false;
    case 0x20:
      switch (f3) {
      case 0: r = a - b; return true;
      case 4: r = ~(a ^ b); return true;
      case 5: r = (X)((SX)a >> (b & sm)); return true;
      case 6: r = a | ~b; return true;
      case 7: r = a & ~b; return true;
      }
      return false;
    case 0x01:
      switch (f3) {
      case 0: r = a * b; return true;
      case 1:
        if (XL == 32) r = (X)(((int64_t)(int32_t)a * (int64_t)(int32_t)b) >> 32);
        else r = (X)(((__int128)(int64_t)a * (__int128)(int64_t)b) >> 64);
        return true;
      case 2:
        if (XL == 32) r = (X)(((int64_t)(int32_t)a * (int64_t)b) >> 32);   // (b as a 32-bit unsigned value, as before)
        else r = (X)(((__int128)(int64_t)a * (__int128)(uint64_t)b) >> 64);
        return true;
      case 3:
        if (XL == 32) r = (X)(((uint64_t)a * (uint64_t)b) >> 32);
        else r = (X)(((unsigned __int128)(uint64_t)a * (unsigned __int128)(uint64_t)b) >> 64);
        return true;
      case 4:
        if (b == 0) r = ~(X)0;
        else if ((SX)a == std::numeric_limits<SX>::min() && (SX)b == -1) r = a;
        else r = (X)((SX)a / (SX)b);
        return true;
      case 5: r = b == 0 ? ~(X)0 : a / b; return true;
      case 6:
        if (b == 0) r = a;
        else if ((SX)a == std::numeric_limits<SX>::min() && (SX)b == -1) r = 0;
        else r = (X)((SX)a % (SX)b);
        return true;
      case 7: r = b == 0 ? a : a % b; return true;
      }
      return false;
    case 0x10:
      if (f3 == 2) { r = (a << 1) + b; return true; }
      if (f3 == 4) { r = (a << 2) + b; return true; }
      if (f3 == 6) { r = (a << 3) + b; return true; }
      return false;
    case 0x05:
      switch (f3) {
      case 1: if (XL != 32) return false; r = clmul((uint32_t)a, (uint32_t)b, 0); return true;
      case 2: if (XL != 32) return false; r = clmul((uint32_t)a, (uint32_t)b, 1); return true;
      case 3: if (XL != 32) return false; r = clmul((uint32_t)a, (uint32_t)b, 2); return true;
      case 4: r = (SX)a < (SX)b ? a : b; return true;
      case 5: r = a < b ? a : b; return true;
      case 6: r = (SX)a > (SX)b ? a : b; return true;
      case 7: r = a > b ? a : b; return true;
      }
      return false;
    case 0x30:
      if (f3 == 1) { r = rolx(a, b); return true; }
      if (f3 == 5) { r = rorx(a, b); return true; }
      return false;
    case 0x14: if (f3 == 1) { r = a | ((X)1 << (b & sm)); return true; } return false;
    case 0x24:
      if (f3 == 1) { r = a & ~((X)1 << (b & sm)); return true; }
      if (f3 == 5) { r = (a >> (b & sm)) & 1; return true; }
      return false;
    case 0x34: if (f3 == 1) { r = a ^ ((X)1 << (b & sm)); return true; } return false;
    case 0x04: if (XL == 32 && f3 == 4 && rs2f == 0) { r = a & 0xffff; return true; } return false;   // zext.h (rv32; on rv64 it is op 0x3b)
    }
    return false;
  }
  return false;
}

inline bool eval_branch(uint32_t f3, X a, X b, bool &taken) {
  switch (f3) {
  case 0: taken = a == b; return true;
  case 1: taken = a != b; return true;
  case 4: taken = (SX)a < (SX)b; return true;
  case 5: taken = (SX)a >= (SX)b; return true;
  case 6: taken = a < b; return true;
  case 7: taken = a >= b; return true;
  }
  return false;
}

inline X load_extend(X raw, uint32_t f3) {
  switch (f3 & 7) {
  case 0: return xs(msext((uint32_t)raw & 0xff, 8));
  case 1: return xs(msext((uint32_t)raw & 0xffff, 16));
  case 2: return XL == 64 ? xs((int32_t)(uint32_t)raw) : raw;   // lw: sign-extended on rv64
  case 4: return raw & 0xff;
  case 5: return raw & 0xffff;
  case 6: return XL == 64 ? (raw & 0xffffffffu) : raw;           // lwu
  default: return raw;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Consumers.  Called by the functional unit when it takes an instruction (Fu::compute), after the facts were applied.
// ---------------------------------------------------------------------------------------------------------------
inline bool trace_on() { static const bool on = std::getenv("MSD_TRACE") != nullptr; return on; }
inline void set_taint(uint32_t preg, bool t, X v, uint8_t depth) {
  if (preg == 0 || preg >= (uint32_t)TAG_NUM) return;
  if (trace_on() && trace_win::ok() && t) std::fprintf(stderr, "[MSD] cyc=%lld taint p%u v=%llx depth=%u\n", (long long)sim_time, preg, (unsigned long long)v, depth);
  TV &c = taint[preg];
  if (t && c.t && prev_on()) { c.pt = true; c.pv = c.v; c.pdepth = c.depth; c.pwhen = (long long)sim_time + 1; }   // a stale value replaced by another one: the old one stays visible one more cycle
  else c.pt = false;
  c.t = t;
  c.v = v;
  c.depth = depth;
  c.until = 0;
}

// Effects are written into `inst` (BRU: mispred / br_taken / diag_val; SDU: result = store data; AGU / load: result = the stale address).
template <class Inst>
inline void exec(Inst &inst) {
  const bool d_en = inst.dest_en && inst->dest_preg != 0;
  const auto &u = *inst.operator->();
  const bool s1 = u.src1_en && u.src1_preg != 0 && live(u.src1_preg);
  const bool s2 = u.src2_en && u.src2_preg != 0 && live(u.src2_preg);
  if (!s1 && !s2) {
    if (d_en && inst.op != UOP_LOAD) taint[inst->dest_preg] = TV{};   // a clean producer: its register holds the true value
    return;
  }
  const tb::Fact *f = tbhook::fact_by_rob(inst->rob_idx);
  if (!f) {  // killed instruction: no fact
    if (d_en && inst.op != UOP_LOAD) taint[inst->dest_preg] = TV{};
    return;
  }
  const TV v1 = s1 ? view(u.src1_preg) : TV{}, v2 = s2 ? view(u.src2_preg) : TV{};
  const X a = s1 ? v1.v : f->rs1_value;
  const X b = s2 ? v2.v : f->rs2_value;
  const uint8_t depth = (uint8_t)(std::max(s1 ? (int)v1.depth : 0, s2 ? (int)v2.depth : 0) + 1);
  const uint32_t w = f->inst, opc = w & 0x7f, f3 = mbits(w, 14, 12);
  switch (inst.op) {
  case UOP_ADD:
  case UOP_MUL:
  case UOP_DIV: {
    X r;
    if (!eval_alu(w, f->pc, a, b, r)) { if (d_en) taint[inst->dest_preg] = TV{}; return; }
    if (d_en) {
      if (f->rd_valid && r != f->rd_value) set_taint(inst->dest_preg, true, r, depth);
      else taint[inst->dest_preg] = TV{};
    }
    return;
  }
  case UOP_BR: {
    bool taken;
    if (!eval_branch(f3, a, b, taken) || taken == f->br_taken) return;
    const bool pred_taken = u.ftq_pred_taken;
    const X pc = u.pc, pc_br = pc + xs((int32_t)(uint32_t)u.imm);
    const X pred_target = pred_taken ? (X)u.ftq_next_pc : pc + ilen_of(u.is_rvc);
    inst.mispred = !((taken && pred_taken && pred_target == pc_br) || (!taken && !pred_taken));
    if (trace_on() && trace_win::ok()) std::fprintf(stderr, "[MSD] cyc=%lld BRANCH FLIP pc=%llx stale_taken=%d true_taken=%d mispred=%d\n", (long long)sim_time, (unsigned long long)u.pc, (int)taken, (int)f->br_taken, (int)inst.mispred);
    inst.br_taken = taken;
    inst.diag_val = taken ? pc_br : pc + ilen_of(u.is_rvc);
    return;
  }
  case UOP_JUMP: {
    if (opc != 0x67) return;  // jal has no source
    const X tgt = (a + mimmI(w)) & ~(X)1;
    if (tgt == f->next_pc) return;
    const bool pred_taken = u.ftq_pred_taken;
    const X pred_target = pred_taken ? (X)u.ftq_next_pc : (X)u.pc + ilen_of(u.is_rvc);
    inst.mispred = !(pred_taken && pred_target == tgt);
    if (trace_on() && trace_win::ok()) std::fprintf(stderr, "[MSD] cyc=%lld JALR FLIP pc=%llx tgt=%llx\n", (long long)sim_time, (unsigned long long)u.pc, (unsigned long long)tgt);
    inst.br_taken = true;
    inst.diag_val = tgt;
    return;
  }
  case UOP_LOAD:
  case UOP_STA: {
    // loads: I-type immediate (int 0x03, FP 0x07); stores: S-type (int 0x23, FP 0x27); AMO / LR / SC (0x2f) have none
    const X imm = inst.op == UOP_LOAD ? ((opc == 0x03 || opc == 0x07) ? mimmI(w) : 0) : ((opc == 0x23 || opc == 0x27) ? mimmS(w) : 0);
    const X va = a + imm;
    if (va != f->vaddr) inst.result = va;
    return;
  }
  case UOP_STD: {
    if (opc == 0x23 && b != f->rs2_value) inst.result = b;
    return;
  }
  default:
    if (d_en) taint[inst->dest_preg] = TV{};
    return;
  }
}

}  // namespace msd
