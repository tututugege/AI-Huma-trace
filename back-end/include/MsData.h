// MsData.h -- value path of the memory-dependence speculative window (switch CFG_LSU_MS_DATA / env LSU_MS_DATA=1, needs LSU_MEMSPEC).
//
// The model is oracle driven: every value is a fact of the trace.  The RTL executes real data, so a load that passed an older store
// whose address was still unknown returns the STALE byte, its consumers compute with it, a dependent branch can resolve the wrong way, a
// dependent load/store can get a wrong address.  All of that dies at the commit-time flush of the violating load, but it changes timing
// before.  This header keeps (a) a taint table over physical registers (stale value + chain depth), (b) an RV32IMAB evaluator for the
// consumers of a tainted register, (c) the statistics.  The stale value of a load is computed in RealLsuPipe.cpp (it needs the STQ).
// Taint exists from the first (deferred) write-back of a stale load to its second write-back, and is dropped at every ROB flush.
#pragma once
#include "config.h"
#include "trace_hook.h"
#include "types.h"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace msd {

inline const bool on = lsu_pipe_env_mask("LSU_MS_DATA", CFG_LSU_MS_DATA) != 0 && lsu_pipe_env_mask("LSU_MEMSPEC", CFG_LSU_MEMSPEC) != 0;
// LSU_MS_DATA bit mask: 1 = branches resolve on stale values (mispredict redirect), 2 = tainted loads/stores use the stale address (not
// modelled yet: counted only), 4 = tainted store data goes into the STQ, 8 = evaluator self check against the facts (debug).
inline const unsigned mode = lsu_pipe_env_mask("LSU_MS_DATA", CFG_LSU_MS_DATA) == 1 ? 1u | 4u : lsu_pipe_env_mask("LSU_MS_DATA", CFG_LSU_MS_DATA);
inline const bool trace = std::getenv("LSU_MS_DTRACE") != nullptr;

struct TV {
  bool t = false;
  uint8_t depth = 0;  // 0 = the stale load itself
  uint32_t v = 0;
};
inline TV taint[TAG_NUM];

struct Stats {
  uint64_t loads_checked = 0, loads_check_bad = 0;                       // self check of the load value logic (non-speculative loads)
  uint64_t spec_wb = 0, stale = 0, silent = 0, stale_no_vio = 0;          // first write-backs of speculative loads
  uint64_t stale_vio = 0, silent_vio = 0;
  uint64_t consumers = 0, masked = 0, unmodelled = 0;                    // consumers of a tainted register
  uint64_t br = 0, br_flip = 0, br_flip_to_mispred = 0, br_flip_to_correct = 0, jalr_flip = 0;
  uint64_t ld_addr_stale = 0, ld_addr_same = 0;                          // loads at a stale address: value differs from / equals the true one
  uint64_t ld_addr = 0, st_addr = 0, st_data = 0;                        // tainted address of a load / store, tainted store data
  uint64_t dep_depth[8] = {};                                            // chain length of results that differ from the true value
  uint64_t br_depth[8] = {};
  uint64_t chk_n = 0, chk_bad = 0;                                       // evaluator self check
  ~Stats() {
    const char *e = std::getenv("LSU_PIPE_STATS");
    if (!on || !e || e[0] == '0') return;
    std::fprintf(stderr,
                 "[MS_DATA] spec_wb=%llu stale=%llu (vio=%llu) silent=%llu (vio=%llu) stale_without_vio=%llu | load check %llu bad %llu\n",
                 (unsigned long long)spec_wb, (unsigned long long)stale, (unsigned long long)stale_vio, (unsigned long long)silent,
                 (unsigned long long)silent_vio, (unsigned long long)stale_no_vio, (unsigned long long)loads_checked, (unsigned long long)loads_check_bad);
    std::fprintf(stderr,
                 "[MS_DATA] consumers=%llu masked=%llu unmodelled=%llu | branches=%llu flip=%llu (to_mispredict=%llu to_correct=%llu) jalr_flip=%llu | "
                 "tainted ld_addr=%llu (value differs %llu, same %llu) st_addr=%llu st_data=%llu | eval self-check %llu bad %llu\n",
                 (unsigned long long)consumers, (unsigned long long)masked, (unsigned long long)unmodelled, (unsigned long long)br,
                 (unsigned long long)br_flip, (unsigned long long)br_flip_to_mispred, (unsigned long long)br_flip_to_correct,
                 (unsigned long long)jalr_flip, (unsigned long long)ld_addr, (unsigned long long)ld_addr_stale, (unsigned long long)ld_addr_same, (unsigned long long)st_addr, (unsigned long long)st_data,
                 (unsigned long long)chk_n, (unsigned long long)chk_bad);
    std::fprintf(stderr, "[MS_DATA] chain length of differing results (1..7+):");
    for (int i = 1; i < 8; i++) std::fprintf(stderr, " %llu", (unsigned long long)dep_depth[i]);
    std::fprintf(stderr, " | branch flips by chain length:");
    for (int i = 1; i < 8; i++) std::fprintf(stderr, " %llu", (unsigned long long)br_depth[i]);
    std::fprintf(stderr, "\n");
  }
};
inline Stats st;

inline void clear_all() {
  for (auto &t : taint) t = TV{};
}

// ---------------------------------------------------------------------------------------------------------------
// RV32IMAB integer evaluator (stateless): result of instruction word `in` at `pc` with source values a (rs1) and b (rs2).
// Returns false for anything it does not model (CSR, FP, AMO, system).
// ---------------------------------------------------------------------------------------------------------------
inline uint32_t mbits(uint32_t v, int hi, int lo) { return (v >> lo) & ((1u << (hi - lo + 1)) - 1); }
inline int32_t msext(uint32_t v, int n) { return (int32_t)(v << (32 - n)) >> (32 - n); }
inline uint32_t mimmI(uint32_t i) { return (uint32_t)msext(mbits(i, 31, 20), 12); }
inline uint32_t mimmS(uint32_t i) { return (uint32_t)msext((mbits(i, 31, 25) << 5) | mbits(i, 11, 7), 12); }
inline uint32_t mimmB(uint32_t i) {
  return (uint32_t)msext((mbits(i, 31, 31) << 12) | (mbits(i, 7, 7) << 11) | (mbits(i, 30, 25) << 5) | (mbits(i, 11, 8) << 1), 13);
}
inline uint32_t rol32(uint32_t x, uint32_t s) { s &= 31; return s ? (x << s) | (x >> (32 - s)) : x; }
inline uint32_t ror32(uint32_t x, uint32_t s) { s &= 31; return s ? (x >> s) | (x << (32 - s)) : x; }
inline uint32_t clmul(uint32_t a, uint32_t b, int mode) {  // 0 clmul, 1 clmulr, 2 clmulh
  uint64_t r = 0;
  for (int i = 0; i < 32; i++)
    if (b >> i & 1) r ^= (uint64_t)a << i;
  return mode == 0 ? (uint32_t)r : mode == 2 ? (uint32_t)(r >> 32) : (uint32_t)(r >> 31);
}

inline bool eval_alu(uint32_t in, uint32_t pc, uint32_t a, uint32_t b, uint32_t &r) {
  const uint32_t op = in & 0x7f, f3 = mbits(in, 14, 12), f7 = mbits(in, 31, 25), rs2f = mbits(in, 24, 20);
  if (op == 0x37) { r = in & 0xfffff000u; return true; }
  if (op == 0x17) { r = pc + (in & 0xfffff000u); return true; }
  if (op == 0x13) {  // OP-IMM
    const uint32_t imm = mimmI(in), sh = rs2f;
    switch (f3) {
    case 0: r = a + imm; return true;
    case 2: r = (int32_t)a < (int32_t)imm; return true;
    case 3: r = a < imm; return true;
    case 4: r = a ^ imm; return true;
    case 6: r = a | imm; return true;
    case 7: r = a & imm; return true;
    case 1:
      if (f7 == 0x00) { r = a << sh; return true; }
      if (f7 == 0x14) { r = a | (1u << sh); return true; }   // bseti
      if (f7 == 0x24) { r = a & ~(1u << sh); return true; }  // bclri
      if (f7 == 0x34) { r = a ^ (1u << sh); return true; }   // binvi
      if (f7 == 0x30) {
        if (sh == 0) { r = a == 0 ? 32 : __builtin_clz(a); return true; }
        if (sh == 1) { r = a == 0 ? 32 : __builtin_ctz(a); return true; }
        if (sh == 2) { r = __builtin_popcount(a); return true; }
        if (sh == 4) { r = (uint32_t)msext(a & 0xff, 8); return true; }
        if (sh == 5) { r = (uint32_t)msext(a & 0xffff, 16); return true; }
      }
      return false;
    case 5:
      if (f7 == 0x00) { r = a >> sh; return true; }
      if (f7 == 0x20) { r = (uint32_t)((int32_t)a >> sh); return true; }
      if (f7 == 0x30) { r = ror32(a, sh); return true; }
      if (f7 == 0x24) { r = (a >> sh) & 1; return true; }  // bexti
      if (in >> 20 == 0x287) {                               // orc.b
        r = 0;
        for (int i = 0; i < 4; i++) if ((a >> (8 * i)) & 0xff) r |= 0xffu << (8 * i);
        return true;
      }
      if (in >> 20 == 0x698) { r = __builtin_bswap32(a); return true; }  // rev8
      return false;
    }
    return false;
  }
  if (op == 0x33) {  // OP
    switch (f7) {
    case 0x00:
      switch (f3) {
      case 0: r = a + b; return true;
      case 1: r = a << (b & 31); return true;
      case 2: r = (int32_t)a < (int32_t)b; return true;
      case 3: r = a < b; return true;
      case 4: r = a ^ b; return true;
      case 5: r = a >> (b & 31); return true;
      case 6: r = a | b; return true;
      case 7: r = a & b; return true;
      }
      return false;
    case 0x20:
      switch (f3) {
      case 0: r = a - b; return true;
      case 4: r = ~(a ^ b); return true;
      case 5: r = (uint32_t)((int32_t)a >> (b & 31)); return true;
      case 6: r = a | ~b; return true;
      case 7: r = a & ~b; return true;
      }
      return false;
    case 0x01:
      switch (f3) {
      case 0: r = a * b; return true;
      case 1: r = (uint32_t)(((int64_t)(int32_t)a * (int64_t)(int32_t)b) >> 32); return true;
      case 2: r = (uint32_t)(((int64_t)(int32_t)a * (int64_t)b) >> 32); return true;
      case 3: r = (uint32_t)(((uint64_t)a * (uint64_t)b) >> 32); return true;
      case 4:
        if (b == 0) r = 0xffffffffu;
        else if ((int32_t)a == INT32_MIN && (int32_t)b == -1) r = a;
        else r = (uint32_t)((int32_t)a / (int32_t)b);
        return true;
      case 5: r = b == 0 ? 0xffffffffu : a / b; return true;
      case 6:
        if (b == 0) r = a;
        else if ((int32_t)a == INT32_MIN && (int32_t)b == -1) r = 0;
        else r = (uint32_t)((int32_t)a % (int32_t)b);
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
      case 1: r = clmul(a, b, 0); return true;
      case 2: r = clmul(a, b, 1); return true;
      case 3: r = clmul(a, b, 2); return true;
      case 4: r = (int32_t)a < (int32_t)b ? a : b; return true;
      case 5: r = a < b ? a : b; return true;
      case 6: r = (int32_t)a > (int32_t)b ? a : b; return true;
      case 7: r = a > b ? a : b; return true;
      }
      return false;
    case 0x30:
      if (f3 == 1) { r = rol32(a, b); return true; }
      if (f3 == 5) { r = ror32(a, b); return true; }
      return false;
    case 0x14: if (f3 == 1) { r = a | (1u << (b & 31)); return true; } return false;
    case 0x24:
      if (f3 == 1) { r = a & ~(1u << (b & 31)); return true; }
      if (f3 == 5) { r = (a >> (b & 31)) & 1; return true; }
      return false;
    case 0x34: if (f3 == 1) { r = a ^ (1u << (b & 31)); return true; } return false;
    case 0x04: if (f3 == 4 && rs2f == 0) { r = a & 0xffff; return true; } return false;
    }
    return false;
  }
  return false;
}

inline bool eval_branch(uint32_t f3, uint32_t a, uint32_t b, bool &taken) {
  switch (f3) {
  case 0: taken = a == b; return true;
  case 1: taken = a != b; return true;
  case 4: taken = (int32_t)a < (int32_t)b; return true;
  case 5: taken = (int32_t)a >= (int32_t)b; return true;
  case 6: taken = a < b; return true;
  case 7: taken = a >= b; return true;
  }
  return false;
}

inline uint32_t load_extend(uint32_t raw, uint32_t f3) {
  switch (f3 & 7) {
  case 0: return (uint32_t)msext(raw & 0xff, 8);
  case 1: return (uint32_t)msext(raw & 0xffff, 16);
  case 4: return raw & 0xff;
  case 5: return raw & 0xffff;
  default: return raw;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Consumers.  Called by the functional unit when it takes an instruction (Fu::compute), after the facts were applied.
// ---------------------------------------------------------------------------------------------------------------
inline void set_taint(uint32_t preg, bool t, uint32_t v, uint8_t depth) {
  if (preg == 0 || preg >= (uint32_t)TAG_NUM) return;
  taint[preg].t = t;
  taint[preg].v = v;
  taint[preg].depth = depth;
}

// `kind`: 0 ALU/MUL/DIV/CSR/FPU (any uop with a destination), 1 AGU, 2 SDU, 3 BRU.  Returns nothing; the effects are written into `inst`
// (BRU: mispred / br_taken / diag_val; SDU: result = store data; AGU: result stays the true address unless mode bit 2 is set).
template <class Inst>
inline void exec(int kind, Inst &inst) {
  const bool d_en = inst.dest_en && inst.dest_preg != 0;
  const auto &u = *inst.operator->();
  const bool s1 = u.src1_en && u.src1_preg != 0 && taint[u.src1_preg].t;
  const bool s2 = u.src2_en && u.src2_preg != 0 && taint[u.src2_preg].t;
  if (!s1 && !s2) {
    if (d_en && inst.op != UOP_LOAD) {
      if (trace && taint[inst.dest_preg].t) std::fprintf(stderr, "MSD t=%lld clean producer rob=%u op=%d clears p%u\n", (long long)sim_time, (unsigned)inst.rob_idx, (int)inst.op, (unsigned)inst.dest_preg);
      taint[inst.dest_preg] = TV{};
    }  // a clean producer: its register holds the true value
    return;
  }
  const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx);
  if (!f) {  // killed instruction: no fact
    if (d_en && inst.op != UOP_LOAD) taint[inst.dest_preg] = TV{};
    return;
  }
  st.consumers++;
  const uint32_t a = s1 ? taint[u.src1_preg].v : f->rs1_value;
  const uint32_t b = s2 ? taint[u.src2_preg].v : f->rs2_value;
  const uint8_t depth = (uint8_t)(std::max(s1 ? (int)taint[u.src1_preg].depth : 0, s2 ? (int)taint[u.src2_preg].depth : 0) + 1);
  const uint8_t dd = depth > 7 ? 7 : depth;
  const uint32_t w = f->inst, opc = w & 0x7f, f3 = mbits(w, 14, 12);
  switch (inst.op) {
  case UOP_ADD:
  case UOP_MUL:
  case UOP_DIV: {
    uint32_t r;
    if (!eval_alu(w, f->pc, a, b, r)) { st.unmodelled++; if (d_en) taint[inst.dest_preg] = TV{}; return; }
    if (d_en) {
      if (f->rd_valid && r != f->rd_value) { set_taint(inst.dest_preg, true, r, depth); st.dep_depth[dd]++; }
      else { taint[inst.dest_preg] = TV{}; st.masked++; }
    }
    return;
  }
  case UOP_BR: {
    bool taken;
    if (!eval_branch(f3, a, b, taken)) { st.unmodelled++; return; }
    st.br++;
    if (taken != f->br_taken) {
      st.br_flip++;
      st.br_depth[dd]++;
      if (mode & 1) {
        const bool pred_taken = u.ftq_pred_taken;
        const uint32_t pc = u.pc, pc_br = pc + u.imm;
        const uint32_t pred_target = pred_taken ? (uint32_t)u.ftq_next_pc : pc + 4;
        const bool mis = !((taken && pred_taken && pred_target == pc_br) || (!taken && !pred_taken));
        if (mis && !inst.mispred) st.br_flip_to_mispred++;
        if (!mis && inst.mispred) st.br_flip_to_correct++;
        inst.mispred = mis;
        inst.br_taken = taken;
        inst.diag_val = taken ? pc_br : pc + 4;
        if (trace) std::fprintf(stderr, "MSD t=%lld branch rob=%u pc=%x flip true=%d stale=%d mispred=%d | s1=%d(p%u v=%x) s2=%d(p%u v=%x) rs1=%x rs2=%x\n", (long long)sim_time, (unsigned)inst.rob_idx, pc, (int)f->br_taken, (int)taken, (int)mis, (int)s1, (unsigned)u.src1_preg, a, (int)s2, (unsigned)u.src2_preg, b, f->rs1_value, f->rs2_value);
      }
    } else {
      st.masked++;
    }
    return;
  }
  case UOP_JUMP: {
    if (opc != 0x67) return;  // jal has no source
    const uint32_t tgt = (a + mimmI(w)) & ~1u;
    st.br++;
    if (tgt != f->next_pc) {
      st.jalr_flip++;
      st.br_depth[dd]++;
      if (mode & 1) {
        const bool pred_taken = u.ftq_pred_taken;
        const uint32_t pred_target = pred_taken ? (uint32_t)u.ftq_next_pc : (uint32_t)u.pc + 4;
        const bool mis = !(pred_taken && pred_target == tgt);
        inst.mispred = mis;
        inst.br_taken = true;
        inst.diag_val = tgt;
      }
    }
    return;
  }
  case UOP_LOAD:
  case UOP_STA: {
    const uint32_t imm = inst.op == UOP_LOAD ? (opc == 0x03 ? mimmI(w) : 0) : (opc == 0x23 ? mimmS(w) : 0);
    const uint32_t va = a + imm;
    if (va != f->vaddr) {
      (inst.op == UOP_LOAD ? st.ld_addr : st.st_addr)++;
      if (mode & 2) inst.result = va;
    } else {
      st.masked++;
    }
    return;
  }
  case UOP_STD: {
    if (opc != 0x23) { st.unmodelled++; return; }
    if (b != f->rs2_value) {
      st.st_data++;
      if (mode & 4) inst.result = b;
    } else {
      st.masked++;
    }
    return;
  }
  default:
    st.unmodelled++;
    if (d_en) taint[inst.dest_preg] = TV{};
    return;
  }
}

// debug: check the evaluator on every untainted ALU/branch uop against the facts (mode bit 8)
template <class Inst>
inline void self_check(Inst &inst) {
  if (!(mode & 8)) return;
  const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx);
  if (!f || f->blocked != tb::Blocked::None || f->trapped) return;
  const auto &u = *inst.operator->();
  const uint32_t w = f->inst;
  if (inst.op == UOP_ADD || inst.op == UOP_MUL || inst.op == UOP_DIV) {
    const uint32_t opc = w & 0x7f;
    if (opc != 0x33 && opc != 0x13 && opc != 0x37 && opc != 0x17) return;
    if (!u.src1_en && !u.src2_en && opc != 0x37 && opc != 0x17) return;  // link ADD of jal/jalr
    uint32_t r;
    if (!f->rd_valid || !eval_alu(w, f->pc, f->rs1_value, f->rs2_value, r)) return;
    st.chk_n++;
    if (r != f->rd_value) {
      if (st.chk_bad++ < 5) std::fprintf(stderr, "[MS_DATA] eval mismatch inst=%08x pc=%x a=%x b=%x got=%x want=%x\n", w, f->pc, f->rs1_value, f->rs2_value, r, f->rd_value);
    }
  } else if (inst.op == UOP_BR) {
    bool t;
    if (!eval_branch(mbits(w, 14, 12), f->rs1_value, f->rs2_value, t)) return;
    st.chk_n++;
    if (t != f->br_taken) st.chk_bad++;
  }
}

}  // namespace msd
