#pragma once
// The staged LSU's shared helpers (store-queue CAM, STLF confirmation, memory-speculation values, write-buffer helpers),
// included by the LSU's phase files (RealLsuPipe.cpp, LsuPipeOutputs.cpp, LsuPipeWriteback.cpp, LsuPipeTranslate.cpp,
// LsuPipeRequests.cpp).
#include <type_traits>
#include "env.h"
#include "BackPorts.h"
#include "RealLsu.h"
#include "DcacheConfig.h"
#include "MemUtils.h"
#include "SimCpu.h"
#include "config.h"
#include "trace_hook.h"
#include "MsData.h"
#include "mmio_address_map.h"
#include "util.h"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

using namespace lp;



#define LP_PORT_CHECK(used, max, what)                                                                  \
  do {                                                                                                  \
    if ((int)(used) > (int)(max)) {                                                                     \
      std::fprintf(stderr, "[LSU_PIPE] port limit exceeded: %s used=%d max=%d\n", what, (int)(used), (int)(max)); \
      std::abort();                                                                                     \
    }                                                                                                   \
  } while (0)

namespace lsu_pipe_detail {


inline uint32_t stq_rank(uint32_t idx, uint32_t head) { return (idx + STQ_SIZE - head) % STQ_SIZE; }
inline uint32_t ld_width(uint32_t func3) { return get_mem_width(func3); }

enum Rel { DISJOINT = 0, COVER = 1, PARTIAL = 2 };

// fixed-width range relation of load [lo, lo+lw) against store [so, so+sw) on `bits` address bits (12: partial / 32: full)
inline Rel range_rel(uint32_t lo, uint32_t lw, uint32_t so, uint32_t sw, int bits) {
  const uint64_t m = bits >= 32 ? 0xFFFFFFFFull : ((1ull << bits) - 1);
  const uint64_t l0 = lo & m, l1 = (lo & m) + lw, s0 = so & m, s1 = (so & m) + sw;
  if (s0 <= l0 && s1 >= l1) return COVER;
  if (l1 <= s0 || l0 >= s1) return DISJOINT;
  return PARTIAL;
}

inline uint32_t replay_wait(ReplayType r) {
  return r == ReplayType::MSHR_HIT || r == ReplayType::MSHR_FULL ? LSU_PEPLAY_WAIT_CYCLES : 0;
}

// Wait table (Alpha 21264 style, EXP-018/019): 2^CFG_LSU_MDP_BITS one-bit entries indexed by a PC hash, set when a load of that PC
// violated, cleared every CFG_LSU_MDP_CLR cycles. A load whose bit is set does not pass unknown-address stores. Training as in the RTL:
// the violators of a cycle are only flagged (one bit per LDQ entry, a register); one flagged live entry per cycle (the lowest LDQ index)
// trains the table in the following cycle, and the bit is written at the start of the cycle after that.
// youngest older store that may conflict (partial CAM on page offset + size; unknown address always conflicts).
struct CamRes {
  bool v = false;
  uint8_t idx = 0;
  bool unk = false;  // memory speculation: an older store younger than the candidate had no address (the load passes it)
  stq_mask_t skip = 0; // LSU_MS_DATA: the unknown-address stores that were passed (bit = STQ index)
};
// `ov`: STQ entries whose STA arrives in this very cycle (AGU output): their vaddr/size bypass into the CAM (a 4x4
// comparator set against the STA ports in hardware), so a load issued together with an older STA is not stalled.
inline CamRes stq_cam(const LsuPipeState &s, const LsuPipeState &n, stq_mask_t ov, const PLdq &l, bool full, bool spec_ok = false) {
  CamRes r;
  int unk_rank = -1;
  const uint32_t head = s.stq_head;
  uint32_t older = 0;
  {  // distance(head tag, snapshot tag) on the 2N tag ring, clipped to the allocated count
    const uint32_t h = (s.stq_head_flag ? STQ_SIZE : 0) + head;
    const uint32_t b = (l.snap_flag ? STQ_SIZE : 0) + l.snap_idx;
    older = (b + 2 * STQ_SIZE - h) % (2 * STQ_SIZE);
    older = std::min<uint32_t>(older, s.stq_count);
  }
  const uint32_t lw = ld_width(l.uop().func3);
  int best = -1;
  for (uint32_t j = 0; j < STQ_SIZE; j++) {  // 64 parallel comparators in hardware
    const uint32_t rank = stq_rank(j, head);
    if (rank >= older) continue;
    const PStq &st = s.stq[j];
    if (st.suppress) continue;
    const PStq &sv = (ov >> j & 1) ? n.stq[j] : st;  // address view with the STA bypass
    bool c;
    if (!sv.va_v) {
      if (spec_ok) {  // pass the unknown-address store (checked when its address arrives)
        unk_rank = std::max(unk_rank, (int)rank);
        r.skip |= (stq_mask_t)1 << j;
        continue;
      }
      c = true;
    } else if (full && st.pa_v) {
      c = range_rel(l.paddr, lw, st.paddr, ld_width(st.func3), 32) != DISJOINT;
    } else {
      c = range_rel(l.vaddr, lw, sv.vaddr, ld_width(sv.func3), 12) != DISJOINT;
    }
    if (c && (int)rank > best) {
      best = rank;
      r.v = true;
      r.idx = j;
    }
  }
  r.unk = unk_rank > best;
  return r;
}

// STLF confirm of load `e` against its blocking store `s` (registered / this-cycle values, full physical address)
enum Conf { CF_NOADDR, CF_REPLAY, CF_HOLD, CF_DATA, CF_FWD };
inline Conf stlf_confirm(const PLdq &e, const PStq &s, bool alive) {
  if (!alive || s.suppress) return CF_REPLAY;
  if (!s.va_v) return CF_NOADDR;
  if (!s.pa_v) {  // address known but not translated yet: the page offset can still prove independence
    if (range_rel(e.vaddr, ld_width(e.uop().func3), s.vaddr, ld_width(s.func3), 12) == DISJOINT) return CF_REPLAY;
    return CF_NOADDR;
  }
  const Rel r = range_rel(e.paddr, ld_width(e.uop().func3), s.paddr, ld_width(s.func3), 32);
  if (r == DISJOINT) return CF_REPLAY;
  if (r == PARTIAL) return CF_HOLD;
  if (!s.data_v) return CF_DATA;
  if (s.lrsc && !s.sc_pass) return CF_HOLD;
  return CF_FWD;
}

// LSU_MS_DATA: the value a load returns when the stores in `l.skip` are not visible (apply_skip) or when everything older is visible.
// Memory image = the word before the oldest STQ store that writes it (the cache content), or the core RAM when no STQ store writes it, with the
// visible older STQ stores applied in age order (the youngest wins, as STLF does).  Returns false when it cannot say (MMIO, no trace info).
inline bool ms_load_value(const LsuPipeState &S, const MsStq *ms, const PLdq &l, const MsLdq &ml, bool apply_skip, xlen_t &val) {
  constexpr uint32_t XB = XLEN / 8;   // the unit of the model: one word (rv32) / one aligned dword (rv64)
  if (!l.pa_v || !is_ram_addr(l.paddr & ~(XB - 1), XB)) return false;
  const uint32_t W = l.paddr & ~(XB - 1), off = l.paddr & (XB - 1), lw = ld_width(l.uop().func3);
  if (off + lw > XB) return false;   // crosses the unit: not modelled
  const uint32_t head = S.stq_head;
  uint32_t older;
  {
    const uint32_t h = (S.stq_head_flag ? STQ_SIZE : 0) + head;
    const uint32_t b = (l.snap_flag ? STQ_SIZE : 0) + l.snap_idx;
    older = std::min<uint32_t>((b + 2 * STQ_SIZE - h) % (2 * STQ_SIZE), S.stq_count);
  }
  xlen_t word = 0;
  bool have = false;
  uint32_t base_rank = 0;
  for (uint32_t rank = 0; rank < S.stq_count && !have; rank++) {
    const uint32_t j = (head + rank) % STQ_SIZE;
    const PStq &t = S.stq[j];
    const MsStq &m = ms[j];
    if (!m.v || m.w != W || t.done || t.suppress) continue;
    word = (xlen_t)m.old_;
    have = true;
    base_rank = rank;
  }
  if (!have) {
    word = tbhook::mem_read(W);
    word |= xlen_t(tbhook::mem_read(W + 4)) << 32;
  }
  for (uint32_t rank = 0; rank < older; rank++) {
    const uint32_t j = (head + rank) % STQ_SIZE;
    const PStq &t = S.stq[j];
    const MsStq &m = ms[j];
    if (!m.v || m.w != W || t.suppress || m.atnt) continue;
    if (t.done && !(have && rank > base_rank)) continue;   // a done store younger than the base store is in the cache but not in its m_old snapshot
    if (apply_skip && (ml.skip >> j & 1)) continue;
    if (m.amo) {
      word = (xlen_t)m.new_;
    } else {
      const xlen_t data = t.data_v ? (xlen_t)t.data : (xlen_t)m.rs2;
      for (uint32_t k = m.off; k < m.off + m.sz && k < XB; k++) word = (word & ~((xlen_t)0xff << (8 * k))) | (((data >> (8 * (k - m.off))) & 0xff) << (8 * k));
    }
  }
  const xlen_t mask = lw >= 8 ? ~(xlen_t)0 : (xlen_t)((1ull << (8 * lw)) - 1);
  const xlen_t raw = lw >= XB ? word : (word >> (8 * off)) & mask;
  val = msd::load_extend(raw, l.uop().func3);
  return true;
}

}  // namespace lsu_pipe_detail
using namespace lsu_pipe_detail;

// ---- EXP-024 write buffer helpers ----
// ---------------------------------------------------------------------------------------------------------------
// cycle start
// ---------------------------------------------------------------------------------------------------------------
// Critical-path cuts (docs/LSU_PIPE.md sections 6 and 7; they were the bit masks LSU_PIPE_CUT = 127 and LSU_PIPE_L2 = 115, the
// variants were deleted 2026-11-15). The letters mark the places in the code.
//   B: wake-up confirm uses the registered STQ window minus ev_ret (no same-cycle count chain)
//   R: STQ retire reads registered state only (ccount / done of the cycle start)
//   D: replay registers pick only from RDY bits that were registered at the cycle start
//   F: write-back of waiting loads (WBP) picks only from WBP bits registered at the cycle start; FM: the load whose MMIO response
//      arrives in this cycle may still be picked in this cycle
//   G: a store woken by an MSHR-full fill can be drained from the next cycle on
//   H: the store drain only sees stores that were already committed at the cycle start
//   a: L3 register: the direct completion decided at L2 is written back one cycle later (LDQ state L_WB3); a2: the consumers are woken
//      from the L3 register in the cycle of the write-back request
//   S: at most one SC completion per cycle
//   M: an MMIO load that becomes MMW at L2 in this cycle is issued to the bus one cycle later
//   (L2) F: a load that gets MSHR_FULL in this very cycle is not woken by this cycle's MSHR fill
constexpr int lp_rp_ports = CFG_LSU_PIPE_RP;  // replay registers
constexpr int lp_drain = std::max(1, std::min<int>(DRAIN_MAX, CFG_LSU_PIPE_DRAIN));
// EXP-024 write buffer: committed stores of one cache line merge in commit order into the tail entry (if it has not been sent yet), the
// oldest entries are sent as ONE DCache request each (their stores complete together with the response). lp_wbuf entries, lp_wbins stores
// inserted per cycle, lp_wbscan entries looked at per cycle, lp_wmax stores per entry.
constexpr int lp_wbuf = std::min<int>(WB_MAX, CFG_LSU_PIPE_WBUF);
constexpr int lp_wbins = CFG_LSU_PIPE_WBINS;
constexpr int lp_wbscan = CFG_LSU_PIPE_WBSCAN;
constexpr int lp_wmax = std::min<int>(WB_MEM, CFG_LSU_PIPE_WBMAX);
constexpr int lp_mgran_sh = 6;  // merge granule 64 B = one cache line

// Insertion after the drain decision of the same cycle (the decision only reads the registered buffer C = state at the cycle start):
// up to lp_wbins uncommitted-to-buffer stores (committed at the cycle start, oldest first; MMIO / suppressed ones are consumed too) are
// merged into the youngest entry when it is still waiting (not sent in this cycle), else they start a new entry (free slot, registered occupancy).
inline void wb_insert(LsuPipeState &S, const LsuPipeState &C, uint32_t n, const bool *issued_now) {
  int budget = lp_wbins;
  int allocated = 0;
  int used = 0;
  for (int k = 0; k < WB_MAX; k++) used += C.wb[k].v;
  for (uint32_t rank = 0; rank < n && budget > 0; rank++) {
    const uint32_t j = (S.stq_head + rank) % STQ_SIZE;
    PStq &s = S.stq[j];
    if (s.wbi != 0 || !s.cmt) continue;
    budget--;
    if (s.mmio || s.done) { s.wbi = 2; continue; }  // MMIO and suppressed stores do not use the buffer
    const bool mergeable = !s.lrsc;
    const uint32_t line = s.paddr >> lp_mgran_sh;
    int tail = -1;  // youngest entry: registered ones and the ones started in this cycle
    for (int e = 0; e < WB_MAX; e++) {
      const bool reg = C.wb[e].v, now = !C.wb[e].v && S.wb[e].v;
      if (!reg && !now) continue;
      const uint64_t sq_e = reg ? C.wb[e].seq : S.wb[e].seq;
      if (tail < 0 || sq_e > (C.wb[tail].v ? C.wb[tail].seq : S.wb[tail].seq)) tail = e;
    }
    int e = -1;
    if (tail >= 0) {
      const bool reg = C.wb[tail].v;
      const bool waiting = reg ? (C.wb[tail].st == 0 && !issued_now[tail]) : true;
      const PWbE &t = S.wb[tail];
      if (mergeable && waiting && t.mergeable && t.line == line && t.cnt < lp_wmax) e = tail;
    }
    if (e < 0) {
      int fr = -1;
      for (int k = 0; k < WB_MAX && fr < 0; k++) if (!C.wb[k].v && !S.wb[k].v) fr = k;
      if (used + allocated >= lp_wbuf || fr < 0) break;  // full: this and the younger stores wait in the STQ
      e = fr;
      PWbE &w = S.wb[e];
      w = PWbE{};
      w.v = true;
      w.seq = ++S.wb_seq;
      w.line = line;
      w.mergeable = mergeable;
      allocated++;
    }
    PWbE &w = S.wb[e];
    w.mem[w.cnt++] = j;
    s.wbi = 1;
    s.wbe = e;
  }
}
// the buffer completes with the response of its first store
inline void wb_response(LsuPipeState &S, uint32_t leader, bool hit) {
  PStq &s = S.stq[leader];
  if (s.wbi != 1) return;
  PWbE &w = S.wb[s.wbe];
  if (!w.v || w.mem[0] != leader) return;
  if (hit) w = PWbE{};
  else w.st = 0;
}
