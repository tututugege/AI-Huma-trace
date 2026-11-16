// RealLsuPipe.cpp -- the staged LSU (the only LSU since the legacy one was deleted). Design and rules: docs/LSU_PIPE.md.
//
// Hardware reading of this file: pcur = registers, pnxt = next register values (pnxt == pcur at the start of a cycle,
// seq() copies pnxt to pcur). Each comb phase reads registers (pcur) and writes next values (pnxt) or per-cycle wires (pw).
// The phases run in the BackTop order:
//   comb_cal / lsu2dis / lsu2rob     cycle-start views
//   comb_stq_commit                  ROB commit marks stores committed
//   comb_mmio_in                     MMIO response
//   comb_dcache2lsu_ldq              L2: DCache response, STLF confirm, hit/forward select, park / replay decision
//   comb_dcache2lsu_stq              store drain responses
//   comb_lsu2exe                     write-back ports (direct L2 completion > SC > pending loads), STA completions
//   comb_exe2lsu                     AGU arrivals (STD write, STA, load)
//   comb_tlb_out / MMU / tlb_in_*    L1: DTLB lookup (new loads, replay registers), STA translation
//   comb_stlf                        L1: partial-address CAM over the STQ, MMIO issue
//   comb_dis2lsu, comb_flush         allocation, flush / mispredict / clear_mask
//   comb_lsu2dcache_ldq              L1 -> DCache request, L1/L2 register write
//   comb_lsu2dcache_stq              committed-store drain
//   comb_mmio_out, comb_check        MMIO request, STQ retire, wake-up pass, replay-register fill
#include <type_traits>
#include "Wires.h"
#include "RealLsu.h"
#include "DcacheConfig.h"
#include "MemUtils.h"
#include "SimCpu.h"
#include "config.h"
#include "trace_hook.h"
#include "MsData.h"
#include "PhysMemory.h"
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

namespace {


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

constexpr unsigned lp_memspec = CFG_LSU_MEMSPEC;  // EXP-018, see config.h
// Wait table (Alpha 21264 style, EXP-018/019): 2^CFG_LSU_MDP_BITS one-bit entries indexed by a PC hash, set when a load of that PC
// violated, cleared every CFG_LSU_MDP_CLR cycles. A load whose bit is set does not pass unknown-address stores. Training as in the RTL:
// the violators of a cycle are only flagged (one bit per LDQ entry, a register); one flagged live entry per cycle (the lowest LDQ index)
// trains the table in the following cycle, and the bit is written at the start of the cycle after that.
constexpr unsigned lp_mdp_bits = CFG_LSU_MDP_BITS;
constexpr unsigned lp_mdp_clr = CFG_LSU_MDP_CLR;
static uint64_t g_trn = 0;  // flagged violators (LDQ bit mask)
static std::vector<int> g_mdp_pend;  // table indices trained in the previous cycle, written at the start of this one
static inline uint32_t mdp_idx(uint32_t pc) { return ((pc >> SLOT_SHIFT) ^ (pc >> (SLOT_SHIFT + lp_mdp_bits))) & ((1u << lp_mdp_bits) - 1); }   // (RVC=1: instructions are 2-byte aligned)
static std::vector<bool> g_mdp;
static long long g_mdp_next_clear = 0;
static inline bool mdp_wait(uint32_t pc) {
  if (g_mdp.empty()) g_mdp.assign(1u << lp_mdp_bits, false);
  return g_mdp[mdp_idx(pc)];
}
static inline void mdp_write_pending() {
  if (g_mdp_pend.empty()) return;
  if (g_mdp.empty()) g_mdp.assign(1u << lp_mdp_bits, false);
  for (int ix : g_mdp_pend) g_mdp[ix] = true;
  g_mdp_pend.clear();
}

// youngest older store that may conflict (partial CAM on page offset + size; unknown address always conflicts).
struct CamRes {
  bool v = false;
  uint8_t idx = 0;
  bool unk = false;  // memory speculation: an older store younger than the candidate had no address (the load passes it)
  stq_mask_t skip = 0; // LSU_MS_DATA: the unknown-address stores that were passed (bit = STQ index)
};
// `ov`: STQ entries whose STA arrives in this very cycle (AGU output): their vaddr/size bypass into the CAM (a 4x4
// comparator set against the STA ports in hardware), so a load issued together with an older STA is not stalled.
CamRes stq_cam(const LsuPipeState &s, const LsuPipeState &n, stq_mask_t ov, const PLdq &l, bool full, bool spec_ok = false) {
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
  const uint32_t lw = ld_width(l.func3);
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
Conf stlf_confirm(const PLdq &e, const PStq &s, bool alive) {
  if (!alive || s.suppress) return CF_REPLAY;
  if (!s.va_v) return CF_NOADDR;
  if (!s.pa_v) {  // address known but not translated yet: the page offset can still prove independence
    if (range_rel(e.vaddr, ld_width(e.func3), s.vaddr, ld_width(s.func3), 12) == DISJOINT) return CF_REPLAY;
    return CF_NOADDR;
  }
  const Rel r = range_rel(e.paddr, ld_width(e.func3), s.paddr, ld_width(s.func3), 32);
  if (r == DISJOINT) return CF_REPLAY;
  if (r == PARTIAL) return CF_HOLD;
  if (!s.data_v) return CF_DATA;
  if (s.lrsc && !s.sc_pass) return CF_HOLD;
  return CF_FWD;
}

// LSU_MS_DATA: the value a load returns when the stores in `l.skip` are not visible (apply_skip) or when everything older is visible.
// Memory image = the word before the oldest STQ store that writes it (the cache content), or the core RAM when no STQ store writes it, with the
// visible older STQ stores applied in age order (the youngest wins, as STLF does).  Returns false when it cannot say (MMIO, no trace info).
bool ms_load_value(const LsuPipeState &S, const PLdq &l, bool apply_skip, xlen_t &val) {
  constexpr uint32_t XB = XLEN / 8;   // the unit of the model: one word (rv32) / one aligned dword (rv64)
  if (!l.pa_v || !pmem_is_ram_addr(l.paddr & ~(XB - 1), XB)) return false;
  const uint32_t W = l.paddr & ~(XB - 1), off = l.paddr & (XB - 1), lw = ld_width(l.func3);
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
  for (uint32_t rank = 0; rank < S.stq_count && !have; rank++) {
    const PStq &t = S.stq[(head + rank) % STQ_SIZE];
    if (!t.m_v || t.m_w != W || t.done || t.suppress) continue;
    word = (xlen_t)t.m_old;
    have = true;
  }
  if (!have) {
    word = tbhook::mem_read(W);
    word |= xlen_t(tbhook::mem_read(W + 4)) << 32;
  }
  for (uint32_t rank = 0; rank < older; rank++) {
    const uint32_t j = (head + rank) % STQ_SIZE;
    const PStq &t = S.stq[j];
    if (!t.m_v || t.m_w != W || t.done || t.suppress || t.m_atnt) continue;
    if (apply_skip && (l.skip >> j & 1)) continue;
    if (t.m_amo) {
      word = (xlen_t)t.m_new;
    } else {
      const xlen_t data = t.data_v ? (xlen_t)t.data : (xlen_t)t.m_rs2;
      for (uint32_t k = t.m_off; k < t.m_off + t.m_sz && k < XB; k++) word = (word & ~((xlen_t)0xff << (8 * k))) | (((data >> (8 * (k - t.m_off))) & 0xff) << (8 * k));
    }
  }
  const xlen_t mask = lw >= 8 ? ~(xlen_t)0 : (xlen_t)((1ull << (8 * lw)) - 1);
  const xlen_t raw = lw >= XB ? word : (word >> (8 * off)) & mask;
  val = msd::load_extend(raw, l.func3);
  return true;
}

}  // namespace

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
static void wb_insert(LsuPipeState &S, const LsuPipeState &C, uint32_t n, const bool *issued_now) {
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
static void wb_response(LsuPipeState &S, uint32_t leader, bool hit) {
  PStq &s = S.stq[leader];
  if (s.wbi != 1) return;
  PWbE &w = S.wb[s.wbe];
  if (!w.v || w.mem[0] != leader) return;
  if (hit) w = PWbE{};
  else w.st = 0;
}


// pcur = pnxt. The staged LSU's registers are 12 KB, nearly all of it the 64 LDQ and 64 STQ entries, of which only a few are live.
//  * LDQ: an entry that is L_EMPTY in both states is dead; the only field read from a dead entry is `epoch` (allocation bumps it), so
//    that one is copied and nothing else.
//  * STQ: only the entries inside the ring window [head, head + count) of either state are live (allocation rewrites an entry completely).
// LSU_SEQ_POISON fills the skipped entries of both states with garbage: a read of a stale field then changes the result.
void RealLsu::seq_pipe() {
  static_assert(std::is_trivially_copyable_v<lp::LsuPipeState>);
  using namespace lp;
  const LsuPipeState &n = pnxt;
  LsuPipeState &c = pcur;
  for (int i = 0; i < LDQ_SIZE; i++) {
    if (c.ldq[i].st != L_EMPTY || n.ldq[i].st != L_EMPTY) {
      c.ldq[i] = n.ldq[i];
    } else {
#ifdef LSU_SEQ_POISON
      const auto ep = n.ldq[i].epoch;
      std::memset(&c.ldq[i], 0xA5, sizeof(PLdq));
      std::memset(&pnxt.ldq[i], 0xA5, sizeof(PLdq));
      c.ldq[i].st = pnxt.ldq[i].st = L_EMPTY;
      c.ldq[i].epoch = pnxt.ldq[i].epoch = ep;
#else
      c.ldq[i].epoch = n.ldq[i].epoch;
#endif
    }
  }
#ifdef LSU_SEQ_POISON
  bool live[STQ_SIZE] = {};
  for (uint32_t k = 0; k < c.stq_count; k++) live[(c.stq_head + k) % STQ_SIZE] = true;
  for (uint32_t k = 0; k < n.stq_count; k++) live[(n.stq_head + k) % STQ_SIZE] = true;
  for (int i = 0; i < STQ_SIZE; i++) {
    if (live[i]) c.stq[i] = n.stq[i];
    else { std::memset(&c.stq[i], 0xA5, sizeof(PStq)); std::memset(&pnxt.stq[i], 0xA5, sizeof(PStq)); }
  }
#else
  for (uint32_t k = 0; k < c.stq_count; k++) { const uint32_t i = (c.stq_head + k) % STQ_SIZE; c.stq[i] = n.stq[i]; }
  for (uint32_t k = 0; k < n.stq_count; k++) { const uint32_t i = (n.stq_head + k) % STQ_SIZE; c.stq[i] = n.stq[i]; }
#endif
  // everything after the two big arrays (write-back buffer, L1/L2/L3 registers, replay registers, pointers)
  char *dp = reinterpret_cast<char *>(&c);
  const char *sp = reinterpret_cast<const char *>(&n);
  const size_t off = reinterpret_cast<const char *>(&c.wb) - dp;
  std::memcpy(dp + off, sp + off, sizeof(LsuPipeState) - off);
}

void RealLsu::comb_cal() {
  pw = {};
  mdp_write_pending();
  {  // train one flagged live entry (flags of earlier cycles); entries that died are dropped
    for (int i = 0; i < LDQ_SIZE && g_trn; i++)  // the flag of an entry that died is dropped (the RTL clears it when the entry is EMPTY)
      if ((g_trn >> i & 1) && pcur.ldq[i].st == L_EMPTY) g_trn &= ~(1ULL << i);
    for (int i = 0; i < LDQ_SIZE && g_trn; i++)
      if (g_trn >> i & 1) { g_trn &= ~(1ULL << i); g_mdp_pend.push_back(mdp_idx(pcur.ldq[i].pc)); break; }
  }
  for (int k = 0; k < LD; k++) pnxt.l12[k] = {};  // L1/L2 registers are rewritten every cycle
  for (int k = 0; k < LD; k++) pnxt.l3[k] = {};
  for (auto &w : wires->lsu_awake.wake) w = {};
  {  // trace info of the stores allocated in an earlier cycle (the fact leaves the bridge when the store commits)
    for (uint32_t k = 0; k < pnxt.stq_count; k++) {
      PStq &t = pnxt.stq[(pnxt.stq_head + k) % STQ_SIZE];
      if (t.m_init) continue;
      t.m_init = true;
      const tb::Fact *f = tbhook::fact_by_rob(t.rob_idx);
      if (!f || !f->is_store || !f->has_mem_write || f->trapped || f->blocked != tb::Blocked::None) continue;
      t.m_v = true;
      t.m_amo = (f->inst & 0x7f) == 0x2f;
      t.m_w = f->mem_waddr;
      t.m_old = f->mem_old;
      t.m_new = f->mem_wdata;
      t.m_rs2 = f->rs2_value;
      t.m_off = f->vaddr & (XLEN / 8 - 1);
      t.m_sz = f->size;
    }
  }
}

void RealLsu::comb_lsu2rob() {
  wires->lsu2rob = {};
  wires->lsu2rob.committed_store_pending = pcur.stq_ccount != 0;
  std::bitset<ROB_NUM> m;  // ROB_NUM bits: a 64-bit mask lost the entries above 63 
  for (const PLdq &l : pcur.ldq)
    if (l.miss && l.st != L_EMPTY) m.set(l.rob_idx);
  wires->lsu2rob.tma.miss_mask = m;
}

void RealLsu::comb_lsu2dis() {
  wires->lsu2dis = {};
  const uint32_t head = pcur.stq_head, count = pcur.stq_count;
  wires->lsu2dis.stq_tail = (head + count) % STQ_SIZE;
  wires->lsu2dis.stq_tail_flag = stq_tail_flag(head, count, pcur.stq_head_flag);
  uint32_t empty = 0;
  for (const PLdq &l : pcur.ldq) empty += l.st == L_EMPTY;
  wires->lsu2dis.ldq_free = empty;
  wires->lsu2dis.stq_free = STQ_SIZE - count;
  uint32_t n = 0;
  for (int i = 0; i < LDQ_SIZE && n < (uint32_t)MAX_LDQ_DISPATCH_WIDTH; i++)
    if (pcur.ldq[i].st == L_EMPTY) {
      wires->lsu2dis.ldq_alloc_idx[n] = i;
      wires->lsu2dis.ldq_alloc_valid[n++] = true;
    }
}

// ---------------------------------------------------------------------------------------------------------------
// ROB commit: STQ entries become committed in order
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::comb_stq_commit() {
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    if (!wires->rob_commit.commit_entry[i].valid) continue;
    const auto &u = wires->rob_commit.commit_entry[i].uop;
    if (!is_store(u)) continue;
    const int idx = u.stq_idx;
    if (idx != pnxt.stq_commit || !pnxt.stq[idx].wbd) continue;
    PStq &s = pnxt.stq[idx];
    s.cmt = 1;
    if (static_cast<bool>(wires->rob_bcast.flush) || (s.lrsc && !s.sc_pass)) {
      s.suppress = 1;  // flushed or failed SC: retire without writing
      s.done = 1;
    }
    pnxt.stq_commit = (pnxt.stq_commit + 1) % STQ_SIZE;
    if (pnxt.stq_ccount < STQ_SIZE) pnxt.stq_ccount++;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// MMIO
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::comb_mmio_in() {
  if (!(wires->in.peripheral_resp.is_mmio && cur.uncached_unit.valid)) return;
  const uint32_t idx = cur.uncached_unit.idx;
  if (cur.uncached_unit.is_load) {
    PLdq &e = pnxt.ldq[idx];
    if (e.st == L_MMR) {
      e.result = wires->in.peripheral_resp.mmio_rdata;
      e.st = L_WBP;
      pw.mmio_ld = idx;
    }
  } else if (pnxt.stq[idx].mmw) {
    pnxt.stq[idx].mmw = 0;
    pnxt.stq[idx].done = 1;
  }
  nxt.uncached_unit.valid = false;
}

void RealLsu::comb_mmio_out() {
  wires->out.peripheral_req = {};
  if (!wires->in.peripheral_resp.ready) return;
  const UncachedUnit &u = nxt.uncached_unit;
  wires->out.peripheral_req.is_mmio = u.valid;
  if (u.valid) {
    wires->out.peripheral_req.wen = !u.is_load;
    wires->out.peripheral_req.mmio_addr = u.addr;
    wires->out.peripheral_req.mmio_wdata = u.wdata;
    wires->out.peripheral_req.mmio_fun3 = u.func3;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// L2: DCache response + STLF confirm + hit/forward select. Reads the L1/L2 registers and the registered STQ state.
// A load that cannot complete is parked in the LDQ entry (cause + wake condition), nothing is queued anywhere.
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::comb_dcache2lsu_ldq() {
  const DcacheLsuIO &dc = wires->in.dcache2lsu;
  pw.fill = dc.mshr_fill && LSU_WAIT_MSHR_FILL;
  pw.fill_addr = dc.mshr_fill_addr;

  auto park = [&](PLdq &e, uint8_t cause, uint32_t blk, uint32_t wait) {
    e.st = L_RPW;
    e.rcause = cause;
    e.blk_idx = blk;
    e.rwait = wait;
  };

  for (int i = 0; i < LD; i++) {
    const PL12 &sl = pcur.l12[i];
    if (!sl.v) continue;
    PLdq &e = pnxt.ldq[sl.ldq];
    if (!(e.st == L_PIPE && e.epoch == sl.epoch)) continue;  // killed after L1
    if (sl.tlb == 2) {  // page fault: reported through the load write-back
      pw.wbd_v[i] = true;
      pw.wbd_ldq[i] = sl.ldq;
      continue;
    }
    if (sl.tlb == 1) {
      park(e, RC_TLB, 0, 0);
      continue;
    }
    if (sl.mmio) {
      e.st = L_MMW;
      continue;
    }
    if (!sl.cand_v && sl.dcblk) {  // not launched (hidden retry owns the port): replay like a bank conflict
      park(e, RC_DC, 0, replay_wait(ReplayType::CONFLICT));
      continue;
    }
    if (!sl.cand_v) {  // no conflicting older store: the DCache read of L1 decides
      e.st = L_DCW;
      continue;
    }
    // STLF confirm against the registered STQ entry (full physical address check)
    const uint32_t j = sl.cand;
    const PStq &s = pcur.stq[j];
    const uint32_t rank = stq_rank(j, pcur.stq_head);
    switch (stlf_confirm(e, s, rank < pcur.stq_count)) {
    case CF_REPLAY: e.st = L_RDY; break;  // false partial match / candidate gone: replay with the full compare
    case CF_NOADDR: park(e, RC_ADDR, j, 0); break;
    case CF_HOLD: park(e, RC_HOLD, j, 0); break;
    case CF_DATA: park(e, RC_DATA, j, 0); break;
    case CF_FWD:  // hit/forward select picks the STQ data
      pw.wbd_v[i] = true;
      pw.wbd_ldq[i] = sl.ldq;
      break;
    }
  }

  for (int i = 0; i < LD; i++) {  // DCache responses (same port as the request, also late bank-conflict retries)
    const auto &r = dc.resp_ports.load_resps[i];
    if (!r.valid || (r.req_id & (1u << 31))) continue;
    const uint32_t idx = r.req_id & kLsuReqIdIdxMask;
    if (idx >= LDQ_SIZE) continue;
    const uint32_t ep = (r.req_id >> LDQ_IDX_WIDTH) & ((1u << EPOCH_W) - 1);
    PLdq &e = pnxt.ldq[idx];
    if (!(e.st == L_DCW && e.epoch == ep)) continue;   // request of a load that forwarded / replayed / was killed
    e.rtype = (uint8_t)r.replay;
    if (r.replay == ReplayType::HIT) {
      if (pw.wbd_v[i]) {
        e.st = L_WBP;
      } else {
        pw.wbd_v[i] = true;
        pw.wbd_ldq[i] = idx;
      }
    } else {
      e.miss = 1;
      park(e, RC_DC, 0, replay_wait(r.replay));
    }
  }

  // a: the direct completions go through the L3 register (written back in the next cycle)
  for (int i = 0; i < LD; i++) {
    if (!pw.wbd_v[i]) continue;
    pnxt.l3[i].v = 1;
    pnxt.l3[i].ldq = pw.wbd_ldq[i];
    pnxt.ldq[pw.wbd_ldq[i]].st = L_WB3;
    pw.wbd_v[i] = false;
  }
}

void RealLsu::comb_dcache2lsu_stq() {
  const DcacheLsuIO &dc = wires->in.dcache2lsu;
  for (int i = 0; i < LSU_STA_COUNT; i++) {
    const auto &r = dc.resp_ports.store_resps[i];
    if (!r.valid || r.req_id >= STQ_SIZE) continue;
    PStq &s = pnxt.stq[r.req_id];
    if (!s.iss) continue;
    s.iss = 0;
    s.rtype = (uint8_t)r.replay;
    if (r.replay == ReplayType::HIT) {
      s.done = 1;
      s.rwait = 0;
    } else {
      s.rwait = replay_wait(r.replay);
    }
    wb_response(pnxt, r.req_id, r.replay == ReplayType::HIT);
  }
  {  // stores merged into a write-buffer entry complete with their leader's response
    for (int i = 0; i < LSU_STA_COUNT; i++) {
      const auto &r = dc.resp_ports.store_resps[i];
      if (!r.valid || r.req_id >= STQ_SIZE) continue;
      for (uint32_t k = 0; k < pnxt.stq_count; k++) {
        PStq &f = pnxt.stq[(pnxt.stq_head + k) % STQ_SIZE];
        if (!f.iss || !f.mf || f.ml != r.req_id) continue;
        f.iss = 0;
        f.mf = 0;
        f.rtype = (uint8_t)r.replay;
        if (r.replay == ReplayType::HIT) {
          f.done = 1;
          f.rwait = 0;
        } else {
          f.rwait = replay_wait(r.replay);
        }
      }
    }
  }
  if (pw.fill || (dc.mshr_fill && LSU_WAIT_MSHR_FILL)) {
    uint32_t full = 0;
    const uint32_t n = std::min<uint32_t>(pnxt.stq_ccount, pnxt.stq_count);
    for (uint32_t k = 0; k < n; k++) {  // MSHR fill wake of waiting stores
      PStq &s = pnxt.stq[(pnxt.stq_head + k) % STQ_SIZE];
      if (!s.cmt || s.done || s.iss || s.rwait == 0) continue;
      if (s.rtype == (uint8_t)ReplayType::MSHR_HIT && cache_line_match(s.paddr, dc.mshr_fill_addr)) {
        s.rwait = 0;
      } else if (s.rtype == (uint8_t)ReplayType::MSHR_FULL && full < (uint32_t)LSU_STA_COUNT) {
        pw.st_fullwake |= (stq_mask_t)1 << ((pnxt.stq_head + k) % STQ_SIZE);
        pw.st_rwpre[(pnxt.stq_head + k) % STQ_SIZE] = s.rwait;
        s.rwait = 0;
        full++;
      }
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Write-back ports
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::lp_emit_load_wb(int port, int idx, bool from_l3) {
  PLdq &l = pnxt.ldq[idx];
  MicroOp wb;
  wb.op = UOP_LOAD;
  wb.rob_idx = l.rob_idx;
  wb.rob_flag = l.rob_flag;
  wb.br_mask = l.br_mask;
  wb.diag_val = l.vaddr;
  wb.result = l.pf ? l.vaddr : l.result;
  wb.dest_preg = l.pf ? 0 : l.dest_preg;   // a faulting load does not write its destination (RTL: lp_wb)
  wb.page_fault_load = l.pf;
  wb.illegal_inst = l.illegal;
  wb.dest_en = !l.pf && l.dest_preg != 0;  // loads to x0 still complete without a destination
  wb.is_mmio = l.mmio;
  const bool second = l.second;
  const bool defer = lp_memspec && !second && l.spec && !l.pf && !l.mmio && !l.lrsc;
  bool early = second;
  if (defer) wb.rob_defer = true;
  if (second) wb.flush_pipe = l.vio;
  if (!early && from_l3 && !l.pf && l.dest_preg != 0 && !wires->rob_bcast.flush &&
      !(wires->dec_bcast.mispred && (l.br_mask & wires->dec_bcast.br_mask) != 0)) {  // a2: wake from the L3 register
    wires->lsu_awake.wake[LD + port].valid = true;
    wires->lsu_awake.wake[LD + port].preg = l.dest_preg;
    early = true;
  }
  wb.early_woken = early;
  wires->lsu2exe.wb_req[port].valid = true;
  wires->lsu2exe.wb_req[port].uop = wb;
  if (l.lrsc && !l.pf) {
    LrScUnit &r = nxt.lrsc_unit;
    r.reserve_valid = true;
    r.reserve_addr = l.paddr;
    r.reserve_rob_idx = l.rob_idx;
    r.reserve_rob_flag = l.rob_flag;
    r.reserve_br_mask = l.br_mask;
  }
  // every load write-back leaves its register without a stale value unless the paths below set one (MMIO / LR-SC loads and the faulting ones do not go there)
  if ((l.pf || l.mmio || l.lrsc) && l.dest_preg != 0) msd::taint[l.dest_preg] = msd::TV{};
  if (!l.pf && !l.mmio && !l.lrsc) {
    const tb::Fact *f = tbhook::fact_by_rob(l.rob_idx);
    const bool plain = f && f->is_load && f->rd_valid && (f->inst & 0x7f) == 0x03;
    xlen_t v = 0;
    if (l.atnt) {  // stale base register: the load reads the stale address, its value stays stale (the load is never re-executed with a good address)
      msd::taint[l.dest_preg] = msd::TV{};
      if (plain && ms_load_value(pnxt, l, defer, v) && v != f->rd_value) msd::set_taint(l.dest_preg, true, v, 1);
    } else if (defer) {  // first write-back of a speculative load: the value of the load without the stores it passed
      msd::taint[l.dest_preg] = msd::TV{};
      if (plain && ms_load_value(pnxt, l, true, v)) {
        l.stale = v != f->rd_value;
        if (l.stale) msd::set_taint(l.dest_preg, true, v, 0);
      }
    } else {
      if (second) l.stale = false;
      msd::release(l.dest_preg);  // the register holds the true value from here on (for consumers issuing from the next cycle) ...
      if (plain && ms_load_value(pnxt, l, false, v)) {
        // ... unless it was forwarded from a store whose DATA is stale (mode 4: the STQ holds the stale data): the load returns that
        if (v != f->rd_value) {
          msd::set_taint(l.dest_preg, true, v, 1);
        }
      }
    }
  }
  if (defer) {  // keep the entry until the older store addresses are known (then the completion write-back, or a re-execution)
    if (l.vio) {
      l.second = 1;
      l.st = L_RDY;
    } else {
      l.st = L_SPD;
    }
  } else {
    l.st = L_EMPTY;
  }
}

void RealLsu::comb_lsu2exe() {
  wires->lsu2exe = {};
  bool busy[LD] = {};
  for (int p = 0; p < LD; p++) {  // 1. the completions registered in L3 own their load port
    if (pcur.l3[p].v && pnxt.ldq[pcur.l3[p].ldq].st == L_WB3) {
      busy[p] = true;
      lp_emit_load_wb(p, pcur.l3[p].ldq, true);
    }
  }

  // 2. STA completions (age order from the head, first STA_COUNT; SC through a free load port)
  int sta_n = 0, sc_n = 0;
  for (uint32_t rank = 0; rank < pnxt.stq_count; rank++) {
    const uint32_t j = (pnxt.stq_head + rank) % STQ_SIZE;
    PStq &s = pnxt.stq[j];
    if (!s.wbp) continue;
    MicroOp wb;
    wb.rob_idx = s.rob_idx;
    wb.rob_flag = s.rob_flag;
    wb.br_mask = s.br_mask;
    wb.diag_val = s.vaddr;
    wb.page_fault_store = s.pf;
    wb.dest_en = false;
    wb.is_mmio = s.mmio;
    if (s.lrsc) {  // SC reports pass/fail through a load write-back port
      if (sc_n >= 1) continue;  // S: one SC completion per cycle
      int p = -1;
      for (int q = 0; q < LD && p < 0; q++)
        if (!busy[q]) p = q;
      if (p < 0) continue;
      wb.op = UOP_LOAD;
      if (s.pf) {
        wb.result = s.vaddr;
      } else {
        const bool pass = cur.lrsc_unit.reserve_valid && cur.lrsc_unit.reserve_addr == s.paddr;
        s.sc_pass = pass;
        s.suppress = !pass;
        nxt.lrsc_unit.reserve_valid = false;
        wb.result = pass ? 0 : 1;
        wb.dest_preg = s.dest_preg;
        wb.dest_en = s.dest_preg != 0;
        pw.ev_sc |= (stq_mask_t)1 << j;
      }
      busy[p] = true;
      sc_n++;
      wires->lsu2exe.wb_req[p].valid = true;
      wires->lsu2exe.wb_req[p].uop = wb;
    } else {
      if (sta_n >= LSU_STA_COUNT) continue;
      wb.op = UOP_STA;
      wb.result = s.vaddr;
      wires->lsu2exe.sta_wb_req[sta_n].valid = true;
      wires->lsu2exe.sta_wb_req[sta_n++].uop = wb;
    }
    s.wbp = 0;
    s.wbd = 1;
  }
  LP_PORT_CHECK(sta_n, LSU_STA_COUNT, "sta_wb");

  // 2b. memory speculation: a written-back load whose older store addresses are all known now sends its completion
  if (lp_memspec) {
    for (int i = 0; i < LDQ_SIZE; i++) {
      PLdq &l = pnxt.ldq[i];
      if (l.st != L_SPD) continue;
      const uint32_t h = (pnxt.stq_head_flag ? STQ_SIZE : 0) + pnxt.stq_head;
      const uint32_t b = (l.snap_flag ? STQ_SIZE : 0) + l.snap_idx;
      const uint32_t older = std::min<uint32_t>((b + 2 * STQ_SIZE - h) % (2 * STQ_SIZE), pnxt.stq_count);
      bool unknown = false;
      for (uint32_t k = 0; k < older && !unknown; k++) {
        const PStq &s = pnxt.stq[(pnxt.stq_head + k) % STQ_SIZE];
        unknown = !s.va_v && !s.suppress;
      }
      if (!unknown) {
        l.second = 1;
        l.st = L_WBP;
      }
    }
  }

  // 3. loads waiting for a port (forwarded after a data wait, MMIO data): lowest LDQ index first
  for (int i = 0; i < LDQ_SIZE; i++) {
    if (pnxt.ldq[i].st != L_WBP) continue;
    if (pcur.ldq[i].st != L_WBP && pw.mmio_ld != i) continue;  // F / FM: registered WBP bits (and this cycle's MMIO answer)
    int p = -1;
    for (int q = 0; q < LD && p < 0; q++)
      if (!busy[q]) p = q;
    if (p < 0) break;
    busy[p] = true;
    lp_emit_load_wb(p, i);
  }
}

// ---------------------------------------------------------------------------------------------------------------
// AGU arrivals: STD write, STA (va), load (L1 input)
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::comb_exe2lsu() {
  const ExeLsuIO &x = wires->exe2lsu;
  for (int i = 0; i < LSU_SDU_COUNT; i++) {
    if (!x.sdu_req[i].valid) continue;
    const MicroOp &inst = x.sdu_req[i].uop;
    if (inst.stq_idx >= STQ_SIZE) continue;
    PStq &e = pnxt.stq[inst.stq_idx];
    if (e.rob_flag != inst.rob_flag || e.rob_idx != inst.rob_idx) continue;
    e.data = inst.result;
    e.data_v = 1;
    pw.ev_data |= (stq_mask_t)1 << inst.stq_idx;
    if (e.pa_v && !e.wbp && !e.wbd) e.wbp = 1;
  }
  int ns = 0;
  int vio_new = 0;  // loads marked as violating in this cycle
  uint64_t vio_set = 0;
  for (int i = 0; i < LSU_AGU_COUNT; i++) {
    if (!(x.agu_req[i].valid && x.agu_req[i].uop.op == UOP_STA)) continue;
    const MicroOp &inst = x.agu_req[i].uop;
    if (inst.stq_idx >= STQ_SIZE) continue;
    PStq &e = pnxt.stq[inst.stq_idx];
    if (e.rob_flag != inst.rob_flag || e.rob_idx != inst.rob_idx) continue;
    e.va_v = 1;
    e.vaddr = inst.result;
    {  // a store whose base register was stale: its data goes to the stale address (never to the true one)
      const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx);
      e.m_atnt = f && f->is_store && f->vaddr != static_cast<xlen_t>(inst.result);
    }
    e.pa_v = 0;
    e.paddr = 0;
    e.func3 = inst->func3;
    e.dest_preg = inst.dest_preg;
    e.lrsc = inst->is_atomic && ((inst->func7 >> 2) == AmoOp::SC);
    e.trp = 0;
    LP_PORT_CHECK(ns + 1, LSU_STA_COUNT, "sta_agu");
    pw.s1[ns].v = true;
    pw.s1[ns].stq = inst.stq_idx;
    ns++;
    pw.ev_sta |= (stq_mask_t)1 << inst.stq_idx;  // the partial address is known from now on
    if (lp_memspec) {  // memory-order check: younger loads that passed this store while its address was unknown
      const uint32_t j = inst.stq_idx;
      const uint32_t h = (pcur.stq_head_flag ? STQ_SIZE : 0) + pcur.stq_head;
      const uint32_t sw = ld_width(inst->func3);
      for (int i = 0; i < LDQ_SIZE; i++) {
        PLdq &l = pnxt.ldq[i];
        if (!l.spec || l.vio || l.second) continue;
        // L_RPW too: a load parked behind an older store (ADDR / HOLD / DATA) forwards from it when woken, without another CAM pass, and would
        // miss a younger store whose address it passed (found by the RTL Linux check, s06: stale byte forwarded)
        if (l.st != L_PIPE && l.st != L_DCW && l.st != L_WBP && l.st != L_WB3 && l.st != L_SPD && l.st != L_RPW) continue;
        const uint32_t b = (l.snap_flag ? STQ_SIZE : 0) + l.snap_idx;
        const uint32_t older = std::min<uint32_t>((b + 2 * STQ_SIZE - h) % (2 * STQ_SIZE), pcur.stq_count);
        if (stq_rank(j, pcur.stq_head) >= older) continue;  // the store is younger than the load
        if (range_rel(l.vaddr, ld_width(l.func3), inst.result, sw, 12) == DISJOINT) continue;
        vio_new++;
        vio_set |= 1ULL << i;
        l.vio = 1;
        if (l.st == L_SPD) {  // already written back: re-execute with the store visible
          l.second = 1;
          l.st = L_RDY;
        }
      }
    }
  }
  if (vio_new) {
    g_trn |= vio_set;
  }
  int nl = 0;
  for (int i = 0; i < LSU_AGU_COUNT; i++) {
    if (!(x.agu_req[i].valid && x.agu_req[i].uop.op == UOP_LOAD)) continue;
    const MicroOp &inst = x.agu_req[i].uop;
    if (inst.ldq_idx >= LDQ_SIZE) continue;
    PLdq &e = pnxt.ldq[inst.ldq_idx];
    if (e.rob_flag != inst.rob_flag || e.rob_idx != inst.rob_idx) continue;
    e.vaddr = inst.result;
    {  // a load whose base register was stale reads the stale address
      const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx);
      e.atnt = f && f->is_load && f->vaddr != static_cast<xlen_t>(inst.result);
    }
    e.dest_preg = inst.dest_preg;
    e.pa_v = 0;
    e.paddr = 0;
    e.func3 = inst->func3;
    e.illegal = inst.illegal_inst;
    e.st = L_PIPE;
    e.snap_idx = inst.stq_idx;
    e.snap_flag = inst.stq_flag;
    e.lrsc = is_amo_lr_uop(inst);
    LP_PORT_CHECK(nl + 1, LD, "load_agu");
    pw.l1[nl].v = true;
    pw.l1[nl].need_tlb = true;
    pw.l1[nl].ldq = inst.ldq_idx;
    nl++;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// L1: DTLB request (new loads, replay registers fill the remaining load ports), STA translation
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::comb_tlb_out() {
  wires->lsu2mmu_io = {};
  wires->lsu2mmu_io.csr_status = wires->csr_status;

  int n = 0;
  while (n < LD && pw.l1[n].v) n++;
  for (int r = 0; r < lp_rp_ports; r++) {
    if (n >= LD) continue;
    int src = -1;
    if (pcur.rp[r].v) {
      src = pcur.rp[r].ldq;
      pnxt.rp[r].v = 0;
    }
    if (src < 0) continue;
    PLdq &e = pnxt.ldq[src];
    if (e.st != L_PIPE) continue;
    pw.l1[n].v = true;
    pw.l1[n].replay = true;
    pw.l1[n].need_tlb = !e.pa_v;
    pw.l1[n].ldq = src;
    n++;
  }
  for (int k = 0; k < LD; k++) {
    if (!pw.l1[k].v || !pw.l1[k].need_tlb) continue;
    auto &rq = wires->lsu2mmu_io.ldq_req[k];
    rq.valid = true;
    rq.vaddr = pnxt.ldq[pw.l1[k].ldq].vaddr;
    rq.entry_idx = pw.l1[k].ldq;
    rq.wait_idx = pw.l1[k].ldq;
  }

  int ns = 0;
  while (ns < STA && pw.s1[ns].v) ns++;
  if (ns < STA && pcur.srp.v) {  // STA replay register takes a free STA port
    const uint32_t j = pcur.srp.ldq;
    pnxt.srp.v = 0;
    if (stq_rank(j, pnxt.stq_head) < pnxt.stq_count && pnxt.stq[j].va_v && !pnxt.stq[j].pa_v && !pnxt.stq[j].pf) {
      pw.s1[ns].v = true;
      pw.s1[ns].replay = true;
      pw.s1[ns].stq = j;
      ns++;
    }
  }
  for (int k = 0; k < STA; k++) {
    if (!pw.s1[k].v) continue;
    auto &rq = wires->lsu2mmu_io.stq_req[k];
    rq.valid = true;
    rq.vaddr = pnxt.stq[pw.s1[k].stq].vaddr;
    rq.entry_idx = pw.s1[k].stq;
    rq.wait_idx = pw.s1[k].stq;
  }
}

void RealLsu::comb_tlb_in_ldq() {
  for (int k = 0; k < LD; k++) {
    PL1 &l = pw.l1[k];
    if (!l.v) continue;
    PLdq &e = pnxt.ldq[l.ldq];
    if (!l.need_tlb) {  // replay with a known physical address
      l.hit = true;
      l.mmio = e.mmio;
      l.paddr = e.paddr;
      continue;
    }
    const auto &resp = wires->mmu2lsu_io.ldq_resp[k];
    if (!resp.valid || resp.entry_idx != l.ldq) {
      l.tlb = 1;
    } else if (resp.result == MMUResultType::HIT) {
      e.pa_v = 1;
      e.paddr = resp.paddr;
      e.mmio = lsu_is_mmio_addr(resp.paddr);
      l.hit = true;
      l.mmio = e.mmio;
      l.paddr = resp.paddr;
    } else if (resp.result == MMUResultType::PAGE_FAULT) {
      e.pf = 1;
      e.result = e.vaddr;
      nxt.lrsc_unit.reserve_valid = false;
      l.tlb = 2;
    } else {
      l.tlb = 1;
    }
  }
}

void RealLsu::comb_tlb_in_stq() {
  for (int k = 0; k < STA; k++) {
    PSta &s1 = pw.s1[k];
    if (!s1.v) continue;
    PStq &s = pnxt.stq[s1.stq];
    const auto &resp = wires->mmu2lsu_io.stq_resp[k];
    if (!resp.valid || resp.entry_idx != s1.stq || resp.result == MMUResultType::MISS) {
      s.trp = 1;  // retry through the STA replay register
      continue;
    }
    if (resp.result == MMUResultType::HIT) {
      s.pa_v = 1;
      s.paddr = resp.paddr;
      s.mmio = lsu_is_mmio_addr(resp.paddr);
      if (s.data_v && !s.wbp && !s.wbd) s.wbp = 1;
    } else {  // page fault: completes without data, raised at commit
      s.pf = 1;
      nxt.lrsc_unit.reserve_valid = false;
      if (!s.wbp && !s.wbd) s.wbp = 1;
    }
    pw.ev_sta |= (stq_mask_t)1 << s1.stq;
    if (s1.replay) continue;
    // memory-order check: CAM over the LDQ with the partial address; a younger load that already read the cache /
    // forwarded while this store address was unknown must replay. The default policy never lets a load pass an
    // unknown-address older store, so this CAM is an assertion-like detector (counts must stay 0).
    const uint32_t j = s1.stq;
    for (int i = 0; i < LDQ_SIZE && !lp_memspec; i++) {
      PLdq &l = pnxt.ldq[i];
      if (l.st != L_DCW && l.st != L_WBP) continue;
      const uint32_t h = (pcur.stq_head_flag ? STQ_SIZE : 0) + pcur.stq_head;
      const uint32_t b = (l.snap_flag ? STQ_SIZE : 0) + l.snap_idx;
      const uint32_t older = std::min<uint32_t>((b + 2 * STQ_SIZE - h) % (2 * STQ_SIZE), pcur.stq_count);
      if (stq_rank(j, pcur.stq_head) >= older) continue;  // store is younger than the load
      if (range_rel(l.vaddr, ld_width(l.func3), s.vaddr, ld_width(s.func3), 12) == DISJOINT) continue;
      l.st = L_RDY;  // replay (a flush from the load would be needed if its result had been consumed)
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------
// L1: partial CAM over the STQ (parallel with the DCache read), MMIO issue
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::comb_stlf() {
  for (int k = 0; k < LD; k++) {
    PL1 &l = pw.l1[k];
    if (!l.v || !l.hit || l.mmio) continue;
    const PLdq &e = pnxt.ldq[l.ldq];
    PLdq tmp = e;
    tmp.paddr = l.paddr;
    stq_mask_t ov = 0;
    for (int q = 0; q < STA; q++)
      if (pw.s1[q].v && !pw.s1[q].replay) ov |= (stq_mask_t)1 << pw.s1[q].stq;
    const CamRes c = stq_cam(pcur, pnxt, ov, tmp, /*full=*/!l.need_tlb, /*spec_ok=*/lp_memspec && !e.second && !e.lrsc && !e.wait);
    pnxt.ldq[l.ldq].spec = c.unk;
    pnxt.ldq[l.ldq].skip = c.skip;
    l.cand_v = c.v;
    l.cand = c.idx;
  }

  // MMIO loads: only the oldest instruction, with no older store left in the STQ, one on the bus at a time
  for (int i = 0; i < LDQ_SIZE; i++) {
    PLdq &e = pnxt.ldq[i];
    if (e.st != L_MMW || nxt.uncached_unit.valid) continue;
    if (pcur.ldq[i].st != L_MMW) continue;  // M: only entries that were already MMW at the start of the cycle
    if (!lsu_mmio_is_oldest_unfinished(&wires->rob_bcast, e.rob_idx)) continue;
    const uint32_t h = (pcur.stq_head_flag ? STQ_SIZE : 0) + pcur.stq_head;
    const uint32_t b = (e.snap_flag ? STQ_SIZE : 0) + e.snap_idx;
    if ((b + 2 * STQ_SIZE - h) % (2 * STQ_SIZE) != 0) continue;
    e.st = L_MMR;
    UncachedUnit &u = nxt.uncached_unit;
    u.valid = true;
    u.is_load = true;
    u.addr = e.paddr;
    u.func3 = e.func3;
    u.idx = i;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Allocation
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::comb_dis2lsu() {
  const DisLsuIO &d = wires->dis2lsu;
  for (int i = 0; i < MAX_LDQ_DISPATCH_WIDTH; i++) {
    if (!d.ldq_alloc_req[i]) continue;
    PLdq &e = pnxt.ldq[d.ldq_idx[i]];
    const uint8_t ep = (e.epoch + 1) & ((1u << EPOCH_W) - 1);
    e = {};
    e.epoch = ep;
    e.rob_idx = d.ldq_rob_idx[i];
    e.rob_flag = d.ldq_rob_flag[i];
    e.br_mask = d.ldq_br_mask[i];
    e.pc = d.ldq_pc[i];
    if (lp_memspec) {
      if (lp_mdp_clr && (long long)sim_time >= g_mdp_next_clear) {
        std::fill(g_mdp.begin(), g_mdp.end(), false);
        g_mdp_next_clear = (long long)sim_time + lp_mdp_clr;
      }
      e.wait = mdp_wait(e.pc);
    }
    e.st = L_ALLOC;
  }
  for (int i = 0; i < MAX_STQ_DISPATCH_WIDTH; i++) {
    if (!d.alloc_req[i]) continue;
    if (pnxt.stq_count >= STQ_SIZE) continue;
    if (d.stq_flag[i] != stq_tail_flag(pnxt.stq_head, pnxt.stq_count, pnxt.stq_head_flag)) continue;
    PStq &e = pnxt.stq[(pnxt.stq_head + pnxt.stq_count) % STQ_SIZE];
    e = {};
    e.rob_idx = d.rob_idx[i];
    e.rob_flag = d.rob_flag[i];
    e.br_mask = d.br_mask[i];
    e.pc = d.st_pc[i];
    e.func3 = d.func3[i];
    pnxt.stq_count++;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// flush / mispredict / clear_mask
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::comb_flush() {
  if (wires->rob_bcast.flush) {
    msd::clear_all();  // every stale value dies with the flush
    for (PLdq &l : pnxt.ldq) {
            l.st = L_EMPTY;
    }
    const uint32_t keep = std::min<uint32_t>(pnxt.stq_ccount, pnxt.stq_count);
    pnxt.stq_count = pnxt.stq_ccount = keep;
    pnxt.stq_commit = stq_idx_after(pnxt.stq_head, keep);
    const bool keep_store = nxt.uncached_unit.valid && !nxt.uncached_unit.is_load &&
                            stq_idx_alive_after_flush(nxt.uncached_unit.idx, pnxt.stq_head, keep) &&
                            pnxt.stq[nxt.uncached_unit.idx].mmw;
    if (!keep_store) nxt.uncached_unit.valid = false;
  }

  const DecBroadcastIO &dec = wires->dec_bcast;
  if (dec.mispred) {
    for (PLdq &l : pnxt.ldq)
      if (l.st != L_EMPTY && (l.br_mask & dec.br_mask) != 0) {
        l.st = L_EMPTY;
      }
    uint32_t keep = pnxt.stq_count;
    for (uint32_t i = 0; i < pnxt.stq_count; i++)
      if ((pnxt.stq[(pnxt.stq_head + i) % STQ_SIZE].br_mask & dec.br_mask) != 0) {
        keep = i;
        break;
      }
    if (keep < pnxt.stq_ccount) {  // invariant used by the RTL (LP_CUT bit 7): a mispredict never cuts a committed store
      std::fprintf(stderr, "[LSU_PIPE] invariant violated: mispredict cut %u below ccount %u\n", keep, pnxt.stq_ccount);
      std::abort();
    }
    pnxt.stq_count = keep;
    UncachedUnit &u = nxt.uncached_unit;
    if (u.valid && (u.is_load ? pnxt.ldq[u.idx].st == L_EMPTY : !stq_idx_alive_after_flush(u.idx, pnxt.stq_head, pnxt.stq_count)))
      u.valid = false;
    if (nxt.lrsc_unit.reserve_valid && (nxt.lrsc_unit.reserve_br_mask & dec.br_mask) != 0) nxt.lrsc_unit.reserve_valid = false;
    pnxt.stq_ccount = std::min<uint32_t>(pnxt.stq_ccount, pnxt.stq_count);
    pnxt.stq_commit = stq_idx_after(pnxt.stq_head, pnxt.stq_ccount);
  }
  if (wires->rob_bcast.flush || dec.mispred) {  // replay registers of dead entries
    for (int r = 0; r < RP_MAX; r++)
      if (pnxt.rp[r].v && pnxt.ldq[pnxt.rp[r].ldq].st == L_EMPTY) pnxt.rp[r].v = 0;
    if (pnxt.srp.v && stq_rank(pnxt.srp.ldq, pnxt.stq_head) >= pnxt.stq_count) pnxt.srp.v = 0;
  }

  if (dec.clear_mask) {  // a branch resolved correctly: its mask bit is free again
    for (int i = 0; i < LDQ_SIZE; i++)
      if (pcur.ldq[i].st != L_EMPTY) pnxt.ldq[i].br_mask &= ~dec.clear_mask;
    for (uint32_t i = 0; i < pcur.stq_count; i++) pnxt.stq[(pcur.stq_head + i) % STQ_SIZE].br_mask &= ~dec.clear_mask;
    if (nxt.lrsc_unit.reserve_valid) nxt.lrsc_unit.reserve_br_mask &= ~dec.clear_mask;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// L1 -> DCache request and the L1/L2 pipeline register
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::comb_lsu2dcache_ldq() {
  auto &ports = wires->out.lsu2dcache.req_ports.load_ports;
  for (int i = 0; i < LSU_LDU_COUNT; i++) ports[i].valid = ports[i].replay = false;
  for (int k = 0; k < LD; k++) {
    const PL1 &l = pw.l1[k];
    if (!l.v) continue;
    PLdq &e = pnxt.ldq[l.ldq];
    if (e.st != L_PIPE) continue;  // killed in this cycle
    PL12 &sl = pnxt.l12[k];
    sl.v = 1;
    sl.ldq = l.ldq;
    sl.tlb = l.tlb;
    sl.mmio = l.mmio;
    sl.cand_v = l.cand_v;
    sl.cand = l.cand;
    sl.dcblk = 0;
    // the DCache is read in parallel with the partial CAM; the read enable is qualified by "no conflicting older store"
    // (12-bit compare, ready with the DTLB result); a port with a pending DCache hidden retry does not launch.
    if (l.hit && !l.mmio && !l.cand_v && wires->in.dcache2lsu.load_retry_busy[k]) {
      sl.dcblk = 1;  // the port belongs to the DCache hidden retry this cycle
    } else if (l.hit && !l.mmio && !l.cand_v) {
      e.epoch = (e.epoch + 1) & ((1u << EPOCH_W) - 1);
      ports[k].valid = true;
      ports[k].addr = l.paddr;
      ports[k].req_id = (uint32_t(e.epoch) << LDQ_IDX_WIDTH) | l.ldq;
      ports[k].replay = e.rtype != (uint8_t)ReplayType::HIT;
    }
    sl.epoch = e.epoch;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// committed-store drain (RTL-aligned form, see rtl docs/analysis/lsu-pipe.md "model changes" M4):
//   1. every eligible store of the committed window counts its replay countdown down (all of them, every cycle);
//   2. the head store, when MMIO, takes the uncached unit;
//   3. the other committed stores go through the write buffer (EXP-024): the oldest registered entries whose stores are all ready and
//      that do not overlap an older entry are sent, one DCache request each, on the store ports without a pending hidden retry
//      (first lp_drain ports); then the stores committed at the cycle start are inserted (wb_insert).
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::comb_lsu2dcache_stq() {
  auto &ports = wires->out.lsu2dcache.req_ports.store_ports;
  for (int i = 0; i < LSU_STA_COUNT; i++) ports[i].valid = ports[i].replay = false;

  const uint32_t n = std::min<uint32_t>(pcur.stq_ccount, pnxt.stq_count);  // H: stores committed at the cycle start
  bool rdy[STQ_SIZE] = {};  // by rank
  for (uint32_t rank = 0; rank < n; rank++) {
    const uint32_t j = (pnxt.stq_head + rank) % STQ_SIZE;
    PStq &s = pnxt.stq[j];
    if (!(s.cmt && !s.done && !s.mmw)) continue;
    if (pcur.stq[j].iss) continue;  // in flight, or answered this cycle: re-send earliest next cycle
    if (s.suppress) { std::fprintf(stderr, "[LSU_PIPE] committed suppress store not done (unreachable)\n"); std::abort(); }
    const uint32_t rw_seen = (pw.st_fullwake >> j & 1) ? pw.st_rwpre[j] : s.rwait;  // G
    if (s.rwait > 0) s.rwait--;
    rdy[rank] = rw_seen <= 1;
  }
  if (n > 0 && rdy[0] && pnxt.stq[pnxt.stq_head].mmio && !nxt.uncached_unit.valid) {  // MMIO store: only the head
    PStq &s = pnxt.stq[pnxt.stq_head];
    UncachedUnit &u = nxt.uncached_unit;
    u.valid = true;
    u.is_load = false;
    u.addr = s.paddr;
    u.wdata = s.data;
    u.func3 = s.func3;
    u.idx = pnxt.stq_head;
    s.mmw = 1;
  }
  // free ports among the first lp_drain (no hidden retry pending)
  int free_port[DRAIN_MAX], nfree = 0;
  for (int t = 0; t < lp_drain; t++)
    if (!wires->in.dcache2lsu.store_retry_busy[t]) free_port[nfree++] = t;
  {  // EXP-024: the committed stores go through the write buffer
    bool issued_now[WB_MAX] = {};
    const LsuPipeState &C = pcur;  // the decision reads the registered buffer only
    int order[WB_MAX], no = 0;
    for (int e = 0; e < WB_MAX; e++) if (C.wb[e].v) order[no++] = e;
    std::sort(order, order + no, [&](int a, int b) { return C.wb[a].seq < C.wb[b].seq; });
    int used = 0;
    for (int k = 0; k < no && k < lp_wbscan && used < nfree; k++) {
      const PWbE &w = C.wb[order[k]];
      if (w.st != 0) continue;
      bool ready = true, blocked = false;
      for (int x = 0; x < w.cnt && ready; x++) {
        const uint32_t jx = w.mem[x], rx = stq_rank(jx, pnxt.stq_head);
        if (rx >= n || !rdy[rx]) { ready = false; break; }
        const PStq &sx = pnxt.stq[jx];
        const uint8_t stx = get_store_strb(sx.paddr, sx.func3);
        for (int k2 = 0; k2 < k && !blocked; k2++) {  // older registered entries (not done: they are still valid)
          const PWbE &o = C.wb[order[k2]];
          for (int y = 0; y < o.cnt && !blocked; y++) {
            const PStq &sy = pnxt.stq[o.mem[y]];
            if (CheckAddr(sy.paddr, get_store_strb(sy.paddr, sy.func3), sx.paddr, stx)) blocked = true;
          }
        }
      }
      if (!ready) continue;
      if (blocked) { continue; }
      PStq &lead = pnxt.stq[w.mem[0]];
      auto &p = ports[free_port[used++]];
      p.valid = true;
      p.addr = lead.paddr;
      p.strb = get_store_strb(lead.paddr, lead.func3);
      p.req_id = w.mem[0];
      p.replay = lead.rtype != (uint8_t)ReplayType::HIT;
      lead.mf = 0;
      lead.iss = 1;
      for (int x = 1; x < w.cnt; x++) {
        PStq &f = pnxt.stq[w.mem[x]];
        f.iss = 1;
        f.mf = 1;
        f.ml = w.mem[0];
      }
      pnxt.wb[order[k]].st = 1;
      issued_now[order[k]] = true;
    }
    wb_insert(pnxt, C, n, issued_now);
  }
}

// ---------------------------------------------------------------------------------------------------------------
// STQ retire, wake-up pass (events of this cycle -> LDQ entries), replay-register fill
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::comb_check() {
  int ret = 0;
  while (ret < RETIRE_PORTS && pnxt.stq_ccount > 0 && pnxt.stq_count > 0 && pnxt.stq[pnxt.stq_head].done &&
         (uint32_t)ret < pcur.stq_ccount && (uint32_t)ret < pcur.stq_count && pcur.stq[pnxt.stq_head].done) {  // R: registered state only
    pw.ev_ret |= (stq_mask_t)1 << pnxt.stq_head;
    pnxt.stq_count--;
    pnxt.stq_ccount--;
    pnxt.stq_head = (pnxt.stq_head + 1) % STQ_SIZE;
    if (pnxt.stq_head == 0) pnxt.stq_head_flag = !pnxt.stq_head_flag;
    ret++;
  }
  LP_PORT_CHECK(ret, RETIRE_PORTS, "stq_retire");

  // DCache response watchdog: a request whose response never arrives (lost bank-retry latch in the DCache) is replayed
  for (int i = 0; i < LDQ_SIZE; i++) {
    PLdq &e = pnxt.ldq[i];
    if (e.st != L_DCW) { if (e.wd) e.wd = 0; continue; }   // (no store to an idle entry: it would dirty the line every cycle)
    if (++e.wd == 63) { e.wd = 0; e.st = L_RDY; }
  }
  for (int j = 0; j < STQ_SIZE; j++) {
    PStq &t = pnxt.stq[j];
    if (!t.iss) { if (t.wd) t.wd = 0; continue; }
    if (++t.wd == 63) { t.wd = 0; t.iss = 0; t.rwait = 0; if (lp_wbuf > 0 && t.wbi == 1) { PWbE &w = pnxt.wb[t.wbe]; if (w.v && w.mem[0] == (uint8_t)j) w.st = 0; } }
  }
  // wake-up pass
  uint32_t full_wakes = 0;
  for (int i = 0; i < LDQ_SIZE; i++) {
    PLdq &e = pnxt.ldq[i];
    if (e.st != L_RPW) continue;
    const stq_mask_t b = (stq_mask_t)1 << e.blk_idx;
    switch (e.rcause) {
    case RC_TLB:
      if (e.rwait > 0) e.rwait--;
      if (e.rwait == 0) e.st = L_RDY;
      break;
    case RC_DC:
      if (pw.fill && e.rtype == (uint8_t)ReplayType::MSHR_HIT && cache_line_match(e.paddr, pw.fill_addr)) {
        e.rwait = 0;
      } else if (pw.fill && e.rtype == (uint8_t)ReplayType::MSHR_FULL && full_wakes < (uint32_t)LD &&
                 pcur.ldq[i].st == L_RPW) {
        e.rwait = 0;
        full_wakes++;
      } else if (e.rwait > 0) {
        e.rwait--;
      }
      if (e.rwait == 0) e.st = L_RDY;
      break;
    case RC_ADDR:
      if ((pw.ev_sta | pw.ev_ret) & b) {  // STA broadcast: confirm against the store now (same decision as L2)
        const PStq &s = pnxt.stq[e.blk_idx];
        const Conf c = stlf_confirm(e, s, stq_rank(e.blk_idx, pcur.stq_head) < pcur.stq_count && !(pw.ev_ret & b));  // B
        if (c == CF_REPLAY) e.st = L_RDY;
        else if (c == CF_HOLD) e.rcause = RC_HOLD;
        else if (c == CF_DATA) e.rcause = RC_DATA;
        else if (c == CF_FWD) e.st = L_WBP;
      }
      break;
    case RC_HOLD:
      if ((pw.ev_ret | pw.ev_sc) & b) e.st = L_RDY;
      break;
    case RC_DATA:
      if (pw.ev_data & b)
        e.st = L_WBP;  // the STD write is forwarded straight to the write-back (decision was already made at L2)
      else if (pw.ev_ret & b)
        e.st = L_RDY;
      break;
    default: break;
    }
  }

  // replay registers: find-first over the RDY mask including this cycle's wake-ups (selected entry re-enters at L1 next cycle)
  for (int r = 0; r < lp_rp_ports; r++) {
    if (pnxt.rp[r].v) continue;
    for (int k = 0; k < LDQ_SIZE; k++) {  // find-first over the 64-bit RDY vector rotated by rp_ptr
      const int i = (pnxt.rp_ptr + k) % LDQ_SIZE;
      if (pnxt.ldq[i].st != L_RDY) continue;
      if (pcur.ldq[i].st != L_RDY) continue;  // D: registered RDY bits
      pnxt.ldq[i].st = L_PIPE;
      pnxt.rp[r].v = 1;
      pnxt.rp[r].ldq = i;
      pnxt.rp_ptr = (i + 1) % LDQ_SIZE;
      break;
    }
  }
  if (!pnxt.srp.v) {  // STA replay register: store with a pending DTLB retry, round-robin over the STQ index
    // Liveness: the DTLB walk is shared, a store of another page only gets RETRY (OTHER_WALK_ACTIVE) until the owner of the walk
    // asks again. Always taking the oldest store starves a younger walk owner (Linux boot, ROB deadlock), so the pick rotates
    // (like rp_ptr for the loads).
    for (uint32_t k = 0; k < STQ_SIZE; k++) {
      const uint32_t j = (pnxt.srp_ptr + k) % STQ_SIZE;
      if (stq_rank(j, pnxt.stq_head) >= pnxt.stq_count || !pnxt.stq[j].trp) continue;
      pnxt.stq[j].trp = 0;
      pnxt.srp.v = 1;
      pnxt.srp.ldq = j;
      pnxt.srp_ptr = (j + 1) % STQ_SIZE;
      break;
    }
  }
}
