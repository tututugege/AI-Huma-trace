// RealLsuPipe.cpp -- staged LSU (switch LSU_PIPE / CFG_LSU_PIPE). Design and rules: docs/LSU_PIPE.md.
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

// EXP-019: cost of a memory-order violation (LSU_PIPE_STATS prints the averages).  det[] = cycle of the detection per ROB index;
// the ROB calls on_flush_commit() when the violating load commits and flushes, and on_commit() for every later commit.
struct MsPenalty {
  long long det[256], det_zero[256];
  long long pending = -1;
  uint64_t zero_cum = 0, ep_zero0 = 0;  // cycles without any commit so far / at the detection of the current episode
  uint64_t n = 0, n_det = 0, det_flush_sum = 0, resume_sum = 0, resume_n = 0, flushed_sum = 0, idle_sum = 0;
  MsPenalty() { for (auto &d : det) d = -1; }
  void on_detect(unsigned rob_idx) { det[rob_idx & 255] = sim_time; det_zero[rob_idx & 255] = zero_cum; }
  void on_flush_commit(unsigned rob_idx, long long younger) {
    n++;
    flushed_sum += younger;
    long long &d = det[rob_idx & 255];
    ep_zero0 = zero_cum;
    if (d >= 0) { n_det++; det_flush_sum += sim_time - d; ep_zero0 = det_zero[rob_idx & 255]; d = -1; }
    pending = sim_time;
  }
  void tick(bool committed) {  // once per cycle from the ROB
    if (!committed) zero_cum++;
    else if (pending >= 0 && sim_time > pending) { resume_sum += sim_time - pending; resume_n++; idle_sum += zero_cum - ep_zero0; pending = -1; }
  }
} g_msp;

void g_msp_flush_commit(unsigned rob_idx, long long younger) { g_msp.on_flush_commit(rob_idx, younger); }
void g_msp_tick(bool committed) { g_msp.tick(committed); }

uint64_t g_lp_cancel_entries = 0;  // entries put back to waiting by a cancelled speculative wake-up (LSU_PIPE_L2 bit 8)

#define LP_PORT_CHECK(used, max, what)                                                                  \
  do {                                                                                                  \
    if ((int)(used) > (int)(max)) {                                                                     \
      std::fprintf(stderr, "[LSU_PIPE] port limit exceeded: %s used=%d max=%d\n", what, (int)(used), (int)(max)); \
      std::abort();                                                                                     \
    }                                                                                                   \
  } while (0)

namespace {

struct LpStats {
  uint64_t l1_new = 0, l1_replay = 0, l1_tlb = 0;
  uint64_t dc_launch = 0, dc_wasted = 0;
  uint64_t hit = 0, fwd = 0, fwd_wait = 0;
  uint64_t cand = 0, cand_false = 0;
  uint64_t rp_tlb = 0, rp_addr = 0, rp_hold = 0, rp_data = 0, rp_dc = 0, rp_dc_conflict = 0, rp_dc_mshr = 0;
  uint64_t rdy_cycles_blocked = 0;
  uint64_t unk_wait_overlap = 0, unk_wait_disjoint = 0;
  uint64_t order_violation = 0, spec_loads = 0, spec_released = 0, vio_multi = 0;
  uint64_t st_drain = 0, st_conflict = 0, st_replay = 0, st_merge_pair = 0;
  uint64_t sta_replay = 0, bounce = 0, wd_ld = 0, wd_st = 0, rblk = 0;
  uint64_t mmio_ld = 0;
  uint64_t spec_pred = 0, spec_ok = 0, spec_fail = 0, oracle_wake = 0, l3_wake = 0, l3_wb = 0, spec_sw_wbp = 0;
  ~LpStats() {
    const char *e = std::getenv("LSU_PIPE_STATS");
    if (!e || e[0] == '0' || l1_new == 0) return;
    std::fprintf(stderr,
                 "[LSU_PIPE_STATS] l1_new=%llu l1_replay=%llu l1_tlb=%llu dc_launch=%llu dc_wasted=%llu hit=%llu fwd=%llu fwd_wait=%llu "
                 "cand=%llu cand_false=%llu\n",
                 (unsigned long long)l1_new, (unsigned long long)l1_replay, (unsigned long long)l1_tlb, (unsigned long long)dc_launch,
                 (unsigned long long)dc_wasted, (unsigned long long)hit, (unsigned long long)fwd, (unsigned long long)fwd_wait,
                 (unsigned long long)cand, (unsigned long long)cand_false);
    std::fprintf(stderr,
                 "[LSU_PIPE_STATS] replay: tlb=%llu addr=%llu hold=%llu data=%llu dc=%llu (conflict=%llu mshr=%llu) "
                 "unk_addr_wait_overlap=%llu unk_addr_wait_disjoint=%llu order_violation=%llu spec_loads=%llu spec_released=%llu vio_multi_cycles=%llu\n",
                 (unsigned long long)rp_tlb, (unsigned long long)rp_addr, (unsigned long long)rp_hold, (unsigned long long)rp_data,
                 (unsigned long long)rp_dc, (unsigned long long)rp_dc_conflict, (unsigned long long)rp_dc_mshr,
                 (unsigned long long)unk_wait_overlap, (unsigned long long)unk_wait_disjoint, (unsigned long long)order_violation, (unsigned long long)spec_loads, (unsigned long long)spec_released, (unsigned long long)vio_multi);
    if (g_msp.n)
      std::fprintf(stderr,
                   "[MS_PENALTY] violations=%llu flush_commits=%llu detect_to_flush_commit_avg=%.2f flush_commit_to_next_commit_avg=%.2f "
                   "total_avg=%.2f flushed_younger_avg=%.1f idle_commit_cycles_avg=%.2f\n",
                   (unsigned long long)order_violation, (unsigned long long)g_msp.n,
                   g_msp.n_det ? (double)g_msp.det_flush_sum / g_msp.n_det : 0.0,
                   g_msp.resume_n ? (double)g_msp.resume_sum / g_msp.resume_n : 0.0,
                   (g_msp.n_det ? (double)g_msp.det_flush_sum / g_msp.n_det : 0.0) + (g_msp.resume_n ? (double)g_msp.resume_sum / g_msp.resume_n : 0.0),
                   (double)g_msp.flushed_sum / g_msp.n, g_msp.resume_n ? (double)g_msp.idle_sum / g_msp.resume_n : 0.0);
    std::fprintf(stderr,
                 "[LSU_PIPE_STATS] watchdog ld=%llu st=%llu rblk=%llu\n", (unsigned long long)wd_ld, (unsigned long long)wd_st, (unsigned long long)rblk);
    std::fprintf(stderr, "[LSU_PIPE_STATS] L2cut: l3_wb=%llu l3_wake=%llu spec_pred=%llu spec_ok=%llu spec_fail=%llu oracle_wake=%llu cancel_entries=%llu\n",
                 (unsigned long long)l3_wb, (unsigned long long)l3_wake, (unsigned long long)spec_pred, (unsigned long long)spec_ok,
                 (unsigned long long)spec_fail, (unsigned long long)oracle_wake, (unsigned long long)g_lp_cancel_entries);
    std::fprintf(stderr, "[LSU_PIPE_STATS] store: drain=%llu conflict_block=%llu replay=%llu merge_pair=%llu sta_replay=%llu mmio_ld=%llu bounce=%llu\n",
                 (unsigned long long)st_drain, (unsigned long long)st_conflict, (unsigned long long)st_replay,
                 (unsigned long long)st_merge_pair, (unsigned long long)sta_replay, (unsigned long long)mmio_ld, (unsigned long long)bounce);
  }
} g_st;

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

inline bool lp_line_eq(uint32_t a, uint32_t b) { return (a >> DCACHE_OFFSET_BITS) == (b >> DCACHE_OFFSET_BITS); }

inline uint32_t replay_wait(ReplayType r) {
  return r == ReplayType::MSHR_HIT || r == ReplayType::MSHR_FULL ? LSU_PEPLAY_WAIT_CYCLES : 0;
}

static const unsigned lp_memspec = lsu_pipe_env_mask("LSU_MEMSPEC", CFG_LSU_MEMSPEC);  // EXP-018, see config.h
// Wait table (Alpha 21264 style): 2^LSU_MDP_BITS one-bit entries indexed by a PC hash, set when a load of that PC violated, cleared every
// LSU_MDP_CLR cycles (0 = never). A load whose bit is set does not pass unknown-address stores. LSU_MDP_BITS=0 disables the predictor.
static const unsigned lp_mdp_bits = lsu_pipe_env_mask("LSU_MDP_BITS", CFG_LSU_MDP_BITS);
static const unsigned lp_mdp_clr = lsu_pipe_env_mask("LSU_MDP_CLR", CFG_LSU_MDP_CLR);
// Mode 1 = store sets (Chrysos/Emer): a violating (load, store) pair gets the same set id in a PC-indexed table (merging keeps the smaller id);
// a load waits only for unknown-address older stores of its own set. Mode 0 = the wait table above.
static const unsigned lp_mdp_mode = lsu_pipe_env_mask("LSU_MDP_MODE", 0);
// LSU_MDP_HW (default 1, mode 0 only): the implementable training of the RTL (EXP-019). The violating loads of a cycle are all marked, but only
// the lowest LDQ index trains the wait table (one write port), and the table bit is written at the clock edge, i.e. the lookups of the next
// cycle see it (the lookups of the cycle of the violation do not). LSU_MDP_HW=0: every violating load trains at once (EXP-018 behaviour).
static const unsigned lp_mdp_hwm = lp_mdp_mode == 0 ? lsu_pipe_env_mask("LSU_MDP_HW", 1) : 0;
static const bool lp_mdp_hw = lp_mdp_hwm != 0;
// LSU_MDP_HW=2: the violators of a cycle are only flagged (one bit per LDQ entry, a register); one flagged live entry per cycle (the lowest LDQ index)
// trains the table in the following cycle, so the table is written two cycles after the violation, from registers (no STA-arrival-to-table-write path)
static const bool lp_mdp_hw2 = lp_mdp_hwm == 2;
static uint64_t g_trn = 0;
// LSU_MS_SLOTS=N (default 0 = unlimited): at most N loads (entries with `spec` set that are not yet second-pass) may be speculative at a time, a
// cheaper variant for the RTL (the violation CAM / SPD release scan only needs N tracking entries instead of every LDQ entry).
static const unsigned lp_ms_slots = lsu_pipe_env_mask("LSU_MS_SLOTS", 0);
static const bool lp_ms_trace = std::getenv("LSU_MS_TRACE") != nullptr;  // debug aid: memory-speculation events (matches the RTL MS_DBG print)
static const unsigned lp_mdp_ntr = lsu_pipe_env_mask("LSU_MDP_NTR", 2);  // table write ports (trains per cycle, lowest LDQ indices first)
static std::vector<int> g_mdp_pend;  // table indices trained in the previous cycle, written at the start of this one
static inline uint32_t mdp_idx(uint32_t pc) { return ((pc >> 2) ^ (pc >> (2 + lp_mdp_bits))) & ((1u << lp_mdp_bits) - 1); }
static std::vector<uint16_t> g_ssit;
static unsigned g_ssit_next = 1;
static inline uint16_t ss_get(uint32_t pc) {
  if (g_ssit.empty()) g_ssit.assign(1u << lp_mdp_bits, 0);
  return g_ssit[mdp_idx(pc)];
}
static inline void ss_train(uint32_t lpc, uint32_t spc) {
  if (g_ssit.empty()) g_ssit.assign(1u << lp_mdp_bits, 0);
  uint16_t &a = g_ssit[mdp_idx(lpc)], &b = g_ssit[mdp_idx(spc)];
  if (!a && !b) { a = b = (uint16_t)g_ssit_next; g_ssit_next = g_ssit_next >= 65535 ? 1 : g_ssit_next + 1; }
  else if (a && !b) b = a;
  else if (!a && b) a = b;
  else a = b = std::min(a, b);
}
// Confidence counters (modes 0 and 1): a violation adds 1 (saturating at 3), a load that passed unknown-address stores without a violation
// subtracts LSU_MDP_DEC; a load/set waits only while the counter is >= LSU_MDP_THR (THR=1, DEC=0: the original sticky 1-bit behaviour).
static const unsigned lp_mdp_thr = lsu_pipe_env_mask("LSU_MDP_THR", 1);
static const unsigned lp_mdp_dec = lsu_pipe_env_mask("LSU_MDP_DEC", 0);
static std::vector<uint8_t> g_mdp;
static long long g_mdp_next_clear = 0;
static inline bool mdp_wait(uint32_t pc) {
  if (!lp_mdp_bits) return false;
  if (g_mdp.empty()) g_mdp.assign(1u << lp_mdp_bits, 0);
  return g_mdp[mdp_idx(pc)] >= lp_mdp_thr;
}
static inline void mdp_write_pending();
static inline void mdp_train(uint32_t pc) {
  if (!lp_mdp_bits) return;
  if (g_mdp.empty()) g_mdp.assign(1u << lp_mdp_bits, 0);
  uint8_t &c = g_mdp[mdp_idx(pc)];
  if (c < 3) c++;
}
static inline void mdp_write_pending() {
  if (g_mdp_pend.empty()) return;
  if (g_mdp.empty()) g_mdp.assign(1u << lp_mdp_bits, 0);
  for (int ix : g_mdp_pend) {
    uint8_t &c = g_mdp[ix];
    if (c < 3) c++;
  }
  g_mdp_pend.clear();
}
static inline void mdp_clean(uint32_t pc) {
  if (!lp_mdp_bits || !lp_mdp_dec || g_mdp.empty()) return;
  uint8_t &c = g_mdp[mdp_idx(pc)];
  c = c > lp_mdp_dec ? c - lp_mdp_dec : 0;
}

// youngest older store that may conflict (partial CAM on page offset + size; unknown address always conflicts).
struct CamRes {
  bool v = false;
  uint8_t idx = 0;
  bool unk = false;  // memory speculation: an older store younger than the candidate had no address (the load passes it)
  uint64_t skip = 0; // LSU_MS_DATA: the unknown-address stores that were passed (bit = STQ index)
};
// `ov`: STQ entries whose STA arrives in this very cycle (AGU output): their vaddr/size bypass into the CAM (a 4x4
// comparator set against the STA ports in hardware), so a load issued together with an older STA is not stalled.
CamRes stq_cam(const LsuPipeState &s, const LsuPipeState &n, uint64_t ov, const PLdq &l, bool full, bool spec_ok = false) {
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
      if (spec_ok && !(lp_mdp_mode == 1 && l.sset && st.sset == l.sset)) {  // pass the unknown-address store (checked when its address arrives)
        unk_rank = std::max(unk_rank, (int)rank);
        r.skip |= 1ULL << j;
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
bool ms_load_value(const LsuPipeState &S, const PLdq &l, bool apply_skip, uint32_t &val) {
  if (!l.pa_v || !pmem_is_ram_addr(l.paddr & ~3u, 4u)) return false;
  const uint32_t W = l.paddr & ~3u, off = l.paddr & 3, lw = ld_width(l.func3);
  const uint32_t head = S.stq_head;
  uint32_t older;
  {
    const uint32_t h = (S.stq_head_flag ? STQ_SIZE : 0) + head;
    const uint32_t b = (l.snap_flag ? STQ_SIZE : 0) + l.snap_idx;
    older = std::min<uint32_t>((b + 2 * STQ_SIZE - h) % (2 * STQ_SIZE), S.stq_count);
  }
  uint32_t word = 0;
  bool have = false;
  for (uint32_t rank = 0; rank < S.stq_count && !have; rank++) {
    const PStq &t = S.stq[(head + rank) % STQ_SIZE];
    if (!t.m_v || t.m_w != W || t.done || t.suppress) continue;
    word = t.m_old;
    have = true;
  }
  if (!have) word = tbhook::mem_read(W);
  for (uint32_t rank = 0; rank < older; rank++) {
    const uint32_t j = (head + rank) % STQ_SIZE;
    const PStq &t = S.stq[j];
    if (!t.m_v || t.m_w != W || t.done || t.suppress || t.m_atnt) continue;
    if (apply_skip && (l.skip >> j & 1)) continue;
    if (t.m_amo) {
      word = t.m_new;
    } else {
      const uint32_t data = t.data_v ? (uint32_t)t.data : t.m_rs2;
      for (uint32_t k = t.m_off; k < t.m_off + t.m_sz && k < 4; k++) word = (word & ~(0xffu << (8 * k))) | (((data >> (8 * (k - t.m_off))) & 0xffu) << (8 * k));
    }
  }
  const uint32_t raw = lw >= 4 ? word : (word >> (8 * off)) & mem_mask_for_width(lw);
  val = msd::load_extend(raw, l.func3);
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------------------------------------------
// cycle start
// ---------------------------------------------------------------------------------------------------------------
// Critical-path experiments (docs/LSU_PIPE.md section 6): LSU_PIPE_CUT is a bit mask, default CFG_LSU_PIPE_CUT (= the model of section 5 when 0).
//   1 = B: wake-up confirm uses the registered STQ window minus ev_ret (no same-cycle count chain)
//   2 = R: STQ retire reads registered state only (ccount / done of the cycle start)
//   4 = D: replay registers pick only from RDY bits that were registered at the cycle start
//   32 = H: the store drain only sees stores that were already committed at the cycle start
//   16 = G: a store woken by an MSHR-full fill can be drained from the next cycle on (the wake is not seen by this cycle's drain)
//   8 = F: write-back of waiting loads (WBP) picks only from WBP bits registered at the cycle start
//   64 = FM: (with F) the load whose MMIO response arrives in this cycle may still be picked in this cycle (MMIO loads run one at a time and
//       the next one waits for this one to finish, so the extra cycle of F lands on every MMIO load: h264ref sp9 has 2.8M of them, -16%)
static const unsigned lp_cut = lsu_pipe_env_mask("LSU_PIPE_CUT", CFG_LSU_PIPE_CUT);
// L2 cuts (docs/LSU_PIPE.md section 7), bit mask in LSU_PIPE_L2, default CFG_LSU_PIPE_L2:
//   1 = a: L3 register: the direct completion decided at L2 is written back one cycle later (LDQ state L_WB3, load-to-use +1)
//   2 = a2: (needs 1) the consumers are woken from the L3 register in the cycle of the write-back request, not one cycle later from the
//       PRF write-back register (no speculation: the decision is registered)
//   4 = oracle: speculative wake-up at L2 for loads that really hit (upper bound of 8: no cancel, no wrong wake-ups)
//   128 = Y: (with 8) no speculative wake-up for a load that comes from a replay register (only first-pass loads)
//   64 = S: at most one SC completion per cycle
//   32 = M: an MMIO load that becomes MMW at L2 in this cycle is issued to the bus one cycle later (the issue looks at registered MMW only)
//   16 = F: a load that gets MSHR_FULL in this very cycle is not woken by this cycle's MSHR fill (it waits for its counter); only
//       loads parked at the start of the cycle compete for the (first LD) MSHR-FULL wake-ups
//   8 = d: speculative wake-up at L2 from the hit prediction made at L1 (TLB hit, no MMIO, no CAM candidate, no hidden-retry block);
//       a wrong prediction is cancelled at the start of the next cycle (woken entries wait again) before the select
static const unsigned lp_l2 = lsu_pipe_env_mask("LSU_PIPE_L2", CFG_LSU_PIPE_L2);
static const int lp_drain = std::max(1, std::min<int>(DRAIN_MAX, (int)lsu_pipe_env_mask("LSU_PIPE_DRAIN", CFG_LSU_PIPE_DRAIN)));
static const int lp_dcand = (int)lsu_pipe_env_mask("LSU_PIPE_DCAND", CFG_LSU_PIPE_DCAND);

void RealLsu::comb_cal_p() {
  pw = {};
  if (lp_mdp_hw && lp_mdp_bits) mdp_write_pending();
  if (lp_mdp_hw2 && lp_mdp_bits) {  // train one flagged live entry (flags of earlier cycles); entries that died are dropped
    for (int i = 0; i < LDQ_SIZE && g_trn; i++)  // the flag of an entry that died is dropped (the RTL clears it when the entry is EMPTY)
      if ((g_trn >> i & 1) && pcur.ldq[i].st == L_EMPTY) g_trn &= ~(1ULL << i);
    for (int i = 0; i < LDQ_SIZE && g_trn; i++)
      if (g_trn >> i & 1) { g_trn &= ~(1ULL << i); g_mdp_pend.push_back(mdp_idx(pcur.ldq[i].pc)); break; }
  }
  for (int k = 0; k < LD; k++) pnxt.l12[k] = {};  // L1/L2 registers are rewritten every cycle
  for (int k = 0; k < LD; k++) pnxt.l3[k] = {};
  for (int k = 0; k < LD; k++) pnxt.sf[k] = {};
  for (auto &w : wires->lsu_awake.wake) w = {};
  if (msd::on) {  // trace info of the stores allocated in an earlier cycle (the fact leaves the bridge when the store commits)
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
      t.m_off = f->vaddr & 3;
      t.m_sz = f->size;
    }
  }
}

// start of the cycle: the cancel registers of the previous cycle go to the issue queues / busy table (before the select)
void RealLsu::comb_spec_cancel_p() {
  for (int k = 0; k < LD; k++) {
    wires->lsu_awake.cancel[k].valid = pcur.sf[k].v;
    wires->lsu_awake.cancel[k].preg = pcur.sf[k].preg;
  }
}

void RealLsu::comb_lsu2rob_p() {
  wires->lsu2rob = {};
  wires->lsu2rob.committed_store_pending = pcur.stq_ccount != 0;
  uint64_t m = 0;
  for (const PLdq &l : pcur.ldq)
    if (l.miss && l.st != L_EMPTY) m |= 1ULL << l.rob_idx;
  wires->lsu2rob.tma.miss_mask = m;
}

void RealLsu::comb_lsu2dis_p() {
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
void RealLsu::comb_stq_commit_p() {
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
void RealLsu::comb_mmio_in_p() {
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

void RealLsu::comb_mmio_out_p() {
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
void RealLsu::comb_dcache2lsu_ldq_p() {
  const DcacheLsuIO &dc = wires->in.dcache2lsu;
  pw.fill = dc.mshr_fill && LSU_WAIT_MSHR_FILL;
  pw.fill_addr = dc.mshr_fill_addr;

  auto park = [&](PLdq &e, uint8_t cause, uint32_t blk, uint32_t wait) {
    e.st = L_RPW;
    e.rcause = cause;
    e.blk_idx = blk;
    e.rwait = wait;
    switch (cause) {
    case RC_TLB: g_st.rp_tlb++; break;
    case RC_ADDR: g_st.rp_addr++; break;
    case RC_HOLD: g_st.rp_hold++; break;
    case RC_DATA: g_st.rp_data++; break;
    default: g_st.rp_dc++; break;
    }
  };

  bool predhit[LD] = {}, spec[LD] = {};
  const bool dead_now = wires->rob_bcast.flush;
  auto killed_now = [&](const PLdq &e) { return dead_now || (wires->dec_bcast.mispred && (e.br_mask & wires->dec_bcast.br_mask) != 0); };
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
      g_st.mmio_ld++;
      continue;
    }
    if (!sl.cand_v && sl.dcblk) {  // not launched (hidden retry owns the port): replay like a bank conflict
      park(e, RC_DC, 0, replay_wait(ReplayType::CONFLICT));
      continue;
    }
    if (!sl.cand_v) {  // no conflicting older store: the DCache read of L1 decides
      e.st = L_DCW;
      predhit[i] = true;  // hit predicted at the end of L1 (the DCache read was launched)
      if ((lp_l2 & 8) && !((lp_l2 & 128) && sl.rpl) && e.dest_preg != 0 && !killed_now(e)) {  // d: speculative wake-up from the L1/L2 register
        wires->lsu_awake.wake[i].valid = true;
        wires->lsu_awake.wake[i].preg = e.dest_preg;
        spec[i] = true;
        g_st.spec_pred++;
      }
      continue;
    }
    // STLF confirm against the registered STQ entry (full physical address check)
    g_st.cand++;
    const uint32_t j = sl.cand;
    const PStq &s = pcur.stq[j];
    const uint32_t rank = stq_rank(j, pcur.stq_head);
    switch (stlf_confirm(e, s, rank < pcur.stq_count)) {
    case CF_REPLAY: e.st = L_RDY; g_st.cand_false++; break;  // false partial match / candidate gone: replay with the full compare
    case CF_NOADDR: park(e, RC_ADDR, j, 0); break;
    case CF_HOLD: park(e, RC_HOLD, j, 0); break;
    case CF_DATA: park(e, RC_DATA, j, 0); g_st.fwd_wait++; break;
    case CF_FWD:  // hit/forward select picks the STQ data
      g_st.fwd++;
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
    if (!(e.st == L_DCW && e.epoch == ep)) {
      g_st.dc_wasted++;  // request of a load that forwarded / replayed / was killed
      continue;
    }
    e.rtype = (uint8_t)r.replay;
    if (r.replay == ReplayType::HIT) {
      g_st.hit++;
      if (pw.wbd_v[i]) {
        e.st = L_WBP;
      } else {
        pw.wbd_v[i] = true;
        pw.wbd_ldq[i] = idx;
      }
    } else {
      e.miss = 1;
      park(e, RC_DC, 0, replay_wait(r.replay));
      if (r.replay == ReplayType::CONFLICT) g_st.rp_dc_conflict++; else g_st.rp_dc_mshr++;
    }
  }

  // speculative / oracle wake-up bookkeeping: did the predicted-hit load of port i complete directly (DCache HIT this cycle)?
  for (int i = 0; i < LD; i++) {
    if (!predhit[i]) continue;
    const PL12 &sl = pcur.l12[i];
    PLdq &e = pnxt.ldq[sl.ldq];
    const bool ok = pw.wbd_v[i] && pw.wbd_ldq[i] == sl.ldq;
    if (spec[i]) {
      if (ok) {
        e.sw = 1;
        g_st.spec_ok++;
      } else {
        pnxt.sf[i].v = 1;
        pnxt.sf[i].preg = e.dest_preg;
        g_st.spec_fail++;
      }
    } else if ((lp_l2 & 4) && ok && e.dest_preg != 0 && !killed_now(e)) {  // oracle: wake only the loads that hit
      wires->lsu_awake.wake[i].valid = true;
      wires->lsu_awake.wake[i].preg = e.dest_preg;
      e.sw = 1;
      g_st.oracle_wake++;
    }
  }

  if (lp_l2 & 1) {  // a: the direct completions go through the L3 register
    for (int i = 0; i < LD; i++) {
      if (!pw.wbd_v[i]) continue;
      pnxt.l3[i].v = 1;
      pnxt.l3[i].ldq = pw.wbd_ldq[i];
      pnxt.ldq[pw.wbd_ldq[i]].st = L_WB3;
      pw.wbd_v[i] = false;
    }
  }
}

void RealLsu::comb_dcache2lsu_stq_p() {
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
      g_st.st_replay++;
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
        pw.st_fullwake |= 1ULL << ((pnxt.stq_head + k) % STQ_SIZE);
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
  if (tbhook::poison && !l.pf && !l.mmio) wb.result ^= 0xA5C3A5C3u;
  wb.dest_preg = l.dest_preg;
  wb.page_fault_load = l.pf;
  wb.dest_en = l.dest_preg != 0;  // loads to x0 still complete without a destination
  wb.is_mmio = l.mmio;
  const bool second = l.second;
  const bool defer = lp_memspec && !second && l.spec && !l.pf && !l.mmio && !l.lrsc;
  bool early = l.sw || second;
  if (defer) wb.rob_defer = true;
  if (second) wb.flush_pipe = l.vio;
  if (!early && from_l3 && (lp_l2 & 2) && l.dest_preg != 0 && !wires->rob_bcast.flush &&
      !(wires->dec_bcast.mispred && (l.br_mask & wires->dec_bcast.br_mask) != 0)) {  // a2: wake from the L3 register
    wires->lsu_awake.wake[LD + port].valid = true;
    wires->lsu_awake.wake[LD + port].preg = l.dest_preg;
    early = true;
    g_st.l3_wake++;
  }
  if (from_l3) g_st.l3_wb++;
  wb.early_woken = early;
  l.sw = 0;
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
  if (msd::on && !l.pf && !l.mmio && !l.lrsc) {
    const tb::Fact *f = tbhook::fact_by_rob(l.rob_idx);
    const bool plain = f && f->is_load && f->rd_valid && (f->inst & 0x7f) == 0x03;
    uint32_t v = 0;
    if (l.atnt) {  // stale base register: the load reads the stale address, its value stays stale (the load is never re-executed with a good address)
      msd::taint[l.dest_preg] = msd::TV{};
      if (plain && ms_load_value(pnxt, l, defer, v) && v != f->rd_value) {
        msd::set_taint(l.dest_preg, true, v, 1);
        msd::st.ld_addr_stale++;
        if (msd::trace) std::fprintf(stderr, "MSD t=%lld stale-address load preg=%d rob=%d pc=%x va=%x pa=%x value=%x true=%x\n", (long long)sim_time, (int)l.dest_preg, (int)l.rob_idx, (unsigned)l.pc, (unsigned)l.vaddr, (unsigned)l.paddr, (unsigned)v, (unsigned)f->rd_value);
      } else if (plain) {
        msd::st.ld_addr_same++;
      }
    } else if (defer) {  // first write-back of a speculative load: the value of the load without the stores it passed
      msd::taint[l.dest_preg] = msd::TV{};
      if (plain && ms_load_value(pnxt, l, true, v)) {
        msd::st.spec_wb++;
        l.stale = v != f->rd_value;
        if (l.stale) {
          msd::st.stale++;
          msd::set_taint(l.dest_preg, true, v, 0);
          if (msd::trace) std::fprintf(stderr, "MSD t=%lld stale load preg=%d rob=%d pc=%x va=%x pa=%x stale=%x true=%x vio=%d\n", (long long)sim_time, (int)l.dest_preg, (int)l.rob_idx, (unsigned)l.pc, (unsigned)l.vaddr, (unsigned)l.paddr, (unsigned)v, (unsigned)f->rd_value, (int)l.vio);
        } else {
          msd::st.silent++;
        }
      }
    } else {
      if (second) {
        if (l.vio) (l.stale ? msd::st.stale_vio : msd::st.silent_vio)++;
        else if (l.stale) msd::st.stale_no_vio++;
        l.stale = false;
      }
      msd::taint[l.dest_preg] = msd::TV{};  // the register holds the true value from here on
      if ((msd::mode & 8) && plain && ms_load_value(pnxt, l, false, v)) {  // self check of the value logic
        msd::st.loads_checked++;
        if (v != f->rd_value) {
          if (msd::st.loads_check_bad++ < 8)
            std::fprintf(stderr, "[MS_DATA] load value mismatch t=%lld rob=%d pc=%x pa=%x got=%x want=%x second=%d vio=%d\n", (long long)sim_time, (int)l.rob_idx, (unsigned)l.pc, (unsigned)l.paddr, (unsigned)v, (unsigned)f->rd_value, (int)second, (int)l.vio);
        }
      }
    }
  }
  if (lp_ms_trace && (defer || second)) std::fprintf(stderr, "MS t=%lld wb ldq=%d rob=%d defer=%d second=%d vio=%d\n", (long long)sim_time, idx, (int)l.rob_idx, (int)defer, (int)second, (int)l.vio);
  if (defer) {  // keep the entry until the older store addresses are known (then the completion write-back, or a re-execution)
    if (l.vio) {
      l.second = 1;
      l.st = L_RDY;
    } else {
      l.st = L_SPD;
    }
    g_st.spec_loads++;
  } else {
    l.st = L_EMPTY;
  }
}

void RealLsu::comb_lsu2exe_p() {
  wires->lsu2exe = {};
  bool busy[LD] = {};
  for (int p = 0; p < LD; p++) {  // 1. L2 completions own their load port (a: the ones registered in L3)
    if (pw.wbd_v[p]) {
      busy[p] = true;
      lp_emit_load_wb(p, pw.wbd_ldq[p]);
    } else if ((lp_l2 & 1) && pcur.l3[p].v && pnxt.ldq[pcur.l3[p].ldq].st == L_WB3) {
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
      if ((lp_l2 & 64) && sc_n >= 1) continue;  // S: one SC completion per cycle
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
        pw.ev_sc |= 1ULL << j;
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
        if (lp_mdp_mode == 0 && !l.vio) mdp_clean(l.pc);
        l.second = 1;
        l.st = L_WBP;
        if (lp_ms_trace) std::fprintf(stderr, "MS t=%lld rel ldq=%d rob=%d\n", (long long)sim_time, i, (int)l.rob_idx);
        g_st.spec_released++;
      }
    }
  }

  // 3. loads waiting for a port (forwarded after a data wait, MMIO data): lowest LDQ index first
  for (int i = 0; i < LDQ_SIZE; i++) {
    if (pnxt.ldq[i].st != L_WBP) continue;
    if ((lp_cut & 8) && pcur.ldq[i].st != L_WBP && !((lp_cut & 64) && pw.mmio_ld == i)) continue;
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
void RealLsu::comb_exe2lsu_p() {
  const ExeLsuIO &x = wires->exe2lsu;
  for (int i = 0; i < LSU_SDU_COUNT; i++) {
    if (!x.sdu_req[i].valid) continue;
    const MicroOp &inst = x.sdu_req[i].uop;
    if (inst.stq_idx >= STQ_SIZE) continue;
    PStq &e = pnxt.stq[inst.stq_idx];
    if (e.rob_flag != inst.rob_flag || e.rob_idx != inst.rob_idx) continue;
    e.data = inst.result;
    e.data_v = 1;
    pw.ev_data |= 1ULL << inst.stq_idx;
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
    if (msd::on && (msd::mode & 2)) {  // a store whose base register was stale: its data goes to the stale address (never to the true one)
      const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx);
      e.m_atnt = f && f->is_store && f->vaddr != (uint32_t)inst.result;
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
    pw.ev_sta |= 1ULL << inst.stq_idx;  // the partial address is known from now on
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
        g_st.order_violation++;
        if (lp_ms_trace) std::fprintf(stderr, "MS t=%lld vio ldq=%d rob=%d stq=%u st=%d\n", (long long)sim_time, i, (int)l.rob_idx, j, (int)l.st);
        g_msp.on_detect(l.rob_idx);
        if (lp_mdp_mode == 1) ss_train(l.pc, pnxt.stq[j].pc);
        else if (lp_mdp_hw) { vio_new++; vio_set |= 1ULL << i; }
        else mdp_train(l.pc);
        l.vio = 1;
        if (l.st == L_SPD) {  // already written back: re-execute with the store visible
          l.second = 1;
          l.st = L_RDY;
        }
      }
    }
  }
  if (vio_new) {
    if (vio_new > 1) g_st.vio_multi++;
    if (lp_mdp_hw2) g_trn |= vio_set;
    else if (lp_mdp_hw && lp_mdp_bits)
      for (int i = 0, n = 0; i < LDQ_SIZE && n < (int)lp_mdp_ntr; i++)
        if (vio_set >> i & 1) { g_mdp_pend.push_back(mdp_idx(pnxt.ldq[i].pc)); n++; }
  }
  int nl = 0;
  for (int i = 0; i < LSU_AGU_COUNT; i++) {
    if (!(x.agu_req[i].valid && x.agu_req[i].uop.op == UOP_LOAD)) continue;
    const MicroOp &inst = x.agu_req[i].uop;
    if (inst.ldq_idx >= LDQ_SIZE) continue;
    PLdq &e = pnxt.ldq[inst.ldq_idx];
    if (e.rob_flag != inst.rob_flag || e.rob_idx != inst.rob_idx) continue;
    e.vaddr = inst.result;
    if (msd::on && (msd::mode & 2)) {  // a load whose base register was stale reads the stale address
      const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx);
      e.atnt = f && f->is_load && f->vaddr != (uint32_t)inst.result;
    }
    e.dest_preg = inst.dest_preg;
    e.pa_v = 0;
    e.paddr = 0;
    e.func3 = inst->func3;
    e.st = L_PIPE;
    e.snap_idx = inst.stq_idx;
    e.snap_flag = inst.stq_flag;
    e.lrsc = is_amo_lr_uop(inst);
    LP_PORT_CHECK(nl + 1, LD, "load_agu");
    pw.l1[nl].v = true;
    pw.l1[nl].need_tlb = true;
    pw.l1[nl].ldq = inst.ldq_idx;
    nl++;
    g_st.l1_new++;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// L1: DTLB request (new loads, replay registers fill the remaining load ports), STA translation
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::comb_tlb_out_p() {
  wires->lsu2mmu_io = {};
  wires->lsu2mmu_io.csr_status = wires->csr_status;

  int n = 0;
  while (n < LD && pw.l1[n].v) n++;
  for (int r = 0; r < lp_rp_ports; r++) {
    if (n >= LD && lp_bounce > 0 && pcur.rp[r].v && pcur.rp[r].age >= lp_bounce) {
      // aging bounce: the last new load goes to replay (its AGU request is consumed, it re-enters through RP)
      PL1 &b = pw.l1[LD - 1];
      if (b.v && !b.replay) {
        pnxt.ldq[b.ldq].st = L_RDY;
        b = PL1{};
        n = LD - 1;
        g_st.bounce++;
      }
    }
    if (n >= LD) {
      if (pcur.rp[r].v && pnxt.rp[r].age < 7) pnxt.rp[r].age++;
      continue;
    }
    int src = -1;
    if (pcur.rp[r].v) {
      src = pcur.rp[r].ldq;
      pnxt.rp[r].v = 0;
      pnxt.rp[r].age = 0;
    }
    if (src < 0) continue;
    PLdq &e = pnxt.ldq[src];
    if (e.st != L_PIPE) continue;
    pw.l1[n].v = true;
    pw.l1[n].replay = true;
    pw.l1[n].need_tlb = !e.pa_v;
    pw.l1[n].ldq = src;
    n++;
    g_st.l1_replay++;
  }
  for (int k = 0; k < LD; k++) {
    if (!pw.l1[k].v || !pw.l1[k].need_tlb) continue;
    auto &rq = wires->lsu2mmu_io.ldq_req[k];
    rq.valid = true;
    rq.vaddr = pnxt.ldq[pw.l1[k].ldq].vaddr;
    rq.entry_idx = pw.l1[k].ldq;
    rq.wait_idx = pw.l1[k].ldq;
    g_st.l1_tlb++;
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
      g_st.sta_replay++;
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

void RealLsu::comb_tlb_in_ldq_p() {
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

void RealLsu::comb_tlb_in_stq_p() {
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
    pw.ev_sta |= 1ULL << s1.stq;
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
      g_st.order_violation++;
      l.st = L_RDY;  // replay (a flush from the load would be needed if its result had been consumed)
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------
// L1: partial CAM over the STQ (parallel with the DCache read), MMIO issue
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::comb_stlf_p() {
  int live = 0;
  if (lp_memspec && lp_ms_slots)
    for (const PLdq &x : pnxt.ldq) live += x.st != L_EMPTY && x.spec && !x.second;
  for (int k = 0; k < LD; k++) {
    PL1 &l = pw.l1[k];
    if (!l.v || !l.hit || l.mmio) continue;
    const PLdq &e = pnxt.ldq[l.ldq];
    PLdq tmp = e;
    tmp.paddr = l.paddr;
    uint64_t ov = 0;
    for (int q = 0; q < STA; q++)
      if (pw.s1[q].v && !pw.s1[q].replay) ov |= 1ULL << pw.s1[q].stq;
    const bool slot_ok = !lp_ms_slots || e.spec || live < (int)lp_ms_slots;
    const CamRes c = stq_cam(pcur, pnxt, ov, tmp, /*full=*/!l.need_tlb, /*spec_ok=*/lp_memspec && slot_ok && !e.second && !e.lrsc && !(lp_mdp_mode == 0 && e.wait));
    if (lp_ms_slots) live += (int)c.unk - (int)e.spec;
    pnxt.ldq[l.ldq].spec = c.unk;
    pnxt.ldq[l.ldq].skip = c.skip;
    l.cand_v = c.v;
    l.cand = c.idx;
  }

  // MMIO loads: only the oldest instruction, with no older store left in the STQ, one on the bus at a time
  for (int i = 0; i < LDQ_SIZE; i++) {
    PLdq &e = pnxt.ldq[i];
    if (e.st != L_MMW || nxt.uncached_unit.valid) continue;
    if ((lp_l2 & 32) && pcur.ldq[i].st != L_MMW) continue;  // M: only entries that were already MMW at the start of the cycle
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
void RealLsu::comb_dis2lsu_p() {
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
        std::fill(g_mdp.begin(), g_mdp.end(), 0);
        std::fill(g_ssit.begin(), g_ssit.end(), 0);
        g_mdp_next_clear = (long long)sim_time + lp_mdp_clr;
      }
      if (lp_mdp_mode == 1) e.sset = ss_get(e.pc);
      else e.wait = mdp_wait(e.pc);
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
    if (lp_memspec && lp_mdp_mode == 1) e.sset = ss_get(e.pc);
    e.func3 = d.func3[i];
    pnxt.stq_count++;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// flush / mispredict / clear_mask
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::comb_flush_p() {
  if (wires->rob_bcast.flush) {
    if (msd::on) msd::clear_all();  // every stale value dies with the flush
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
  if (lp_ms_trace && (dec.mispred || wires->rob_bcast.flush)) std::fprintf(stderr, "MS t=%lld %s\n", (long long)sim_time, wires->rob_bcast.flush ? "flush" : "mispred");
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
void RealLsu::comb_lsu2dcache_ldq_p() {
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
    sl.rpl = l.replay;
    // the DCache is read in parallel with the partial CAM; the read enable is qualified by "no conflicting older store"
    // (12-bit compare, ready with the DTLB result). lp_gate=0: ungated read (wasted bank slots, measured as a variant).
    if (l.hit && !l.mmio && !l.cand_v && lp_rblk && wires->in.dcache2lsu.load_retry_busy[k]) {
      sl.dcblk = 1;  // the port belongs to the DCache hidden retry this cycle
      g_st.rblk++;
    } else if (l.hit && !l.mmio && (!l.cand_v || !lp_gate)) {
      e.epoch = (e.epoch + 1) & ((1u << EPOCH_W) - 1);
      ports[k].valid = true;
      ports[k].addr = l.paddr;
      ports[k].req_id = (uint32_t(e.epoch) << LDQ_IDX_WIDTH) | l.ldq;
      ports[k].replay = e.rtype != (uint8_t)ReplayType::HIT;
      g_st.dc_launch++;
    }
    sl.epoch = e.epoch;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// committed-store drain (RTL-aligned form, see rtl docs/analysis/lsu-pipe.md "model changes" M4):
//   1. every eligible store of the committed window counts its replay countdown down (all of them, every cycle);
//   2. the head store, when MMIO, takes the uncached unit;
//   3. the first lp_dcand (4) oldest ready non-MMIO stores are the candidates; a candidate is dropped for this cycle when an older
//      committed, not done store overlaps its bytes (a dropped candidate does not take a port); the m-th remaining candidate
//      (m < lp_drain = 3) uses the m-th store port without a pending DCache hidden retry. lp_dcand = 0 is the first-version
//      drain: the first lp_drain ready candidates, candidate m on port m, a dropped candidate keeps its port idle.
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::comb_lsu2dcache_stq_p() {
  auto &ports = wires->out.lsu2dcache.req_ports.store_ports;
  for (int i = 0; i < LSU_STA_COUNT; i++) ports[i].valid = ports[i].replay = false;

  const uint32_t n = std::min<uint32_t>((lp_cut & 32) ? pcur.stq_ccount : pnxt.stq_ccount, pnxt.stq_count);
  bool rdy[STQ_SIZE] = {};  // by rank
  for (uint32_t rank = 0; rank < n; rank++) {
    const uint32_t j = (pnxt.stq_head + rank) % STQ_SIZE;
    PStq &s = pnxt.stq[j];
    if (!(s.cmt && !s.done && !s.mmw)) continue;
    if (pcur.stq[j].iss) continue;  // in flight, or answered this cycle: re-send earliest next cycle
    if (s.suppress) { std::fprintf(stderr, "[LSU_PIPE] committed suppress store not done (unreachable)\n"); std::abort(); }
    const uint32_t rw_seen = ((lp_cut & 16) && (pw.st_fullwake >> j & 1)) ? pw.st_rwpre[j] : s.rwait;
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
    if (!(lp_rblk && wires->in.dcache2lsu.store_retry_busy[t])) free_port[nfree++] = t;
  int m = 0, seen = 0;
  for (uint32_t rank = 0; rank < n && m < lp_drain; rank++) {
    const uint32_t j = (pnxt.stq_head + rank) % STQ_SIZE;
    PStq &s = pnxt.stq[j];
    if (!rdy[rank] || s.mmio) continue;
    if (lp_dcand > 0 && seen++ >= lp_dcand) break;  // only the first lp_dcand ready candidates have a CAM
    int cm = 0;
    if (lp_dcand == 0) {  // first-version drain: the candidate index is the port index, a blocked candidate keeps its port idle
      cm = m++;
      if (cm >= nfree) continue;
    }
    const uint8_t strb = get_store_strb(s.paddr, s.func3);
    bool blocked = false;
    for (uint32_t q = 0; q < rank && !blocked; q++) {  // lp_dcand x 64 CAM in hardware
      const PStq &o = pnxt.stq[(pnxt.stq_head + q) % STQ_SIZE];
      if (o.done || !o.cmt) continue;
      if (CheckAddr(o.paddr, get_store_strb(o.paddr, o.func3), s.paddr, strb)) blocked = true;
    }
    if (blocked) {
      g_st.st_conflict++;
      continue;
    }
    if (lp_dcand > 0) {  // the m-th unblocked candidate uses the m-th free port
      cm = m++;
      if (cm >= nfree) continue;
    }
    auto &p = ports[free_port[cm]];
    p.valid = true;
    p.addr = s.paddr;
    p.strb = strb;
    p.req_id = j;
    p.replay = s.rtype != (uint8_t)ReplayType::HIT;
    s.iss = 1;
    g_st.st_drain++;
  }
  LP_PORT_CHECK(m, lp_drain + 1, "store_drain");
}

// ---------------------------------------------------------------------------------------------------------------
// STQ retire, wake-up pass (events of this cycle -> LDQ entries), replay-register fill
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::comb_check_p() {
  int ret = 0;
  while (ret < RETIRE_PORTS && pnxt.stq_ccount > 0 && pnxt.stq_count > 0 && pnxt.stq[pnxt.stq_head].done &&
         (!(lp_cut & 2) || ((uint32_t)ret < pcur.stq_ccount && (uint32_t)ret < pcur.stq_count && pcur.stq[pnxt.stq_head].done))) {
    pw.ev_ret |= 1ULL << pnxt.stq_head;
    pnxt.stq_count--;
    pnxt.stq_ccount--;
    pnxt.stq_head = (pnxt.stq_head + 1) % STQ_SIZE;
    if (pnxt.stq_head == 0) pnxt.stq_head_flag = !pnxt.stq_head_flag;
    ret++;
  }
  LP_PORT_CHECK(ret, RETIRE_PORTS, "stq_retire");

  if (const char *d = std::getenv("LSU_PIPE_DUMP")) {  // debug aid: dump the queues at one cycle (env = cycle number)
    if ((long long)sim_time == std::atoll(d)) {
      for (int i = 0; i < LDQ_SIZE; i++) {
        const PLdq &l = pnxt.ldq[i];
        if (l.st == L_EMPTY) continue;
        std::fprintf(stderr, "LDQ[%d] st=%d cause=%d blk=%d wait=%d rob=%d/%d va=0x%x pa=%d/0x%x mmio=%d pf=%d snap=%d/%d epoch=%d\n", i, (int)l.st,
                     (int)l.rcause, (int)l.blk_idx, (int)l.rwait, (int)l.rob_idx, (int)l.rob_flag, (unsigned)l.vaddr, (int)l.pa_v,
                     (unsigned)l.paddr, (int)l.mmio, (int)l.pf, (int)l.snap_idx, (int)l.snap_flag, (int)l.epoch);
      }
      std::fprintf(stderr, "STQ head=%d/%d cnt=%d ccnt=%d commit=%d unc=%d\n", (int)pnxt.stq_head, (int)pnxt.stq_head_flag, (int)pnxt.stq_count,
                   (int)pnxt.stq_ccount, (int)pnxt.stq_commit, (int)nxt.uncached_unit.valid);
      for (uint32_t k = 0; k < pnxt.stq_count; k++) {
        const PStq &t = pnxt.stq[(pnxt.stq_head + k) % STQ_SIZE];
        std::fprintf(stderr, "STQ[%d] rob=%d va=%d pa=%d d=%d pf=%d trp=%d wbp=%d wbd=%d cmt=%d iss=%d mmw=%d done=%d sup=%d mmio=%d lrsc=%d paddr=0x%x rw=%d\n",
                     (int)((pnxt.stq_head + k) % STQ_SIZE), (int)t.rob_idx, (int)t.va_v, (int)t.pa_v, (int)t.data_v, (int)t.pf, (int)t.trp, (int)t.wbp,
                     (int)t.wbd, (int)t.cmt, (int)t.iss, (int)t.mmw, (int)t.done, (int)t.suppress, (int)t.mmio, (int)t.lrsc, (unsigned)t.paddr, (int)t.rwait);
      }
      for (int r = 0; r < RP_MAX; r++) std::fprintf(stderr, "RP[%d] v=%d ldq=%d age=%d\n", r, (int)pnxt.rp[r].v, (int)pnxt.rp[r].ldq, (int)pnxt.rp[r].age);
      std::fprintf(stderr, "srp v=%d stq=%d; rob head valid=%d idx=%d inc_valid=%d inc_idx=%d\n", (int)pnxt.srp.v, (int)pnxt.srp.ldq,
                   (int)wires->rob_bcast.head_valid, (int)wires->rob_bcast.head_rob_idx, (int)wires->rob_bcast.head_incomplete_valid,
                   (int)wires->rob_bcast.head_incomplete_rob_idx);
    }
  }
  // DCache response watchdog: a request whose response never arrives (lost bank-retry latch in the DCache) is replayed
  for (int i = 0; i < LDQ_SIZE; i++) {
    PLdq &e = pnxt.ldq[i];
    if (e.st != L_DCW) { e.wd = 0; continue; }
    if (++e.wd == 63) { static const bool nowd = std::getenv("LSU_PIPE_NOWD"); if (nowd) { std::fprintf(stderr, "[WD] load ldq=%d stuck in DCW at cycle %lld\n", i, (long long)sim_time); } else { e.wd = 0; e.st = L_RDY; g_st.wd_ld++; } }
  }
  for (int j = 0; j < STQ_SIZE; j++) {
    PStq &t = pnxt.stq[j];
    if (!t.iss) { t.wd = 0; continue; }
    if (++t.wd == 63) { static const bool nowd = std::getenv("LSU_PIPE_NOWD"); if (nowd) { std::fprintf(stderr, "[WD] store stq=%d stuck issued at cycle %lld\n", j, (long long)sim_time); } else { t.wd = 0; t.iss = 0; t.rwait = 0; g_st.wd_st++; } }
  }
  if (static const bool ev = std::getenv("LSU_PIPE_EVT") != nullptr; ev) {  // debug aid: per-cycle LSU boundary events (matches the RTL tb print)
    std::fprintf(stderr, "E t=%lld wb:", (long long)sim_time);
    for (int p = 0; p < LD; p++)
      if (wires->lsu2exe.wb_req[p].valid) std::fprintf(stderr, " %d=r%d/p%d/e%d/b%llx", p, (int)wires->lsu2exe.wb_req[p].uop.rob_idx, (int)wires->lsu2exe.wb_req[p].uop.dest_preg,
                                                       (int)wires->lsu2exe.wb_req[p].uop.dest_en, (unsigned long long)wires->lsu2exe.wb_req[p].uop.br_mask),
        std::fprintf(stderr, "/v%x/d%x/f%d%d", (unsigned)wires->lsu2exe.wb_req[p].uop.result, (unsigned)wires->lsu2exe.wb_req[p].uop.diag_val,
                     (int)wires->lsu2exe.wb_req[p].uop.page_fault_load, (int)wires->lsu2exe.wb_req[p].uop.page_fault_store);
    std::fprintf(stderr, " sta:");
    for (int p = 0; p < STA; p++)
      if (wires->lsu2exe.sta_wb_req[p].valid) std::fprintf(stderr, " %d=r%d/b%llx", p, (int)wires->lsu2exe.sta_wb_req[p].uop.rob_idx, (unsigned long long)wires->lsu2exe.sta_wb_req[p].uop.br_mask),
        std::fprintf(stderr, "/v%x/d%x/f%d", (unsigned)wires->lsu2exe.sta_wb_req[p].uop.result, (unsigned)wires->lsu2exe.sta_wb_req[p].uop.diag_val,
                     (int)wires->lsu2exe.sta_wb_req[p].uop.page_fault_store);
    std::fprintf(stderr, " dl:");
    for (int p = 0; p < LD; p++) {
      const auto &q = wires->out.lsu2dcache.req_ports.load_ports[p];
      if (q.valid) std::fprintf(stderr, " %d=%x/%x", p, (unsigned)q.addr, (unsigned)(q.req_id & 0x7fffffffu));
    }
    std::fprintf(stderr, " ds:");
    for (int p = 0; p < STA; p++) {
      const auto &q = wires->out.lsu2dcache.req_ports.store_ports[p];
      if (q.valid) std::fprintf(stderr, " %d=%x/%d", p, (unsigned)q.addr, (int)q.req_id);
    }
    std::fprintf(stderr, " mmu:");
    for (int p = 0; p < LD; p++)
      if (wires->lsu2mmu_io.ldq_req[p].valid) std::fprintf(stderr, " L%d=%d", p, (int)wires->lsu2mmu_io.ldq_req[p].entry_idx);
    for (int p = 0; p < STA; p++)
      if (wires->lsu2mmu_io.stq_req[p].valid) std::fprintf(stderr, " S%d=%d", p, (int)wires->lsu2mmu_io.stq_req[p].entry_idx);
    std::fprintf(stderr, " d: sf=%d lf=%d tl=%d/%d cp=%d", (int)wires->lsu2dis.stq_free, (int)wires->lsu2dis.ldq_free, (int)wires->lsu2dis.stq_tail,
                 (int)wires->lsu2dis.stq_tail_flag, (int)wires->lsu2rob.committed_store_pending);
    {
      unsigned dsa = 0, dla = 0, cm = 0;
      for (int i = 0; i < MAX_STQ_DISPATCH_WIDTH; i++) dsa |= (unsigned)wires->dis2lsu.alloc_req[i] << i;
      for (int i = 0; i < MAX_LDQ_DISPATCH_WIDTH; i++) dla |= (unsigned)wires->dis2lsu.ldq_alloc_req[i] << i;
      for (int i = 0; i < COMMIT_WIDTH; i++) cm |= (unsigned)wires->rob_commit.commit_entry[i].valid << i;
      std::fprintf(stderr, " b: mp=%d fl=%d cl=%llx dsa=%x dla=%x cm=%x", (int)wires->dec_bcast.mispred, (int)wires->rob_bcast.flush,
                   (unsigned long long)wires->dec_bcast.clear_mask, dsa, dla, cm);
    }
    std::fprintf(stderr, " in:");
    for (int p = 0; p < LD; p++)
      if (wires->mmu2lsu_io.ldq_resp[p].valid) std::fprintf(stderr, " ml%d=%d/%x", p, (int)wires->mmu2lsu_io.ldq_resp[p].result, (unsigned)wires->mmu2lsu_io.ldq_resp[p].paddr);
    for (int p = 0; p < STA; p++)
      if (wires->mmu2lsu_io.stq_resp[p].valid) std::fprintf(stderr, " ms%d=%d/%x", p, (int)wires->mmu2lsu_io.stq_resp[p].result, (unsigned)wires->mmu2lsu_io.stq_resp[p].paddr);
    for (int p = 0; p < LD; p++) {
      const auto &r = wires->in.dcache2lsu.resp_ports.load_resps[p];
      if (r.valid) std::fprintf(stderr, " dr%d=%x/%d", p, (unsigned)(r.req_id & 0x7fffffffu), (int)r.replay);
    }
    for (int p = 0; p < STA; p++) {
      const auto &r = wires->in.dcache2lsu.resp_ports.store_resps[p];
      if (r.valid) std::fprintf(stderr, " sr%d=%d/%d", p, (int)r.req_id, (int)r.replay);
    }
    if (wires->in.dcache2lsu.mshr_fill) std::fprintf(stderr, " fill=%x", (unsigned)(wires->in.dcache2lsu.mshr_fill_addr >> 6));
    for (int p = 0; p < LD; p++) if (wires->in.dcache2lsu.load_retry_busy[p]) std::fprintf(stderr, " rb%d", p);
    for (int p = 0; p < STA; p++) if (wires->in.dcache2lsu.store_retry_busy[p]) std::fprintf(stderr, " sb%d", p);
    std::fprintf(stderr, " ai:");
    for (int p = 0; p < MAX_LDQ_DISPATCH_WIDTH; p++)
      if (wires->lsu2dis.ldq_alloc_valid[p]) std::fprintf(stderr, " %d", (int)wires->lsu2dis.ldq_alloc_idx[p]);
    std::fprintf(stderr, " x:");
    for (int p = 0; p < LSU_SDU_COUNT; p++)
      if (wires->exe2lsu.sdu_req[p].valid) std::fprintf(stderr, " D%d=%d", p, (int)wires->exe2lsu.sdu_req[p].uop.stq_idx);
    for (int p = 0; p < LSU_AGU_COUNT; p++)
      if (wires->exe2lsu.agu_req[p].valid && wires->exe2lsu.agu_req[p].uop.op == UOP_STA)
        std::fprintf(stderr, " A%d=%d", p - LD, (int)wires->exe2lsu.agu_req[p].uop.stq_idx);
    for (int p = 0; p < LSU_AGU_COUNT; p++)
      if (wires->exe2lsu.agu_req[p].valid && wires->exe2lsu.agu_req[p].uop.op == UOP_LOAD)
        std::fprintf(stderr, " G%d=%d", p, (int)wires->exe2lsu.agu_req[p].uop.ldq_idx);
    if (const char *si = std::getenv("LSU_PIPE_STQ")) {
      const PStq &q = pcur.stq[std::atoi(si)];  // registers at the start of the cycle (RTL prints the registers)
      std::fprintf(stderr, " Q[%s]: va=%d pa=%d dv=%d pf=%d wbp=%d wbd=%d cmt=%d iss=%d done=%d", si, (int)q.va_v, (int)q.pa_v, (int)q.data_v, (int)q.pf,
                   (int)q.wbp, (int)q.wbd, (int)q.cmt, (int)q.iss, (int)q.done);
    }
    std::fprintf(stderr, "\n");
  }
  // wake-up pass
  uint32_t full_wakes = 0;
  for (int i = 0; i < LDQ_SIZE; i++) {
    PLdq &e = pnxt.ldq[i];
    if (e.st != L_RPW) continue;
    const uint64_t b = 1ULL << e.blk_idx;
    switch (e.rcause) {
    case RC_TLB:
      if (e.rwait > 0) e.rwait--;
      if (e.rwait == 0) e.st = L_RDY;
      break;
    case RC_DC:
      if (pw.fill && e.rtype == (uint8_t)ReplayType::MSHR_HIT && cache_line_match(e.paddr, pw.fill_addr)) {
        e.rwait = 0;
      } else if (pw.fill && e.rtype == (uint8_t)ReplayType::MSHR_FULL && full_wakes < (uint32_t)LD &&
                 (!(lp_l2 & 16) || pcur.ldq[i].st == L_RPW)) {
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
        const Conf c = stlf_confirm(e, s, ((lp_cut & 1) ? stq_rank(e.blk_idx, pcur.stq_head) < pcur.stq_count : stq_rank(e.blk_idx, pnxt.stq_head) < pnxt.stq_count) && !(pw.ev_ret & b));
        if (s.va_v && range_rel(e.vaddr, ld_width(e.func3), s.vaddr, ld_width(s.func3), 12) == DISJOINT)
          g_st.unk_wait_disjoint++;  // a load that had passed the unknown address would have been right
        else
          g_st.unk_wait_overlap++;   // ... would have violated the memory order
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
      if ((lp_cut & 4) && pcur.ldq[i].st != L_RDY) continue;
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
