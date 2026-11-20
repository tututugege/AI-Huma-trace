#pragma once
// L1D data prefetcher, frozen design "h1" (EXP-009). This is the specification for the RTL (rtl/rtl/memsys/dcache/dcache_pf.sv).
//
// What it is: a single-entry global stride detector (one stream, no PC) trained on L1D load misses and on the first use of
// a prefetched line, a one-line next-line engine whose lines only go to the LLC (they are not filled into the L1D), a
// request FIFO, and nothing else. No accuracy throttle (it was worth 0.1 points with the LLC engine), no PC from the LSU.
//
// Cycle behaviour (all registers update at the end of the cycle, comb reads only registers and this cycle's inputs):
//   - stage 2 of the L1D produces up to LSU_LDU_COUNT access events (miss / first use of a prefetched line);
//   - seq(): (1) the FIFO head is popped if the L1D took it for a tag probe, (2) the stride generator pushes one candidate,
//     (3) otherwise the LLC engine pushes one candidate, (4) the events registered last cycle train the detector,
//     (5) up to PF_TRAIN_PORTS new events are selected into the input registers (extra events are dropped).
// On by default (EXP-009); the environment variable PF_ENABLE=0 turns it off.
// Circuit rules (pf::Table / pf::Fifo): fixed port counts per cycle, widths truncate on write; violations abort.
#include "Prefetch.h"
#include "Bop.h"
#include <algorithm>
#include <deque>
#include <unordered_map>
extern long long sim_time;

constexpr int PF_QUEUE = 8;         // request FIFO depth
constexpr int PF_TRAIN_PORTS = 2;   // training events taken per cycle
constexpr int PF_DEGREE = 8;        // candidates of one stride burst
constexpr int PF_DIST = 2;          // the burst starts DIST strides ahead of the trigger line
constexpr int PF_CONF_THR = 2;      // confidence needed to start a burst
constexpr int PF_STRIDE_BITS = 8;   // signed stride in lines
constexpr int PF_LLC_LINES = 1;     // next-line engine: lines after every load miss that only go to the LLC
constexpr int PF_MSHR_RESERVE = 0;  // a probe allocates an MSHR whenever one is free
constexpr int PF_LINE_BITS = 32 - 6;  // physical line number (64 B lines, 32-bit physical address)

#include <map>
inline void extern_reset_xq();
// EXP-048: the L1D prefetcher's translation of a virtual address (a side-effect-free DTLB lookup, set by the LSU); false when
// the DTLB does not hold the page
inline bool (*g_pf_translate)(uint32_t &pa, uint64_t va) = nullptr;
// EXP-048 PF_BERTI_WALK=1: pages whose translation a cross-page candidate missed; the LSU sends one to the DTLB on a free
// load port while no walk is in flight (a miss there starts the walk: the translation is prefetched)
struct PfWalkQ {
  uint64_t va[4] = {};
  int n = 0;
  uint64_t pushed = 0, sent = 0, hit = 0, walk = 0, fault = 0;
  void push(uint64_t v) {
    const uint64_t page = v >> 12;
    for (int i = 0; i < n; i++) if ((va[i] >> 12) == page) return;
    if (n == 4) { for (int i = 1; i < 4; i++) va[i - 1] = va[i]; n--; }
    va[n++] = v;
    pushed++;
  }
  uint64_t pop() { const uint64_t v = va[0]; for (int i = 1; i < n; i++) va[i - 1] = va[i]; n--; return v; }
};
inline PfWalkQ g_pf_walk;
// EXP-048 PF_BERTI_XPORT=1: cross-page candidates are translated on real DTLB ports. The prefetcher queues virtual lines
// (g_pf_xq); the LSU sends one per load port nobody uses that cycle; a hit returns the physical line (g_pf_xans, taken by the
// prefetcher the next cycle), a miss starts the walk when the walker is idle (the candidate is dropped), a fault is dropped.
struct PfXReq { uint64_t va = 0; bool llc = false; };
struct PfXQ {
  static constexpr int N = 8;
  PfXReq q[N];
  int n = 0;
  uint64_t pushed = 0, dropped = 0, sent = 0, hit = 0, miss = 0, fault = 0;
  void push(const PfXReq &r) {
    for (int i = 0; i < n; i++) if ((q[i].va >> 6) == (r.va >> 6)) return;
    if (n == N) { for (int i = 1; i < N; i++) q[i - 1] = q[i]; n--; dropped++; }
    q[n++] = r;
    pushed++;
  }
  PfXReq pop() { const PfXReq r = q[0]; for (int i = 1; i < n; i++) q[i - 1] = q[i]; n--; return r; }
};
inline PfXQ g_pf_xq;
struct PfXAns { uint32_t line = 0; bool llc = false; };
inline PfXAns g_pf_xans[8];
inline int g_pf_xans_n = 0;
inline void extern_reset_xq() { g_pf_xq.pushed = g_pf_xq.dropped = g_pf_xq.sent = g_pf_xq.hit = g_pf_xq.miss = g_pf_xq.fault = 0; g_pf_walk.pushed = g_pf_walk.sent = g_pf_walk.hit = g_pf_walk.walk = g_pf_walk.fault = 0; }
struct PfStats {  // model-only counters, not hardware
  uint64_t issued = 0, useful = 0, late = 0, demand_miss = 0, demand_hit = 0;   // read by the PMU
  // EXP-048 diagnostics: what became of each candidate / tag probe (model-only)
  uint64_t q_push = 0, q_dup = 0, q_full = 0;                          // request FIFO: pushed / already queued / FIFO full
  uint64_t pr_alloc = 0, pr_present = 0, pr_inflight = 0, pr_nomshr = 0;   // probe: MSHR allocated / line in cache or WB /
                                                                        // in an MSHR / no MSHR left (dropped)
  std::map<uint32_t, std::pair<uint64_t, uint64_t>> pc_miss;   // PF_STATS: load PC -> {L1D demand misses, first uses of prefetched lines}
  uint64_t x_ok = 0, x_miss = 0, x_port = 0;   // virtual-mode cross-page candidates: translated / DTLB miss / no lookup left
  uint64_t off_miss[8] = {}, off_pfu[8] = {};   // PF_STATS: misses / first uses of prefetched lines by position in the 4 KiB page (8-line bins)
  void diag_reset() {
    q_push = q_dup = q_full = pr_alloc = pr_present = pr_inflight = pr_nomshr = 0; pc_miss.clear();
    x_ok = x_miss = x_port = 0;
    extern_reset_xq();
    for (auto &x : off_miss) x = 0;
    for (auto &x : off_pfu) x = 0;
  }
};
inline PfStats g_pf_stats;

// one load access seen at stage 2 of the L1D pipeline
struct PfAccess {
  bool valid = false;
  uint32_t paddr = 0;
  bool miss = false;     // L1D miss (new, or merged into an in-flight line)
  bool pf_hit = false;   // hit on a line that was brought in by a prefetch and not used before
  uint32_t pc = 0;       // EXP-048: the load's PC (Berti)
  uint64_t va = 0;       // EXP-048: the load's virtual address (Berti in virtual mode)
};
// one line written into the L1D (EXP-048 BOP: the recent-requests table is written at fill time)
struct PfFill {
  bool valid = false;
  uint32_t paddr = 0;
  bool is_prefetch = false;
};

#define PF_REQ_FIELDS(X) X(line, PF_LINE_BITS) X(llc, 1)
PF_ENTRY(PfReqE, PF_REQ_FIELDS)
#undef PF_REQ_FIELDS
#define PF_STRIDE_FIELDS(X) X(valid, 1) X(last, PF_LINE_BITS) X(stride, PF_STRIDE_BITS) X(conf, 2)
PF_ENTRY(PfStrideE, PF_STRIDE_FIELDS)
#undef PF_STRIDE_FIELDS

// Berti (Navarro-Torres et al., MICRO'22), ported from EXP-043 (trace-model exp/pf-l1l2 c1cef63 / 04fcbc0) for EXP-048 (PF_KIND=2), physical-address form: deltas in lines, targets kept in the page.
//   history: 8 sets x 16 ways (PC hash -> {pc tag, line, time}), FIFO per set; written on every training event;
//   delta table: 16 PCs x 16 deltas {delta, coverage}, a search counter per PC;
//   training: when a demand miss is filled (latency L = fill - request) or a prefetched line is used for the first time
//   (L = its prefetch latency), the history of the same PC is searched for accesses at least L earlier than the demand:
//   their deltas would have been timely; each found delta gains one coverage. Every BERTI_ROUND searches the coverage
//   decides the status: >= 65 % -> fill the L1D, >= 35 % -> LLC only, else none; coverage restarts.
//   trigger: every training event issues the PC's deltas with a status (L1 first, by coverage), at most PF_BERTI_DEG.
// Model-level structures (latency per prefetched line in a map, a 16-entry in-flight table); no circuit-rule checks.
// ---------------------------------------------------------------------------------------------------------------
class BertiPf {
 public:
  static constexpr int HSETS = 8, HWAYS = 64, DPCS = 16, DDELTAS = 16, ROUND = 16, INFL = 16;
  int hw = 16;   // history ways in use (PF_BERTI_HW, <= HWAYS)
  int hdedup = 0;   // PF_BERTI_HDEDUP: 0=off, 1=set-wide (pc,line) dedup (keep oldest), 2=last-entry dedup
  int deg = 12, hi = 65, lo = 35;
  bool alloc_found = false;
  bool late_fix = true;
  bool far_first = false;
  bool cross_page = false;   // EXP-048 PF_BERTI_VA=1: virtual lines, targets may leave the page (the prefetcher translates them)
  uint64_t st_latefix = 0, st_hrec = 0, st_hdedup = 0;
  struct Cand { uint32_t line; bool llc; };
  // diagnostics (PF_STATS=1)
  uint64_t st_trig = 0, st_trig_c = 0, st_cand = 0, st_xpage = 0, st_replaced = 0, st_search = 0, st_found = 0, st_search_none = 0,
           st_nopc = 0, st_pushed = 0;
  // structure diagnostics: history reach at each search (time from this PC's oldest entry in its set to the demand, vs the
  // latency it must cover), why a search found nothing, how the set is shared, delta-table pressure and slot use, warm-up
  uint64_t r_none = 0, r_lt_half = 0, r_lt_1 = 0, r_ge_1 = 0, nf_short = 0, nf_other = 0;
  double share_sum = 0, npc_sum = 0, same_sum = 0;
  uint64_t t_alloc = 0, t_evict_live = 0, u_n = 0; double u_pcs = 0, u_live = 0, u_slots = 0, u_stslots = 0;
  std::unordered_map<uint32_t, uint32_t> warm;   // PC -> searches so far (until its first delta with a status)
  uint64_t warm_n = 0; double warm_sum = 0; std::unordered_map<uint32_t, bool> seen_pc;
  ~BertiPf() {
    if (!std::getenv("PF_STATS") || !st_trig) return;
    const double sr = st_search ? double(st_search) : 1.0, un = u_n ? double(u_n) : 1.0;
    std::fprintf(stderr, "[BERTI-HIST] ways=%d hdedup=%d rec=%llu dedup=%llu searches=%llu reach(no own entry / <0.5 lat / 0.5-1 lat / >=1 lat)=%.1f%%/%.1f%%/%.1f%%/%.1f%% "
                 "found_nothing=%.1f%% (window shorter than the latency %.1f%%, other %.1f%%) own_entries=%.1f own_share=%.1f%% pcs_per_set=%.1f\n",
                 hw, hdedup, (unsigned long long)st_hrec, (unsigned long long)st_hdedup,
                 (unsigned long long)st_search, 100 * r_none / sr, 100 * r_lt_half / sr, 100 * r_lt_1 / sr, 100 * r_ge_1 / sr,
                 100.0 * st_search_none / sr, 100 * nf_short / sr, 100 * nf_other / sr, same_sum / sr, 100 * share_sum / sr, npc_sum / sr);
    std::fprintf(stderr, "[BERTI-TAB] distinct_pcs_searched=%zu table=%d allocs=%llu evictions_with_status=%llu avg_valid_pcs=%.1f "
                 "avg_pcs_with_status=%.1f delta_slots_used=%.1f%% delta_slots_with_status=%.1f%% warm_pcs=%llu avg_searches_to_first_status=%.1f\n",
                 seen_pc.size(), DPCS, (unsigned long long)t_alloc, (unsigned long long)t_evict_live, u_pcs / un, u_live / un,
                 100 * u_slots / un / (DPCS * DDELTAS), 100 * u_stslots / un / (DPCS * DDELTAS), (unsigned long long)warm_n,
                 warm_n ? warm_sum / warm_n : 0.0);
    std::fprintf(stderr, "[BERTI-LATE] demand fills timed from the prefetch issue: %llu\n", (unsigned long long)st_latefix);
    std::fprintf(stderr, "[BERTI] trig=%llu with_cand=%llu cand=%llu xpage=%llu replaced=%llu pushed=%llu search=%llu found_deltas=%llu "
                 "search_none=%llu no_pc_entry=%llu\n", (unsigned long long)st_trig, (unsigned long long)st_trig_c,
                 (unsigned long long)st_cand, (unsigned long long)st_xpage, (unsigned long long)st_replaced, (unsigned long long)st_pushed,
                 (unsigned long long)st_search, (unsigned long long)st_found, (unsigned long long)st_search_none, (unsigned long long)st_nopc);
    for (auto &p : tab) {
      if (!p.v) continue;
      std::fprintf(stderr, "[BERTI-PC] pc=%#x cnt=%d deltas:", p.pc, p.cnt);
      for (auto &x : p.dl) if (x.v) std::fprintf(stderr, " %d(c%d,s%d)", x.d, x.last_cov, x.st);
      std::fprintf(stderr, "\n");
    }
  }

  // line: the training line (virtual with PF_BERTI_VA=1, else physical); pline: its physical line (fills and prefetch issue
  // times are physical), defaults to line
  void miss(uint32_t pc, uint32_t line, long long now, uint32_t pline = UINT32_MAX) {
    if (pline == UINT32_MAX) pline = line;
    bool have = false;
    for (auto &f : infl) if (f.v && f.pline == pline) have = true;
    if (!have) { infl[infl_ptr] = {true, line, pc, now, pline}; infl_ptr = (infl_ptr + 1) % INFL; }
    record(pc, line, now);
  }
  void pf_hit(uint32_t pc, uint32_t line, long long now, uint32_t pline = UINT32_MAX) {
    if (pline == UINT32_MAX) pline = line;
    auto it = pf_lat.find(pline);
    if (it != pf_lat.end()) { search(pc, line, now, it->second); pf_lat.erase(it); }
    record(pc, line, now);
  }
  void pf_issued(uint32_t line, long long now) {
    if (pf_issue.size() > 8192) pf_issue.clear();
    pf_issue[line] = now;
  }
  void fill(uint32_t line, bool is_prefetch, long long now) {
    if (is_prefetch) {
      auto it = pf_issue.find(line);
      if (it != pf_issue.end()) {
        if (pf_lat.size() > 8192) pf_lat.clear();
        pf_lat[line] = now - it->second; pf_issue.erase(it);
      }
      return;
    }
    // EXP-048 late fix (PF_BERTI_LATEFIX, default 1): a demand that caught up with an in-flight prefetch fills as a demand
    // line; its latency is the prefetch's full latency (issue -> fill), not the rest after the demand arrived, which made
    // short deltas look timely
    long long lat_full = -1;
    if (late_fix) {
      auto it = pf_issue.find(line);
      if (it != pf_issue.end()) {
        if (now - it->second <= 4000) { lat_full = now - it->second; st_latefix++; }
        pf_issue.erase(it);
      }
    }
    for (auto &f : infl)
      if (f.v && f.pline == line) { search(f.pc, f.line, f.t, lat_full >= 0 ? lat_full : now - f.t); f.v = false; }
  }
  // the PC's deltas with a status, L1 ones first, most covered first
  int trigger(uint32_t pc, uint32_t line, Cand *out) {
    st_trig++;
    if ((st_trig & 63) == 0) {   // table use, sampled
      u_n++;
      for (auto &p : tab) {
        if (!p.v) continue;
        u_pcs++; bool live = false;
        for (auto &x : p.dl) { if (x.v) u_slots++; if (x.v && x.st) { u_stslots++; live = true; } }
        if (live) u_live++;
      }
    }
    const int e = find_pc(pc);
    if (e < 0) { st_nopc++; return 0; }
    struct D { int d; int cov; bool l1; } v[DDELTAS]; int n = 0;
    for (auto &x : tab[e].dl) if (x.v && x.st) v[n++] = {x.d, x.last_cov, x.st == 1};
    if (far_first)   // EXP-048 PF_BERTI_FAR=1: within a status the farthest delta first (the line least likely already fetched)
      std::sort(v, v + n, [](const D &a, const D &b) { return a.l1 != b.l1 ? a.l1 : std::abs(a.d) > std::abs(b.d); });
    else
      std::sort(v, v + n, [](const D &a, const D &b) { return a.l1 != b.l1 ? a.l1 : a.cov > b.cov; });
    int k = 0;
    for (int i = 0; i < n && k < deg; i++) {
      const uint32_t t = line + uint32_t(v[i].d);
      if (!cross_page && (t >> 6) != (line >> 6)) { st_xpage++; continue; }   // same 4 KiB page (virtual mode: translated later)
      out[k++] = {t, !v[i].l1};
    }
    if (k) { st_trig_c++; st_cand += k; }
    return k;
  }

 private:
  struct H { bool v = false; uint32_t pc = 0, line = 0; long long t = 0; };
  struct Dl { bool v = false; int d = 0, cov = 0, last_cov = 0, st = 0; };   // st: 0 none, 1 L1, 2 LLC
  struct P { bool v = false; uint32_t pc = 0; int cnt = 0; long long lru = 0; Dl dl[DDELTAS]; };
  struct F { bool v = false; uint32_t line = 0, pc = 0; long long t = 0; uint32_t pline = 0; };
  H hist[HSETS][HWAYS]; int hptr[HSETS] = {};
  P tab[DPCS]; long long tick_ = 0; bool search_short_ = false;
  F infl[INFL]; int infl_ptr = 0;
  std::unordered_map<uint32_t, long long> pf_issue, pf_lat;

  static int hset(uint32_t pc) { return int((pc >> 1) ^ (pc >> 4) ^ (pc >> 7)) & (HSETS - 1); }
  void record(uint32_t pc, uint32_t line, long long now) {
    const int s = hset(pc);
    st_hrec++;
    if (hdedup == 1) {
      for (int w = 0; w < hw; w++) {
        const H &h = hist[s][w];
        if (h.v && h.pc == pc && h.line == line) { st_hdedup++; return; }
      }
    } else if (hdedup == 2) {
      const H &h = hist[s][(hptr[s] + hw - 1) % hw];
      if (h.v && h.pc == pc && h.line == line) { st_hdedup++; return; }
    }
    hist[s][hptr[s]] = {true, pc, line, now};
    hptr[s] = (hptr[s] + 1) % hw;
  }
  int find_pc(uint32_t pc) const {
    for (int i = 0; i < DPCS; i++) if (tab[i].v && tab[i].pc == pc) return i;
    return -1;
  }
  void search(uint32_t pc, uint32_t line, long long t_demand, long long lat) {
    int found[HWAYS]; int nf = 0;
    const int s = hset(pc);
    {   // structure diagnostics
      int same = 0, valid = 0, npc = 0; long long oldest = -1; uint32_t pcs[HWAYS];
      bool any = false;
      for (int w = 0; w < hw; w++) {
        const H &h = hist[s][w];
        if (!h.v) continue;
        valid++;
        bool dupc = false;
        for (int i = 0; i < npc; i++) dupc |= pcs[i] == h.pc;
        if (!dupc) pcs[npc++] = h.pc;
        if (h.pc == pc) { same++; if (oldest < 0 || h.t < oldest) oldest = h.t; if (h.t + lat <= t_demand) any = true; }
      }
      same_sum += same; npc_sum += npc; share_sum += valid ? double(same) / valid : 0.0;
      const double reach = oldest < 0 ? -1.0 : double(t_demand - oldest);
      if (reach < 0) r_none++; else if (reach < 0.5 * double(lat)) r_lt_half++; else if (reach < double(lat)) r_lt_1++; else r_ge_1++;
      search_short_ = !any;
      seen_pc[pc] = true;
    }
    for (int w = 0; w < hw; w++) {
      const H &h = hist[s][w];
      if (!h.v || h.pc != pc || h.t + lat > t_demand) continue;
      const int d = int(line) - int(h.line);
      if (d == 0 || d >= 64 || d <= -64) continue;
      bool dup = false;
      for (int i = 0; i < nf; i++) dup |= found[i] == d;
      if (!dup) found[nf++] = d;
    }
    st_search++; st_found += uint64_t(nf); if (!nf) { st_search_none++; (search_short_ ? nf_short : nf_other)++; }
    { uint32_t &wc = warm[pc]; if (wc != 0xffffffffu) wc++; }
    int e = find_pc(pc);
    if (e < 0 && nf == 0 && alloc_found) return;   // PF_BERTI_ALLOC=1: a PC gets an entry only when a timely delta was found
    if (e < 0) {   // allocate the least recently trained PC
      e = 0;
      for (int i = 0; i < DPCS; i++) { if (!tab[i].v) { e = i; break; } if (tab[i].lru < tab[e].lru) e = i; }
      if (tab[e].v) { bool live = false; for (auto &x : tab[e].dl) live |= x.v && x.st; if (live) t_evict_live++; }
      t_alloc++;
      tab[e] = P{}; tab[e].v = true; tab[e].pc = pc;
    }
    P &p = tab[e];
    p.lru = ++tick_;
    p.cnt++;
    for (int i = 0; i < nf; i++) {
      Dl *slot = nullptr;
      for (auto &x : p.dl) if (x.v && x.d == found[i]) slot = &x;
      if (!slot) {   // a free slot, else the least covered one without a status
        for (auto &x : p.dl) if (!x.v) { slot = &x; break; }
        if (!slot) for (auto &x : p.dl) if (!x.st && (!slot || x.cov < slot->cov)) slot = &x;
        if (!slot) continue;
        *slot = Dl{}; slot->v = true; slot->d = found[i];
      }
      slot->cov++;
    }
    if (p.cnt >= ROUND) {
      for (auto &x : p.dl) {
        if (!x.v) continue;
        const int pct = x.cov * 100 / p.cnt;
        x.st = pct >= hi ? 1 : pct >= lo ? 2 : 0;
        if (x.st) { uint32_t &w = warm[pc]; if (w != 0xffffffffu) { warm_sum += w; warm_n++; w = 0xffffffffu; } }
        x.last_cov = x.cov;
        x.cov = 0;
      }
      p.cnt = 0;
    }
  }
};


// ---------------------------------------------------------------------------------------------------------------
// EXP-048 M3: Hardware-faithful Berti (PF_KIND=4) — exact 1-op/cycle RTL specification for dcache_pf_berti_*.sv:
//   * 20-bit wrapping cycle counter, 16-entry pf_issue CAM, 64-entry pf_lat CAM, 16-entry infl CAM
//   * 8x16 history table with set-wide (pc, line) dedup, 1 write + 1 search per cycle
//   * 16x16 PC delta table with 4-bit exact LRU, parallel slot-hit + up to 2 new-delta allocs per search
//   * 4-entry train FIFO (accepts up to 2/cycle, drains 1/cycle) + 4-entry search FIFO (1/cycle)
//   * 16-delta tournament burst generator (1 candidate/cycle -> same-page to reqq or cross-page to g_pf_xq)
// ---------------------------------------------------------------------------------------------------------------
class BertiHw {
 public:
  static constexpr int HSETS = 8, HWAYS = 16, DPCS = 16, DDELTAS = 16, INFL = 16;
  static constexpr int PFI_N = 16, PFL_N = 64, ROUND = 16, HI_COV = 11, LO_COV = 6, DEG = 12;
  static constexpr int TRNQ_N = 4, SRCHQ_N = 4;
  static constexpr uint32_t CYC_MASK = (1u << 20) - 1u;

  struct TrnEv { uint32_t pc = 0, vline = 0, pline = 0; bool vaneg = false, miss = false; };
  struct SrchEv { uint32_t pc = 0, line = 0, t_dem = 0; uint16_t lat = 0; };
  struct H { bool v = false; uint32_t pc = 0, line = 0, t = 0; };
  struct Dl { bool v = false; int8_t d = 0; uint8_t cov = 0, last_cov = 0, st = 0; };
  struct P { bool v = false; uint32_t pc = 0; uint8_t cnt = 0, lru = 0; Dl dl[DDELTAS]; };
  struct F { bool v = false; uint32_t pline = 0, vline = 0, pc = 0, t = 0; };
  struct PfI { bool v = false; uint32_t pline = 0, t = 0; };
  struct PfL { bool v = false; uint32_t pline = 0; uint16_t lat = 0; };

  struct Gen {
    uint16_t mask = 0;
    int8_t d[DDELTAS] = {};
    uint8_t st[DDELTAS] = {}, cov[DDELTAS] = {};
    uint32_t vline = 0, pline = 0;
    bool vaneg = false;
    uint8_t left = 0;
  };

  uint32_t cyc = 0;
  H hist[HSETS][HWAYS];
  uint8_t hptr[HSETS] = {};
  P tab[DPCS];
  F infl[INFL];
  uint8_t infl_ptr = 0;
  PfI pfi[PFI_N];
  uint8_t pfi_ptr = 0;
  PfL pfl[PFL_N];
  uint8_t pfl_ptr = 0;
  TrnEv trnq[TRNQ_N];
  int trnq_n = 0;
  SrchEv srchq[SRCHQ_N];
  int srchq_n = 0;
  Gen gen;
  int alloc_per_srch = 2;

  static int hset(uint32_t pc) { return int((pc >> 1) ^ (pc >> 4) ^ (pc >> 7)) & (HSETS - 1); }

  void pf_issued(uint32_t pline) {
    for (int i = 0; i < PFI_N; i++) {
      if (pfi[i].v && pfi[i].pline == pline) { pfi[i].t = cyc; return; }
    }
    pfi[pfi_ptr] = {true, pline, cyc};
    pfi_ptr = (pfi_ptr + 1) & (PFI_N - 1);
  }

  void push_srch(const SrchEv &e) {
    if (srchq_n < SRCHQ_N) srchq[srchq_n++] = e;
  }

  void on_fill(uint32_t pline, bool is_pf) {
    if (is_pf) {
      for (int i = 0; i < PFI_N; i++) {
        if (pfi[i].v && pfi[i].pline == pline) {
          uint32_t diff = (cyc - pfi[i].t) & CYC_MASK;
          uint16_t lat12 = uint16_t(diff > 4095u ? 4095u : diff);
          pfi[i].v = false;
          for (int k = 0; k < PFL_N; k++) {
            if (pfl[k].v && pfl[k].pline == pline) { pfl[k].lat = lat12; return; }
          }
          pfl[pfl_ptr] = {true, pline, lat12};
          pfl_ptr = (pfl_ptr + 1) & (PFL_N - 1);
          return;
        }
      }
      return;
    }
    for (int i = 0; i < INFL; i++) {
      if (infl[i].v && infl[i].pline == pline) {
        uint32_t diff = (cyc - infl[i].t) & CYC_MASK;
        uint16_t lat12 = uint16_t(diff > 4095u ? 4095u : diff);
        push_srch({infl[i].pc, infl[i].vline, infl[i].t, lat12});
        infl[i].v = false;
        return;
      }
    }
  }

  void record(uint32_t pc, uint32_t line) {
    const int s = hset(pc);
    for (int w = 0; w < HWAYS; w++) {
      if (hist[s][w].v && hist[s][w].pc == pc && hist[s][w].line == line) return;
    }
    hist[s][hptr[s]] = {true, pc, line, cyc};
    hptr[s] = (hptr[s] + 1) & (HWAYS - 1);
  }

  void run_search(const SrchEv &ev) {
    const int s = hset(ev.pc);
    bool ok[HWAYS] = {};
    int8_t dw[HWAYS] = {};
    for (int w = 0; w < HWAYS; w++) {
      const H &h = hist[s][w];
      if (!h.v || h.pc != ev.pc) continue;
      uint32_t age = (ev.t_dem - h.t) & CYC_MASK;
      if (age >= (1u << 19) || age < ev.lat) continue;
      int32_t d = int32_t(ev.line - h.line);
      if (d == 0 || d >= 64 || d <= -64) continue;
      ok[w] = true;
      dw[w] = int8_t(d);
    }
    bool uniq[HWAYS] = {};
    bool any_u = false;
    for (int w = 0; w < HWAYS; w++) {
      if (!ok[w]) continue;
      bool dup = false;
      for (int j = 0; j < w; j++) if (ok[j] && dw[j] == dw[w]) dup = true;
      if (!dup) { uniq[w] = true; any_u = true; }
    }
    int e = -1;
    for (int i = 0; i < DPCS; i++) if (tab[i].v && tab[i].pc == ev.pc) { e = i; break; }
    if (e < 0 && !any_u) return;
    bool hit_pc = (e >= 0);
    if (e < 0) {
      for (int i = 0; i < DPCS; i++) if (!tab[i].v) { e = i; break; }
      if (e < 0) {
        e = 0;
        for (int i = 0; i < DPCS; i++) if (tab[i].lru == DPCS - 1) { e = i; break; }
      }
      tab[e] = P{};
      tab[e].v = true;
      tab[e].pc = ev.pc;
    }
    const uint8_t old_lru = hit_pc ? tab[e].lru : uint8_t(DPCS - 1);
    for (int i = 0; i < DPCS; i++) {
      if (tab[i].v && i != e && tab[i].lru < old_lru) tab[i].lru++;
    }
    tab[e].lru = 0;

    P &p = tab[e];
    // 1. Parallel hit increment on existing delta slots
    bool matched_w[HWAYS] = {};
    for (int m = 0; m < DDELTAS; m++) {
      if (!p.dl[m].v) continue;
      bool hit_m = false;
      for (int w = 0; w < HWAYS; w++) {
        if (uniq[w] && dw[w] == p.dl[m].d) { hit_m = true; matched_w[w] = true; }
      }
      if (hit_m) p.dl[m].cov++;
    }
    // 2. Allocate up to alloc_per_srch unmatched unique deltas into free or lowest-cov non-status slots
    int ins = 0;
    for (int w = 0; w < HWAYS && ins < alloc_per_srch; w++) {
      if (!uniq[w] || matched_w[w]) continue;
      int slot = -1;
      for (int m = 0; m < DDELTAS; m++) if (!p.dl[m].v) { slot = m; break; }
      if (slot < 0) {
        for (int m = 0; m < DDELTAS; m++)
          if (p.dl[m].st == 0 && (slot < 0 || p.dl[m].cov < p.dl[slot].cov)) slot = m;
      }
      if (slot < 0) break;
      p.dl[slot] = Dl{true, dw[w], 1, 0, 0};
      ins++;
    }
    p.cnt++;
    if (p.cnt >= ROUND) {
      for (int m = 0; m < DDELTAS; m++) {
        if (!p.dl[m].v) continue;
        p.dl[m].st = p.dl[m].cov >= HI_COV ? 1 : p.dl[m].cov >= LO_COV ? 2 : 0;
        p.dl[m].last_cov = p.dl[m].cov;
        p.dl[m].cov = 0;
      }
      p.cnt = 0;
    }
  }

  void start_gen(const TrnEv &a) {
    int e = -1;
    for (int i = 0; i < DPCS; i++) if (tab[i].v && tab[i].pc == a.pc) { e = i; break; }
    if (e < 0) return;
    uint16_t mask = 0;
    for (int m = 0; m < DDELTAS; m++) if (tab[e].dl[m].v && tab[e].dl[m].st != 0) mask |= uint16_t(1u << m);
    if (!mask) return;
    gen.mask = mask;
    for (int m = 0; m < DDELTAS; m++) {
      gen.d[m] = tab[e].dl[m].d;
      gen.st[m] = tab[e].dl[m].st;
      gen.cov[m] = tab[e].dl[m].last_cov;
    }
    gen.vline = a.vline;
    gen.pline = a.pline;
    gen.vaneg = a.vaneg;
    gen.left = DEG;
  }

  // Step generator: returns 0 (idle/stalled), 1 (same-page -> out_line/out_llc), 2 (cross-page -> pushed to g_pf_xq)
  int step_gen(bool reqq_busy, uint32_t &out_line, bool &out_llc) {
    if (gen.left == 0 || gen.mask == 0) return 0;
    int best = -1;
    uint8_t best_key = 0;
    for (int m = 0; m < DDELTAS; m++) {
      if (!(gen.mask & (1u << m))) continue;
      uint8_t key = uint8_t(((gen.st[m] == 1 ? 1u : 0u) << 5) | (gen.cov[m] & 31u));
      if (best < 0 || key > best_key) { best = m; best_key = key; }
    }
    const uint32_t t = gen.vline + uint32_t(int32_t(gen.d[best]));
    const bool same_page = (t >> 6) == (gen.vline >> 6);
    if (same_page && reqq_busy) return 0;
    gen.mask &= uint16_t(~(1u << best));
    gen.left--;
    if (same_page) {
      out_line = ((gen.pline >> 6) << 6) | (t & 63u);
      out_llc = (gen.st[best] == 2);
      return 1;
    }
    const uint64_t hi = gen.vaneg ? ~uint64_t(0x3fffffffffull) : 0ull;
    g_pf_xq.push({hi | (uint64_t(t) << 6), gen.st[best] == 2});
    return 2;
  }
};

// ---------------------------------------------------------------------------------------------------------------
class DataPrefetcher {
 public:
  // ---- combinational inputs (set by the L1D in stage 2 of the current cycle) ----
  PfAccess acc[LSU_LDU_COUNT];
  PfFill fill_in;   // EXP-048: the line the L1D fills this cycle
  // ---- combinational output (stage 1 of the L1D): head of the request FIFO ----
  bool head_valid() const { return enabled_ && !reqq.empty(); }
  uint32_t head_addr() const { return reqq.front().line << 6; }
  bool head_llc_only() const { return reqq.front().llc; }
  // set by the L1D in stage 1: the head was sent to the tag probe this cycle
  bool head_taken = false;
  bool enabled() const { return enabled_; }

  DataPrefetcher()
      : reqq("pf_reqq", 1), stride("pf_stride", PF_TRAIN_PORTS, PF_TRAIN_PORTS) {
    enabled_ = pf::param("ENABLE", 1) != 0;   // default on; PF_ENABLE=0 turns the prefetcher off
    pc_stats_ = std::getenv("PF_STATS") != nullptr;
    const char *k = std::getenv("PF_KIND");   // EXP-048: 0 = h1 (default), 1 = Best-Offset (Bop.h, parameters PF_BOP_*)
    kind_ = k ? std::atoi(k) : 4;
    if (kind_ == 1) { bop_ = new Bop("PF_BOP"); g_bop_l1 = bop_; }
    if (kind_ == 4) { const char *a = std::getenv("PF_BERTI_HWALLOC"); if (a) bertihw_.alloc_per_srch = std::max(1, std::atoi(a)); }
    if (kind_ == 2 || kind_ == 3) {   // EXP-048 Berti (3: together with h1): defaults = the best EXP-043 variant (ALLOC + 64 history ways)
      auto g = [](const char *n, int d) { const char *v = std::getenv(n); return v ? std::atoi(v) : d; };
      berti_.deg = g("PF_BERTI_DEG", 12);
      berti_.hi = g("PF_BERTI_HI", 65);
      berti_.lo = g("PF_BERTI_LO", 35);
      berti_.alloc_found = g("PF_BERTI_ALLOC", 1) != 0;
      berti_.hw = std::max(1, std::min(BertiPf::HWAYS, g("PF_BERTI_HW", 64)));
      berti_.hdedup = g("PF_BERTI_HDEDUP", 0);
      berti_.late_fix = g("PF_BERTI_LATEFIX", 1) != 0;
      berti_.far_first = g("PF_BERTI_FAR", 0) != 0;
      bappend_ = g("PF_BERTI_APPEND", 0) != 0;
      bramp_ = g("PF_BERTI_RAMP", 0);
      bva_ = g("PF_BERTI_VA", 0) != 0;
      berti_.cross_page = bva_;
      bxlate_ = g("PF_BERTI_XLATE", 1);
      bwalk_ = g("PF_BERTI_WALK", 0) != 0;
      bxport_ = g("PF_BERTI_XPORT", 0) != 0;   // cross-page translations per cycle (a DTLB lookup each; the rest dropped)
      brf_n_ = std::max(0, std::min(256, g("PF_BERTI_RFILT", 64)));   // recently pushed lines not queued again (0 = off)
    }
  }

  // end of cycle: consume the inputs, update the state
  void seq() {
    bxlate_used_ = 0;
    if (head_taken) {
      if ((kind_ == 2 || kind_ == 3) && !reqq.front().llc) berti_.pf_issued(reqq.front().line, sim_time);
      if (kind_ == 4 && !reqq.front().llc) bertihw_.pf_issued(reqq.front().line);   // Berti: prefetch latency from here
      reqq.pop();
    }
    head_taken = false;
    if (!enabled_) { clear_in(); return; }
    if (pc_stats_)
      for (int i = 0; i < LSU_LDU_COUNT; i++) {
        const PfAccess &a = acc[i];
        if (a.valid && a.miss) { g_pf_stats.pc_miss[a.pc].first++; g_pf_stats.off_miss[(a.paddr >> 9) & 7]++; }
        if (a.valid && a.pf_hit) { g_pf_stats.pc_miss[a.pc].second++; g_pf_stats.off_pfu[(a.paddr >> 9) & 7]++; }
      }
    if (kind_ == 1) { seq_bop(); return; }
    if (kind_ == 2) { seq_berti(); return; }
    if (kind_ == 4) { seq_berti_hw(); return; }

    // EXP-048 PF_KIND=3: h1 and Berti together (one FIFO push per cycle: h1 stride burst > Berti > h1 LLC next-line)
    if (kind_ == 3 && fill_in.valid) berti_.fill((fill_in.paddr >> 6) & pf::wmask(PF_LINE_BITS), fill_in.is_prefetch, sim_time);
    // 1. the stride generator pushes one candidate per cycle (page-limited, de-duplicated against the FIFO)
    bool port_used = false;
    if (gen.valid) {
      port_used = true;
      const uint32_t line = gen.next;
      if ((line >> 6) != (gen.base >> 6)) {          // stay inside the 4 KiB page: no translation is needed
        gen.valid = false;
      } else {
        push_candidate(line, /*llc=*/false);
        gen.pushed++;
        gen.next = (gen.next + uint32_t(gen.stride)) & pf::wmask(PF_LINE_BITS);
        if (--gen.left == 0) gen.valid = false;
      }
    }
    if (kind_ == 3 && !port_used && g_pf_xans_n > 0) {
      port_used = true;
      push_candidate(g_pf_xans[0].line, g_pf_xans[0].llc);
      for (int i = 1; i < g_pf_xans_n; i++) g_pf_xans[i - 1] = g_pf_xans[i];
      g_pf_xans_n--;
    }
    if (kind_ == 3 && !port_used && bn_head < bn_cnt) {
      port_used = true;
      push_candidate(bcand[bn_head].line, bcand[bn_head].llc);
      berti_.st_pushed++;
      bn_head++;
    }
    // 2. the LLC next-line engine uses the FIFO write port only when the stride generator did not
    if (!port_used && gen2.valid) {
      const uint32_t line = gen2.next;
      if ((line >> 6) != (gen2.base >> 6)) {
        gen2.valid = false;
      } else {
        push_candidate(line, /*llc=*/true);
        gen2.next = (gen2.next + 1) & pf::wmask(PF_LINE_BITS);
        if (--gen2.left == 0) gen2.valid = false;
      }
    }
    // 3. training on the events registered in the previous cycle (keeps the prefetcher off the L1D stage-2 path)
    for (int k = 0; k < PF_TRAIN_PORTS; k++) if (q_acc[k].valid) train(q_acc[k]);
    if (kind_ == 3) for (int k = 0; k < PF_TRAIN_PORTS; k++) if (q_acc[k].valid) berti_train(q_acc[k]);

    // 4. input registers: the first PF_TRAIN_PORTS events (in load-port order) that are misses or first uses of prefetched lines
    {
      PfAccess nq[PF_TRAIN_PORTS];
      int taken = 0;
      for (int i = 0; i < LSU_LDU_COUNT; i++) {
        const PfAccess &a = acc[i];
        if (!a.valid || !(a.miss || a.pf_hit)) continue;
        if (taken >= PF_TRAIN_PORTS) continue;
        nq[taken++] = a;
      }
      for (int k = 0; k < PF_TRAIN_PORTS; k++) q_acc[k] = nq[k];
    }
    clear_in();
    reqq.tick(); stride.tick();
  }

 private:
  void clear_in() { for (auto &a : acc) a = PfAccess{}; fill_in = PfFill{}; }

  // EXP-048 Best-Offset in place of h1: the fill writes RR, every registered trigger asks for X + D; the candidates wait in a
  // small buffer and go into the request FIFO one per cycle (its one write port)
  void seq_bop() {
    if (fill_in.valid) bop_->fill(fill_in.paddr >> 6, fill_in.is_prefetch);
    if (!bop_pend_.empty()) { push_candidate(bop_pend_.front(), /*llc=*/false); bop_pend_.pop_front(); bop_->stats_sent++; }
    for (int k = 0; k < PF_TRAIN_PORTS; k++) {
      if (!q_acc[k].valid) continue;
      bop_->stats_useful += q_acc[k].pf_hit;
      uint32_t out[8];
      const int n = bop_->trigger((q_acc[k].paddr >> 6) & pf::wmask(PF_LINE_BITS), out, 8);
      for (int i = 0; i < n; i++) {
        if (bop_pend_.size() >= 8) bop_pend_.pop_front();
        bop_pend_.push_back(out[i]);
      }
    }
    PfAccess nq[PF_TRAIN_PORTS];
    int taken = 0;
    for (int i = 0; i < LSU_LDU_COUNT; i++) {
      const PfAccess &a = acc[i];
      if (!a.valid || !(a.miss || a.pf_hit) || taken >= PF_TRAIN_PORTS) continue;
      nq[taken++] = a;
    }
    for (int k = 0; k < PF_TRAIN_PORTS; k++) q_acc[k] = nq[k];
    clear_in();
    reqq.tick(); stride.tick();
  }

  // EXP-048 Berti in place of h1 (PF_KIND=2): fills measure latencies, the registered triggers train and produce candidates
  // (a new trigger with candidates replaces the pending ones), one candidate per cycle into the request FIFO
  // EXP-048 M3: hardware-faithful Berti (PF_KIND=4)
  void seq_berti_hw() {
    // 1. Pop 1 pending search from srchq (registered from previous cycle) and run it
    if (bertihw_.srchq_n > 0) {
      BertiHw::SrchEv ev = bertihw_.srchq[0];
      for (int i = 1; i < bertihw_.srchq_n; i++) bertihw_.srchq[i - 1] = bertihw_.srchq[i];
      bertihw_.srchq_n--;
      bertihw_.run_search(ev);
    }
    // 2. Push 1 candidate to reqq (xans_q takes priority; gen steps same-page if free, or cross-page into g_pf_xq)
    bool reqq_used = false;
    if (g_pf_xans_n > 0) {
      push_candidate(g_pf_xans[0].line, g_pf_xans[0].llc);
      for (int i = 1; i < g_pf_xans_n; i++) g_pf_xans[i - 1] = g_pf_xans[i];
      g_pf_xans_n--;
      reqq_used = true;
    }
    uint32_t gline = 0; bool gllc = false;
    if (bertihw_.step_gen(reqq_used, gline, gllc) == 1) push_candidate(gline, gllc);
    // 3. Process fill_in (latency measurement -> pf_lat or demand search enqueue)
    if (fill_in.valid) bertihw_.on_fill((fill_in.paddr >> 6) & pf::wmask(PF_LINE_BITS), fill_in.is_prefetch);
    // 4. Pop 1 training event from trnq -> miss/pf_hit + history record + start_gen
    if (bertihw_.trnq_n > 0) {
      BertiHw::TrnEv a = bertihw_.trnq[0];
      for (int i = 1; i < bertihw_.trnq_n; i++) bertihw_.trnq[i - 1] = bertihw_.trnq[i];
      bertihw_.trnq_n--;
      if (a.miss) {
        bool have = false;
        for (int i = 0; i < BertiHw::INFL; i++) if (bertihw_.infl[i].v && bertihw_.infl[i].pline == a.pline) have = true;
        if (!have) {
          bertihw_.infl[bertihw_.infl_ptr] = {true, a.pline, a.vline, a.pc, bertihw_.cyc};
          bertihw_.infl_ptr = (bertihw_.infl_ptr + 1) & (BertiHw::INFL - 1);
        }
      } else {
        for (int i = 0; i < BertiHw::PFL_N; i++) {
          if (bertihw_.pfl[i].v && bertihw_.pfl[i].pline == a.pline) {
            bertihw_.push_srch({a.pc, a.vline, bertihw_.cyc, bertihw_.pfl[i].lat});
            bertihw_.pfl[i].v = false;
            break;
          }
        }
      }
      bertihw_.record(a.pc, a.vline);
      bertihw_.start_gen(a);
    }
    // 5. Enqueue up to PF_TRAIN_PORTS new load miss/pf_hit events into trnq
    int taken = 0;
    for (int i = 0; i < LSU_LDU_COUNT && taken < PF_TRAIN_PORTS; i++) {
      const PfAccess &a = acc[i];
      if (!a.valid || !(a.miss || a.pf_hit)) continue;
      if (bertihw_.trnq_n < BertiHw::TRNQ_N) {
        bertihw_.trnq[bertihw_.trnq_n++] = {
          a.pc, uint32_t(a.va >> 6), (a.paddr >> 6) & pf::wmask(PF_LINE_BITS), ((a.va >> 38) & 1ull) != 0, a.miss
        };
      }
      taken++;
    }
    bertihw_.cyc = (bertihw_.cyc + 1u) & BertiHw::CYC_MASK;
    clear_in();
    reqq.tick(); stride.tick();
  }

  void seq_berti() {
    if (fill_in.valid) berti_.fill((fill_in.paddr >> 6) & pf::wmask(PF_LINE_BITS), fill_in.is_prefetch, sim_time);
    if (g_pf_xans_n > 0) {   // EXP-048 XPORT: a translated cross-page candidate takes this cycle's FIFO push
      push_candidate(g_pf_xans[0].line, g_pf_xans[0].llc);
      for (int i = 1; i < g_pf_xans_n; i++) g_pf_xans[i - 1] = g_pf_xans[i];
      g_pf_xans_n--;
    } else if (bn_head < bn_cnt) {
      push_candidate(bcand[bn_head].line, bcand[bn_head].llc);
      berti_.st_pushed++;
      if (brf_n_ > 0) { brf_[brf_ptr_] = bcand[bn_head].line + 1; brf_ptr_ = (brf_ptr_ + 1) % brf_n_; }
      bn_head++;
    }
    for (int k = 0; k < PF_TRAIN_PORTS; k++) if (q_acc[k].valid) berti_train(q_acc[k]);
    PfAccess nq[PF_TRAIN_PORTS];
    int taken = 0;
    for (int i = 0; i < LSU_LDU_COUNT; i++) {
      const PfAccess &a = acc[i];
      if (!a.valid || !(a.miss || a.pf_hit) || taken >= PF_TRAIN_PORTS) continue;
      nq[taken++] = a;
    }
    for (int k = 0; k < PF_TRAIN_PORTS; k++) q_acc[k] = nq[k];
    clear_in();
    reqq.tick(); stride.tick();
  }

  // one registered training event: Berti learns, its candidates (with the page-start ramp) go to the pending buffer
  void berti_train(const PfAccess &a) {
    {
      const uint32_t pline = (a.paddr >> 6) & pf::wmask(PF_LINE_BITS);
      const uint32_t line = bva_ ? uint32_t(a.va >> 6) : pline;   // virtual mode: the training / target lines are virtual
      if (a.miss) berti_.miss(a.pc, line, sim_time, pline); else berti_.pf_hit(a.pc, line, sim_time, pline);
      BertiPf::Cand c[64];
      int n = berti_.trigger(a.pc, line, c);
      if (bva_) {   // EXP-048: virtual targets -> physical lines: same page from the trigger's frame, another page by a DTLB lookup
        int m = 0;
        for (int i = 0; i < n; i++) {
          const uint32_t t = c[i].line;
          if ((t >> 6) == (line >> 6)) {
            c[m++] = {((pline >> 6) << 6) | (t & 63u), c[i].llc};
            continue;
          }
          if (bxport_) {   // translated on a real DTLB port by the LSU (next cycles)
            g_pf_xq.push({(a.va & ~uint64_t(0x3fffffffffull)) | (uint64_t(t) << 6), c[i].llc});
            continue;
          }
          if (bxlate_used_ >= bxlate_) { g_pf_stats.x_port++; continue; }
          bxlate_used_++;
          uint32_t pa = 0;
          const uint64_t tva = (a.va & ~uint64_t(0x3fffffffffull)) | (uint64_t(t) << 6);   // (the line keeps 32 bits of the vaddr)
          if (g_pf_translate && g_pf_translate(pa, tva)) { g_pf_stats.x_ok++; c[m++] = {(pa >> 6) & pf::wmask(PF_LINE_BITS), c[i].llc}; }
          else { g_pf_stats.x_miss++; if (bwalk_) g_pf_walk.push(tva); }
        }
        n = m;
      }
      if (bramp_ > 0 && a.miss && n > 0) {   // EXP-048 PF_BERTI_RAMP=n: a demand miss also asks for the lines below the nearest timely
        int dmin = 64;                         // L1 delta (X+1 .. X+dmin-1, at most n, same page), first: a page start is not covered
        for (int i = 0; i < n; i++) if (!c[i].llc) { const int d = int(c[i].line - line); if (d > 0 && d < dmin) dmin = d; }
        BertiPf::Cand r[64];
        int m = 0;
        for (int d = 1; d < dmin && d <= bramp_ && m < 64; d++) {
          const uint32_t t = line + uint32_t(d);
          if ((t >> 6) != (line >> 6)) break;
          r[m++] = {t, false};
        }
        for (int i = 0; i < n && m < 64; i++) r[m++] = c[i];
        for (int i = 0; i < m; i++) c[i] = r[i];
        n = m;
      }
      if (brf_n_ > 0) {   // EXP-048: drop candidates pushed recently (in steady state only the farthest delta is a new line)
        int m = 0;
        for (int i = 0; i < n; i++) {
          bool recent = false;
          for (int j = 0; j < brf_n_ && !recent; j++) recent = brf_[j] == c[i].line + 1;
          if (!recent) c[m++] = c[i]; else st_rfilt_++;
        }
        n = m;
      }
      if (n > 0 && bappend_) {   // EXP-048 PF_BERTI_APPEND=1: queue behind the pending candidates (oldest dropped when full)
        const int cap = int(sizeof(bcand) / sizeof(bcand[0]));
        int m = 0;
        BertiPf::Cand t[sizeof(bcand) / sizeof(bcand[0]) + 64];
        for (int i = bn_head; i < bn_cnt; i++) t[m++] = bcand[i];
        for (int i = 0; i < n; i++) {
          bool dup = false;
          for (int j = 0; j < m && !dup; j++) dup = t[j].line == c[i].line;
          if (dup) continue;
          if (m == cap) { for (int j = 1; j < m; j++) t[j - 1] = t[j]; m--; berti_.st_replaced++; }
          t[m++] = c[i];
        }
        for (int i = 0; i < m; i++) bcand[i] = t[i];
        bn_head = 0;
        bn_cnt = m;
      } else if (n > 0) {
        berti_.st_replaced += uint64_t(bn_cnt - bn_head);
        bn_cnt = std::min(n, int(sizeof(bcand) / sizeof(bcand[0])));
        for (int i = 0; i < bn_cnt; i++) bcand[i] = c[i];
        bn_head = 0;
      }
    }
  }

  void push_candidate(uint32_t line, bool llc) {
    const bool dup = reqq.contains([&](const PfReqE &x) { return x.line == line; }, PF_LINE_BITS);
    if (dup) { g_pf_stats.q_dup++; return; }
    if (reqq.count() == PF_QUEUE) g_pf_stats.q_full++; else g_pf_stats.q_push++;
    PfReqE e; e.line = line; e.llc = llc;
    reqq.push(e);   // a full queue drops the candidate
  }

  // A stride burst: the first candidate is DIST strides after the trigger line, then one every stride (a new trigger replaces it).
  void start_gen(uint32_t base_line, int32_t stride_lines) {
    gen.valid = true;
    gen.base = base_line;
    gen.stride = stride_lines;
    gen.next = (base_line + uint32_t(stride_lines * PF_DIST)) & pf::wmask(PF_LINE_BITS);
    gen.left = PF_DEGREE;
    gen.pushed = 0;
  }

  void train(const PfAccess &a) {
    const uint32_t line = (a.paddr >> 6) & pf::wmask(PF_LINE_BITS);
    if (a.miss) {  // LLC engine: the next PF_LLC_LINES lines of every load miss
      gen2.valid = true; gen2.base = line; gen2.next = (line + 1) & pf::wmask(PF_LINE_BITS); gen2.left = PF_LLC_LINES;
    }
    // single global stride entry: allocate on the first event, then compare the line distance with the stored stride
    PfStrideE e = stride.rd(0);
    if (!e.valid) {
      PfStrideE n; n.valid = 1; n.last = line; n.stride = 0; n.conf = 0;
      stride.wr(0, n);
      return;
    }
    const int64_t d = pf::sext((line - e.last) & pf::wmask(PF_LINE_BITS), PF_LINE_BITS);
    if (d == 0) return;                                   // same line again: no update
    const bool in_range = d >= -(int64_t(1) << (PF_STRIDE_BITS - 1)) && d < (int64_t(1) << (PF_STRIDE_BITS - 1));
    const int64_t cur = pf::sext(e.stride, PF_STRIDE_BITS);
    if (in_range && d == cur) {
      if (e.conf < 3) e.conf++;
    } else if (e.conf > 0) {
      e.conf--;
      if (e.conf == 0 && in_range) e.stride = uint32_t(d) & pf::wmask(PF_STRIDE_BITS);
    } else {
      e.stride = in_range ? (uint32_t(d) & pf::wmask(PF_STRIDE_BITS)) : 0;
    }
    e.last = line;
    stride.wr(0, e);
    const int32_t sv = int32_t(pf::sext(e.stride, PF_STRIDE_BITS));
    if (e.conf >= uint32_t(PF_CONF_THR) && sv != 0) start_gen(line, sv);
  }

  PfAccess q_acc[PF_TRAIN_PORTS];   // registered training events (selected in the previous cycle)
  struct Gen { bool valid = false; uint32_t next = 0, base = 0, pushed = 0; int32_t stride = 0; int left = 0; } gen;
  struct Gen2 { bool valid = false; uint32_t next = 0, base = 0; int left = 0; } gen2;
  pf::Fifo<PfReqE, PF_QUEUE> reqq;
  pf::Table<PfStrideE, 1> stride;
  bool enabled_ = false;
  int kind_ = 0;
  bool pc_stats_ = false;
  Bop *bop_ = nullptr;
  std::deque<uint32_t> bop_pend_;
  BertiPf berti_;
  BertiHw bertihw_;
  uint32_t brf_[256] = {};   // recently pushed lines + 1 (0 = empty)
  int brf_n_ = 0, brf_ptr_ = 0;
  uint64_t st_rfilt_ = 0;
  bool bappend_ = false;
  int bramp_ = 0;
  bool bva_ = false;
  bool bwalk_ = false;
  bool bxport_ = false;
  int bxlate_ = 1, bxlate_used_ = 0;
  BertiPf::Cand bcand[24];
  int bn_head = 0, bn_cnt = 0;
};
