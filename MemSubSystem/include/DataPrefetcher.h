#pragma once
// L1D data prefetcher core (EXP-009). See Prefetch.h for the circuit-style rules.
#include "Prefetch.h"

struct PfStats {
  uint64_t train_events = 0, train_drop = 0, cand = 0, q_dup = 0, q_full = 0, q_pushed = 0;
  uint64_t probe = 0, probe_hit = 0, probe_inflight = 0, probe_mshr_full = 0, probe_wb = 0, probe_conflict = 0;
  uint64_t issued = 0, filled = 0, useful = 0, late = 0, useless_evict = 0, xpage_drop = 0;
  uint64_t demand_miss = 0, demand_hit = 0, llc_only_done = 0;
};
inline PfStats g_pf_stats;
inline uint64_t g_ic_req = 0, g_ic_miss = 0, g_ic_penalty = 0;  // I-cache requests, misses, total miss penalty cycles

// one load access seen at stage 2 of the L1D pipeline
struct PfAccess {
  bool valid = false;
  uint32_t paddr = 0;
  uint32_t pc = 0;       // hashed PC (PF_PC_BITS)
  bool miss = false;     // L1D miss (new or merged into an in-flight line)
  bool pf_hit = false;   // hit on a line that was brought in by a prefetch and not used before
};

#define PF_REQ_FIELDS(X) X(line, PF_LINE_BITS) X(llc, 1)
PF_ENTRY(PfReqE, PF_REQ_FIELDS)
#define PF_RR_FIELDS(X) X(tag, 12)
PF_ENTRY(PfRrE, PF_RR_FIELDS)
#undef PF_RR_FIELDS
#define PF_SCORE_FIELDS(X) X(score, 6)
PF_ENTRY(PfScoreE, PF_SCORE_FIELDS)
#undef PF_SCORE_FIELDS

// a line was filled into the L1D this cycle (feedback for timeliness)
struct PfFill {
  bool valid = false;
  uint32_t paddr = 0;
  bool is_prefetch = false;
  bool victim_unused = false;  // the line it replaces was brought in by a prefetch and never used
};
#undef PF_REQ_FIELDS
#ifndef CFG_PF_STRIDE_BITS
#define CFG_PF_STRIDE_BITS 8   // signed stride range in lines; 18 covers the 45,548-line stride of mcf's arc scan
#endif
constexpr int PF_STRIDE_BITS = CFG_PF_STRIDE_BITS;
#define PF_STRIDE_FIELDS(X) X(tag, 4) X(last, PF_LINE_BITS) X(stride, CFG_PF_STRIDE_BITS) X(conf, 2) X(valid, 1)
PF_ENTRY(PfStrideE, PF_STRIDE_FIELDS)
#undef PF_STRIDE_FIELDS

class DataPrefetcher {
 public:
  // ---- combinational inputs (set by the L1D in stage 2 of the current cycle) ----
  PfAccess acc[LSU_LDU_COUNT];
  PfFill fill;
  // ---- combinational output (stage 1 of the L1D): head of the request FIFO ----
  bool head_valid() const { return !reqq.empty(); }
  uint32_t head_addr() const { return reqq.front().line << 6; }
  bool head_llc_only() const { return reqq.front().llc; }
  // set by the L1D in stage 1: the head was sent to the tag probe this cycle
  bool head_taken = false;

  DataPrefetcher()
      : reqq("pf_reqq", 1), stride("pf_stride", PF_TRAIN_PORTS, PF_TRAIN_PORTS),
        gstride("pf_gstride", PF_TRAIN_PORTS, PF_TRAIN_PORTS), rr("pf_bop_rr", PF_TRAIN_PORTS, 1), score("pf_bop_score", PF_TRAIN_PORTS, PF_TRAIN_PORTS) {
    degree = pf::param("DEGREE", 4);
    dist = pf::param("DIST", 1);
    conf_thr = pf::param("CONF", 2);
    xpage = pf::param("XPAGE", 0);
    near_lines = pf::param("NEAR", 1000);
    throttle = pf::param("THROTTLE", 0);
    hybrid = pf::param("HYBRID", 0);
    nopc = pf::param("NOPC", 0);          // 1: every load uses PC hash 0 (a global stride detector; needs no PC from the LSU)
    nllc = pf::param("NLLC", 0);          // LLC-only next-line engine: lines +1..+nllc of every L1D load miss
    nllc_dist = pf::param("NLLC_DIST", 1);
    mshr_reserve = pf::param("RESERVE", PF_MSHR_RESERVE);
    acc_hi = pf::param("ACC_HI", 70);
    acc_lo = pf::param("ACC_LO", 35);
    acc_win = pf::param("ACC_WIN", 256);
    stride_sets = 64;
    bop_scoremax = pf::param("BOP_SCOREMAX", 31);
    bop_roundmax = pf::param("BOP_ROUNDMAX", 100);
    bop_badscore = pf::param("BOP_BAD", 2);
    pf::StateReport r;
    report(r);
    if (std::getenv("PF_STATS")) {
      std::fprintf(stderr, "[PF] algo=%d degree=%d dist=%d conf=%d state=%ld bits (%.1f B)\n", PF_ALGO, degree, dist,
                   conf_thr, r.total(), r.total() / 8.0);
      for (auto &i : r.items)
        std::fprintf(stderr, "[PF]   %-12s %6ld bits  rd=%d wr=%d cam=%d (cam bits %ld)\n", i.name, i.bits, i.rd, i.wr,
                     i.cam, i.cam_bits);
    }
    if (r.total() > CFG_PF_STATE_BUDGET_BITS) pf::fail("state budget exceeded", "total");
  }

  ~DataPrefetcher() {
    if (std::getenv("PF_STATS")) {
      const PfStats &s = g_pf_stats;
      std::fprintf(stderr,
                   "[PF-STATS] demand_miss=%llu demand_hit=%llu train=%llu train_drop=%llu cand=%llu q_dup=%llu q_full=%llu "
                   "pushed=%llu probe=%llu probe_hit=%llu probe_inflight=%llu probe_wb=%llu probe_mshr_full=%llu "
                   "ic_req=%llu ic_miss=%llu ic_penalty=%llu "
                   "probe_conflict=%llu issued=%llu filled=%llu useful=%llu late=%llu useless_evict=%llu xpage_drop=%llu llc_only_done=%llu\n",
                   (unsigned long long)s.demand_miss, (unsigned long long)s.demand_hit, (unsigned long long)s.train_events,
                   (unsigned long long)s.train_drop, (unsigned long long)s.cand, (unsigned long long)s.q_dup,
                   (unsigned long long)s.q_full, (unsigned long long)s.q_pushed, (unsigned long long)s.probe,
                   (unsigned long long)s.probe_hit, (unsigned long long)s.probe_inflight, (unsigned long long)s.probe_wb,
                   (unsigned long long)s.probe_mshr_full,
                   (unsigned long long)g_ic_req, (unsigned long long)g_ic_miss, (unsigned long long)g_ic_penalty,
                   (unsigned long long)s.probe_conflict,
                   (unsigned long long)s.issued, (unsigned long long)s.filled, (unsigned long long)s.useful,
                   (unsigned long long)s.late, (unsigned long long)s.useless_evict, (unsigned long long)s.xpage_drop,
                   (unsigned long long)s.llc_only_done);
    }
  }

  void report(pf::StateReport &r) const {
    reqq.report(r);
    r.items.push_back({"pf_gen", PF_LINE_BITS + 8 + 4 + 1, 1, 1, 0, 0});
    r.items.push_back({"pf_throttle", 10 + 10 + 2, 1, 1, 0, 0});
    r.items.push_back({"pf_inreg", PF_TRAIN_PORTS * (PF_LINE_BITS + 2 + PF_PC_BITS) + (PF_LINE_BITS + 3) + 3, 1, 1, 0, 0});
    if (nllc > 0) r.items.push_back({"pf_gen2", PF_LINE_BITS + 4 + 1, 1, 1, 0, 0});
    if (PF_ALGO == 2 || PF_ALGO == 4) { stride.report(r); if (hybrid) gstride.report(r); }
    if (PF_ALGO == 3 || PF_ALGO == 4) {
      rr.report(r); score.report(r);
      r.items.push_back({"pf_bop_ctl", 5 + 7 + 6 + 6 + 1, 1, 1, 0, 0});  // offset idx, round, best offset, best score, on
    }
  }

  // end of cycle: consume the inputs, update the state
  void seq() {
    if (head_taken) reqq.pop();
    head_taken = false;
    if (PF_ALGO == 0) { clear_in(); return; }

    // 1. generator emits one candidate per cycle into the request FIFO
    bool pri_used_port = false;
    if (gen.valid) {
      pri_used_port = true;
      const uint32_t line = gen.next;
      const bool same_page = xpage || ((line >> 6) == (gen.base >> 6));
      if (!same_page) {
        g_pf_stats.xpage_drop++;
        gen.valid = false;
      } else {
        g_pf_stats.cand++;
        const bool dup = reqq.contains([&](const PfReqE &x) { return x.line == line; }, PF_LINE_BITS);
        if (dup) {
          g_pf_stats.q_dup++;
        } else {
          PfReqE e; e.line = line;
          e.llc = gen.pushed >= uint32_t(near_lines);   // the first `near` lines of a burst fill the L1D, the rest only the LLC
          if (reqq.push(e)) g_pf_stats.q_pushed++; else g_pf_stats.q_full++;
        }
        gen.pushed++;
        gen.next = (gen.next + uint32_t(gen.stride)) & pf::wmask(PF_LINE_BITS);
        if (--gen.left == 0) gen.valid = false;
      }
    }

    // 1-llc. secondary generator (LLC-only next lines): uses the FIFO enqueue port only when the primary one is idle
    if (!pri_used_port && gen2.valid) {
      const uint32_t line = gen2.next;
      if ((line >> 6) != (gen2.base >> 6)) {
        gen2.valid = false;
      } else {
        g_pf_stats.cand++;
        const bool dup = reqq.contains([&](const PfReqE &x) { return x.line == line; }, PF_LINE_BITS);
        if (dup) g_pf_stats.q_dup++;
        else { PfReqE e; e.line = line; e.llc = 1; if (reqq.push(e)) g_pf_stats.q_pushed++; else g_pf_stats.q_full++; }
        gen2.next = (gen2.next + 1) & pf::wmask(PF_LINE_BITS);
        if (--gen2.left == 0) gen2.valid = false;
      }
    }

    // 1a. accuracy feedback (10-bit counters): +1 per first use of a prefetched line, +1 per unused one evicted
    if (throttle) {
      fb_useful += q_useful_n; fb_total += q_useful_n;
      if (q_fill.valid && q_fill.victim_unused) fb_total++;
      if (int(fb_total) >= acc_win) {
        if (fb_useful * 100 >= uint32_t(acc_hi) * fb_total) { if (level < 3) level++; }
        else if (fb_useful * 100 <= uint32_t(acc_lo) * fb_total) { if (level > 0) level--; }
        fb_useful = fb_total = 0;
      }
    }

    // 1b. fill feedback for the best-offset learner (one RR write per fill)
    if (q_fill.valid && (PF_ALGO == 3 || PF_ALGO == 4)) {
      const uint32_t y = (q_fill.paddr >> 6) & pf::wmask(PF_LINE_BITS);
      const uint32_t base = q_fill.is_prefetch ? ((y - uint32_t(bop_d)) & pf::wmask(PF_LINE_BITS)) : y;
      if (q_fill.is_prefetch ? bop_on : true) {
        PfRrE e; e.tag = rr_tag(base);
        rr.wr(rr_idx(base), e);
      }
    }

    // 2. training on the events registered in the previous cycle (keeps the prefetcher off the L1D stage-2 path)
    for (int k = 0; k < PF_TRAIN_PORTS; k++) if (q_acc[k].valid) train(q_acc[k]);

    // 3. input registers: select up to PF_TRAIN_PORTS interesting events (misses and first uses of prefetched lines)
    {
      PfAccess nq[PF_TRAIN_PORTS];
      int taken = 0;
      uint32_t n_useful = 0;
      for (int i = 0; i < LSU_LDU_COUNT; i++) {
        const PfAccess &a = acc[i];
        if (!a.valid) continue;
        if (a.pf_hit) n_useful++;
        if (!(a.miss || a.pf_hit)) continue;
        g_pf_stats.train_events++;
        if (taken >= PF_TRAIN_PORTS) { g_pf_stats.train_drop++; continue; }
        nq[taken++] = a;
      }
      for (int k = 0; k < PF_TRAIN_PORTS; k++) q_acc[k] = nq[k];
      q_fill = fill;
      q_useful_n = n_useful;
    }
    clear_in();
    reqq.tick(); stride.tick(); gstride.tick(); rr.tick(); score.tick();
  }

 private:
  void clear_in() { for (auto &a : acc) a = PfAccess{}; fill = PfFill{}; }

  static uint32_t rr_idx(uint32_t line) { return (line ^ (line >> 7)) & 255u; }
  static uint32_t rr_tag(uint32_t line) { return ((line >> 3) ^ (line >> 11) ^ (line >> 17)) & 0xFFFu; }

  // Best-offset learning: one candidate offset is tested per trigger; the winner of a phase becomes the prefetch offset.
  static constexpr int kOffsets[26] = {1, 2, 3, 4, 5, 6, 8, 9, 10, 12, 15, 16, 18, 20, 24, 25, 27, 30, 32, 36, 40, 45, 48, 50, 54, 60};
  void bop_train(const PfAccess &a, bool issue) {
    const uint32_t x = (a.paddr >> 6) & pf::wmask(PF_LINE_BITS);
    const int off = kOffsets[bop_idx];
    const uint32_t probe = (x - uint32_t(off)) & pf::wmask(PF_LINE_BITS);
    if (((probe >> 6) == (x >> 6)) && rr.rd(rr_idx(probe)).tag == rr_tag(probe)) {
      PfScoreE s = score.rd(bop_idx);
      s.score++;
      score.wr(bop_idx, s);
      if (int(s.score) > bop_best_score) { bop_best_score = s.score; bop_best = off; }
      if (int(s.score) >= bop_scoremax) bop_end_phase();
    }
    if (++bop_idx == 26) { bop_idx = 0; if (++bop_round >= bop_roundmax) bop_end_phase(); }
    if (issue && bop_on) start_gen(x, bop_d, eff_degree(), 1);
  }
  void bop_end_phase() {
    bop_on = bop_best_score >= bop_badscore;
    if (bop_on) bop_d = bop_best;
    for (int i = 0; i < 26; i++) { PfScoreE z; score.e[i] = z; }  // parallel clear of the score registers
    bop_best_score = 0; bop_best = 1; bop_round = 0; bop_idx = 0;
  }

  void start_gen(uint32_t base_line, int32_t stride_lines, int count, int distance) {
    gen.valid = true;
    gen.base = base_line;
    gen.stride = stride_lines;   // signed: descending streams add a negative step (it used to be masked to 8 bits unsigned)
    gen.next = (base_line + uint32_t(stride_lines * distance)) & pf::wmask(PF_LINE_BITS);
    gen.left = count;
    gen.pushed = 0;
  }

  void train(const PfAccess &a) {
    const uint32_t line = (a.paddr >> 6) & pf::wmask(PF_LINE_BITS);
    if (nllc > 0 && a.miss) {
      gen2.valid = true; gen2.base = line; gen2.next = (line + nllc_dist) & pf::wmask(PF_LINE_BITS); gen2.left = nllc;
    }
    if (PF_ALGO == 1) {  // next-line on misses
      if (a.miss) start_gen(line, 1, eff_degree(), dist);
      return;
    }
    if (PF_ALGO == 3) { bop_train(a, true); return; }
    if (PF_ALGO == 2 || PF_ALGO == 4) {  // stride: per-PC table, optionally plus one global entry (PF_HYBRID=1)
      const uint32_t pc_ = nopc ? 0u : a.pc;
      const uint32_t idx = pc_ & (stride_sets - 1), tag = (pc_ >> 6) & 0xF;
      int32_t sv = 0, gv = 0;
      bool alloc = false, galloc = false;
      const bool ok = stride_step(stride, idx, tag, line, &sv, &alloc);
      const bool gok = hybrid ? stride_step(gstride, 0, 0, line, &gv, &galloc) : false;
      if (ok) start_gen(line, sv, eff_degree(), dist);          // the per-PC prediction has priority
      else if (gok) start_gen(line, gv, eff_degree(), dist);    // otherwise the global stream detector
      if (PF_ALGO == 4) bop_train(a, alloc ? true : !(ok || gok));
      return;
    }
  }

  // One stride-table step for a line address: allocate on a tag miss, otherwise update stride / confidence.
  template <int N>
  bool stride_step(pf::Table<PfStrideE, N> &t, uint32_t idx, uint32_t tag, uint32_t line, int32_t *stride_out, bool *allocated) {
    PfStrideE e = t.rd(idx);
    if (!e.valid || e.tag != tag) {
      PfStrideE n; n.valid = 1; n.tag = tag; n.last = line; n.stride = 0; n.conf = 0;
      t.wr(idx, n);
      *allocated = true;
      return false;
    }
    const int64_t d = pf::sext((line - e.last) & pf::wmask(PF_LINE_BITS), PF_LINE_BITS);
    if (d == 0) return false;
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
    t.wr(idx, e);
    *stride_out = int32_t(pf::sext(e.stride, PF_STRIDE_BITS));
    return e.conf >= uint32_t(conf_thr) && *stride_out != 0;
  }

  PfAccess q_acc[PF_TRAIN_PORTS];   // registered training events (selected in the previous cycle)
  PfFill q_fill;                    // registered fill feedback
  uint32_t q_useful_n = 0;          // registered count of first uses this cycle
  struct Gen { bool valid = false; uint32_t next = 0, base = 0, pushed = 0; int32_t stride = 0; int left = 0; } gen;
  pf::Fifo<PfReqE, PF_QUEUE> reqq;
  pf::Table<PfStrideE, 64> stride;
  pf::Table<PfStrideE, 1> gstride;   // one global stream-detector entry (hybrid mode)
  int hybrid = 0;
  pf::Table<PfRrE, 256> rr;
  pf::Table<PfScoreE, 26> score;
  int bop_idx = 0, bop_round = 0, bop_best = 1, bop_best_score = 0, bop_d = 1, bop_scoremax = 31, bop_roundmax = 100, bop_badscore = 2;
  bool bop_on = true;
  int degree = 4, dist = 1, conf_thr = 2, xpage = 0, near_lines = 1000;
public:
  int mshr_reserve = 2;  // a probe allocates an MSHR only if more than this many stay free
private:
  int nopc = 0;
  int nllc = 0, nllc_dist = 1;
  struct Gen2 { bool valid = false; uint32_t next = 0, base = 0; int left = 0; } gen2;
  int throttle = 0, acc_hi = 70, acc_lo = 35, acc_win = 256;
  // accuracy feedback: counters of used / unused prefetched lines, evaluated every acc_win events
  uint32_t fb_useful = 0, fb_total = 0;
  int level = 3;  // degree = min(degree, 1 << level)
  int eff_degree() const { int d = 1 << level; return d < degree ? d : degree; }
  uint32_t stride_sets = 64;
};
