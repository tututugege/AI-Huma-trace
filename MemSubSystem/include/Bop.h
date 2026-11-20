#pragma once
// EXP-048 Best-Offset Prefetcher (Michaud, HPCA 2016), behavioural model shared by the L1D (DataPrefetcher, PF_KIND=1) and
// the LLC (AXI_LLC, LLC_BOP=1). Model experiment: plain C++ state, not the circuit style of h1.
//
// Learning: every trigger access X (a demand miss, or the first demand use of a prefetched line) tests one offset d of the
// list: if X - d is in the recent-requests (RR) table, d scores. A round tests every offset once; a phase ends after
// ROUND_MAX rounds or when an offset reaches SCORE_MAX; the best offset becomes D, prefetching is off when its score is
// <= BAD_SCORE. RR is written when a line fills: a prefetched line Y inserts Y - D (the base it was issued for), a demand
// fill inserts its own line while prefetching is off. Each trigger issues X + D (and X + 2D ... up to DEG) in the same
// 4 KiB page. Parameters from the environment, <prefix>_<name>.
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

class Bop {
 public:
  explicit Bop(const char *prefix) : prefix_(prefix) {
    round_max_ = param("ROUNDS", 100);
    score_max_ = param("SCORE_MAX", 31);
    bad_score_ = param("BAD", 1);
    deg_ = param("DEG", 1);
    rr_bits_ = param("RR_BITS", 8);
    const int maxoff = param("MAXOFF", 63);   // offsets stay inside a 4 KiB page (64 lines)
    const bool neg = param("NEG", 0) != 0;
    for (int d = 1; d <= maxoff; d++) {
      int x = d;
      for (int p : {2, 3, 5}) while (x % p == 0) x /= p;
      if (x != 1) continue;
      offs_.push_back(d);
      if (neg) offs_.push_back(-d);
    }
    score_.assign(offs_.size(), 0);
    rr_.assign(size_t(1) << rr_bits_, UINT32_MAX);
    best_ = 1;   // starts with next-line, on
    on_ = true;
  }

  // a trigger access on line X (cache-line number): learn, return the lines to prefetch (in the same 4 KiB page)
  int trigger(uint32_t x, uint32_t out[], int max_out) {
    learn(x);
    int n = 0;
    if (!on_) return 0;
    for (int k = 1; k <= deg_ && n < max_out; k++) {
      const uint32_t y = uint32_t(int64_t(x) + int64_t(best_) * k);
      if ((y >> 6) != (x >> 6)) break;
      out[n++] = y;
    }
    stats_issued += n;
    return n;
  }
  // a line filled the cache; was_prefetch: brought in by this prefetcher
  void fill(uint32_t y, bool was_prefetch) {
    if (was_prefetch) rr_insert(uint32_t(int64_t(y) - best_));
    else if (!on_) rr_insert(y);
  }

  bool on() const { return on_; }
  int best() const { return best_; }
  uint64_t stats_issued = 0, stats_phases = 0, stats_off_phases = 0, stats_sent = 0, stats_useful = 0;   // sent / useful: kept by the user
  void print(const char *name) const {
    std::printf("[BOP] %s: best offset %d (%s), phases %llu (off %llu), candidates %llu, sent %llu, useful %llu, offsets %zu\n", name, best_,
                on_ ? "on" : "off", (unsigned long long)stats_phases, (unsigned long long)stats_off_phases, (unsigned long long)stats_issued,
                (unsigned long long)stats_sent, (unsigned long long)stats_useful, offs_.size());
  }

 private:
  int param(const char *n, int d) const {
    const char *v = std::getenv((prefix_ + "_" + n).c_str());
    return v ? std::atoi(v) : d;
  }
  uint32_t rr_idx(uint32_t line) const { return (line ^ (line >> rr_bits_)) & ((1u << rr_bits_) - 1); }
  void rr_insert(uint32_t line) { rr_[rr_idx(line)] = line; }
  bool rr_hit(uint32_t line) const { return rr_[rr_idx(line)] == line; }

  void learn(uint32_t x) {
    const int d = offs_[test_];
    if (rr_hit(uint32_t(int64_t(x) - d))) {
      if (++score_[test_] >= score_max_) { end_phase(); return; }
    }
    if (++test_ == int(offs_.size())) {
      test_ = 0;
      if (++round_ >= round_max_) end_phase();
    }
  }
  void end_phase() {
    int bi = 0;
    for (size_t i = 1; i < score_.size(); i++) if (score_[i] > score_[bi]) bi = int(i);
    on_ = score_[bi] > bad_score_;
    if (on_) best_ = offs_[bi];
    stats_phases++;
    stats_off_phases += !on_;
    std::fill(score_.begin(), score_.end(), 0);
    round_ = test_ = 0;
  }

  std::string prefix_;
  std::vector<int> offs_, score_;
  std::vector<uint32_t> rr_;
  int round_max_, score_max_, bad_score_, deg_, rr_bits_;
  int test_ = 0, round_ = 0, best_ = 1;
  bool on_ = true;
};

// the instances, for the end-of-run report (Pmu.cpp)
inline Bop *g_bop_l1 = nullptr;
inline Bop *g_bop_llc = nullptr;
