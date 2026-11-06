#pragma once
// Study-only instrumentation (compile with -DPF_STUDY): per-PC L1D load-miss profile and an "ideal L1D" switch.
// Not part of the hardware model; default builds do not contain it.
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <string>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct PfStudy {
  struct Pc { uint64_t n = 0; uint32_t last = 0; bool has = false; std::map<int32_t, uint64_t> d; uint64_t same = 0; };
  std::unordered_map<uint32_t, Pc> pcs;
  uint64_t total = 0, hits_seen = 0;
  uint32_t last_line = 0; bool has_last = false;
  uint64_t gd_le1 = 0, gd_le4 = 0, gd_le64 = 0, gd_far = 0;      // distance to the previous miss (any PC), in lines
  std::deque<uint32_t> recent; std::unordered_set<uint32_t> recent_set; uint64_t reuse_hits = 0;
  uint64_t seen_lines_n = 0; std::unordered_set<uint32_t> seen_lines;
  std::unordered_set<uint32_t> ideal_pcs;          // PF_IDEAL_PC=hex,hex,...: loads of these PCs always hit (per-PC upper bound)
  PfStudy() {
    if (const char *v = std::getenv("PF_IDEAL_PC")) {
      std::string t(v), cur;
      for (char c : t + ",") { if (c == ',') { if (!cur.empty()) ideal_pcs.insert(uint32_t(std::strtoul(cur.c_str(), nullptr, 16))); cur.clear(); } else cur += c; }
    }
  }
  bool ideal_pc(uint32_t pc) const { return ideal_pcs.count(pc) != 0; }
  void miss(uint32_t pc, uint32_t addr) {
    const uint32_t line = addr >> 6;
    total++;
    Pc &p = pcs[pc]; p.n++;
    if (p.has) { int32_t d = int32_t(line) - int32_t(p.last); if (d == 0) p.same++; else p.d[d]++; }
    p.last = line; p.has = true;
    if (has_last) {
      int64_t g = int64_t(line) - int64_t(last_line); if (g < 0) g = -g;
      if (g <= 1) gd_le1++; else if (g <= 4) gd_le4++; else if (g <= 64) gd_le64++; else gd_far++;
    }
    last_line = line; has_last = true;
    if (recent_set.count(line)) reuse_hits++;
    recent.push_back(line); recent_set.insert(line);
    if (recent.size() > 4096) { uint32_t o = recent.front(); recent.pop_front(); bool dup = false; for (auto x : recent) if (x == o) { dup = true; break; } if (!dup) recent_set.erase(o); }
    seen_lines.insert(line);
  }
  ~PfStudy() {
    if (!std::getenv("PF_STUDY_OUT")) return;
    std::fprintf(stderr, "[STUDY] load misses (first pass): %llu, distinct lines %zu, reuse within last 4096 misses %.1f%%\n",
                 (unsigned long long)total, seen_lines.size(), total ? 100.0 * reuse_hits / total : 0.0);
    const double g = double(gd_le1 + gd_le4 + gd_le64 + gd_far);
    std::fprintf(stderr, "[STUDY] distance to previous miss (any PC): <=1 line %.1f%%, <=4 %.1f%%, <=64 %.1f%%, farther %.1f%%\n",
                 100 * gd_le1 / g, 100 * gd_le4 / g, 100 * gd_le64 / g, 100 * gd_far / g);
    std::vector<std::pair<uint64_t, uint32_t>> v;
    for (auto &kv : pcs) v.push_back({kv.second.n, kv.first});
    std::sort(v.rbegin(), v.rend());
    uint64_t cum = 0;
    for (size_t i = 0; i < v.size() && i < 24; i++) {
      Pc &p = pcs[v[i].second]; cum += p.n;
      uint64_t cnt = 0; for (auto &kv : p.d) cnt += kv.second;
      uint64_t le1 = 0; for (auto &kv : p.d) if (kv.first >= -1 && kv.first <= 1) le1 += kv.second;
      std::vector<std::pair<uint64_t, int32_t>> dv; for (auto &kv : p.d) dv.push_back({kv.second, kv.first});
      std::sort(dv.rbegin(), dv.rend());
      std::fprintf(stderr, "[STUDY] pc %08x  %6.2f%% (cum %5.1f%%)  same-line %.0f%%  |d|<=1 %.0f%%  top deltas:", v[i].second,
                   100.0 * p.n / total, 100.0 * cum / total, 100.0 * p.same / p.n, cnt ? 100.0 * le1 / (cnt + p.same) : 0.0);
      for (size_t k = 0; k < dv.size() && k < 3; k++) std::fprintf(stderr, " %+d:%.0f%%", dv[k].second, 100.0 * dv[k].first / p.n);
      std::fprintf(stderr, "\n");
    }
    std::fprintf(stderr, "[STUDY] static load PCs that missed: %zu\n", pcs.size());
  }
};
inline PfStudy g_study;
