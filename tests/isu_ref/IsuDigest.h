// Canonical digest of an Isu: every issue queue (committed or working entries and counts) and the delayed-wakeup state.
#pragma once
#include <cstdint>
struct IsuDigestAccess {
  template <class I>
  static void poke_div(I &x, bool working, int idx, int countdown, int preg) {
    auto &s = (working ? x.div_wake_slots_1 : x.div_wake_slots)[idx];
    s.valid = true; s.countdown = countdown; s.dest_preg = preg; s.br_mask = 0;
  }
  template <class I>
  static uint64_t digest(const I &x, bool working) {
    uint64_t h = 1469598103934665603ull;
    auto m = [&](uint64_t v) { h = (h ^ (v + 0x9e3779b97f4a7c15ull)) * 1099511628211ull; };
    auto bytes = [&](const void *p, size_t n) { for (size_t i = 0; i < n; i++) m(static_cast<const uint8_t *>(p)[i]); };
    for (const auto &q : x.iqs) {
      const auto &e = working ? q.entry_1 : q.entry;
      m(working ? q.count_1 : q.count);
      for (size_t i = 0; i < e.size(); i++) { m(e[i].valid); if (e[i].valid) bytes(&e[i].uop, sizeof(Uop)); }
    }
    auto wk = [&](const auto &s) {
      m(s.valid);
      if (s.valid) { m(s.dest_preg); m(s.br_mask); if constexpr (requires { s.countdown; }) m(uint64_t(int64_t(s.countdown))); }
    };
    for (const auto &stage : (working ? x.mul_wake_pipe_1 : x.mul_wake_pipe)) for (const auto &s : stage) wk(s);
    for (const auto &s : (working ? x.div_wake_slots_1 : x.div_wake_slots)) wk(s);
    for (const auto &s : (working ? x.fp_wake_slots_1 : x.fp_wake_slots)) wk(s);
    return h;
  }
};
