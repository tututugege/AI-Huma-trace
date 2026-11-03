// Canonical digest of an Exu: FU input slots (registered and working) and the per-FU request/response buses.
#pragma once
#include <cstdint>
struct ExuDigestAccess {
  template <class E>
  static uint64_t digest(const E &x, bool working) {
    uint64_t h = 1469598103934665603ull;
    auto m = [&](uint64_t v) { h = (h ^ (v + 0x9e3779b97f4a7c15ull)) * 1099511628211ull; };
    auto bytes = [&](const void *p, size_t n) { for (size_t i = 0; i < n; i++) m(static_cast<const uint8_t *>(p)[i]); };
    for (int i = 0; i < TOTAL_FU_COUNT; i++) {
      const auto &s = working ? x.fu_inst_r_1[i] : x.fu_inst_r[i];
      m(s.valid);
      if (s.valid) bytes(&s.uop, sizeof(Uop));
      if (working) {
        const auto &a = x.fu2exu_io.entry[i];
        m(a.ready); m(a.complete);
        if (a.complete) bytes(&a.inst, sizeof(Uop));
        const auto &b = x.exu2fu_io.entry[i];
        m(b.en); m(b.consume); m(b.flush); m(b.flush_mask); m(b.clear_mask);
        if (b.en) bytes(&b.inst, sizeof(Uop));
      }
    }
    return h;
  }
};
