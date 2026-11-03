// Canonical digest of a Dispatch: registered state (or the working copy) plus the per-cycle intermediates comb_dispatch/comb_fire pass along.
#pragma once
#include <cstdint>
struct DisDigestAccess {
  template <class D>
  static uint64_t digest(const D &d, bool working) {
    uint64_t h = 1469598103934665603ull;
    auto m = [&](uint64_t v) { h = (h ^ (v + 0x9e3779b97f4a7c15ull)) * 1099511628211ull; };
    auto bytes = [&](const void *p, size_t n) { for (size_t i = 0; i < n; i++) m(static_cast<const uint8_t *>(p)[i]); };
    for (int i = 0; i < DECODE_WIDTH; i++) {
      bytes(working ? &d.inst_r_1[i] : &d.inst_r[i], sizeof(Uop));
      m(working ? d.inst_valid_1[i] : d.inst_valid[i]);
    }
    for (int i = 0; i < PRF_NUM; i++) m(working ? d.busy_table_1[i] : d.busy_table[i]);
    if (working) {
      for (int i = 0; i < DECODE_WIDTH; i++) {
        bytes(&d.inst_alloc[i], sizeof(Uop)); m(d.dispatch_success_flags[i]); m(d.dispatch_cache[i].count);
        for (int k = 0; k < d.dispatch_cache[i].count; k++) m(d.dispatch_cache[i].iq_ids[k]);
      }
      for (int k = 0; k < MAX_STQ_DISPATCH_WIDTH; k++) m(uint64_t(int64_t(d.stq_port_owner[k])));
      for (int k = 0; k < MAX_LDQ_DISPATCH_WIDTH; k++) m(uint64_t(int64_t(d.ldq_port_owner[k])));
    }
    return h;
  }
};
