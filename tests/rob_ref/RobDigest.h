// Canonical digest of a Rob (committed state, or the working copy when `working` is set): pointers/flags and every valid entry.
#pragma once
#include <cstdint>
#include <cstring>
template <class R>
uint64_t rob_digest(const R &r, bool working = false) {
  uint64_t h = 1469598103934665603ull;
  auto m = [&](uint64_t v) { h = (h ^ (v + 0x9e3779b97f4a7c15ull)) * 1099511628211ull; };
  m(working ? r.enq_ptr_1 : r.enq_ptr); m(working ? r.deq_ptr_1 : r.deq_ptr);
  m(working ? r.enq_flag_1 : r.enq_flag); m(working ? r.deq_flag_1 : r.deq_flag);
  for (int i = 0; i < ROB_BANK_NUM; i++)
    for (int j = 0; j < ROB_LINE_NUM; j++) {
      const auto &e = working ? r.entry_1[i][j] : r.entry[i][j];
      m(e.valid);
      if (e.valid) for (size_t k = 0; k < sizeof(e.uop); k++) m(reinterpret_cast<const uint8_t *>(&e.uop)[k]);
    }
  return h;
}
