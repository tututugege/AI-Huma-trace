#pragma once
// Alignment fingerprints (model side).  The RTL (rtl/pmu/fpr_*.sv, simulation only) prints the same lines, scripts/align_check.py compares them.
// Two event streams are fingerprinted, per bin of FPR cycles (commutative sum of a mixed value per event, so the order of events inside one cycle
// does not matter):
//   PT  a block written into the PTAB (base pc, predicted next pc, per-slot direction mask)
//   UP  an update-queue entry applied to the predictor tables (pc, direction, type, target, prediction metadata, 16-bit flag, hash of the TAGE indices and tags)
// Environment: FPR=<bin cycles> enables ("FPR PT bin=<k> n=<events> h=<hash>" for every non-empty bin, stderr);
//              FPR_DUMP=<from>:<to> also prints every event of that cycle window ("FPRRAW ...").
// The event cycle is the cycle of the RTL's free-running counter (0 in the first cycle after reset): the model's cycle counter is one ahead.
#include <cstdint>
#include "trace_win.h"
#include "env.h"
#include <cstdio>
#include <cstdlib>

namespace fpr {
inline uint32_t mix(uint32_t x) {
  x ^= x >> 16; x *= 0x7feb352dU; x ^= x >> 15; x *= 0x846ca68bU; x ^= x >> 16;
  return x;
}
struct Stream {
  const char *name;
  uint32_t cur_bin = 0xffffffffu, n = 0, h = 0;
  void flush() {
    if (n) std::fprintf(stderr, "FPR %s bin=%u n=%u h=%08x\n", name, cur_bin, n, h);
    n = 0; h = 0;
  }
  void add(uint32_t bin, uint32_t x) {
    if (bin != cur_bin) { flush(); cur_bin = bin; }
    h += x; n++;
  }
};
struct State {
  uint32_t bin = 0, from = 1, to = 0;
  bool on = false, dump = false;
  Stream pt{"PT"}, up{"UP"};
  uint64_t cycle = 0;   // set by the front end once per cycle (the model's cycle - 1)
  State() {
    const SimEnv &e = sim_env();
    on = e.fpr;
    if (!on) return;
    bin = e.fpr_bin;
    if (e.fpr_dump) { from = e.fpr_from; to = e.fpr_to; dump = true; }
  }
  ~State() { if (on) { pt.flush(); up.flush(); } }
};
inline State &st() { static State s; return s; }
inline bool on() { return st().on; }
inline bool in_dump() { const State &s = st(); return s.dump && s.cycle >= s.from && s.cycle <= s.to; }
inline uint64_t cycle() { return st().cycle; }

inline void set_cycle(uint64_t model_cycle) { st().cycle = model_cycle ? model_cycle - 1 : 0; trace_win::cycle = st().cycle; trace_win::active = st().dump; trace_win::from = st().from; trace_win::to = st().to; }

// the RTL testbench: its PMU cycle counter, 0 in the first cycle after reset
inline void pt(uint32_t base, uint32_t next, uint32_t dir) {
  State &s = st(); if (!s.on) return;
  const uint32_t c = static_cast<uint32_t>(s.cycle);
  s.pt.add(c / s.bin, mix(base + mix(next + mix(dir + 1u))));
  if (s.dump && c >= s.from && c <= s.to) std::fprintf(stderr, "FPRRAW PT cyc=%u base=%x next=%x dir=%04x\n", c, base, next, dir);
}
inline void up(uint32_t pc, uint32_t dir, uint32_t ty, uint32_t tgt, uint32_t pred, uint32_t pcpn, uint32_t rvc, uint64_t idxp = 0, uint32_t tagp = 0) {
  State &s = st(); if (!s.on) return;
  const uint32_t c = static_cast<uint32_t>(s.cycle);
  const uint32_t meta = dir | (ty << 1) | (pred << 4) | (pcpn << 5) | (rvc << 8);
  const uint32_t ix = mix(static_cast<uint32_t>(idxp) + mix(static_cast<uint32_t>(idxp >> 32) + mix(tagp + 1u)));   // TAGE indices (4 x 10 bit) and tags (4 x 8 bit) of the entry
  s.up.add(c / s.bin, mix(pc + mix(tgt + mix(meta + 1u + mix(ix)))));
  if (s.dump && c >= s.from && c <= s.to) std::fprintf(stderr, "FPRRAW UP cyc=%u pc=%x dir=%u ty=%u tgt=%x pred=%u pcpn=%u rvc=%u idx=%010llx tag=%08x\n", c, pc, dir, ty, tgt, pred, pcpn, rvc, static_cast<unsigned long long>(idxp), tagp);
}
}  // namespace fpr
