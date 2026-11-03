#include "DdrShadow.h"
#include "DdrDigest.h"
#include "SimDDRRef.h"
#include <cstdio>
#include <cstdlib>
#include <map>


namespace {
struct Shadow { sim_ddr::SimDDRRef ref; unsigned long long calls = 0; };
Shadow &get(sim_ddr::SimDDR *d) {
  static std::map<sim_ddr::SimDDR *, Shadow *> all;
  Shadow *&s = all[d];
  if (!s) { s = new Shadow; s->ref.init(); }
  return *s;
}
[[noreturn]] void fail(const char *what, int phase) {
  fprintf(stderr, "[DDR_SHADOW] MISMATCH %s (%s)\n", what, phase ? "seq" : "comb_outputs");
  exit(2);
}
}  // namespace

void ddr_shadow_begin(sim_ddr::SimDDR *d, sim_ddr::SimDDR_IO_t *io) { get(d).ref.io = *io; }

void ddr_shadow_end(sim_ddr::SimDDR *d, sim_ddr::SimDDR_IO_t *io, int phase) {
  Shadow &s = get(d);
  s.calls++;
  if (phase == 0) s.ref.comb_outputs(); else s.ref.seq();
  const auto &a = *io, &b = s.ref.io;
  if (phase == 0) {
    if (a.ar.arready != b.ar.arready) fail("arready", phase);
    if (a.r.rvalid != b.r.rvalid || a.r.rid != b.r.rid || a.r.rlast != b.r.rlast || a.r.rresp != b.r.rresp) fail("R channel", phase);
    if (a.aw.awready != b.aw.awready) fail("awready", phase);
    if (a.w.wready != b.w.wready) fail("wready", phase);
    if (a.b.bvalid != b.b.bvalid || a.b.bid != b.b.bid || a.b.bresp != b.b.bresp) fail("B channel", phase);
  }
  if (sim_ddr::DdrShadowAccess::digest(*d) != s.ref.digest()) fail("state digest", phase);
}
