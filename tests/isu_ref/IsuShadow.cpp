#include "IsuShadow.h"
#include "IsuDigest.h"
#include "IsuRef.h"
#include "Wires.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>

namespace {
struct Shadow {
  IsuRef ref;
  Wires w;
  explicit Shadow(Isu &x) : ref(nullptr) { ref.wires = &w; ref.init(); (void)x; }
};
[[noreturn]] void fail(const char *name, const char *what) {
  fprintf(stderr, "[ISU_SHADOW] MISMATCH in %s: %s\n", name, what);
  exit(2);
}
template <class A, class B> bool same(const A &a, const B &b) { static_assert(sizeof(A) == sizeof(B)); return !memcmp(&a, &b, sizeof(A)); }
}  // namespace

// Directed scenarios for wakeup-state corner cases no program reaches: identical hand-built state goes into the real and the
// reference Isu, one phase runs on both, digests must match (ISU_SHADOW_DIRECTED=1).
namespace {
void run_directed() {
  int bad = 0;
  for (int scenario = 0; scenario < 2; scenario++) {
    Wires wa, wb;
    Isu a(nullptr); IsuRef b(nullptr);
    a.wires = &wa; b.wires = &wb; a.init(); b.init();
    if (scenario == 0) {  // a DIV wakeup pending when the pipeline is flushed must vanish
      IsuDigestAccess::poke_div(a, false, 0, 3, 7); IsuDigestAccess::poke_div(a, true, 0, 3, 7);
      IsuDigestAccess::poke_div(b, false, 0, 3, 7); IsuDigestAccess::poke_div(b, true, 0, 3, 7);
      wa.rob_bcast.flush = wb.rob_bcast.flush = true;
      a.comb_flush(); b.comb_flush();
    } else {  // a DIV issued without a destination register must not start a delayed wakeup
      for (Wires *w : {&wa, &wb}) { w->iss2prf.iss_entry[0].valid = true; w->iss2prf.iss_entry[0].uop.op = UOP_DIV; w->iss2prf.iss_entry[0].uop.dest_en = false; }
      a.comb_calc_latency_next(); b.comb_calc_latency_next();
    }
    const bool ok = IsuDigestAccess::digest(a, true) == IsuDigestAccess::digest(b, true);
    printf("[ISU_DIRECTED] %-48s %s\n", scenario == 0 ? "flush clears a pending DIV wakeup" : "DIV without destination starts no wakeup", ok ? "same" : "MISMATCH");
    bad += !ok;
  }
  if (bad) exit(2);
}
}  // namespace

void isu_shadow(Isu &x, IsuPhase p, const char *name) {
  static bool directed_done = false;
  if (!directed_done) { directed_done = true; if (getenv("ISU_SHADOW_DIRECTED")) run_directed(); }
  static std::map<Isu *, Shadow *> all;
  Shadow *&sp = all[&x];
  if (!sp) sp = new Shadow(x);
  Shadow &s = *sp;
  Wires &rw = *x.wires;
  s.w.dis2iss = rw.dis2iss; s.w.prf_awake = rw.prf_awake; s.w.exe2iss = rw.exe2iss; s.w.rob_bcast = rw.rob_bcast; s.w.dec_bcast = rw.dec_bcast;
  if (p == IsuPhase::comb_begin) { s.w.iss2dis = rw.iss2dis; s.w.iss2prf = rw.iss2prf; s.w.iss_awake = rw.iss_awake; }
  switch (p) {
#define X(n) case IsuPhase::n: s.ref.n(); break;
    ISU_PHASES(X)
#undef X
  }
  if (!same(rw.iss2dis, s.w.iss2dis)) fail(name, "iss2dis");
  if (!same(rw.iss2prf, s.w.iss2prf)) fail(name, "iss2prf");
  if (!same(rw.iss_awake, s.w.iss_awake)) fail(name, "iss_awake");
  const bool committed = p == IsuPhase::seq;
  if (IsuDigestAccess::digest(x, !committed) != IsuDigestAccess::digest(s.ref, !committed)) fail(name, committed ? "state digest (committed)" : "state digest (working)");
}
