#include "DisShadow.h"
#include "DisDigest.h"
#include "DispatchRef.h"
#include "Wires.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <vector>

namespace {
struct Shadow {
  DispatchRef ref;
  Wires w;
  explicit Shadow(Dispatch &d) : ref(d.ctx) { ref.wires = &w; ref.init(); }
};
[[noreturn]] void fail(const char *name, const char *what) {
  fprintf(stderr, "[DIS_SHADOW] MISMATCH in %s: %s\n", name, what);
  exit(2);
}
template <class A, class B> bool same(const A &a, const B &b) { static_assert(sizeof(A) == sizeof(B)); return !memcmp(&a, &b, sizeof(A)); }
}  // namespace

// Directed scenarios for states no program reaches (an older slot writing x0, a full issue queue): identical hand-built
// instructions go into the real and the reference Dispatch, alloc/wake/dispatch run on both, outputs and digests must match.
namespace {
void run_directed() {
  int bad = 0;
  for (int scenario = 0; scenario < 2; scenario++) {
    Wires wa, wb;
    Dispatch a(nullptr); DispatchRef b(nullptr);
    a.wires = &wa; b.wires = &wb; a.init(); b.init();
    const uint16_t id0 = uop_alloc(), id1 = uop_alloc();
    auto setup = [&](auto &d, Wires &w) {
      for (int i = 0; i < 2; i++) { d.inst_r[i] = {}; d.inst_r[i].id = i ? id1 : id0; d.inst_valid[i] = true; d.inst_r[i]->type = ADD; }
      d.inst_r[0].dest_en = true; d.inst_r[0].dest_preg = 5; d.inst_r[0]->dest_areg = 0;       // older slot writes x0's preg
      d.inst_r[1]->src1_en = true; d.inst_r[1]->src1_preg = 5;
      w.rob2dis.ready = true;
      for (int q = 0; q < IQ_NUM; q++) w.iss2dis.ready_num[q] = scenario == 1 ? 0 : 8;       // scenario 1: every issue queue full
    };
    setup(a, wa); setup(b, wb);
    for (auto ph : {0, 1, 2}) {
      if (ph == 0) { a.comb_alloc(); b.comb_alloc(); } else if (ph == 1) { a.comb_wake(); b.comb_wake(); } else { a.comb_dispatch(); b.comb_dispatch(); }
    }
    const bool ok = same(wa.dis2iss, wb.dis2iss) && same(wa.dis2rob, wb.dis2rob) &&
                    DisDigestAccess::digest(a, true) == DisDigestAccess::digest(b, true);
    printf("[DIS_DIRECTED] %-44s %s\n", scenario == 0 ? "older slot with dest x0 does not keep src busy" : "full issue queue stops the group", ok ? "same" : "MISMATCH");
    bad += !ok;
  }
  if (bad) exit(2);
}
}  // namespace

void dis_shadow(Dispatch &d, DisPhase p, const char *name) {
  static bool directed_done = false;
  if (!directed_done) { directed_done = true; if (getenv("DIS_SHADOW_DIRECTED")) run_directed(); }
  static std::map<Dispatch *, Shadow *> all;
  Shadow *&sp = all[&d];
  if (!sp) sp = new Shadow(d);
  Shadow &s = *sp;
  Wires &rw = *d.wires;
  s.w.lsu2dis = rw.lsu2dis; s.w.ren2dis = rw.ren2dis; s.w.iss2dis = rw.iss2dis; s.w.rob2dis = rw.rob2dis;
  s.w.prf_awake = rw.prf_awake; s.w.iss_awake = rw.iss_awake; s.w.rob_bcast = rw.rob_bcast; s.w.dec_bcast = rw.dec_bcast;
  if (p == DisPhase::comb_begin) { s.w.dis2ren = rw.dis2ren; s.w.dis2iss = rw.dis2iss; s.w.dis2rob = rw.dis2rob; s.w.dis2lsu = rw.dis2lsu; }
  // decompose_inst writes shared uop-pool slots; remember what the real Dispatch wrote before the reference overwrites them
  std::vector<std::pair<uint16_t, UopS>> pool;
  if (p == DisPhase::comb_dispatch || p == DisPhase::comb_fire)
    for (int i = 0; i < IQ_NUM; i++)
      for (int k = 0; k < MAX_IQ_DISPATCH_WIDTH; k++)
        if (rw.dis2iss.req[i][k].valid) pool.push_back({rw.dis2iss.req[i][k].uop.id, uop_pool[rw.dis2iss.req[i][k].uop.id]});
  switch (p) {
#define X(n) case DisPhase::n: s.ref.n(); break;
    DIS_PHASES(X)
#undef X
  }
  for (auto &e : pool) if (memcmp(&e.second, &uop_pool[e.first], sizeof(UopS))) fail(name, "uop pool entry of an issued uop");
  if (!same(rw.dis2ren, s.w.dis2ren)) fail(name, "dis2ren");
  if (!same(rw.dis2iss, s.w.dis2iss)) fail(name, "dis2iss");
  if (!same(rw.dis2rob, s.w.dis2rob)) fail(name, "dis2rob");
  if (!same(rw.dis2lsu, s.w.dis2lsu)) fail(name, "dis2lsu");
  const bool committed = p == DisPhase::seq;
  if (DisDigestAccess::digest(d, !committed) != DisDigestAccess::digest(s.ref, !committed)) fail(name, committed ? "state digest (committed)" : "state digest (working)");
}
