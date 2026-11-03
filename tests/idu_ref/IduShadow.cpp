#include "IduShadow.h"
#include "IduRef.h"
#include "Wires.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <vector>

namespace {
struct Shadow {
  IduRef ref;
  Wires w;
  explicit Shadow(Idu &x) : ref(x.ctx, x.max_br_per_cycle) { ref.wires = &w; ref.init(); }
};
[[noreturn]] void fail(const char *name, const char *what) {
  fprintf(stderr, "[IDU_SHADOW] MISMATCH in %s: %s\n", name, what);
  exit(2);
}
template <class A, class B> bool same(const A &a, const B &b) { static_assert(sizeof(A) == sizeof(B)); return !memcmp(&a, &b, sizeof(A)); }
template <class I> uint64_t digest(const I &d, bool working) {
  uint64_t h = 1469598103934665603ull;
  auto m = [&](uint64_t v) { h = (h ^ (v + 0x9e3779b97f4a7c15ull)) * 1099511628211ull; };
  m(working ? d.now_br_mask_1 : d.now_br_mask);
  for (int i = 0; i < MAX_BR_NUM; i++) { m(working ? d.br_mask_cp_1[i] : d.br_mask_cp[i]); m(working ? d.tag_vec_1[i] : d.tag_vec[i]); }
  const auto &l = working ? d.br_latch_1 : d.br_latch;
  m(l.mispred); m(l.br_id); m(l.redirect_rob_idx); m(l.clear_mask); m(l.redirect_pc); m(l.ftq_idx);
  return h;
}
}  // namespace

void idu_shadow(Idu &x, IduPhase p, const char *name) {
  static std::map<Idu *, Shadow *> all;
  Shadow *&sp = all[&x];
  if (!sp) sp = new Shadow(x);
  Shadow &s = *sp;
  Wires &rw = *x.wires;
  s.w.pre_issue = rw.pre_issue; s.w.exu2id = rw.exu2id; s.w.rob_bcast = rw.rob_bcast; s.w.ren2dec = rw.ren2dec;
  if (p == IduPhase::comb_begin) { s.w.dec2ren = rw.dec2ren; s.w.dec_bcast = rw.dec_bcast; s.w.idu_consume = rw.idu_consume; }
  // the decoder writes shared uop-pool slots; remember what the real Idu wrote before the reference overwrites them
  std::vector<std::pair<uint16_t, UopS>> pool;
  if (p == IduPhase::comb_decode)
    for (int i = 0; i < DECODE_WIDTH; i++)
      if (rw.dec2ren.valid[i]) pool.push_back({rw.dec2ren.uop[i].id, uop_pool[rw.dec2ren.uop[i].id]});
  switch (p) {
#define X(n) case IduPhase::n: s.ref.n(); break;
    IDU_PHASES(X)
#undef X
  }
  for (auto &e : pool) if (memcmp(&e.second, &uop_pool[e.first], sizeof(UopS))) fail(name, "uop pool entry of a decoded instruction");
  if (!same(rw.dec2ren, s.w.dec2ren)) fail(name, "dec2ren");
  if (!same(rw.dec_bcast, s.w.dec_bcast)) fail(name, "dec_bcast");
  if (!same(rw.idu_consume, s.w.idu_consume)) fail(name, "idu_consume");
  const bool committed = p == IduPhase::seq;
  if (digest(x, !committed) != digest(s.ref, !committed)) fail(name, committed ? "state digest (committed)" : "state digest (working)");
}
