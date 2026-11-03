#include "RenShadow.h"
#include "RenRef.h"
#include "Wires.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <vector>

namespace {
struct Shadow {
  RenRef ref;
  Wires w;
  explicit Shadow(Ren &x) : ref(x.ctx) { ref.wires = &w; ref.init(); }
};
[[noreturn]] void fail(const char *name, const char *what) {
  fprintf(stderr, "[REN_SHADOW] MISMATCH in %s: %s\n", name, what);
  exit(2);
}
template <class A, class B> bool same(const A &a, const B &b) { static_assert(sizeof(A) == sizeof(B)); return !memcmp(&a, &b, sizeof(A)); }
// working copies (*_1) when `working`, else the committed registers
template <class R> bool state_equal(const Ren &x, const R &r, bool working) {
  auto eq = [&](const void *a, const void *b, size_t n) { return !memcmp(a, b, n); };
#define CMP(f) (working ? eq(&x.f##_1, &r.f##_1, sizeof(x.f##_1)) : eq(&x.f, &r.f, sizeof(x.f)))
  return CMP(inst_r) && CMP(inst_valid) && CMP(arch_RAT) && CMP(spec_RAT) && CMP(RAT_checkpoint) && CMP(free_list) &&
         CMP(free_head) && CMP(free_head_commit) && CMP(free_tail) && CMP(alloc_checkpoint_head);
#undef CMP
}
}  // namespace

void ren_shadow(Ren &x, RenPhase p, const char *name) {
  static std::map<Ren *, Shadow *> all;
  Shadow *&sp = all[&x];
  if (!sp) sp = new Shadow(x);
  Shadow &s = *sp;
  Wires &rw = *x.wires;
  s.w.dec2ren = rw.dec2ren; s.w.dis2ren = rw.dis2ren; s.w.rob_commit = rw.rob_commit; s.w.rob_bcast = rw.rob_bcast; s.w.dec_bcast = rw.dec_bcast;
  // the renamer writes shared uop-pool slots; remember what the real Ren wrote before the reference overwrites them
  std::vector<std::pair<uint16_t, UopS>> pool;
  if (p == RenPhase::comb_rename)
    for (int i = 0; i < DECODE_WIDTH; i++)
      if (rw.ren2dis.valid[i]) pool.push_back({rw.ren2dis.uop[i].id, uop_pool[rw.ren2dis.uop[i].id]});
  switch (p) {
#define X(n) case RenPhase::n: s.ref.n(); break;
    REN_PHASES(X)
#undef X
  }
  for (auto &e : pool) if (memcmp(&e.second, &uop_pool[e.first], sizeof(UopS))) fail(name, "uop pool entry of a renamed instruction");
  if (!same(rw.ren2dec, s.w.ren2dec)) fail(name, "ren2dec");
  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (rw.ren2dis.valid[i] != s.w.ren2dis.valid[i]) fail(name, "ren2dis.valid");
    if (p == RenPhase::comb_rename && rw.ren2dis.valid[i] && !same(rw.ren2dis.uop[i], s.w.ren2dis.uop[i])) fail(name, "ren2dis.uop");
  }
  const bool committed = p == RenPhase::seq;
  if (!state_equal(x, s.ref, !committed)) fail(name, committed ? "state (committed)" : "state (working)");
}
