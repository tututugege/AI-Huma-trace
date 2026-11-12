#include "ExuShadow.h"
#include "ExuRef.h"
#include "Wires.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include "util.h"

namespace {
struct Shadow {
  ExuRef ref;
  Wires w;
  explicit Shadow(Exu &x) : ref(x.ctx) { ref.wires = &w; ref.init(); }
};
[[noreturn]] void fail(const char *name, const char *what) {
  fprintf(stderr, "[EXU_SHADOW] MISMATCH in %s: %s\n", name, what);
  exit(2);
}
template <class A, class B> bool same(const A &a, const B &b) { static_assert(sizeof(A) == sizeof(B)); return !memcmp(&a, &b, sizeof(A)); }
}  // namespace

void exu_shadow(Exu &x, ExuPhase p, const char *name) {
  static std::map<Exu *, Shadow *> all;
  Shadow *&sp = all[&x];
  if (!sp) sp = new Shadow(x);
  Shadow &s = *sp;
  Wires &rw = *x.wires;
  s.w.prf2exe = rw.prf2exe; s.w.dec_bcast = rw.dec_bcast; s.w.rob_bcast = rw.rob_bcast; s.w.lsu2exe = rw.lsu2exe;
  s.w.csr2exe = rw.csr2exe; s.w.csr_status = rw.csr_status;
  if (p == ExuPhase::comb_begin) {  // outputs start every cycle from the cleared wires
    s.w.exe2csr = rw.exe2csr; s.w.exe2prf = rw.exe2prf; s.w.exe2iss = rw.exe2iss; s.w.exe2lsu = rw.exe2lsu;
    s.w.exu2id = rw.exu2id; s.w.exu2rob = rw.exu2rob;
  }
  switch (p) {
#define X(n) case ExuPhase::n: s.ref.n(); break;
    EXU_PHASES(X)
#undef X
  }
  if (!same(rw.exe2csr, s.w.exe2csr)) fail(name, "exe2csr");
  if (!same(rw.exe2prf, s.w.exe2prf)) fail(name, "exe2prf");
  if (!same(rw.exe2iss, s.w.exe2iss)) fail(name, "exe2iss");
  if (!same(rw.exe2lsu, s.w.exe2lsu)) fail(name, "exe2lsu");
  if (!same(rw.exu2id, s.w.exu2id)) {
    const auto &a = rw.exu2id, &b = s.w.exu2id;
    fprintf(stderr, "t=%lld new: mis=%d pc=%llx rob=%d br=%d ftq=%d clr=%llx off=%d type=%d\n", (long long)sim_time, (int)a.mispred, (unsigned long long)a.redirect_pc, (int)a.redirect_rob_idx, (int)a.br_id, (int)a.ftq_idx, (unsigned long long)a.clear_mask, (int)a.ftq_off, (int)a.br_type);
    fprintf(stderr, "t=%lld ref: mis=%d pc=%llx rob=%d br=%d ftq=%d clr=%llx off=%d type=%d\n", (long long)sim_time, (int)b.mispred, (unsigned long long)b.redirect_pc, (int)b.redirect_rob_idx, (int)b.br_id, (int)b.ftq_idx, (unsigned long long)b.clear_mask, (int)b.ftq_off, (int)b.br_type);
    fail(name, "exu2id");
  }
  if (!same(rw.exu2rob, s.w.exu2rob)) {
    for (int i = 0; i < ISSUE_WIDTH; i++) {
      const auto &a = rw.exu2rob.entry[i], &b = s.w.exu2rob.entry[i];
      if (!same(a, b)) {
        const unsigned char *pa = (const unsigned char *)&a, *pb = (const unsigned char *)&b;
        size_t k = 0; while (k < sizeof(a) && pa[k] == pb[k]) k++;
        fprintf(stderr, "t=%lld port %d valid new=%d ref=%d first diff byte %zu of %zu (uop at %zu) rob new=%d ref=%d op %d %d br_mask %llx %llx\n", (long long)sim_time, i, (int)a.valid, (int)b.valid, k, sizeof(a), (size_t)((const char *)&a.uop - (const char *)&a), (int)a.uop.rob_idx, (int)b.uop.rob_idx, (int)a.uop.op, (int)b.uop.op, (unsigned long long)a.uop.br_mask, (unsigned long long)b.uop.br_mask);
      }
    }
    fail(name, "exu2rob");
  }
}
