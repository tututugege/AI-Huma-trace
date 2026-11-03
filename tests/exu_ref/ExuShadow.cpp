#include "ExuShadow.h"
#include "ExuDigest.h"
#include "ExuRef.h"
#include "Wires.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>

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
  if (!same(rw.exu2id, s.w.exu2id)) fail(name, "exu2id");
  if (!same(rw.exu2rob, s.w.exu2rob)) fail(name, "exu2rob");
  const bool committed = p == ExuPhase::seq;
  if (ExuDigestAccess::digest(x, !committed) != ExuDigestAccess::digest(s.ref, !committed)) fail(name, committed ? "state digest (committed)" : "state digest (working)");
}
