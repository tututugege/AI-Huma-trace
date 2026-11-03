#include "CsrShadow.h"
#include "CsrRef.h"
#include "Wires.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>

namespace {
struct Shadow {
  CsrRef ref;
  Wires w;
  bool adopted = false;
  explicit Shadow(Csr &x) { ref.wires = &w; ref.init(); }
};
[[noreturn]] void fail(const char *name, const char *what) {
  fprintf(stderr, "[CSR_SHADOW] MISMATCH in %s: %s\n", name, what);
  exit(2);
}
template <class A, class B> bool same(const A &a, const B &b) { static_assert(sizeof(A) == sizeof(B)); return !memcmp(&a, &b, sizeof(A)); }
template <class C> bool state_equal(const Csr &x, const C &r, bool working) {
  if (working)
    return !memcmp(x.CSR_RegFile_1, r.CSR_RegFile_1, sizeof(x.CSR_RegFile_1)) && x.privilege_1 == r.privilege_1 && x.csr_idx_1 == r.csr_idx_1 &&
           x.csr_wdata_1 == r.csr_wdata_1 && x.csr_wcmd_1 == r.csr_wcmd_1 && x.csr_we_1 == r.csr_we_1;
  return !memcmp(x.CSR_RegFile, r.CSR_RegFile, sizeof(x.CSR_RegFile)) && x.privilege == r.privilege && x.csr_idx == r.csr_idx &&
         x.csr_wdata == r.csr_wdata && x.csr_wcmd == r.csr_wcmd && x.csr_we == r.csr_we;
}
void copy_state(const Csr &x, CsrRef &r) {
  memcpy(r.CSR_RegFile, x.CSR_RegFile, sizeof(x.CSR_RegFile)); memcpy(r.CSR_RegFile_1, x.CSR_RegFile_1, sizeof(x.CSR_RegFile_1));
  r.privilege = x.privilege; r.privilege_1 = x.privilege_1;
  r.csr_idx = x.csr_idx; r.csr_idx_1 = x.csr_idx_1; r.csr_wdata = x.csr_wdata; r.csr_wdata_1 = x.csr_wdata_1;
  r.csr_wcmd = x.csr_wcmd; r.csr_wcmd_1 = x.csr_wcmd_1; r.csr_we = x.csr_we; r.csr_we_1 = x.csr_we_1;
}
}  // namespace

void csr_shadow(Csr &x, CsrPhase p, const char *name) {
  static std::map<Csr *, Shadow *> all;
  Shadow *&sp = all[&x];
  if (!sp) sp = new Shadow(x);
  Shadow &s = *sp;
  Wires &rw = *x.wires;
  s.w.exe2csr = rw.exe2csr; s.w.rob2csr = rw.rob2csr; s.w.rob_bcast = rw.rob_bcast; s.w.csr_interrupt_inject_io = rw.csr_interrupt_inject_io;
  // the machine state is also loaded from outside (checkpoint / difftest init): adopt it once, then it must stay in sync
  if (!s.adopted) { copy_state(x, s.ref); s.adopted = true; }
  switch (p) {
#define X(n) case CsrPhase::n: s.ref.n(); break;
    CSR_PHASES(X)
#undef X
  }
  if (!same(rw.csr_status, s.w.csr_status)) fail(name, "csr_status");
  if (!same(rw.csr2exe, s.w.csr2exe)) fail(name, "csr2exe");
  if (!same(rw.csr2rob, s.w.csr2rob)) fail(name, "csr2rob");
  if (!same(rw.csr2front, s.w.csr2front)) fail(name, "csr2front");
  const bool committed = p == CsrPhase::seq;
  if (!state_equal(x, s.ref, !committed)) fail(name, committed ? "state (committed)" : "state (working)");
}
