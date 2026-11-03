#include "PreShadow.h"
#include "PreIduQueueRef.h"
#include "Wires.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>

namespace {
[[noreturn]] void fail(const char *name, const char *what) {
  fprintf(stderr, "[PRE_SHADOW] MISMATCH in %s: %s\n", name, what);
  exit(2);
}
template <class A, class B> bool same(const A &a, const B &b) { static_assert(sizeof(A) == sizeof(B)); return !memcmp(&a, &b, sizeof(A)); }
bool same_entry(InstructionBufferEntry a, InstructionBufferEntry b) { a.uop_id = b.uop_id = 0; return a.valid == b.valid && a.inst == b.inst && a.pc == b.pc && a.page_fault_inst == b.page_fault_inst && a.ftq_idx == b.ftq_idx && a.ftq_offset == b.ftq_offset && a.ftq_is_last == b.ftq_is_last; }
bool same_ibuf(const InstructionBuffer &a, const InstructionBuffer &b) {
  if (a.count() != b.count()) return false;
  for (int i = 0; i < a.count(); i++) if (!same_entry(a.peek(i), b.peek(i))) return false;
  return true;
}
}  // namespace

struct PreShadow {
  PreIduQueueRef ref;
  Wires w;
  static bool state_equal(const PreIduQueue &x, const PreIduQueueRef &r, bool working) {
    const InstructionBuffer &xi = working ? x.ibuf_1 : x.ibuf, &ri = working ? r.ibuf_1 : r.ibuf;
    if (!same_ibuf(xi, ri)) return false;
    if (working) {
      if (x.ftq_head_1 != r.ftq_head_1 || x.ftq_tail_1 != r.ftq_tail_1 || x.ftq_count_1 != r.ftq_count_1) return false;
      return !memcmp(x.ftq_valid_1, r.ftq_valid_1, sizeof(x.ftq_valid_1));
    }
    if (x.ftq_head != r.ftq_head || x.ftq_tail != r.ftq_tail || x.ftq_count != r.ftq_count) return false;
    if (memcmp(x.ftq_valid, r.ftq_valid, sizeof(x.ftq_valid))) return false;
    // entry payloads only matter while valid
    for (int i = 0; i < FTQ_SIZE; i++) {
      if (!x.ftq_valid[i]) continue;
      if (!same(x.ftq_lookup_entries[i], r.ftq_lookup_entries[i]) || !same(x.ftq_train_meta_fifo[i], r.ftq_train_meta_fifo[i])) return false;
    }
    return true;
  }
  explicit PreShadow(PreIduQueue &x) : ref(x.ctx) { ref.wires = &w; w.idu_br_latch = x.wires->idu_br_latch; ref.init(); }
};


void pre_shadow(PreIduQueue &x, PrePhase p, const char *name) {
  static std::map<PreIduQueue *, PreShadow *> all;
  PreShadow *&sp = all[&x];
  if (!sp) sp = new PreShadow(x);
  PreShadow &s = *sp;
  Wires &rw = *x.wires;
  s.w.in = rw.in; s.w.rob_bcast = rw.rob_bcast; s.w.idu_consume = rw.idu_consume; s.w.rob_commit = rw.rob_commit;
  s.w.ftq_prf_pc_req = rw.ftq_prf_pc_req; s.w.ftq_rob_pc_req = rw.ftq_rob_pc_req;
  switch (p) {
#define X(n) case PrePhase::n: s.ref.n(); break;
    PRE_PHASES(X)
#undef X
  }
  for (int i = 0; i < DECODE_WIDTH; i++) if (!same_entry(rw.pre_issue.entries[i], s.w.pre_issue.entries[i])) fail(name, "pre_issue");
  if (!same(rw.pre2front, s.w.pre2front)) fail(name, "pre2front");
  if (!same(rw.ftq_prf_pc_resp, s.w.ftq_prf_pc_resp)) fail(name, "ftq_prf_pc_resp");
  if (!same(rw.ftq_rob_pc_resp, s.w.ftq_rob_pc_resp)) fail(name, "ftq_rob_pc_resp");
  if (!same(rw.ftq_commit_info, s.w.ftq_commit_info)) fail(name, "ftq_commit_info");
  const bool committed = p == PrePhase::seq;
  if (!PreShadow::state_equal(x, s.ref, !committed)) fail(name, committed ? "state (committed)" : "state (working)");
}
