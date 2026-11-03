#include "RobShadow.h"
#include "RobDigest.h"
#include "RobRef.h"
#include "Wires.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>

namespace {
struct Shadow {
  RobRef ref;
  Wires w;
  unsigned long long phases = 0;
  explicit Shadow(Rob &r) : ref(r.ctx) { ref.wires = &w; ref.init(); }
};
[[noreturn]] void fail(const char *name, const char *what) {
  fprintf(stderr, "[ROB_SHADOW] MISMATCH in %s: %s\n", name, what);
  exit(2);
}
template <class A, class B> bool same(const A &a, const B &b) { static_assert(sizeof(A) == sizeof(B)); return !memcmp(&a, &b, sizeof(A)); }
}  // namespace

// Directed scenarios for completion corner cases no program reaches: identical hand-built entries go into the real and
// the reference Rob, comb_complete runs on both, working-state digests must match (ROB_SHADOW_DIRECTED=1).
namespace {
void run_directed() {
  int bad = 0;
  for (int scenario = 0; scenario < 2; scenario++) {
    Wires wa, wb;
    Rob a(nullptr); RobRef b(nullptr);
    a.wires = &wa; b.wires = &wb; a.init(); b.init();
    const int p1 = IQ_STA_PORT_BASE, p2 = IQ_STA_PORT_BASE + 1;  // two uops of one store write back to the same entry
    for (auto *e : {&a.entry_1[0][0], (decltype(&a.entry_1[0][0]))nullptr}) { (void)e; }
    auto setup = [&](auto &rob) {
      rob.entry_1[0][0].valid = true;
      rob.entry_1[0][0].uop.rob_idx = 0;
      rob.entry_1[0][0].uop.expect_mask = rob_cplt_mask_from_issue_port(p1) | rob_cplt_mask_from_issue_port(p2);
      rob.entry_1[0][0].uop.cplt_mask = 0;
    };
    setup(a); setup(b);
    for (Wires *w : {&wa, &wb}) {
      w->exu2rob.entry[p1].valid = true; w->exu2rob.entry[p1].uop.rob_idx = 0;
      w->exu2rob.entry[p2].valid = true; w->exu2rob.entry[p2].uop.rob_idx = 0;
      if (scenario == 0) { w->exu2rob.entry[p1].uop.flush_pipe = true; }                       // later uop must not clear it
      else { w->exu2rob.entry[p1].uop.page_fault_store = true; w->exu2rob.entry[p1].uop.page_fault_load = true; }  // store fault wins
    }
    a.comb_complete(); b.comb_complete();
    const bool ok = rob_digest(a, true) == rob_digest(b, true);
    printf("[ROB_DIRECTED] %-40s %s\n", scenario == 0 ? "flush_pipe accumulates over STA/STD uops" : "store page fault clears load fault", ok ? "same" : "MISMATCH");
    bad += !ok;
  }
  if (bad) exit(2);
}
}  // namespace

void rob_shadow(Rob &r, RobPhase p, const char *name) {
  static bool directed_done = false;
  if (!directed_done) { directed_done = true; if (getenv("ROB_SHADOW_DIRECTED")) run_directed(); }
  static std::map<Rob *, Shadow *> all;
  Shadow *&sp = all[&r];
  if (!sp) sp = new Shadow(r);
  Shadow &s = *sp;
  Wires &rw = *r.wires;
  // inputs the Rob reads
  s.w.dec_bcast = rw.dec_bcast; s.w.csr2rob = rw.csr2rob; s.w.exu2rob = rw.exu2rob; s.w.ftq_rob_pc_resp = rw.ftq_rob_pc_resp;
  s.w.lsu2rob = rw.lsu2rob; s.w.dis2rob = rw.dis2rob; s.w.in.front_stall = rw.in.front_stall;
  if (p == RobPhase::comb_begin) {  // outputs start every cycle from the cleared wires
    s.w.rob_bcast = rw.rob_bcast; s.w.rob_commit = rw.rob_commit; s.w.rob2dis = rw.rob2dis; s.w.rob2csr = rw.rob2csr;
    s.w.ftq_rob_pc_req = rw.ftq_rob_pc_req;
  }
  switch (p) {
#define X(n) case RobPhase::n: s.ref.n(); break;
    ROB_PHASES(X)
#undef X
  }
  s.phases++;
  if (!same(rw.rob2dis, s.w.rob2dis)) fail(name, "rob2dis");
  if (!same(rw.rob2csr, s.w.rob2csr)) fail(name, "rob2csr");
  if (!same(rw.rob_bcast, s.w.rob_bcast)) fail(name, "rob_bcast");
  if (!same(rw.ftq_rob_pc_req, s.w.ftq_rob_pc_req)) fail(name, "ftq_rob_pc_req");
  if (p == RobPhase::comb_commit && !same(rw.rob_commit, s.w.rob_commit)) fail(name, "rob_commit");  // later stages patch diag_val in place
  for (int i = 0; i < COMMIT_WIDTH; i++)
    if (r.out.commit_extra[i].src1_areg != s.ref.out.commit_extra[i].src1_areg || r.out.commit_extra[i].imm != s.ref.out.commit_extra[i].imm) fail(name, "commit_extra");
  if (p == RobPhase::seq) { if (rob_digest(r) != s.ref.digest()) fail(name, "state digest (committed)"); }
  else if (rob_digest(r, true) != rob_digest(s.ref, true)) fail(name, "state digest (working)");
}
