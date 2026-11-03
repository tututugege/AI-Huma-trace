#include "LsuShadow.h"
#include "LsuDigest.h"
#include "RealLsuRef.h"
#include "Wires.h"
#include "SimCpu.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>

namespace {
// Digest of the compact LSU: same canonical order as lsu_digest() on the reference state.
uint64_t lsu_digest_new(const LsuState &s) {
  LsuDigest d;
  for (int i = 0; i < LDQ_SIZE; i++) {
    const auto &e = s.ldq[i];
    d(static_cast<uint32_t>(e.load_state));
    if (e.load_state == LoadState::Empty) continue;
    d(e.v_addr_valid); d(e.v_addr); d(e.p_addr_valid); d(e.p_addr); d(static_cast<uint32_t>(e.replay_type));
    d(e.func3); d(e.result); d(e.dest_preg); d(e.rob_idx); d(e.rob_flag); d(e.br_mask); d(e.is_mmio);
    d(e.diag_val); d(e.page_fault); d(e.is_lrsc); d(e.stq_snapshot.idx); d(e.stq_snapshot.flag); d(e.cache_miss);
  }
  d(s.stq_head); d(s.stq_commit); d(s.stq_head_flag); d(s.stq_commit_count); d(s.stq_count);
  for (uint32_t i = 0; i < (uint32_t)s.stq_count; i++) {
    const auto &e = s.stq[(s.stq_head + i) % STQ_SIZE];
    d(e.data_valid); d(e.data); d(e.func3); d(e.vaddr_valid); d(e.vaddr); d(e.paddr_valid); d(e.paddr); d(e.page_fault);
    d(e.suppress_write); d(e.is_mmio); d(e.is_lrsc); d(e.sc_pass); d(e.dest_preg); d(static_cast<uint32_t>(e.store_state));
    d(static_cast<uint32_t>(e.replay_type)); d(e.replay_wait_cycles); d(e.br_mask); d(e.rob_idx); d(e.rob_flag); d(e.stq_flag);
  }
  auto q = [&](const auto &r, uint32_t size, auto fn) {
    d(r.head); d(r.count);
    for (uint32_t i = 0; i < r.count; i++) { const auto &e = r.e[(r.head + i) % size]; d(e.valid); fn(e); }
  };
  q(s.wait_mmu_stq, STQ_SIZE, [&](const auto &e) { d(e.idx); });
  q(s.wait_mmu_ldq, LDQ_SIZE, [&](const auto &e) { d(e.idx); });
  q(s.mmu_done_stq, STQ_SIZE, [&](const auto &e) { d(e.idx); });
  q(s.finish, LDQ_SIZE, [&](const auto &e) { d(e.idx); });
  q(s.stlf_queue, LDQ_SIZE, [&](const auto &e) { d(e.idx); });
  q(s.wait_dcache_ldq, LDQ_SIZE, [&](const auto &e) { d(e.idx); d(e.req_gen); d(e.replay_wait_cycles); });
  d(s.uncached_unit.valid); d(s.uncached_unit.is_load); d(s.uncached_unit.addr); d(s.uncached_unit.wdata);
  d(s.uncached_unit.func3); d(s.uncached_unit.idx);
  d(s.lrsc_unit.reserve_valid); d(s.lrsc_unit.reserve_addr); d(s.lrsc_unit.reserve_rob_idx);
  d(s.lrsc_unit.reserve_rob_flag); d(s.lrsc_unit.reserve_br_mask);
  d(s.req_gen);
  return d.h;
}
struct Shadow {
  RealLsuRef ref;
  LsuDisIO dis{};
  LsuRobIO rob{};
  LsuExeIO exe{};
  PeripheralReqIO per{};
  LsuDcacheIO dc{};
  LsuMMUIO mmu{};
  uint64_t phases = 0, cycles = 0;
  explicit Shadow(SimContext *c) : ref(c) {
    ref.out = LsuOutRef{&dis, &rob, &exe, &per, &dc, &mmu};
    ref.init();
  }
  ~Shadow() { fprintf(stderr, "[LSU_SHADOW] %llu phases, %llu cycles compared, no mismatch\n", (unsigned long long)phases, (unsigned long long)cycles); }
};
[[noreturn]] void fail(const char *name, const char *what, const void *a = nullptr, const void *b = nullptr, size_t n = 0) {
  fprintf(stderr, "[LSU_SHADOW] MISMATCH in %s: %s\n", name, what);
  if (a) {
    int shown = 0;
    for (size_t i = 0; i < n && shown < 8; i++)
      if (((const uint8_t *)a)[i] != ((const uint8_t *)b)[i]) { fprintf(stderr, "  byte %zu: new=%02x ref=%02x\n", i, ((const uint8_t *)a)[i], ((const uint8_t *)b)[i]); shown++; }
  }
  exit(2);
}
bool same_exe(const LsuExeIO &a, const LsuExeIO &b) {  // entry padding after `valid` is indeterminate
  auto eq = [](const auto &x, const auto &y) { return x.valid == y.valid && !memcmp(&x.uop, &y.uop, sizeof(x.uop)); };
  for (int i = 0; i < LSU_LOAD_WB_WIDTH; i++) if (!eq(a.wb_req[i], b.wb_req[i])) return false;
  for (int i = 0; i < LSU_STA_COUNT; i++) if (!eq(a.sta_wb_req[i], b.sta_wb_req[i])) return false;
  return true;
}
}  // namespace

// Directed state-injection scenarios for paths no program reaches: identical hand-built states go into the real
// and the reference LSU, one phase runs on both, the state digests must match (LSU_SHADOW_DIRECTED=1).
namespace {
LsuInRef ref_in(Wires &w) {
  return LsuInRef{&w.rob_commit, &w.rob_bcast, &w.dec_bcast, &w.csr_status, &w.dis2lsu, &w.exe2lsu, &w.in.peripheral_resp, &w.in.dcache2lsu, &w.mmu2lsu_io};
}
LsuOutRef ref_out(Wires &w) {
  return LsuOutRef{&w.lsu2dis, &w.lsu2rob, &w.lsu2exe, &w.out.peripheral_req, &w.out.lsu2dcache, &w.lsu2mmu_io};
}
struct Ios {
  Wires w;
  DcacheLsuIO &dc = w.in.dcache2lsu;
  RobBroadcastIO &rb = w.rob_bcast;
};
int directed_failures = 0;
void check_same(const char *what, RealLsu &a, RealLsuRef &b) {
  const bool ok = lsu_digest_new(a.nxt) == b.digest_nxt();
  printf("[LSU_DIRECTED] %-34s %s\n", what, ok ? "same" : "MISMATCH");
  directed_failures += !ok;
}
template <class S> void poke_stq_entry(S &st, int i, bool lrsc, bool sc_pass, StoreState state, uint32_t paddr) {
  st.stq[i].paddr_valid = true; st.stq[i].paddr = paddr; st.stq[i].func3 = 2; st.stq[i].data_valid = true;
  st.stq[i].is_lrsc = lrsc; st.stq[i].sc_pass = sc_pass; st.stq[i].store_state = state;
}
void run_directed() {
  // 1) one MSHR fill with six MSHR_FULL replays waiting: only LSU_LDU_COUNT may be woken
  { Ios io; RealLsu a(nullptr); RealLsuRef b(nullptr); a.wires = &io.w; b.in = ref_in(io.w); b.out = ref_out(io.w);
    io.dc.mshr_fill = true; io.dc.mshr_fill_addr = 0xdead0000u;
    for (int i = 0; i < 6; i++) {
      for (LsuState *st : {&a.cur, &a.nxt}) { st->ldq[i].load_state = LoadState::ReadyToIssue; st->ldq[i].replay_type = ReplayType::MSHR_FULL; st->ldq[i].p_addr = 0x1000 + 64 * i;
        st->wait_dcache_ldq.e[i].valid = true; st->wait_dcache_ldq.e[i].idx = i; st->wait_dcache_ldq.e[i].replay_wait_cycles = 4; }
      for (LsuStateRef *st : {&b.cur, &b.nxt}) { st->ldq[i].load_state = LoadStateRef::ReadyToIssue; st->ldq[i].replay_type = ReplayType::MSHR_FULL; st->ldq[i].p_addr = 0x1000 + 64 * i;
        st->wait_dcache_ldq[i].valid = true; st->wait_dcache_ldq[i].ldq_idx = i; st->wait_dcache_ldq[i].replay_wait_cycles = 4; }
    }
    a.cur.wait_dcache_ldq.count = a.nxt.wait_dcache_ldq.count = 6; b.cur.wait_dcache_ldq_count = b.nxt.wait_dcache_ldq_count = 6;
    a.comb_dcache2lsu_ldq(); b.comb_dcache2lsu_ldq(); check_same("mshr_fill wakes at most LDU_COUNT", a, b); }
  // 2) a load fully covered by a store-conditional that is not decided yet must wait, with and without data
  for (int data : {1, 0}) { Ios io; RealLsu a(nullptr); RealLsuRef b(nullptr); a.wires = &io.w; b.in = ref_in(io.w); b.out = ref_out(io.w);
    for (LsuState *st : {&a.cur, &a.nxt}) { st->stq_count = 1; poke_stq_entry(*st, 0, true, false, StoreState::Ready, 0x2000); st->stq[0].data_valid = data;
      st->ldq[0].load_state = LoadState::CheckStlf; st->ldq[0].p_addr = 0x2000; st->ldq[0].func3 = 2; st->ldq[0].stq_snapshot.idx = 1;
      st->stlf_queue.e[0].valid = true; st->stlf_queue.e[0].idx = 0; st->stlf_queue.count = 1; }
    for (LsuStateRef *st : {&b.cur, &b.nxt}) { st->stq_count = 1; poke_stq_entry(*st, 0, true, false, StoreState::Ready, 0x2000); st->stq[0].data_valid = data;
      st->ldq[0].load_state = LoadStateRef::CheckStlf; st->ldq[0].p_addr = 0x2000; st->ldq[0].func3 = 2; st->ldq[0].stq_snapshot.idx = 1;
      st->stlf_queue[0].valid = true; st->stlf_queue[0].ldq_idx = 0; st->stlf_queue_count = 1; }
    a.comb_stlf(); b.comb_stlf(); check_same(data ? "STLF behind undecided SC (data valid)" : "STLF behind SC without data", a, b); }
  // 3) flush with an MMIO store on the bus: kept only if still inside the committed window
  for (int idx : {0, 1}) { Ios io; RealLsu a(nullptr); RealLsuRef b(nullptr); a.wires = &io.w; b.in = ref_in(io.w); b.out = ref_out(io.w);
    io.rb.flush = true;
    for (LsuState *st : {&a.cur, &a.nxt}) { st->stq_count = 2; st->stq_commit_count = 1; st->stq[idx].store_state = StoreState::WaitMmioResp;
      st->uncached_unit.valid = true; st->uncached_unit.is_load = false; st->uncached_unit.idx = idx; }
    for (LsuStateRef *st : {&b.cur, &b.nxt}) { st->stq_count = 2; st->stq_commit_count = 1; st->stq[idx].store_state = StoreState::WaitMmioResp;
      st->uncached_unit.valid = true; st->uncached_unit.is_load = false; st->uncached_unit.idx = idx; }
    a.comb_flush(); b.comb_flush(); check_same(idx == 0 ? "flush keeps committed MMIO store" : "flush drops uncommitted MMIO store", a, b); }
  if (directed_failures) exit(2);
}
}  // namespace

void lsu_shadow(RealLsu &l, LsuPhase p, const char *name) {
  static bool directed_done = false;
  if (!directed_done) { directed_done = true; if (getenv("LSU_SHADOW_DIRECTED")) run_directed(); }
  static std::map<RealLsu *, Shadow *> all;
  Shadow *&s = all[&l];
  if (!s) s = new Shadow(l.ctx);
  s->ref.in = ref_in(*l.wires);
  switch (p) {
#define X(n) case LsuPhase::n: s->ref.n(); break;
    LSU_PHASES(X)
#undef X
  }
  s->phases++;
  switch (p) {
    case LsuPhase::comb_lsu2dis: if (memcmp((&l.wires->lsu2dis), &s->dis, sizeof(LsuDisIO))) fail(name, "lsu2dis"); break;
    case LsuPhase::comb_lsu2rob: if (memcmp((&l.wires->lsu2rob), &s->rob, sizeof(LsuRobIO))) fail(name, "lsu2rob"); break;
    case LsuPhase::comb_lsu2exe: if (!same_exe(l.wires->lsu2exe, s->exe)) fail(name, "lsu2exe"); break;
    case LsuPhase::comb_mmio_out: if (memcmp((&l.wires->out.peripheral_req), &s->per, sizeof(PeripheralReqIO))) fail(name, "peripheral_req"); break;
    case LsuPhase::comb_tlb_out: if (memcmp((&l.wires->lsu2mmu_io), &s->mmu, sizeof(LsuMMUIO))) fail(name, "lsu2mmu"); break;
    case LsuPhase::comb_lsu2dcache_ldq:
    case LsuPhase::comb_lsu2dcache_stq:
      if (memcmp(&l.wires->out.lsu2dcache.req_ports, &s->dc.req_ports, sizeof(s->dc.req_ports))) fail(name, "lsu2dcache.req_ports");
      break;
    default: break;
  }
  const uint64_t a = lsu_digest_new(l.nxt), b = lsu_digest(s->ref.nxt, LoadStateRef::Empty);
  if (a != b) fail(name, "state digest (nxt)");
  if (p == LsuPhase::seq) {
    s->cycles++;
    if (lsu_digest_new(l.cur) != s->ref.digest()) fail(name, "state digest (cur)");
  }
}
