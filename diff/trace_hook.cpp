#include "trace_hook.h"
#include "TraceBridge.h"
#include "config.h"
#include "diff.h"
#include "PhysMemory.h"
#include "refcpu_api.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>

namespace tbhook {
bool enabled = std::getenv("TB_CHECK") != nullptr;

namespace {
RefCpuContext *g_ctx = nullptr;
std::unique_ptr<tb::Bridge> g_bridge;
bool g_resync = false;

struct Counters {
  uint64_t dispatch = 0, commit = 0, commit_front_miss = 0, flush = 0, mispredict = 0;
  uint64_t cmp_rd = 0, cmp_br = 0, cmp_addr = 0, cmp_target = 0;
  uint64_t mis_rd = 0, mis_br = 0, mis_addr = 0, mis_target = 0;
  uint64_t skip_nofact = 0, skip_blocked = 0, skip_trapped = 0, skip_fault = 0, skip_other = 0;
  uint64_t mis_by_op[32] = {0}, seen_by_op[32] = {0}, nofact_by_op[32] = {0};
  uint64_t printed = 0, uid_conflict = 0, patch_mmio_load = 0, patch_csr_read = 0;
  ~Counters() {
    if (!enabled) return;
    std::fprintf(stderr,
      "[TB_CHECK] dispatch=%llu commit=%llu commit_front_miss=%llu flush=%llu mispredict=%llu\n"
      "[TB_CHECK] compared rd=%llu br=%llu addr=%llu target=%llu | mismatches rd=%llu br=%llu addr=%llu target=%llu\n"
      "[TB_CHECK] uid_conflict(dbg vs rob map)=%llu patches: mmio_load=%llu csr_read=%llu\n[TB_CHECK] skipped nofact=%llu blocked=%llu trapped=%llu fault=%llu other=%llu\n",
      (unsigned long long)dispatch, (unsigned long long)commit, (unsigned long long)commit_front_miss,
      (unsigned long long)flush, (unsigned long long)mispredict,
      (unsigned long long)cmp_rd, (unsigned long long)cmp_br, (unsigned long long)cmp_addr,
      (unsigned long long)cmp_target, (unsigned long long)mis_rd, (unsigned long long)mis_br,
      (unsigned long long)mis_addr, (unsigned long long)mis_target, (unsigned long long)uid_conflict, (unsigned long long)patch_mmio_load, (unsigned long long)patch_csr_read, (unsigned long long)skip_nofact,
      (unsigned long long)skip_blocked, (unsigned long long)skip_trapped, (unsigned long long)skip_fault,
      (unsigned long long)skip_other);
    std::fprintf(stderr, "[TB_CHECK] seen/mismatch by op:");
    for (int i = 0; i < 32; i++) if (seen_by_op[i]) std::fprintf(stderr, " op%d=%llu/%llu", i, (unsigned long long)seen_by_op[i], (unsigned long long)mis_by_op[i]);
    std::fprintf(stderr, "\n[TB_CHECK] nofact by op:");
    for (int i = 0; i < 32; i++) if (nofact_by_op[i]) std::fprintf(stderr, " op%d=%llu", i, (unsigned long long)nofact_by_op[i]);
    std::fprintf(stderr, "\n");
    if (g_bridge)
      std::fprintf(stderr, "[TB_CHECK] bridge forced_jumps=%llu rewinds=%llu dropped=%llu trapped=%llu blocked=%llu/%llu/%llu/%llu\n",
        (unsigned long long)g_bridge->stats.forced_jumps, (unsigned long long)g_bridge->stats.rewinds,
        (unsigned long long)g_bridge->stats.entries_dropped, (unsigned long long)g_bridge->stats.trapped,
        (unsigned long long)g_bridge->stats.blocked[1], (unsigned long long)g_bridge->stats.blocked[2],
        (unsigned long long)g_bridge->stats.blocked[3], (unsigned long long)g_bridge->stats.blocked[4]);
  }
} g_c;

void ensure() {
  if (g_bridge) return;
  g_ctx = refcpu_init(0, static_cast<uint32_t>(RAM_SIZE));
  refcpu_set_uart_print(g_ctx, false);
  const char *e = std::getenv("TB_IMG_BYTES");
  size_t bytes = e ? strtoull(e, 0, 0) : (1u << 20);
  refcpu_sync_ram_from_dut(g_ctx, pmem_ram_ptr(), bytes);
  g_bridge.reset(new tb::Bridge(g_ctx, 0x80000000u, static_cast<uint32_t>(RAM_SIZE)));
}
} // namespace

static uint64_t g_rob_uid[4096];
void on_dispatch(uint64_t uid, uint32_t pc, uint32_t inst, uint32_t rob_idx) {
  ensure();
  g_rob_uid[rob_idx & 4095] = uid;
  if (g_resync) {
    // After a pipeline flush the committed state (traps, CSR side effects, privilege) comes from
    // the difftest reference, which executed the committing instruction for real.
    g_bridge->rewind_after(-1);
    RefCpuState s;
    if (difftest_get_ref_state(&s)) refcpu_set_state(g_ctx, &s);
    g_resync = false;
  }
  g_bridge->dispatch(uid, pc, inst);
  g_c.dispatch++;
}

void on_mispredict(uint64_t branch_uid) {
  ensure();
  g_c.mispredict++;
  if (std::getenv("TB_NEG_NO_REWIND")) return;  // negative control: the check must fail
  g_bridge->rewind_after((int64_t)branch_uid);
}

void on_commit(uint64_t uid) {
  ensure();
  g_c.commit++;
  // Entries older than the committing one that never committed were killed without a rewind: drop them.
  while (g_bridge->in_flight() && g_bridge->oldest_uid() < uid) { g_bridge->drop_oldest(); g_c.commit_front_miss++; }
  g_bridge->commit(uid);
}

void on_flush() { ensure(); g_c.flush++; g_resync = true; }

void on_exec(const ExecObs &o) {
  ensure();
  const uint64_t uid = g_rob_uid[o.rob_idx & 4095];
  if (o.uid_dbg != 0 && o.uid_dbg != uid) { g_c.uid_conflict++; if (g_c.uid_conflict <= 4) std::fprintf(stderr, "[TB_CHECK] uid_conflict rob_idx=%u map_uid=%llu dbg_uid=%llu op=%d result=0x%x diag=0x%x\n", o.rob_idx, (unsigned long long)uid, (unsigned long long)o.uid_dbg, o.op, o.result, o.diag_val); }
  const tb::Fact *f = g_bridge->find(uid);
  if (!f) { g_c.skip_nofact++; g_c.nofact_by_op[o.op & 31]++; if (g_c.nofact_by_op[o.op & 31] <= 3) std::fprintf(stderr, "[TB_CHECK] nofact op=%d uid=%llu oldest=%llu in_flight=%zu\n", o.op, (unsigned long long)uid, (unsigned long long)(g_bridge->in_flight() ? g_bridge->oldest_uid() : 0), g_bridge->in_flight()); return; }
  // Values the bridge cannot know: patch them from the model, then compare as usual.
  if (o.op == UOP_LOAD && f->blocked == tb::Blocked::MemOutOfRam && !o.page_fault) {
    if (g_bridge->patch_rd(uid, o.result)) { g_c.patch_mmio_load++; return; }
  }
  if (o.op == UOP_CSR && f->rd_valid && o.result != f->rd_value && f->blocked == tb::Blocked::None && !f->trapped) {
    if (g_bridge->patch_rd(uid, o.result)) { g_c.patch_csr_read++; f = g_bridge->find(uid); }
  }
  if (f->blocked != tb::Blocked::None) { g_c.skip_blocked++; return; }
  if (f->trapped) { g_c.skip_trapped++; return; }
  if (o.page_fault) { g_c.skip_fault++; return; }
  auto report = [&](const char *what, uint32_t got, uint32_t want, uint64_t &cnt) {
    cnt++; g_c.mis_by_op[o.op & 31]++;
    if (g_c.printed++ < 20)
      std::fprintf(stderr, "[TB_CHECK][MISMATCH] %s uid=%llu pc=0x%08x inst=0x%08x op=%d model=0x%08x bridge=0x%08x forced_jump=%d\n",
                   what, (unsigned long long)uid, f->pc, f->inst, o.op, got, want, (int)f->forced_jump);
  };
  switch (o.op) {
  case UOP_ADD: case UOP_MUL: case UOP_DIV: case UOP_CSR: case UOP_FP: case UOP_LOAD:
    if (f->rd_valid) {
      g_c.cmp_rd++;
      if (o.result != f->rd_value) report("rd", o.result, f->rd_value, g_c.mis_rd);
    } else g_c.skip_other++;
    break;
  default: break;
  }
  g_c.seen_by_op[o.op & 31]++;
  if (o.op == UOP_JUMP || o.op == UOP_BR) {
    // diag_val carries the resolved next pc on the exu->rob interface.
    g_c.cmp_target++;
    if (o.diag_val != f->next_pc) report("target", o.diag_val, f->next_pc, g_c.mis_target);
  }
  if (o.op == UOP_BR) {
    g_c.cmp_br++;
    if (o.br_taken != f->br_taken) report("br_taken", o.br_taken, f->br_taken, g_c.mis_br);
  }
  if (o.op == UOP_LOAD || o.op == UOP_STA) {
    g_c.cmp_addr++;
    if (o.diag_val != f->vaddr) report("addr", o.diag_val, f->vaddr, g_c.mis_addr);
  }
}
} // namespace tbhook
