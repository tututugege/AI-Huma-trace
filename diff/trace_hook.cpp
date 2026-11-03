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
// P3: the value datapath is gone, facts are mandatory (TB_NO_INJECT only for debugging old builds).
bool inject = std::getenv("TB_NO_INJECT") == nullptr;
bool poison = std::getenv("TB_POISON") != nullptr;
bool enabled = std::getenv("TB_CHECK") != nullptr || inject;
uint64_t inject_miss = 0;

namespace {
RefCpuContext *g_ctx = nullptr;
std::unique_ptr<tb::Bridge> g_bridge;
bool g_resync = false;
uint64_t g_rob_uid[4096];

struct Counters {
  uint64_t dispatch = 0, commit = 0, commit_front_miss = 0, flush = 0, mispredict = 0;
  uint64_t cmp_rd = 0, cmp_br = 0, cmp_addr = 0, cmp_target = 0;
  uint64_t mis_rd = 0, mis_br = 0, mis_addr = 0, mis_target = 0;
  uint64_t skip_nofact = 0, skip_blocked = 0, skip_trapped = 0, skip_fault = 0, skip_other = 0;
  uint64_t mis_by_op[32] = {0}, seen_by_op[32] = {0}, nofact_by_op[32] = {0};
  uint64_t mem_writes = 0, interrupt_rejected = 0, printed = 0, uid_conflict = 0, patch_mmio_load = 0, patch_csr_read = 0;
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
  g_ctx = pmem_core_ctx();  // the RAM image was loaded straight into the core
  // Boot ROM (word-backed MMIO at 0x0): the core fetches the reset stub from there.
  for (uint32_t a = 0; a < 0x100000u; a += 4) {
    const uint32_t w = pmem_read(a);
    if (w) refcpu_set_io_word(g_ctx, a, w);
  }
  g_bridge.reset(new tb::Bridge(g_ctx, 0x80000000u, static_cast<uint32_t>(RAM_SIZE)));
  g_bridge->set_boot_region(0, 0x100000u);
}
} // namespace

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
  if (pc >= 0x80000000u && inst != pmem_read(pc) && std::getenv("TB_DEBUG_FLUSH")) { static int n = 0; if (n++ < 4) std::fprintf(stderr, "[BAD-FETCH] uid=%llu pc=0x%x inst=0x%08x pmem=0x%08x\n", (unsigned long long)uid, pc, inst, pmem_read(pc)); }
  g_bridge->dispatch(uid, pc, inst);
  g_c.dispatch++;
}

void on_mispredict(uint64_t branch_uid) {
  ensure();
  g_c.mispredict++;
  if (std::getenv("TB_NEG_NO_REWIND")) return;  // negative control: the check must fail
  g_bridge->rewind_after((int64_t)branch_uid);
}

static uint64_t g_ov[5] = {0}, g_ov_diff[5] = {0};
void note_override(int kind, uint32_t computed, uint32_t injected, uint32_t rob_idx) {
  g_ov[kind]++;
  if (computed != injected) { g_ov_diff[kind]++; if (g_ov_diff[kind] <= 3) { const tb::Fact *ff = g_bridge ? g_bridge->find(g_rob_uid[rob_idx & 4095]) : nullptr; std::fprintf(stderr, "[TB_INJECT] kind=%d computed=0x%x injected=0x%x rob_idx=%u uid=%llu fact pc=0x%x inst=0x%x blocked=%d trapped=%d forced=%d\n", kind, computed, injected, rob_idx, (unsigned long long)g_rob_uid[rob_idx & 4095], ff ? ff->pc : 0, ff ? ff->inst : 0, ff ? (int)ff->blocked : -1, ff ? (int)ff->trapped : -1, ff ? (int)ff->forced_jump : -1); } }
}
struct OvPrint { ~OvPrint() { if (!inject) return; std::fprintf(stderr, "[TB_INJECT] overrides (n/differs): br=%llu/%llu jump=%llu/%llu agu=%llu/%llu std=%llu/%llu csr=%llu/%llu miss=%llu\n", (unsigned long long)g_ov[0], (unsigned long long)g_ov_diff[0], (unsigned long long)g_ov[1], (unsigned long long)g_ov_diff[1], (unsigned long long)g_ov[2], (unsigned long long)g_ov_diff[2], (unsigned long long)g_ov[3], (unsigned long long)g_ov_diff[3], (unsigned long long)g_ov[4], (unsigned long long)g_ov_diff[4], (unsigned long long)inject_miss); } } g_ovp;
uint32_t mem_read(uint32_t paddr) {
  if (inject) {
    ensure();
    uint32_t w;
    if (g_bridge->read_word(paddr, w)) return w;
  }
  return pmem_read(paddr);
}

const tb::Fact *fact_by_rob(uint32_t rob_idx) {
  if (!inject || !g_bridge) return nullptr;
  const tb::Fact *f = g_bridge->find(g_rob_uid[rob_idx & 4095]);
  if (!f) { inject_miss++; return f; }
  // Positive control: every Nth branch fact is flipped; the run must then differ from the baseline.
  static const long n = std::getenv("TB_PERTURB_BR") ? std::atol(std::getenv("TB_PERTURB_BR")) : 0;
  static const long m = std::getenv("TB_PERTURB_ADDR") ? std::atol(std::getenv("TB_PERTURB_ADDR")) : 0;
  static long cnt_br = 0, cnt_ad = 0;
  static tb::Fact tmp;
  if (n && f->is_br && (++cnt_br % n) == 0) { tmp = *f; tmp.br_taken = !tmp.br_taken; return &tmp; }
  if (m && (f->is_load || f->is_store) && (++cnt_ad % m) == 0) { tmp = *f; tmp.vaddr ^= 0x1000; return &tmp; }
  return f;
}

static void apply_mem_write(const tb::Fact *f) {
  static const bool neg = std::getenv("TB_NEG_NO_MEMWRITE") != nullptr;  // negative control
  if (!neg && f && f->has_mem_write && !f->trapped && f->blocked == tb::Blocked::None) {
    // (no functional-memory write: the core owns the memory; instruction words and PTEs are read from it)
    g_c.mem_writes++;
    if (g_c.mem_writes <= 3 && std::getenv("TB_DEBUG_DDR")) std::fprintf(stderr, "[TB] commit mem write word=0x%08x data=0x%08x\n", f->mem_waddr, f->mem_wdata);
  }
}

void on_commit(uint64_t uid) {
  ensure();
  // Entries older than the committing one that never committed were killed without a rewind (wrong-path stragglers
  // dispatched after the mispredict): undo them and re-execute the younger ones before the fact is used.
  if (const size_t k = g_bridge->kill_older_than(uid)) g_c.commit_front_miss += k;
  apply_mem_write(g_bridge->find(uid));
  if (static const bool tr = std::getenv("TB_TRACE_COMMIT") != nullptr; tr) { const tb::Fact *tf = g_bridge->find(uid); if (tf) std::fprintf(stderr, "[COMMIT] pc=0x%08x inst=0x%08x rd=%d v=0x%08x\n", tf->pc, tf->inst, tf->rd_valid ? tf->rd : -1, tf->rd_value); }
  g_c.commit++;
  g_bridge->commit(uid);
}

void on_flush(int64_t uid, bool interrupt) {
  ensure();
  g_c.flush++;
  static const bool use_ref = std::getenv("TB_RESYNC_REF") != nullptr;  // P1b behaviour, for comparison
  if (use_ref) { g_resync = true; return; }
  if (std::getenv("TB_DEBUG_FLUSH")) { const tb::Fact *ff = uid >= 0 ? g_bridge->find((uint64_t)uid) : nullptr; std::fprintf(stderr, "[FLUSH] uid=%lld interrupt=%d pc=0x%x inst=0x%x trapped=%d\n", (long long)uid, (int)interrupt, ff ? ff->pc : 0, ff ? ff->inst : 0, ff ? (int)ff->trapped : -1); }
  if (interrupt || uid < 0) return;  // interrupt: handled by on_interrupt; no committing instruction: nothing to do yet
  apply_mem_write(g_bridge->find((uint64_t)uid));
  g_bridge->commit_flush((uint64_t)uid);
}

void on_interrupt(uint32_t cause, uint8_t target_priv, uint32_t pending) {
  ensure();
  static const bool use_ref = std::getenv("TB_RESYNC_REF") != nullptr;
  if (use_ref) return;
  if (!g_bridge->take_interrupt(cause, target_priv, pending)) g_c.interrupt_rejected++;
}

void on_exec(const ExecObs &o) {
  ensure();
  const uint64_t uid = g_rob_uid[o.rob_idx & 4095];
  if (o.uid_dbg != 0 && o.uid_dbg != uid) { g_c.uid_conflict++; if (g_c.uid_conflict <= 4) std::fprintf(stderr, "[TB_CHECK] uid_conflict rob_idx=%u map_uid=%llu dbg_uid=%llu op=%d result=0x%x diag=0x%x\n", o.rob_idx, (unsigned long long)uid, (unsigned long long)o.uid_dbg, o.op, o.result, o.diag_val); }
  const tb::Fact *f = g_bridge->find(uid);
  if (!f) { g_c.skip_nofact++; g_c.nofact_by_op[o.op & 31]++; if (g_c.nofact_by_op[o.op & 31] <= 3) std::fprintf(stderr, "[TB_CHECK] nofact op=%d uid=%llu oldest=%llu in_flight=%zu\n", o.op, (unsigned long long)uid, (unsigned long long)(g_bridge->in_flight() ? g_bridge->oldest_uid() : 0), g_bridge->in_flight()); return; }
  // Values the bridge cannot know: patch them from the model, then compare as usual.
  if (o.op == UOP_LOAD && f->blocked == tb::Blocked::MemOutOfRam && !o.page_fault) {
    if (std::getenv("TB_DEBUG_PATCH") && g_c.patch_mmio_load < 6) std::fprintf(stderr, "[PATCH] uid=%llu pc=0x%x vaddr=0x%x model=0x%x\n", (unsigned long long)uid, f->pc, f->vaddr, o.result);
    if (g_bridge->patch_rd(uid, o.result)) { g_c.patch_mmio_load++; return; }
  }
  if (o.op == UOP_CSR && f->rd_valid && o.result != f->rd_value && f->blocked == tb::Blocked::None && !f->trapped) {
    if (g_bridge->patch_rd(uid, o.result)) { g_c.patch_csr_read++; f = g_bridge->find(uid); }
  }
  // The value comparison of P1b only means something while the model still computes values.
  static const bool compare = std::getenv("TB_CHECK") != nullptr;
  if (!compare) return;
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
