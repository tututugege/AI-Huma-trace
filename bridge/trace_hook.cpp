#include "trace_hook.h"
#include "TraceBridge.h"
extern long long sim_time;   // timing-model cycle counter (main.cpp)
#include "config.h"
#include "RefCore.h"
#include "PhysMemory.h"
#include "refcpu_api.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>

namespace tbhook {

namespace {
RefCpuContext *g_ctx = nullptr;
std::unique_ptr<tb::Bridge> g_bridge;
uint64_t g_rob_uid[4096];


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

void on_dispatch(uint64_t uid, xlen_t pc, uint32_t inst, uint32_t rob_idx, uint32_t len) {
  ensure();
  g_rob_uid[rob_idx & 4095] = uid;
  g_bridge->dispatch(uid, pc, inst, len);
}

void on_mispredict(uint64_t branch_uid) {
  ensure();
  g_bridge->rewind_after((int64_t)branch_uid);
}

uint32_t mem_read(uint32_t paddr) {
  ensure();
  uint32_t w;
  if (g_bridge->read_word(paddr, w)) return w;
  return pmem_read(paddr);
}

const tb::Fact *fact_by_rob(uint32_t rob_idx) {
  if (!g_bridge) return nullptr;
  return g_bridge->find(g_rob_uid[rob_idx & 4095]);
}

bool fs_dirty_at_commit(uint64_t uid) {
  ensure();
  const tb::Fact *f = g_bridge->find(uid);
  return f && f->fs_dirty && !f->trapped && f->blocked == tb::Blocked::None;
}

void on_commit(uint64_t uid) {
  ensure();
  // Entries older than the committing one that never committed were killed without a rewind (wrong-path stragglers
  // dispatched after the mispredict): undo them and re-execute the younger ones before the fact is used.
  g_bridge->kill_older_than(uid);
  g_bridge->commit(uid);
}

void on_flush(int64_t uid, bool interrupt) {
  ensure();
  if (interrupt || uid < 0) return;  // interrupt: handled by on_interrupt; no committing instruction: nothing to do yet
  g_bridge->commit_flush((uint64_t)uid);
}

void on_interrupt(xlen_t cause, uint8_t target_priv, uint32_t pending) {
  ensure();
  g_bridge->take_interrupt(cause, target_priv, pending);
}

// The model's results are 32-bit values (not widened yet): a load result is the sign (or, for lwu, zero) extension of
// that value to the register width, everything else is zero-extended.
static xlen_t widen_result(const ExecObs &o, uint32_t inst) {
  if (o.op == UOP_LOAD && ((inst >> 12) & 7) <= 2)   // lb / lh / lw: sign-extended 32-bit pattern; lbu/lhu/lwu zero, ld full width
    return static_cast<xlen_t>(static_cast<sxlen_t>(static_cast<int32_t>(o.result)));
  return o.result;
}

// Values the bridge cannot know (an MMIO load, a CSR read of the timing model's own counters): patched from the model's result.
void on_exec(const ExecObs &o) {
  ensure();
  const uint64_t uid = g_rob_uid[o.rob_idx & 4095];
  const tb::Fact *f = g_bridge->find(uid);
  if (!f) return;
  if (o.op == UOP_LOAD && f->blocked == tb::Blocked::MemOutOfRam && !o.page_fault) {
    g_bridge->patch_rd(uid, widen_result(o, f->inst));
    return;
  }
  // (fflags / frm / fcsr are the function model's own state: the timing model's CSR file keeps no values for them)
  const uint32_t csr_no = f->inst >> 20;
  if (o.op == UOP_CSR && !(csr_no >= 1 && csr_no <= 3) && f->rd_valid && widen_result(o, f->inst) != f->rd_value &&
      f->blocked == tb::Blocked::None && !f->trapped)
    g_bridge->patch_rd(uid, widen_result(o, f->inst));
}
} // namespace tbhook
