// The reference core is the trace bridge's own core (refcore_ctx(): the librefcpu instance whose RAM is the model's
// functional memory). This file prepares it for an image or a checkpoint, steps it through the checkpoint prewarm and
// hands its architectural state to the timing model. There is no second core and no comparison: the timing model
// computes no values, the bridge (TraceBridge.cpp) supplies them and TB_CHECK compares what the model still decides.
#include "RefCore.h"
#include "Csr.h"
#include "IoBacking.h"
#include "RISCV.h"
#include "config.h"
#include "mmio_address_map.h"
#include "util.h"

#include <cstring>
#include <iostream>

namespace {

RefCpuContext *g_core = nullptr;   // the core whose RAM is the model's functional memory

RefCpuContext *core() {
  Assert(g_core != nullptr && "reference core: refcore_init() has not run");
  return g_core;
}

// Zero a buffer page by page, touching only pages that are not zero already: a freshly calloc'ed 2 GiB stays unmapped
// (the result is the same as memset, the resident size is not).
void clear_sparse(void *base, size_t bytes) {
  constexpr size_t kPage = 4096;
  auto *p = static_cast<uint8_t *>(base);
  static const uint8_t zeros[kPage] = {};
  for (size_t off = 0; off < bytes; off += kPage) {
    const size_t n = bytes - off < kPage ? bytes - off : kPage;
    if (std::memcmp(p + off, zeros, n) != 0) std::memset(p + off, 0, n);
  }
}

RefCpuState to_ref_state(const CPU_state &state, uint8_t privilege) {
  RefCpuState ref_state{};
  std::memcpy(ref_state.gpr, state.gpr, sizeof(ref_state.gpr));
  std::memcpy(ref_state.fpr, state.fpr, sizeof(ref_state.fpr));
  ref_state.fcsr = state.fcsr;
  std::memcpy(ref_state.csr, state.csr, sizeof(ref_state.csr));
  ref_state.pc = state.pc;
  ref_state.store_addr = state.store_addr;
  ref_state.store_data = state.store_data;
  ref_state.store_strb = state.store_strb;
  ref_state.store = state.store;
  ref_state.instruction = state.instruction;
  ref_state.page_fault_inst = state.page_fault_inst;
  ref_state.page_fault_load = state.page_fault_load;
  ref_state.page_fault_store = state.page_fault_store;
  ref_state.inst_idx = state.inst_idx;
  ref_state.commit_pc = state.commit_pc;
  ref_state.reserve_valid = state.reserve_valid;
  ref_state.reserve_addr = state.reserve_addr;
  ref_state.privilege = privilege;
  return ref_state;
}

CPU_state from_ref_state(const RefCpuState &ref_state) {
  CPU_state state{};
  std::memcpy(state.gpr, ref_state.gpr, sizeof(state.gpr));
  std::memcpy(state.fpr, ref_state.fpr, sizeof(state.fpr));
  state.fcsr = ref_state.fcsr;
  std::memcpy(state.csr, ref_state.csr, sizeof(state.csr));
  state.pc = ref_state.pc;
  state.store_addr = ref_state.store_addr;
  state.store_data = ref_state.store_data;
  state.store_strb = ref_state.store_strb;
  state.store = ref_state.store;
  state.instruction = ref_state.instruction;
  state.page_fault_inst = ref_state.page_fault_inst;
  state.page_fault_load = ref_state.page_fault_load;
  state.page_fault_store = ref_state.page_fault_store;
  state.inst_idx = ref_state.inst_idx;
  state.commit_pc = ref_state.commit_pc;
  state.reserve_valid = ref_state.reserve_valid;
  state.reserve_addr = ref_state.reserve_addr;
  return state;
}

} // namespace

bool refcore_init() {
  g_core = refcpu_init(0, static_cast<uint32_t>(RAM_SIZE));
  if (g_core == nullptr) return false;
  refcpu_set_uart_print(g_core, false);
  return refcpu_get_ram_ptr(g_core) != nullptr && refcpu_get_sdram_ptr(g_core) != nullptr;
}

void refcore_release() {
  if (g_core != nullptr) refcpu_destroy(g_core);
  g_core = nullptr;
  io_clear();
}

uint32_t *refcore_ram_ptr() { return refcpu_get_ram_ptr(core()); }

// RAM, SDRAM and the device registers back to zero (before a checkpoint is restored into them).
void refcore_clear_memory() {
  clear_sparse(refcpu_get_ram_ptr(core()), RAM_SIZE);
  clear_sparse(refcpu_get_sdram_ptr(core()), PMEM_SDRAM_SIZE);
  io_clear();
}

// Device registers of the checkpoint / boot image (timer, interrupt controller, UART, SD controller, boot window), as
// the loaders left them in the model's IO backing, into the core.
void refcore_seed_devices() {
  RefCpuContext *c = core();
  refcpu_clear_io_words(c);
  for (const auto &range : kCheckpointMmioRanges) {
    for (uint32_t off = 0; off + 4u <= range.size; off += 4u) {
      refcpu_set_io_word(c, range.base + off, io_read(range.base + off));
    }
  }
  refcpu_sync_mmio_devices_from_backing(c);
}

void refcore_init_ckpt(CPU_state ckpt_state, uint8_t privilege) {
  RefCpuContext *c = core();
  const RefCpuState ref_state = to_ref_state(ckpt_state, privilege);
  refcpu_set_state(c, &ref_state);
  refcore_seed_devices();
  // only probes a translation when SATP is active; the restored PC need not be translatable yet
  (void)refcpu_probe_pc_translation(c);
}

uint64_t refcore_restore_ckpt(const std::string &path, CPU_state &state, uint8_t &privilege) {
  RefCpuContext *c = core();
  RefCpuState restored{};
  const uint64_t interval = refcpu_restore_checkpoint(c, path.c_str(), false, &restored);
  state = from_ref_state(restored);
  privilege = restored.privilege;
  state.instruction = 0;   // fields a checkpoint does not have
  state.page_fault_inst = state.page_fault_load = state.page_fault_store = false;
  state.inst_idx = 0;
  // the checkpoint's device registers, as stored, into the model's IO backing (the timing model's devices read them)
  for (const auto &range : kCheckpointMmioRanges) {
    for (uint32_t off = 0; off + 4u <= range.size; off += 4u) {
      const uint32_t addr = range.base + off;
      io_write(addr, refcpu_get_io_backing_word(c, addr));
    }
  }
  refcore_init_ckpt(state, privilege);
  return interval;
}

uint64_t refcore_restore_ckpt_memory(const std::string &path) {
  return refcpu_restore_checkpoint(core(), path.c_str(), true, nullptr);
}

// Architectural state of the core for the timing model; its device registers go back to the model's IO backing (the
// RAM is shared, nothing to copy).
void refcore_get_state(CPU_state &dut_state, uint8_t &privilege) {
  RefCpuContext *c = core();
  RefCpuState ref_state{};
  refcpu_get_state(c, &ref_state);
  dut_state = from_ref_state(ref_state);
  privilege = ref_state.privilege;
  for (const auto &range : kCheckpointMmioRanges) {
    for (uint32_t off = 0; off + 4u <= range.size; off += 4u) {
      const uint32_t addr = range.base + off;
      io_write(addr, refcpu_get_io_word(c, addr));
    }
  }
}

// One instruction of the checkpoint prewarm.
void refcore_step() {
  RefCpuContext *c = core();
  refcpu_set_sim_time(c, static_cast<uint64_t>(sim_time));
  refcpu_step(c, 1);
}

// After the prewarm / fast warm-up: the core's clock restarts at 0 together with the timing model's (the timer offset
// the warm-up left in the device model keeps mtime continuous).
void refcore_end_warmup() { refcpu_set_sim_time(core(), 0); }

RefCpuContext *refcore_ctx() { return core(); }

bool refcore_sim_end() {
  RefCpuStepInfo info{};
  refcpu_get_step_info(core(), &info);
  return info.sim_end;
}

bool refcore_last_inst_is_wfi() {
  RefCpuStepInfo info{};
  refcpu_get_step_info(core(), &info);
  return info.instruction == INST_WFI;
}
