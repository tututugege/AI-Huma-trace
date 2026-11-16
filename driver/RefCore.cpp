// The reference core is the trace bridge's own core (pmem_core_ctx(): the librefcpu instance whose RAM is the model's
// functional memory). This file prepares it for an image or a checkpoint, steps it through the checkpoint prewarm and
// hands its architectural state to the timing model. There is no second core and no comparison: the timing model
// computes no values, the bridge (TraceBridge.cpp) supplies them and TB_CHECK compares what the model still decides.
#include "RefCore.h"
#include "Csr.h"
#include "PhysMemory.h"
#include "RISCV.h"
#include "config.h"
#include "mmio_address_map.h"
#include "util.h"

#include <cstring>
#include <iostream>

namespace {

RefCpuContext *core() {
  RefCpuContext *c = pmem_core_ctx();
  Assert(c != nullptr && "reference core: pmem_init() has not run");
  return c;
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

// Device registers of the checkpoint / boot image (timer, interrupt controller, UART, SD controller, boot window), as
// the loaders left them in the model's IO backing, into the core.
void refcore_seed_devices() {
  RefCpuContext *c = core();
  refcpu_clear_io_words(c);
  for (const auto &range : kCheckpointMmioRanges) {
    for (uint32_t off = 0; off + 4u <= range.size; off += 4u) {
      refcpu_set_io_word(c, range.base + off, pmem_read(range.base + off));
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
      pmem_write(addr, refcpu_get_io_word(c, addr));
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
