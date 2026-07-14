#include "diff.h"
#include "Csr.h"
#include "DiffMemTrace.h"
#include "PhysMemory.h"
#include "RISCV.h"
#include "config.h"
#include "util.h"

#include <algorithm>
#include <cstring>
#include <iostream>

CPU_state dut_cpu;
static RefCpuContext *ref_cpu_ctx = nullptr;
static uint8_t dut_privilege = RISCV_MODE_M;

namespace {
struct IoRange {
  uint32_t base;
  uint32_t size;
};

constexpr IoRange kCkptIoRanges[] = {
    {BOOT_IO_BASE, BOOT_IO_SIZE},
    {UART_ADDR_BASE, UART_MMIO_SIZE},
    {XPS_INTC_ADDR_BASE, XPS_INTC_MMIO_SIZE},
    {OPENSBI_TIMER_BASE, OPENSBI_TIMER_MMIO_SIZE},
    {OCSDC_ADDR_BASE, OCSDC_MMIO_SIZE},
};

std::string g_sdcard_image_path;
uint64_t g_diff_total_steps = 0;

RefCpuState to_ref_state(const CPU_state &state, uint8_t privilege) {
  RefCpuState ref_state{};
  std::memcpy(ref_state.gpr, state.gpr, sizeof(ref_state.gpr));
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

void ensure_ref_cpu() {
  if (ref_cpu_ctx == nullptr) {
    ref_cpu_ctx = refcpu_init(0, static_cast<uint32_t>(RAM_SIZE));
    Assert(ref_cpu_ctx != nullptr && "refcpu_init failed");
  }
}

void configure_ref_runtime() {
  ensure_ref_cpu();
  refcpu_set_uart_print(ref_cpu_ctx, false);
  refcpu_set_ref_only(ref_cpu_ctx, true);
  refcpu_set_device_effects(ref_cpu_ctx, false);
  refcpu_set_interrupt_delivery(ref_cpu_ctx, false);
}

RefCpuState current_ref_state() {
  ensure_ref_cpu();
  RefCpuState state{};
  refcpu_get_state(ref_cpu_ctx, &state);
  return state;
}

RefCpuStepInfo current_step_info() {
  ensure_ref_cpu();
  RefCpuStepInfo info{};
  refcpu_get_step_info(ref_cpu_ctx, &info);
  return info;
}

inline void seed_ref_io_from_backing() {
  ensure_ref_cpu();
  refcpu_clear_io_words(ref_cpu_ctx);
  for (const auto &range : kCkptIoRanges) {
    for (uint32_t off = 0; off + 4u <= range.size; off += 4u) {
      const uint32_t addr = range.base + off;
      const uint32_t data = pmem_read(addr);
      refcpu_set_io_word(ref_cpu_ctx, addr, data);
    }
  }
}

void diff_watchdog_tick() {
  g_diff_total_steps++;
}

} // namespace

// relocate the init_difftest function to avoid multiple definition error
void init_difftest(int img_size) {
  if (ref_cpu_ctx != nullptr) {
    refcpu_destroy(ref_cpu_ctx);
  }
  ref_cpu_ctx = refcpu_init(0, static_cast<uint32_t>(RAM_SIZE));
  Assert(ref_cpu_ctx != nullptr && "init_difftest: refcpu_init failed");
  configure_ref_runtime();
  refcpu_sync_ram_from_dut(ref_cpu_ctx, pmem_ram_ptr(),
                           static_cast<size_t>(img_size));
  seed_ref_io_from_backing();
}

void difftest_configure_run_assets(const char *sdcard_image_path) {
  g_sdcard_image_path = (sdcard_image_path == nullptr) ? "" : sdcard_image_path;
}

void difftest_init_run_state() {
  if (ref_cpu_ctx != nullptr) {
    refcpu_destroy(ref_cpu_ctx);
  }
  ref_cpu_ctx = refcpu_init(0, static_cast<uint32_t>(RAM_SIZE));
  Assert(ref_cpu_ctx != nullptr && "difftest_init_run_state: refcpu_init failed");
  configure_ref_runtime();
  if (!g_sdcard_image_path.empty()) {
    refcpu_load_sdcard_image(ref_cpu_ctx, g_sdcard_image_path.c_str());
  }
  refcpu_sync_ram_from_dut(ref_cpu_ctx, pmem_ram_ptr(),
                           static_cast<size_t>(RAM_SIZE));
  seed_ref_io_from_backing();
}

void init_diff_ckpt(CPU_state ckpt_state, uint8_t privilege) {
  std::cout << "Restore for ref cpu " << std::endl;
  if (ref_cpu_ctx != nullptr) {
    refcpu_destroy(ref_cpu_ctx);
  }
  ref_cpu_ctx = refcpu_init(0, static_cast<uint32_t>(RAM_SIZE));
  Assert(ref_cpu_ctx != nullptr && "init_diff_ckpt: refcpu_init failed");
  configure_ref_runtime();
  RefCpuState ref_state = to_ref_state(ckpt_state, privilege);
  refcpu_set_state(ref_cpu_ctx, &ref_state);

  const uint32_t *ckpt_memory = pmem_ram_ptr();
  Assert(ckpt_memory != nullptr &&
         "init_diff_ckpt: pmem RAM backend is not initialized");
  refcpu_sync_ram_from_dut(ref_cpu_ctx, ckpt_memory,
                           static_cast<size_t>(RAM_SIZE));
  seed_ref_io_from_backing();

  // Keep checkpoint bootstrap aligned with GenSimpoint ref execution: only probe a
  // translation when SATP is active, and do not require the restored PC to be
  // immediately translatable at init time.
  (void)refcpu_probe_pc_translation(ref_cpu_ctx);
}

void get_state(CPU_state &dut_state, uint8_t &privilege) {
  const RefCpuState ref_state = current_ref_state();
  dut_state = from_ref_state(ref_state);
  privilege = ref_state.privilege;
  uint32_t *dut_memory = pmem_ram_ptr();
  Assert(dut_memory != nullptr &&
         "get_state: pmem RAM backend is not initialized");
  uint32_t *ref_memory = refcpu_get_ram_ptr(ref_cpu_ctx);
  Assert(ref_memory != nullptr && "get_state: refcpu RAM is not initialized");
  std::memcpy(dut_memory, ref_memory, static_cast<size_t>(RAM_SIZE));
  for (const auto &range : kCkptIoRanges) {
    for (uint32_t off = 0; off + 4u <= range.size; off += 4u) {
      const uint32_t addr = range.base + off;
      pmem_write(addr, refcpu_get_io_word(ref_cpu_ctx, addr));
    }
  }
}

static void checkregs() {
  int i;
  const RefCpuState ref_state = current_ref_state();
  const RefCpuStepInfo step_info = current_step_info();
  uint32_t ref_csrs[CSR_NUM];
  uint32_t dut_csrs[CSR_NUM];
  std::memcpy(ref_csrs, ref_state.csr, sizeof(ref_csrs));
  std::memcpy(dut_csrs, dut_cpu.csr, sizeof(dut_csrs));
  constexpr uint32_t kAsyncPendingMask = MIP_MTIP | MIP_MEIP | MIP_SEIP;
  ref_csrs[csr_mip] &= ~kAsyncPendingMask;
  dut_csrs[csr_mip] &= ~kAsyncPendingMask;
  ref_csrs[csr_sip] = ref_csrs[csr_mip] & 0x00000333u;
  dut_csrs[csr_sip] = dut_csrs[csr_mip] & 0x00000333u;

  if (ref_state.privilege != dut_privilege)
    goto fault;

  if (ref_state.pc != dut_cpu.pc)
    goto fault;

  // 如果没有指令缺页异常，且指令不匹配，报错
  if (!step_info.page_fault_inst && step_info.instruction != dut_cpu.instruction)
    goto fault;

  if (step_info.page_fault_inst != dut_cpu.page_fault_inst)
    goto fault;
  if (step_info.page_fault_load != dut_cpu.page_fault_load)
    goto fault;
  if (step_info.page_fault_store != dut_cpu.page_fault_store)
    goto fault;

  // 通用寄存器
  for (i = 0; i < 32; i++) {
    if (ref_state.gpr[i] != dut_cpu.gpr[i])
      goto fault;
  }

  // csr
  for (i = 0; i < CSR_NUM; i++) {
    if (ref_csrs[i] != dut_csrs[i])
      goto fault;
  }

  if (dut_cpu.store != ref_state.store)
    goto fault;
  if (ref_state.store) {
    if (dut_cpu.store_data != ref_state.store_data)
      goto fault;

    if (dut_cpu.store_addr != ref_state.store_addr)
      goto fault;
  }

  return;

fault:
  cout << "Difftest: error" << endl;
  cout << "cycle: " << dec << sim_time << endl;
  const uint64_t suggested_fast_forward =
      g_diff_total_steps > 100000 ? (g_diff_total_steps - 100000) : 0;
  std::printf(
      "[DIFF][COMMIT_COUNT] committed=%llu suggested_fast_forward=%llu\n",
      static_cast<unsigned long long>(g_diff_total_steps),
      static_cast<unsigned long long>(suggested_fast_forward));

  auto print_mismatch = [](const char *name, uint32_t ref, uint32_t dut) {
    printf("%10s:\t%08x\t%08x%s\n", name, ref, dut,
           (ref != dut ? "\t Error" : ""));
  };

  printf("        PC:\t%08x\t%08x\n", ref_state.pc, dut_cpu.pc);
  printf(" privilege:\t%u\t%u%s\n", ref_state.privilege, dut_privilege,
         (ref_state.privilege != dut_privilege ? "\t Error" : ""));
  cout << "\t\tReference\tDut" << endl;
  for (int i = 0; i < 32; i++) {
    print_mismatch(reg_names[i].c_str(), ref_state.gpr[i], dut_cpu.gpr[i]);
  }

  cout << endl;
  for (int i = 0; i < CSR_NUM; i++) {
    print_mismatch(csr_names[i].c_str(), ref_csrs[i], dut_csrs[i]);
  }

  cout << endl;
  print_mismatch("store", ref_state.store, dut_cpu.store);
  print_mismatch("data", ref_state.store_data, dut_cpu.store_data);
  print_mismatch("addr", ref_state.store_addr, dut_cpu.store_addr);

  printf("Ref Inst: %08x\tDUT Inst: %08x\n", step_info.instruction,
         dut_cpu.instruction);
  std::printf("Commit PC: 0x%08x\tDUT next PC: 0x%08x\tREF next PC: 0x%08x\n",
              dut_cpu.commit_pc, dut_cpu.pc, ref_state.pc);
#if defined(LOG_ENABLE) && defined(LOG_LSU_MEM_ENABLE)
  diff_mem_trace::dump_recent();
#endif

  Assert(0 && "Difftest: Register or Memory mismatch detected.");
  exit(1);
}

void difftest_sync_ram_from_dut(uint32_t paddr, size_t size) {
  ensure_ref_cpu();
  if (size == 0) {
    return;
  }
  Assert(size <= UINT32_MAX &&
         pmem_is_ram_addr(paddr, static_cast<uint32_t>(size)) &&
         "difftest DMA sync range is outside DUT RAM");
  uint32_t *dut_ram = pmem_ram_ptr();
  uint32_t *ref_ram = refcpu_get_ram_ptr(ref_cpu_ctx);
  Assert(dut_ram != nullptr && ref_ram != nullptr &&
         "difftest DMA sync RAM is not initialized");
  const size_t offset = static_cast<size_t>(paddr - PMEM_RAM_BASE);
  std::memcpy(reinterpret_cast<uint8_t *>(ref_ram) + offset,
              reinterpret_cast<const uint8_t *>(dut_ram) + offset, size);
}

void difftest_ref_set_next_mmio_read(uint32_t paddr, uint32_t result) {
  ensure_ref_cpu();
  refcpu_set_next_mmio_read(ref_cpu_ctx, paddr, result);
}

void difftest_ref_set_next_csr_read(uint32_t csr_addr, uint32_t value) {
  ensure_ref_cpu();
  refcpu_set_next_csr_read(ref_cpu_ctx, csr_addr, value);
}

void difftest_take_interrupt(uint32_t cause, uint8_t target_privilege,
                             uint32_t pending_snapshot) {
  ensure_ref_cpu();
  dut_privilege = target_privilege;
  const bool taken = refcpu_take_forced_interrupt(
      ref_cpu_ctx, cause, target_privilege, pending_snapshot);
  Assert(taken && "reference rejected DUT forced interrupt");
  checkregs();
}

void difftest_set_dut_privilege(uint8_t privilege) { dut_privilege = privilege; }

void difftest_step(bool check) {
  ensure_ref_cpu();
  if (!check) {
    refcpu_set_sim_time(ref_cpu_ctx, static_cast<uint64_t>(sim_time));
  }
  refcpu_step(ref_cpu_ctx, 1);
  diff_watchdog_tick();
  if (check)
    checkregs();
}

bool difftest_ref_sim_end() { return current_step_info().sim_end; }

bool difftest_ref_last_inst_is_wfi() {
  return current_step_info().instruction == INST_WFI;
}

void difftest_ref_set_uart_print(bool enable) {
  ensure_ref_cpu();
  refcpu_set_uart_print(ref_cpu_ctx, enable);
}

void difftest_ref_set_ref_only(bool enable) {
  ensure_ref_cpu();
  refcpu_set_ref_only(ref_cpu_ctx, enable);
}

void difftest_ref_set_device_effects(bool enable) {
  ensure_ref_cpu();
  refcpu_set_device_effects(ref_cpu_ctx, enable);
}

void difftest_ref_set_interrupt_delivery(bool enable) {
  ensure_ref_cpu();
  refcpu_set_interrupt_delivery(ref_cpu_ctx, enable);
}
