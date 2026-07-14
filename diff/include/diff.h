#pragma once
#include "ref.h"
#include "refcpu_api.h"
#include <cstddef>
#include <cstdint>

enum { DIFFTEST_TO_DUT, DIFFTEST_TO_REF };

extern CPU_state dut_cpu;
class SimContext;

void init_difftest(int);
void difftest_configure_run_assets(const char *sdcard_image_path);
void difftest_init_run_state();
void init_diff_ckpt(CPU_state ckpt_state, uint8_t privilege);
void get_state(CPU_state &dut_state, uint8_t &privilege);
void difftest_step(bool);
void difftest_set_dut_privilege(uint8_t privilege);
void difftest_sync_ram_from_dut(uint32_t paddr, size_t size);
void difftest_ref_set_next_mmio_read(uint32_t paddr, uint32_t result);
void difftest_ref_set_next_csr_read(uint32_t csr_addr, uint32_t value);
void difftest_take_interrupt(uint32_t cause, uint8_t target_privilege,
                             uint32_t pending_snapshot);
bool difftest_ref_sim_end();
bool difftest_ref_last_inst_is_wfi();
void difftest_ref_set_uart_print(bool enable);
void difftest_ref_set_ref_only(bool enable);
void difftest_ref_set_device_effects(bool enable);
void difftest_ref_set_interrupt_delivery(bool enable);
