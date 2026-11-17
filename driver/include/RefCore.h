#pragma once
// The reference core: the trace bridge's librefcpu core, set up for an image / a checkpoint, stepped through the
// checkpoint prewarm and fast warm-up (FastWarm.cpp), and the hand-over of its state to the timing model (RefCore.cpp).
#include "ref.h"
#include "refcpu_api.h"
#include <cstddef>
#include <cstdint>
#include <string>

bool refcore_init();          // creates the core (its RAM is the model's functional memory)
void refcore_release();
uint32_t *refcore_ram_ptr();   // the DDR window (an image is loaded straight into it)
void refcore_clear_memory();   // RAM, SDRAM and the device registers to zero
void refcore_seed_devices();
void refcore_init_ckpt(CPU_state ckpt_state, uint8_t privilege);
// A checkpoint into the core (refcpu_restore_checkpoint): memory, devices, CPU state; the device registers also into
// the model's IO backing. state / privilege: as the checkpoint has them. Returns the interval length.
uint64_t refcore_restore_ckpt(const std::string &path, CPU_state &state, uint8_t &privilege);
// Only the checkpoint's memory (a front-door restore). Returns the interval length.
uint64_t refcore_restore_ckpt_memory(const std::string &path);
void refcore_get_state(CPU_state &dut_state, uint8_t &privilege);
void refcore_step();
void refcore_end_warmup();
RefCpuContext *refcore_ctx();
bool refcore_sim_end();
bool refcore_last_inst_is_wfi();
