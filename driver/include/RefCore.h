#pragma once
// The reference core: the trace bridge's librefcpu core, set up for an image / a checkpoint, stepped through the
// checkpoint prewarm and fast warm-up (FastWarm.cpp), and the hand-over of its state to the timing model (RefCore.cpp).
#include "ref.h"
#include "refcpu_api.h"
#include <cstddef>
#include <cstdint>

void refcore_seed_devices();
void refcore_init_ckpt(CPU_state ckpt_state, uint8_t privilege);
void refcore_get_state(CPU_state &dut_state, uint8_t &privilege);
void refcore_step();
void refcore_end_warmup();
RefCpuContext *refcore_ctx();
bool refcore_sim_end();
bool refcore_last_inst_is_wfi();
