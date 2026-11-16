#pragma once
// Loading a program image or a checkpoint (driver/Load.cpp).
#include <cstdint>
#include <string>

class BackTop;

// RUN mode: the image at the RAM base, the boot stub at 0, the device defaults.
void load_image(const std::string &filename);
// Back-door restore: memory, devices and the CPU state of the checkpoint (the function model and the timing model's
// architectural registers). Returns the checkpoint's interval length (0 for a special checkpoint).
uint64_t restore_checkpoint(BackTop &back, const std::string &filename);
// Front-door restore: only the checkpoint's memory, and the restore program (stub) in the boot window; the core starts
// from reset and executes it. Returns the interval length.
uint64_t restore_checkpoint_front_door(const std::string &filename, const std::string &stub_file);
// After the checkpoint prewarm: the function model's state into the timing model.
void restore_from_ref(BackTop &back);
