#pragma once
#include <cstdint>
// EXP-047 fast DSE driver (driver/DseFork.cpp, docs/DSE_FORK.md).
bool dse_init(uint64_t warmup_target);  // reads DSE_CONFIGS etc.; false when DSE is off
int dse_fork();  // in the parent: forks the children, waits, returns the exit code; in a child: returns -1 (continue)
