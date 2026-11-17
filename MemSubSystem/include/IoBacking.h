#pragma once
// Device registers: the words of the MMIO window (boot window, UART, interrupt controller, timer, SD controller) as the
// timing model's devices see them. The loaders seed them, the peripheral model and the AXI MMIO path read and write
// them, the reference core is synchronized with them at load / checkpoint time (driver/RefCore.cpp). Not memory: the RAM
// is the function model's (refcore).
#include <cstdint>

uint32_t io_read(uint32_t paddr);              // a register that was never written reads 0
void io_write(uint32_t paddr, uint32_t data);
void io_clear();
