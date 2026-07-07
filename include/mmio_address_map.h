#pragma once

#include "PhysMemory.h"
#include "config.h"

#include <cstdint>

struct SimMmioRange {
  uint32_t base;
  uint32_t size;
};

static_assert(RAM_SIZE <= 0x80000000ull,
              "RAM window must fit in the 32-bit physical address space");
static_assert(PMEM_RAM_BASE + RAM_SIZE == 0x100000000ull,
              "DDR window must cover [0x80000000, 0x100000000)");

inline constexpr SimMmioRange kSimulatorMmioRanges[] = {
    {BOOT_IO_BASE, BOOT_IO_SIZE},
    {UART_ADDR_BASE, UART_MMIO_SIZE},
    {XPS_INTC_ADDR_BASE, XPS_INTC_MMIO_SIZE},
    {TIMER_BASE, 0x00000010u},
    {OCSDC_ADDR_BASE, OCSDC_MMIO_SIZE},
    {SIM_UNCACHE_SDRAM_BASE, SIM_UNCACHE_SDRAM_SIZE},
    {SIM_UNCACHE_DDR_RESERVED_BASE, SIM_UNCACHE_DDR_RESERVED_SIZE},
    {SIM_UNCACHE_HIGH_BASE, SIM_UNCACHE_HIGH_SIZE},
};

static constexpr bool simulator_addr_in_range(uint32_t addr,
                                              const SimMmioRange &range) {
  const auto start = static_cast<uint64_t>(range.base);
  const auto end = start + static_cast<uint64_t>(range.size);
  const auto value = static_cast<uint64_t>(addr);
  return value >= start && value < end;
}

static inline bool simulator_is_mmio_addr(uint32_t addr) {
  for (const auto &range : kSimulatorMmioRanges) {
    if (simulator_addr_in_range(addr, range)) {
      return true;
    }
  }
  return false;
}
