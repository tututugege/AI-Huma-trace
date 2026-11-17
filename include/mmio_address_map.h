#pragma once

#include "config.h"

#include <cstdint>

// The physical memory map: DDR at 0x80000000 (RAM_SIZE), SDRAM at 0x40000000; the rest is the MMIO window.
constexpr uint32_t PMEM_SDRAM_BASE = 0x40000000u;
constexpr uint32_t PMEM_SDRAM_SIZE = 0x10000000u;
constexpr uint32_t PMEM_RAM_BASE = 0x80000000u;

inline bool pmem_range_within(uint32_t paddr, uint32_t size, uint64_t base, uint64_t span) {
  if (size == 0 || static_cast<uint64_t>(paddr) < base) return false;
  return static_cast<uint64_t>(paddr) + size - 1u < base + span;
}
inline bool is_sdram_addr(uint32_t paddr, uint32_t size = 4u) { return pmem_range_within(paddr, size, PMEM_SDRAM_BASE, PMEM_SDRAM_SIZE); }
inline bool is_ram_addr(uint32_t paddr, uint32_t size = 4u) {   // DDR or SDRAM
  return is_sdram_addr(paddr, size) || pmem_range_within(paddr, size, PMEM_RAM_BASE, RAM_SIZE);
}

struct SimMmioRange {
  uint32_t base;
  uint32_t size;
};

static_assert(RAM_SIZE <= 0x80000000ull,
              "RAM window must fit in the 32-bit physical address space");
static_assert(PMEM_RAM_BASE + RAM_SIZE == 0x100000000ull,
              "DDR window must cover [0x80000000, 0x100000000)");
static_assert(PMEM_SDRAM_BASE + PMEM_SDRAM_SIZE == 0x50000000ull,
              "SDRAM window must cover [0x40000000, 0x50000000)");

inline constexpr SimMmioRange kCheckpointMmioRanges[] = {
    {BOOT_IO_BASE, BOOT_IO_SIZE},
    {UART_ADDR_BASE, UART_MMIO_SIZE},
    {XPS_INTC_ADDR_BASE, XPS_INTC_MMIO_SIZE},
    {TIMER_BASE, TIMER_MMIO_SIZE},
    {OCSDC_ADDR_BASE, OCSDC_MMIO_SIZE},
};


static inline bool simulator_is_cache_addr(uint32_t addr) {
  return (addr >= SIM_CACHE_ADDR_1_MIN && addr < SIM_CACHE_ADDR_1_MAX) || addr >= SIM_CACHE_ADDR_2_MIN;
}


static inline bool simulator_is_mmio_addr(uint32_t addr) { return !simulator_is_cache_addr(addr); }
