#pragma once

#include "PhysMemory.h"
#include "config.h"

#include <cstdint>

struct SimMmioRange {
  uint32_t base;
  uint32_t size;
};

inline constexpr SimMmioRange kSimulatorMmioRanges[] = {
    {UART_ADDR_BASE, UART_MMIO_SIZE},
    {XPS_INTC_ADDR_BASE, XPS_INTC_MMIO_SIZE},
    {TIMER_BASE, 0x00000010u},
    {0x1fe10000u, 0x00000064u},
    {0x4ff00000u, 0x00100000u},
    {0xfff00000u, 0x00100000u},
};

static constexpr bool simulator_ranges_overlap(uint32_t lhs_base,
                                               uint64_t lhs_size,
                                               uint32_t rhs_base,
                                               uint64_t rhs_size) {
  if (lhs_size == 0 || rhs_size == 0) {
    return false;
  }
  const uint64_t lhs_start = static_cast<uint64_t>(lhs_base);
  const uint64_t lhs_end = lhs_start + lhs_size;
  const uint64_t rhs_start = static_cast<uint64_t>(rhs_base);
  const uint64_t rhs_end = rhs_start + rhs_size;
  return lhs_start < rhs_end && rhs_start < lhs_end;
}

static constexpr bool simulator_mmio_range_overlaps_ram(
    const SimMmioRange &range) {
  return simulator_ranges_overlap(range.base, range.size, PMEM_RAM_BASE,
                                  RAM_SIZE);
}

static constexpr bool simulator_mmio_ranges_overlap_ram() {
  for (const auto &range : kSimulatorMmioRanges) {
    if (simulator_mmio_range_overlaps_ram(range)) {
      return true;
    }
  }
  return false;
}

static_assert(!simulator_mmio_ranges_overlap_ram(),
              "simulator MMIO ranges must not overlap RAM");

static constexpr bool simulator_addr_in_range(uint32_t addr,
                                              const SimMmioRange &range) {
  const auto start = static_cast<uint64_t>(range.base);
  const auto end = start + static_cast<uint64_t>(range.size);
  const auto value = static_cast<uint64_t>(addr);
  return value >= start && value < end;
}

static inline bool simulator_is_mmio_addr(uint32_t addr) {
  if (simulator_addr_in_range(
          addr, SimMmioRange{PMEM_RAM_BASE, static_cast<uint32_t>(RAM_SIZE)})) {
    return false;
  }
  for (const auto &range : kSimulatorMmioRanges) {
    if (simulator_addr_in_range(addr, range)) {
      return true;
    }
  }
  return false;
}
