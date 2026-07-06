#pragma once

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
    {0x87f00000u, 0x00100000u},
    {0xfff00000u, 0x00100000u},
};

static inline bool simulator_addr_in_range(uint32_t addr,
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
