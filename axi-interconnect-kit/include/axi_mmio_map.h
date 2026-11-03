#pragma once

#include "axi_interconnect_compat.h"
#include <cstdint>

struct AxiMmioRange {
  uint32_t base;
  uint32_t size;
};

static constexpr AxiMmioRange kAxiMmioRanges[] = {
    {MMIO_BASE, MMIO_SIZE},
};

#ifndef MMIO_RANGE_BASE
#define MMIO_RANGE_BASE MMIO_BASE
#endif

#ifndef MMIO_RANGE_SIZE
#define MMIO_RANGE_SIZE MMIO_SIZE
#endif

static inline bool is_mmio_addr(uint32_t addr) {
  for (const auto &range : kAxiMmioRanges) {
    if (addr >= range.base && addr < (range.base + range.size)) {
      return true;
    }
  }
  return false;
}
