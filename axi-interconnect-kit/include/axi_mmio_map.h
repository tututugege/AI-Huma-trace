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

