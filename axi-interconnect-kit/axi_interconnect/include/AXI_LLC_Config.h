#pragma once

#include <cstdint>

namespace axi_interconnect {

struct AXI_LLCConfig {
  bool enable = false;
  uint64_t size_bytes = 8ull << 20;
  // Mainline simulator profiles keep the line shape and policy fixed.
  uint32_t line_bytes = 64;
  uint32_t ways = 16;
  uint32_t mshr_num = 8;
  uint32_t lookup_latency = 3;
  bool prefetch_enable = false;
  uint32_t prefetch_degree = 1;
  bool nine = true;
  bool unified = true;
  bool pipt = true;

  bool valid() const {
    return line_bytes != 0 && ways != 0 && mshr_num != 0 &&
           (size_bytes % (static_cast<uint64_t>(line_bytes) * ways)) == 0;
  }

  uint32_t set_count() const {
    if (!valid()) {
      return 0;
    }
    return static_cast<uint32_t>(
        size_bytes / (static_cast<uint64_t>(line_bytes) * ways));
  }
};

} // namespace axi_interconnect
