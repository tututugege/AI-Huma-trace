#pragma once
#include <cstdint>

namespace mmio {

class MMIO_Device {
public:
  virtual ~MMIO_Device() = default;
  virtual void read(uint32_t addr, uint8_t *data, uint32_t len) = 0;
  virtual void write(uint32_t addr, const uint8_t *data, uint32_t len,
                     uint32_t wstrb) = 0;
  virtual void tick() {}
};

} // namespace mmio
