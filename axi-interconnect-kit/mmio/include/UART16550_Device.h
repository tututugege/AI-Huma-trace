#pragma once

#include "MMIO_Device.h"
#include <cstdint>
#include <string>

namespace mmio {

// Minimal 16550-like UART model for QEMU-virt style mapping at 0x1000_0000.
// - THR (offset 0): write prints a byte to stdout
// - LSR (offset 5): read always reports TX ready (0x60)
class UART16550_Device final : public MMIO_Device {
public:
  explicit UART16550_Device(uint32_t base_addr);
  ~UART16550_Device() override;
  void sync_from_backing(const uint32_t *memory);

  void read(uint32_t addr, uint8_t *data, uint32_t len) override;
  void write(uint32_t addr, const uint8_t *data, uint32_t len,
             uint32_t wstrb) override;

private:
  void flush_tx_buffer(bool append_newline);

  uint32_t base;
  uint8_t regs[8] = {};
  std::string tx_buffer_;
};

} // namespace mmio
