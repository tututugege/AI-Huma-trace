/**
 * @file UART16550_Device.cpp
 * @brief Minimal 16550-like UART MMIO device (TX-only).
 */

#include "UART16550_Device.h"
#include <cstring>
#include <iostream>

namespace mmio {

static constexpr uint32_t UART_REG_THR = 0x0;
static constexpr uint32_t UART_REG_LSR = 0x5;

static constexpr uint8_t UART_LSR_THRE = 0x20; // Transmit-hold-register empty
static constexpr uint8_t UART_LSR_TEMT = 0x40; // Transmitter empty

UART16550_Device::UART16550_Device(uint32_t base_addr) : base(base_addr) {
  std::memset(regs, 0, sizeof(regs));
}

UART16550_Device::~UART16550_Device() { flush_tx_buffer(false); }

void UART16550_Device::flush_tx_buffer(bool append_newline) {
  if (tx_buffer_.empty() && !append_newline) {
    return;
  }
  std::cout << tx_buffer_;
  if (append_newline) {
    std::cout.put('\n');
  }
  std::cout.flush();
  tx_buffer_.clear();
}

void UART16550_Device::sync_from_backing(const uint32_t *memory) {
  if (memory == nullptr) {
    return;
  }
  const uint32_t word0 = memory[base >> 2];
  const uint32_t word1 = memory[(base >> 2) + 1u];
  regs[0] = static_cast<uint8_t>(word0 & 0xFFu);
  regs[1] = static_cast<uint8_t>((word0 >> 8) & 0xFFu);
  regs[2] = static_cast<uint8_t>((word0 >> 16) & 0xFFu);
  regs[3] = static_cast<uint8_t>((word0 >> 24) & 0xFFu);
  regs[4] = static_cast<uint8_t>(word1 & 0xFFu);
  regs[5] = static_cast<uint8_t>((word1 >> 8) & 0xFFu);
  regs[6] = static_cast<uint8_t>((word1 >> 16) & 0xFFu);
  regs[7] = static_cast<uint8_t>((word1 >> 24) & 0xFFu);
}

void UART16550_Device::read(uint32_t addr, uint8_t *data, uint32_t len) {
  if (!data || len == 0) {
    return;
  }

  std::memset(data, 0, len);

  const uint32_t off = addr - base;
  for (uint32_t i = 0; i < len; i++) {
    const uint32_t reg_off = off + i;
    if (reg_off >= sizeof(regs)) {
      continue;
    }
    uint8_t val = regs[reg_off];
    // Report "always ready to transmit" to avoid software deadlock while still
    // preserving any software-visible sticky bits written by the guest.
    if (reg_off == UART_REG_LSR) {
      val = static_cast<uint8_t>(val | UART_LSR_THRE | UART_LSR_TEMT);
    }
    data[i] = val;
  }
}

void UART16550_Device::write(uint32_t addr, const uint8_t *data, uint32_t len,
                             uint32_t wstrb) {
  if (!data || len == 0) {
    return;
  }

  const uint32_t off0 = addr - base;

  // Handle byte-lane writes; THR is 8-bit at offset 0.
  for (uint32_t i = 0; i < len && i < 32; i++) {
    if (((wstrb >> i) & 1u) == 0) {
      continue;
    }
    const uint32_t off = off0 + i;
    if (off >= sizeof(regs)) {
      continue;
    }
    regs[off] = data[i];
    if (off == UART_REG_THR) {
      const uint8_t ch = data[i];
      // Keep legacy behavior: do not print ESC (27).
      if (ch != 27) {
        if (ch == '\r') {
          continue;
        }
        if (ch == '\n') {
          flush_tx_buffer(true);
        } else {
          tx_buffer_.push_back(static_cast<char>(ch));
          if (tx_buffer_.size() >= 1024u) {
            flush_tx_buffer(false);
          }
        }
      }
      // Match the legacy backing-memory model: TX writes do not remain readable
      // from THR once consumed by the device.
      regs[off] = 0;
    }
  }
}

} // namespace mmio
