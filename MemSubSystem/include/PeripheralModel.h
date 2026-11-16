#pragma once

#include "IO.h"
#include "PhysMemory.h"
#include "config.h"
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <vector>

extern long long sim_time;

class PeripheralModel {
public:
  static constexpr uint32_t kUartRegRxTxDll = 0x0u;
  static constexpr uint32_t kUartRegIerDlm = 0x1u;
  static constexpr uint32_t kUartRegIirFcr = 0x2u;
  static constexpr uint32_t kUartRegLsr = 0x5u;
  static constexpr uint8_t kUartLcrDlab = 0x80u;
  static constexpr uint8_t kUartLsrThre = 0x20u;
  static constexpr uint8_t kUartLsrTemt = 0x40u;
  static constexpr uint8_t kUartIerThri = 0x02u;
  static constexpr uint8_t kUartIirNoInt = 0x01u;
  static constexpr uint8_t kUartIirThre = 0x02u;
  static constexpr uint8_t kUartIirFifoEnabled = 0xc0u;

  CsrInterruptInjectIO *csr_interrupt_inject = nullptr;   // the interrupt lines into the CSR (PeripheralAxi's out.csr_interrupt_inject)
  uint64_t timer_offset_ = 0;
  uint64_t timer_cmp_ = UINT64_MAX;

  void configure_interrupt_latency(uint64_t seed, uint32_t min_cycles,
                                   uint32_t max_cycles) {
    irq_delay_min_cycles_ = min_cycles == 0 ? 1u : min_cycles;
    irq_delay_max_cycles_ =
        max_cycles < irq_delay_min_cycles_ ? irq_delay_min_cycles_ : max_cycles;
    irq_rng_state_ = seed == 0 ? 1u : seed;
    timer_irq_delay_ = {};
    external_irq_delay_ = {};
  }

  void reset_devices() {   // (after csr_interrupt_inject is bound)
    timer_reset();
    xps_intc_reset();
  }

  void sync_from_backing() {
    const uint64_t timer_value =
        static_cast<uint64_t>(pmem_read(OPENSBI_TIMER_LOW_ADDR)) |
        (static_cast<uint64_t>(pmem_read(OPENSBI_TIMER_HIGH_ADDR)) << 32);
    const uint64_t now = sim_time >= 0 ? static_cast<uint64_t>(sim_time) : 0ull;
    // Preserve the serialized 64-bit timer value modulo 2^64. Saturating here
    // would change when the next compare interrupt becomes pending.
    timer_offset_ = timer_value - now;
    timer_cmp_ =
        static_cast<uint64_t>(pmem_read(OPENSBI_TIMERCMP_LOW_ADDR)) |
        (static_cast<uint64_t>(pmem_read(OPENSBI_TIMERCMP_HIGH_ADDR)) << 32);

    xps_intc_isr_ = pmem_read(XPS_INTC_ADDR_BASE + 0x00);
    xps_intc_ier_ = pmem_read(XPS_INTC_ADDR_BASE + 0x08);
    xps_intc_mer_ = pmem_read(XPS_INTC_ADDR_BASE + 0x1c);

    update_external_irq();
    update_timer_irq();
  }

  static bool is_modeled_mmio(uint32_t paddr) {
    const auto in_range = [](uint32_t addr, uint32_t base, uint32_t size) {
      if (addr < base) {
        return false;
      }
      return static_cast<uint64_t>(addr - base) < static_cast<uint64_t>(size);
    };
    return in_range(paddr, BOOT_IO_BASE, BOOT_IO_SIZE) ||
           in_range(paddr, UART_ADDR_BASE, UART_MMIO_SIZE) ||
           in_range(paddr, XPS_INTC_ADDR_BASE, XPS_INTC_MMIO_SIZE) ||
           in_range(paddr, OPENSBI_TIMER_BASE, OPENSBI_TIMER_MMIO_SIZE);
  }

  // 64-bit value of a load (rv64 ld reads two aligned words; everything else is read_load())
  uint64_t read_load64(uint32_t paddr, uint8_t func3) const {
    if ((func3 & 0x3u) == 0x3u) {
      return static_cast<uint64_t>(read_load(paddr, 2)) | (static_cast<uint64_t>(read_load(paddr + 4u, 2)) << 32);
    }
    return read_load(paddr, func3);
  }

  uint32_t read_load(uint32_t paddr, uint8_t func3) const {
    if (!is_modeled_mmio(paddr)) {
      return 0;
    }

    if (paddr == OPENSBI_TIMER_LOW_ADDR) {
      return static_cast<uint32_t>(effective_timer_value());
    }
    if (paddr == OPENSBI_TIMER_HIGH_ADDR) {
      return static_cast<uint32_t>(effective_timer_value() >> 32);
    }
    if (paddr == OPENSBI_TIMERCMP_LOW_ADDR) {
      return static_cast<uint32_t>(timer_cmp_);
    }
    if (paddr == OPENSBI_TIMERCMP_HIGH_ADDR) {
      return static_cast<uint32_t>(timer_cmp_ >> 32);
    }
    uint32_t data = 0;
    if (paddr >= XPS_INTC_ADDR_BASE &&
               paddr < (XPS_INTC_ADDR_BASE + XPS_INTC_MMIO_SIZE)) {
      data = xps_intc_read_reg(paddr - XPS_INTC_ADDR_BASE);
    } else {
      const uint32_t shift = (paddr & 0x3u) * 8u;
      data = pmem_read(paddr) >> shift;
    }

    const uint32_t size = func3 & 0x3u;
    uint32_t sign = 0;
    uint32_t mask = 0xffffffffu;
    if (size == 0u) {
      mask = 0xffu;
      if (data & 0x80u) {
        sign = 0xffffff00u;
      }
    } else if (size == 1u) {
      mask = 0xffffu;
      if (data & 0x8000u) {
        sign = 0xffff0000u;
      }
    }

    data &= mask;
    if ((func3 & 0x4u) == 0) {
      data |= sign;
    }
    return data;
  }

  void on_commit_store(uint32_t paddr, uint64_t data64, uint8_t func3) {
    if (csr_interrupt_inject == nullptr || !is_modeled_mmio(paddr)) {
      return;
    }
    // rv64 sd: the devices have 32-bit registers, an 8-byte store is two aligned word stores (OpenSBI writes mtimecmp with one)
    if ((func3 & 0x3u) == 0x3u) {
      on_commit_store(paddr, static_cast<uint32_t>(data64), 2);
      on_commit_store(paddr + 4u, static_cast<uint32_t>(data64 >> 32), 2);
      return;
    }
    const uint32_t data = static_cast<uint32_t>(data64);

    if (paddr == OPENSBI_TIMER_LOW_ADDR || paddr == OPENSBI_TIMER_HIGH_ADDR) {
      const uint64_t current_timer = effective_timer_value();
      uint64_t new_timer = current_timer;
      if (paddr == OPENSBI_TIMER_LOW_ADDR) {
        new_timer = (current_timer & 0xFFFFFFFF00000000ull) |
                    static_cast<uint64_t>(data);
      } else {
        new_timer = (static_cast<uint64_t>(data) << 32) |
                    (current_timer & 0x00000000FFFFFFFFull);
      }
      timer_offset_ = new_timer - current_sim_time_u64();
      update_timer_irq();
      return;
    }

    if (paddr == OPENSBI_TIMERCMP_LOW_ADDR) {
      timer_cmp_ =
          (timer_cmp_ & 0xFFFFFFFF00000000ull) | static_cast<uint64_t>(data);
      update_timer_irq();
      return;
    }

    if (paddr == OPENSBI_TIMERCMP_HIGH_ADDR) {
      timer_cmp_ = (static_cast<uint64_t>(data) << 32) |
                   (timer_cmp_ & 0x00000000FFFFFFFFull);
      update_timer_irq();
      return;
    }

    if (paddr >= XPS_INTC_ADDR_BASE &&
        paddr < (XPS_INTC_ADDR_BASE + XPS_INTC_MMIO_SIZE)) {
      const uint32_t reg_off = paddr - XPS_INTC_ADDR_BASE;
      const uint32_t shift = (paddr & 0x3u) * 8u;
      const uint32_t store_data = mask_store_data(data, func3);
      const uint32_t mask = mask_store_mask(func3) << shift;
      const uint32_t wdata = store_data << shift;
      const uint32_t old_data = xps_intc_read_reg(reg_off & ~0x3u);
      const uint32_t new_data = (old_data & ~mask) | (wdata & mask);
      xps_intc_write_reg(reg_off & ~0x3u, new_data);
      return;
    }


    const uint32_t shift = (paddr & 0x3u) * 8u;
    const uint32_t store_data = mask_store_data(data, func3);
    const uint32_t mask = mask_store_mask(func3) << shift;
    const uint32_t wdata = store_data << shift;
    const uint32_t old_data = pmem_read(paddr);
    pmem_write(paddr, (old_data & ~mask) | (wdata & mask));

    const bool is_uart =
        paddr >= UART_ADDR_BASE && paddr < (UART_ADDR_BASE + UART_MMIO_SIZE);
    const uint8_t uart_lcr =
        is_uart ? static_cast<uint8_t>((pmem_read(UART_ADDR_BASE) >> 24) & 0xffu)
                : 0u;
    const bool uart_dlab = (uart_lcr & kUartLcrDlab) != 0;

    if (paddr == (UART_ADDR_BASE + kUartRegRxTxDll)) {
      if (uart_dlab) {
        return;
      }
      pmem_write(UART_ADDR_BASE, pmem_read(UART_ADDR_BASE) & 0xffffff00u);
      const uint32_t lsr_value =
          static_cast<uint32_t>(kUartLsrThre | kUartLsrTemt) << 8;
      pmem_write(UART_ADDR_BASE + 4u,
                 (pmem_read(UART_ADDR_BASE + 4u) & 0xffff00ffu) | lsr_value);
      char temp = static_cast<char>(data & 0xffu);
      std::cout << temp << std::flush;
      uart_refresh_interrupt();
      return;
    }

    if (paddr == (UART_ADDR_BASE + kUartRegIerDlm)) {
      if (uart_dlab) {
        return;
      }
      uart_refresh_interrupt();
      return;
    }

    if (paddr == (UART_ADDR_BASE + kUartRegIirFcr)) {
      const uint32_t iir_value =
          static_cast<uint32_t>(kUartIirNoInt | kUartIirFifoEnabled) << 16;
      pmem_write(UART_ADDR_BASE,
                 (pmem_read(UART_ADDR_BASE) & 0xff00ffffu) | iir_value);
      uart_refresh_interrupt();
      return;
    }
  }

  void tick() {
    update_timer_irq();
    update_external_irq();
  }
  bool timer_irq_level() const { return effective_timer_value() >= timer_cmp_; }


private:
  static uint64_t current_sim_time_u64() {
    return sim_time >= 0 ? static_cast<uint64_t>(sim_time) : 0ull;
  }

  uint64_t effective_timer_value() const {
    return current_sim_time_u64() + timer_offset_;
  }

  static uint8_t uart_read_reg8(uint32_t reg_off) {
    const uint32_t word_addr = UART_ADDR_BASE + (reg_off & ~0x3u);
    const uint32_t shift = (reg_off & 0x3u) * 8u;
    return static_cast<uint8_t>((pmem_read(word_addr) >> shift) & 0xffu);
  }

  static void uart_write_reg8(uint32_t reg_off, uint8_t value) {
    const uint32_t word_addr = UART_ADDR_BASE + (reg_off & ~0x3u);
    const uint32_t shift = (reg_off & 0x3u) * 8u;
    const uint32_t mask = 0xffu << shift;
    pmem_write(word_addr, (pmem_read(word_addr) & ~mask) |
                              (static_cast<uint32_t>(value) << shift));
  }

  void uart_refresh_interrupt() {
    const uint8_t ier = uart_read_reg8(kUartRegIerDlm);
    const uint8_t lsr = uart_read_reg8(kUartRegLsr);
    const bool thre_pending = ((ier & kUartIerThri) != 0) &&
                              ((lsr & kUartLsrThre) != 0);
    uart_write_reg8(kUartRegIirFcr,
                    static_cast<uint8_t>(kUartIirFifoEnabled |
                                         (thre_pending ? kUartIirThre
                                                       : kUartIirNoInt)));
    xps_intc_set_irq_level(kUartIrqId, thre_pending);
  }

  void timer_reset() {
    timer_offset_ = 0;
    timer_cmp_ = UINT64_MAX;
    update_timer_irq();
  }

  void update_timer_irq() {
    if (csr_interrupt_inject == nullptr) {
      return;
    }
    update_delayed_irq(timer_irq_delay_,
                       effective_timer_value() >= timer_cmp_);
    csr_interrupt_inject->timer_irq_pending_valid = true;
    csr_interrupt_inject->timer_irq_pending = timer_irq_delay_.delivered;
  }

  void update_external_irq() {
    if (csr_interrupt_inject == nullptr) {
      return;
    }
    const bool xps_global_enable =
        (xps_intc_mer_ & (kXpsMerMe | kXpsMerHie)) ==
        (kXpsMerMe | kXpsMerHie);
    const bool xps_pending =
        xps_global_enable && ((xps_intc_isr_ & xps_intc_ier_) != 0);
    update_delayed_irq(external_irq_delay_, xps_pending);
    csr_interrupt_inject->external_irq_pending_valid = true;
    csr_interrupt_inject->external_irq_pending = false;
    csr_interrupt_inject->supervisor_external_irq_pending_valid = true;
    csr_interrupt_inject->supervisor_external_irq_pending =
        external_irq_delay_.delivered;
  }


  static constexpr uint32_t kXpsIsr = 0x00;
  static constexpr uint32_t kXpsIpr = 0x04;
  static constexpr uint32_t kXpsIer = 0x08;
  static constexpr uint32_t kXpsIar = 0x0c;
  static constexpr uint32_t kXpsSie = 0x10;
  static constexpr uint32_t kXpsCie = 0x14;
  static constexpr uint32_t kXpsIvr = 0x18;
  static constexpr uint32_t kXpsMer = 0x1c;
  static constexpr uint32_t kXpsMerMe = 1u << 0;
  static constexpr uint32_t kXpsMerHie = 1u << 1;
  static constexpr uint32_t kUartIrqId = 2u;
  uint32_t xps_intc_isr_ = 0;
  uint32_t xps_intc_ier_ = 0;
  uint32_t xps_intc_mer_ = 0;

  struct DelayedIrq {
    bool raw = false;
    bool delivered = false;
    bool waiting = false;
    uint64_t due_cycle = 0;
  };

  DelayedIrq timer_irq_delay_;
  DelayedIrq external_irq_delay_;
  uint64_t irq_rng_state_ = 1;
  uint32_t irq_delay_min_cycles_ = 8;
  uint32_t irq_delay_max_cycles_ = 64;

  uint32_t next_irq_delay_cycles() {
    uint64_t x = irq_rng_state_;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    irq_rng_state_ = x;
    const uint64_t span =
        static_cast<uint64_t>(irq_delay_max_cycles_) -
        static_cast<uint64_t>(irq_delay_min_cycles_) + 1u;
    return irq_delay_min_cycles_ + static_cast<uint32_t>(x % span);
  }

  void update_delayed_irq(DelayedIrq &irq, bool raw) {
    const uint64_t now = current_sim_time_u64();
    if (!raw) {
      irq.raw = false;
      irq.delivered = false;
      irq.waiting = false;
      irq.due_cycle = 0;
      return;
    }

    if (!irq.raw) {
      irq.raw = true;
      irq.delivered = false;
      irq.waiting = true;
      irq.due_cycle = now + next_irq_delay_cycles();
    }
    if (irq.waiting && now >= irq.due_cycle) {
      irq.waiting = false;
      irq.delivered = true;
    }
  }

  static uint32_t mask_store_data(uint32_t data, uint8_t func3) {
    if ((func3 & 0x3u) == 0u) {
      return data & 0xffu;
    }
    if ((func3 & 0x3u) == 1u) {
      return data & 0xffffu;
    }
    return data;
  }

  static uint32_t mask_store_mask(uint8_t func3) {
    if ((func3 & 0x3u) == 0u) {
      return 0xffu;
    }
    if ((func3 & 0x3u) == 1u) {
      return 0xffffu;
    }
    return 0xffffffffu;
  }


  void xps_intc_reset() {
    xps_intc_isr_ = 0;
    xps_intc_ier_ = 0;
    xps_intc_mer_ = 0;
    update_external_irq();
  }

  uint32_t xps_intc_read_reg(uint32_t reg_off) const {
    switch (reg_off) {
    case kXpsIsr:
      return xps_intc_isr_;
    case kXpsIpr:
      return xps_intc_isr_ & xps_intc_ier_;
    case kXpsIer:
      return xps_intc_ier_;
    case kXpsIvr: {
      const uint32_t pending = xps_intc_isr_ & xps_intc_ier_;
      if (pending == 0) {
        return 0xffffffffu;
      }
      for (uint32_t i = 0; i < 32; ++i) {
        if ((pending & (1u << i)) != 0) {
          return i;
        }
      }
      return 0xffffffffu;
    }
    case kXpsMer:
      return xps_intc_mer_;
    default:
      return 0;
    }
  }

  void xps_intc_set_irq_level(uint32_t irq_id, bool asserted) {
    if (irq_id >= 32) {
      return;
    }
    const uint32_t mask = 1u << irq_id;
    if (asserted) {
      xps_intc_isr_ |= mask;
    } else {
      xps_intc_isr_ &= ~mask;
    }
    update_external_irq();
  }

  void xps_intc_write_reg(uint32_t reg_off, uint32_t value) {
    switch (reg_off) {
    case kXpsIsr:
    case kXpsIar:
      xps_intc_isr_ &= ~value;
      break;
    case kXpsIer:
      xps_intc_ier_ = value;
      break;
    case kXpsSie:
      xps_intc_ier_ |= value;
      break;
    case kXpsCie:
      xps_intc_ier_ &= ~value;
      break;
    case kXpsMer:
      xps_intc_mer_ = value;
      break;
    default:
      break;
    }
    update_external_irq();
  }







};
