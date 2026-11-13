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
  static constexpr uint32_t kUartRegLcr = 0x3u;
  static constexpr uint32_t kUartRegLsr = 0x5u;
  static constexpr uint8_t kUartLcrDlab = 0x80u;
  static constexpr uint8_t kUartLsrThre = 0x20u;
  static constexpr uint8_t kUartLsrTemt = 0x40u;
  static constexpr uint8_t kUartIerThri = 0x02u;
  static constexpr uint8_t kUartIirNoInt = 0x01u;
  static constexpr uint8_t kUartIirThre = 0x02u;
  static constexpr uint8_t kUartIirFifoEnabled = 0xc0u;

  CsrInterruptInjectIO *csr_interrupt_inject = nullptr;
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

  void bind(CsrInterruptInjectIO *csr_interrupt_inject_ptr) {
    csr_interrupt_inject = csr_interrupt_inject_ptr;
    timer_reset();
    xps_intc_reset();
  }

  void load_sdcard_image(const std::string &path) {
    std::ifstream image(path, std::ios::binary);
    if (!image.is_open()) {
      throw std::runtime_error("failed to open sdcard image: " + path);
    }
    image.seekg(0, std::ios::end);
    const std::streamsize size = image.tellg();
    image.seekg(0, std::ios::beg);
    if (size < 0) {
      throw std::runtime_error("invalid sdcard image size");
    }
    sdcard_image_.resize(static_cast<size_t>(size));
    if (size > 0 &&
        !image.read(reinterpret_cast<char *>(sdcard_image_.data()), size)) {
      throw std::runtime_error("failed to read sdcard image");
    }
    const uint64_t rounded_capacity =
        ((static_cast<uint64_t>(sdcard_image_.size()) + 524287u) / 524288u) *
        524288u;
    if (rounded_capacity > sdcard_image_.size()) {
      sdcard_image_.resize(static_cast<size_t>(rounded_capacity), 0);
    }
    xps_intc_reset();
    ocsdc_reset();
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

    ocsdc_regs_.clear();
    for (uint32_t off = 0; off + 4u <= OCSDC_MMIO_SIZE; off += 4u) {
      ocsdc_regs_[off] = pmem_read(OCSDC_ADDR_BASE + off);
    }
    // OCSDC DMA completes atomically in this model, so no transaction may be
    // carried across the checkpoint boundary.
    dma_write_pending_ = false;
    dma_write_paddr_ = 0;
    dma_write_size_ = 0;
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
           in_range(paddr, OPENSBI_TIMER_BASE, OPENSBI_TIMER_MMIO_SIZE) ||
           in_range(paddr, OCSDC_ADDR_BASE, OCSDC_MMIO_SIZE);
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
      { static const bool dbg = std::getenv("TB_TIMER_DBG") != nullptr; static unsigned long long n = 0;
        if (dbg && (++n <= 20 || n % 20000 == 0)) std::fprintf(stderr, "[TIMER_DBG] read #%llu mtime=%llu sim_time=%llu offset=%llu cmp=%llx\n", n, (unsigned long long)effective_timer_value(), (unsigned long long)current_sim_time_u64(), (unsigned long long)timer_offset_, (unsigned long long)timer_cmp_); }
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
    } else if (paddr >= OCSDC_ADDR_BASE &&
               paddr < (OCSDC_ADDR_BASE + OCSDC_MMIO_SIZE)) {
      data = ocsdc_read_reg(paddr - OCSDC_ADDR_BASE);
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

    if (paddr >= OCSDC_ADDR_BASE && paddr < (OCSDC_ADDR_BASE + OCSDC_MMIO_SIZE)) {
      const uint32_t reg_off = paddr - OCSDC_ADDR_BASE;
      const uint32_t shift = (paddr & 0x3u) * 8u;
      const uint32_t store_data = mask_store_data(data, func3);
      const uint32_t mask = mask_store_mask(func3) << shift;
      const uint32_t wdata = store_data << shift;
      const uint32_t old_data = ocsdc_read_reg(reg_off & ~0x3u);
      const uint32_t new_data = (old_data & ~mask) | (wdata & mask);
      ocsdc_regs_[reg_off & ~0x3u] = new_data;
      ocsdc_write_reg(reg_off & ~0x3u, new_data);
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

  bool consume_dma_write(uint32_t &paddr, size_t &size) {
    if (!dma_write_pending_) {
      return false;
    }
    paddr = dma_write_paddr_;
    size = dma_write_size_;
    dma_write_pending_ = false;
    return true;
  }

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

  static constexpr uint32_t kOcsdcArgument = 0x00;
  static constexpr uint32_t kOcsdcCommand = 0x04;
  static constexpr uint32_t kOcsdcResponse1 = 0x08;
  static constexpr uint32_t kOcsdcResponse2 = 0x0c;
  static constexpr uint32_t kOcsdcResponse3 = 0x10;
  static constexpr uint32_t kOcsdcResponse4 = 0x14;
  static constexpr uint32_t kOcsdcControl = 0x1c;
  static constexpr uint32_t kOcsdcTimeout = 0x20;
  static constexpr uint32_t kOcsdcClockDivider = 0x24;
  static constexpr uint32_t kOcsdcSoftwareReset = 0x28;
  static constexpr uint32_t kOcsdcPowerControl = 0x2c;
  static constexpr uint32_t kOcsdcCapability = 0x30;
  static constexpr uint32_t kOcsdcCmdIntStatus = 0x34;
  static constexpr uint32_t kOcsdcCmdIntEnable = 0x38;
  static constexpr uint32_t kOcsdcDatIntStatus = 0x3c;
  static constexpr uint32_t kOcsdcDatIntEnable = 0x40;
  static constexpr uint32_t kOcsdcBlockSize = 0x44;
  static constexpr uint32_t kOcsdcBlockCount = 0x48;
  static constexpr uint32_t kOcsdcDstSrcAddr = 0x60;

  static constexpr uint32_t kCmdIntCc = 0x0001;
  static constexpr uint32_t kCmdIntEi = 0x0002;
  static constexpr uint32_t kCmdIntCte = 0x0004;
  static constexpr uint32_t kDatIntCc = 0x0001;
  static constexpr uint32_t kDatIntTrs = 0x0001;
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
  static constexpr uint32_t kOcsdcCmdIrqId = 3u;
  static constexpr uint32_t kOcsdcDatIrqId = 4u;
  std::vector<uint8_t> sdcard_image_;
  std::unordered_map<uint32_t, uint32_t> ocsdc_regs_;
  uint32_t xps_intc_isr_ = 0;
  uint32_t xps_intc_ier_ = 0;
  uint32_t xps_intc_mer_ = 0;
  bool dma_write_pending_ = false;
  uint32_t dma_write_paddr_ = 0;
  size_t dma_write_size_ = 0;

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

  struct SdCardId {
    uint32_t ocr;
    uint32_t cid[4];
    uint32_t csd[4];
  };

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

  void ocsdc_reset() {
    dma_write_pending_ = false;
    dma_write_paddr_ = 0;
    dma_write_size_ = 0;
    ocsdc_regs_.clear();
    ocsdc_regs_[kOcsdcArgument] = 0;
    ocsdc_regs_[kOcsdcCommand] = 0;
    ocsdc_regs_[kOcsdcPowerControl] = 3300u;
    ocsdc_regs_[kOcsdcCapability] = 0;
    ocsdc_regs_[kOcsdcCmdIntStatus] = 0;
    ocsdc_regs_[kOcsdcCmdIntEnable] = 0;
    ocsdc_regs_[kOcsdcDatIntStatus] = 0;
    ocsdc_regs_[kOcsdcDatIntEnable] = 0;
    xps_intc_set_irq_level(kOcsdcCmdIrqId, false);
    xps_intc_set_irq_level(kOcsdcDatIrqId, false);
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

  uint32_t ocsdc_read_reg(uint32_t reg_off) const {
    auto it = ocsdc_regs_.find(reg_off & ~0x3u);
    const uint32_t value = (it == ocsdc_regs_.end()) ? 0u : it->second;
    if (it == ocsdc_regs_.end()) {
      return 0;
    }
    return value;
  }

  static SdCardId make_sdcard_id(size_t image_size) {
    SdCardId id = {};
    constexpr uint32_t kSdCmdClass =
        (1u << 2) |  // CCC_BLOCK_READ
        (1u << 4) |  // CCC_BLOCK_WRITE
        (1u << 5) |  // CCC_ERASE
        (1u << 6) |  // CCC_WRITE_PROT
        (1u << 8) |  // CCC_APP_SPEC
        (1u << 10);  // CCC_SWITCH

    id.ocr = 0xc0ff8000u;
    id.cid[0] = 0x03534453u;
    id.cid[1] = 0x30303030u;
    id.cid[2] = 0x12345678u;
    id.cid[3] = 0x01020304u;

    const uint64_t units =
        (static_cast<uint64_t>(image_size) + 524287u) / 524288u;
    const uint32_t csize =
        units == 0 ? 0u : static_cast<uint32_t>(units - 1u);

    id.csd[0] = 0x400e0032u;
    id.csd[1] = kSdCmdClass << 20;
    id.csd[1] |= 9u << 16;
    id.csd[1] |= (csize >> 16) & 0x3fu;
    id.csd[2] = (csize & 0xffffu) << 16;
    id.csd[3] = 0u;
    return id;
  }

  void write_payload(uint32_t dst_addr, const uint8_t *src, size_t len) {
    if (len == 0 || src == nullptr) {
      return;
    }
    if (!pmem_is_ram_addr(dst_addr, static_cast<uint32_t>(len))) {
      throw std::runtime_error("OCSDC DMA destination is not RAM");
    }
    pmem_memcpy_to_ram(dst_addr, src, len);
    dma_write_pending_ = true;
    dma_write_paddr_ = dst_addr;
    dma_write_size_ = len;
  }

  void write_data_buffer(uint32_t dst_addr, size_t sd_offset_bytes,
                         uint32_t block_size, uint32_t block_count) {
    if (block_size == 0 || block_count == 0) {
      return;
    }
    const size_t total_bytes =
        static_cast<size_t>(block_size) * static_cast<size_t>(block_count);
    if (sdcard_image_.empty()) {
      throw std::runtime_error(
          "OCSDC read requested but no sdcard image loaded");
    }
    if (sd_offset_bytes > sdcard_image_.size() ||
        total_bytes > (sdcard_image_.size() - sd_offset_bytes)) {
      throw std::runtime_error("OCSDC read beyond sdcard image");
    }
    write_payload(dst_addr, sdcard_image_.data() + sd_offset_bytes,
                  total_bytes);
  }

  void set_r1(uint32_t value) {
    ocsdc_regs_[kOcsdcResponse1] = value;
    ocsdc_regs_[kOcsdcResponse2] = 0;
    ocsdc_regs_[kOcsdcResponse3] = 0;
    ocsdc_regs_[kOcsdcResponse4] = 0;
  }

  void ocsdc_execute_command() {
    constexpr uint32_t kCmdResp48 = 0x1;
    constexpr uint32_t kCmdResp136 = 0x2;
    constexpr uint32_t kCmdDataRead = 0x20;
    constexpr uint32_t kCmdDataWrite = 0x40;
    constexpr uint32_t kCmdIdxShift = 8;

    const uint32_t cmd_reg = ocsdc_read_reg(kOcsdcCommand);
    const uint32_t arg = ocsdc_read_reg(kOcsdcArgument);
    const uint32_t opcode = cmd_reg >> kCmdIdxShift;
    const bool wants_resp = (cmd_reg & (kCmdResp48 | kCmdResp136)) != 0;
    const bool data_read = (cmd_reg & kCmdDataRead) != 0;
    const bool data_write = (cmd_reg & kCmdDataWrite) != 0;
    const uint32_t block_size = (ocsdc_read_reg(kOcsdcBlockSize) & 0xffffu) + 1u;
    const uint32_t block_count = (ocsdc_read_reg(kOcsdcBlockCount) & 0xffffu) + 1u;
    const uint32_t dst_addr = ocsdc_read_reg(kOcsdcDstSrcAddr);
    const SdCardId card_id = make_sdcard_id(sdcard_image_.size());

    ocsdc_regs_[kOcsdcCmdIntStatus] = 0;
    ocsdc_regs_[kOcsdcDatIntStatus] = 0;
    set_r1(0);

    switch (opcode) {
    case 0:
      set_r1(0);
      break;
    case 8:
      set_r1(arg);
      break;
    case 55:
      set_r1(0x20u);
      break;
    case 41:
      set_r1(card_id.ocr);
      break;
    case 2:
      ocsdc_regs_[kOcsdcResponse1] = card_id.cid[0];
      ocsdc_regs_[kOcsdcResponse2] = card_id.cid[1];
      ocsdc_regs_[kOcsdcResponse3] = card_id.cid[2];
      ocsdc_regs_[kOcsdcResponse4] = card_id.cid[3];
      break;
    case 3:
      set_r1(1u << 16);
      break;
    case 7:
      set_r1(0x00000900u);
      break;
    case 9:
      ocsdc_regs_[kOcsdcResponse1] = card_id.csd[0];
      ocsdc_regs_[kOcsdcResponse2] = card_id.csd[1];
      ocsdc_regs_[kOcsdcResponse3] = card_id.csd[2];
      ocsdc_regs_[kOcsdcResponse4] = card_id.csd[3];
      break;
    case 13:
      set_r1(0x00000900u | (4u << 9) | (1u << 8));
      break;
    case 16:
      set_r1(0);
      break;
    case 17:
      set_r1(0);
      break;
    case 18:
      set_r1(0x00000900u);
      break;
    case 6:
      set_r1(0);
      break;
    default:
      set_r1(0);
      break;
    }

    if (data_read || data_write) {
      if (data_read && opcode == 51 && block_size == 8 &&
          block_count == 1) {
        static constexpr uint8_t kScrPayload[8] = {0x02, 0x05, 0x00, 0x00,
                                                   0x00, 0x00, 0x00, 0x00};
        write_payload(dst_addr, kScrPayload, sizeof(kScrPayload));
      } else if (data_read && opcode == 6 && block_size == 64 &&
                 block_count == 1) {
        uint32_t switch_status[16] = {};
        const bool do_switch = ((arg >> 31) & 1u) != 0;
        switch_status[3] = 0x00020000u;
        switch_status[4] = do_switch ? 0x01000000u : 0x00000000u;
        switch_status[7] = 0x00000000u;
        write_payload(dst_addr, reinterpret_cast<const uint8_t *>(switch_status),
                      sizeof(switch_status));
      } else if (data_read) {
        const size_t sd_offset = static_cast<size_t>(arg) * 512ull;
        write_data_buffer(dst_addr, sd_offset, block_size, block_count);
      }
      ocsdc_regs_[kOcsdcDatIntStatus] = kDatIntCc | kDatIntTrs;
      if (ocsdc_read_reg(kOcsdcDatIntEnable) != 0) {
        xps_intc_set_irq_level(kOcsdcDatIrqId, true);
      }
    }

    if (wants_resp || opcode == 0) {
      ocsdc_regs_[kOcsdcCmdIntStatus] = kCmdIntCc;
      if (ocsdc_read_reg(kOcsdcCmdIntEnable) != 0) {
        xps_intc_set_irq_level(kOcsdcCmdIrqId, true);
      }
    }

  }

  void ocsdc_write_reg(uint32_t reg_off, uint32_t value) {
    switch (reg_off) {
    case kOcsdcCmdIntStatus:
      ocsdc_regs_[reg_off] = 0;
      xps_intc_set_irq_level(kOcsdcCmdIrqId, false);
      break;
    case kOcsdcDatIntStatus:
      ocsdc_regs_[reg_off] = 0;
      xps_intc_set_irq_level(kOcsdcDatIrqId, false);
      break;
    case kOcsdcSoftwareReset:
      ocsdc_regs_[reg_off] = value;
      if ((value & 1u) != 0) {
        ocsdc_reset();
      }
      break;
    case kOcsdcCmdIntEnable:
      ocsdc_regs_[reg_off] = value;
      if (value != 0 && ocsdc_read_reg(kOcsdcCmdIntStatus) != 0) {
        xps_intc_set_irq_level(kOcsdcCmdIrqId, true);
      } else if (value == 0) {
        xps_intc_set_irq_level(kOcsdcCmdIrqId, false);
      }
      break;
    case kOcsdcDatIntEnable:
      ocsdc_regs_[reg_off] = value;
      if (value != 0 && ocsdc_read_reg(kOcsdcDatIntStatus) != 0) {
        xps_intc_set_irq_level(kOcsdcDatIrqId, true);
      } else if (value == 0) {
        xps_intc_set_irq_level(kOcsdcDatIrqId, false);
      }
      break;
    case kOcsdcArgument:
      ocsdc_regs_[reg_off] = value;
      ocsdc_execute_command();
      break;
    default:
      ocsdc_regs_[reg_off] = value;
      break;
    }
  }
};
