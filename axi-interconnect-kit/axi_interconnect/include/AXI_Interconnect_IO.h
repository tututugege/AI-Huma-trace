#pragma once
/**
 * @file AXI_Interconnect_IO.h
 * @brief AXI-Interconnect Upstream Interface Definitions
 *
 * Simplified master interfaces for icache/dcache/uncore-lsu/extra:
 * - Read response can return one full upstream transaction (up to 256B)
 * - Write request payload can carry one full cache-line transaction
 * - total_size specifies transfer width in bytes minus 1
 * - ID for out-of-order response routing
 */

#include "axi_interconnect_compat.h"
#include <cstdint>

namespace axi_interconnect {

// ============================================================================
// Configuration
// ============================================================================
constexpr uint8_t NUM_READ_MASTERS = 4;  // icache, dcache, uncore-lsu, extra
constexpr uint8_t NUM_WRITE_MASTERS = 2; // dcache + uncore-lsu
#ifndef AXI_KIT_MAX_OUTSTANDING
#ifdef CONFIG_AXI_KIT_MAX_OUTSTANDING
#define AXI_KIT_MAX_OUTSTANDING CONFIG_AXI_KIT_MAX_OUTSTANDING
#else
#define AXI_KIT_MAX_OUTSTANDING 8
#endif
#endif
constexpr uint8_t MAX_OUTSTANDING = AXI_KIT_MAX_OUTSTANDING;

#ifndef AXI_KIT_MAX_WRITE_TRANSACTION_BYTES
#define AXI_KIT_MAX_WRITE_TRANSACTION_BYTES 64
#endif

static_assert((AXI_KIT_MAX_WRITE_TRANSACTION_BYTES % sizeof(uint32_t)) == 0,
              "AXI_KIT_MAX_WRITE_TRANSACTION_BYTES must be word-aligned");
static_assert(AXI_KIT_MAX_WRITE_TRANSACTION_BYTES > 0,
              "AXI_KIT_MAX_WRITE_TRANSACTION_BYTES must be non-zero");
static_assert(AXI_KIT_MAX_WRITE_TRANSACTION_BYTES <= 64,
              "AXI_KIT_MAX_WRITE_TRANSACTION_BYTES exceeds 64B strobe width");

constexpr uint16_t MAX_WRITE_TRANSACTION_BYTES =
    AXI_KIT_MAX_WRITE_TRANSACTION_BYTES;
constexpr uint16_t MAX_WRITE_TRANSACTION_WORDS =
    MAX_WRITE_TRANSACTION_BYTES / sizeof(uint32_t);
#ifndef AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER
#ifdef CONFIG_AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER
#define AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER \
  CONFIG_AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER
#else
#define AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER 4
#endif
#endif
constexpr uint8_t MAX_READ_OUTSTANDING_PER_MASTER =
    AXI_KIT_MAX_READ_OUTSTANDING_PER_MASTER;
#ifndef AXI_KIT_MAX_WRITE_OUTSTANDING
#ifdef CONFIG_AXI_KIT_MAX_WRITE_OUTSTANDING
#define AXI_KIT_MAX_WRITE_OUTSTANDING CONFIG_AXI_KIT_MAX_WRITE_OUTSTANDING
#else
#define AXI_KIT_MAX_WRITE_OUTSTANDING 8
#endif
#endif
constexpr uint8_t MAX_WRITE_OUTSTANDING = AXI_KIT_MAX_WRITE_OUTSTANDING;
constexpr uint8_t AXI_UPSTREAM_PAYLOAD_WORDS = MAX_WRITE_TRANSACTION_WORDS;
constexpr uint8_t AXI_UPSTREAM_PAYLOAD_BYTES =
    AXI_UPSTREAM_PAYLOAD_WORDS * sizeof(uint32_t);
constexpr uint8_t AXI_BEAT_WORDS =
    AXI_UPSTREAM_PAYLOAD_WORDS; // legacy alias
constexpr uint8_t AXI_BEAT_BYTES =
    AXI_UPSTREAM_PAYLOAD_BYTES; // legacy alias
constexpr uint8_t CACHELINE_WORDS =
    AXI_UPSTREAM_PAYLOAD_WORDS; // legacy beat-width alias
constexpr uint16_t MAX_READ_TRANSACTION_BYTES = 256;
constexpr uint16_t MAX_READ_TRANSACTION_WORDS =
    MAX_READ_TRANSACTION_BYTES / sizeof(uint32_t);

// Master IDs
constexpr uint8_t MASTER_ICACHE = 0;
constexpr uint8_t MASTER_DCACHE_R = 1;
constexpr uint8_t MASTER_UNCORE_LSU_R = 2;
constexpr uint8_t MASTER_EXTRA_R = 3;

constexpr uint8_t MASTER_DCACHE_W = 0;
constexpr uint8_t MASTER_UNCORE_LSU_W = 1;

// Legacy aliases kept for source compatibility while simulator wiring migrates.
constexpr uint8_t MASTER_MMU = MASTER_UNCORE_LSU_R;
constexpr uint8_t MASTER_EXTRA_W = MASTER_UNCORE_LSU_W;

// ============================================================================
// Read Response Data Type (up to 256B = 64 x 32-bit words)
// ============================================================================
// Trace-driven build: cacheable line traffic carries no values and instruction words / page-table
// entries come from the core's memory, so only the MMIO / peripheral read value (one word) travels
// here. Indices past the first word alias it (junk, never read).
constexpr int kWideReadKeptWords = 1;
struct WideReadData_t {
  uint32_t w_[kWideReadKeptWords] = {};

  void clear() {
    for (int i = 0; i < kWideReadKeptWords; i++)
      w_[i] = 0;
  }

  uint32_t &operator[](int idx) { return w_[idx < kWideReadKeptWords ? idx : kWideReadKeptWords - 1]; }
  const uint32_t &operator[](int idx) const { return w_[idx < kWideReadKeptWords ? idx : kWideReadKeptWords - 1]; }
};

// ============================================================================
// Write Payload Data Type (up to 64B by default)
// ============================================================================
struct WideData256_t;

struct WideWriteData_t {
  uint32_t w0_ = 0;  // MMIO/peripheral write value; other indices alias it (see WideReadData_t)

  void clear() { w0_ = 0; }

  uint32_t &operator[](int) { return w0_; }
  const uint32_t &operator[](int) const { return w0_; }

  WideWriteData_t() = default;
  WideWriteData_t(const WideData256_t &other);
  WideWriteData_t &operator=(const WideData256_t &other);
};

struct WideWriteStrb_t {
  uint8_t bytes[MAX_WRITE_TRANSACTION_BYTES];

  void clear() {
    for (int i = 0; i < MAX_WRITE_TRANSACTION_BYTES; i++) {
      bytes[i] = 0;
    }
  }

  bool test(uint32_t idx) const {
    return idx < MAX_WRITE_TRANSACTION_BYTES && bytes[idx] != 0;
  }

  void set(uint32_t idx, bool enable) {
    if (idx < MAX_WRITE_TRANSACTION_BYTES) {
      bytes[idx] = enable ? 1u : 0u;
    }
  }

  uint32_t slice_u32(uint32_t first_byte) const {
    uint32_t out = 0;
    for (uint32_t i = 0; i < 32; ++i) {
      if (test(first_byte + i)) {
        out |= (1u << i);
      }
    }
    return out;
  }

  template <typename T,
            typename = std::enable_if_t<std::is_integral_v<T>>>
  WideWriteStrb_t &operator=(T mask) {
    clear();
    const uint64_t wide_mask = static_cast<uint64_t>(mask);
    for (uint32_t i = 0; i < 64 && i < MAX_WRITE_TRANSACTION_BYTES; ++i) {
      bytes[i] = static_cast<uint8_t>((wide_mask >> i) & 0x1u);
    }
    return *this;
  }

  explicit operator uint64_t() const {
    uint64_t mask = 0;
    for (uint32_t i = 0; i < 64 && i < MAX_WRITE_TRANSACTION_BYTES; ++i) {
      mask |= static_cast<uint64_t>(bytes[i] != 0) << i;
    }
    return mask;
  }
};

struct WideData256_t {
  // Historical name retained for source compatibility. The upstream payload
  // remains bounded by AXI_KIT_MAX_WRITE_TRANSACTION_BYTES, while downstream
  // SimDDR beats may now use a true wire<256> carrier in standalone mode.
  uint32_t words[AXI_UPSTREAM_PAYLOAD_WORDS];

  void clear() {
    for (int i = 0; i < AXI_UPSTREAM_PAYLOAD_WORDS; i++)
      words[i] = 0;
  }

  uint32_t &operator[](int idx) { return words[idx]; }
  const uint32_t &operator[](int idx) const { return words[idx]; }
};

inline WideWriteData_t::WideWriteData_t(const WideData256_t &other) { w0_ = other.words[0]; }

inline WideWriteData_t &WideWriteData_t::operator=(const WideData256_t &other) {
  w0_ = other.words[0];
  return *this;
}

// ============================================================================
// Read Master Interface (for icache/dcache/uncore-lsu/extra)
// ============================================================================

// Read Request: Master → Interleaver
struct ReadMasterReq_t {
  wire<1> valid;
  wire<1> ready;       // ← Output from interleaver
  wire<1> accepted;    // ← One-cycle pulse when request is truly accepted
  wire<4> accepted_id; // ← ID of the request accepted in this pulse
  wire<32> addr;       // Byte address
  wire<8> total_size;  // 0=1B ... 255=256B
  wire<4> id;          // Transaction ID (for out-of-order)
  wire<1> bypass;      // Skip LLC / cacheable path and force memory/MMIO routing
};

// Read Response: Interleaver → Master
struct ReadMasterResp_t {
  wire<1> valid; // ← Output from interleaver
  wire<1> ready;
  WideReadData_t data; // Wide data (up to one 256B upstream transaction)
  wire<4> id;          // Matching transaction ID
};

// Combined Read Master Port
struct ReadMasterPort_t {
  ReadMasterReq_t req;
  ReadMasterResp_t resp;
};

// ============================================================================
// Write Master Interface
// ============================================================================

// Write Request: Master → Interleaver (AW+W combined)
struct WriteMasterReq_t {
  wire<1> valid;
  wire<1> ready;         // ← Output from interleaver
  wire<1> accepted;      // ← One-cycle pulse when request is truly accepted
  wire<32> addr;         // Byte address
  WideWriteData_t wdata; // Wide write data
  WideWriteStrb_t wstrb; // Byte strobe (1 bit per byte, up to 64B)
  wire<8> total_size;    // 0=1B ... 63=64B (default cap)
  wire<4> id;            // Transaction ID
  wire<1> bypass;        // Skip LLC / cacheable path and force memory/MMIO routing
};

// Write Response: Interleaver → Master
struct WriteMasterResp_t {
  wire<1> valid; // ← Output from interleaver
  wire<1> ready;
  wire<4> id;   // Matching transaction ID
  wire<2> resp; // AXI response (OKAY, SLVERR, etc.)
};

// Combined Write Master Port
struct WriteMasterPort_t {
  WriteMasterReq_t req;
  WriteMasterResp_t resp;
};

// ============================================================================
// AXI-Interconnect Combined IO
// ============================================================================

struct AXI_Interconnect_IO_t {
  // Upstream: Read Masters (3 ports)
  ReadMasterPort_t read_masters[NUM_READ_MASTERS];

  // Upstream: Write Masters (2 ports)
  WriteMasterPort_t write_masters[NUM_WRITE_MASTERS];

  // Downstream: AXI4 to SimDDR
  // (Use SimDDR_IO_t from sim_ddr module)
};

} // namespace axi_interconnect
