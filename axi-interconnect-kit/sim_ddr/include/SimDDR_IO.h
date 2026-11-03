#pragma once
/**
 * @file SimDDR_IO.h
 * @brief AXI4 interface signal type definitions for SimDDR
 *
 * Standard AXI4 protocol with 5 channels:
 * - Write Address (AW): Master -> Slave
 * - Write Data (W): Master -> Slave
 * - Write Response (B): Slave -> Master
 * - Read Address (AR): Master -> Slave
 * - Read Data (R): Slave -> Master
 */

#include "axi_interconnect_compat.h"
#include <cstdint>

namespace sim_ddr {

// ============================================================================
// AXI4 Configuration
// ============================================================================
#ifndef AXI_KIT_AXI_ID_WIDTH
#define AXI_KIT_AXI_ID_WIDTH 6
#endif
constexpr uint8_t AXI_ID_WIDTH =
    AXI_KIT_AXI_ID_WIDTH; // configurable, 6-bit default supports 64 IDs

#ifndef AXI_KIT_SIM_DDR_BEAT_BYTES
#ifdef CONFIG_AXI_KIT_SIM_DDR_BEAT_BYTES
#define AXI_KIT_SIM_DDR_BEAT_BYTES CONFIG_AXI_KIT_SIM_DDR_BEAT_BYTES
#else
#define AXI_KIT_SIM_DDR_BEAT_BYTES 4
#endif
#endif
static_assert(AXI_KIT_SIM_DDR_BEAT_BYTES == 4 ||
                  AXI_KIT_SIM_DDR_BEAT_BYTES == 8 ||
                  AXI_KIT_SIM_DDR_BEAT_BYTES == 16 ||
                  AXI_KIT_SIM_DDR_BEAT_BYTES == 32,
              "AXI_KIT_SIM_DDR_BEAT_BYTES must be 4, 8, 16, or 32");
constexpr uint8_t SIM_DDR_BEAT_BYTES = AXI_KIT_SIM_DDR_BEAT_BYTES;
constexpr uint8_t AXI_DATA_BYTES = SIM_DDR_BEAT_BYTES; // legacy alias
constexpr uint8_t AXI_STRB_STORAGE_BYTES =
    (SIM_DDR_BEAT_BYTES + 7u) / 8u;
constexpr uint8_t AXI_DATA_WORDS =
    SIM_DDR_BEAT_BYTES / static_cast<uint8_t>(sizeof(uint32_t));
constexpr uint8_t AXI_SIZE_CODE =
    (SIM_DDR_BEAT_BYTES == 32)
        ? 5u
        : ((SIM_DDR_BEAT_BYTES == 16) ? 4u
                                      : ((SIM_DDR_BEAT_BYTES == 8) ? 3u : 2u));
#if AXI_KIT_SIM_DDR_BEAT_BYTES == 32
using axi_data_t = wire<256>;
using axi_strb_t = wire<32>;
#elif AXI_KIT_SIM_DDR_BEAT_BYTES == 16
using axi_data_t = wire<128>;
using axi_strb_t = wire<16>;
#elif AXI_KIT_SIM_DDR_BEAT_BYTES == 8
using axi_data_t = wire<64>;
using axi_strb_t = wire<8>;
#else
using axi_data_t = wire<32>;
using axi_strb_t = wire<4>;
#endif
static_assert(sizeof(axi_data_t) >= AXI_DATA_BYTES,
              "wire carrier is narrower than the configured DDR beat width");
static_assert(sizeof(axi_strb_t) >= AXI_STRB_STORAGE_BYTES,
              "wire carrier is narrower than the configured DDR strobe width");

// ============================================================================
// AXI4 Burst Types
// ============================================================================
constexpr uint8_t AXI_BURST_FIXED = 0b00;
constexpr uint8_t AXI_BURST_INCR = 0b01;
constexpr uint8_t AXI_BURST_WRAP = 0b10;

// ============================================================================
// AXI4 Response Types
// ============================================================================
constexpr uint8_t AXI_RESP_OKAY = 0b00;
constexpr uint8_t AXI_RESP_EXOKAY = 0b01;
constexpr uint8_t AXI_RESP_SLVERR = 0b10;
constexpr uint8_t AXI_RESP_DECERR = 0b11;

// ============================================================================
// AXI4 Write Address Channel (AW)
// Master -> Slave
// ============================================================================
struct AXI4_AW_t {
  // Handshake signals
  wire<1> awvalid; // Address write valid (Master output)
  wire<1> awready; // Address write ready (Slave output)

  // Transaction ID
  wire<8> awid; // Write transaction ID (Master output)

  // Address and control
  wire<32> awaddr; // Write address (byte address)
  wire<8> awlen;   // Burst length - 1 (0 = 1 beat, 255 = 256 beats)
  wire<3> awsize;  // Burst size (0=1B, 1=2B, 2=4B, 3=8B...)
  wire<2> awburst; // Burst type (0=FIXED, 1=INCR, 2=WRAP)
};

// ============================================================================
// AXI4 Write Data Channel (W)
// Master -> Slave
// ============================================================================
struct AXI4_W_t {
  // Handshake signals
  wire<1> wvalid; // Write data valid (Master output)
  wire<1> wready; // Write data ready (Slave output)

  // Data
  axi_data_t wdata; // Write data
  axi_strb_t wstrb; // Write strobes (byte enables)
  wire<1> wlast;    // Last beat of burst (Master output)
};

// ============================================================================
// AXI4 Write Response Channel (B)
// Slave -> Master
// ============================================================================
struct AXI4_B_t {
  // Handshake signals
  wire<1> bvalid; // Write response valid (Slave output)
  wire<1> bready; // Write response ready (Master output)

  // Transaction ID
  wire<8> bid; // Write response ID (Slave output, matches awid)

  // Response
  wire<2> bresp; // Write response (OKAY/EXOKAY/SLVERR/DECERR)
};

// ============================================================================
// AXI4 Read Address Channel (AR)
// Master -> Slave
// ============================================================================
struct AXI4_AR_t {
  // Handshake signals
  wire<1> arvalid; // Address read valid (Master output)
  wire<1> arready; // Address read ready (Slave output)

  // Transaction ID
  wire<8> arid; // Read transaction ID (Master output)

  // Address and control
  wire<32> araddr; // Read address (byte address)
  wire<8> arlen;   // Burst length - 1 (0 = 1 beat, 255 = 256 beats)
  wire<3> arsize;  // Burst size (0=1B, 1=2B, 2=4B, 3=8B...)
  wire<2> arburst; // Burst type (0=FIXED, 1=INCR, 2=WRAP)
};

// ============================================================================
// AXI4 Read Data Channel (R)
// Slave -> Master
// ============================================================================
struct AXI4_R_t {
  // Handshake signals
  wire<1> rvalid; // Read data valid (Slave output)
  wire<1> rready; // Read data ready (Master output)

  // Transaction ID
  wire<8> rid; // Read data ID (Slave output, matches arid)

  // Data and response
  axi_data_t rdata; // Read data
  wire<2> rresp;    // Read response (OKAY/EXOKAY/SLVERR/DECERR)
  wire<1> rlast;    // Last beat of burst (Slave output)
};

// ============================================================================
// Combined SimDDR IO Interface
// ============================================================================
struct SimDDR_IO_t {
  // Write channels (Master signals as input, Slave signals as output)
  AXI4_AW_t aw;
  AXI4_W_t w;
  AXI4_B_t b;

  // Read channels
  AXI4_AR_t ar;
  AXI4_R_t r;
};

} // namespace sim_ddr
