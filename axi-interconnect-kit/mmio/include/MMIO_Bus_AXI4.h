#pragma once
/**
 * @file MMIO_Bus_AXI4.h
 * @brief AXI4 MMIO bus (single outstanding read/write stream, 32-bit data).
 */

#include "MMIO_Device.h"
#include "SimDDR_IO.h"
#include <cstdint>
#include <vector>

namespace mmio {

class MMIO_Bus_AXI4 {
public:
  void init();
  void add_device(uint32_t base, uint32_t size, MMIO_Device *dev);

  void comb_outputs();
  void comb_inputs();
  void seq();

  // AXI4 IO (router-facing)
  sim_ddr::SimDDR_IO_t io;

private:
  struct Region {
    uint32_t base;
    uint32_t size;
    MMIO_Device *dev;
  };

  struct PendingRead {
    bool active;
    uint8_t id;
    uint32_t addr;
    uint8_t len;
    uint8_t size;
    uint8_t burst;
    uint8_t beat_idx;
    uint32_t latency_cnt;
    bool beat_valid;
    sim_ddr::axi_data_t beat_data;
    uint8_t beat_resp;
  };

  struct PendingWrite {
    bool active;
    uint8_t id;
    uint32_t addr;
    uint8_t len;
    uint8_t size;
    uint8_t burst;
    uint8_t beat_idx;
    uint8_t resp;
  };

  struct PendingWriteResp {
    bool active;
    uint8_t id;
    uint8_t resp;
    uint32_t latency_cnt;
  };

  std::vector<Region> regions;
  PendingRead r_pending;
  PendingWrite w_pending;
  PendingWriteResp w_resp;

  static constexpr uint32_t MMIO_LATENCY = 1;

  MMIO_Device *find_device(uint32_t addr, uint32_t &base) const;
  static uint8_t beat_bytes(uint8_t size);
  static uint32_t beat_addr(uint32_t base_addr, uint8_t burst, uint8_t size,
                            uint8_t beat_idx);
  static sim_ddr::axi_data_t load_beat(const uint8_t *p, uint8_t nbytes);
  static void store_beat(uint8_t *p, sim_ddr::axi_data_t v, uint8_t nbytes);
  void build_read_beat();
};

} // namespace mmio
