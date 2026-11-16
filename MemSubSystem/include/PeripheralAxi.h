#pragma once
#include "AxiMasterPort.h"

#include "DcacheConfig.h"
#include "IO.h"
#include <cstdint>
#include <cstring>

class PeripheralModel;

struct PeripheralAxiIn {
  const PeripheralReqIO *req = nullptr;                          // the LSU's MMIO request
  const axi_interconnect::ReadMasterPort_t *read = nullptr;    // the EXTRA_R / EXTRA_W ports, interconnect side
  const axi_interconnect::WriteMasterPort_t *write = nullptr;
};

struct PeripheralAxiOut {
  PeripheralRespIO *resp = nullptr;                              // the MMIO response to the LSU (SimCpu's register d)
  axi_interconnect::ReadMasterPort_t *read = nullptr;          // the same ports, the request side (bypass: MMIO)
  axi_interconnect::WriteMasterPort_t *write = nullptr;
};

struct PeripheralAxiState {
  bool busy = false;
  bool write = false;
  bool req_accepted = false;
  bool resp_valid = false;
  uint32_t addr = 0;
  uint32_t wdata = 0;
  uint8_t func3 = 0;
  uint64_t rdata = 0;
  uint8_t req_id = 0;
};

class PeripheralAxi {
public:
  PeripheralModel *peripheral_model = nullptr;

  void init();
  void comb();   // request / response of the MMIO path on the EXTRA ports, then the next state
  void seq();

  PeripheralAxiIn in;
  PeripheralAxiOut out;
  PeripheralAxiState cur;
  PeripheralAxiState nxt;

private:
  void comb_outputs();
  void comb_inputs();
  static uint8_t access_size_bytes(uint8_t func3);
  static uint32_t pack_write_data(uint32_t data, uint8_t func3);
  static uint64_t pack_write_strb(uint8_t func3);
  static uint32_t extract_read_data(uint32_t raw, uint8_t func3);
};
