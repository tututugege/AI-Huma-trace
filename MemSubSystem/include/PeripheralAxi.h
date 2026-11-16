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
  const CommitStoreIO *commit_store = nullptr;                   // the back end's device stores of this cycle's commit
};

struct PeripheralAxiOut {
  PeripheralRespIO *resp = nullptr;                              // the MMIO response to the LSU (SimCpu's register d)
  axi_interconnect::ReadMasterPort_t *read = nullptr;          // the same ports, the request side (bypass: MMIO)
  axi_interconnect::WriteMasterPort_t *write = nullptr;
  CsrInterruptInjectIO *csr_interrupt_inject = nullptr;         // the interrupt lines into the CSR (the devices drive them)
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

// Comb phases in this block's own ports (MemSubsystem::init binds them; scripts/comb/modules.py checks them):
//   CONTRACT comb_commit_store  in: in.commit_store  out: out.csr_interrupt_inject
//   CONTRACT comb  in: in.read.req.accepted in.read.resp.data in.read.resp.valid in.req in.write.req.accepted in.write.resp.valid out.resp  out: out.read.req.addr out.read.req.bypass out.read.req.id out.read.req.total_size out.read.req.valid out.read.resp.ready out.resp out.write.req.addr out.write.req.bypass out.write.req.id out.write.req.total_size out.write.req.valid out.write.req.wdata out.write.req.wstrb out.write.resp.ready
class PeripheralAxi {
public:
  PeripheralModel *peripheral_model = nullptr;   // the devices (a submodule: its interrupt lines are out.csr_interrupt_inject)

  void init();
  void comb_commit_store();   // the device stores of this cycle's commit (each valid slot, in slot order)
  void comb();   // request / response of the MMIO path on the EXTRA ports, then the next state
  void seq();

  PeripheralAxiIn in;
  PeripheralAxiOut out;
  using Regs = PeripheralAxiState;   // registers: q (read in comb through a const reference, written only by seq / init) and d (comb writes it; d == q at the start of every cycle)
  Regs d;
  const Regs &q() const { return q_; }

private:
  Regs q_;
  void comb_outputs();
  void comb_inputs();
  static uint8_t access_size_bytes(uint8_t func3);
  static uint32_t pack_write_data(uint32_t data, uint8_t func3);
  static uint64_t pack_write_strb(uint8_t func3);
  static uint32_t extract_read_data(uint32_t raw, uint8_t func3);
};
