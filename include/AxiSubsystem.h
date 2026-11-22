#pragma once
#include "AXI_Interconnect.h"
#include "MMIO_Bus_AXI4.h"
#include "SimDDR.h"
#include "UART16550_Device.h"
#include "axi_mmio_map.h"

class MemSubsystem;

// The AXI side of the SoC: the interconnect (with the LLC data path), the DDR and MMIO slaves, the UART on the MMIO
// bus. The LLC's lookup table lives in the memory subsystem (llc, bound by SimCpu::init): comb_outputs runs its
// lookup first, seq its table update. Its ports are the masters' ports of the interconnect (the interconnect's code,
// AXI_Interconnect, is scanned as its submodule): comb_begin clears the requests' valid before any master drives one;
// comb_outputs reads the I-cache's request of this cycle and the other masters' before they drive them (cleared).
//   CONTRACT comb_begin  default: out.read_ports.req.valid out.write_ports.req.valid
//   CONTRACT comb_outputs  in: out.read_ports[MASTER_ICACHE].req out.read_ports.resp.valid  pre: out.read_ports.req out.write_ports.req  out: out.read_ports.req.accepted out.read_ports.req.accepted_id out.read_ports.req.ready out.read_ports.resp.data out.read_ports.resp.id out.read_ports.resp.valid out.write_ports.req.accepted out.write_ports.req.ready out.write_ports.resp.id out.write_ports.resp.resp out.write_ports.resp.valid
//   CONTRACT comb_inputs  in: out.read_ports.req.addr out.read_ports.req.bypass out.read_ports.req.id out.read_ports.req.pc_hash out.read_ports.req.total_size out.read_ports.req.valid out.write_ports.req.addr out.write_ports.req.bypass out.write_ports.req.id out.write_ports.req.ready out.write_ports.req.total_size out.write_ports.req.valid out.write_ports.req.wdata out.write_ports.req.wstrb  out: out.read_ports.req.ready out.write_ports.req.ready
class AxiSubsystem {
public:
  axi_interconnect::AXI_Interconnect interconnect;
  sim_ddr::SimDDR ddr;
  mmio::MMIO_Bus_AXI4 mmio;
  mmio::UART16550_Device uart{MMIO_RANGE_BASE};
  MemSubsystem *llc = nullptr;   // the LLC's lookup table (MemSubsystem::llc_*), bound by SimCpu::init
  struct Out {   // the masters' ports of the interconnect (its read_ports / write_ports), bound by SimCpu::init
    axi_interconnect::ReadMasterPort_t *read_ports = nullptr;
    axi_interconnect::WriteMasterPort_t *write_ports = nullptr;
  } out;

  void init();
  void sync_uart_from_backing();
  // the masters' request valid cleared (a master that requests this cycle sets it and drives the other fields;
  // resp.ready stays: the interconnect samples it at the clock edge)
  void comb_begin();
  // the LLC lookup, then slave outputs -> interconnect outputs (req.ready / accepted / resp on the master ports)
  void comb_outputs();
  // master requests -> interconnect -> slave inputs
  void comb_inputs();
  void seq();   // the LLC table update, then the interconnect and the slaves

private:
  static void connect_slave_outputs(sim_ddr::SimDDR_IO_t &master_io, const sim_ddr::SimDDR_IO_t &slave_io);
  static void connect_master_outputs(const sim_ddr::SimDDR_IO_t &master_io, sim_ddr::SimDDR_IO_t &slave_io);
};
