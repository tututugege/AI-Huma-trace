#pragma once
#include "AXI_Interconnect.h"
#include "MMIO_Bus_AXI4.h"
#include "SimDDR.h"
#include "UART16550_Device.h"
#include "axi_mmio_map.h"

class MemSubsystem;

// The AXI side of the SoC: the interconnect (with the LLC data path), the DDR and MMIO slaves, the UART on the MMIO
// bus. The LLC's lookup table lives in the memory subsystem (llc, bound by SimCpu::init): comb_outputs runs its
// lookup first, seq its table update. SimCpu-level phases (ports in include/SimCpu.h, scripts/comb/gen_top_order.py):
//   EXPORT comb_begin    default: axi_ic_req axi_mem_req
//   EXPORT comb_outputs  in: axi_ic_req  pre: axi_mem_req  out: axi_slave
//   EXPORT comb_inputs   in: axi_ic_req axi_mem_req  out: axi_ready_next
class AxiSubsystem {
public:
  axi_interconnect::AXI_Interconnect interconnect;
  sim_ddr::SimDDR ddr;
  mmio::MMIO_Bus_AXI4 mmio;
  mmio::UART16550_Device uart{MMIO_RANGE_BASE};
  MemSubsystem *llc = nullptr;   // the LLC's lookup table (MemSubsystem::llc_*), bound by SimCpu::init

  void init();
  void sync_uart_from_backing();
  // the masters' request fields cleared (each master drives them anew every cycle; resp.ready stays: the
  // interconnect samples it at the clock edge)
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
