#pragma once
#include "AXI_Interconnect.h"
#include "MMIO_Bus_AXI4.h"
#include "SimDDR.h"
#include "UART16550_Device.h"
#include "axi_mmio_map.h"

// The AXI side of the SoC: the interconnect (with the LLC data path), the DDR and MMIO slaves, the UART on the MMIO
// bus. Its comb phases are SimCpu's axi_comb_* (the ports are declared in include/SimCpu.h); the LLC's lookup table
// lives in MemSubsystem, so SimCpu passes the lookup in and the table out.
class AxiSubsystem {
public:
  axi_interconnect::AXI_Interconnect interconnect;
  sim_ddr::SimDDR ddr;
  mmio::MMIO_Bus_AXI4 mmio;
  mmio::UART16550_Device uart{MMIO_RANGE_BASE};

  void init();
  void sync_uart_from_backing();
  // the masters' request fields cleared (each master drives them anew every cycle; resp.ready stays: the
  // interconnect samples it at the clock edge)
  void comb_begin();
  // slave outputs -> interconnect outputs (req.ready / accepted / resp on the master ports)
  void comb_outputs(const axi_interconnect::AXI_LLC_LookupIn_t &llc_lookup);
  // master requests -> interconnect -> slave inputs
  void comb_inputs();
  void seq();

private:
  static void connect_slave_outputs(sim_ddr::SimDDR_IO_t &master_io, const sim_ddr::SimDDR_IO_t &slave_io);
  static void connect_master_outputs(const sim_ddr::SimDDR_IO_t &master_io, sim_ddr::SimDDR_IO_t &slave_io);
};
