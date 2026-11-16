#pragma once
// The memory subsystem's AXI masters (MSHR refills, write-buffer write-backs, the peripheral MMIO path) drive the
// interconnect's master ports directly. A master owns the request fields and resp.ready; the interconnect drives the
// rest (req.ready / accepted, resp.valid / data / id).
#include "AXI_Interconnect_IO.h"

inline void clear_master_fields(axi_interconnect::ReadMasterPort_t &p) {
  p.req.valid = false;
  p.req.addr = 0;
  p.req.total_size = 0;
  p.req.id = 0;
  p.resp.ready = false;
}

inline void clear_master_fields(axi_interconnect::WriteMasterPort_t &p) {
  p.req.valid = false;
  p.req.addr = 0;
  p.req.total_size = 0;
  p.req.id = 0;
  p.req.wstrb = 0;
  p.resp.ready = false;
}
