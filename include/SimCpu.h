#pragma once
#include "AxiSubsystem.h"
#include "BackTop.h"
#include "Back2Front.h"
#include "Front2Back.h"
#include "FrontTop.h"
#include "MemSubsystem.h"
#include "front_IO.h"

// SimCpu holds the tops and binds the ports between them (init). Every comb phase of the CPU is a phase of a module
// (scripts/comb/modules.py: the modules' CONTRACT lines in their own port names); scripts/comb/gen_top_order.py generates
// the comb part of cycle() from them (docs/UNIFIED_MODULES.md).
class SimCpu {
public:
  ICacheMemPortReq icache_req{}; // the I-cache probe request (FrontTop drives it; the memory subsystem reads it)
  // The I-cache's view of its memory side is registered: the ICACHE read port of the interconnect (req.ready / accepted /
  // resp, driven by axi.interconnect.comb_outputs()) and the probe response (driven by mem_subsystem.comb_outputs())
  // both come after the front end in the cycle, so the front end sees them as of the previous cycle.
  axi_interconnect::ReadMasterPort_t icache_axi_q{};
  ICacheMemPortResp icache_resp_q{};
  void icache_port_regs_seq();
  ICacheMemPortResp icache_resp{}; // the probe response (the memory subsystem drives it; the I-cache reads it through icache_resp_q)
  SimCpu() : back(&this->ctx), mem_subsystem(&this->ctx) {};
  SimContext ctx;
  BackTop back;
  FrontTop front;
  MemSubsystem mem_subsystem;
  AxiSubsystem axi;
  Front2Back f2b;   // front end -> back end (the RTL's core_front2back)
  Back2Front b2f;   // back end -> front end register (the RTL's core_back2front)

  void init();
  void sync_mmio_devices_from_backing();
  void restore_pc(pc_t pc);
  void cycle();
};
