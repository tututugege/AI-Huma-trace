#pragma once
#include "AxiSubsystem.h"
#include "BackTop.h"
#include "Back2Front.h"
#include "Front2Back.h"
#include "FrontTop.h"
#include "MemSubsystem.h"
#include "front_IO.h"

// SimCpu wires the tops together. Each top exports its comb phases with the ports they read and write (the lines above
// the class: FrontTop, BackTop, MemSubsystem; the AXI and SimCpu glue phases below); scripts/comb/gen_top_order.py checks
// them against the code and generates the comb part of SimCpu::cycle() from them (docs/MODULE_CONTRACT.md).
//   in:      read after this cycle's writer of the port
//   pre:     read before it (the value of the clock edge or the default driver), so it runs before that writer
//   out:     written (one writer per port; an alias names one master's part of a field the scan sees per port array)
//   default: driven first, every cycle, to a default the other writers override (an always_comb default assignment)
// The ports (fields as SimCpu paths; {a,b} lists alternatives):
//   PORT csr_status         = back.csr_status
//   PORT back_in            = back.in
//   PORT front_in           = front.in front.read_data   (the front end's inputs and registers of the cycle)
//   PORT front_out          = front.out front.can_read
//   PORT b2f_reg            = b2f.q_                 (Back2Front's register: the front end's inputs of the cycle)
//   PORT fetch_group        = back.in.{inst,pc,is_rvc,valid,front_stall,front_upd_free,front_tq_free,predict_dir,meta,predict_next_fetch_address,page_fault_inst}
//   PORT dcache2lsu         = back.in.dcache2lsu
//   PORT lsu2dcache         = back.out.lsu2dcache
//   PORT mmio_req           = back.out.peripheral_req
//   PORT mmio_resp          = back.lsu.mmio_resp_d
//   PORT commit_store       = back.out.commit_store
//   PORT csr_inject         = back.csr_interrupt_inject_io
//   PORT periph             = mem_subsystem.peripheral_model_      (the peripheral model's device state)
//   PORT back2front         = back.out.{commit_entry,commit_extra,commit_ftq,fence_i,fire,flush,itlb_flush,mispred,redirect_pc,rf_blk,rf_is_rvc,rf_pc,rf_slot,rf_type,rf_valid,stall}
//   PORT dtlb_walk          = mem_subsystem.dtlb_walk
//   PORT itlb_walk          = mem_subsystem.itlb_walk
//   PORT icache_probe       = icache_req
//   PORT axi_ic_req         = axi.interconnect.read_ports.req.{valid,addr,total_size,id,bypass}
//   PORT axi_mem_req        = axi.interconnect.{read_ports,write_ports}.req.{valid,addr,total_size,id,bypass,wdata,wstrb}
//   PORT axi_ic_resp_ready  = axi.interconnect.read_ports.resp.ready        (sampled by the interconnect at the edge)
//   PORT axi_mem_resp_ready = axi.interconnect.{read_ports,write_ports}.resp.ready
//   PORT axi_slave          = axi.interconnect.{read_ports,write_ports}.req.{ready,accepted,accepted_id} axi.interconnect.{read_ports,write_ports}.resp.{valid,data,id,resp}
//   PORT axi_ready_next     = axi.interconnect.{read_ports,write_ports}.req.ready   (the I-cache's port register samples it)
// The glue phases (axi_*: the interconnect and its slaves; the others: SimCpu's registers):
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
