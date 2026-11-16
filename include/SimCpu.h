#pragma once
#include "AXI_Interconnect.h"
#include "BackTop.h"
#include "FrontTop.h"
#include "MMIO_Bus_AXI4.h"
#include "MemSubsystem.h"
#include "SimDDR.h"
using AxiInterconnectImpl = axi_interconnect::AXI_Interconnect;
using AxiDdrImpl = sim_ddr::SimDDR;
using AxiMmioImpl = mmio::MMIO_Bus_AXI4;
#include "UART16550_Device.h"
#include "axi_mmio_map.h"
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
//   PORT fetch_group        = back.in.{inst,pc,is_rvc,valid,front_stall,front_upd_free,front_tq_free,predict_dir,meta,predict_next_fetch_address,page_fault_inst}
//   PORT dcache2lsu         = back.in.dcache2lsu
//   PORT lsu2dcache         = back.out.lsu2dcache
//   PORT mmio_req           = back.out.peripheral_req
//   PORT mmio_resp          = peripheral_resp_hold_1
//   PORT back2front         = back.out.{commit_entry,commit_extra,commit_ftq,fence_i,fire,flush,itlb_flush,mispred,redirect_pc,rf_blk,rf_is_rvc,rf_pc,rf_slot,rf_type,rf_valid,stall}
//   PORT dtlb_walk          = mem_subsystem.dtlb_walk
//   PORT itlb_walk          = mem_subsystem.itlb_walk
//   PORT icache_probe       = icache_req
//   PORT axi_ic_req         = axi_interconnect.read_ports.req.{valid,addr,total_size,id,bypass}
//   PORT axi_mem_req        = axi_interconnect.{read_ports,write_ports}.req.{valid,addr,total_size,id,bypass,wdata,wstrb}
//   PORT axi_ic_resp_ready  = axi_interconnect.read_ports.resp.ready        (sampled by the interconnect at the edge)
//   PORT axi_mem_resp_ready = axi_interconnect.{read_ports,write_ports}.resp.ready
//   PORT axi_slave          = axi_interconnect.{read_ports,write_ports}.req.{ready,accepted,accepted_id} axi_interconnect.{read_ports,write_ports}.resp.{valid,data,id,resp}
//   PORT axi_ready_next     = axi_interconnect.{read_ports,write_ports}.req.ready   (the I-cache's port register samples it)
// The glue phases (axi_*: the interconnect and its slaves; the others: SimCpu's registers):
//   EXPORT back_in_from_regs     default: back_in
//   EXPORT axi_comb_begin        default: axi_ic_req axi_mem_req
//   EXPORT axi_comb_outputs      in: axi_ic_req  pre: axi_mem_req  out: axi_slave
//   EXPORT axi_comb_inputs       in: axi_ic_req axi_mem_req  out: axi_ready_next
//   EXPORT front2back_hold_comb  in: fetch_group back2front
class SimCpu {
  // 性能计数器
public:
  ICacheMemPortReq icache_req{}; // 供 FrontTop 直接驱动的 ICache 请求端口（可选，取决于 CONFIG_ICACHE_USE_AXI_MEM_PORT）
  // The I-cache's view of its memory side is registered: the ICACHE read port of the interconnect (req.ready / accepted /
  // resp, driven by axi_interconnect.comb_outputs()) and the probe response (driven by mem_subsystem.comb_outputs())
  // both come after the front end in the cycle, so the front end sees them as of the previous cycle.
  axi_interconnect::ReadMasterPort_t icache_axi_q{};
  ICacheMemPortResp icache_resp_q{};
  void icache_port_regs_seq();
  ICacheMemPortResp icache_resp{}; // 供 FrontTop 直接驱动的 ICache 响应端口（可选，取决于 CONFIG_ICACHE_USE
  SimCpu() : back(&this->ctx), mem_subsystem(&this->ctx) {};
  SimContext ctx;
  BackTop back;
  FrontTop front;
  MemSubsystem mem_subsystem;
  AxiInterconnectImpl axi_interconnect;
  AxiDdrImpl axi_ddr;
  AxiMmioImpl axi_mmio;
  mmio::UART16550_Device axi_uart{MMIO_RANGE_BASE};
  // Oracle 模式下的一拍保留寄存，避免“后端当拍阻塞”导致前端指令丢失。
  bool oracle_pending_valid = false;
  front_top_out oracle_pending_out = {};
  // Explicit front-to-back hold register. Previously BackTop::in itself
  // retained this payload across a backend stall.
  FrontPreIO front2back_hold{};
  FrontPreIO front2back_hold_1{};
  bool front2back_hold_live = false;
  bool front2back_hold_1_live = false;
  // PeripheralAxi produces this response after BackTop::comb(). Model the
  // intended one-cycle delay explicitly instead of retaining BackTop::in.
  PeripheralRespIO peripheral_resp_hold{};
  PeripheralRespIO peripheral_resp_hold_1{};

  void init();
  void sync_mmio_devices_from_backing();
  void restore_pc(pc_t pc);
  void cycle();
  void front2back_hold_comb();
  void back_in_from_regs();
  void axi_comb_begin();
  void axi_comb_outputs();
  void axi_comb_inputs();
  void front2back_hold_seq();
  void peripheral_resp_hold_seq();
  uint32_t get_reg(uint8_t arch_idx) { return back.get_reg(arch_idx); }
  // 由 SimContext 在提交路径调用的本地辅助逻辑。
  void commit_sync(InstInfo *inst, int commit_slot);
};
