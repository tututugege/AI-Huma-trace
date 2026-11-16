#pragma once

#include "IO.h"
#include "MSHR.h"
#include "MemPtwBlock.h"
#include "PeripheralAxi.h"
#include "PeripheralModel.h"
#include "MemRouteBlock.h"
#include "PtwWalkPort.h"
#include "RealDcache.h"
#include "WriteBuffer.h"
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <FrontTop.h>


class SimContext;
class Csr;
class MemSubsystemPtwWalkPortAdapter;
class RealLsu;
struct AxiKitRuntime;
namespace axi_interconnect {
struct ReadMasterPort_t;
struct AXI_LLCConfig;
struct AXI_LLC_LookupIn_t;
struct AXI_LLC_TableOut_t;
}
// SimCpu-level phases (include/SimCpu.h, scripts/comb/gen_top_order.py):
//   EXPORT comb_outputs  in: axi_slave  out: dcache2lsu
//   EXPORT comb_inputs   in: lsu2dcache mmio_req icache_probe itlb_walk dtlb_walk axi_slave  out: axi_mem_req axi_mem_resp_ready mmio_resp
// Inside: the blocks declare their comb methods in their own headers (their own ports); MemSubsystem's own comb functions
// that drive a block are declared here, in MemSubsystem's signal names, with the block. scripts/comb/gen_mem_order.py
// checks both against the code and generates comb_outputs / comb_inputs (a block's phases in order; a writer before
// the readers; the outputs / inputs split derived from dcache2lsu and the back end's / front end's requests; ties by
// the priority):
//   CONTRACT priority: wb_ mshr_ dcache_ mem_route_block ptw_block peripheral_axi_
//   CONTRACT dcache_stage2 (dcache_)  in: dcache_line_read_resp_ dcache_mshr_io_.mshr_req dcache_resp_meta_cur_ dcache_resp_raw_ dcache_resp_route_ dcache_s1_q_ fill_in_ fill_writes_ lru_updates_ mshr_dcache_io_ pending_writes_ wb_dcache_io_  out: dcache_mshr_io_.fill_resp dcache_mshr_io_.mshr_req dcache_resp_raw_ dcache_resp_route_ dcache_wb_io_.dirty_info fill_in_ fill_store_dirty_ fill_writes_ lru_updates_ pending_writes_
//   CONTRACT ptw_request (ptw_block)  in: dtlb_walk_port_inst itlb_walk_port_inst  out: ptw_walk_req
//   CONTRACT dcache_stage1 (dcache_)  in: dcache_mshr_io_.find_req dcache_req_mux_ dcache_resp_meta_nxt_ dcache_resp_route_ dcache_wb_io_.bypass_req dcache_wb_io_.merge_req fill_store_dirty_ fill_writes_ mshr_dcache_io_  out: dcache_line_read_req_ dcache_mshr_io_.find_req dcache_resp_meta_nxt_ dcache_wb_io_.bypass_req dcache_wb_io_.merge_req fill_out_
//   CONTRACT ptw_feedback (ptw_block)  in: dtlb_walk_port_inst itlb_walk_port_inst ptw_events ptw_grant wakeup  out: -
class MemSubsystem {
public:
  enum class PtwClient : uint8_t {
    DTLB = 0,
    ITLB = 1,
    NUM_CLIENTS = 2,
  };

  explicit MemSubsystem(SimContext *ctx);
  ~MemSubsystem();

  ICacheMemPortResp *icache_resp = nullptr;
  ICacheMemPortReq *icache_req = nullptr;

  // External ports — LSU <-> DCache (RealDcache multi-port interface)
  LsuDcacheIO  *lsu2dcache  = nullptr;  // LSU → DCache requests
  DcacheLsuIO  *dcache2lsu  = nullptr;  // DCache → LSU responses

  // AXI interfaces exposed for the memory interconnect to drive/read each cycle.
  // the interconnect's master ports the D-cache (MSHR refills, write-buffer write-backs) and the MMIO path drive directly
  // (bound by SimCpu::init before init())
  axi_interconnect::ReadMasterPort_t *dcache_axi_r = nullptr;
  axi_interconnect::WriteMasterPort_t *dcache_axi_w = nullptr;
  axi_interconnect::ReadMasterPort_t *extra_axi_r = nullptr;
  axi_interconnect::WriteMasterPort_t *extra_axi_w = nullptr;

  // the PTW walk ports of the TLBs (bound by SimCpu)
  PtwWalkPort *dtlb_walk_port = nullptr;
  PtwWalkPort *itlb_walk_port = nullptr;
  PeripheralReqIO *peripheral_req = nullptr;
  PeripheralRespIO *peripheral_resp = nullptr;
  CsrInterruptInjectIO *csr_interrupt_inject = nullptr;

  void init();
  void comb_outputs();
  void comb_inputs();
  void seq();
  void configure_interrupt_latency(uint64_t seed, uint32_t min_cycles,
                                   uint32_t max_cycles);
  bool dcache_warm_access(uint32_t paddr, bool is_store, uint32_t *wb_line) { return dcache_.arrays.warm_access(paddr, is_store, wb_line); }   // fast-warmup replay
  void on_commit_store(uint32_t paddr, uint64_t data, uint8_t func3);
  void sync_mmio_devices_from_backing();
  void set_ptw_coherent_source(RealLsu *lsu) { ptw_coherent_source_ = lsu; }
  void set_llc_config(const axi_interconnect::AXI_LLCConfig &cfg);
  bool timer_irq_level() const { return peripheral_model_.timer_irq_level(); }
  // Fast-warmup replay (EXP-011): write one LLC set (per way: tag, valid, dirty) and its round-robin pointer.
  void llc_warm_set(uint32_t set, const uint32_t *tag, const uint8_t *valid, const uint8_t *dirty, uint32_t repl_next);
  void llc_comb_outputs();
  const axi_interconnect::AXI_LLC_LookupIn_t &llc_lookup_in() const;
  void llc_seq(const axi_interconnect::AXI_LLC_TableOut_t &table_out);

  PeripheralAxi &get_peripheral_axi() { return peripheral_axi_; }
  const PeripheralAxi &get_peripheral_axi() const { return peripheral_axi_; }

private:
  // Sub-modules
  RealDcache  dcache_;
  MSHR          mshr_;
  WriteBuffer   wb_;
  PeripheralAxi peripheral_axi_;
  PeripheralModel peripheral_model_;
  MemPtwBlock   ptw_block;
  MemRouteBlock mem_route_block;
  LsuDcacheIO dcache_req_mux_{};
  DcacheLsuIO dcache_resp_raw_{};
  DcacheLsuIO dcache_resp_route_{};
  LsuDcacheIO dcache_empty_req_{};

  struct DcacheRespMeta {
    bool valid = false;
    bool is_store = false;
    uint32_t addr = 0;
  };
  DcacheRespMeta dcache_resp_meta_cur_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
  DcacheRespMeta dcache_resp_meta_nxt_[LSU_LDU_COUNT + LSU_STA_COUNT]{};

  MSHRDcacheIO mshr_dcache_io_{};
  DcacheMSHRIO dcache_mshr_io_{};

  WBDcacheIO wb_dcache_io_{};
  DcacheWBIO dcache_wb_io_{};

  DcacheLineReadResp dcache_line_read_resp_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
  DcacheLineReadReq dcache_line_read_req_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
  PendingWrite pending_writes_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
  LruUpdate lru_updates_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
  FillWrite fill_writes_{};
  FillIn fill_in_{};
  FillOut fill_out_{};
  bool fill_store_dirty_ = false;   // drive_dcache_response -> DCache stage 1: a store merged into the arriving fill
  DcacheS1Reg dcache_s1_q_{};   // register: stage-1 requests (find / bypass / merge, line reads, fill read) of the last cycle

  PtwReq ptw_walk_req;

  PtwGrant ptw_grant;
  PtwEvent ptw_events;

  ReplayWakeup wakeup;

  std::unique_ptr<AxiKitRuntime> axi_kit_runtime;

  void sync_ptw_port_outputs();
  void dcache_stage2();
  void dcache_stage1();
  void ptw_request();
  void ptw_feedback();
  void dcache_domain_seq(bool sample_mshr_perf);
  void record_dcache_stage1_metadata();
  void drive_dcache_response();

  friend class MemSubsystemPtwWalkPortAdapter;

  std::unique_ptr<MemSubsystemPtwWalkPortAdapter> dtlb_walk_port_inst;
  std::unique_ptr<MemSubsystemPtwWalkPortAdapter> itlb_walk_port_inst;

  RealLsu *ptw_coherent_source_ = nullptr;
};
