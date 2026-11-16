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
  MshrAxiIn   mshr_axi_in{};   // IC read-channel  → MSHR  (set by caller)
  MshrAxiOut  mshr_axi_out{};  // MSHR → IC read-channel   (read by caller)
  WbAxiIn     wb_axi_in{};     // IC write-channel → WB    (set by caller)
  WbAxiOut    wb_axi_out{};    // WB → IC write-channel    (read by caller)
  PeripheralAxiReadIn peripheral_axi_read_in{};
  PeripheralAxiReadOut peripheral_axi_read_out{};
  PeripheralAxiWriteIn peripheral_axi_write_in{};
  PeripheralAxiWriteOut peripheral_axi_write_out{};

  // PTW 端口连线（对外直接赋值/读取）
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
  DcacheLsuIO dcache_resp_pending_{};
  DcacheLsuIO dcache_resp_route_{};
  LsuDcacheIO dcache_empty_req_{};

  struct DcacheRespMeta {
    bool valid = false;
    bool is_store = false;
    uint32_t addr = 0;
  };
  DcacheRespMeta dcache_resp_meta_cur_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
  DcacheRespMeta dcache_resp_meta_nxt_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
  DcacheRespMeta dcache_resp_pending_meta_[LSU_LDU_COUNT + LSU_STA_COUNT]{};

  MSHRDcacheIO mshr_dcache_io_{};
  DcacheMSHRIO dcache_mshr_io_{};

  WBDcacheIO wb_dcache_io_{};
  DcacheWBIO dcache_wb_io_{};

  DcacheLineReadResp dcache_line_read_resp_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
  DcacheLineReadReq dcache_line_read_req_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
  PendingWrite pending_writes_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
  LruUpdate lru_updates_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
  FILLWrite fill_writes_{};
  FillIn fill_in_{};
  FillOut fill_out_{};
  DcacheS1Reg dcache_s1_q_{};   // register: stage-1 requests (find / bypass / merge, line reads, fill read) of the last cycle

  PtwReq ptw_walk_req;

  PtwGrant ptw_grant;
  PtwEvent ptw_events;

  ReplayWakeup wakeup;

  std::unique_ptr<AxiKitRuntime> axi_kit_runtime;

  void sync_ptw_port_outputs();
  void dcache_domain_comb_outputs();
  void dcache_domain_comb_inputs();
  void dcache_domain_seq(bool sample_mshr_perf);
  void record_dcache_stage1_metadata();
  void capture_dcache_raw_response();
  void forward_pending_response_from_fill();
  void publish_pending_dcache_response();
  void clear_pending_dcache_response();

  friend class MemSubsystemPtwWalkPortAdapter;

  std::unique_ptr<MemSubsystemPtwWalkPortAdapter> dtlb_walk_port_inst;
  std::unique_ptr<MemSubsystemPtwWalkPortAdapter> itlb_walk_port_inst;

  RealLsu *ptw_coherent_source_ = nullptr;
};
