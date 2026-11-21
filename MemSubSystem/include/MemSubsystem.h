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
// The memory subsystem's modules (the blocks below) declare their comb phases above their classes, in their own port
// names; MemSubsystem::init binds their ports (scripts/comb/modules.py, gen_top_order.py).
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

  // the TLBs' requests to the page-table walker (their walk ports: written by the TLBs, read and cleared by the walker)
  PtwWalkPortCombIn dtlb_walk_req{}, itlb_walk_req{};
  PeripheralReqIO *peripheral_req = nullptr;
  PeripheralRespIO *peripheral_resp = nullptr;
  CsrInterruptInjectIO *csr_interrupt_inject = nullptr;
  const CommitStoreIO *commit_store = nullptr;   // the back end's device stores of this cycle's commit

  void init();
  void seq();
  void configure_interrupt_latency(uint64_t seed, uint32_t min_cycles,
                                   uint32_t max_cycles);
  bool dcache_warm_access(uint32_t paddr, bool is_store, uint32_t *wb_line) { return dcache_.arrays.warm_access(paddr, is_store, wb_line); }   // fast-warmup replay
  // Read-only dirty-line snoop on cycle-start DCache + WriteBuffer registers (ICache FDIP coherency)
  bool is_line_dirty_q(uint32_t paddr) const {
    const uint32_t line = paddr & ~uint32_t(DCACHE_LINE_SIZE - 1u);
    const AddrFields f = decode(line);
    for (int w = 0; w < DCACHE_WAYS_NUM; ++w)
      if (dcache_.arrays.valid_array[f.set_idx][w] && dcache_.arrays.dirty_array[f.set_idx][w] &&
          dcache_.arrays.tag_array[f.set_idx][w] == f.tag)
        return true;
    const auto &wq = wb_.q();
    for (int i = 0; i < int(wq.count); ++i) {
      const int idx = (int(wq.head) + i) % DCACHE_WB_ENTRIES;
      if ((wq.write_buffer[idx].addr & ~uint32_t(DCACHE_LINE_SIZE - 1u)) == line) return true;
    }
    return false;
  }
  void sync_mmio_devices_from_backing();
  void set_ptw_coherent_source(RealLsu *lsu) { ptw_coherent_source_ = lsu; }
  void set_llc_config(const axi_interconnect::AXI_LLCConfig &cfg);

  // Fast-warmup replay (EXP-011): write one LLC set (per way: tag, valid, dirty) and its round-robin pointer.
  void llc_warm_set(uint32_t set, const uint32_t *tag, const uint8_t *valid, const uint8_t *dirty, uint32_t repl_next);
  void llc_comb_outputs();
  const axi_interconnect::AXI_LLC_LookupIn_t &llc_lookup_in() const;
  void llc_seq(const axi_interconnect::AXI_LLC_TableOut_t &table_out);

  // Modules: their comb phases (the CONTRACT lines above their classes) are SimCpu-level phases
  // (scripts/comb/gen_top_order.py schedules them with the other modules')
  RealDcache  dcache_;
  MSHR          mshr_;
  WriteBuffer   wb_;
  PeripheralAxi peripheral_axi_;
  PeripheralModel peripheral_model_;
  MemPtwBlock   ptw_block;
  MemRouteBlock mem_route_block;

private:
  LsuDcacheIO dcache_req_mux_{};
  DcacheLsuIO dcache_resp_route_{};
  LsuDcacheIO dcache_empty_req_{};

  MSHRDcacheIO mshr_dcache_io_{};
  DcacheMSHRIO dcache_mshr_io_{};

  WBDcacheIO wb_dcache_io_{};
  DcacheWBIO dcache_wb_io_{};

  PtwReq ptw_walk_req;

  PtwGrant ptw_grant;
  PtwEvent ptw_events;

  ReplayWakeup wakeup;

  std::unique_ptr<AxiKitRuntime> axi_kit_runtime;

  void dcache_domain_seq(bool sample_mshr_perf);



  RealLsu *ptw_coherent_source_ = nullptr;
};
