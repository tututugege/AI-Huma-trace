#pragma once
// I-cache top: the tag / valid arrays, the ITLB (TlbMmu), the memory ports and the icache_module state machine, one cycle per
// comb_calc() call (comb and the I-cache's own clock edge: the I-cache steps inside the front end's comb). The module computes the request / miss / refill state; this file only connects it:
//   arrays (register file, read in the cycle of the lookup) -> module meta lookup -> ITLB translation -> module meta + data lookup
//   -> memory read / D-cache probe requests -> fetch-group output (instruction words from the functional memory) -> clock edge.
#include "../front_IO.h"
#include "../front_module.h"
#include "AXI_Interconnect_IO.h"
#include "GenericTable.h"
#include "TlbMmu.h"
#include "config.h"
#include "include/icache_module.h"
#include <functional>
#include <memory>

class PtwWalkPort;
class SimContext;
struct ICacheMemPortReq;
struct ICacheMemPortResp;

class ICacheTop {
public:
  // The ports, bound by FrontTop (they take effect at the I-cache's next step).
  struct Ports {
    PtwWalkPort *ptw_walk = nullptr;
    ICacheMemPortReq *probe_req = nullptr;                          // the D-cache probe (I-cache miss -> D-cache lookup)
    ICacheMemPortResp *probe_resp = nullptr;                        // its response, through SimCpu's register
    axi_interconnect::ReadMasterPort_t *read = nullptr;             // the AXI read master: requests driven this cycle
    const axi_interconnect::ReadMasterPort_t *read_q = nullptr;     // the port's outputs of the last cycle (SimCpu::icache_axi_q)
  };
  void set_ports(const Ports &p) { ports_set_ = p; }
  void set_context(SimContext *ctx) { bind_.ctx_set = ctx; }
  std::function<void(uint32_t index, uint32_t tag)> fill_hook;   // FE_IPF: demand fills are reported to the prefetcher

  bool read_ready(bool hold_for_recovery, bool reset) const;   // from the registers (before the comb of the cycle)
  void comb_calc(ICacheIn *in, ICacheOut *out);              // the fetch step R1 = N(R0)
  void invalidate_step(CsrStatusIO *csr_status, ICacheOut *out);   // a predecode flush: R2 = K(R1)
  void seq();                                                  // the clock edge
  bool peek_hit(uint32_t index, uint32_t ptag);
  bool fetch_second(uint32_t a_vpc, uint32_t a_paddr, uint32_t b_vpc, int nwords, uint32_t *words);
  bool peek_translate(uint32_t v_addr, CsrStatusIO *status, uint32_t &p_addr);
  bool prefetch_install(uint32_t index, uint32_t ptag);
  void warm_install(const uint32_t tag[ICACHE_SET_NUM][ICACHE_WAY_NUM], const bool valid[ICACHE_SET_NUM][ICACHE_WAY_NUM]);
  void warm_install_itlb(const void *entries, int count, int repl_ptr);

private:
  using TagTable = GenericTable<ICACHE_SET_NUM, ICACHE_WAY_NUM, ICACHE_TAG_BITS>;
  using ValidTable = GenericTable<ICACHE_SET_NUM, ICACHE_WAY_NUM, 1>;
  icache_module_n::ICache mod_;   // the I-cache module (rtl/frontend/icache/icache_ctl.sv)
  TagTable tags_;
  ValidTable valid_;
  Ports ports_set_;
  uint8_t prefetch_rot_[ICACHE_SET_NUM] = {};   // FE_IPF: rotating victim per set
  // The array read of this cycle, as the module sees it.
  struct LookupResp {
    bool meta_valid = false, data_valid = false;
    uint32_t index = 0;
    uint32_t tag[ICACHE_WAY_NUM] = {};
    bool valid[ICACHE_WAY_NUM] = {};
  } lookup_;

  // What the I-cache is bound to: the context, the ports (taken from ports_set_ at the next step), the ITLB.
  struct Binding {
    SimContext *ctx = nullptr, *ctx_set = nullptr;   // ctx_set: icache_set_context(); ctx: bound at the next cycle
    PtwWalkPort *ptw_walk = nullptr;
    axi_interconnect::ReadMasterPort_t *read = nullptr;         // requests (driven this cycle)
    const axi_interconnect::ReadMasterPort_t *read_q = nullptr;  // the port's outputs of the last cycle (SimCpu::icache_axi_q)
    ICacheMemPortReq *probe_req = nullptr;
    ICacheMemPortResp *probe_resp = nullptr;
    std::unique_ptr<TlbMmu> itlb;

    void bind(const Ports &p) {
      if (ptw_walk != p.ptw_walk) { ptw_walk = p.ptw_walk; if (itlb) itlb->set_ptw_walk_port(ptw_walk); }
      read = p.read;
      read_q = p.read_q;
      probe_req = p.probe_req;
      probe_resp = p.probe_resp;
      ctx = ctx_set;
    }
    TlbMmu *mmu() {
      if (!itlb && ctx) {
        itlb = std::make_unique<TlbMmu>(ctx, ITLB_ENTRIES, TlbMmu::Kind::ITLB);
        if (ptw_walk) itlb->set_ptw_walk_port(ptw_walk);
      }
      return itlb.get();
    }
  } bind_;

  // One I-cache cycle as in the RTL (rtl/frontend/icache/icache_ctl.sv): the fetch step R1 = N(R0) (comb_calc), on a
  // predecode flush R2 = K(R1) on top of it (invalidate_step); the module registers, the ITLB and the tag / valid arrays
  // are updated once, at the clock edge (seq). The table writes of the steps wait for the edge; a read of the arrays in
  // the same cycle sees them (forwarded, as through a write-through bypass).
  struct Edge {
    bool reset = false;                 // reset this cycle: arrays and module back to the reset state at the edge
    int n = 0;                          // steps of this cycle (1, or 2 with the invalidate)
    icache_module_n::ICacheTableWrite tw[2] = {};   // each step's table write, applied in order at the edge
    bool kill_step = false;             // inside invalidate_step (K must not use the ITLB)
  } edge_;

  template <class TagP, class ValidP> void forward_pending(uint32_t index, TagP &tag, ValidP &valid) const;
  static void clear_outputs(ICacheOut *out);
  void drive_fetch_group(ICacheOut *out);
  void drive_perf(ICacheOut *out, bool req_valid, bool itlb_invoked, TlbMmu::Result itlb_ret);
  void fill_lookup_meta();
  void fill_lookup_data();
  void comb(ICacheIn *in, ICacheOut *out);
  void step(ICacheIn *in, ICacheOut *out);
};
