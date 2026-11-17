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

// The front end's fetch step is a phase of the I-cache (ports named after the front end's wires):
//   CONTRACT comb_fetch  in: in.axi_q in.csr_status in.fetch_addr_head in.global_refetch in.global_reset in.in.ctrl in.probe_resp out.itlb_walk.comb_out  out: out.axi.req.addr out.axi.req.bypass out.axi.req.id out.axi.req.total_size out.axi.req.valid out.axi.resp.ready out.ic_in out.ic_k out.ic_out out.itlb_walk.comb_input out.itlb_walk.seq_input out.probe_req
//   CONTRACT comb_invalidate  in: in.axi_q in.csr_status in.predecode_flush in.probe_resp out.itlb_walk.comb_out  out: out.axi.req.addr out.axi.req.bypass out.axi.req.id out.axi.req.total_size out.axi.req.valid out.axi.resp.ready out.ic_k out.itlb_walk.comb_input out.itlb_walk.seq_input out.probe_req
class ICacheTop {
public:
  struct In {
    const bool *global_reset = nullptr;
    const bool *global_refetch = nullptr;
    const FrontTopIn *in = nullptr;                 // the front end's inputs (redirect / fence / csr_status)
    const FetchAddrFifoOut *fetch_addr_head = nullptr;
    const bool *predecode_flush = nullptr;          // the checker's refetch (the invalidate step)
    CsrStatusIO *csr_status = nullptr;                             // the CSR's status (translation mode, privilege)
    const axi_interconnect::ReadMasterPort_t *axi_q = nullptr;   // the AXI read port's outputs of the last cycle (a register)
    const ICacheMemPortResp *probe_resp = nullptr;                 // the D-cache probe's response (a register)
  } in;
  struct Out {
    ICacheIn ic_in{};
    ICacheOut ic_out{};
    ICacheOut ic_k{};      // the outputs of the invalidate step K (a predecode flush), which replace ic_out
    axi_interconnect::ReadMasterPort_t *axi = nullptr;   // the AXI read master: requests driven this cycle
    ICacheMemPortReq *probe_req = nullptr;               // the D-cache probe (I-cache miss -> D-cache lookup)
    PtwWalkPort *itlb_walk = nullptr;                    // the ITLB's page-table walk port (its TLB, below, uses it)
  } out;
  void comb_fetch();
  void comb_invalidate();   // a predecode flush: the invalidate step K   // the address popped from the fetch-address FIFO into the I-cache
  void set_context(SimContext *ctx) { bind_.ctx_set = ctx; }
  // the ITLB's walk port is the I-cache's out.itlb_walk (FrontTop::init, after SimCpu bound it)
  void bind_walk() { itlb.set_ptw_walk_port(out.itlb_walk); }
  std::function<void(uint32_t index, uint32_t tag)> fill_hook;   // FE_IPF: demand fills are reported to the prefetcher

  bool read_ready(bool hold_for_recovery, bool reset) const;   // from the registers (before the comb of the cycle)
  void comb_calc(ICacheIn *ci, ICacheOut *co);              // the fetch step R1 = N(R0)
  void invalidate_step(CsrStatusIO *csr_status, ICacheOut *co);   // a predecode flush: R2 = K(R1)
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
  uint8_t prefetch_rot_[ICACHE_SET_NUM] = {};   // FE_IPF: rotating victim per set
  // The array read of this cycle, as the module sees it.
  struct LookupResp {
    bool meta_valid = false, data_valid = false;
    uint32_t index = 0;
    uint32_t tag[ICACHE_WAY_NUM] = {};
    bool valid[ICACHE_WAY_NUM] = {};
  } lookup_;

  // What the I-cache is bound to: the context (taken at the next step), the ITLB and its walk port.
  struct Binding {
    SimContext *ctx = nullptr, *ctx_set = nullptr;   // ctx_set: icache_set_context(); ctx: bound at the next cycle

    void bind() { ctx = ctx_set; }
  } bind_;
  TlbMmu itlb{ITLB_ENTRIES, TlbMmu::Kind::ITLB};   // the ITLB (looked up in the fetch step, its edge in seq)

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
  static void clear_outputs(ICacheOut *co);
  void drive_fetch_group(ICacheOut *co);
  void drive_perf(ICacheOut *co, bool req_valid, bool itlb_invoked, TlbMmu::Result itlb_ret);
  void fill_lookup_meta();
  void fill_lookup_data();
  void comb(ICacheIn *ci, ICacheOut *co);
  void step(ICacheIn *ci, ICacheOut *co);
};
