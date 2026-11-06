#ifndef ICACHE_TOP_H
#define ICACHE_TOP_H

// ICacheTop is glue logic only:
// - it binds MMU/PTW/AXI runtime objects
// - it folds external table responses into the icache_module interface
// - it translates external MMU/PTW/AXI views into explicit glue IO
//
// Do not add front-end timing/register state here. Request/miss/refill state
// must stay inside icache_module or dedicated external adapters. Top-level
// perf/accounting should also stay outside this glue layer.

#include "../../front_IO.h"
#include <cstdint>

class SimContext;
class PtwMemPort;
class PtwWalkPort;
struct ICacheMemPortReq;
struct ICacheMemPortResp;
namespace axi_interconnect {
struct ReadMasterPort_t;
}

class ICacheTop {
protected:
  struct icache_in *in = nullptr;
  struct icache_out *out = nullptr;
  SimContext *ctx = nullptr;

public:
  void setIO(struct icache_in *in_ptr, struct icache_out *out_ptr) {
    in = in_ptr;
    out = out_ptr;
  }

  void setContext(SimContext *c) { ctx = c; }

  virtual void comb() = 0;
  virtual void seq() = 0;
  virtual void peek_ready() = 0;
  virtual void dump_debug_state() const = 0;
  virtual void set_ptw_mem_port(PtwMemPort *port) { (void)port; }
  virtual void set_ptw_walk_port(PtwWalkPort *port) { (void)port; }
  virtual void set_mem_read_port(axi_interconnect::ReadMasterPort_t *port) {
    (void)port;
  }
  virtual void set_mem_probe_ports(ICacheMemPortReq *req_port,
                                   ICacheMemPortResp *resp_port) {
    (void)req_port;
    (void)resp_port;
  }

  // Fast-warmup replay (EXP-011): load a ready-made ITLB state (const TlbMmu::TlbEntry *, slot order, round-robin pointer).
  virtual void warm_install_itlb(const void *entries, int count, int repl_ptr) {
    (void)entries; (void)count; (void)repl_ptr;
  }

  virtual ~ICacheTop() {}
};

ICacheTop *get_icache_instance();

namespace icache_top_n {

inline constexpr int ICACHE_TOP_TRUE_GLUE_LINE_WORDS = ICACHE_LINE_SIZE / 4;

} // namespace icache_top_n

#endif
