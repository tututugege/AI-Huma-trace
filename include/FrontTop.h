#pragma once

#include "front_IO.h"
#include <memory>

class PtwWalkPort;
class BackTop;
struct FrontPreIO;
struct Back2FrontRegs;
struct CsrStatusIO;
class SimContext;
struct FrontReadData;
class FrontCore;
namespace axi_interconnect {
struct ReadMasterPort_t;
}

struct ICacheMemPortReq{
  bool req_valid = false;
  uint32_t req_addr = 0;
};
struct ICacheMemPortResp{
  bool resp_valid = false;
  bool resp_retry = false;
  bool resp_miss = false;
  uint32_t resp_data;
};

// SimCpu-level phases (include/SimCpu.h, scripts/comb/gen_top_order.py):
//   EXPORT comb_begin      in: csr_status b2f_reg  out: front_in front_out
// The front end's modules (core->...) have their own CONTRACT lines; SimCpu::cycle() calls their phases.
class FrontTop {
public:
  FrontTopIn in;
  FrontTopOut out;
  SimContext *ctx = nullptr;
  PtwWalkPort *icache_ptw_walk_port = nullptr;
  ICacheMemPortReq *icache_mem_req_port = nullptr;
  ICacheMemPortResp *icache_mem_resp_port = nullptr;
  axi_interconnect::ReadMasterPort_t *icache_mem_read_port = nullptr;    // the I-cache drives its requests here
  const axi_interconnect::ReadMasterPort_t *icache_mem_read_q = nullptr; // and reads the port's outputs here (a register)
  const Back2FrontRegs *from_back = nullptr;   // the back end's registered outputs (Back2Front's q), bound by SimCpu
  bool can_read = false;   // the back end takes a group from out this cycle (it did not stall, or a refetch is pending)

  FrontTop();
  ~FrontTop();
  void init();
  void read_core_q();
  void comb_core();
  void seq_core();
  void comb_begin(CsrStatusIO *csr_status);   // the start of a cycle; the front end's modules' phases follow
  // between cycles (a switch from fast mode): the fetch restarts at pc next cycle
  void restart_at(pc_t pc);
  void step_bpu();
  // fast-warmup replay (EXP-011): BPU training, ready-made L1I tag / valid and ITLB state
  void warm_commit(pc_t pc, br_type_t type, bool dir, target_addr_t target, bool rvc = false);
  void warm_finish();
  void warm_icache(const uint32_t tag[ICACHE_SET_NUM][ICACHE_WAY_NUM], const bool valid[ICACHE_SET_NUM][ICACHE_WAY_NUM]);
  void warm_itlb(const void *entries, int count, int repl_ptr);

  std::unique_ptr<FrontCore> core;   // all state of the front end, its modules (front-end/front_core.h)

private:
  std::unique_ptr<FrontReadData> read_data_;
  void bind_ports();
};
