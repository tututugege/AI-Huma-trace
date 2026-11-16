#pragma once

#include "front_IO.h"
#include <memory>

class PtwWalkPort;
class BackTop;
struct FrontPreIO;
struct CsrStatusIO;
class SimContext;
struct FrontReadData;
struct FrontUpdateRequest;
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
//   EXPORT comb            in: csr_status  pre: fetch_group  out: fetch_group axi_ic_req axi_ic_resp_ready icache_probe itlb_walk
//   EXPORT comb_from_back  in: csr_status back2front
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
  FrontTopIn back2front = {};
  FrontTopIn back2front_1 = {};
  reg<1> back2front_stall = false;
  reg<1> back2front_stall_1 = false;

  FrontTop();
  ~FrontTop();
  void init();
  void read_core_q();
  void comb_core();
  void seq_core();
  void seq_back2front();
  void comb(CsrStatusIO *csr_status, FrontPreIO &to_back);   // one cycle; the fetch group for the back end
  void comb_from_back(BackTop &back);                        // next cycle's inputs from the back end (back2front_1)
  // between cycles (a switch from fast mode): the fetch restarts at pc next cycle
  void restart_at(pc_t pc);
  void step_bpu();
  // fast-warmup replay (EXP-011): BPU training, ready-made L1I tag / valid and ITLB state
  void warm_commit(pc_t pc, br_type_t type, bool dir, target_addr_t target, bool rvc = false);
  void warm_finish();
  void warm_icache(const uint32_t tag[ICACHE_SET_NUM][ICACHE_WAY_NUM], const bool valid[ICACHE_SET_NUM][ICACHE_WAY_NUM]);
  void warm_itlb(const void *entries, int count, int repl_ptr);

private:
  std::unique_ptr<FrontCore> core_;   // all state of the front end (front-end/front_core.h)
  std::unique_ptr<FrontReadData> read_data_;
  std::unique_ptr<FrontUpdateRequest> update_req_;
  void bind_ports();
};
