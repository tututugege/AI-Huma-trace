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

class FrontTop {
public:
  front_top_in in;
  front_top_out out;
  SimContext *ctx = nullptr;
  PtwWalkPort *icache_ptw_walk_port = nullptr;
  ICacheMemPortReq *icache_mem_req_port = nullptr;
  ICacheMemPortResp *icache_mem_resp_port = nullptr;
  axi_interconnect::ReadMasterPort_t *icache_mem_read_port = nullptr;    // the I-cache drives its requests here
  const axi_interconnect::ReadMasterPort_t *icache_mem_read_q = nullptr; // and reads the port's outputs here (a register)
  front_top_in back2front = {};
  front_top_in back2front_1 = {};
  reg<1> back2front_stall = false;
  reg<1> back2front_stall_1 = false;

  FrontTop();
  ~FrontTop();
  void init();
  void seq_read_bpu();
  void comb_bpu();
  void seq_write_bpu();
  void seq_back2front();
  void comb(CsrStatusIO *csr_status, FrontPreIO &to_back);   // one cycle; the fetch group for the back end
  void comb_from_back(BackTop &back);                        // next cycle's inputs from the back end (back2front_1)
  void force_back2front_refetch(pc_t pc);
  void step_bpu();

private:
  std::unique_ptr<FrontReadData> read_data_;
  std::unique_ptr<FrontUpdateRequest> update_req_;
};
