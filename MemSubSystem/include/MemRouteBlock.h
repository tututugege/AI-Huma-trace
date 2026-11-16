#pragma once

#include "DcacheConfig.h"
#include "FrontTop.h"
#include "IO.h"
#include <cstdint>
#include <cstdio>
class SimContext;
constexpr uint32_t DcacheReqIdRouteBit = 1u << 31;
constexpr uint32_t MemRouteBlockReqID = DcacheReqIdRouteBit;

static inline bool dcache_req_id_is_route(uint32_t req_id) {
  return (req_id & DcacheReqIdRouteBit) != 0;
}

static inline uint32_t next_route_req_id(uint32_t req_id) {
  uint32_t next = req_id + 1;
  return dcache_req_id_is_route(next) ? next : MemRouteBlockReqID;
}

enum class Owner : uint8_t {
  NONE = 0,
  LSU,
  ICACHE,
  PTW_WALK,
};
struct PtwReq {
  wire<1> valid;
  wire<32> addr;
};
struct PtwGrant {
  wire<1> valid;
  Owner owner;
  uint32_t req_id;
};
struct PtwEvent {
  wire<1> valid;
  Owner owner;
  uint64_t data;   // the PTE for page-walk reads (rv64: 8 bytes)
  uint8_t replay;
  uint32_t req_addr;
  uint32_t req_id;
};
struct ReplayWakeup {
  wire<1> walk;   // the page-table walker's read must be re-issued
};
struct MemRouteIn {
  DcacheLsuIO *dcache_resp;

  DCacheReqPorts *lsu_req;
  ICacheMemPortReq *icache_req;

  PtwReq *ptw_walk_req;
};

struct MemRouteOut {
  LsuDcacheIO *dcache_req;

  DcacheLsuIO *lsu_resp;
  ICacheMemPortResp *icache_resp;

  PtwGrant *ptw_grant;
  PtwEvent *ptw_events;

  ReplayWakeup *wakeup;
};
struct IssueTag {
  Owner owner = Owner::NONE;
  uint32_t req_addr = 0;
  uint32_t req_id = 0;
};
struct MemRouteBlockState {
  IssueTag issued_tags;
  ReplayWakeup ptw_wait;

  wire<1> lsu_replay_valid;
  uint32_t lsu_replay_req_id;

  uint32_t next_req_id;
};

// Comb phases in this block's own ports (MemSubsystem::init binds them; scripts/comb/modules.py checks them):
//   CONTRACT comb_response  in: in.dcache_resp out.lsu_resp  out: out.icache_resp out.lsu_resp out.ptw_events out.wakeup
//   CONTRACT comb_request  in: in.icache_req in.lsu_req in.ptw_walk_req  out: out.dcache_req out.ptw_grant
class MemRouteBlock {
public:
  using Regs = MemRouteBlockState;   // registers: q (read in comb through a const reference, written only by seq / init) and d (comb writes it; d == q at the start of every cycle)
  Regs d;
  const Regs &q() const { return q_; }
  MemRouteIn in;
  MemRouteOut out;

  void init();
  void comb_response();
  void comb_request();
  void seq();


private:
  Regs q_;
};
