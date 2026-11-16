#include "trace_hook.h"
#include "PhysMemory.h"
#include "MemRouteBlock.h"
#include "types.h"


void MemRouteBlock::init() {
  q_ = {};
  q_.next_req_id = MemRouteBlockReqID;
}
void MemRouteBlock::comb_response() {
    const Regs &q = q_;

  out.icache_resp->resp_valid = out.icache_resp->resp_retry = out.icache_resp->resp_miss = false;
  out.lsu_resp->clear_valid();
  out.ptw_events->valid = false;
  out.wakeup->walk = false;
  wait_walk_ = -1;

  memcpy(out.lsu_resp, in.dcache_resp, sizeof(DcacheLsuIO));

  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    auto &resp = out.lsu_resp->resp_ports.load_resps[i];
    if (resp.valid && dcache_req_id_is_route(resp.req_id)) {
      resp.valid = false;
    }
  }

  if (q.issued_tags.owner != Owner::NONE && in.dcache_resp->resp_ports.load_resps[LSU_LDU_COUNT - 1].valid && in.dcache_resp->resp_ports.load_resps[LSU_LDU_COUNT - 1].req_id == q.issued_tags.req_id) {
    out.lsu_resp->resp_ports.load_resps[LSU_LDU_COUNT - 1].valid = q.lsu_replay_valid;
    out.lsu_resp->resp_ports.load_resps[LSU_LDU_COUNT - 1].req_id = q.lsu_replay_req_id;
    out.lsu_resp->resp_ports.load_resps[LSU_LDU_COUNT - 1].replay = ReplayType::CONFLICT; // Set replay type as needed

    Owner owner = q.issued_tags.owner;
    ReplayType replay = in.dcache_resp->resp_ports.load_resps[LSU_LDU_COUNT - 1].replay;
    if (replay == ReplayType::HIT) {
      if (owner == Owner::ICACHE) {
        out.icache_resp->resp_valid = true;
        out.icache_resp->resp_miss = false;
        // Instruction data is functional memory (the L1D stores no values in the trace-driven build).
        out.icache_resp->resp_data = tbhook::mem_read(q.issued_tags.req_addr);
      } else {
        out.ptw_events->valid = true;
        out.ptw_events->owner = owner;
        // Page table entries are functional memory too.
        out.ptw_events->data = tbhook::mem_read(q.issued_tags.req_addr);
        // an Sv39 PTE is 8 bytes (aligned): the second word is in the same line
        out.ptw_events->data |= static_cast<uint64_t>(tbhook::mem_read(q.issued_tags.req_addr + 4)) << 32;
        out.ptw_events->replay = 0;
        out.ptw_events->req_addr = q.issued_tags.req_addr;
        out.ptw_events->req_id = q.issued_tags.req_id;
      }
    } else if (replay == ReplayType::CONFLICT) {
      const bool icache_retry =
          (owner == Owner::ICACHE) &&
          in.dcache_resp->resp_ports.load_resps[LSU_LDU_COUNT - 1].retry;
      out.icache_resp->resp_valid = (owner == Owner::ICACHE);
      out.icache_resp->resp_retry = icache_retry;
      out.icache_resp->resp_miss = (owner == Owner::ICACHE) && !icache_retry;
      out.wakeup->walk = (owner == Owner::PTW_WALK);
    } else {
      out.icache_resp->resp_valid = (owner == Owner::ICACHE);
      out.icache_resp->resp_miss = (owner == Owner::ICACHE);
      if (in.dcache_resp->mshr_fill) {
        out.wakeup->walk = (owner == Owner::PTW_WALK);
      } else {
        wait_walk_ = (owner == Owner::PTW_WALK);   // the walker waits for the fill (seq)
      }
    }
  }
  if (in.dcache_resp->mshr_fill) {
    out.wakeup->walk |= q.ptw_wait.walk;
  }
}

void MemRouteBlock::comb_request() {
    const Regs &q = q_;
  out.ptw_grant->valid = false;
  out.dcache_req->req_ports.clear();
  out.dcache_req->icache_req = LSU_LDU_COUNT;

  memcpy(&out.dcache_req->req_ports, in.lsu_req, sizeof(DCacheReqPorts));

  Owner ptw_owner = Owner::NONE;
  uint32_t ptw_addr = 0;
  if (in.icache_req->req_valid && q.issued_tags.owner != Owner::ICACHE) {
    ptw_owner = Owner::ICACHE;
    ptw_addr = in.icache_req->req_addr;
  } else if (in.ptw_walk_req->valid && q.issued_tags.owner != Owner::PTW_WALK) {
    ptw_owner = Owner::PTW_WALK;
    ptw_addr = in.ptw_walk_req->addr;
  }

  req_owner_ = ptw_owner;   // the injected request of this cycle (seq records it)
  req_addr_ = ptw_addr;
  if (ptw_owner != Owner::NONE) {
    out.dcache_req->req_ports.load_ports[LSU_LDU_COUNT - 1].valid = true;
    out.dcache_req->req_ports.load_ports[LSU_LDU_COUNT - 1].addr = ptw_addr;
    out.dcache_req->req_ports.load_ports[LSU_LDU_COUNT - 1].req_id = q.next_req_id;
    out.dcache_req->req_ports.load_ports[LSU_LDU_COUNT - 1].replay = false;
    out.dcache_req->icache_req = (ptw_owner == Owner::ICACHE) ? LSU_LDU_COUNT - 1 : LSU_LDU_COUNT; // Inject into port 0
  }

  out.ptw_grant->valid = (ptw_owner != Owner::NONE) && (ptw_owner != Owner::ICACHE);
  out.ptw_grant->owner = ptw_owner;
  out.ptw_grant->req_id = q.next_req_id;
}
// The clock edge: the walker's wait for a fill; the request injected on the last load port this cycle (its owner and the
// LSU request it displaced, replayed with its id).
void MemRouteBlock::seq() {
  if (wait_walk_ >= 0) q_.ptw_wait.walk = wait_walk_;
  if (in.dcache_resp->mshr_fill) q_.ptw_wait = {};
  const auto req_id = q_.next_req_id;
  q_.issued_tags.owner = req_owner_;
  q_.issued_tags.req_addr = req_addr_;
  q_.issued_tags.req_id = req_id;
  if (req_owner_ != Owner::NONE) {
    q_.lsu_replay_valid = in.lsu_req->load_ports[LSU_LDU_COUNT - 1].valid;
    q_.lsu_replay_req_id = in.lsu_req->load_ports[LSU_LDU_COUNT - 1].req_id;
    q_.next_req_id = next_route_req_id(req_id);
  }
}
