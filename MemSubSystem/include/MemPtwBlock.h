#pragma once

#include "PtwWalkPort.h"
#include "ref.h"
#include "types.h"
#define PTW_COMPACT_CORE_KERNEL_TOP_EVENT_SLOTS LSU_LDU_COUNT
#include <array>
#include <cstddef>
#include <cstdio>

class SimContext;

// Shared PTW submodule. Port-side requests are latched in comb_begin(), memory
// feedback is applied in comb_finish(), and seq() is the only state commit.
class MemPtwBlock {
public:
  enum class Client : uint8_t {
    DTLB = 0,
    ITLB = 1,
    NUM_CLIENTS = 2,
  };

  enum class WalkRespResult : uint8_t {
    HANDLED,
    DROPPED,
    IGNORED,
  };

  enum class GrantOwner : uint8_t {
    NONE = 0,
    MEM_DTLB,
    MEM_ITLB,
    WALK,
  };

  enum class RoutedEventOwner : uint8_t {
    NONE = 0,
    MEM_DTLB,
    MEM_ITLB,
    WALK,
  };

  struct MemClientIn {
    bool req_valid = false;
    uint32_t req_addr = 0;
    bool resp_consumed = false;
  };

  struct MemClientOut {
    bool req_ready = false;
    bool resp_valid = false;
    uint32_t resp_data = 0;
  };

  struct WalkClientIn {
    bool req_valid = false;
    PtwWalkReq req = {};
    bool resp_consumed = false;
    bool flush = false;
  };

  struct WalkClientOut {
    bool req_ready = false;
    bool resp_valid = false;
    PtwWalkResp resp = {};
  };

  struct PortIn {
    std::array<MemClientIn, static_cast<size_t>(Client::NUM_CLIENTS)>
        mem_clients{};
    std::array<WalkClientIn, static_cast<size_t>(Client::NUM_CLIENTS)>
        walk_clients{};
  };

  struct SeqIn {
    std::array<bool, static_cast<size_t>(Client::NUM_CLIENTS)>
        walk_client_flush{};
  };

  struct RoutedEvent {
    bool valid = false;
    RoutedEventOwner owner = RoutedEventOwner::NONE;
    // Memory-side resolved 32-bit word presented to PTW for this event. The
    // surrounding memory subsystem may rewrite the raw routed response into the
    // PTW-visible value before filling FeedbackIn.
    uint32_t data = 0;
    uint8_t replay = 0;
    uint32_t req_addr = 0;
    size_t req_id = 0;
  };

  struct FeedbackIn {
    bool grant_valid = false;
    GrantOwner grant_owner = GrantOwner::NONE;
    size_t grant_req_id = 0;
    std::array<RoutedEvent, LSU_LDU_COUNT> events{};
    uint8_t event_count = 0;
    bool wakeup_dtlb = false;
    bool wakeup_itlb = false;
    bool wakeup_walk = false;
  };

  struct CombOut {
    std::array<MemClientOut, static_cast<size_t>(Client::NUM_CLIENTS)>
        mem_clients{};
    std::array<WalkClientOut, static_cast<size_t>(Client::NUM_CLIENTS)>
        walk_clients{};
    bool issue_walk_read = false;
    uint32_t walk_read_addr = 0;
    bool mem_req_pending[2] = {false, false};
    uint32_t mem_req_addr[2] = {0, 0};
  };

  explicit MemPtwBlock(SimContext *ctx = nullptr) : ctx(ctx) { init(); }

  void bind_context(SimContext *c) { ctx = c; }

  void init() {
    reset_state(cur_);
    reset_state(nxt_);
    reset_outputs(comb_);
    refresh_outputs();
  }

  void comb_begin(const PortIn &in) {
    nxt_ = cur_;
    reset_outputs(comb_);
    apply_port_inputs(in);
    count_wait_cycles();
    select_walk_owner();
    refresh_outputs();
  }

  void comb_finish(const FeedbackIn &in) {
    apply_grant(in);
    apply_events(in);
    apply_wakeups(in);
    refresh_outputs();
  }

  void seq() {
    SeqIn in{};
    seq(in);
  }

  void seq(const SeqIn &in) {
    cur_ = nxt_;
    apply_seq_inputs(cur_, in);
    nxt_ = cur_;
    refresh_outputs();
  }

  const CombOut &comb_outputs() const { return comb_; }


private:
  struct PtwClientState {
    bool req_pending = false;
    bool req_inflight = false;
    uint32_t req_addr = 0;
    bool resp_valid = false;
    uint32_t resp_data = 0;
  };

  struct WalkClientState {
    bool req_pending = false;
    bool req_inflight = false;
    PtwWalkReq req = {};
    bool resp_valid = false;
    PtwWalkResp resp = {};
  };

  enum class WalkState : uint8_t {
    IDLE,
    L1_REQ,
    L1_WAIT_RESP,
    L2_REQ,
    L2_WAIT_RESP,
  };

  static constexpr size_t kClientCount =
      static_cast<size_t>(Client::NUM_CLIENTS);

  struct State {
    std::array<PtwClientState, kClientCount> mem_clients{};
    std::array<WalkClientState, kClientCount> walk_clients{};
    WalkState walk_state = WalkState::IDLE;
    bool walk_active = false;
    Client walk_owner = Client::DTLB;
    Client walk_rr_next = Client::DTLB;
    uint32_t walk_l1_pte = 0;
    uint32_t walk_drop_resp_credit = 0;
    bool walk_req_id_valid = false;
    size_t walk_req_id = 0;
  };

  static size_t client_idx(Client c) { return static_cast<size_t>(c); }

  static void reset_state(State &state) {
    state = {};
    state.walk_state = WalkState::IDLE;
    state.walk_owner = Client::DTLB;
    state.walk_rr_next = Client::DTLB;
  }

  static void reset_outputs(CombOut &out) { out = {}; }

  static bool walk_state_waits_for_response(WalkState state) {
    return state == WalkState::L1_WAIT_RESP ||
           state == WalkState::L2_WAIT_RESP;
  }

  static void compact_core_comb(const State &state_q, const PortIn &port_in,
                                const SeqIn &seq_in,
                                const FeedbackIn &feedback, State &state_d) {
    MemPtwBlock block(nullptr);
    block.cur_ = state_q;
    block.nxt_ = state_q;
    reset_outputs(block.comb_);
    block.apply_port_inputs(port_in);
    block.select_walk_owner();
    block.apply_grant(feedback);
    block.apply_events(feedback);
    block.apply_wakeups(feedback);
    apply_seq_inputs(block.nxt_, seq_in);
    state_d = block.nxt_;
  }

  void apply_port_inputs(const PortIn &in) {
    for (size_t i = 0; i < kClientCount; i++) {
      const Client client = static_cast<Client>(i);
      auto &mem_state = nxt_.mem_clients[i];
      if (in.mem_clients[i].resp_consumed) {
        mem_state.resp_valid = false;
      }
      if (in.mem_clients[i].req_valid) {
        send_mem_req(client, in.mem_clients[i].req_addr);
      }

      auto &walk_state = nxt_.walk_clients[i];
      if (in.walk_clients[i].resp_consumed) {
        walk_state.resp_valid = false;
      }
      if (in.walk_clients[i].req_valid) {
        send_walk_req(client, in.walk_clients[i].req);
      }
    }
  }

  static void apply_seq_inputs(State &state, const SeqIn &in) {
    for (size_t i = 0; i < kClientCount; i++) {
      if (in.walk_client_flush[i]) {
        flush_walk_client_state(state, static_cast<Client>(i));
      }
    }
  }

  void count_wait_cycles() const {
    if (ctx == nullptr) {
      return;
    }
    if (nxt_.mem_clients[client_idx(Client::DTLB)].req_pending ||
        nxt_.mem_clients[client_idx(Client::DTLB)].req_inflight ||
        nxt_.walk_clients[client_idx(Client::DTLB)].req_pending ||
        nxt_.walk_clients[client_idx(Client::DTLB)].req_inflight) {

    }
    if (nxt_.mem_clients[client_idx(Client::ITLB)].req_pending ||
        nxt_.mem_clients[client_idx(Client::ITLB)].req_inflight ||
        nxt_.walk_clients[client_idx(Client::ITLB)].req_pending ||
        nxt_.walk_clients[client_idx(Client::ITLB)].req_inflight) {

    }
  }

  void select_walk_owner() {
    if (nxt_.walk_active || nxt_.walk_state != WalkState::IDLE) {
      return;
    }

    auto grant_walk_owner = [&](Client owner) {
      auto &wc = nxt_.walk_clients[client_idx(owner)];
      wc.req_pending = false;
      wc.req_inflight = true;
      nxt_.walk_owner = owner;
      nxt_.walk_active = true;
      nxt_.walk_state = WalkState::L1_REQ;
      nxt_.walk_l1_pte = 0;
      nxt_.walk_rr_next = (owner == Client::DTLB) ? Client::ITLB
                                                  : Client::DTLB;
    };

    const Client first = nxt_.walk_rr_next;
    const Client second = (first == Client::DTLB) ? Client::ITLB
                                                  : Client::DTLB;
    if (nxt_.walk_clients[client_idx(first)].req_pending) {
      grant_walk_owner(first);
    } else if (nxt_.walk_clients[client_idx(second)].req_pending) {
      grant_walk_owner(second);
    }
  }

  static void project_outputs_from_state(const State &state, CombOut &out) {
    reset_outputs(out);
    for (size_t i = 0; i < kClientCount; i++) {
      const auto &mem_state = state.mem_clients[i];
      out.mem_clients[i].req_ready =
          !mem_state.req_pending && !mem_state.req_inflight;
      out.mem_clients[i].resp_valid = mem_state.resp_valid;
      out.mem_clients[i].resp_data = mem_state.resp_data;
      out.mem_req_pending[i] = mem_state.req_pending;
      out.mem_req_addr[i] = mem_state.req_addr;

      const auto &walk_state = state.walk_clients[i];
      out.walk_clients[i].req_ready =
          !walk_state.req_pending && !walk_state.req_inflight;
      out.walk_clients[i].resp_valid = walk_state.resp_valid;
      out.walk_clients[i].resp = walk_state.resp;
    }

    if (!state.walk_active) {
      return;
    }

    const auto &req = state.walk_clients[client_idx(state.walk_owner)].req;
    if (state.walk_state == WalkState::L1_REQ) {
      const uint32_t root_ppn = req.satp & 0x3FFFFF;
      const uint32_t vpn1 = (req.vaddr >> 22) & 0x3FF;
      out.issue_walk_read = true;
      out.walk_read_addr = (root_ppn << 12) + (vpn1 << 2);
    } else if (state.walk_state == WalkState::L2_REQ) {
      const uint32_t ppn = (state.walk_l1_pte >> 10) & 0x3FFFFF;
      const uint32_t vpn0 = (req.vaddr >> 12) & 0x3FF;
      out.issue_walk_read = true;
      out.walk_read_addr = (ppn << 12) + (vpn0 << 2);
    }
  }

  void refresh_outputs() { project_outputs_from_state(nxt_, comb_); }

  void apply_grant(const FeedbackIn &in) {
    if (!in.grant_valid) {
      return;
    }

    switch (in.grant_owner) {
    case GrantOwner::MEM_DTLB:
      on_mem_read_granted(Client::DTLB);
      break;
    case GrantOwner::MEM_ITLB:
      on_mem_read_granted(Client::ITLB);
      break;
    case GrantOwner::WALK:
      on_walk_read_granted(in.grant_req_id);
      break;
    default:
      break;
    }
  }

  void apply_events(const FeedbackIn &in) {
    for (uint8_t i = 0; i < in.event_count; i++) {
      const auto &evt = in.events[i];
      if (!evt.valid) {
        continue;
      }
      switch (evt.owner) {
      case RoutedEventOwner::MEM_DTLB:
        if (evt.replay == 0) {
          on_mem_resp_client(Client::DTLB, evt.data);
        }
        break;
      case RoutedEventOwner::MEM_ITLB:
        if (evt.replay == 0) {
          on_mem_resp_client(Client::ITLB, evt.data);
        }
        break;
      case RoutedEventOwner::WALK:
        if (evt.replay == 0) {
          (void)on_walk_mem_resp(evt.req_id, evt.data);
        } else {
          (void)on_walk_mem_replay(evt.req_id, evt.replay);
        }
        break;
      default:
        break;
      }
    }
  }

  void apply_wakeups(const FeedbackIn &in) {
    if (in.wakeup_dtlb) {
      retry_mem_req(Client::DTLB);
    }
    if (in.wakeup_itlb) {
      retry_mem_req(Client::ITLB);
    }
    if (in.wakeup_walk) {
      retry_active_walk();
    }
  }

  void send_mem_req(Client client, uint32_t paddr) {
    auto &s = nxt_.mem_clients[client_idx(client)];
    if (ctx != nullptr) {
      if (client == Client::DTLB) {

      } else {

      }
    }
    s.resp_valid = false;
    if (s.req_pending || s.req_inflight) {
      if (ctx != nullptr) {
        if (client == Client::DTLB) {

        } else {

        }
      }
      return;
    }
    s.req_pending = true;
    s.req_addr = paddr;
  }

  void send_walk_req(Client client, const PtwWalkReq &req) {
    auto &s = nxt_.walk_clients[client_idx(client)];
    if (ctx != nullptr) {
      if (client == Client::DTLB) {

      } else {

      }
    }
    s.resp_valid = false;
    if (s.req_pending || s.req_inflight) {
      if (ctx != nullptr) {
        if (client == Client::DTLB) {

        } else {

        }
      }
      return;
    }
    s.req_pending = true;
    s.req = req;
  }

  void on_mem_read_granted(Client client) {
    auto &s = nxt_.mem_clients[client_idx(client)];
    s.req_pending = false;
    s.req_inflight = true;
    if (ctx != nullptr) {
      if (client == Client::DTLB) {

      } else {

      }
    }
  }

  void on_mem_resp_client(Client client, uint32_t data) {
    auto &s = nxt_.mem_clients[client_idx(client)];
    s.resp_valid = true;
    s.resp_data = data;
    s.req_inflight = false;
    if (ctx != nullptr) {
      if (client == Client::DTLB) {

      } else {

      }
    }
  }

  void on_walk_read_granted(size_t req_id) {
    if (nxt_.walk_state == WalkState::L1_REQ) {
      nxt_.walk_state = WalkState::L1_WAIT_RESP;
    } else if (nxt_.walk_state == WalkState::L2_REQ) {
      nxt_.walk_state = WalkState::L2_WAIT_RESP;
    }
    nxt_.walk_req_id_valid = true;
    nxt_.walk_req_id = req_id;
    if (ctx != nullptr) {
      if (nxt_.walk_owner == Client::DTLB) {

      } else {

      }
    }
  }

  WalkRespResult on_walk_mem_resp(size_t req_id, uint32_t pte) {
    const bool is_active_req =
        nxt_.walk_active && nxt_.walk_req_id_valid && (nxt_.walk_req_id == req_id);
    if (nxt_.walk_drop_resp_credit > 0 && !is_active_req) {
      nxt_.walk_drop_resp_credit--;
      return WalkRespResult::DROPPED;
    }
    if (!nxt_.walk_active || !nxt_.walk_req_id_valid || !is_active_req) {
      return WalkRespResult::IGNORED;
    }

    auto &wc = nxt_.walk_clients[client_idx(nxt_.walk_owner)];
    const bool v = (pte & PTE_V) != 0;
    const bool r = (pte & PTE_R) != 0;
    const bool w = (pte & PTE_W) != 0;
    const bool x = (pte & PTE_X) != 0;

    auto publish_fault = [&]() {
      wc.req_inflight = false;
      wc.resp_valid = true;
      wc.resp = {};
      wc.resp.fault = true;
      wc.resp.vaddr = wc.req.vaddr;
      nxt_.walk_active = false;
      nxt_.walk_state = WalkState::IDLE;
      nxt_.walk_req_id_valid = false;
      nxt_.walk_req_id = 0;
    };
    auto publish_leaf = [&](uint8_t leaf_level) {
      wc.req_inflight = false;
      wc.resp_valid = true;
      wc.resp = {};
      wc.resp.fault = false;
      wc.resp.vaddr = wc.req.vaddr;
      wc.resp.leaf_pte = pte;
      wc.resp.leaf_level = leaf_level;
      nxt_.walk_active = false;
      nxt_.walk_state = WalkState::IDLE;
      nxt_.walk_req_id_valid = false;
      nxt_.walk_req_id = 0;
    };

    if (nxt_.walk_state == WalkState::L1_WAIT_RESP) {
      if (!v || (!r && w)) {
        publish_fault();
      } else if (r || x) {
        if (((pte >> 10) & 0x3FFu) != 0) {
          publish_fault();
        } else {
          publish_leaf(1);
        }
      } else {
        nxt_.walk_l1_pte = pte;
        nxt_.walk_state = WalkState::L2_REQ;
        nxt_.walk_req_id_valid = false;
        nxt_.walk_req_id = 0;
      }
    } else if (nxt_.walk_state == WalkState::L2_WAIT_RESP) {
      if (!v || (!r && w) || !(r || x)) {
        publish_fault();
      } else {
        publish_leaf(0);
      }
    } else {
      publish_fault();
    }

    if (ctx != nullptr && wc.resp_valid) {
      if (nxt_.walk_owner == Client::DTLB) {

      } else {

      }
    }
    return WalkRespResult::HANDLED;
  }

  WalkRespResult on_walk_mem_replay(size_t req_id, uint8_t replay_reason) {
    const bool is_active_req =
        nxt_.walk_active && nxt_.walk_req_id_valid && (nxt_.walk_req_id == req_id);
    if (nxt_.walk_drop_resp_credit > 0 && !is_active_req) {
      nxt_.walk_drop_resp_credit--;
      return WalkRespResult::DROPPED;
    }
    if (!nxt_.walk_active || !is_active_req) {
      return WalkRespResult::IGNORED;
    }

    auto &wc = nxt_.walk_clients[client_idx(nxt_.walk_owner)];
    wc.req_inflight = false;

    static constexpr uint8_t kReplayWaitFill = 2;
    if (replay_reason == kReplayWaitFill) {
      if (nxt_.walk_state == WalkState::L1_WAIT_RESP) {
        nxt_.walk_state = WalkState::L1_REQ;
      } else if (nxt_.walk_state == WalkState::L2_WAIT_RESP) {
        nxt_.walk_state = WalkState::L2_REQ;
      }
      nxt_.walk_req_id_valid = false;
      nxt_.walk_req_id = 0;
      return WalkRespResult::HANDLED;
    }

    nxt_.walk_req_id_valid = false;
    nxt_.walk_req_id = 0;
    return WalkRespResult::HANDLED;
  }

  void retry_mem_req(Client client) {
    auto &s = nxt_.mem_clients[client_idx(client)];
    if (!s.req_inflight) {
      return;
    }
    s.req_inflight = false;
    s.req_pending = true;
  }

  void retry_active_walk() {
    if (!nxt_.walk_active) {
      return;
    }
    if (nxt_.walk_state == WalkState::L1_WAIT_RESP) {
      nxt_.walk_state = WalkState::L1_REQ;
    } else if (nxt_.walk_state == WalkState::L2_WAIT_RESP) {
      nxt_.walk_state = WalkState::L2_REQ;
    }
    nxt_.walk_req_id_valid = false;
    nxt_.walk_req_id = 0;
  }

  static void flush_walk_client_state(State &state, Client client) {
    auto &s = state.walk_clients[client_idx(client)];
    s.req_pending = false;
    s.req_inflight = false;
    s.resp_valid = false;

    if (state.walk_active && state.walk_owner == client) {
      if (state.walk_state == WalkState::L1_WAIT_RESP ||
          state.walk_state == WalkState::L2_WAIT_RESP) {
        state.walk_drop_resp_credit++;
      }
      state.walk_active = false;
      state.walk_state = WalkState::IDLE;
      state.walk_l1_pte = 0;
      state.walk_req_id_valid = false;
      state.walk_req_id = 0;
    }
  }

  SimContext *ctx = nullptr;
  State cur_{};
  State nxt_{};
  CombOut comb_{};
};
