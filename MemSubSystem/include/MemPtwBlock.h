#pragma once

#include "PtwWalkPort.h"
#include "ref.h"
#include "types.h"
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
    std::array<WalkClientIn, static_cast<size_t>(Client::NUM_CLIENTS)>
        walk_clients{};
  };

  struct SeqIn {
    std::array<bool, static_cast<size_t>(Client::NUM_CLIENTS)>
        walk_client_flush{};
  };

  struct RoutedEvent {
    bool valid = false;   // a read of the page-table walker
    // Memory-side resolved 32-bit word presented to PTW for this event. The
    // surrounding memory subsystem may rewrite the raw routed response into the
    // PTW-visible value before filling FeedbackIn.
    uint64_t data = 0;   // a PTE: one word (Sv32) or two (Sv39)
    uint8_t replay = 0;
    uint32_t req_addr = 0;
    size_t req_id = 0;
  };

  struct FeedbackIn {
    bool grant_valid = false;   // the walker's read was sent to the D-cache
    size_t grant_req_id = 0;
    std::array<RoutedEvent, LSU_LDU_COUNT> events{};
    uint8_t event_count = 0;
    bool wakeup_walk = false;
  };

  struct CombOut {
    std::array<WalkClientOut, static_cast<size_t>(Client::NUM_CLIENTS)>
        walk_clients{};
    bool issue_walk_read = false;
    uint32_t walk_read_addr = 0;
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
    std::array<WalkClientState, kClientCount> walk_clients{};
    WalkState walk_state = WalkState::IDLE;
    bool walk_active = false;
    Client walk_owner = Client::DTLB;
    Client walk_rr_next = Client::DTLB;
    uint64_t walk_l1_pte = 0;   // the non-leaf PTE the next level is read from
    uint8_t walk_level = 0;     // level of the PTE read in flight (Sv32: 1, 0; Sv39: 2, 1, 0)
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

  void apply_port_inputs(const PortIn &in) {
    for (size_t i = 0; i < kClientCount; i++) {
      const Client client = static_cast<Client>(i);
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
      nxt_.walk_level = 2;   // Sv39: three levels
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
    // PTE address: table base + VPN[level] * PTE size (Sv32: 10-bit VPNs, 4-byte PTEs; Sv39: 9-bit VPNs, 8-byte PTEs).  The
    // physical address must fit this SoC's 32-bit space (an Sv39 PPN is 44 bits).
    auto pte_addr = [&](uint64_t ppn, uint32_t level) -> uint32_t {
      const uint32_t vpn_bits = 9, pte_shift = 3;
      const uint64_t vpn = (static_cast<uint64_t>(req.vaddr) >> (12 + vpn_bits * level)) & ((1ull << vpn_bits) - 1);
      const uint64_t a = (ppn << 12) + (vpn << pte_shift);
      if ((a >> 32) != 0) {
        std::fprintf(stderr, "[ptw] page-table address 0x%llx is beyond the 32-bit physical space\n", static_cast<unsigned long long>(a));
        std::exit(2);
      }
      return static_cast<uint32_t>(a);
    };
    if (state.walk_state == WalkState::L1_REQ) {
      const uint64_t root_ppn = (static_cast<uint64_t>(req.satp) & 0xFFFFFFFFFFFull);
      out.issue_walk_read = true;
      out.walk_read_addr = pte_addr(root_ppn, state.walk_level);
    } else if (state.walk_state == WalkState::L2_REQ) {
      const uint64_t ppn = ((state.walk_l1_pte >> 10) & 0xFFFFFFFFFFFull);
      out.issue_walk_read = true;
      out.walk_read_addr = pte_addr(ppn, state.walk_level);
    }
  }

  void refresh_outputs() { project_outputs_from_state(nxt_, comb_); }

  void apply_grant(const FeedbackIn &in) {
    if (in.grant_valid) on_walk_read_granted(in.grant_req_id);
  }

  void apply_events(const FeedbackIn &in) {
    for (uint8_t i = 0; i < in.event_count; i++) {
      const auto &evt = in.events[i];
      if (!evt.valid) {
        continue;
      }
      if (evt.replay == 0) (void)on_walk_mem_resp(evt.req_id, evt.data);
      else (void)on_walk_mem_replay(evt.req_id, evt.replay);
    }
  }

  void apply_wakeups(const FeedbackIn &in) {
    if (in.wakeup_walk) {
      retry_active_walk();
    }
  }

  void send_walk_req(Client client, const PtwWalkReq &req) {
    auto &s = nxt_.walk_clients[client_idx(client)];
    s.resp_valid = false;
    if (s.req_pending || s.req_inflight) {
      return;
    }
    s.req_pending = true;
    s.req = req;
  }

  void on_walk_read_granted(size_t req_id) {
    if (nxt_.walk_state == WalkState::L1_REQ) {
      nxt_.walk_state = WalkState::L1_WAIT_RESP;
    } else if (nxt_.walk_state == WalkState::L2_REQ) {
      nxt_.walk_state = WalkState::L2_WAIT_RESP;
    }
    nxt_.walk_req_id_valid = true;
    nxt_.walk_req_id = req_id;
  }

  WalkRespResult on_walk_mem_resp(size_t req_id, uint64_t pte) {
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

    if (nxt_.walk_state == WalkState::L1_WAIT_RESP || nxt_.walk_state == WalkState::L2_WAIT_RESP) {
      const uint8_t level = nxt_.walk_level;
      // Sv39: PTE bits 63:54 are reserved (Svnapot / Svpbmt are not implemented): a PTE with any of them set faults
      const bool reserved_bits = (pte >> 54) != 0;
      // the PPN bits below a superpage's level must be zero (Sv32: PPN[0] = pte[19:10]; Sv39: PPN[level-1:0])
      const uint64_t ppn = (pte >> 10) & 0xFFFFFFFFFFFull;
      const uint32_t vpn_bits = 9;
      const bool misaligned = level > 0 && (ppn & ((1ull << (vpn_bits * level)) - 1)) != 0;
      if (!v || (!r && w) || reserved_bits) {
        publish_fault();
      } else if (r || x) {
        if (misaligned) {
          publish_fault();
        } else {
          publish_leaf(level);
        }
      } else if (level == 0) {
        publish_fault();   // a pointer where a leaf is required
      } else {
        nxt_.walk_l1_pte = pte;
        nxt_.walk_level = level - 1;
        nxt_.walk_state = WalkState::L2_REQ;
        nxt_.walk_req_id_valid = false;
        nxt_.walk_req_id = 0;
      }
    } else {
      publish_fault();
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
      state.walk_level = 0;
      state.walk_req_id_valid = false;
      state.walk_req_id = 0;
    }
  }

  SimContext *ctx = nullptr;
  State cur_{};
  State nxt_{};
  CombOut comb_{};
};
