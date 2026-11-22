#pragma once

#include "MemRouteBlock.h"
#include "PtwWalkPort.h"
#include "ref.h"
#include "types.h"
#include <array>
#include <cstddef>
#include <cstdio>

class SimContext;

// Shared PTW submodule. Port-side requests are latched in comb_begin(), memory
// feedback is applied in comb_finish(), and seq() is the only state commit.
//   CONTRACT comb_request  in: in.dtlb_walk in.itlb_walk  out: out.ptw_walk_req
//   CONTRACT comb_feedback  in: in.dtlb_walk in.itlb_walk in.ptw_events in.ptw_grant in.wakeup  out: out.dtlb_walk out.itlb_walk
class MemPtwBlock {
public:
  struct In {
    const PtwWalkPortCombIn *dtlb_walk = nullptr, *itlb_walk = nullptr;   // the TLBs' walk requests / flushes of the cycle
    const PtwGrant *ptw_grant = nullptr;     // the route block's grant / response / wake-up
    const PtwEvent *ptw_events = nullptr;
    const ReplayWakeup *wakeup = nullptr;
  } in;
  struct Out {
    PtwReq *ptw_walk_req = nullptr;                                       // the walker's memory read request
    PtwWalkPortCombIn *dtlb_walk = nullptr, *itlb_walk = nullptr;          // consumed at the end of the cycle
  } out;
  void comb_request();    // the TLBs' walk requests in, the walker's memory read request out
  void comb_feedback();   // the route block's grant / response back; the TLBs' requests of this cycle consumed

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

  using WalkClientIn = PtwWalkPortCombIn;    // a TLB's requests (PtwWalkPort.h)
  using WalkClientOut = PtwWalkPortCombOut;  // the walker's outputs to a TLB

  struct PortIn {
    std::array<WalkClientIn, static_cast<size_t>(Client::NUM_CLIENTS)>
        walk_clients{};
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
    // a TLB dropped its walk this cycle (flush / cancel): the client's walk state is cleared at the edge
    std::array<bool, static_cast<size_t>(Client::NUM_CLIENTS)> walk_client_flush{};
    bool sfence_flush = false;
  };

  struct CombOut {
    std::array<WalkClientOut, static_cast<size_t>(Client::NUM_CLIENTS)>
        walk_clients{};
    bool issue_walk_read = false;
    uint32_t walk_read_addr = 0;
  };

  MemPtwBlock() { init(); }


  void init() {
    reset_state(q_);
    reset_state(d);
    reset_l2tlb();
    reset_outputs(comb_);
    refresh_outputs();
    latch_client_outputs();
  }

  void comb_begin(const PortIn &in) {
    d = q_;
    d.l2_op = {};
    reset_outputs(comb_);
    apply_port_inputs(in);
    select_walk_owner();
    refresh_outputs();
  }

  void comb_finish(const FeedbackIn &in) {
    apply_stlb_lookup();
    apply_grant(in);
    apply_events(in);
    apply_wakeups(in);
    refresh_outputs();
    // (after the outputs: the flush shows only from the next cycle on)
    for (size_t i = 0; i < kClientCount; i++)
      if (in.walk_client_flush[i]) flush_walk_client_state(d, static_cast<Client>(i));
    if (in.sfence_flush) d.l2_op.flush_all = true;
  }

  void seq() {
    apply_l2tlb_seq(d.l2_op);
    d.l2_op = {};
    q_ = d;
    refresh_outputs();
    latch_client_outputs();
  }
  // the outputs to the TLBs as of the last clock edge (each TLB's walk port reads its own)
  PtwWalkPortCombOut dtlb_q{}, itlb_q{};
  void latch_client_outputs() {
    dtlb_q = comb_.walk_clients[static_cast<size_t>(Client::DTLB)];
    itlb_q = comb_.walk_clients[static_cast<size_t>(Client::ITLB)];
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
    STLB_LK,
  };

  static constexpr size_t kClientCount =
      static_cast<size_t>(Client::NUM_CLIENTS);

  static constexpr size_t kStlbWays   = L2TLB_WAYS;
  static constexpr size_t kStlbSets   = L2TLB_ENTRIES / L2TLB_WAYS;
  static constexpr size_t kSpEntries  = L2TLB_SP_ENTRIES;
  static constexpr size_t kPwcEntries = L2TLB_PWC_ENTRIES;

  struct StlbEntry {
    bool valid = false;
    bool global = false;
    uint32_t root = 0;   // satp.ppn[19:0]
    uint32_t vpn = 0;    // vaddr[38:12] (27b)
    uint32_t ppn = 0;    // pte[29:10] (20b)
    uint8_t perm = 0;    // pte[7:0]
  };

  struct SpEntry {
    bool valid = false;
    bool global = false;
    uint8_t level = 0;   // 1 (2MB) or 2 (1GB)
    uint32_t root = 0;   // 20b
    uint16_t vpn2 = 0;   // 9b
    uint16_t vpn1 = 0;   // 9b
    uint32_t ppn = 0;    // 20b
    uint8_t perm = 0;    // 8b
  };

  struct PwcEntry {
    bool valid = false;
    bool is_l1 = false;  // false = L2 (top level), true = L1 (mid level)
    uint32_t root = 0;   // 20b
    uint16_t vpn2 = 0;   // 9b
    uint16_t vpn1 = 0;   // 9b (0 for L2)
    uint32_t ppn = 0;    // 20b
  };

  struct L2TlbOp {
    bool flush_all = false;
    bool touch_plru = false;
    uint16_t touch_set = 0;
    uint8_t touch_way = 0;
    bool wr_stlb = false;
    bool wr_sp = false;
    bool wr_pwc = false;
    bool wr_global = false;
    uint8_t wr_level = 0;
    uint32_t wr_root = 0;
    uint32_t wr_vpn = 0;
    uint32_t wr_ppn = 0;
    uint8_t wr_perm = 0;
  };

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
    L2TlbOp l2_op{};
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
      auto &walk_state = d.walk_clients[i];
      if (in.walk_clients[i].resp_consumed) {
        walk_state.resp_valid = false;
      }
      if (in.walk_clients[i].req_valid) {
        send_walk_req(client, in.walk_clients[i].req);
      }
    }
  }

  void select_walk_owner() {
    if (d.walk_active || d.walk_state != WalkState::IDLE) {
      return;
    }

    auto grant_walk_owner = [&](Client owner) {
      auto &wc = d.walk_clients[client_idx(owner)];
      wc.req_pending = false;
      wc.req_inflight = true;
      d.walk_owner = owner;
      d.walk_active = true;
      d.walk_state = WalkState::STLB_LK;
      d.walk_l1_pte = 0;
      d.walk_level = 2;   // Sv39: three levels
      d.walk_rr_next = (owner == Client::DTLB) ? Client::ITLB
                                                  : Client::DTLB;
    };

    const Client first = d.walk_rr_next;
    const Client second = (first == Client::DTLB) ? Client::ITLB
                                                  : Client::DTLB;
    if (d.walk_clients[client_idx(first)].req_pending) {
      grant_walk_owner(first);
    } else if (d.walk_clients[client_idx(second)].req_pending) {
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

  void refresh_outputs() { project_outputs_from_state(d, comb_); }

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
    auto &s = d.walk_clients[client_idx(client)];
    s.resp_valid = false;
    if (s.req_pending || s.req_inflight) {
      return;
    }
    s.req_pending = true;
    s.req = req;
  }

  void on_walk_read_granted(size_t req_id) {
    if (d.walk_state == WalkState::L1_REQ) {
      d.walk_state = WalkState::L1_WAIT_RESP;
    } else if (d.walk_state == WalkState::L2_REQ) {
      d.walk_state = WalkState::L2_WAIT_RESP;
    }
    d.walk_req_id_valid = true;
    d.walk_req_id = req_id;
  }

  WalkRespResult on_walk_mem_resp(size_t req_id, uint64_t pte) {
    const bool is_active_req =
        d.walk_active && d.walk_req_id_valid && (d.walk_req_id == req_id);
    if (d.walk_drop_resp_credit > 0 && !is_active_req) {
      d.walk_drop_resp_credit--;
      return WalkRespResult::DROPPED;
    }
    if (!d.walk_active || !d.walk_req_id_valid || !is_active_req) {
      return WalkRespResult::IGNORED;
    }

    auto &wc = d.walk_clients[client_idx(d.walk_owner)];
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
      d.walk_active = false;
      d.walk_state = WalkState::IDLE;
      d.walk_req_id_valid = false;
      d.walk_req_id = 0;
    };
    auto publish_leaf = [&](uint8_t leaf_level) {
      wc.req_inflight = false;
      wc.resp_valid = true;
      wc.resp = {};
      wc.resp.fault = false;
      wc.resp.vaddr = wc.req.vaddr;
      wc.resp.leaf_pte = pte;
      wc.resp.leaf_level = leaf_level;
      d.walk_active = false;
      d.walk_state = WalkState::IDLE;
      d.walk_req_id_valid = false;
      d.walk_req_id = 0;
    };

    if (d.walk_state == WalkState::L1_WAIT_RESP || d.walk_state == WalkState::L2_WAIT_RESP) {
      const uint8_t level = d.walk_level;
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
          const uint32_t own_vpn = static_cast<uint32_t>((wc.req.vaddr >> 12) & 0x7FFFFFFu);
          const uint32_t own_root = static_cast<uint32_t>(wc.req.satp & 0xFFFFFu);
          d.l2_op.wr_stlb = (level == 0);
          d.l2_op.wr_sp = (level > 0);
          d.l2_op.wr_global = (pte & PTE_G) != 0;
          d.l2_op.wr_level = level;
          d.l2_op.wr_root = own_root;
          d.l2_op.wr_vpn = own_vpn;
          d.l2_op.wr_ppn = static_cast<uint32_t>(ppn & 0xFFFFFu);
          d.l2_op.wr_perm = static_cast<uint8_t>(pte & 0xFFu);
          publish_leaf(level);
        }
      } else if (level == 0) {
        publish_fault();   // a pointer where a leaf is required
      } else {
        const uint32_t own_vpn = static_cast<uint32_t>((wc.req.vaddr >> 12) & 0x7FFFFFFu);
        const uint32_t own_root = static_cast<uint32_t>(wc.req.satp & 0xFFFFFu);
        d.l2_op.wr_pwc = true;
        d.l2_op.wr_level = level;
        d.l2_op.wr_root = own_root;
        d.l2_op.wr_vpn = own_vpn;
        d.l2_op.wr_ppn = static_cast<uint32_t>(ppn & 0xFFFFFu);
        d.walk_l1_pte = pte;
        d.walk_level = level - 1;
        d.walk_state = WalkState::L2_REQ;
        d.walk_req_id_valid = false;
        d.walk_req_id = 0;
      }
    } else {
      publish_fault();
    }

    return WalkRespResult::HANDLED;
  }

  WalkRespResult on_walk_mem_replay(size_t req_id, uint8_t replay_reason) {
    const bool is_active_req =
        d.walk_active && d.walk_req_id_valid && (d.walk_req_id == req_id);
    if (d.walk_drop_resp_credit > 0 && !is_active_req) {
      d.walk_drop_resp_credit--;
      return WalkRespResult::DROPPED;
    }
    if (!d.walk_active || !is_active_req) {
      return WalkRespResult::IGNORED;
    }

    auto &wc = d.walk_clients[client_idx(d.walk_owner)];
    wc.req_inflight = false;

    static constexpr uint8_t kReplayWaitFill = 2;
    if (replay_reason == kReplayWaitFill) {
      if (d.walk_state == WalkState::L1_WAIT_RESP) {
        d.walk_state = WalkState::L1_REQ;
      } else if (d.walk_state == WalkState::L2_WAIT_RESP) {
        d.walk_state = WalkState::L2_REQ;
      }
      d.walk_req_id_valid = false;
      d.walk_req_id = 0;
      return WalkRespResult::HANDLED;
    }

    d.walk_req_id_valid = false;
    d.walk_req_id = 0;
    return WalkRespResult::HANDLED;
  }

  void retry_active_walk() {
    if (!d.walk_active) {
      return;
    }
    if (d.walk_state == WalkState::L1_WAIT_RESP) {
      d.walk_state = WalkState::L1_REQ;
    } else if (d.walk_state == WalkState::L2_WAIT_RESP) {
      d.walk_state = WalkState::L2_REQ;
    }
    d.walk_req_id_valid = false;
    d.walk_req_id = 0;
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

  static uint16_t stlb_index(uint32_t vpn) {
    return static_cast<uint16_t>(((vpn & 0xFFu) ^ ((vpn >> 9) & 0xFFu)) & (kStlbSets - 1));
  }

  static uint8_t plru_update(uint8_t p, uint8_t way) {
    uint8_t n = p & 0x7u;
    const uint8_t w1 = (way >> 1) & 1u;
    const uint8_t w0 = way & 1u;
    n = static_cast<uint8_t>((n & ~1u) | (w1 ^ 1u));
    if (w1 == 0) n = static_cast<uint8_t>((n & ~2u) | ((w0 ^ 1u) << 1));
    else         n = static_cast<uint8_t>((n & ~4u) | ((w0 ^ 1u) << 2));
    return n;
  }

  static uint8_t plru_victim(uint8_t p) {
    const uint8_t root = p & 1u;
    const uint8_t leaf = root ? ((p >> 2) & 1u) : ((p >> 1) & 1u);
    return static_cast<uint8_t>((root << 1) | leaf);
  }

  void reset_l2tlb() {
    for (auto &set : stlb_) for (auto &e : set) e = {};
    stlb_plru_.fill(0);
    for (auto &e : sp_cam_) e = {};
    sp_rr_ = 0;
    for (auto &e : pwc_) e = {};
    pwc_rr_ = 0;
  }

  void apply_stlb_lookup() {
    if (q_.walk_state != WalkState::STLB_LK) return;
    auto &wc = d.walk_clients[client_idx(q_.walk_owner)];
    const uint32_t own_vpn = static_cast<uint32_t>((wc.req.vaddr >> 12) & 0x7FFFFFFu);
    const uint32_t own_root = static_cast<uint32_t>(wc.req.satp & 0xFFFFFu);
    const uint16_t vpn2 = static_cast<uint16_t>((own_vpn >> 18) & 0x1FFu);
    const uint16_t vpn1 = static_cast<uint16_t>((own_vpn >> 9) & 0x1FFu);
    const uint16_t idx = stlb_index(own_vpn);

    // 1. 4KB STLB lookup (lowest matching way)
    for (size_t w = 0; w < kStlbWays; ++w) {
      const auto &e = stlb_[idx][w];
      if (e.valid && e.vpn == own_vpn && (e.global || e.root == own_root)) {
        d.l2_op.touch_plru = true;
        d.l2_op.touch_set = idx;
        d.l2_op.touch_way = static_cast<uint8_t>(w);
        wc.req_inflight = false;
        wc.resp_valid = true;
        wc.resp = {};
        wc.resp.fault = false;
        wc.resp.vaddr = wc.req.vaddr;
        wc.resp.leaf_pte = (static_cast<uint64_t>(e.ppn) << 10) | static_cast<uint64_t>(e.perm);
        wc.resp.leaf_level = 0;
        d.walk_active = false;
        d.walk_state = WalkState::IDLE;
        d.walk_req_id_valid = false;
        d.walk_req_id = 0;
        return;
      }
    }

    // 2. Superpage CAM lookup (lowest matching entry)
    for (size_t i = 0; i < kSpEntries; ++i) {
      const auto &e = sp_cam_[i];
      if (e.valid && (e.global || e.root == own_root) && e.vpn2 == vpn2 &&
          (e.level == 2 || e.vpn1 == vpn1)) {
        wc.req_inflight = false;
        wc.resp_valid = true;
        wc.resp = {};
        wc.resp.fault = false;
        wc.resp.vaddr = wc.req.vaddr;
        wc.resp.leaf_pte = (static_cast<uint64_t>(e.ppn) << 10) | static_cast<uint64_t>(e.perm);
        wc.resp.leaf_level = e.level;
        d.walk_active = false;
        d.walk_state = WalkState::IDLE;
        d.walk_req_id_valid = false;
        d.walk_req_id = 0;
        return;
      }
    }

    // 3. PWC lookup: L1 (level 1 -> next level 0) first, then L2 (level 2 -> next level 1)
    for (size_t i = 0; i < kPwcEntries; ++i) {
      const auto &e = pwc_[i];
      if (e.valid && e.is_l1 && e.root == own_root && e.vpn2 == vpn2 && e.vpn1 == vpn1) {
        d.walk_l1_pte = static_cast<uint64_t>(e.ppn) << 10;
        d.walk_level = 0;
        d.walk_state = WalkState::L2_REQ;
        return;
      }
    }
    for (size_t i = 0; i < kPwcEntries; ++i) {
      const auto &e = pwc_[i];
      if (e.valid && !e.is_l1 && e.root == own_root && e.vpn2 == vpn2) {
        d.walk_l1_pte = static_cast<uint64_t>(e.ppn) << 10;
        d.walk_level = 1;
        d.walk_state = WalkState::L2_REQ;
        return;
      }
    }

    // 4. Full miss: start 3-level walk from L2
    d.walk_level = 2;
    d.walk_state = WalkState::L1_REQ;
  }

  void apply_l2tlb_seq(const L2TlbOp &op) {
    if (op.flush_all) {
      reset_l2tlb();
      return;
    }
    if (op.touch_plru) {
      stlb_plru_[op.touch_set] = plru_update(stlb_plru_[op.touch_set], op.touch_way);
    }
    if (op.wr_stlb) {
      const uint16_t idx = stlb_index(op.wr_vpn);
      size_t way = kStlbWays;
      for (size_t w = 0; w < kStlbWays; ++w) {
        if (!stlb_[idx][w].valid) { way = w; break; }
      }
      if (way == kStlbWays) way = plru_victim(stlb_plru_[idx]);
      auto &dst = stlb_[idx][way];
      dst.valid = true;
      dst.global = op.wr_global;
      dst.root = op.wr_root;
      dst.vpn = op.wr_vpn;
      dst.ppn = op.wr_ppn;
      dst.perm = op.wr_perm;
      stlb_plru_[idx] = plru_update(stlb_plru_[idx], static_cast<uint8_t>(way));
    }
    if (op.wr_sp) {
      const uint16_t vpn2 = static_cast<uint16_t>((op.wr_vpn >> 18) & 0x1FFu);
      const uint16_t vpn1 = static_cast<uint16_t>((op.wr_vpn >> 9) & 0x1FFu);
      size_t slot = kSpEntries;
      for (size_t i = 0; i < kSpEntries; ++i) {
        const auto &e = sp_cam_[i];
        if (e.valid && e.level == op.wr_level && (e.global || e.root == op.wr_root) &&
            e.vpn2 == vpn2 && (op.wr_level == 2 || e.vpn1 == vpn1)) {
          slot = i;
          break;
        }
      }
      if (slot == kSpEntries) {
        for (size_t i = 0; i < kSpEntries; ++i) {
          if (!sp_cam_[i].valid) { slot = i; break; }
        }
      }
      if (slot == kSpEntries) {
        slot = sp_rr_;
        sp_rr_ = static_cast<uint8_t>((sp_rr_ + 1) & (kSpEntries - 1));
      }
      auto &dst = sp_cam_[slot];
      dst.valid = true;
      dst.global = op.wr_global;
      dst.level = op.wr_level;
      dst.root = op.wr_root;
      dst.vpn2 = vpn2;
      dst.vpn1 = vpn1;
      dst.ppn = op.wr_ppn;
      dst.perm = op.wr_perm;
    }
    if (op.wr_pwc) {
      const bool is_l1 = (op.wr_level == 1);
      const uint16_t vpn2 = static_cast<uint16_t>((op.wr_vpn >> 18) & 0x1FFu);
      const uint16_t vpn1 = is_l1 ? static_cast<uint16_t>((op.wr_vpn >> 9) & 0x1FFu) : 0u;
      size_t slot = kPwcEntries;
      for (size_t i = 0; i < kPwcEntries; ++i) {
        const auto &e = pwc_[i];
        if (e.valid && e.is_l1 == is_l1 && e.root == op.wr_root && e.vpn2 == vpn2 &&
            (!is_l1 || e.vpn1 == vpn1)) {
          slot = i;
          break;
        }
      }
      if (slot == kPwcEntries) {
        for (size_t i = 0; i < kPwcEntries; ++i) {
          if (!pwc_[i].valid) { slot = i; break; }
        }
      }
      if (slot == kPwcEntries) {
        slot = pwc_rr_;
        pwc_rr_ = static_cast<uint8_t>((pwc_rr_ + 1) & (kPwcEntries - 1));
      }
      auto &dst = pwc_[slot];
      dst.valid = true;
      dst.is_l1 = is_l1;
      dst.root = op.wr_root;
      dst.vpn2 = vpn2;
      dst.vpn1 = vpn1;
      dst.ppn = op.wr_ppn;
    }
  }

  // registers: q (read in comb only by comb_begin, written only by seq / init) and d (comb writes it; comb_begin starts it from q)
  using Regs = State;
  Regs q_{};
  Regs d{};
  CombOut comb_{};
  std::array<std::array<StlbEntry, kStlbWays>, kStlbSets> stlb_{};
  std::array<uint8_t, kStlbSets> stlb_plru_{};
  std::array<SpEntry, kSpEntries> sp_cam_{};
  uint8_t sp_rr_ = 0;
  std::array<PwcEntry, kPwcEntries> pwc_{};
  uint8_t pwc_rr_ = 0;
};
