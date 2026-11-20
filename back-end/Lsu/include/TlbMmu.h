#pragma once

#include "IO.h"
#include "PtwWalkPort.h"
#include "config.h"
#include <cstdint>
#include <vector>

using vaddr_t = uint64_t;   // a virtual address (Sv39: 39 bits, sign-extended)

// A TLB: the LSU's DTLB (LSU_LDU_COUNT + LSU_STA_COUNT ports) or the I-cache's ITLB (one port). Fully associative,
// round-robin replacement, one page-table walk in flight through its walk port to the shared walker. comb() serves one
// cycle: a flush (the table is cleared at the edge), a cancel of the walk, then the requests in port order (a hit, a
// fault, or a miss that starts / waits for the walk; the walk's refill is visible to the later requests of the cycle);
// seq() writes the refill / the flush at the clock edge.
class TlbMmu {
public:
  enum class Kind : uint8_t { DTLB, ITLB };
  enum class Result : uint8_t { OK, FAULT, RETRY };   // RETRY: a miss (try again; the walk is under way)
  struct Req {
    bool valid = false;
    vaddr_t vaddr = 0;
    uint8_t type = 0;   // 0 fetch, 1 load, 2 store
  };
  struct Resp {
    bool valid = false;   // the request was served this cycle (not on a flush / cancel cycle)
    Result result = Result::RETRY;
    uint32_t paddr = 0;   // (OK only)
  };
  struct TlbEntry {
    reg<1> valid = false;
    reg<1> global = false;
    reg<16> asid = 0;   // Sv39 satp[59:44]
    reg<2> level = 0;   // leaf level: 2 = 1 GiB, 1 = 2 MiB, 0 = 4 KiB
    reg<10> vpn2 = 0;
    reg<10> vpn1 = 0;
    reg<10> vpn0 = 0;
    reg<44> ppn = 0;
    reg<8> perm = 0;    // V R W X U G A D
  };

  // the walk port (bound by the owner): the walker's registered outputs to this TLB, this TLB's requests to it
  const PtwWalkPortCombOut *walk_resp = nullptr;
  PtwWalkPortCombIn *walk_req = nullptr;

  explicit TlbMmu(int entries = DTLB_ENTRIES, Kind kind = Kind::DTLB);
  void comb(bool flush, bool cancel, const Req *req, Resp *resp, int n);
  void seq();
  void reset();   // empty table, no walk (init; the walk port is not touched)
  // FE_IPF: translation without any side effect (no walk, no TLB update); false when the TLB cannot translate it now
  bool peek_translate(uint32_t &p_addr, vaddr_t v_addr, uint32_t type) const;
  bool walk_busy() const { return walk_.active; }   // EXP-048: a page-table walk is in flight (registered state)
  // EXP-048: the page of the prefetch translation sent this cycle; a walk for it that completes with a fault is dropped (nobody
  // asks for that page again to consume it)
  void set_pf_page(vaddr_t v_addr) { pf_page_ = v_addr >> 12; }
  vaddr_t walk_page_next() const { const WalkRegs &w = walk(); return w.active ? (w.v_addr >> 12) : ~vaddr_t(0); }
  vaddr_t pf_page_ = ~vaddr_t(0);
  // Fast-warmup replay (EXP-011): load a ready-made TLB state (entries in slot order, round-robin pointer).
  void warm_install(const TlbEntry *entries, int count, int repl_ptr);

private:
  struct WalkRegs {   // the walk in flight, with the translation context of the request that started it
    bool active = false, req_sent = false;
    vaddr_t v_addr = 0;
    uint8_t type = 0;
    vaddr_t satp = 0;
    int eff_priv = 0;
    bool mxr = false, sum = false;
  };
  struct Refill {     // a completed walk's entry: visible to the lookups of this cycle, written at the edge
    bool valid = false;
    TlbEntry entry{};
    int slot = 0, next_repl = 0;
  };
  struct Ctx { vaddr_t satp = 0; int eff_priv = 0; bool mxr = false, sum = false; };

  Kind kind_;
  // registers
  std::vector<TlbEntry> entries_;
  int capacity_;
  int repl_ptr_ = 0;
  // TLB_REPL=nru: one used bit per entry; plru: a binary tree over the entries (capacity-1 nodes). Updated at the edge
  // from this cycle's table hits (touch_) and the refill.
  std::vector<uint8_t> used_, plru_;
  std::vector<int> touch_;
  void repl_touch(int slot);
  int repl_victim() const;
  WalkRegs walk_{};
  // this cycle's wires (comb -> seq): the next walk state, the refill, the flush; the walk port as this cycle's
  // requests / consumes / flushes leave it (a later step of the cycle sees it so)
  WalkRegs walk_next_{};
  bool walk_next_valid_ = false;
  Refill refill_{};
  bool flush_ = false;
  PtwWalkPortCombOut port_{};
  bool port_valid_ = false;

  static Ctx context(uint32_t type);
  static bool entry_matches(const TlbEntry &e, vaddr_t v_addr, uint16_t asid);
  static uint32_t compose_paddr(vaddr_t v_addr, const TlbEntry &e);
  static bool check_perm(uint8_t perm, uint32_t type, int eff_priv, bool sum, bool mxr);
  const TlbEntry *lookup(vaddr_t v_addr, uint16_t asid) const;   // the table only
  Refill make_refill(vaddr_t v_addr, uint16_t asid, const PtwWalkResp &wr) const;
  const WalkRegs &walk() const { return walk_next_valid_ ? walk_next_ : walk_; }
  PtwWalkPortCombOut &port();
  void send(const PtwWalkReq &r);
  void consume();
  void drop_walk();
  bool take_completed_walk(bool consume_fault);
  Resp translate_one(const Req &r, const Ctx &c, WalkRegs &walk);
};
