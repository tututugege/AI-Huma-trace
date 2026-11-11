#pragma once

#include "IO.h"
#include <type_traits>
#include "PtwWalker.h"
#include "PtwWalkPort.h"
#include "config.h"
#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>

class SimContext;
class PtwMemPort;

// register-width virtual address / satp (Sv32 on rv32, Sv39 on rv64)
using vaddr_t = std::conditional_t<(XLEN == 64), uint64_t, uint32_t>;

class TlbMmu {
public:
  enum class Kind : uint8_t {
    DTLB,
    ITLB,
  };

  enum class Result : uint8_t {
    OK,
    FAULT,
    RETRY,
  };

  enum class RetryReason : uint8_t {
    NONE = 0,
    OTHER_WALK_ACTIVE = 1,
    WALK_REQ_BLOCKED = 2,
    WAIT_WALK_RESP = 3,
    LOCAL_WALKER_BUSY = 4,
  };

  enum class TraceSource : uint8_t {
    NONE = 0,
    IDENTITY,
    TLB_HIT,
    TLB_PERM_FAULT,
    WALK_SHARED,
    WALK_LOCAL,
    FLUSH,
  };

  static constexpr int kItlbTopPortCount = 1;
  static constexpr int kDtlbTopPortCount = LSU_LDU_COUNT + LSU_STA_COUNT;
  static constexpr int kMaxTopPortCount =
      (kDtlbTopPortCount > kItlbTopPortCount) ? kDtlbTopPortCount
                                              : kItlbTopPortCount;

  struct TranslateContext {
    wire<XLEN> satp = 0;
    wire<2> eff_priv = 0;
    wire<1> mxr = false;
    wire<1> sum = false;
  };

  struct TlbEntry {
    reg<1> valid = false;
    reg<1> global = false;
    reg<16> asid = 0; // Sv32 satp ASID is 9 bits (satp[30:22]), Sv39 16 bits (satp[59:44])
    reg<2> level = 0; // leaf level: Sv32 1 = megapage / 0 = page; Sv39 2 = 1 GiB, 1 = 2 MiB, 0 = 4 KiB
    reg<10> vpn2 = 0; // Sv39 only
    reg<10> vpn1 = 0;
    reg<10> vpn0 = 0;
    reg<XLEN == 64 ? 44 : 22> ppn = 0; // Sv32 PPN is 22 bits, Sv39 44 bits
    reg<8> perm = 0; // bit[0:7]=V,R,W,X,U,G,A,D
  };

  struct WalkRegs {
    reg<1> active = false;
    reg<1> req_sent = false;
    reg<XLEN> v_addr = 0;
    reg<2> type = 0;
    reg<XLEN> satp = 0;
    reg<2> eff_priv = 0;
    reg<1> mxr = false;
    reg<1> sum = false;
  };

  static constexpr int kMaxTlbEntries =
      (ITLB_ENTRIES > DTLB_ENTRIES) ? ITLB_ENTRIES : DTLB_ENTRIES;
  static constexpr int kMaxSlotBits =
      kMaxTlbEntries <= 1 ? 1 : clog2(static_cast<uint64_t>(kMaxTlbEntries));

  struct PendingRefill {
    reg<1> valid = false;
    TlbEntry entry{};
    reg<kMaxSlotBits> slot = 0;
    reg<kMaxSlotBits> next_repl_ptr = 0;
  };

  struct LookupInput {
    wire<1> valid = false;
    wire<XLEN> v_addr = 0;
    wire<16> asid = 0;
  };

  struct LookupOutput {
    wire<1> hit = false;
    TlbEntry entry{};
  };

  struct CoreInput {
    wire<1> translate_valid = false;
    wire<XLEN> v_addr = 0;
    wire<2> type = 0;
    wire<XLEN> satp = 0;
    wire<2> privilege = 0;
    wire<1> mstatus_mxr = false;
    wire<1> mstatus_sum = false;
    wire<1> mstatus_mprv = false;
    wire<2> mstatus_mpp = 0;

    LookupOutput lookup{};
    WalkRegs walk{};
    PendingRefill refill{};
    int repl_ptr = 0;
    int tlb_capacity = 1;

    wire<1> walk_req_ready = false;
    wire<1> walk_resp_valid = false;
    PtwWalkResp walk_resp{};
  };

  struct CoreOutput {
    wire<1> resp_valid = false;
    wire<2> result = 0;
    wire<32> p_addr = 0;
    wire<3> retry_reason = 0;

    WalkRegs walk_next{};
    PendingRefill refill_next{};

    wire<1> walk_req_valid = false;
    PtwWalkReq walk_req{};
    wire<1> walk_resp_consumed = false;

    wire<3> trace_source = 0;
    wire<8> trace_level = 0xff;
    wire<8> trace_perm = 0;
  };

  struct TopPortInput {
    wire<1> valid = false;
    wire<XLEN> v_addr = 0;
    wire<2> type = 0;
    LookupOutput lookup{};
  };

  struct TopPortOutput {
    wire<1> resp_valid = false;
    wire<2> result = 0;
    wire<32> p_addr = 0;
    wire<3> retry_reason = 0;
    wire<3> trace_source = 0;
    wire<8> trace_level = 0xff;
    wire<8> trace_perm = 0;
  };

  struct TopInput {
    wire<1> flush = false;
    wire<1> cancel = false;
    wire<XLEN> satp = 0;
    wire<2> privilege = 0;
    wire<1> mstatus_mxr = false;
    wire<1> mstatus_sum = false;
    wire<1> mstatus_mprv = false;
    wire<2> mstatus_mpp = 0;

    WalkRegs walk{};
    PendingRefill refill{};
    int repl_ptr = 0;
    int tlb_capacity = 1;

    wire<1> walk_req_ready = false;
    wire<1> walk_resp_valid = false;
    PtwWalkResp walk_resp{};

    std::array<TopPortInput, kMaxTopPortCount> port{};
    int port_count = 0;
  };

  struct TopOutput {
    std::array<TopPortOutput, kMaxTopPortCount> port{};

    wire<1> table_flush = false;
    WalkRegs walk_next{};
    PendingRefill refill_next{};

    wire<1> walk_req_valid = false;
    PtwWalkReq walk_req{};
    wire<1> walk_resp_consumed = false;
    wire<1> walk_client_flush = false;
  };

  TlbMmu(SimContext *ctx, PtwMemPort *port = nullptr,
         int tlb_entries = DTLB_ENTRIES, Kind kind = Kind::DTLB);

  Result translate(uint32_t &p_addr, vaddr_t v_addr, uint32_t type,
                   CsrStatusIO *status);
  void translate_lsu_ports(const LsuMMUIO &req, MMULsuIO &resp);
  // FE_IPF: translation without any side effect (no walk, no TLB update); false when the TLB cannot translate it right now
  bool peek_translate(uint32_t &p_addr, vaddr_t v_addr, uint32_t type, CsrStatusIO *status) const;
  void flush();
  void seq();
  void cancel_pending_walk();
  // Fast-warmup replay (EXP-011): load a ready-made TLB state (entries in slot order, round-robin pointer).
  void warm_install(const TlbEntry *entries, int count, int repl_ptr);
  void set_ptw_mem_port(PtwMemPort *port);
  void set_ptw_walk_port(PtwWalkPort *port);
  RetryReason last_retry_reason() const { return last_retry_reason_; }

  static LookupOutput lookup_table_comb(const LookupInput &in,
                                        const TlbEntry *entries,
                                        int entry_count);
  static CoreOutput core_comb(const CoreInput &in);
  static TopOutput top_comb(const TopInput &in, int entries);

private:

  SimContext *ctx;
  Kind kind_;
  PtwWalkPort *walk_port = nullptr;
  PtwWalker walker;
  // Stable TLB register file. translate() only reads from this array; any
  // refill first lands in refill_comb_ and is committed in seq().
  std::vector<TlbEntry> tlb_entries_;
  int tlb_capacity_;
  int repl_ptr_;
  RetryReason last_retry_reason_ = RetryReason::NONE;

  WalkRegs walk_regs_{};
  // Per-cycle comb shadow for the in-flight miss record. Multiple translate()
  // calls in the same cycle share and update this view before seq() commits it.
  WalkRegs walk_comb_{};
  bool walk_comb_valid_ = false;
  // Same-cycle refill bypass. This keeps a completed walk visible to later
  // translate() calls in the same cycle without making the TLB regfile itself
  // look combinationally writable.
  PendingRefill refill_comb_{};
  bool flush_pending_ = false;

  const WalkRegs &visible_walk_regs() const;
  WalkRegs &ensure_walk_comb();
  void clear_tlb_entries();
  static TranslateContext build_translate_context(uint32_t type,
                                                  CsrStatusIO *status);
  static TranslateContext build_translate_context(uint32_t type, vaddr_t satp,
                                                  uint32_t privilege,
                                                  bool mstatus_mxr,
                                                  bool mstatus_sum,
                                                  bool mstatus_mprv,
                                                  uint32_t mstatus_mpp);
  static bool entry_matches(const TlbEntry &entry, vaddr_t v_addr,
                            uint16_t asid);
  bool lookup(vaddr_t v_addr, uint16_t asid, TlbEntry &hit) const;
  LookupOutput lookup_table_access(vaddr_t v_addr, uint16_t asid) const;
  static uint32_t compose_paddr(vaddr_t v_addr, const TlbEntry &e);
  static bool check_perm(uint8_t perm, uint32_t type, int eff_priv, bool sum,
                         bool mxr);
  Result translate_shared_via_io_generator(uint32_t &p_addr, vaddr_t v_addr,
                                           uint32_t type,
                                           CsrStatusIO *status);
  TopInput build_top_common_input(const CsrStatusIO *status) const;
  TopOutput run_top_via_io_generator(const TopInput &in) const;
  void apply_top_output(const TopOutput &out);
  void run_control_via_top(bool flush, bool cancel);
  bool consume_completed_walk_refill(bool consume_fault);
  Result walk_and_refill_shared(uint32_t &p_addr, vaddr_t v_addr,
                                uint32_t type, CsrStatusIO *status);
  Result walk_and_refill(uint32_t &p_addr, vaddr_t v_addr, uint32_t type,
                         CsrStatusIO *status);
  void schedule_refill(vaddr_t v_addr, uint16_t asid, uint8_t level,
                       uint64_t pte, bool global);
};
