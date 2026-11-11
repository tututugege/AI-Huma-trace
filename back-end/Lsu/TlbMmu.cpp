#include "TlbMmu.h"
#include "PtwMemPort.h"
#include "ref.h"

namespace {
#ifndef CONFIG_ITLB_FOCUS_VADDR_BEGIN
#define CONFIG_ITLB_FOCUS_VADDR_BEGIN 0u
#endif

#ifndef CONFIG_ITLB_FOCUS_VADDR_END
#define CONFIG_ITLB_FOCUS_VADDR_END 0u
#endif


// Sv32 (rv32): 2 levels of 10-bit VPNs, 9-bit ASID in satp[30:22]; Sv39 (rv64): 3 levels of 9-bit VPNs, 16-bit ASID in satp[59:44]
constexpr int kLevels = XLEN == 64 ? 3 : 2;
constexpr int kVpnBits = XLEN == 64 ? 9 : 10;

inline uint16_t sv_asid(vaddr_t satp) {
  if (XLEN == 64) return static_cast<uint16_t>((satp >> 44) & 0xFFFFu);
  return static_cast<uint16_t>((satp >> 22) & 0x1FFu);
}
// translation enabled in satp (Sv32: MODE bit 31; Sv39: MODE = 8)
inline bool sv_enabled(vaddr_t satp) {
  return XLEN == 64 ? (satp >> 60) == 8 : (satp & 0x80000000u) != 0;
}
// Sv39 virtual addresses must be sign-extended from bit 38 (always true on Sv32)
inline bool va_canonical(vaddr_t v) {
  if (XLEN != 64) return true;
  const int64_t top = static_cast<int64_t>(v) >> 38;
  return top == 0 || top == -1;
}
constexpr uint64_t kPpnMask = XLEN == 64 ? 0xFFFFFFFFFFFull : 0x3FFFFFull;
// untranslated address as a physical one; above the 32-bit physical space it maps to an address outside RAM
inline uint32_t identity_pa(vaddr_t v) {
  return (XLEN == 64 && (static_cast<uint64_t>(v) >> 32) != 0) ? 0x7fffffffu : static_cast<uint32_t>(v);
}
inline uint32_t vpn_at(vaddr_t v, int level) {
  return static_cast<uint32_t>((v >> (12 + kVpnBits * level)) & ((1u << kVpnBits) - 1));
}


MMUResultType to_lsu_mmu_result(TlbMmu::Result result) {
  switch (result) {
  case TlbMmu::Result::OK:
    return MMUResultType::HIT;
  case TlbMmu::Result::FAULT:
    return MMUResultType::PAGE_FAULT;
  case TlbMmu::Result::RETRY:
  default:
    return MMUResultType::MISS;
  }
}
} // namespace

TlbMmu::TlbMmu(SimContext *ctx, PtwMemPort *port, int tlb_entries, Kind kind)
    : ctx(ctx), kind_(kind), walker(port),
      tlb_entries_(tlb_entries > 0 ? tlb_entries : 1),
      tlb_capacity_(tlb_entries > 0 ? tlb_entries : 1), repl_ptr_(0) {
  (void)this->ctx;
  flush_pending_ = true;
  seq();
}

void TlbMmu::set_ptw_mem_port(PtwMemPort *port) { walker.set_mem_port(port); }
void TlbMmu::set_ptw_walk_port(PtwWalkPort *port) { walk_port = port; }

void TlbMmu::warm_install(const TlbEntry *entries, int count, int repl_ptr) {
  const int n = count < tlb_capacity_ ? count : tlb_capacity_;
  for (int i = 0; i < n; ++i) tlb_entries_[i] = entries[i];
  repl_ptr_ = tlb_capacity_ > 0 ? repl_ptr % tlb_capacity_ : 0;
}

void TlbMmu::cancel_pending_walk() {
  consume_completed_walk_refill(true);
  walker.flush();
  run_control_via_top(false, true);
  last_retry_reason_ = RetryReason::NONE;
}

void TlbMmu::flush() {
  const WalkRegs &walk = visible_walk_regs();
  if (walk.active) {

  }
  walker.flush();
  run_control_via_top(true, false);
  last_retry_reason_ = RetryReason::NONE;
}


void TlbMmu::seq() {
  if (flush_pending_) {
    clear_tlb_entries();
    repl_ptr_ = 0;
    walk_regs_ = {};
    walk_comb_ = {};
    walk_comb_valid_ = false;
    refill_comb_ = {};
    flush_pending_ = false;
    return;
  }

  if (refill_comb_.valid) {
    tlb_entries_[refill_comb_.slot] = refill_comb_.entry;
    repl_ptr_ = refill_comb_.next_repl_ptr;
    refill_comb_ = {};
  }

  if (walk_comb_valid_) {
    walk_regs_ = walk_comb_;
    walk_comb_valid_ = false;
  }
}


const TlbMmu::WalkRegs &TlbMmu::visible_walk_regs() const {
  return walk_comb_valid_ ? walk_comb_ : walk_regs_;
}

TlbMmu::WalkRegs &TlbMmu::ensure_walk_comb() {
  if (!walk_comb_valid_) {
    walk_comb_ = walk_regs_;
    walk_comb_valid_ = true;
  }
  return walk_comb_;
}



void TlbMmu::clear_tlb_entries() {
  for (auto &entry : tlb_entries_) {
    entry = {};
    entry.valid = false;
  }
}

TlbMmu::TranslateContext TlbMmu::build_translate_context(uint32_t type,
                                                         CsrStatusIO *status) {
  if (status == nullptr) {
    return {};
  }
  return build_translate_context(
      type, static_cast<vaddr_t>(status->satp),
      static_cast<uint32_t>(status->privilege),
      (static_cast<uint32_t>(status->mstatus) & MSTATUS_MXR) != 0,
      (static_cast<uint32_t>(status->mstatus) & MSTATUS_SUM) != 0,
      (static_cast<uint32_t>(status->mstatus) & MSTATUS_MPRV) != 0,
      (static_cast<uint32_t>(status->mstatus) >> MSTATUS_MPP_SHIFT) & 0x3u);   // (status bits below 32 only)
}

TlbMmu::TranslateContext
TlbMmu::build_translate_context(uint32_t type, vaddr_t satp,
                                uint32_t privilege, bool mstatus_mxr,
                                bool mstatus_sum, bool mstatus_mprv,
                                uint32_t mstatus_mpp) {
  TranslateContext ctx{};
  ctx.satp = satp;
  ctx.eff_priv = static_cast<int>(privilege & 0x3u);
  ctx.mxr = mstatus_mxr;
  ctx.sum = mstatus_sum;
  if (type != 0 && mstatus_mprv) {
    ctx.eff_priv = static_cast<int>(mstatus_mpp & 0x3u);
  }
  return ctx;
}

bool TlbMmu::entry_matches(const TlbEntry &entry, vaddr_t v_addr,
                           uint16_t asid) {
  if (!entry.valid) {
    return false;
  }
  if (!(entry.global || entry.asid == asid)) {
    return false;
  }
  // every VPN above the leaf's level must match (a superpage ignores the lower ones)
  const uint32_t stored[3] = {entry.vpn0, entry.vpn1, entry.vpn2};
  for (int k = kLevels - 1; k >= static_cast<int>(entry.level); --k) {
    if (stored[k] != vpn_at(v_addr, k)) {
      return false;
    }
  }
  return true;
}

TlbMmu::LookupOutput TlbMmu::lookup_table_comb(const LookupInput &in,
                                               const TlbEntry *entries,
                                               int entry_count) {
  LookupOutput out{};
  if (!in.valid || entries == nullptr || entry_count <= 0) {
    return out;
  }
  for (int i = 0; i < entry_count; ++i) {
    const TlbEntry &entry = entries[i];
    if (entry_matches(entry, in.v_addr, in.asid)) {
      out.hit = true;
      out.entry = entry;
      return out;
    }
  }
  return out;
}

bool TlbMmu::lookup(vaddr_t v_addr, uint16_t asid, TlbEntry &hit) const {
  if (refill_comb_.valid && entry_matches(refill_comb_.entry, v_addr, asid)) {
    hit = refill_comb_.entry;
    return true;
  }
  const LookupOutput table_lookup = lookup_table_access(v_addr, asid);
  if (table_lookup.hit) {
    hit = table_lookup.entry;
    return true;
  }
  return false;
}

TlbMmu::LookupOutput
TlbMmu::lookup_table_access(vaddr_t v_addr, uint16_t asid) const {
  LookupInput req{};
  req.valid = true;
  req.v_addr = v_addr;
  req.asid = asid;
  const int entry_count = kind_ == Kind::ITLB ? ITLB_ENTRIES : DTLB_ENTRIES;
  return lookup_table_comb(req, tlb_entries_.data(), entry_count);
}

uint32_t TlbMmu::compose_paddr(vaddr_t v_addr, const TlbEntry &e) {
  // the offset below a leaf of level L is 12 + kVpnBits * L bits; the physical space is 32 bits (a wider PPN faults in the walk)
  const uint64_t mask = (1ull << (12 + kVpnBits * static_cast<int>(e.level))) - 1ull;
  return static_cast<uint32_t>((((static_cast<uint64_t>(e.ppn) << 12) & ~mask) | (static_cast<uint64_t>(v_addr) & mask)) & 0xFFFFFFFFull);
}

bool TlbMmu::check_perm(uint8_t perm, uint32_t type, int eff_priv, bool sum,
                        bool mxr) {
  bool r = perm & PTE_R;
  bool w = perm & PTE_W;
  bool x = perm & PTE_X;
  bool u = perm & PTE_U;
  bool a = perm & PTE_A;
  bool d = perm & PTE_D;

  if (type == 0 && !x) {
    return false;
  }
  if (type == 1 && !r && !(mxr && x)) {
    return false;
  }
  if (type == 2 && !w) {
    return false;
  }
  if (eff_priv == 0 && !u) {
    return false;
  }
  if (eff_priv == 1 && u) {
    if (type == 0) {
      return false;
    }
    if (!sum) {
      return false;
    }
  }
  if (!a) {
    return false;
  }
  if (type == 2 && !d) {
    return false;
  }
  return true;
}

TlbMmu::CoreOutput TlbMmu::core_comb(const CoreInput &in) {
  CoreOutput out{};
  out.walk_next = in.walk;
  out.refill_next = in.refill;
  out.result = static_cast<uint32_t>(Result::OK);
  out.retry_reason = static_cast<uint32_t>(RetryReason::NONE);
  out.trace_source = static_cast<uint32_t>(TraceSource::NONE);
  out.trace_level = 0xff;

  if (!in.translate_valid) {
    return out;
  }

  out.resp_valid = true;
  const TranslateContext ctx_view = build_translate_context(
      in.type, in.satp, in.privilege, in.mstatus_mxr, in.mstatus_sum,
      in.mstatus_mprv, in.mstatus_mpp);
  const vaddr_t satp = ctx_view.satp;
  const int eff_priv = ctx_view.eff_priv;
  const uint32_t type = in.type;

  if (eff_priv == 3 || !sv_enabled(satp)) {
    out.p_addr = identity_pa(in.v_addr);
    out.trace_source = static_cast<uint32_t>(TraceSource::IDENTITY);
    return out;
  }
  if (!va_canonical(in.v_addr)) {   // Sv39: a non-canonical address is a page fault
    out.result = static_cast<uint32_t>(Result::FAULT);
    out.trace_source = static_cast<uint32_t>(TraceSource::TLB_PERM_FAULT);
    return out;
  }

  const uint16_t asid = sv_asid(satp);
  TlbEntry hit{};
  bool hit_valid = false;
  if (in.refill.valid && entry_matches(in.refill.entry, in.v_addr, asid)) {
    hit = in.refill.entry;
    hit_valid = true;
  } else if (in.lookup.hit) {
    hit = in.lookup.entry;
    hit_valid = true;
  }

  if (hit_valid) {
    const uint8_t perm = static_cast<uint8_t>(hit.perm);
    if (!check_perm(perm, type, eff_priv, ctx_view.sum, ctx_view.mxr)) {
      out.result = static_cast<uint32_t>(Result::FAULT);
      out.trace_source = static_cast<uint32_t>(TraceSource::TLB_PERM_FAULT);
      out.trace_level = hit.level;
      out.trace_perm = perm;
      return out;
    }
    out.p_addr = compose_paddr(in.v_addr, hit);
    out.trace_source = static_cast<uint32_t>(TraceSource::TLB_HIT);
    out.trace_level = hit.level;
    out.trace_perm = perm;
    return out;
  }

  WalkRegs walk = in.walk;
  if (!walk.active) {
    walk = {};
    walk.active = true;
    walk.req_sent = false;
    walk.v_addr = in.v_addr;
    walk.type = type;
    walk.satp = satp;
    walk.eff_priv = ctx_view.eff_priv;
    walk.mxr = ctx_view.mxr;
    walk.sum = ctx_view.sum;
  } else {
    const uint64_t walk_page = static_cast<uint64_t>(walk.v_addr) >> 12;
    const uint64_t req_page = static_cast<uint64_t>(in.v_addr) >> 12;
    if (walk_page != req_page || static_cast<vaddr_t>(walk.satp) != satp) {
      out.result = static_cast<uint32_t>(Result::RETRY);
      out.retry_reason = static_cast<uint32_t>(RetryReason::OTHER_WALK_ACTIVE);
      out.trace_source = static_cast<uint32_t>(TraceSource::WALK_SHARED);
      out.walk_next = walk;
      return out;
    }
  }

  if (!walk.req_sent) {
    if (!in.walk_req_ready) {
      out.result = static_cast<uint32_t>(Result::RETRY);
      out.retry_reason = static_cast<uint32_t>(RetryReason::WALK_REQ_BLOCKED);
      out.trace_source = static_cast<uint32_t>(TraceSource::WALK_SHARED);
      out.walk_next = walk;
      return out;
    }
    PtwWalkReq req{};
    req.vaddr = walk.v_addr;
    req.satp = walk.satp;
    req.access_type = walk.type;
    out.walk_req_valid = true;
    out.walk_req = req;
    walk.req_sent = true;
    out.result = static_cast<uint32_t>(Result::RETRY);
    out.retry_reason = static_cast<uint32_t>(RetryReason::WAIT_WALK_RESP);
    out.trace_source = static_cast<uint32_t>(TraceSource::WALK_SHARED);
    out.walk_next = walk;
    return out;
  }

  if (!in.walk_resp_valid) {
    out.result = static_cast<uint32_t>(Result::RETRY);
    out.retry_reason = static_cast<uint32_t>(RetryReason::WAIT_WALK_RESP);
    out.trace_source = static_cast<uint32_t>(TraceSource::WALK_SHARED);
    out.walk_next = walk;
    return out;
  }

  const PtwWalkResp wr = in.walk_resp;
  if ((static_cast<uint64_t>(wr.vaddr) >> 12) !=
      (static_cast<uint64_t>(walk.v_addr) >> 12)) {
    out.walk_resp_consumed = true;
    out.result = static_cast<uint32_t>(Result::RETRY);
    out.retry_reason = static_cast<uint32_t>(RetryReason::WAIT_WALK_RESP);
    out.trace_source = static_cast<uint32_t>(TraceSource::WALK_SHARED);
    out.walk_next = walk;
    return out;
  }

  out.walk_resp_consumed = true;
  out.walk_next = {};
  out.trace_source = static_cast<uint32_t>(TraceSource::WALK_SHARED);
  out.trace_level = wr.leaf_level;
  out.trace_perm = static_cast<uint8_t>(wr.leaf_pte & 0xFFu);

  if (wr.fault) {
    out.result = static_cast<uint32_t>(Result::FAULT);
    return out;
  }

  const uint8_t perm = static_cast<uint8_t>(wr.leaf_pte & 0xFFu);
  if (!check_perm(perm, type, eff_priv, ctx_view.sum, ctx_view.mxr)) {
    out.result = static_cast<uint32_t>(Result::FAULT);
    return out;
  }

  const int capacity = in.tlb_capacity > 0 ? in.tlb_capacity : 1;
  const int refill_slot = in.repl_ptr % capacity;
  PendingRefill refill{};
  refill.valid = true;
  refill.slot = refill_slot;
  refill.next_repl_ptr = (refill_slot + 1) % capacity;
  refill.entry = {};
  refill.entry.valid = true;
  refill.entry.global = (wr.leaf_pte & PTE_G) != 0;
  refill.entry.asid = asid;
  refill.entry.level = static_cast<uint32_t>(wr.leaf_level) & 3u;
  refill.entry.vpn2 = XLEN == 64 ? vpn_at(in.v_addr, 2) : 0u;
  refill.entry.vpn1 = vpn_at(in.v_addr, 1);
  refill.entry.vpn0 = vpn_at(in.v_addr, 0);
  refill.entry.ppn = (static_cast<uint64_t>(wr.leaf_pte) >> 10) & kPpnMask;
  refill.entry.perm = perm;
  out.refill_next = refill;
  out.p_addr = compose_paddr(in.v_addr, refill.entry);
  return out;
}

TlbMmu::TopOutput TlbMmu::top_comb(const TopInput &in, int entries) {
  TopOutput out{};
  out.walk_next = in.walk;
  out.refill_next = in.refill;

  if (in.flush) {
    out.table_flush = true;
    out.walk_next = {};
    out.refill_next = {};
    out.walk_client_flush = true;
    return out;
  }
  if (in.cancel) {
    out.walk_next = {};
    out.walk_client_flush = true;
    return out;
  }

  WalkRegs shadow_walk = in.walk;
  PendingRefill shadow_refill = in.refill;
  bool shadow_walk_req_ready = in.walk_req_ready;
  bool shadow_walk_resp_valid = in.walk_resp_valid;
  PtwWalkResp shadow_walk_resp = in.walk_resp;
  const int port_count =
      in.port_count < kMaxTopPortCount ? in.port_count : kMaxTopPortCount;

  for (int i = 0; i < port_count; ++i) {
    const TopPortInput &port = in.port[i];
    CoreInput core_in{};
    core_in.translate_valid = port.valid;
    core_in.v_addr = port.v_addr;
    core_in.type = port.type;
    core_in.satp = in.satp;
    core_in.privilege = in.privilege;
    core_in.mstatus_mxr = in.mstatus_mxr;
    core_in.mstatus_sum = in.mstatus_sum;
    core_in.mstatus_mprv = in.mstatus_mprv;
    core_in.mstatus_mpp = in.mstatus_mpp;
    core_in.lookup = port.lookup;
    core_in.walk = shadow_walk;
    core_in.refill = shadow_refill;
    core_in.repl_ptr = in.repl_ptr;
    core_in.tlb_capacity = in.tlb_capacity;
    core_in.walk_req_ready = shadow_walk_req_ready;
    core_in.walk_resp_valid = shadow_walk_resp_valid;
    core_in.walk_resp = shadow_walk_resp;

    const CoreOutput core_out = core_comb(core_in);
    TopPortOutput &port_out = out.port[i];
    port_out.resp_valid = core_out.resp_valid;
    port_out.result = core_out.result;
    port_out.p_addr = core_out.p_addr;
    port_out.retry_reason = core_out.retry_reason;
    port_out.trace_source = core_out.trace_source;
    port_out.trace_level = core_out.trace_level;
    port_out.trace_perm = core_out.trace_perm;

    shadow_walk = core_out.walk_next;
    shadow_refill = core_out.refill_next;
    if (core_out.walk_req_valid) {
      out.walk_req_valid = true;
      out.walk_req = core_out.walk_req;
      shadow_walk_req_ready = false;
      shadow_walk_resp_valid = false;
    }
    if (core_out.walk_resp_consumed) {
      out.walk_resp_consumed = true;
      shadow_walk_resp_valid = false;
    }
  }

  out.walk_next = shadow_walk;
  out.refill_next = shadow_refill;
  return out;
}




TlbMmu::Result TlbMmu::translate_shared_via_io_generator(
    uint32_t &p_addr, vaddr_t v_addr, uint32_t type, CsrStatusIO *status) {
  TopInput in = build_top_common_input(status);
  int port_idx = 0;
  if (kind_ == Kind::DTLB && type == 2) {
    port_idx = LSU_LDU_COUNT;
  }
  in.port[port_idx].valid = true;
  in.port[port_idx].v_addr = v_addr;
  in.port[port_idx].type = type;
  in.port[port_idx].lookup = lookup_table_access(v_addr, sv_asid(in.satp));

  const TopOutput out = run_top_via_io_generator(in);
  apply_top_output(out);

  const TopPortOutput &port_out = out.port[port_idx];
  last_retry_reason_ = static_cast<RetryReason>(
      static_cast<uint32_t>(port_out.retry_reason) & 0x7u);

  p_addr = port_out.p_addr;
  const Result result =
      static_cast<Result>(static_cast<uint32_t>(port_out.result) & 0x3u);
  const TraceSource trace_source = static_cast<TraceSource>(
      static_cast<uint32_t>(port_out.trace_source) & 0x7u);
  if (trace_source != TraceSource::NONE) {

  }
  return result;
}

TlbMmu::TopInput
TlbMmu::build_top_common_input(const CsrStatusIO *status) const {
  const vaddr_t satp =
      status != nullptr ? static_cast<vaddr_t>(status->satp) : vaddr_t(0);
  const uint32_t mstatus =
      status != nullptr ? static_cast<uint32_t>(status->mstatus) : 0u;
  const uint32_t privilege =
      status != nullptr ? static_cast<uint32_t>(status->privilege) : 0u;
  const PtwWalkPortCombOut walk_port_out =
      walk_port != nullptr ? walk_port->comb_output() : PtwWalkPortCombOut{};

  TopInput in{};
  in.satp = satp;
  in.privilege = privilege;
  in.mstatus_mxr = (mstatus & MSTATUS_MXR) != 0;
  in.mstatus_sum = (mstatus & MSTATUS_SUM) != 0;
  in.mstatus_mprv = (mstatus & MSTATUS_MPRV) != 0;
  in.mstatus_mpp = (mstatus >> MSTATUS_MPP_SHIFT) & 0x3u;
  in.walk = visible_walk_regs();
  in.refill = refill_comb_;
  in.repl_ptr = repl_ptr_;
  in.tlb_capacity = tlb_capacity_ > 0 ? tlb_capacity_ : 1;
  in.walk_req_ready = walk_port_out.req_ready;
  in.walk_resp_valid = walk_port_out.resp_valid;
  in.walk_resp = walk_port_out.resp;
  in.port_count = kind_ == Kind::ITLB ? kItlbTopPortCount : kDtlbTopPortCount;
  return in;
}

TlbMmu::TopOutput TlbMmu::run_top_via_io_generator(const TopInput &in) const {
  return top_comb(in, kind_ == Kind::ITLB ? ITLB_ENTRIES : DTLB_ENTRIES);
}

void TlbMmu::apply_top_output(const TopOutput &out) {
  if (out.walk_client_flush && walk_port != nullptr) {
    walk_port->flush_client();
  }
  if (out.walk_req_valid && walk_port != nullptr) {
    const bool accepted = walk_port->send_walk_req(out.walk_req);
    Assert(accepted &&
           "TLB top emitted a PTW walk request only when req_ready was set");
  }
  if (out.walk_resp_consumed && walk_port != nullptr) {
    walk_port->consume_resp();
  }

  if (out.table_flush) {
    flush_pending_ = true;
  }
  walk_comb_ = out.walk_next;
  walk_comb_valid_ = true;
  refill_comb_ = out.refill_next;
}

void TlbMmu::run_control_via_top(bool flush, bool cancel) {
  TopInput in = build_top_common_input(nullptr);
  in.flush = flush;
  in.cancel = cancel;
  const TopOutput out = run_top_via_io_generator(in);
  apply_top_output(out);
}

bool TlbMmu::consume_completed_walk_refill(bool consume_fault) {
  if (walk_port == nullptr) {
    return false;
  }

  const WalkRegs &walk = visible_walk_regs();
  const PtwWalkPortCombOut walk_out = walk_port->comb_output();
  if (!walk.active || !walk.req_sent || !walk_out.resp_valid ||
      ((static_cast<uint64_t>(walk_out.resp.vaddr) >> 12) !=
       (static_cast<uint64_t>(walk.v_addr) >> 12))) {
    return false;
  }

  if (walk_out.resp.fault) {
    if (!consume_fault) {
      return false;
    }
  } else {
    const uint8_t perm = walk_out.resp.leaf_pte & 0xFF;
    const bool perm_ok =
        check_perm(perm, walk.type, walk.eff_priv, walk.sum, walk.mxr);
    if (!perm_ok && !consume_fault) {
      return false;
    }
    if (perm_ok) {
      const bool global = (walk_out.resp.leaf_pte & PTE_G) != 0;
      schedule_refill(walk.v_addr, sv_asid(walk.satp),
                      walk_out.resp.leaf_level, walk_out.resp.leaf_pte,
                      global);
    }
  }

  walk_port->consume_resp();
  return true;
}

void TlbMmu::translate_lsu_ports(const LsuMMUIO &req, MMULsuIO &resp) {
  resp = {};
  if (consume_completed_walk_refill(false)) {
    walk_comb_ = {};
    walk_comb_valid_ = true;
  }

  TopInput in = build_top_common_input(&req.csr_status);
  const uint16_t asid = sv_asid(in.satp);

  for (int i = 0; i < LSU_LDU_COUNT; ++i) {
    const MMUReq &ld_req = req.ldq_req[i];
    auto &port = in.port[i];
    port.valid = ld_req.valid;
    port.v_addr = ld_req.vaddr;
    port.type = 1;
    if (ld_req.valid) {
      port.lookup = lookup_table_access(ld_req.vaddr, asid);
    }
  }
  for (int i = 0; i < LSU_STA_COUNT; ++i) {
    const int port_idx = LSU_LDU_COUNT + i;
    const MMUReq &st_req = req.stq_req[i];
    auto &port = in.port[port_idx];
    port.valid = st_req.valid;
    port.v_addr = st_req.vaddr;
    port.type = 2;
    if (st_req.valid) {
      port.lookup = lookup_table_access(st_req.vaddr, asid);
    }
  }

  const TopOutput out = run_top_via_io_generator(in);
  apply_top_output(out);

  int last_valid_port = -1;
  for (int i = 0; i < LSU_LDU_COUNT; ++i) {
    if (!req.ldq_req[i].valid) {
      continue;
    }
    const TopPortOutput &port = out.port[i];
    MMUResp &ld_resp = resp.ldq_resp[i];
    const Result result =
        static_cast<Result>(static_cast<uint32_t>(port.result) & 0x3u);
    ld_resp.valid = port.resp_valid;
    ld_resp.result = to_lsu_mmu_result(result);
    ld_resp.paddr = result == Result::OK ? port.p_addr : 0;
    ld_resp.entry_idx = req.ldq_req[i].entry_idx;
    ld_resp.wait_idx = req.ldq_req[i].wait_idx;
    last_valid_port = i;
    const TraceSource trace_source = static_cast<TraceSource>(
        static_cast<uint32_t>(port.trace_source) & 0x7u);
    if (trace_source != TraceSource::NONE) {

    }
  }
  for (int i = 0; i < LSU_STA_COUNT; ++i) {
    if (!req.stq_req[i].valid) {
      continue;
    }
    const int port_idx = LSU_LDU_COUNT + i;
    const TopPortOutput &port = out.port[port_idx];
    MMUResp &st_resp = resp.stq_resp[i];
    const Result result =
        static_cast<Result>(static_cast<uint32_t>(port.result) & 0x3u);
    st_resp.valid = port.resp_valid;
    st_resp.result = to_lsu_mmu_result(result);
    st_resp.paddr = result == Result::OK ? port.p_addr : 0;
    st_resp.entry_idx = req.stq_req[i].entry_idx;
    st_resp.wait_idx = req.stq_req[i].wait_idx;
    last_valid_port = port_idx;
    const TraceSource trace_source = static_cast<TraceSource>(
        static_cast<uint32_t>(port.trace_source) & 0x7u);
    if (trace_source != TraceSource::NONE) {

    }
  }
  if (last_valid_port >= 0) {
    last_retry_reason_ = static_cast<RetryReason>(
        static_cast<uint32_t>(out.port[last_valid_port].retry_reason) & 0x7u);
  }
}

void TlbMmu::schedule_refill(vaddr_t v_addr, uint16_t asid, uint8_t level,
                             uint64_t pte, bool global) {
  PendingRefill refill{};
  refill.valid = true;
  refill.slot = repl_ptr_;
  refill.next_repl_ptr = (repl_ptr_ + 1) % tlb_capacity_;
  refill.entry = {};
  refill.entry.valid = true;
  refill.entry.global = global;
  refill.entry.asid = asid;
  refill.entry.level = level;
  refill.entry.vpn2 = XLEN == 64 ? vpn_at(v_addr, 2) : 0u;
  refill.entry.vpn1 = vpn_at(v_addr, 1);
  refill.entry.vpn0 = vpn_at(v_addr, 0);
  refill.entry.ppn = (pte >> 10) & kPpnMask;
  refill.entry.perm = pte & 0xFF;
  refill_comb_ = refill;
}

TlbMmu::Result TlbMmu::walk_and_refill(uint32_t &p_addr, vaddr_t v_addr,
                                       uint32_t type, CsrStatusIO *status) {
  const TranslateContext ctx_view = build_translate_context(type, status);
  WalkRegs &walk = ensure_walk_comb();

  if (!walk.active) {
    if (!walker.start(v_addr, ctx_view.satp)) {
      last_retry_reason_ = RetryReason::LOCAL_WALKER_BUSY;
      return Result::RETRY;
    }
    walk = {};
    walk.active = true;
    walk.v_addr = v_addr;
    walk.type = type;
    walk.satp = ctx_view.satp;
    walk.eff_priv = ctx_view.eff_priv;
    walk.mxr = ctx_view.mxr;
    walk.sum = ctx_view.sum;
  } else {
    if (walk.v_addr != v_addr || walk.type != type || walk.satp != ctx_view.satp) {
      last_retry_reason_ = RetryReason::LOCAL_WALKER_BUSY;
      return Result::RETRY;
    }
  }

  auto st = walker.tick();
  if (st != PtwWalker::State::DONE) {
    if (st == PtwWalker::State::FAULT) {
      walker.flush();
      walk = {};
      last_retry_reason_ = RetryReason::NONE;

      return Result::FAULT;
    }
    last_retry_reason_ = RetryReason::WAIT_WALK_RESP;

    return Result::RETRY;
  }

  uint64_t leaf_pte = walker.leaf_pte();
  uint8_t leaf_level = walker.leaf_level();
  uint8_t perm = leaf_pte & 0xFF;
  if (!check_perm(perm, walk.type, walk.eff_priv, walk.sum, walk.mxr)) {
    walker.flush();
    walk = {};
    last_retry_reason_ = RetryReason::NONE;

    return Result::FAULT;
  }

  const bool global = (leaf_pte & PTE_G) != 0;
  const uint16_t asid = sv_asid(walk.satp);
  schedule_refill(walk.v_addr, asid, leaf_level, leaf_pte, global);
  TlbEntry refill_hit{};
  const bool hit = lookup(walk.v_addr, asid, refill_hit);
  Assert(hit && "Local PTW refill must be visible through same-cycle bypass");
  p_addr = compose_paddr(walk.v_addr, refill_hit);
  walker.flush();
  walk = {};
  last_retry_reason_ = RetryReason::NONE;

  return Result::OK;
}

TlbMmu::Result TlbMmu::walk_and_refill_shared(uint32_t &p_addr, vaddr_t v_addr,
                                              uint32_t type,
                                              CsrStatusIO *status) {
  const TranslateContext ctx_view = build_translate_context(type, status);
  WalkRegs &walk = ensure_walk_comb();

  if (!walk.active) {
    walk = {};
    walk.active = true;
    walk.req_sent = false;
    walk.v_addr = v_addr;
    walk.type = type;
    walk.satp = ctx_view.satp;
    walk.eff_priv = ctx_view.eff_priv;
    walk.mxr = ctx_view.mxr;
    walk.sum = ctx_view.sum;
  } else {
    const uint64_t walk_page = static_cast<uint64_t>(walk.v_addr) >> 12;
    const uint64_t req_page = static_cast<uint64_t>(v_addr) >> 12;
    if (walk_page != req_page || walk.satp != ctx_view.satp) {
      last_retry_reason_ = RetryReason::OTHER_WALK_ACTIVE;
      return Result::RETRY;
    }
  }

  if (!walk.req_sent) {
    PtwWalkReq req{};
    req.vaddr = walk.v_addr;
    req.satp = walk.satp;
    req.access_type = walk.type;
    if (!walk_port->send_walk_req(req)) {
      last_retry_reason_ = RetryReason::WALK_REQ_BLOCKED;

      return Result::RETRY;
    }
    walk.req_sent = true;
  }

  if (!walk_port->resp_valid()) {
    last_retry_reason_ = RetryReason::WAIT_WALK_RESP;

    return Result::RETRY;
  }

  PtwWalkResp wr = walk_port->resp();
  if ((static_cast<uint64_t>(wr.vaddr) >> 12) != (static_cast<uint64_t>(walk.v_addr) >> 12)) {
    walk_port->consume_resp();
    last_retry_reason_ = RetryReason::WAIT_WALK_RESP;

    return Result::RETRY;
  }
  walk_port->consume_resp();
  walk = {};

  if (wr.fault) {
    last_retry_reason_ = RetryReason::NONE;

    return Result::FAULT;
  }

  uint8_t perm = wr.leaf_pte & 0xFF;
  if (!check_perm(perm, type, ctx_view.eff_priv, ctx_view.sum, ctx_view.mxr)) {
    last_retry_reason_ = RetryReason::NONE;

    return Result::FAULT;
  }

  const bool global = (wr.leaf_pte & PTE_G) != 0;
  const uint16_t asid = sv_asid(ctx_view.satp);
  schedule_refill(v_addr, asid, wr.leaf_level, wr.leaf_pte, global);
  TlbEntry refill_hit{};
  const bool hit = lookup(v_addr, asid, refill_hit);
  Assert(hit && "Shared PTW refill must be visible through same-cycle bypass");
  p_addr = compose_paddr(v_addr, refill_hit);
  last_retry_reason_ = RetryReason::NONE;

  return Result::OK;
}

bool TlbMmu::peek_translate(uint32_t &p_addr, vaddr_t v_addr, uint32_t type, CsrStatusIO *status) const {
  const TranslateContext ctx_view = build_translate_context(type, status);
  const vaddr_t satp = ctx_view.satp;
  if (ctx_view.eff_priv == 3 || !sv_enabled(satp)) { p_addr = identity_pa(v_addr); return true; }
  if (!va_canonical(v_addr)) return false;
  TlbEntry hit = {};
  if (!lookup(v_addr, sv_asid(satp), hit)) return false;
  if (!check_perm(hit.perm, type, ctx_view.eff_priv, ctx_view.sum, ctx_view.mxr)) return false;
  p_addr = compose_paddr(v_addr, hit);
  return true;
}

TlbMmu::Result TlbMmu::translate(uint32_t &p_addr, vaddr_t v_addr,
                                 uint32_t type, CsrStatusIO *status) {
  last_retry_reason_ = RetryReason::NONE;
  if (walk_port != nullptr) {
    const Result ret =
        translate_shared_via_io_generator(p_addr, v_addr, type, status);
    return ret;
  }

  const TranslateContext ctx_view = build_translate_context(type, status);
  const vaddr_t satp = ctx_view.satp;
  const int eff_priv = ctx_view.eff_priv;

  if (eff_priv == 3 || !sv_enabled(satp)) {
    p_addr = identity_pa(v_addr);

    return Result::OK;
  }

  const uint16_t asid = sv_asid(satp);

  TlbEntry hit = {};
  if (lookup(v_addr, asid, hit)) {
    const bool perm_ok =
        check_perm(hit.perm, type, eff_priv, ctx_view.sum, ctx_view.mxr);
    if (!perm_ok) {

      return Result::FAULT;
    }
    p_addr = compose_paddr(v_addr, hit);

    return Result::OK;
  }
  if (walk_port != nullptr) {
    return walk_and_refill_shared(p_addr, v_addr, type, status);
  }
  return walk_and_refill(p_addr, v_addr, type, status);
}
