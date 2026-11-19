#include "TlbMmu.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "Csr.h"
#include "ref.h"

namespace {
// Sv39: 3 levels of 9-bit VPNs, 16-bit ASID in satp[59:44]
constexpr int kLevels = 3;
constexpr int kVpnBits = 9;
constexpr uint64_t kPpnMask = 0xFFFFFFFFFFFull;

inline uint16_t sv_asid(vaddr_t satp) { return static_cast<uint16_t>((satp >> 44) & 0xFFFFu); }
inline bool sv_enabled(vaddr_t satp) { return (satp >> 60) == 8; }   // MODE = Sv39
// Sv39 virtual addresses must be sign-extended from bit 38
inline bool va_canonical(vaddr_t v) {
  const int64_t top = static_cast<int64_t>(v) >> 38;
  return top == 0 || top == -1;
}
// an untranslated address as a physical one; above the 32-bit physical space it maps to an address outside RAM
inline uint32_t identity_pa(vaddr_t v) { return ((static_cast<uint64_t>(v) >> 32) != 0) ? 0x7fffffffu : static_cast<uint32_t>(v); }
inline uint32_t vpn_at(vaddr_t v, int level) { return static_cast<uint32_t>((v >> (12 + kVpnBits * level)) & ((1u << kVpnBits) - 1)); }
inline uint64_t page_of(vaddr_t v) { return static_cast<uint64_t>(v) >> 12; }
}  // namespace

namespace {
// TLB_REPL (read once): rr (default, = RTL) round-robin pointer; nru: not-recently-used bit per entry (a hit on any
// port sets it; when all are set, all but the touched one clear; the victim is the first invalid entry, else the lowest
// unused); plru: tree pseudo-LRU over the entries (first invalid entry first). Both TLBs.
int tlb_repl() {
  static const int m = [] {
    const char *v = std::getenv("TLB_REPL");
    if (!v || std::strcmp(v, "rr") == 0) return 0;
    if (std::strcmp(v, "nru") == 0) return 1;
    if (std::strcmp(v, "plru") == 0) return 2;
    std::fprintf(stderr, "TLB_REPL=%s: unknown\n", v);
    std::abort();
  }();
  return m;
}
}  // namespace

void TlbMmu::repl_touch(int slot) {
  if (tlb_repl() == 1) {
    used_[slot] = 1;
    bool all = true;
    for (uint8_t u : used_) all = all && u;
    if (all) { for (auto &u : used_) u = 0; used_[slot] = 1; }
  } else if (tlb_repl() == 2) {
    int node = 0, lo = 0, hi = capacity_;
    while (hi - lo > 1) {
      const int mid = (lo + hi) / 2;
      const bool right = slot >= mid;
      plru_[node] = right ? 0 : 1;
      node = 2 * node + (right ? 2 : 1);
      if (right) lo = mid; else hi = mid;
    }
  }
}

int TlbMmu::repl_victim() const {
  for (int i = 0; i < capacity_; ++i)
    if (!entries_[i].valid) return i;
  if (tlb_repl() == 1) {
    for (int i = 0; i < capacity_; ++i)
      if (!used_[i]) return i;
    return 0;
  }
  int node = 0, lo = 0, hi = capacity_;
  while (hi - lo > 1) {
    const int mid = (lo + hi) / 2;
    const bool right = plru_[node] != 0;
    node = 2 * node + (right ? 2 : 1);
    if (right) lo = mid; else hi = mid;
  }
  return lo;
}

TlbMmu::TlbMmu(int entries, Kind kind) : kind_(kind), entries_(entries > 0 ? entries : 1), capacity_(entries > 0 ? entries : 1) {
  used_.assign(capacity_, 0);
  plru_.assign(capacity_, 0);
  reset();
}

void TlbMmu::reset() {
  flush_ = true;
  seq();
}

void TlbMmu::warm_install(const TlbEntry *entries, int count, int repl_ptr) {
  const int n = count < capacity_ ? count : capacity_;
  for (int i = 0; i < n; ++i) entries_[i] = entries[i];
  repl_ptr_ = capacity_ > 0 ? repl_ptr % capacity_ : 0;
}

// The translation context of a request: satp, the effective privilege (MPRV: loads / stores at MPP), MXR, SUM.
TlbMmu::Ctx TlbMmu::context(uint32_t type) {
  const CsrStatusIO &s = csr_state().status;
  const uint32_t mstatus = static_cast<uint32_t>(s.mstatus);   // (status bits below 32 only)
  Ctx c;
  c.satp = static_cast<vaddr_t>(s.satp);
  c.eff_priv = static_cast<int>(s.privilege) & 0x3;
  c.mxr = (mstatus & MSTATUS_MXR) != 0;
  c.sum = (mstatus & MSTATUS_SUM) != 0;
  if (type != 0 && (mstatus & MSTATUS_MPRV)) c.eff_priv = (mstatus >> MSTATUS_MPP_SHIFT) & 0x3u;
  return c;
}

bool TlbMmu::entry_matches(const TlbEntry &e, vaddr_t v_addr, uint16_t asid) {
  if (!e.valid || !(e.global || e.asid == asid)) return false;
  // every VPN above the leaf's level must match (a superpage ignores the lower ones)
  const uint32_t stored[3] = {e.vpn0, e.vpn1, e.vpn2};
  for (int k = kLevels - 1; k >= static_cast<int>(e.level); --k)
    if (stored[k] != vpn_at(v_addr, k)) return false;
  return true;
}

const TlbMmu::TlbEntry *TlbMmu::lookup(vaddr_t v_addr, uint16_t asid) const {
  for (const TlbEntry &e : entries_)
    if (entry_matches(e, v_addr, asid)) return &e;
  return nullptr;
}

uint32_t TlbMmu::compose_paddr(vaddr_t v_addr, const TlbEntry &e) {
  // the offset below a leaf of level L is 12 + kVpnBits * L bits; the physical space is 32 bits (a wider PPN faults in the walk)
  const uint64_t mask = (1ull << (12 + kVpnBits * static_cast<int>(e.level))) - 1ull;
  return static_cast<uint32_t>((((static_cast<uint64_t>(e.ppn) << 12) & ~mask) | (static_cast<uint64_t>(v_addr) & mask)) & 0xFFFFFFFFull);
}

bool TlbMmu::check_perm(uint8_t perm, uint32_t type, int eff_priv, bool sum, bool mxr) {
  const bool r = perm & PTE_R, w = perm & PTE_W, x = perm & PTE_X, u = perm & PTE_U, a = perm & PTE_A, d = perm & PTE_D;
  if (type == 0 && !x) return false;
  if (type == 1 && !r && !(mxr && x)) return false;
  if (type == 2 && !w) return false;
  if (eff_priv == 0 && !u) return false;
  if (eff_priv == 1 && u && (type == 0 || !sum)) return false;
  if (!a) return false;
  if (type == 2 && !d) return false;
  return true;
}

// The entry of a completed walk, in the slot of the round-robin pointer.
TlbMmu::Refill TlbMmu::make_refill(vaddr_t v_addr, uint16_t asid, const PtwWalkResp &wr) const {
  Refill f;
  f.valid = true;
  f.slot = tlb_repl() ? repl_victim() : repl_ptr_ % capacity_;
  f.next_repl = (f.slot + 1) % capacity_;
  f.entry.valid = true;
  f.entry.global = (wr.leaf_pte & PTE_G) != 0;
  f.entry.asid = asid;
  f.entry.level = static_cast<uint32_t>(wr.leaf_level) & 3u;
  f.entry.vpn2 = vpn_at(v_addr, 2);
  f.entry.vpn1 = vpn_at(v_addr, 1);
  f.entry.vpn0 = vpn_at(v_addr, 0);
  f.entry.ppn = (static_cast<uint64_t>(wr.leaf_pte) >> 10) & kPpnMask;
  f.entry.perm = static_cast<uint8_t>(wr.leaf_pte & 0xFFu);
  return f;
}

// The walk port as this cycle left it: the walker's registered outputs, then what this cycle's steps did to them (a
// request takes req_ready, a consumed or dropped response is gone, a dropped walk makes the walker ready).
PtwWalkPortCombOut &TlbMmu::port() {
  if (!port_valid_) {
    Assert(walk_resp != nullptr && walk_req != nullptr && "TlbMmu: walk port not bound");
    port_ = *walk_resp;
    port_valid_ = true;
  }
  return port_;
}
void TlbMmu::send(const PtwWalkReq &r) {
  Assert(port().req_ready && "TLB: a walk request only when the walker is ready");
  walk_req->req_valid = true;
  walk_req->req = r;
  port_.req_ready = false;
  port_.resp_valid = false;
  port_.resp = {};
}
void TlbMmu::consume() {
  walk_req->resp_consumed = true;
  port().resp_valid = false;
  port_.resp = {};
}
void TlbMmu::drop_walk() {
  walk_req->flush = true;
  port().req_ready = true;
  port_.resp_valid = false;
  port_.resp = {};
}

// A response of the walk in flight that has arrived: its entry becomes the refill (if the walk's request may use it) and
// the response is consumed. A fault, or a permission the walk's request lacks, is left for that request (unless
// consume_fault: the walk is being dropped).
bool TlbMmu::take_completed_walk(bool consume_fault) {
  const WalkRegs &w = walk();
  const PtwWalkPortCombOut &p = port();
  if (!w.active || !w.req_sent || !p.resp_valid || page_of(p.resp.vaddr) != page_of(w.v_addr)) return false;
  if (p.resp.fault) {
    if (!consume_fault) return false;
  } else {
    const bool perm_ok = check_perm(static_cast<uint8_t>(p.resp.leaf_pte & 0xFF), w.type, w.eff_priv, w.sum, w.mxr);
    if (!perm_ok && !consume_fault) return false;
    if (perm_ok) refill_ = make_refill(w.v_addr, sv_asid(w.satp), p.resp);
  }
  consume();
  return true;
}

// One request: bare / M-mode identity, a non-canonical fault, a hit (this cycle's refill first, then the table) with its
// permission check, else the walk: started for this page, sent when the walker is ready, its response taken (another
// page's response is dropped), a fault or a permission fault, else the refill and the address.
TlbMmu::Resp TlbMmu::translate_one(const Req &r, const Ctx &c, WalkRegs &w) {
  Resp out;
  out.valid = true;
  out.result = Result::OK;
  if (c.eff_priv == 3 || !sv_enabled(c.satp)) { out.paddr = identity_pa(r.vaddr); return out; }
  out.result = Result::FAULT;
  if (!va_canonical(r.vaddr)) return out;
  const uint16_t asid = sv_asid(c.satp);
  const TlbEntry *hit = refill_.valid && entry_matches(refill_.entry, r.vaddr, asid) ? &refill_.entry : lookup(r.vaddr, asid);
  if (hit) {
    if (tlb_repl() && hit != &refill_.entry) touch_.push_back(int(hit - entries_.data()));
    if (!check_perm(static_cast<uint8_t>(hit->perm), r.type, c.eff_priv, c.sum, c.mxr)) return out;
    out.result = Result::OK;
    out.paddr = compose_paddr(r.vaddr, *hit);
    return out;
  }
  out.result = Result::RETRY;
  if (!w.active) {
    w = {};
    w.active = true;
    w.v_addr = r.vaddr;
    w.type = r.type;
    w.satp = c.satp;
    w.eff_priv = c.eff_priv;
    w.mxr = c.mxr;
    w.sum = c.sum;
  } else if (page_of(w.v_addr) != page_of(r.vaddr) || w.satp != c.satp) {
    return out;   // the walk is for another page
  }
  if (!w.req_sent) {
    if (!port().req_ready) return out;
    PtwWalkReq q{};
    q.vaddr = w.v_addr;
    q.satp = w.satp;
    send(q);
    w.req_sent = true;
    return out;
  }
  if (!port().resp_valid) return out;
  const PtwWalkResp wr = port_.resp;
  consume();
  if (page_of(wr.vaddr) != page_of(w.v_addr)) return out;   // a response for another page
  w = {};
  out.result = Result::FAULT;
  if (wr.fault) return out;
  if (!check_perm(static_cast<uint8_t>(wr.leaf_pte & 0xFFu), r.type, c.eff_priv, c.sum, c.mxr)) return out;
  refill_ = make_refill(r.vaddr, asid, wr);
  out.result = Result::OK;
  out.paddr = compose_paddr(r.vaddr, refill_.entry);
  return out;
}

// One cycle of the TLB: a flush (the table and the refill are dropped at the edge, the walk now), a cancel of the walk
// (a response that has arrived is still taken), then n request ports in order (the DTLB first takes a walk response
// that has arrived for its walk; invalid ports get RETRY).
void TlbMmu::comb(bool flush, bool cancel, const Req *req, Resp *resp, int n) {
  if (flush) {
    flush_ = true;
    walk_next_ = {};
    walk_next_valid_ = true;
    refill_ = {};
    drop_walk();
  }
  if (cancel) {
    take_completed_walk(true);
    walk_next_ = {};
    walk_next_valid_ = true;
    drop_walk();
  }
  if (n == 0) return;
  if (kind_ == Kind::DTLB && take_completed_walk(false)) {
    walk_next_ = {};
    walk_next_valid_ = true;
  }
  WalkRegs w = walk();
  for (int p = 0; p < n; p++) resp[p] = req[p].valid ? translate_one(req[p], context(req[p].type), w) : Resp{};
  walk_next_ = w;
  walk_next_valid_ = true;
}

// FE_IPF: lookup and permission only (this cycle's refill first, then the table).
bool TlbMmu::peek_translate(uint32_t &p_addr, vaddr_t v_addr, uint32_t type) const {
  const Ctx c = context(type);
  if (c.eff_priv == 3 || !sv_enabled(c.satp)) { p_addr = identity_pa(v_addr); return true; }
  if (!va_canonical(v_addr)) return false;
  const uint16_t asid = sv_asid(c.satp);
  const TlbEntry *hit = refill_.valid && entry_matches(refill_.entry, v_addr, asid) ? &refill_.entry : lookup(v_addr, asid);
  if (!hit || !check_perm(static_cast<uint8_t>(hit->perm), type, c.eff_priv, c.sum, c.mxr)) return false;
  p_addr = compose_paddr(v_addr, *hit);
  return true;
}

// The clock edge: a flush clears the table, the round-robin pointer and the walk; else the refill is written and the
// walk takes its next state.
void TlbMmu::seq() {
  port_valid_ = false;
  if (flush_) {
    for (auto &e : entries_) e = {};
    repl_ptr_ = 0;
    std::fill(used_.begin(), used_.end(), 0);
    std::fill(plru_.begin(), plru_.end(), 0);
    touch_.clear();
    walk_ = {};
    walk_next_valid_ = false;
    refill_ = {};
    flush_ = false;
    return;
  }
  for (int s : touch_) repl_touch(s);
  touch_.clear();
  if (refill_.valid) {
    if (tlb_repl()) repl_touch(refill_.slot);
    entries_[refill_.slot] = refill_.entry;
    repl_ptr_ = refill_.next_repl;
    refill_ = {};
  }
  if (walk_next_valid_) {
    walk_ = walk_next_;
    walk_next_valid_ = false;
  }
}
