// I-cache top: the tag / valid arrays, the ITLB (TlbMmu), the memory ports and the icache_module state machine, one cycle per
// icache_comb_calc() call. The module computes the request / miss / refill state; this file only connects it:
//   arrays (register file, read in the cycle of the lookup) -> module meta lookup -> ITLB translation -> module meta + data lookup
//   -> memory read / D-cache probe requests -> fetch-group output (instruction words from the functional memory) -> clock edge.
#include "trace_hook.h"
#include "../front_IO.h"
#include "../front_module.h"
#include "../frontend.h"
#include "AXI_Interconnect_IO.h"
#include "Csr.h"
#include "GenericTable.h"
#include "FrontTop.h"
#include "PtwWalkPort.h"
#include "RISCV.h"
#include "TlbMmu.h"
#include "config.h"
#include "include/icache_module.h"
#include <cstdio>
#include <cstdlib>

// ---- ports bound by FrontTop::init (front_module.h) ----------------------------------------------------------------------------------
icache_module_n::ICache icache;
PtwWalkPort *icache_ptw_walk_port = nullptr;
ICacheMemPortReq *icache_mem_req_port = nullptr;
ICacheMemPortResp *icache_mem_resp_port = nullptr;
axi_interconnect::ReadMasterPort_t *icache_mem_read_port = nullptr;
void (*g_icache_fill_hook)(uint32_t index, uint32_t tag) = nullptr;   // FE_IPF: demand fills are reported to the prefetcher

static_assert(ICACHE_LINE_SIZE <= axi_interconnect::MAX_READ_TRANSACTION_BYTES, "ICACHE_LINE_SIZE exceeds the AXI read transaction limit");

namespace {

constexpr int kLineSlots = ICACHE_LINE_SIZE / SLOT_BYTES;
using TagTable = GenericTable<ICACHE_SET_NUM, ICACHE_WAY_NUM, ICACHE_TAG_BITS>;
using ValidTable = GenericTable<ICACHE_SET_NUM, ICACHE_WAY_NUM, 1>;
TagTable g_tags;
ValidTable g_valid;

// The array read of this cycle, as the module sees it.
struct LookupResp {
  bool meta_valid = false, data_valid = false;
  uint32_t index = 0;
  uint32_t tag[ICACHE_WAY_NUM] = {};
  bool valid[ICACHE_WAY_NUM] = {};
} g_lookup;

// What the I-cache is bound to: the context, the ports (taken from the globals at the first cycle that uses them), the ITLB.
struct Binding {
  SimContext *ctx = nullptr, *ctx_set = nullptr;   // ctx_set: icache_set_context(); ctx: bound at the next cycle
  PtwWalkPort *ptw_walk = nullptr;
  axi_interconnect::ReadMasterPort_t *read = nullptr;
  ICacheMemPortReq *probe_req = nullptr;
  ICacheMemPortResp *probe_resp = nullptr;
  TlbMmu *itlb = nullptr;

  void bind() {
    if (ptw_walk != icache_ptw_walk_port) { ptw_walk = icache_ptw_walk_port; if (itlb) itlb->set_ptw_walk_port(ptw_walk); }
    read = icache_mem_read_port;
    probe_req = icache_mem_req_port;
    probe_resp = icache_mem_resp_port;
    ctx = ctx_set;
  }
  TlbMmu *mmu() {
    if (!itlb && ctx) {
      itlb = new TlbMmu(ctx, ITLB_ENTRIES, TlbMmu::Kind::ITLB);
      if (ptw_walk) itlb->set_ptw_walk_port(ptw_walk);
    }
    return itlb;
  }
} g_bind;

void clear_outputs(icache_out *out) {
  out->icache_read_complete = false;
  out->fetch_pc = 0;
  for (int i = 0; i < FETCH_WIDTH; ++i) { out->fetch_group[i] = INST_NOP; out->page_fault_inst[i] = false; out->inst_valid[i] = false; }
  out->perf_req_fire = out->perf_req_blocked = out->perf_resp_fire = out->perf_miss_event = out->perf_miss_busy = false;
  out->perf_outstanding_req = out->perf_itlb_hit = out->perf_itlb_miss = out->perf_itlb_fault = out->perf_itlb_retry = false;
  out->perf_itlb_retry_other_walk = out->perf_itlb_retry_walk_req_blocked = false;
  out->perf_itlb_retry_wait_walk_resp = out->perf_itlb_retry_local_walker_busy = false;
}

// The front end may send a request: the module is idle and ready, no recovery (fence.i / sfence / invalidate) in progress.
bool read_ready(bool hold_for_recovery, bool reset) {
  if (reset) return true;
  const auto &r = icache.io.regs;
  const bool busy = r.lookup_pending_r || r.req_valid_r || r.miss_txid_valid_r || r.state != icache_module_n::IDLE ||
                    r.mem_axi_state != icache_module_n::AXI_IDLE;
  return !hold_for_recovery && r.ifu_req_ready_r && !busy && icache.io.out.ifu_req_ready;
}

// Fetch-group output of the module's response: the words come from the functional memory (the I-cache keeps no data array).
void drive_fetch_group(icache_out *out) {
  const auto &o = icache.io.out;
  if (!o.ifu_resp_valid) return;
  out->icache_read_complete = true;
  out->fetch_pc = o.ifu_resp_pc;
  out->fetch_paddr = o.ifu_resp_paddr;
  out->fetch_single_word = o.ifu_resp_single_word;
  out->fetch_page_fault = o.ifu_page_fault;
  const bool pf = o.ifu_page_fault;
  if (o.ifu_resp_single_word) {   // an uncached single word
    out->fetch_group[0] = pf ? INST_NOP : tbhook::mem_read(o.ifu_resp_paddr);
    out->page_fault_inst[0] = pf;
    out->inst_valid[0] = true;
    if (kRvc) {   // one 4-byte word = two 2-byte slots; the window of the second slot is the word's upper half plus the next word
      out->fetch_group[1] = pf ? INST_NOP : tbhook::fetch_window_at(o.ifu_resp_paddr + SLOT_BYTES);
      out->page_fault_inst[1] = pf;
      out->inst_valid[1] = true;
    }
    return;
  }
  const int base = static_cast<int>((o.ifu_resp_pc & (ICACHE_LINE_SIZE - 1u)) / SLOT_BYTES);
  for (int i = 0; i < FETCH_WIDTH && base + i < kLineSlots; ++i) {
    out->fetch_group[i] = pf ? INST_NOP : tbhook::fetch_window_at(o.ifu_resp_paddr + SLOT_BYTES * i);
    out->page_fault_inst[i] = pf;
    out->inst_valid[i] = true;
  }
}

void drive_perf(icache_out *out, bool req_valid, bool itlb_invoked, TlbMmu::Result itlb_ret, TlbMmu::RetryReason why) {
  const auto &h = icache;
  out->perf_req_fire = req_valid && out->icache_read_ready;
  out->perf_req_blocked = req_valid && !out->icache_read_ready;
  out->perf_resp_fire = h.io.out.ifu_resp_valid && h.io.in.ifu_resp_ready;
  out->perf_miss_event = h.perf.miss_issue_valid;
  const auto st = static_cast<icache_module_n::ICacheState>(h.io.regs.state);
  out->perf_miss_busy = st == icache_module_n::SWAP_IN || st == icache_module_n::WAIT_DCACHE_AFTER_MEM || st == icache_module_n::DRAIN;
  out->perf_outstanding_req = out->perf_miss_busy || h.io.regs.req_valid_r || h.io.regs.lookup_pending_r;
  if (!itlb_invoked) return;
  if (itlb_ret == TlbMmu::Result::OK) out->perf_itlb_hit = true;
  else if (itlb_ret == TlbMmu::Result::FAULT) out->perf_itlb_fault = true;
  else {
    out->perf_itlb_miss = out->perf_itlb_retry = true;
    switch (why) {
    case TlbMmu::RetryReason::OTHER_WALK_ACTIVE: out->perf_itlb_retry_other_walk = true; break;
    case TlbMmu::RetryReason::WALK_REQ_BLOCKED: out->perf_itlb_retry_walk_req_blocked = true; break;
    case TlbMmu::RetryReason::WAIT_WALK_RESP: out->perf_itlb_retry_wait_walk_resp = true; break;
    default: out->perf_itlb_retry_local_walker_busy = true; break;
    }
  }
}

void fill_lookup_meta() {
  auto &d = icache.io.lookup_in;
  d = {};
  d.meta_resp_valid = g_lookup.meta_valid;
  if (!g_lookup.meta_valid) return;
  for (uint32_t w = 0; w < ICACHE_WAY_NUM; ++w) { d.lookup_set_tag[w] = g_lookup.tag[w]; d.lookup_set_valid[w] = g_lookup.valid[w]; }
}
void fill_lookup_data() {
  auto &d = icache.io.lookup_in;
  const auto &o = icache.io.out;
  const uint32_t index = static_cast<uint32_t>(o.lookup_data_req_index), way = static_cast<uint32_t>(o.lookup_data_req_way);
  d.data_resp_valid = false;
  d.data_resp_way = 0;
  if (!o.lookup_data_req_valid || !g_lookup.data_valid || g_lookup.index != index || way >= ICACHE_WAY_NUM) return;
  d.data_resp_valid = true;
  d.data_resp_way = static_cast<uint8_t>(way);
}

// Combinational part of the cycle (module + ITLB + ports).
void comb(icache_in *in, icache_out *out) {
  clear_outputs(out);
  out->fetch_paddr = 0;
  out->fetch_single_word = false;
  out->fetch_page_fault = false;
  auto *rp = g_bind.read;
  if (in->reset) {
    icache.reset();
    if (rp) { rp->req.valid = false; rp->req.addr = 0; rp->req.total_size = 0; rp->req.id = 0; rp->req.bypass = false; rp->req.accepted = false; rp->req.accepted_id = 0; rp->resp.ready = false; }
    if (g_bind.probe_req) { g_bind.probe_req->req_valid = false; g_bind.probe_req->req_addr = 0; }
    if (TlbMmu *m = g_bind.mmu()) m->flush();
    out->icache_read_ready = true;
    return;
  }
  // (a satp write does not flush the ITLB: the architectural shootdown is SFENCE.VMA -> itlb_flush)
  const bool hold_for_recovery = in->itlb_flush || in->fence_i || in->invalidate_req;
  const bool req_valid = in->icache_read_valid;

  auto &hin = icache.io.in;
  hin.refetch = in->refetch || in->invalidate_req || in->fence_i;
  hin.flush = in->fence_i;
  hin.pc = in->fetch_address;
  hin.ifu_req_valid = req_valid;
  hin.ifu_resp_ready = true;
  hin.ppn = 0;
  hin.ppn_valid = false;
  hin.page_fault = false;
  hin.mem_req_ready = rp ? rp->req.ready : false;
  hin.mem_req_accepted = rp ? rp->req.accepted : false;
  hin.mem_req_accepted_id = rp ? static_cast<uint8_t>(rp->req.accepted_id & 0xF) : 0;
  hin.mem_resp_valid = rp ? rp->resp.valid : false;
  hin.mem_resp_id = rp ? static_cast<uint8_t>(rp->resp.id & 0xF) : 0;
  const auto *pr = g_bind.probe_resp;
  hin.dcache_resp_valid = pr ? pr->resp_valid : false;
  hin.dcache_resp_retry = pr ? pr->resp_retry : false;
  hin.dcache_resp_miss = pr ? pr->resp_miss : true;
  hin.dcache_resp_data = pr ? pr->resp_data : 0;

  fill_lookup_meta();
  icache.comb_lookup_meta();

  // ITLB
  bool itlb_invoked = false;
  TlbMmu::Result itlb_ret = TlbMmu::Result::OK;
  TlbMmu::RetryReason why = TlbMmu::RetryReason::NONE;
  if (TlbMmu *m = g_bind.mmu()) {
    if (in->itlb_flush) m->flush();
    if (in->refetch) m->cancel_pending_walk();
    if (!in->refetch && icache.io.out.mmu_req_valid) {
      itlb_invoked = true;
      uint32_t pa = 0;
      itlb_ret = m->translate(pa, static_cast<pc_t>(icache.io.out.mmu_req_vtag) << 12, 0, in->csr_status);
      if (itlb_ret == TlbMmu::Result::OK) { hin.ppn_valid = true; hin.ppn = pa >> 12; }
      else if (itlb_ret == TlbMmu::Result::FAULT) { hin.ppn_valid = true; hin.page_fault = true; }
      why = m->last_retry_reason();
    }
  }

  fill_lookup_meta();
  icache.comb_lookup_meta();
  fill_lookup_data();
  icache.comb_lookup_data();

  const auto &ho = icache.io.out;
  if (rp) {
    rp->req.valid = ho.mem_req_valid;
    rp->req.addr = ho.mem_req_addr;
    rp->req.total_size = static_cast<uint8_t>(ho.mem_req_total_size);
    rp->req.id = static_cast<uint8_t>(ho.mem_req_id & 0xF);
    rp->req.bypass = ho.mem_req_bypass;
    rp->resp.ready = ho.mem_resp_ready;
  }
  if (g_bind.probe_req) { g_bind.probe_req->req_valid = ho.dcache_req_valid; g_bind.probe_req->req_addr = ho.dcache_req_addr; }

  out->icache_read_ready = read_ready(hold_for_recovery, false);
  drive_fetch_group(out);
  drive_perf(out, req_valid, itlb_invoked, itlb_ret, why);
  if (ho.ifu_resp_valid && ho.miss) {
    std::printf("[icache_top] ERROR: miss is true when ifu_resp is valid\n");
    std::exit(1);
  }
}

}  // namespace

void icache_set_context(SimContext *ctx) { g_bind.ctx_set = ctx; }

// Ready as visible before the cycle's comb (the BPU needs it first).
void icache_peek_ready(icache_in *in, icache_out *out) {
  g_bind.bind();
  clear_outputs(out);
  out->icache_read_ready = read_ready(in->itlb_flush || in->fence_i || in->invalidate_req, in->reset);
}

// One cycle: array read, comb, clock edge of the module / ITLB / arrays.
void icache_comb_calc(icache_in *in, icache_out *out) {
  if (in->reset) { g_tags.reset(); g_valid.reset(); }
  g_bind.bind();

  const auto &r = icache.io.regs;
  const pc_t lookup_pc = r.req_valid_r ? r.req_pc_r : r.lookup_pending_r ? r.lookup_pc_r : in->fetch_address;
  const uint32_t index = (lookup_pc >> ICACHE_OFFSET_BITS) & (ICACHE_SET_NUM - 1u);
  TagTable::ReadReq tag_read{};
  ValidTable::ReadReq valid_read{};
  tag_read.enable = !in->reset && !in->refetch &&
                    (ICACHE_LOOKUP_LATENCY == 0 ? (in->icache_read_valid || r.req_valid_r || r.lookup_pending_r) : r.lookup_pending_r);
  valid_read.enable = tag_read.enable;
  tag_read.address = valid_read.address = index;
  TagTable::ReadResp tag_resp{};
  ValidTable::ReadResp valid_resp{};
  g_tags.comb(tag_read, tag_resp);
  g_valid.comb(valid_read, valid_resp);
  g_lookup = {};
  g_lookup.meta_valid = tag_resp.valid && valid_resp.valid;
  g_lookup.data_valid = tag_resp.valid;
  g_lookup.index = index;
  for (uint32_t w = 0; w < ICACHE_WAY_NUM; ++w) { g_lookup.tag[w] = tag_resp.payload.chunks[w][0]; g_lookup.valid[w] = valid_resp.payload.chunks[w][0]; }

  comb(in, out);

  // clock edge
  if (!in->reset) icache.seq();
  if (g_bind.itlb) g_bind.itlb->seq();
  tag_read.enable = r.lookup_pending_r;
  valid_read.enable = tag_read.enable;
  tag_read.address = valid_read.address = r.lookup_index_r;
  TagTable::WriteReq tag_write{};
  ValidTable::WriteReq valid_write{};
  const auto &tw = icache.io.table_write;
  if (tw.we) {
    if (g_icache_fill_hook && tw.valid) g_icache_fill_hook(tw.index, tw.tag);
    tag_write.enable = valid_write.enable = true;
    tag_write.address = valid_write.address = tw.index;
    tag_write.payload.chunks[tw.way][0] = tw.tag;
    valid_write.payload.chunks[tw.way][0] = tw.valid;
    tag_write.chunk_enable[tw.way] = valid_write.chunk_enable[tw.way] = true;
  }
  if (tw.invalidate_all) g_valid.reset();
  g_tags.seq(tag_read, tag_write);
  g_valid.seq(valid_read, valid_write);
}

// FE_PAIR / FE_IPF: functional tag peek (no state change): is the line with this physical tag resident in set `index`?
bool icache_peek_hit(uint32_t index, uint32_t ptag) {
  const auto &tp = g_tags.peek_row(index);
  const auto &vp = g_valid.peek_row(index);
  for (int w = 0; w < ICACHE_WAY_NUM; ++w)
    if (vp.chunks[w][0] && tp.chunks[w][0] == ptag) return true;
  return false;
}

// FE_PAIR: second fetch block of a fetch group read in the same cycle as the first one. Only if it lies in the same 4 KiB page as the
// first block (same translation) and its line is resident; no state is changed. Returns false when it cannot be served.
bool icache_fetch_second(uint32_t a_vpc, uint32_t a_paddr, uint32_t b_vpc, int nwords, uint32_t *words) {
  if ((a_vpc >> 12) != (b_vpc >> 12)) return false;
  const uint32_t pb = (a_paddr & ~0xFFFu) | (b_vpc & 0xFFFu);
  const uint32_t index = (pb >> ICACHE_OFFSET_BITS) & (ICACHE_SET_NUM - 1u);
  if (!icache_peek_hit(index, pb >> 12)) return false;
  for (int i = 0; i < nwords; ++i) words[i] = tbhook::fetch_window_at(pb + SLOT_BYTES * i);
  return true;
}

// FE_IPF: ITLB lookup without side effects.
bool icache_peek_translate(uint32_t v_addr, CsrStatusIO *status, uint32_t &p_addr) {
  return g_bind.itlb && g_bind.itlb->peek_translate(p_addr, v_addr, 0, status);
}

// FE_IPF: install a prefetched line (tag + valid only). An invalid way is used first, else a rotating victim.
bool icache_prefetch_install(uint32_t index, uint32_t ptag) {
  if (icache_peek_hit(index, ptag)) return false;
  static uint8_t rot[ICACHE_SET_NUM] = {};
  const auto &vp = g_valid.peek_row(index);
  int way = -1;
  for (int w = 0; w < ICACHE_WAY_NUM; ++w) if (!vp.chunks[w][0]) { way = w; break; }
  if (way < 0) way = rot[index]++ % ICACHE_WAY_NUM;
  TagTable::WriteReq tw{};
  ValidTable::WriteReq vw{};
  tw.enable = vw.enable = true;
  tw.address = vw.address = index;
  tw.payload.chunks[way][0] = ptag;
  vw.payload.chunks[way][0] = 1;
  tw.chunk_enable[way] = vw.chunk_enable[way] = true;
  g_tags.seq(TagTable::ReadReq{}, tw);
  g_valid.seq(ValidTable::ReadReq{}, vw);
  return true;
}

// Fast-warmup replay (EXP-011): a ready-made L1I tag / valid state and ITLB state.
void icache_warm_install(const uint32_t tag[ICACHE_SET_NUM][ICACHE_WAY_NUM], const bool valid[ICACHE_SET_NUM][ICACHE_WAY_NUM]) {
  for (int set = 0; set < ICACHE_SET_NUM; ++set) {
    TagTable::WriteReq tw{};
    ValidTable::WriteReq vw{};
    tw.enable = vw.enable = true;
    tw.address = vw.address = set;
    for (int w = 0; w < ICACHE_WAY_NUM; ++w) {
      tw.payload.chunks[w][0] = tag[set][w];
      vw.payload.chunks[w][0] = valid[set][w];
      tw.chunk_enable[w] = vw.chunk_enable[w] = true;
    }
    g_tags.seq(TagTable::ReadReq{}, tw);
    g_valid.seq(ValidTable::ReadReq{}, vw);
  }
}
void icache_warm_install_itlb(const void *entries, int count, int repl_ptr) {
  if (TlbMmu *m = g_bind.mmu()) m->warm_install(static_cast<const TlbMmu::TlbEntry *>(entries), count, repl_ptr);
}
