// I-cache top: the tag / valid arrays, the ITLB (TlbMmu), the memory ports and the icache_module state machine, one cycle per
// comb_calc() call (comb and the I-cache's own clock edge: the I-cache steps inside the front end's comb). The module computes the request / miss / refill state; this file only connects it:
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
#include "icache_top.h"
#include <cstdio>
#include <cstdlib>

static_assert(ICACHE_LINE_SIZE <= axi_interconnect::MAX_READ_TRANSACTION_BYTES, "ICACHE_LINE_SIZE exceeds the AXI read transaction limit");

namespace {
constexpr int kLineSlots = ICACHE_LINE_SIZE / SLOT_BYTES;
}  // namespace

template <class TagP, class ValidP>
void ICacheTop::forward_pending(uint32_t index, TagP &tag, ValidP &valid) const {
  if (edge_.reset) { tag = TagP{}; valid = ValidP{}; }
  for (int s = 0; s < edge_.n; ++s) {
    const auto &tw = edge_.tw[s];
    if (tw.invalidate_all) valid = ValidP{};
    if (tw.we && tw.index == index) { tag.chunks[tw.way][0] = tw.tag; valid.chunks[tw.way][0] = tw.valid; }
  }
}

void ICacheTop::clear_outputs(icache_out *out) {
  out->icache_read_complete = false;
  out->fetch_pc = 0;
  for (int i = 0; i < FETCH_WIDTH; ++i) { out->fetch_group[i] = INST_NOP; out->page_fault_inst[i] = false; out->inst_valid[i] = false; }
  out->perf_req_fire = out->perf_miss_event = out->perf_miss_busy = false;
  out->perf_itlb_hit = out->perf_itlb_miss = out->perf_itlb_fault = false;
}

// The front end may send a request: the module is idle and ready, no recovery (fence.i / sfence / invalidate) in progress.
bool ICacheTop::read_ready(bool hold_for_recovery, bool reset) const {
  if (reset) return true;
  const auto &r = mod_.io.regs;
  const bool busy = r.lookup_pending_r || r.req_valid_r || r.miss_txid_valid_r || r.state != icache_module_n::IDLE ||
                    r.mem_axi_state != icache_module_n::AXI_IDLE;
  return !hold_for_recovery && r.ifu_req_ready_r && !busy && mod_.io.out.ifu_req_ready;
}

// Fetch-group output of the module's response: the words come from the functional memory (the I-cache keeps no data array).
void ICacheTop::drive_fetch_group(icache_out *out) {
  const auto &o = mod_.io.out;
  if (!o.ifu_resp_valid) return;
  out->icache_read_complete = true;
  out->fetch_pc = o.ifu_resp_pc;
  out->fetch_paddr = o.ifu_resp_paddr;
  out->fetch_single_word = o.ifu_resp_single_word;
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

void ICacheTop::drive_perf(icache_out *out, bool req_valid, bool itlb_invoked, TlbMmu::Result itlb_ret) {
  const auto &h = mod_;
  out->perf_req_fire = req_valid && out->icache_read_ready;
  out->perf_miss_event = h.perf.miss_issue_valid;
  const auto st = static_cast<icache_module_n::ICacheState>(h.io.regs.state);
  out->perf_miss_busy = st == icache_module_n::SWAP_IN || st == icache_module_n::WAIT_DCACHE_AFTER_MEM || st == icache_module_n::DRAIN;
  if (!itlb_invoked) return;
  if (itlb_ret == TlbMmu::Result::OK) out->perf_itlb_hit = true;
  else if (itlb_ret == TlbMmu::Result::FAULT) out->perf_itlb_fault = true;
  else out->perf_itlb_miss = true;
}

void ICacheTop::fill_lookup_meta() {
  auto &d = mod_.io.lookup_in;
  d = {};
  d.meta_resp_valid = lookup_.meta_valid;
  if (!lookup_.meta_valid) return;
  for (uint32_t w = 0; w < ICACHE_WAY_NUM; ++w) { d.lookup_set_tag[w] = lookup_.tag[w]; d.lookup_set_valid[w] = lookup_.valid[w]; }
}
void ICacheTop::fill_lookup_data() {
  auto &d = mod_.io.lookup_in;
  const auto &o = mod_.io.out;
  const uint32_t index = static_cast<uint32_t>(o.lookup_data_req_index), way = static_cast<uint32_t>(o.lookup_data_req_way);
  d.data_resp_valid = false;
  d.data_resp_way = 0;
  if (!o.lookup_data_req_valid || !lookup_.data_valid || lookup_.index != index || way >= ICACHE_WAY_NUM) return;
  d.data_resp_valid = true;
  d.data_resp_way = static_cast<uint8_t>(way);
}

// Combinational part of the cycle (module + ITLB + ports).
void ICacheTop::comb(icache_in *in, icache_out *out) {
  clear_outputs(out);
  out->fetch_paddr = 0;
  out->fetch_single_word = false;
  auto *rp = bind_.read;
  if (in->reset) {   // (the module itself is reset at the edge: icache_seq)
    if (rp) { rp->req.valid = false; rp->req.addr = 0; rp->req.total_size = 0; rp->req.id = 0; rp->req.bypass = false; rp->resp.ready = false; }
    if (bind_.probe_req) { bind_.probe_req->req_valid = false; bind_.probe_req->req_addr = 0; }
    if (TlbMmu *m = bind_.mmu()) m->flush();
    out->icache_read_ready = true;
    return;
  }
  // (a satp write does not flush the ITLB: the architectural shootdown is SFENCE.VMA -> itlb_flush)
  const bool hold_for_recovery = in->itlb_flush || in->fence_i || in->invalidate_req;
  const bool req_valid = in->icache_read_valid;

  auto &hin = mod_.io.in;
  hin.refetch = in->refetch || in->invalidate_req || in->fence_i;
  hin.flush = in->fence_i;
  hin.pc = in->fetch_address;
  hin.ifu_req_valid = req_valid;
  hin.ppn = 0;
  hin.ppn_valid = false;
  hin.page_fault = false;
  const auto *rq = bind_.read_q;
  hin.mem_req_ready = rq ? rq->req.ready : false;
  hin.mem_req_accepted = rq ? rq->req.accepted : false;
  hin.mem_req_accepted_id = rq ? static_cast<uint8_t>(rq->req.accepted_id & 0xF) : 0;
  hin.mem_resp_valid = rq ? rq->resp.valid : false;
  hin.mem_resp_id = rq ? static_cast<uint8_t>(rq->resp.id & 0xF) : 0;
  const auto *pr = bind_.probe_resp;
  hin.dcache_resp_valid = pr ? pr->resp_valid : false;
  hin.dcache_resp_retry = pr ? pr->resp_retry : false;
  hin.dcache_resp_miss = pr ? pr->resp_miss : true;

  fill_lookup_meta();
  mod_.comb_lookup_meta();

  // ITLB
  bool itlb_invoked = false;
  TlbMmu::Result itlb_ret = TlbMmu::Result::OK;
  if (TlbMmu *m = bind_.mmu()) {
    if (in->itlb_flush) m->flush();
    if (in->refetch) m->cancel_pending_walk();
    if (!in->refetch && mod_.io.out.mmu_req_valid) {
      if (edge_.kill_step) { std::printf("[icache_top] ERROR: the invalidate step (K) uses the ITLB\n"); std::exit(1); }
      itlb_invoked = true;
      uint32_t pa = 0;
      itlb_ret = m->translate(pa, static_cast<pc_t>(mod_.io.out.mmu_req_vtag) << 12, 0, in->csr_status);
      if (itlb_ret == TlbMmu::Result::OK) { hin.ppn_valid = true; hin.ppn = pa >> 12; }
      else if (itlb_ret == TlbMmu::Result::FAULT) { hin.ppn_valid = true; hin.page_fault = true; }
    }
  }

  fill_lookup_meta();
  mod_.comb_lookup_meta();
  fill_lookup_data();
  mod_.comb_lookup_data();

  const auto &ho = mod_.io.out;
  if (rp) {
    rp->req.valid = ho.mem_req_valid;
    rp->req.addr = ho.mem_req_addr;
    rp->req.total_size = static_cast<uint8_t>(ho.mem_req_total_size);
    rp->req.id = static_cast<uint8_t>(ho.mem_req_id & 0xF);
    rp->req.bypass = ho.mem_req_bypass;
    rp->resp.ready = ho.mem_resp_ready;
  }
  if (bind_.probe_req) { bind_.probe_req->req_valid = ho.dcache_req_valid; bind_.probe_req->req_addr = ho.dcache_req_addr; }

  out->icache_read_ready = read_ready(hold_for_recovery, false);
  drive_fetch_group(out);
  drive_perf(out, req_valid, itlb_invoked, itlb_ret);
  if (ho.ifu_resp_valid && ho.miss) {
    std::printf("[icache_top] ERROR: miss is true when ifu_resp is valid\n");
    std::exit(1);
  }
}

// One step: the array read (the pending table writes of this cycle forwarded), the module comb, the step's table write.
void ICacheTop::step(icache_in *in, icache_out *out) {
  bind_.bind(ports_set_);
  const auto &r = mod_.io.regs;
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
  tags_.comb(tag_read, tag_resp);
  valid_.comb(valid_read, valid_resp);
  if (tag_resp.valid) forward_pending(index, tag_resp.payload, valid_resp.payload);
  lookup_ = {};
  lookup_.meta_valid = tag_resp.valid && valid_resp.valid;
  lookup_.data_valid = tag_resp.valid;
  lookup_.index = index;
  for (uint32_t w = 0; w < ICACHE_WAY_NUM; ++w) { lookup_.tag[w] = tag_resp.payload.chunks[w][0]; lookup_.valid[w] = valid_resp.payload.chunks[w][0]; }

  comb(in, out);
  edge_.tw[edge_.n++] = mod_.io.table_write;   // (a reset cycle does not run the module: its last table write again)
}

// The fetch step of the cycle (R1 = N(R0)): outputs, memory-side requests, the ITLB lookup. Nothing is latched here.
void ICacheTop::comb_calc(icache_in *in, icache_out *out) {
  edge_ = {};
  edge_.reset = in->reset;
  step(in, out);
}

// A predecode flush invalidates the I-cache's request in flight: R2 = K(R1), the kill step evaluated on this cycle's
// fetch-step result (the module's comb reads io.regs, so R1 is presented there for the evaluation and the registers are
// restored after it; they change only at the edge). Its outputs replace this cycle's; its memory-side requests are the
// cycle's last (RTL: the memory-side outputs are those of the last step).
void ICacheTop::invalidate_step(CsrStatusIO *csr_status, icache_out *out) {
  if (edge_.reset || edge_.n != 1) { std::printf("[icache_top] ERROR: invalidate step without a fetch step\n"); std::exit(1); }
  icache_in inv{};
  inv.invalidate_req = true;
  inv.csr_status = csr_status;
  const icache_module_n::ICache_regs_t q = mod_.io.regs;
  mod_.io.regs = mod_.io.reg_write;   // R1
  edge_.kill_step = true;
  step(&inv, out);
  edge_.kill_step = false;
  mod_.io.regs = q;
}

// The clock edge: module registers (R1 or R2, or the reset state), the ITLB, the table writes of the steps in order.
void ICacheTop::seq() {
  if (edge_.n == 0) return;   // (no I-cache step this cycle)
  if (edge_.reset) { tags_.reset(); valid_.reset(); mod_.reset(); }
  else mod_.seq();
  if (bind_.itlb) bind_.itlb->seq();   // (the kill step does not use the ITLB: a second seq would be a no-op)
  for (int s = 0; s < edge_.n; ++s) {
    const auto &tw = edge_.tw[s];
    TagTable::WriteReq tag_write{};
    ValidTable::WriteReq valid_write{};
    if (tw.we) {
      if (fill_hook && tw.valid) fill_hook(tw.index, tw.tag);
      tag_write.enable = valid_write.enable = true;
      tag_write.address = valid_write.address = tw.index;
      tag_write.payload.chunks[tw.way][0] = tw.tag;
      valid_write.payload.chunks[tw.way][0] = tw.valid;
      tag_write.chunk_enable[tw.way] = valid_write.chunk_enable[tw.way] = true;
    }
    if (tw.invalidate_all) valid_.reset();
    tags_.seq(TagTable::ReadReq{}, tag_write);
    valid_.seq(ValidTable::ReadReq{}, valid_write);
  }
  edge_ = {};
}

// FE_PAIR / FE_IPF: functional tag peek (no state change): is the line with this physical tag resident in set `index`?
bool ICacheTop::peek_hit(uint32_t index, uint32_t ptag) {
  auto tp = tags_.peek_row(index);
  auto vp = valid_.peek_row(index);
  forward_pending(index, tp, vp);   // (this cycle's table writes, before the edge)
  for (int w = 0; w < ICACHE_WAY_NUM; ++w)
    if (vp.chunks[w][0] && tp.chunks[w][0] == ptag) return true;
  return false;
}

// FE_PAIR: second fetch block of a fetch group read in the same cycle as the first one. Only if it lies in the same 4 KiB page as the
// first block (same translation) and its line is resident; no state is changed. Returns false when it cannot be served.
bool ICacheTop::fetch_second(uint32_t a_vpc, uint32_t a_paddr, uint32_t b_vpc, int nwords, uint32_t *words) {
  if ((a_vpc >> 12) != (b_vpc >> 12)) return false;
  const uint32_t pb = (a_paddr & ~0xFFFu) | (b_vpc & 0xFFFu);
  const uint32_t index = (pb >> ICACHE_OFFSET_BITS) & (ICACHE_SET_NUM - 1u);
  if (!peek_hit(index, pb >> 12)) return false;
  for (int i = 0; i < nwords; ++i) words[i] = tbhook::fetch_window_at(pb + SLOT_BYTES * i);
  return true;
}

// FE_IPF: ITLB lookup without side effects.
bool ICacheTop::peek_translate(uint32_t v_addr, CsrStatusIO *status, uint32_t &p_addr) {
  return bind_.itlb && bind_.itlb->peek_translate(p_addr, v_addr, 0, status);
}

// FE_IPF: install a prefetched line (tag + valid only). An invalid way is used first, else a rotating victim.
bool ICacheTop::prefetch_install(uint32_t index, uint32_t ptag) {
  if (peek_hit(index, ptag)) return false;
  const auto &vp = valid_.peek_row(index);
  int way = -1;
  for (int w = 0; w < ICACHE_WAY_NUM; ++w) if (!vp.chunks[w][0]) { way = w; break; }
  if (way < 0) way = prefetch_rot_[index]++ % ICACHE_WAY_NUM;
  TagTable::WriteReq tw{};
  ValidTable::WriteReq vw{};
  tw.enable = vw.enable = true;
  tw.address = vw.address = index;
  tw.payload.chunks[way][0] = ptag;
  vw.payload.chunks[way][0] = 1;
  tw.chunk_enable[way] = vw.chunk_enable[way] = true;
  tags_.seq(TagTable::ReadReq{}, tw);
  valid_.seq(ValidTable::ReadReq{}, vw);
  return true;
}

// Fast-warmup replay (EXP-011): a ready-made L1I tag / valid state and ITLB state.
void ICacheTop::warm_install(const uint32_t tag[ICACHE_SET_NUM][ICACHE_WAY_NUM], const bool valid[ICACHE_SET_NUM][ICACHE_WAY_NUM]) {
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
    tags_.seq(TagTable::ReadReq{}, tw);
    valid_.seq(ValidTable::ReadReq{}, vw);
  }
}
void ICacheTop::warm_install_itlb(const void *entries, int count, int repl_ptr) {
  if (TlbMmu *m = bind_.mmu()) m->warm_install(static_cast<const TlbMmu::TlbEntry *>(entries), count, repl_ptr);
}
