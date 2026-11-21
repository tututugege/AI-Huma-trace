// I-cache top: the tag / valid arrays, the ITLB (TlbMmu), the memory ports and the icache_module state machine, one cycle per
// comb_calc() call (comb and the I-cache's own clock edge: the I-cache steps inside the front end's comb). The module computes the request / miss / refill state; this file only connects it:
//   arrays (register file, read in the cycle of the lookup) -> module meta lookup -> ITLB translation -> module meta + data lookup
//   -> memory read / D-cache probe requests -> fetch-group output (instruction words from the functional memory) -> clock edge.
#include "trace_hook.h"
#include "fatal.h"
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
#include <cstring>

static_assert(ICACHE_LINE_SIZE <= axi_interconnect::MAX_READ_TRANSACTION_BYTES, "ICACHE_LINE_SIZE exceeds the AXI read transaction limit");

namespace {
constexpr int kLineSlots = ICACHE_LINE_SIZE / SLOT_BYTES;
inline void snapshot_icache_line(uint32_t paddr, uint32_t *dst) {
  const uint32_t base = paddr & ~uint32_t(ICACHE_LINE_SIZE - 1u);
  for (uint32_t w = 0; w < ICACHE_WORD_NUM; ++w) dst[w] = tbhook::mem_read(base + 4u * w);
}
inline uint32_t read_line_window(const uint32_t *line, uint32_t byte_off) {
  const uint32_t w = (byte_off & (ICACHE_LINE_SIZE - 1u)) >> 2;
  if ((byte_off & 2u) == 0u) return line[w];
  const uint32_t lo = line[w] >> 16;
  const uint32_t hi = (w + 1u < ICACHE_WORD_NUM) ? (line[w + 1u] & 0xFFFFu) : 0u;
  return lo | (hi << 16);
}
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

void ICacheTop::clear_outputs(ICacheOut *co) {
  co->icache_read_complete = false;
  co->fetch_pc = 0;
  for (int i = 0; i < FETCH_WIDTH; ++i) { co->fetch_group[i] = INST_NOP; co->page_fault_inst[i] = false; co->inst_valid[i] = false; }
  co->perf_req_fire = co->perf_miss_event = co->perf_miss_busy = false;
  co->perf_itlb_hit = co->perf_itlb_miss = co->perf_itlb_fault = false;
}

// The front end may send a request: the module is idle and ready, no recovery (fence.i / sfence / invalidate) in progress.
bool ICacheTop::read_ready(bool hold_for_recovery, bool reset) const {
  if (reset) return true;
  const auto &r = mod_.io.regs;
  const bool busy = r.lookup_pending_r || r.req_valid_r || r.miss_txid_valid_r || r.state != icache_module_n::IDLE ||
                    r.mem_axi_state != icache_module_n::AXI_IDLE;
  return !hold_for_recovery && r.ifu_req_ready_r && !busy && mod_.io.out.ifu_req_ready;
}

// Fetch-group output of the module's response: words come from the L1I data array (matching RTL icache_arr).
void ICacheTop::drive_fetch_group(ICacheOut *co) {
  const auto &o = mod_.io.out;
  if (!o.ifu_resp_valid) return;
  co->icache_read_complete = true;
  co->fetch_pc = o.ifu_resp_pc;
  co->fetch_paddr = o.ifu_resp_paddr;
  co->fetch_single_word = o.ifu_resp_single_word;
  const bool pf = o.ifu_page_fault;
  if (o.ifu_resp_single_word) {   // an uncached single word
    co->fetch_group[0] = pf ? INST_NOP : tbhook::mem_read(o.ifu_resp_paddr);
    co->page_fault_inst[0] = pf;
    co->inst_valid[0] = true;
    if (kRvc) {   // one 4-byte word = two 2-byte slots; the window of the second slot is the word's upper half plus the next word
      co->fetch_group[1] = pf ? INST_NOP : tbhook::fetch_window_at(o.ifu_resp_paddr + SLOT_BYTES);
      co->page_fault_inst[1] = pf;
      co->inst_valid[1] = true;
    }
    return;
  }
  const int base = static_cast<int>((o.ifu_resp_pc & (ICACHE_LINE_SIZE - 1u)) / SLOT_BYTES);
  uint32_t line_buf[ICACHE_WORD_NUM];
  const uint32_t *line_ptr = nullptr;
  if (!pf) {
    if (mod_.io.table_write.we) {
      snapshot_icache_line(o.ifu_resp_paddr, line_buf);
      line_ptr = line_buf;
    } else {
      const uint32_t idx = (o.ifu_resp_paddr >> ICACHE_OFFSET_BITS) & (ICACHE_SET_NUM - 1u);
      const uint32_t ptag = o.ifu_resp_paddr >> 12;
      const int way = peek_way(idx, ptag);
      if (way >= 0) line_ptr = data_[idx][way];
    }
  }
  for (int i = 0; i < FETCH_WIDTH && base + i < kLineSlots; ++i) {
    if (pf) {
      co->fetch_group[i] = INST_NOP;
    } else if (line_ptr != nullptr) {
      co->fetch_group[i] = read_line_window(line_ptr, (o.ifu_resp_paddr + SLOT_BYTES * i) & (ICACHE_LINE_SIZE - 1u));
    } else {
      co->fetch_group[i] = tbhook::fetch_window_at(o.ifu_resp_paddr + SLOT_BYTES * i);
    }
    co->page_fault_inst[i] = pf;
    co->inst_valid[i] = true;
  }
}

void ICacheTop::drive_perf(ICacheOut *co, bool req_valid, bool itlb_invoked, TlbMmu::Result itlb_ret) {
  const auto &h = mod_;
  co->perf_req_fire = req_valid && co->icache_read_ready;
  co->perf_miss_event = h.perf.miss_issue_valid;
  const auto st = static_cast<icache_module_n::ICacheState>(h.io.regs.state);
  co->perf_miss_busy = st == icache_module_n::SWAP_IN || st == icache_module_n::WAIT_DCACHE_AFTER_MEM || st == icache_module_n::DRAIN;
  if (!itlb_invoked) return;
  if (itlb_ret == TlbMmu::Result::OK) co->perf_itlb_hit = true;
  else if (itlb_ret == TlbMmu::Result::FAULT) co->perf_itlb_fault = true;
  else co->perf_itlb_miss = true;
}

void ICacheTop::fill_lookup_meta() {
  auto &d = mod_.io.lookup_in;
  d = {};
  d.meta_resp_valid = lookup_.meta_valid;
  if (!lookup_.meta_valid) return;
  for (uint32_t w = 0; w < ICACHE_WAY_NUM; ++w) { d.lookup_set_tag[w] = lookup_.tag[w]; d.lookup_set_valid[w] = lookup_.valid[w]; }
  d.lookup_set_repl = repl_[lookup_.index & (ICACHE_SET_NUM - 1u)];
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
void ICacheTop::comb(ICacheIn *ci, ICacheOut *co) {
  clear_outputs(co);
  co->fetch_paddr = 0;
  co->fetch_single_word = false;
  if (ci->reset) {   // (the module itself is reset at the edge: icache_seq)
    out.axi->req.valid = false; out.axi->req.addr = 0; out.axi->req.total_size = 0; out.axi->req.id = 0; out.axi->req.bypass = false; out.axi->resp.ready = false;
    out.probe_req->req_valid = false; out.probe_req->req_addr = 0;
    itlb.comb(true, false, nullptr, nullptr, 0);
    co->icache_read_ready = true;
    fdip_dc_req_v_ = false; fdip_dc_req_addr_ = 0;
    return;
  }
  // (a satp write does not flush the ITLB: the architectural shootdown is SFENCE.VMA -> itlb_flush)
  const bool hold_for_recovery = ci->itlb_flush || ci->fence_i || ci->invalidate_req;
  const bool req_valid = ci->icache_read_valid;

  const uint8_t axi_acc_id = static_cast<uint8_t>(in.axi_q->req.accepted_id & 0xF);
  const uint8_t axi_rsp_id = static_cast<uint8_t>(in.axi_q->resp.id & 0xF);
  const bool fdip_acc = in.axi_q->req.accepted && axi_acc_id >= kFdipTxBase && axi_acc_id < kFdipTxBase + kFdipN;
  const bool fdip_rsp_id = in.axi_q->resp.valid && axi_rsp_id >= kFdipTxBase && axi_rsp_id < kFdipTxBase + kFdipN;
  const bool fdip_rsp = fdip_rsp_id && fdip_q_.slots[axi_rsp_id - kFdipTxBase].active &&
                        fdip_q_.slots[axi_rsp_id - kFdipTxBase].issued;
  const bool fdip_busy_req = fdip_q_.sending >= 0 && !fdip_acc;

  auto &hin = mod_.io.in;
  hin.refetch = ci->refetch || ci->invalidate_req || ci->fence_i;
  hin.flush = ci->fence_i;
  hin.pc = ci->fetch_address;
  hin.ifu_req_valid = req_valid;
  hin.ppn = 0;
  hin.ppn_valid = false;
  hin.page_fault = false;
  hin.mem_req_ready = fdip_busy_req ? false : in.axi_q->req.ready;
  hin.mem_req_accepted = fdip_acc ? false : in.axi_q->req.accepted;
  hin.mem_req_accepted_id = axi_acc_id;
  hin.mem_resp_valid = fdip_rsp_id ? false : in.axi_q->resp.valid;
  hin.mem_resp_id = axi_rsp_id;
  hin.dcache_resp_valid = in.probe_resp->resp_valid;
  hin.dcache_resp_retry = in.probe_resp->resp_retry;
  hin.dcache_resp_miss = in.probe_resp->resp_miss;
  hin.dcache_probe_busy = fdip_q_.probe_v;

  fill_lookup_meta();
  mod_.comb_lookup_meta();

  // ITLB
  bool itlb_invoked = false;
  TlbMmu::Result itlb_ret = TlbMmu::Result::OK;
  {   // a fence.i / sfence flushes the ITLB, a refetch drops its walk, else the fetch's request (if any) is translated
    const bool translate = !ci->refetch && mod_.io.out.mmu_req_valid;
    if (translate && edge_.kill_step) { fatal("[icache_top] ERROR: the invalidate step (K) uses the ITLB\n"); }
    if (ci->itlb_flush || ci->refetch || translate) {
      const TlbMmu::Req req{true, static_cast<pc_t>(mod_.io.out.mmu_req_vtag) << 12, 0};
      TlbMmu::Resp resp;
      itlb.comb(ci->itlb_flush, ci->refetch, &req, &resp, translate ? 1 : 0);
      if (translate) {
        itlb_invoked = true;
        itlb_ret = resp.result;
        if (itlb_ret == TlbMmu::Result::OK) { hin.ppn_valid = true; hin.ppn = resp.paddr >> 12; }
        else if (itlb_ret == TlbMmu::Result::FAULT) { hin.ppn_valid = true; hin.page_fault = true; }
      }
    }
  }

  fill_lookup_meta();
  mod_.comb_lookup_meta();
  fill_lookup_data();
  mod_.comb_lookup_data();

  const auto &ho = mod_.io.out;
  out.axi->req.valid = ho.mem_req_valid;
  out.axi->req.addr = ho.mem_req_addr;
  out.axi->req.total_size = static_cast<uint8_t>(ho.mem_req_total_size);
  out.axi->req.id = static_cast<uint8_t>(ho.mem_req_id & 0xF);
  out.axi->req.bypass = ho.mem_req_bypass;
  const bool resp_ready = ho.mem_resp_ready || fdip_rsp;
  out.axi->resp.ready = resp_ready;
  out.probe_req->req_valid = ho.dcache_req_valid;
  out.probe_req->req_addr = ho.dcache_req_addr;

  // Real-AXI/LLC/DRAM S2-confirmed FDIP (2 slots, txid 8..9, 8-entry VA queue, 1-beat DCache probe before AXI)
  if (!edge_.kill_step) {
    fdip_d_ = fdip_q_;
    fdip_d_valid_ = true;
    fdip_inst_en_ = false;
    fdip_dc_req_v_ = false;
    fdip_dc_req_addr_ = 0;
    if (ci->fence_i) {
      fdip_d_.qh = fdip_d_.qn = 0;
      fdip_d_.probe_retry = false;
      if (fdip_d_.probe_v) fdip_d_.probe_cancel = true;
      for (int s = 0; s < kFdipN; s++) {
        if (fdip_d_.slots[s].active) {
          if (fdip_d_.slots[s].issued || fdip_d_.sending == s) fdip_d_.slots[s].canceled = true;
          else fdip_d_.slots[s].active = false;
        }
      }
    }
    if (fdip_acc) {
      const int s = axi_acc_id - kFdipTxBase;
      fdip_d_.slots[s].issued = true;
      if (fdip_d_.sending == s) fdip_d_.sending = -1;
    }
    if (fdip_rsp) {
      const int s = axi_rsp_id - kFdipTxBase;
      if (fdip_d_.slots[s].active && !fdip_d_.slots[s].canceled && !ci->fence_i) {
        fdip_inst_en_ = true;
        fdip_inst_idx_ = fdip_d_.slots[s].index;
        fdip_inst_ptag_ = fdip_d_.slots[s].ptag;
        std::memcpy(fdip_inst_line_, fdip_d_.slots[s].line, sizeof(fdip_inst_line_));
      }
      fdip_d_.slots[s] = FdipSlot{};
      if (fdip_d_.sending == s) fdip_d_.sending = -1;
    }
    const bool demand_dc_busy =
        ((mod_.io.regs.state != icache_module_n::IDLE) && !mod_.io.regs.dcache_probe_done_r) ||
        mod_.io.regs.dcache_probe_inflight_r || ho.dcache_req_valid;
    auto find_free_slot = [&]() {
      for (int s = 0; s < kFdipN; s++) if (!fdip_d_.slots[s].active) return s;
      return -1;
    };
    auto is_dup_pa = [&](uint32_t pa) {
      if (mod_.io.regs.miss_txid_valid_r &&
          ((mod_.io.regs.ppn_r << 12) | (mod_.io.regs.req_index_r << ICACHE_OFFSET_BITS)) == pa)
        return true;
      for (int s = 0; s < kFdipN; s++) if (fdip_d_.slots[s].active && fdip_d_.slots[s].paddr == pa) return true;
      return false;
    };
    // Complete in-flight 1-beat DCache coherency probe (miss in L1D+WB -> safe to issue AXI read)
    if (fdip_d_.probe_v) {
      if (ci->refetch || ci->fence_i) fdip_d_.probe_cancel = true;
      if (in.probe_resp->resp_valid) {
        fdip_d_.probe_v = false;
        const bool canc = fdip_d_.probe_cancel || ci->refetch || ci->fence_i;
        fdip_d_.probe_cancel = false;
        if (!canc) {
          if (in.probe_resp->resp_retry) {
            if (!demand_dc_busy) fdip_d_.probe_retry = true;
          } else if (in.probe_resp->resp_miss) {
            const uint32_t pa = fdip_d_.probe_pa;
            const uint32_t idx = (pa >> ICACHE_OFFSET_BITS) & (ICACHE_SET_NUM - 1u);
            const uint32_t ptag = pa >> 12;
            const int free_s = find_free_slot();
            if (fdip_d_.sending < 0 && !ho.mem_req_valid && free_s >= 0 && !is_dup_pa(pa) && !peek_hit(idx, ptag)) {
              FdipSlot sl{true, false, false, pa, idx, ptag, {}};
              snapshot_icache_line(pa, sl.line);
              fdip_d_.slots[free_s] = sl;
              fdip_d_.sending = free_s;
            }
          }
        }
      }
    }
    // Launch next 1-beat DCache probe when demand does not need the probe port and no FDIP probe is in flight
    if (ci->refetch || ci->fence_i || demand_dc_busy) {
      fdip_d_.probe_retry = false;
    } else if (!fdip_q_.probe_v) {
      if (fdip_d_.probe_retry) {
        fdip_d_.probe_retry = false;
        fdip_d_.probe_v = true;
        fdip_d_.probe_cancel = false;
        fdip_dc_req_v_ = true;
        fdip_dc_req_addr_ = fdip_d_.probe_pa;
      } else if (fdip_d_.sending < 0 && !ho.mem_req_valid && find_free_slot() >= 0 && fdip_d_.qn > 0) {
        const pc_t va = fdip_d_.q[fdip_d_.qh];
        fdip_d_.qh = (fdip_d_.qh + 1) % kFdipQ;
        fdip_d_.qn--;
        uint32_t pa = 0;
        if (peek_translate(va, pa)) {
          pa &= ~uint32_t(ICACHE_LINE_SIZE - 1u);
          if (pa >= 0x80000000u) {
            const uint32_t idx = (pa >> ICACHE_OFFSET_BITS) & (ICACHE_SET_NUM - 1u);
            const uint32_t ptag = pa >> 12;
            if (!is_dup_pa(pa) && !peek_hit(idx, ptag)) {
              fdip_d_.probe_v = true;
              fdip_d_.probe_cancel = false;
              fdip_d_.probe_pa = pa;
              fdip_dc_req_v_ = true;
              fdip_dc_req_addr_ = pa;
            }
          }
        }
      }
    }
  }
  if (fdip_dc_req_v_) {
    out.probe_req->req_valid = true;
    out.probe_req->req_addr = fdip_dc_req_addr_;
  }
  if (fdip_d_.sending >= 0) {
    const FdipSlot &sl = fdip_d_.slots[fdip_d_.sending];
    out.axi->req.valid = true;
    out.axi->req.addr = sl.paddr;
    out.axi->req.total_size = ICACHE_LINE_SIZE - 1u;
    out.axi->req.id = static_cast<uint8_t>(kFdipTxBase + fdip_d_.sending);
    out.axi->req.bypass = false;
  }

  co->icache_read_ready = read_ready(hold_for_recovery, false);
  drive_fetch_group(co);
  drive_perf(co, req_valid, itlb_invoked, itlb_ret);
  if (ho.ifu_resp_valid && ho.miss) {
    fatal("[icache_top] ERROR: miss is true when ifu_resp is valid\n");
  }
}

// One step: the array read (the pending table writes of this cycle forwarded), the module comb, the step's table write.
void ICacheTop::step(ICacheIn *ci, ICacheOut *co) {
  bind_.bind();
  const auto &r = mod_.io.regs;
  const pc_t lookup_pc = r.req_valid_r ? r.req_pc_r : r.lookup_pending_r ? r.lookup_pc_r : ci->fetch_address;
  const uint32_t index = (lookup_pc >> ICACHE_OFFSET_BITS) & (ICACHE_SET_NUM - 1u);
  TagTable::ReadReq tag_read{};
  ValidTable::ReadReq valid_read{};
  tag_read.enable = !ci->reset && !ci->refetch &&
                    (ICACHE_LOOKUP_LATENCY == 0 ? (ci->icache_read_valid || r.req_valid_r || r.lookup_pending_r) : r.lookup_pending_r);
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

  comb(ci, co);
  edge_.tw[edge_.n++] = mod_.io.table_write;   // (a reset cycle does not run the module: its last table write again)
}

// The fetch step of the cycle (R1 = N(R0)): outputs, memory-side requests, the ITLB lookup. Nothing is latched here.
void ICacheTop::comb_calc(ICacheIn *ci, ICacheOut *co) {
  edge_ = {};
  edge_.reset = ci->reset;
  step(ci, co);
}

// A predecode flush invalidates the I-cache's request in flight: R2 = K(R1), the kill step evaluated on this cycle's
// fetch-step result (the module's comb reads io.regs, so R1 is presented there for the evaluation and the registers are
// restored after it; they change only at the edge). Its outputs replace this cycle's; its memory-side requests are the
// cycle's last (RTL: the memory-side outputs are those of the last step).
void ICacheTop::invalidate_step(ICacheOut *co) {
  if (edge_.reset || edge_.n != 1) { fatal("[icache_top] ERROR: invalidate step without a fetch step\n"); }
  ICacheIn inv{};
  inv.invalidate_req = true;
  const icache_module_n::ICacheRegs q = mod_.io.regs;
  mod_.io.regs = mod_.io.reg_write;   // R1
  edge_.kill_step = true;
  step(&inv, co);
  edge_.kill_step = false;
  mod_.io.regs = q;
}

// The clock edge: module registers (R1 or R2, or the reset state), the ITLB, the table writes of the steps in order.
void ICacheTop::seq() {
  if (edge_.n == 0) return;   // (no I-cache step this cycle)
  if (edge_.reset) {
    tags_.reset(); valid_.reset(); mod_.reset(); for (auto &r : repl_) r = 0;
    fdip_q_ = FdipState{};
    fdip_d_ = FdipState{};
    fdip_d_valid_ = fdip_inst_en_ = fdip_pair_miss_en_ = false;
    fdip_dc_req_v_ = false; fdip_dc_req_addr_ = 0;
  } else {
    mod_.seq();
  }
  itlb.seq();   // (the kill step does not use the ITLB: a second seq would be a no-op)
  bool demand_tw_we = false;
  for (int s = 0; s < edge_.n; ++s) {
    const auto &tw = edge_.tw[s];
    TagTable::WriteReq tag_write{};
    ValidTable::WriteReq valid_write{};
    if (tw.we) {
      demand_tw_we = true;
      tag_write.enable = valid_write.enable = true;
      tag_write.address = valid_write.address = tw.index;
      tag_write.payload.chunks[tw.way][0] = tw.tag;
      valid_write.payload.chunks[tw.way][0] = tw.valid;
      tag_write.chunk_enable[tw.way] = valid_write.chunk_enable[tw.way] = true;
      snapshot_icache_line((static_cast<uint32_t>(tw.tag) << 12) | (static_cast<uint32_t>(tw.index) << ICACHE_OFFSET_BITS),
                           data_[tw.index][tw.way]);
    }
    if (tw.invalidate_all) { demand_tw_we = true; valid_.reset(); }
    if (tw.touch_we) {
      uint16_t &st = repl_[static_cast<uint32_t>(tw.touch_index) & (ICACHE_SET_NUM - 1u)];
      st = icache_module_n::icache_repl_update(st, static_cast<uint32_t>(tw.touch_way), tw.touch_fill);
    }
    tags_.seq(TagTable::ReadReq{}, tag_write);
    valid_.seq(ValidTable::ReadReq{}, valid_write);
  }
  if (!edge_.reset) {
    if (fdip_d_valid_) fdip_q_ = fdip_d_;
    if (fdip_inst_en_ && !demand_tw_we) {
      const uint32_t idx = fdip_inst_idx_ & (ICACHE_SET_NUM - 1u);
      if (!peek_hit(idx, fdip_inst_ptag_)) {
        const auto &vp = valid_.peek_row(idx);
        int way = -1;
        for (int w = 0; w < ICACHE_WAY_NUM; ++w) if (!vp.chunks[w][0]) { way = w; break; }
        if (way < 0) {
          way = (icache_module_n::icache_repl_mode() != 0)
                    ? static_cast<int>(icache_module_n::icache_repl_victim(repl_[idx]))
                    : static_cast<int>(prefetch_rot_[idx]++ % ICACHE_WAY_NUM);
        }
        TagTable::WriteReq tw{};
        ValidTable::WriteReq vw{};
        tw.enable = vw.enable = true;
        tw.address = vw.address = idx;
        tw.payload.chunks[way][0] = fdip_inst_ptag_;
        vw.payload.chunks[way][0] = 1;
        tw.chunk_enable[way] = vw.chunk_enable[way] = true;
        tags_.seq(TagTable::ReadReq{}, tw);
        valid_.seq(ValidTable::ReadReq{}, vw);
        std::memcpy(data_[idx][way], fdip_inst_line_, sizeof(fdip_inst_line_));
        if (icache_module_n::icache_repl_mode() != 0) {
          repl_[idx] = icache_module_n::icache_repl_update(repl_[idx], static_cast<uint32_t>(way), true);
        }
      }
    }
    if (fdip_pair_miss_en_) enqueue_fdip(&fdip_pair_miss_va_, 1, false);
  }
  fdip_d_valid_ = fdip_inst_en_ = fdip_pair_miss_en_ = false;
  edge_ = {};
}

// FE_PAIR / FE_IPF: functional tag peek (no state change): is the line with this physical tag resident in set `index`?
int ICacheTop::peek_way(uint32_t index, uint32_t ptag) {
  auto tp = tags_.peek_row(index);
  auto vp = valid_.peek_row(index);
  forward_pending(index, tp, vp);   // (this cycle's table writes, before the edge)
  for (int w = 0; w < ICACHE_WAY_NUM; ++w)
    if (vp.chunks[w][0] && tp.chunks[w][0] == ptag) return w;
  return -1;
}
bool ICacheTop::peek_hit(uint32_t index, uint32_t ptag) { return peek_way(index, ptag) >= 0; }

// 2-taken second fetch block read in parallel with block A (2-port ITLB + 4-bank 1R1W ICache).
bool ICacheTop::fetch_second(pc_t a_vpc, pc_t a_paddr, pc_t b_vpc, int nwords, uint32_t *words, int &actual_n) {
  actual_n = 0;
  uint32_t pb = 0;
  if ((a_vpc >> 12) == (b_vpc >> 12)) {
    pb = (static_cast<uint32_t>(a_paddr) & ~0xFFFu) | static_cast<uint32_t>(b_vpc & 0xFFFu);
  } else {
    if (!peek_translate(b_vpc, pb)) return false;
  }
  if (pb < 0x80000000u) return false;
  const uint32_t idx0 = (pb >> ICACHE_OFFSET_BITS) & (ICACHE_SET_NUM - 1u);
  const uint32_t ptag0 = pb >> 12;
  const int way0 = peek_way(idx0, ptag0);
  if (way0 < 0) {
    if (!edge_.kill_step) {
      fdip_pair_miss_en_ = true;
      fdip_pair_miss_va_ = b_vpc & ~pc_t(ICACHE_LINE_SIZE - 1u);
    }
    return false;
  }
  const int b_slot0 = static_cast<int>((b_vpc & (ICACHE_LINE_SIZE - 1u)) / SLOT_BYTES);
  int avail = kLineSlots - b_slot0;
  if (avail > nwords) avail = nwords;
  actual_n = avail;
  const bool same_line = (pb >> ICACHE_OFFSET_BITS) == (static_cast<uint32_t>(a_paddr) >> ICACHE_OFFSET_BITS);
  uint32_t line_buf[ICACHE_WORD_NUM];
  const uint32_t *line_ptr = data_[idx0][way0];
  if (same_line && mod_.io.table_write.we) {
    snapshot_icache_line(pb, line_buf);
    line_ptr = line_buf;
  }
  for (int i = 0; i < avail; ++i) words[i] = read_line_window(line_ptr, (pb + SLOT_BYTES * i) & (ICACHE_LINE_SIZE - 1u));
  return avail > 0;
}

void ICacheTop::enqueue_fdip(const pc_t *cand, int n, bool clear_all) {
  if (clear_all) { fdip_q_.qh = fdip_q_.qn = 0; }
  for (int i = 0; i < n; i++) {
    const pc_t lva = cand[i] & ~pc_t(ICACHE_LINE_SIZE - 1u);
    bool dup = false;
    for (int k = 0; k < fdip_q_.qn; k++) {
      if (fdip_q_.q[(fdip_q_.qh + k) % kFdipQ] == lva) { dup = true; break; }
    }
    if (dup) continue;
    if (fdip_q_.qn >= kFdipQ) {
      fdip_q_.qh = (fdip_q_.qh + 1) % kFdipQ;
      fdip_q_.qn--;
    }
    fdip_q_.q[(fdip_q_.qh + fdip_q_.qn) % kFdipQ] = lva;
    fdip_q_.qn++;
  }
}

// ITLB lookup without side effects (read-only CAM peek).
bool ICacheTop::peek_translate(pc_t v_addr, uint32_t &p_addr) {
  return itlb.peek_translate(p_addr, v_addr, 0);
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
      if (valid[set][w]) snapshot_icache_line((tag[set][w] << 12) | (static_cast<uint32_t>(set) << ICACHE_OFFSET_BITS), data_[set][w]);
    }
    tags_.seq(TagTable::ReadReq{}, tw);
    valid_.seq(ValidTable::ReadReq{}, vw);
  }
}
void ICacheTop::warm_install_itlb(const void *entries, int count, int repl_ptr) {
  itlb.warm_install(static_cast<const TlbMmu::TlbEntry *>(entries), count, repl_ptr);
}
