// The staged LSU: writeback phase functions (see RealLsuPipe.cpp, docs/LSU_PIPE.md).
#include "LsuPipeInternal.h"
#include "trace_win.h"

// ---------------------------------------------------------------------------------------------------------------
// ROB commit: STQ entries become committed in order
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::stq_commit() {
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    if (!in.rob_commit->commit_entry[i].valid) continue;
    const auto &u = in.rob_commit->commit_entry[i].uop;
    if (!is_store(u)) continue;
    const int idx = u->stq_idx;
    if (idx != pnxt.stq_commit || !pnxt.stq[idx].wbd) continue;
    PStq &s = pnxt.stq[idx];
    s.cmt = 1;
    if (static_cast<bool>(in.rob_bcast->flush_now) || (s.lrsc && !s.sc_pass)) {   // (the flush of this cycle's commit)
      s.suppress = 1;  // flushed or failed SC: retire without writing
      s.done = 1;
    }
    pnxt.stq_commit = (pnxt.stq_commit + 1) % STQ_SIZE;
    if (pnxt.stq_ccount < STQ_SIZE) pnxt.stq_ccount++;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// MMIO
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::mmio_in() {
  const LsuState &cur = cur_;
  if (!(mmio_resp_q_.is_mmio && cur.uncached_unit.valid)) return;
  const uint32_t idx = cur.uncached_unit.idx;
  if (cur.uncached_unit.is_load) {
    PLdq &e = pnxt.ldq[idx];
    if (e.st == L_MMR) {
      e.result = mmio_resp_q_.mmio_rdata;
      e.st = L_WBP;
      pw.mmio_ld = idx;
    }
  } else if (pnxt.stq[idx].mmw) {
    pnxt.stq[idx].mmw = 0;
    pnxt.stq[idx].done = 1;
  }
  nxt.uncached_unit.valid = false;
}

// ---------------------------------------------------------------------------------------------------------------
// L2: DCache response + STLF confirm + hit/forward select. Reads the L1/L2 registers and the registered STQ state.
// A load that cannot complete is parked in the LDQ entry (cause + wake condition), nothing is queued anywhere.
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::dcache2lsu() {
  dcache2lsu_ldq();
  dcache2lsu_stq();
}

// a load waits for a replay: its cause, the STQ entry it waits for, the countdown
static void park(PLdq &e, uint8_t cause, uint32_t blk, uint32_t wait) {
  e.st = L_RPW;
  e.rcause = cause;
  e.blk_idx = blk;
  e.rwait = wait;
}

void RealLsu::dcache2lsu_ldq() {
  const DcacheLsuIO &dc = in.top_in->dcache2lsu;
  pw.fill = dc.mshr_fill && LSU_WAIT_MSHR_FILL;
  pw.fill_addr = dc.mshr_fill_addr;
  l2_decide();
  dcache_load_responses();
  l3_register();
}

// L2 of each load port: page fault, DTLB miss, MMIO, a port taken by a hidden retry, the DCache read, or the STLF confirm against the candidate store
void RealLsu::l2_decide() {
  const lp::LsuPipeState &pcur = pcur_;
  for (int i = 0; i < LD; i++) {
    const PL12 &sl = pcur.l12[i];
    if (!sl.v) continue;
    PLdq &e = pnxt.ldq[sl.ldq];
    if (!(e.st == L_PIPE && e.epoch == sl.epoch)) continue;  // killed after L1
    if (sl.tlb == 2) {  // page fault: reported through the load write-back
      pw.wbd_v[i] = true;
      pw.wbd_ldq[i] = sl.ldq;
      continue;
    }
    if (sl.tlb == 1) {
      park(e, RC_TLB, 0, 0);
      continue;
    }
    if (sl.mmio) {
      e.st = L_MMW;
      continue;
    }
    if (!sl.cand_v && sl.dcblk) {  // not launched (hidden retry owns the port): replay like a bank conflict
      park(e, RC_DC, 0, replay_wait(ReplayType::CONFLICT));
      continue;
    }
    if (!sl.cand_v) {  // no conflicting older store: the DCache read of L1 decides
      e.st = L_DCW;
      wd_arm(e.wd_t);
      continue;
    }
    // STLF confirm against the registered STQ entry (full physical address check)
    const uint32_t j = sl.cand;
    const PStq &s = pcur.stq[j];
    const uint32_t rank = stq_rank(j, pcur.stq_head);
    switch (stlf_confirm(e, s, rank < pcur.stq_count)) {
    case CF_REPLAY: e.st = L_RDY; break;  // false partial match / candidate gone: replay with the full compare
    case CF_NOADDR: park(e, RC_ADDR, j, 0); break;
    case CF_HOLD: park(e, RC_HOLD, j, 0); break;
    case CF_DATA: park(e, RC_DATA, j, 0); break;
    case CF_FWD:  // hit/forward select picks the STQ data
      pw.wbd_v[i] = true;
      pw.wbd_ldq[i] = sl.ldq;
      break;
    }
  }
}

// the DCache's load responses (same port as the request, also late bank-conflict retries): a hit completes, a replay parks the load
void RealLsu::dcache_load_responses() {
  const DcacheLsuIO &dc = in.top_in->dcache2lsu;
  for (int i = 0; i < LD; i++) {  // DCache responses (same port as the request, also late bank-conflict retries)
    const auto &r = dc.resp_ports.load_resps[i];
    if (!r.valid || (r.req_id & (1u << 31))) continue;
    const uint32_t idx = r.req_id & kLsuReqIdIdxMask;
    if (idx >= LDQ_SIZE) continue;
    const uint32_t ep = (r.req_id >> LDQ_IDX_WIDTH) & ((1u << EPOCH_W) - 1);
    PLdq &e = pnxt.ldq[idx];
    if (!(e.st == L_DCW && e.epoch == ep)) continue;   // request of a load that forwarded / replayed / was killed
    e.rtype = (uint8_t)r.replay;
    if (r.replay == ReplayType::HIT) {
      if (pw.wbd_v[i]) {
        e.st = L_WBP;
      } else {
        pw.wbd_v[i] = true;
        pw.wbd_ldq[i] = idx;
      }
    } else {
      e.miss = 1;
      park(e, RC_DC, 0, replay_wait(r.replay));
    }
  }
}

// a: the direct completions go through the L3 register (written back in the next cycle)
void RealLsu::l3_register() {
  // a: the direct completions go through the L3 register (written back in the next cycle)
  for (int i = 0; i < LD; i++) {
    if (!pw.wbd_v[i]) continue;
    pnxt.l3[i].v = 1;
    pnxt.l3[i].ldq = pw.wbd_ldq[i];
    pnxt.ldq[pw.wbd_ldq[i]].st = L_WB3;
    pw.wbd_v[i] = false;
  }
}


void RealLsu::dcache2lsu_stq() {
  const DcacheLsuIO &dc = in.top_in->dcache2lsu;
  for (int i = 0; i < LSU_STA_COUNT; i++) {
    const auto &r = dc.resp_ports.store_resps[i];
    if (!r.valid || r.req_id >= STQ_SIZE) continue;
    PStq &s = pnxt.stq[r.req_id];
    if (!s.iss) continue;
    s.iss = 0;
    s.rtype = (uint8_t)r.replay;
    if (r.replay == ReplayType::HIT) {
      s.done = 1;
      s.rwait = 0;
    } else {
      s.rwait = replay_wait(r.replay);
    }
    wb_response(pnxt, r.req_id, r.replay == ReplayType::HIT);
  }
  {  // stores merged into a write-buffer entry complete with their leader's response
    for (int i = 0; i < LSU_STA_COUNT; i++) {
      const auto &r = dc.resp_ports.store_resps[i];
      if (!r.valid || r.req_id >= STQ_SIZE) continue;
      for (uint32_t k = 0; k < pnxt.stq_count; k++) {
        PStq &f = pnxt.stq[(pnxt.stq_head + k) % STQ_SIZE];
        if (!f.iss || !f.mf || f.ml != r.req_id) continue;
        f.iss = 0;
        f.mf = 0;
        f.rtype = (uint8_t)r.replay;
        if (r.replay == ReplayType::HIT) {
          f.done = 1;
          f.rwait = 0;
        } else {
          f.rwait = replay_wait(r.replay);
        }
      }
    }
  }
  if (pw.fill || (dc.mshr_fill && LSU_WAIT_MSHR_FILL)) {
    uint32_t full = 0;
    const uint32_t n = std::min<uint32_t>(pnxt.stq_ccount, pnxt.stq_count);
    for (uint32_t k = 0; k < n; k++) {  // MSHR fill wake of waiting stores
      PStq &s = pnxt.stq[(pnxt.stq_head + k) % STQ_SIZE];
      if (!s.cmt || s.done || s.iss || s.rwait == 0) continue;
      if (s.rtype == (uint8_t)ReplayType::MSHR_HIT && cache_line_match(s.paddr, dc.mshr_fill_addr)) {
        s.rwait = 0;
      } else if (s.rtype == (uint8_t)ReplayType::MSHR_FULL && full < (uint32_t)LSU_STA_COUNT) {
        pw.st_fullwake |= (stq_mask_t)1 << ((pnxt.stq_head + k) % STQ_SIZE);
        pw.st_rwpre[(pnxt.stq_head + k) % STQ_SIZE] = s.rwait;
        s.rwait = 0;
        full++;
      }
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Write-back ports
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::lp_emit_load_wb(int port, int idx, bool from_l3) {
  PLdq &l = pnxt.ldq[idx];
  MicroOp wb;
  wb.id = l.id;
  wb.op = UOP_LOAD;
  wb.br_mask = l.br_mask;
  wb.diag_val = l.vaddr;
  wb.result = l.pf ? l.vaddr : l.result;
  wb.page_fault_load = l.pf;
  wb.illegal_inst = l.illegal;
  wb.dest_en = !l.pf && l.uop().dest_preg != 0;  // loads to x0 still complete without a destination
  wb.is_mmio = l.mmio;
  const bool second = l.second;
  const bool defer = !second && l.spec && !l.pf && !l.mmio && !l.lrsc;
  bool early = second;
  if (defer) wb.rob_defer = true;
  if (second) wb.flush_pipe = l.vio;
  if (!early && from_l3 && !l.pf && l.uop().dest_preg != 0 && !in.rob_bcast->flush &&
      !(in.dec_bcast->mispred && (l.br_mask & in.dec_bcast->br_mask) != 0)) {  // a2: wake from the L3 register
    out.lsu_awake.wake[LD + port].valid = true;
    out.lsu_awake.wake[LD + port].preg = l.uop().dest_preg;
    early = true;
  }
  wb.early_woken = early;
  out.lsu2exe.wb_req[port].valid = true;
  out.lsu2exe.wb_req[port].uop = wb;
  if (l.lrsc && !l.pf) {
    LrScUnit &r = nxt.lrsc_unit;
    r.reserve_valid = true;
    r.reserve_addr = l.paddr;
    r.reserve_br_mask = l.br_mask;
  }
  ms_load_written_back(idx, defer);
  if (defer) {  // keep the entry until the older store addresses are known (then the completion write-back, or a re-execution)
    if (l.vio) {
      l.second = 1;
      l.st = L_RDY;
    } else {
      l.st = L_SPD;
    }
  } else {
    l.st = L_EMPTY;
  }
}

void RealLsu::lsu2exe() {
  const lp::LsuPipeState &pcur = pcur_;
  const LsuState &cur = cur_;
  out.lsu2exe = {};
  bool busy[LD] = {};
  for (int p = 0; p < LD; p++) {  // 1. the completions registered in L3 own their load port
    if (pcur.l3[p].v && pnxt.ldq[pcur.l3[p].ldq].st == L_WB3) {
      busy[p] = true;
      lp_emit_load_wb(p, pcur.l3[p].ldq, true);
    }
  }

  // 2. STA completions (age order from the head, first STA_COUNT; SC through a free load port)
  int sta_n = 0, sc_n = 0;
  for (uint32_t rank = 0; rank < pnxt.stq_count; rank++) {
    const uint32_t j = (pnxt.stq_head + rank) % STQ_SIZE;
    PStq &s = pnxt.stq[j];
    if (!s.wbp) continue;
    MicroOp wb;
    wb.id = s.id;
    wb.br_mask = s.br_mask;
    wb.diag_val = s.vaddr;
    wb.page_fault_store = s.pf;
    wb.dest_en = false;
    wb.is_mmio = s.mmio;
    if (s.lrsc) {  // SC reports pass/fail through a load write-back port
      if (sc_n >= 1) continue;  // S: one SC completion per cycle
      int p = -1;
      for (int q = 0; q < LD && p < 0; q++)
        if (!busy[q]) p = q;
      if (p < 0) continue;
      wb.op = UOP_LOAD;
      if (s.pf) {
        wb.result = s.vaddr;
      } else {
        const bool pass = cur.lrsc_unit.reserve_valid && cur.lrsc_unit.reserve_addr == s.paddr;
        s.sc_pass = pass;
        s.suppress = !pass;
        nxt.lrsc_unit.reserve_valid = false;
        wb.result = pass ? 0 : 1;
        wb.dest_en = s.dest_preg != 0;
        pw.ev_sc |= (stq_mask_t)1 << j;
      }
      busy[p] = true;
      sc_n++;
      out.lsu2exe.wb_req[p].valid = true;
      out.lsu2exe.wb_req[p].uop = wb;
    } else {
      if (sta_n >= LSU_STA_COUNT) continue;
      wb.op = UOP_STA;
      wb.result = s.vaddr;
      out.lsu2exe.sta_wb_req[sta_n].valid = true;
      out.lsu2exe.sta_wb_req[sta_n++].uop = wb;
    }
    s.wbp = 0;
    s.wbd = 1;
  }
  LP_PORT_CHECK(sta_n, LSU_STA_COUNT, "sta_wb");

  // 2b. memory speculation: a written-back load whose older store addresses are all known now sends its completion
  {
    for (int i = 0; i < LDQ_SIZE; i++) {
      PLdq &l = pnxt.ldq[i];
      if (l.st != L_SPD) continue;
      const uint32_t h = (pnxt.stq_head_flag ? STQ_SIZE : 0) + pnxt.stq_head;
      const uint32_t b = (l.snap_flag ? STQ_SIZE : 0) + l.snap_idx;
      const uint32_t older = std::min<uint32_t>((b + 2 * STQ_SIZE - h) % (2 * STQ_SIZE), pnxt.stq_count);
      bool unknown = false;
      for (uint32_t k = 0; k < older && !unknown; k++) {
        const PStq &s = pnxt.stq[(pnxt.stq_head + k) % STQ_SIZE];
        unknown = !s.va_v && !s.suppress;
      }
      if (!unknown) {
        l.second = 1;
        l.st = L_WBP;
      }
    }
  }

  // 3. loads waiting for a port (forwarded after a data wait, MMIO data): lowest LDQ index first
  for (int i = 0; i < LDQ_SIZE; i++) {
    if (pnxt.ldq[i].st != L_WBP) continue;
    if (pcur.ldq[i].st != L_WBP && pw.mmio_ld != i) continue;  // F / FM: registered WBP bits (and this cycle's MMIO answer)
    int p = -1;
    for (int q = 0; q < LD && p < 0; q++)
      if (!busy[q]) p = q;
    if (p < 0) break;
    busy[p] = true;
    lp_emit_load_wb(p, i);
  }
}

// LSU_MS_DATA: the value a load's write-back puts into its destination as the RTL would compute it. Every load write-back
// leaves its register without a stale value unless one of the cases below sets one (MMIO / LR-SC loads and the faulting ones
// do not go there): a stale base register (the load read the stale address, its value stays stale: it is never re-executed
// with a good address); the first write-back of a speculative load (the value without the stores it passed); a normal or
// second write-back (the true value from the next cycle on, unless it was forwarded from a store whose data is stale).
void RealLsu::ms_load_written_back(int idx, bool defer) {
  const PLdq &l = pnxt.ldq[idx];
  const MsLdq &ml = ms_ldq_[idx];
  const auto dst = l.uop().dest_preg;
  if (l.pf || l.mmio || l.lrsc) {
    if (dst != 0) msd::taint[dst] = msd::TV{};
    return;
  }
  const tb::Fact *f = tbhook::fact_by_rob(l.uop().rob_idx);
  const bool plain = f && f->is_load && f->rd_valid && (f->inst & 0x7f) == 0x03;
  xlen_t v = 0;
  { static const char *dbg = std::getenv("MSD_DBG_PC");   // MSD_DBG_PC=<hex pc>: dump the STQ view of the stale-value computation of this pc's loads
    if (dbg && f && f->pc == (xlen_t)std::strtoull(dbg, nullptr, 16)) {
      constexpr uint32_t XB = XLEN / 8; const uint32_t W = l.paddr & ~(XB - 1);
      std::fprintf(stderr, "[MSDBG] cyc=%lld load rob=%d pc=%llx W=%x paddr=%x defer=%d skip=%llx stq_head=%u count=%u snap=%u/%u\n", (long long)sim_time, (int)l.uop().rob_idx, (unsigned long long)f->pc, W, l.paddr, (int)defer,
                   (unsigned long long)ml.skip, (unsigned)pnxt.stq_head, (unsigned)pnxt.stq_count, (unsigned)l.snap_flag, (unsigned)l.snap_idx);
      for (uint32_t rank = 0; rank < pnxt.stq_count; rank++) {
        const uint32_t j = (pnxt.stq_head + rank) % STQ_SIZE; const PStq &t = pnxt.stq[j]; const MsStq &m = ms_stq_[j];
        std::fprintf(stderr, "[MSDBG]   (any) rank=%u stq=%u pc=%x va_v=%d pa_v=%d paddr=%x m.init=%d m.v=%d m.w=%x m.atnt=%d\n", rank, j, (unsigned)t.pc, (int)t.va_v, (int)t.pa_v, (unsigned)t.paddr, (int)m.init, (int)m.v, (unsigned)m.w, (int)m.atnt);
        if (!m.v || m.w != W) continue;
        std::fprintf(stderr, "[MSDBG]   rank=%u stq=%u pc=%x cmt=%d done=%d suppress=%d data_v=%d old=%llx new=%llx rs2=%llx skipbit=%d\n", rank, j, (unsigned)t.pc, (int)t.cmt, (int)t.done, (int)t.suppress, (int)t.data_v,
                     (unsigned long long)m.old_, (unsigned long long)m.new_, (unsigned long long)m.rs2, (int)(ml.skip >> j & 1));
      } } }
  if (msd::trace_on() && trace_win::ok() && plain) {
    xlen_t sv = 0; const bool got = ms_load_value(pnxt, ms_stq_, l, ml, defer, sv);
    std::fprintf(stderr, "[MSL] cyc=%lld rob=%d p%u pc=%llx defer=%d atnt=%d stale_calc=%d v=%llx true=%llx\n", (long long)sim_time, (int)l.uop().rob_idx, (unsigned)dst,
                 (unsigned long long)f->pc, (int)defer, (int)ml.atnt, (int)got, (unsigned long long)sv, (unsigned long long)f->rd_value);
  }
  // A load the function model did not execute (its true address faults or is blocked: wrong path) but whose stale base register gave it a valid
  // address executes in the RTL and writes the memory value there into its destination: the register is stale (its true value is "unchanged").
  const bool ghost = f && f->is_load && !f->rd_valid && (f->inst & 0x7f) == 0x03 && ((f->inst >> 7) & 31) != 0 && ml.atnt;
  if (ml.atnt) {
    msd::taint[dst] = msd::TV{};
    if (ghost) { if (ms_load_value(pnxt, ms_stq_, l, ml, defer, v)) msd::set_taint(dst, true, v, 1); }
    else if (plain && ms_load_value(pnxt, ms_stq_, l, ml, defer, v) && v != f->rd_value) msd::set_taint(dst, true, v, 1);
  } else if (defer) {
    msd::taint[dst] = msd::TV{};
    // (the value of the DCache read: the one the L1 CAM pass recorded; the memory image of the write-back cycle may already hold stores the read did not see)
    const bool have = ml.cam_ok ? (v = (xlen_t)ml.cam_val, true) : ms_load_value(pnxt, ms_stq_, l, ml, true, v);
    if (plain && have && v != f->rd_value) msd::set_taint(dst, true, v, 0);
  } else {
    msd::release(dst);
    if (plain && ms_load_value(pnxt, ms_stq_, l, ml, false, v) && v != f->rd_value) msd::set_taint(dst, true, v, 1);
  }
}
