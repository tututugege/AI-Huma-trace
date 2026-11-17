// The staged LSU: requests phase functions (see RealLsuPipe.cpp, docs/LSU_PIPE.md).
#include "LsuPipeInternal.h"

void RealLsu::mmio_out() {
  out.peripheral_req->is_mmio = false;
  if (!mmio_resp_q_.ready) return;
  const UncachedUnit &u = nxt.uncached_unit;
  out.peripheral_req->is_mmio = u.valid;
  if (u.valid) {
    out.peripheral_req->wen = !u.is_load;
    out.peripheral_req->mmio_addr = u.addr;
    out.peripheral_req->mmio_wdata = u.wdata;
    out.peripheral_req->mmio_fun3 = u.func3;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Allocation
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::dis2lsu() {
  const DisLsuIO &d = (*in.dis2lsu);
  for (int i = 0; i < MAX_LDQ_DISPATCH_WIDTH; i++) {
    if (!d.ldq_alloc_req[i]) continue;
    PLdq &e = pnxt.ldq[d.ldq_idx[i]];
    const uint8_t ep = (e.epoch + 1) & ((1u << EPOCH_W) - 1);
    e = {};
    e.epoch = ep;
    e.rob_idx = d.ldq_rob_idx[i];
    e.rob_flag = d.ldq_rob_flag[i];
    e.br_mask = d.ldq_br_mask[i];
    e.pc = d.ldq_pc[i];
    {
      if (MemDepPred::kClr && (long long)sim_time >= mdp_.next_clear) {
        std::fill(mdp_.table.begin(), mdp_.table.end(), false);
        mdp_.next_clear = (long long)sim_time + MemDepPred::kClr;
      }
      e.wait = mdp_.wait(e.pc);
    }
    e.st = L_ALLOC;
  }
  for (int i = 0; i < MAX_STQ_DISPATCH_WIDTH; i++) {
    if (!d.alloc_req[i]) continue;
    if (pnxt.stq_count >= STQ_SIZE) continue;
    if (d.stq_flag[i] != stq_tail_flag(pnxt.stq_head, pnxt.stq_count, pnxt.stq_head_flag)) continue;
    PStq &e = pnxt.stq[(pnxt.stq_head + pnxt.stq_count) % STQ_SIZE];
    e = {};
    e.rob_idx = d.rob_idx[i];
    e.rob_flag = d.rob_flag[i];
    e.br_mask = d.br_mask[i];
    e.pc = d.st_pc[i];
    e.func3 = d.func3[i];
    pnxt.stq_count++;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// flush / mispredict / clear_mask
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::flush() {
  const lp::LsuPipeState &pcur = pcur_;
  if (in.rob_bcast->flush) {
    msd::clear_all();  // every stale value dies with the flush
    for (PLdq &l : pnxt.ldq) {
            l.st = L_EMPTY;
    }
    const uint32_t keep = std::min<uint32_t>(pnxt.stq_ccount, pnxt.stq_count);
    pnxt.stq_count = pnxt.stq_ccount = keep;
    pnxt.stq_commit = stq_idx_after(pnxt.stq_head, keep);
    const bool keep_store = nxt.uncached_unit.valid && !nxt.uncached_unit.is_load &&
                            stq_idx_alive_after_flush(nxt.uncached_unit.idx, pnxt.stq_head, keep) &&
                            pnxt.stq[nxt.uncached_unit.idx].mmw;
    if (!keep_store) nxt.uncached_unit.valid = false;
  }

  const DecBroadcastIO &dec = (*in.dec_bcast);
  if (dec.mispred) {
    for (PLdq &l : pnxt.ldq)
      if (l.st != L_EMPTY && (l.br_mask & dec.br_mask) != 0) {
        l.st = L_EMPTY;
      }
    uint32_t keep = pnxt.stq_count;
    for (uint32_t i = 0; i < pnxt.stq_count; i++)
      if ((pnxt.stq[(pnxt.stq_head + i) % STQ_SIZE].br_mask & dec.br_mask) != 0) {
        keep = i;
        break;
      }
    if (keep < pnxt.stq_ccount) {  // invariant used by the RTL (LP_CUT bit 7): a mispredict never cuts a committed store
      std::fprintf(stderr, "[LSU_PIPE] invariant violated: mispredict cut %u below ccount %u\n", keep, pnxt.stq_ccount);
      std::abort();
    }
    pnxt.stq_count = keep;
    UncachedUnit &u = nxt.uncached_unit;
    if (u.valid && (u.is_load ? pnxt.ldq[u.idx].st == L_EMPTY : !stq_idx_alive_after_flush(u.idx, pnxt.stq_head, pnxt.stq_count)))
      u.valid = false;
    if (nxt.lrsc_unit.reserve_valid && (nxt.lrsc_unit.reserve_br_mask & dec.br_mask) != 0) nxt.lrsc_unit.reserve_valid = false;
    pnxt.stq_ccount = std::min<uint32_t>(pnxt.stq_ccount, pnxt.stq_count);
    pnxt.stq_commit = stq_idx_after(pnxt.stq_head, pnxt.stq_ccount);
  }
  if (in.rob_bcast->flush || dec.mispred) {  // replay registers of dead entries
    for (int r = 0; r < RP_MAX; r++)
      if (pnxt.rp[r].v && pnxt.ldq[pnxt.rp[r].ldq].st == L_EMPTY) pnxt.rp[r].v = 0;
    if (pnxt.srp.v && stq_rank(pnxt.srp.ldq, pnxt.stq_head) >= pnxt.stq_count) pnxt.srp.v = 0;
  }

  if (dec.clear_mask) {  // a branch resolved correctly: its mask bit is free again
    for (int i = 0; i < LDQ_SIZE; i++)
      if (pcur.ldq[i].st != L_EMPTY) pnxt.ldq[i].br_mask &= ~dec.clear_mask;
    for (uint32_t i = 0; i < pcur.stq_count; i++) pnxt.stq[(pcur.stq_head + i) % STQ_SIZE].br_mask &= ~dec.clear_mask;
    if (nxt.lrsc_unit.reserve_valid) nxt.lrsc_unit.reserve_br_mask &= ~dec.clear_mask;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// L1 -> DCache request and the L1/L2 pipeline register
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::lsu2dcache() {
  lsu2dcache_ldq();
  lsu2dcache_stq();
}

void RealLsu::lsu2dcache_ldq() {
  auto &ports = out.lsu2dcache->req_ports.load_ports;
  for (int i = 0; i < LSU_LDU_COUNT; i++) ports[i].valid = ports[i].replay = false;
  for (int k = 0; k < LD; k++) {
    const PL1 &l = pw.l1[k];
    if (!l.v) continue;
    PLdq &e = pnxt.ldq[l.ldq];
    if (e.st != L_PIPE) continue;  // killed in this cycle
    PL12 &sl = pnxt.l12[k];
    sl.v = 1;
    sl.ldq = l.ldq;
    sl.tlb = l.tlb;
    sl.mmio = l.mmio;
    sl.cand_v = l.cand_v;
    sl.cand = l.cand;
    sl.dcblk = 0;
    // the DCache is read in parallel with the partial CAM; the read enable is qualified by "no conflicting older store"
    // (12-bit compare, ready with the DTLB result); a port with a pending DCache hidden retry does not launch.
    if (l.hit && !l.mmio && !l.cand_v && in.top_in->dcache2lsu.load_retry_busy[k]) {
      sl.dcblk = 1;  // the port belongs to the DCache hidden retry this cycle
    } else if (l.hit && !l.mmio && !l.cand_v) {
      e.epoch = (e.epoch + 1) & ((1u << EPOCH_W) - 1);
      ports[k].valid = true;
      ports[k].addr = l.paddr;
      ports[k].req_id = (uint32_t(e.epoch) << LDQ_IDX_WIDTH) | l.ldq;
      ports[k].replay = e.rtype != (uint8_t)ReplayType::HIT;
    }
    sl.epoch = e.epoch;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// committed-store drain (RTL-aligned form, see rtl docs/analysis/lsu-pipe.md "model changes" M4):
//   1. every eligible store of the committed window counts its replay countdown down (all of them, every cycle);
//   2. the head store, when MMIO, takes the uncached unit;
//   3. the other committed stores go through the write buffer (EXP-024): the oldest registered entries whose stores are all ready and
//      that do not overlap an older entry are sent, one DCache request each, on the store ports without a pending hidden retry
//      (first lp_drain ports); then the stores committed at the cycle start are inserted (wb_insert).
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::lsu2dcache_stq() {
  const lp::LsuPipeState &pcur = pcur_;
  auto &ports = out.lsu2dcache->req_ports.store_ports;
  for (int i = 0; i < LSU_STA_COUNT; i++) ports[i].valid = ports[i].replay = false;

  const uint32_t n = std::min<uint32_t>(pcur.stq_ccount, pnxt.stq_count);  // H: stores committed at the cycle start
  bool rdy[STQ_SIZE] = {};  // by rank
  for (uint32_t rank = 0; rank < n; rank++) {
    const uint32_t j = (pnxt.stq_head + rank) % STQ_SIZE;
    PStq &s = pnxt.stq[j];
    if (!(s.cmt && !s.done && !s.mmw)) continue;
    if (pcur.stq[j].iss) continue;  // in flight, or answered this cycle: re-send earliest next cycle
    if (s.suppress) { std::fprintf(stderr, "[LSU_PIPE] committed suppress store not done (unreachable)\n"); std::abort(); }
    const uint32_t rw_seen = (pw.st_fullwake >> j & 1) ? pw.st_rwpre[j] : s.rwait;  // G
    if (s.rwait > 0) s.rwait--;
    rdy[rank] = rw_seen <= 1;
  }
  if (n > 0 && rdy[0] && pnxt.stq[pnxt.stq_head].mmio && !nxt.uncached_unit.valid) {  // MMIO store: only the head
    PStq &s = pnxt.stq[pnxt.stq_head];
    UncachedUnit &u = nxt.uncached_unit;
    u.valid = true;
    u.is_load = false;
    u.addr = s.paddr;
    u.wdata = s.data;
    u.func3 = s.func3;
    u.idx = pnxt.stq_head;
    s.mmw = 1;
  }
  // free ports among the first lp_drain (no hidden retry pending)
  int free_port[DRAIN_MAX], nfree = 0;
  for (int t = 0; t < lp_drain; t++)
    if (!in.top_in->dcache2lsu.store_retry_busy[t]) free_port[nfree++] = t;
  {  // EXP-024: the committed stores go through the write buffer
    bool issued_now[WB_MAX] = {};
    const LsuPipeState &C = pcur;  // the decision reads the registered buffer only
    int order[WB_MAX], no = 0;
    for (int e = 0; e < WB_MAX; e++) if (C.wb[e].v) order[no++] = e;
    std::sort(order, order + no, [&](int a, int b) { return C.wb[a].seq < C.wb[b].seq; });
    int used = 0;
    for (int k = 0; k < no && k < lp_wbscan && used < nfree; k++) {
      const PWbE &w = C.wb[order[k]];
      if (w.st != 0) continue;
      bool ready = true, blocked = false;
      for (int x = 0; x < w.cnt && ready; x++) {
        const uint32_t jx = w.mem[x], rx = stq_rank(jx, pnxt.stq_head);
        if (rx >= n || !rdy[rx]) { ready = false; break; }
        const PStq &sx = pnxt.stq[jx];
        const uint8_t stx = get_store_strb(sx.paddr, sx.func3);
        for (int k2 = 0; k2 < k && !blocked; k2++) {  // older registered entries (not done: they are still valid)
          const PWbE &o = C.wb[order[k2]];
          for (int y = 0; y < o.cnt && !blocked; y++) {
            const PStq &sy = pnxt.stq[o.mem[y]];
            if (CheckAddr(sy.paddr, get_store_strb(sy.paddr, sy.func3), sx.paddr, stx)) blocked = true;
          }
        }
      }
      if (!ready) continue;
      if (blocked) { continue; }
      PStq &lead = pnxt.stq[w.mem[0]];
      auto &p = ports[free_port[used++]];
      p.valid = true;
      p.addr = lead.paddr;
      p.strb = get_store_strb(lead.paddr, lead.func3);
      p.req_id = w.mem[0];
      p.replay = lead.rtype != (uint8_t)ReplayType::HIT;
      lead.mf = 0;
      if (!lead.iss) wd_arm(lead.wd_t);
      lead.iss = 1;
      for (int x = 1; x < w.cnt; x++) {
        PStq &f = pnxt.stq[w.mem[x]];
        if (!f.iss) wd_arm(f.wd_t);
        f.iss = 1;
        f.mf = 1;
        f.ml = w.mem[0];
      }
      pnxt.wb[order[k]].st = 1;
      issued_now[order[k]] = true;
    }
    wb_insert(pnxt, C, n, issued_now);
  }
}

// ---------------------------------------------------------------------------------------------------------------
// STQ retire, wake-up pass (events of this cycle -> LDQ entries), replay-register fill
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::check() {
  const lp::LsuPipeState &pcur = pcur_;
  int ret = 0;
  while (ret < RETIRE_PORTS && pnxt.stq_ccount > 0 && pnxt.stq_count > 0 && pnxt.stq[pnxt.stq_head].done &&
         (uint32_t)ret < pcur.stq_ccount && (uint32_t)ret < pcur.stq_count && pcur.stq[pnxt.stq_head].done) {  // R: registered state only
    pw.ev_ret |= (stq_mask_t)1 << pnxt.stq_head;
    pnxt.stq_count--;
    pnxt.stq_ccount--;
    pnxt.stq_head = (pnxt.stq_head + 1) % STQ_SIZE;
    if (pnxt.stq_head == 0) pnxt.stq_head_flag = !pnxt.stq_head_flag;
    ret++;
  }
  LP_PORT_CHECK(ret, RETIRE_PORTS, "stq_retire");

  // DCache response watchdog: a request whose response never arrives (lost bank-retry latch in the DCache) is replayed.
  // The RTL has none: LSU_PIPE_NOWD=1 turns it off (needed to match the RTL's cycles, e.g. crc32).
  // An entry waits from the cycle it was sent (wd_t, set by wd_arm); it is replayed 62 cycles later. Only the cycle of the earliest
  // deadline scans the queues (a deadline of an entry that got its response meanwhile only costs that scan).
  if (sim_time >= wd_next_) {
    const bool nowd = sim_env().lsu_pipe_nowd;
    long long nx = 0x7fffffffffffffffLL;
    for (int i = 0; i < LDQ_SIZE; i++) {
      PLdq &e = pnxt.ldq[i];
      if (e.st != L_DCW) continue;
      if (e.wd_t + 62 > sim_time) nx = std::min(nx, e.wd_t + 62);
      else if (!nowd) e.st = L_RDY;
    }
    for (int j = 0; j < STQ_SIZE; j++) {
      PStq &t = pnxt.stq[j];
      if (!t.iss) continue;
      if (t.wd_t + 62 > sim_time) nx = std::min(nx, t.wd_t + 62);
      else if (!nowd) { t.iss = 0; t.rwait = 0; if (lp_wbuf > 0 && t.wbi == 1) { PWbE &w = pnxt.wb[t.wbe]; if (w.v && w.mem[0] == (uint8_t)j) w.st = 0; } }
    }
    wd_next_ = nx;
  }
  if (sim_env().lsu_pipe_evt) {  // debug aid: per-cycle LSU boundary events (matches the RTL tb print)
    std::fprintf(stderr, "E t=%lld wb:", (long long)sim_time);
    for (int p = 0; p < LD; p++)
      if (out.lsu2exe.wb_req[p].valid) std::fprintf(stderr, " %d=r%d/p%d/e%d/b%llx", p, (int)out.lsu2exe.wb_req[p].uop->rob_idx, (int)out.lsu2exe.wb_req[p].uop->dest_preg,
                                                       (int)out.lsu2exe.wb_req[p].uop.dest_en, (unsigned long long)out.lsu2exe.wb_req[p].uop.br_mask),
        std::fprintf(stderr, "/v%x/d%x/f%d%d", (unsigned)out.lsu2exe.wb_req[p].uop.result, (unsigned)out.lsu2exe.wb_req[p].uop.diag_val,
                     (int)out.lsu2exe.wb_req[p].uop.page_fault_load, (int)out.lsu2exe.wb_req[p].uop.page_fault_store);
    std::fprintf(stderr, " sta:");
    for (int p = 0; p < STA; p++)
      if (out.lsu2exe.sta_wb_req[p].valid) std::fprintf(stderr, " %d=r%d/b%llx", p, (int)out.lsu2exe.sta_wb_req[p].uop->rob_idx, (unsigned long long)out.lsu2exe.sta_wb_req[p].uop.br_mask),
        std::fprintf(stderr, "/v%x/d%x/f%d", (unsigned)out.lsu2exe.sta_wb_req[p].uop.result, (unsigned)out.lsu2exe.sta_wb_req[p].uop.diag_val,
                     (int)out.lsu2exe.sta_wb_req[p].uop.page_fault_store);
    std::fprintf(stderr, " dl:");
    for (int p = 0; p < LD; p++) {
      const auto &q = out.lsu2dcache->req_ports.load_ports[p];
      if (q.valid) std::fprintf(stderr, " %d=%x/%x", p, (unsigned)q.addr, (unsigned)(q.req_id & 0x7fffffffu));
    }
    std::fprintf(stderr, " ds:");
    for (int p = 0; p < STA; p++) {
      const auto &q = out.lsu2dcache->req_ports.store_ports[p];
      if (q.valid) std::fprintf(stderr, " %d=%x/%d", p, (unsigned)q.addr, (int)q.req_id);
    }
    std::fprintf(stderr, " mmu:");
    for (int p = 0; p < LD; p++)
      if (tlb_req_.ldq_req[p].valid) std::fprintf(stderr, " L%d=%d", p, (int)tlb_req_.ldq_req[p].entry_idx);
    for (int p = 0; p < STA; p++)
      if (tlb_req_.stq_req[p].valid) std::fprintf(stderr, " S%d=%d", p, (int)tlb_req_.stq_req[p].entry_idx);
    std::fprintf(stderr, " d: sf=%d lf=%d tl=%d/%d cp=%d", (int)out.lsu2dis.stq_free, (int)out.lsu2dis.ldq_free, (int)out.lsu2dis.stq_tail,
                 (int)out.lsu2dis.stq_tail_flag, (int)out.lsu2rob.committed_store_pending);
    {
      unsigned dsa = 0, dla = 0, cm = 0;
      for (int i = 0; i < MAX_STQ_DISPATCH_WIDTH; i++) dsa |= (unsigned)in.dis2lsu->alloc_req[i] << i;
      for (int i = 0; i < MAX_LDQ_DISPATCH_WIDTH; i++) dla |= (unsigned)in.dis2lsu->ldq_alloc_req[i] << i;
      for (int i = 0; i < COMMIT_WIDTH; i++) cm |= (unsigned)in.rob_commit->commit_entry[i].valid << i;
      std::fprintf(stderr, " b: mp=%d fl=%d cl=%llx dsa=%x dla=%x cm=%x", (int)in.dec_bcast->mispred, (int)in.rob_bcast->flush,
                   (unsigned long long)in.dec_bcast->clear_mask, dsa, dla, cm);
    }
    std::fprintf(stderr, " in:");
    for (int p = 0; p < LD; p++)
      if (tlb_resp_.ldq_resp[p].valid) std::fprintf(stderr, " ml%d=%d/%x", p, (int)tlb_resp_.ldq_resp[p].result, (unsigned)tlb_resp_.ldq_resp[p].paddr);
    for (int p = 0; p < STA; p++)
      if (tlb_resp_.stq_resp[p].valid) std::fprintf(stderr, " ms%d=%d/%x", p, (int)tlb_resp_.stq_resp[p].result, (unsigned)tlb_resp_.stq_resp[p].paddr);
    for (int p = 0; p < LD; p++) {
      const auto &r = in.top_in->dcache2lsu.resp_ports.load_resps[p];
      if (r.valid) std::fprintf(stderr, " dr%d=%x/%d", p, (unsigned)(r.req_id & 0x7fffffffu), (int)r.replay);
    }
    for (int p = 0; p < STA; p++) {
      const auto &r = in.top_in->dcache2lsu.resp_ports.store_resps[p];
      if (r.valid) std::fprintf(stderr, " sr%d=%d/%d", p, (int)r.req_id, (int)r.replay);
    }
    if (in.top_in->dcache2lsu.mshr_fill) std::fprintf(stderr, " fill=%x", (unsigned)(in.top_in->dcache2lsu.mshr_fill_addr >> 6));
    for (int p = 0; p < LD; p++) if (in.top_in->dcache2lsu.load_retry_busy[p]) std::fprintf(stderr, " rb%d", p);
    for (int p = 0; p < STA; p++) if (in.top_in->dcache2lsu.store_retry_busy[p]) std::fprintf(stderr, " sb%d", p);
    std::fprintf(stderr, " ai:");
    for (int p = 0; p < MAX_LDQ_DISPATCH_WIDTH; p++)
      if (out.lsu2dis.ldq_alloc_valid[p]) std::fprintf(stderr, " %d", (int)out.lsu2dis.ldq_alloc_idx[p]);
    std::fprintf(stderr, " x:");
    for (int p = 0; p < LSU_SDU_COUNT; p++)
      if (in.exe2lsu->sdu_req[p].valid) std::fprintf(stderr, " D%d=%d", p, (int)in.exe2lsu->sdu_req[p].uop->stq_idx);
    for (int p = 0; p < LSU_AGU_COUNT; p++)
      if (in.exe2lsu->agu_req[p].valid && in.exe2lsu->agu_req[p].uop.op == UOP_STA)
        std::fprintf(stderr, " A%d=%d", p - LD, (int)in.exe2lsu->agu_req[p].uop->stq_idx);
    for (int p = 0; p < LSU_AGU_COUNT; p++)
      if (in.exe2lsu->agu_req[p].valid && in.exe2lsu->agu_req[p].uop.op == UOP_LOAD)
        std::fprintf(stderr, " G%d=%d", p, (int)in.exe2lsu->agu_req[p].uop->ldq_idx);
    if (const char *si = sim_env().lsu_pipe_stq) {
      const PStq &q = pcur.stq[std::atoi(si)];  // registers at the start of the cycle (RTL prints the registers)
      std::fprintf(stderr, " Q[%s]: va=%d pa=%d dv=%d pf=%d wbp=%d wbd=%d cmt=%d iss=%d done=%d", si, (int)q.va_v, (int)q.pa_v, (int)q.data_v, (int)q.pf,
                   (int)q.wbp, (int)q.wbd, (int)q.cmt, (int)q.iss, (int)q.done);
    }
    std::fprintf(stderr, "\n");
  }
  // wake-up pass
  uint32_t full_wakes = 0;
  for (int i = 0; i < LDQ_SIZE; i++) {
    PLdq &e = pnxt.ldq[i];
    if (e.st != L_RPW) continue;
    const stq_mask_t b = (stq_mask_t)1 << e.blk_idx;
    switch (e.rcause) {
    case RC_TLB:
      if (e.rwait > 0) e.rwait--;
      if (e.rwait == 0) e.st = L_RDY;
      break;
    case RC_DC:
      if (pw.fill && e.rtype == (uint8_t)ReplayType::MSHR_HIT && cache_line_match(e.paddr, pw.fill_addr)) {
        e.rwait = 0;
      } else if (pw.fill && e.rtype == (uint8_t)ReplayType::MSHR_FULL && full_wakes < (uint32_t)LD &&
                 pcur.ldq[i].st == L_RPW) {
        e.rwait = 0;
        full_wakes++;
      } else if (e.rwait > 0) {
        e.rwait--;
      }
      if (e.rwait == 0) e.st = L_RDY;
      break;
    case RC_ADDR:
      if ((pw.ev_sta | pw.ev_ret) & b) {  // STA broadcast: confirm against the store now (same decision as L2)
        const PStq &s = pnxt.stq[e.blk_idx];
        const Conf c = stlf_confirm(e, s, stq_rank(e.blk_idx, pcur.stq_head) < pcur.stq_count && !(pw.ev_ret & b));  // B
        if (c == CF_REPLAY) e.st = L_RDY;
        else if (c == CF_HOLD) e.rcause = RC_HOLD;
        else if (c == CF_DATA) e.rcause = RC_DATA;
        else if (c == CF_FWD) e.st = L_WBP;
      }
      break;
    case RC_HOLD:
      if ((pw.ev_ret | pw.ev_sc) & b) e.st = L_RDY;
      break;
    case RC_DATA:
      if (pw.ev_data & b)
        e.st = L_WBP;  // the STD write is forwarded straight to the write-back (decision was already made at L2)
      else if (pw.ev_ret & b)
        e.st = L_RDY;
      break;
    default: break;
    }
  }

  // replay registers: find-first over the RDY mask including this cycle's wake-ups (selected entry re-enters at L1 next cycle)
  for (int r = 0; r < lp_rp_ports; r++) {
    if (pnxt.rp[r].v) continue;
    for (int k = 0; k < LDQ_SIZE; k++) {  // find-first over the 64-bit RDY vector rotated by rp_ptr
      const int i = (pnxt.rp_ptr + k) % LDQ_SIZE;
      if (pnxt.ldq[i].st != L_RDY) continue;
      if (pcur.ldq[i].st != L_RDY) continue;  // D: registered RDY bits
      pnxt.ldq[i].st = L_PIPE;
      pnxt.rp[r].v = 1;
      pnxt.rp[r].ldq = i;
      pnxt.rp_ptr = (i + 1) % LDQ_SIZE;
      break;
    }
  }
  if (!pnxt.srp.v) {  // STA replay register: store with a pending DTLB retry, round-robin over the STQ index
    // Liveness: the DTLB walk is shared, a store of another page only gets RETRY (OTHER_WALK_ACTIVE) until the owner of the walk
    // asks again. Always taking the oldest store starves a younger walk owner (Linux boot, ROB deadlock), so the pick rotates
    // (like rp_ptr for the loads).
    for (uint32_t k = 0; k < STQ_SIZE; k++) {
      const uint32_t j = (pnxt.srp_ptr + k) % STQ_SIZE;
      if (stq_rank(j, pnxt.stq_head) >= pnxt.stq_count || !pnxt.stq[j].trp) continue;
      pnxt.stq[j].trp = 0;
      pnxt.srp.v = 1;
      pnxt.srp.ldq = j;
      pnxt.srp_ptr = (j + 1) % STQ_SIZE;
      break;
    }
  }
}
