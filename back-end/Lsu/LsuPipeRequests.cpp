// The staged LSU: requests phase functions (see RealLsuPipe.cpp, docs/LSU_PIPE.md).
#include "LsuPipeInternal.h"
#include "DseCap.h"

void RealLsu::mmio_out() {
  out.peripheral_req->is_mmio = false;
  if (!mmio_resp_q_.ready) return;
  const UncachedUnit &u = cur_.uncached_unit;
  const bool valid_post_resp = u.valid && !mmio_resp_q_.is_mmio;
  const bool kill_ld = u.is_load && (in.rob_bcast->flush || in.dec_bcast->mispred);
  const bool send = valid_post_resp && !kill_ld;
  out.peripheral_req->is_mmio = send;
  if (send) {
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
    const UopS &u = uop_pool[d.ldq_id[i]];
    PLdq &e = pnxt.ldq[u.ldq_idx];
    const uint8_t ep = (e.epoch + 1) & ((1u << EPOCH_W) - 1);
    e = {};
    ms_ldq_[u.ldq_idx] = {};
    e.epoch = ep;
    e.id = d.ldq_id[i];
    {
      if (MemDepPred::kClr && (long long)sim_time >= mdp_.next_clear) {
        std::fill(mdp_.table.begin(), mdp_.table.end(), false);
        mdp_.next_clear = (long long)sim_time + MemDepPred::kClr;
      }
      e.wait = mdp_.wait(e.pc());
    }
    e.st = L_ALLOC;
  }
  for (int i = 0; i < MAX_STQ_DISPATCH_WIDTH; i++) {
    if (!d.alloc_req[i]) continue;
    const UopS &u = uop_pool[d.id[i]];
    if (pnxt.stq_count >= (uint32_t)g_dse.stq) continue;
    if (u.stq_flag != stq_tail_flag(pnxt.stq_head, pnxt.stq_count, pnxt.stq_head_flag)) continue;
    PStq &e = pnxt.stq[(pnxt.stq_head + pnxt.stq_count) % STQ_SIZE];
    e = {};
    ms_stq_[(pnxt.stq_head + pnxt.stq_count) % STQ_SIZE] = {};
    e.rob_idx = u.rob_idx;
    e.rob_flag = u.rob_flag;
    e.pc = u.dbg.pc;
    e.func3 = u.func3;
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

  if (nxt.lrsc_unit.reserve_valid && nxt.lrsc_unit.reserve_spec) {
    for (int i = 0; i < COMMIT_WIDTH; i++) {
      const auto &ce = in.rob_commit->commit_entry[i];
      if (ce.valid && ce.uop->rob_idx == nxt.lrsc_unit.reserve_rob_idx &&
          ce.uop->rob_flag == nxt.lrsc_unit.reserve_rob_flag) {
        nxt.lrsc_unit.reserve_spec = false;
      }
    }
  }

  const DecBroadcastIO &dec = (*in.dec_bcast);
  if (dec.mispred) {
    for (PLdq &l : pnxt.ldq)
      if (l.st != L_EMPTY && is_younger_rob(l.uop().rob_idx, l.uop().rob_flag, dec.redirect_rob_idx, dec.redirect_rob_flag)) {
        l.st = L_EMPTY;
      }
    uint32_t keep = pnxt.stq_count;
    for (uint32_t i = pnxt.stq_ccount; i < pnxt.stq_count; i++) {
      const PStq &st = pnxt.stq[(pnxt.stq_head + i) % STQ_SIZE];
      if (is_younger_rob(st.rob_idx, st.rob_flag, dec.redirect_rob_idx, dec.redirect_rob_flag)) {
        keep = i;
        break;
      }
    }
    pnxt.stq_count = keep;
    UncachedUnit &u = nxt.uncached_unit;
    if (u.valid && (u.is_load ? pnxt.ldq[u.idx].st == L_EMPTY : !stq_idx_alive_after_flush(u.idx, pnxt.stq_head, pnxt.stq_count)))
      u.valid = false;
    if (nxt.lrsc_unit.reserve_valid && nxt.lrsc_unit.reserve_spec &&
        is_younger_rob(nxt.lrsc_unit.reserve_rob_idx, nxt.lrsc_unit.reserve_rob_flag, dec.redirect_rob_idx, dec.redirect_rob_flag))
      nxt.lrsc_unit.reserve_valid = false;
    pnxt.stq_ccount = std::min<uint32_t>(pnxt.stq_ccount, pnxt.stq_count);
    pnxt.stq_commit = stq_idx_after(pnxt.stq_head, pnxt.stq_ccount);
  }
  if (in.rob_bcast->flush || dec.mispred) {  // replay registers of dead entries
    for (int r = 0; r < RP_MAX; r++)
      if (pnxt.rp[r].v && pnxt.ldq[pnxt.rp[r].ldq].st == L_EMPTY) pnxt.rp[r].v = 0;
    if (pnxt.srp.v && stq_rank(pnxt.srp.ldq, pnxt.stq_head) >= pnxt.stq_count) pnxt.srp.v = 0;
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
    sl.dcblk = 0;
    sl.bloom = 0;
    const uint32_t bidx = (uint32_t(e.vaddr) >> 3) & 0xFFu;
    const bool bloom_hit = !l.replay && ((pcur_.stq_bloom[bidx >> 6] >> (bidx & 63u)) & 1u);
    // Launch DCache in L1 unless port is retry-busy or 256-bit STQ word bloom filter predicts STLF hit (first pass only).
    if (l.hit && !l.mmio && in.top_in->dcache2lsu.load_retry_busy[k]) {
      sl.dcblk = 1;  // the port belongs to the DCache hidden retry this cycle
    } else if (l.hit && !l.mmio && bloom_hit) {
      sl.bloom = 1;  // likely STLF: skip speculative DCache launch to avoid bank conflicts / STQ drain starvation
    } else if (l.hit && !l.mmio) {
      e.epoch = (e.epoch + 1) & ((1u << EPOCH_W) - 1);
      ports[k].valid = true;
      ports[k].addr = l.paddr;
      ports[k].req_id = (uint32_t(e.epoch) << LDQ_IDX_WIDTH) | l.ldq;
      ports[k].pc = e.pc();
      ports[k].vaddr = e.vaddr;
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
  auto &ports = out.lsu2dcache->req_ports.store_ports;
  for (int i = 0; i < LSU_STA_COUNT; i++) ports[i].valid = ports[i].replay = false;
  const uint32_t n = std::min<uint32_t>(pcur_.stq_ccount, pnxt.stq_count);  // H: stores committed at the cycle start
  bool rdy[STQ_SIZE] = {};  // by rank
  drain_countdown(n, rdy);
  mmio_store_issue(n, rdy);
  wb_drain(n, rdy);
}

// 1. every eligible committed store counts its replay countdown down; rdy[rank]: it may be sent this cycle
void RealLsu::drain_countdown(uint32_t n, bool *rdy) {
  const lp::LsuPipeState &pcur = pcur_;
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
}

// 2. the head store, when MMIO, takes the uncached unit
void RealLsu::mmio_store_issue(uint32_t n, const bool *rdy) {
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
}

// 3. EXP-024 write buffer: the oldest registered entries whose stores are all ready and that do not overlap an older entry are
// sent, one DCache request each, on the store ports without a pending hidden retry; then this cycle's committed stores are inserted
void RealLsu::wb_drain(uint32_t n, const bool *rdy) {
  const lp::LsuPipeState &pcur = pcur_;
  auto &ports = out.lsu2dcache->req_ports.store_ports;
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
      lead.iss = 1;
      for (int x = 1; x < w.cnt; x++) {
        PStq &f = pnxt.stq[w.mem[x]];
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
// the RTL's code of a DTLB result in the LSU_PIPE_EVT print: miss 0, hit 1, page fault 2
static int tlb_code(TlbMmu::Result r) { return r == TlbMmu::Result::OK ? 1 : r == TlbMmu::Result::FAULT ? 2 : 0; }

// the end of the request phase: retire, events, wake-ups, replay picks (in this order)
void RealLsu::check() {
  stq_retire();
  print_events();
  wake_loads();
  fill_replay_registers();
}

// STQ retire: up to RETIRE_PORTS done stores leave the head (R: registered state only)
void RealLsu::stq_retire() {
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
}


// LSU_PIPE_EVT=1: the per-cycle LSU boundary events (the RTL test bench prints the same)
void RealLsu::print_events() {
  const lp::LsuPipeState &pcur = pcur_;
  if (sim_env().lsu_pipe_evt) {  // debug aid: per-cycle LSU boundary events (matches the RTL tb print)
    std::fprintf(stderr, "E t=%lld wb:", (long long)sim_time);
    for (int p = 0; p < LD; p++)
      if (out.lsu2exe.wb_req[p].valid) std::fprintf(stderr, " %d=r%d/p%d/e%d", p, (int)out.lsu2exe.wb_req[p].uop->rob_idx, (int)out.lsu2exe.wb_req[p].uop->dest_preg,
                                                       (int)out.lsu2exe.wb_req[p].uop.dest_en),
        std::fprintf(stderr, "/v%x/d%x/f%d%d", (unsigned)out.lsu2exe.wb_req[p].uop.result, (unsigned)out.lsu2exe.wb_req[p].uop.diag_val,
                     (int)out.lsu2exe.wb_req[p].uop.page_fault_load, (int)out.lsu2exe.wb_req[p].uop.page_fault_store);
    std::fprintf(stderr, " sta:");
    for (int p = 0; p < STA; p++)
      if (out.lsu2exe.sta_wb_req[p].valid) std::fprintf(stderr, " %d=r%d", p, (int)out.lsu2exe.sta_wb_req[p].uop->rob_idx),
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
      if (tlb_req_[p].valid) std::fprintf(stderr, " L%d=%d", p, (int)pw.l1[p].ldq);
    for (int p = 0; p < STA; p++)
      if (tlb_req_[LD + p].valid) std::fprintf(stderr, " S%d=%d", p, (int)pw.s1[p].stq);
    std::fprintf(stderr, " d: sf=%d lf=%d tl=%d/%d cp=%d", (int)out.lsu2dis.stq_free, (int)out.lsu2dis.ldq_free, (int)out.lsu2dis.stq_tail,
                 (int)out.lsu2dis.stq_tail_flag, (int)out.lsu2rob.committed_store_pending);
    {
      unsigned dsa = 0, dla = 0, cm = 0;
      for (int i = 0; i < MAX_STQ_DISPATCH_WIDTH; i++) dsa |= (unsigned)in.dis2lsu->alloc_req[i] << i;
      for (int i = 0; i < MAX_LDQ_DISPATCH_WIDTH; i++) dla |= (unsigned)in.dis2lsu->ldq_alloc_req[i] << i;
      for (int i = 0; i < COMMIT_WIDTH; i++) cm |= (unsigned)in.rob_commit->commit_entry[i].valid << i;
      std::fprintf(stderr, " b: mp=%d fl=%d dsa=%x dla=%x cm=%x", (int)in.dec_bcast->mispred, (int)in.rob_bcast->flush,
                   dsa, dla, cm);
    }
    std::fprintf(stderr, " in:");
    for (int p = 0; p < LD; p++)
      if (tlb_resp_[p].valid) std::fprintf(stderr, " ml%d=%d/%x", p, tlb_code(tlb_resp_[p].result), (unsigned)tlb_resp_[p].paddr);
    for (int p = 0; p < STA; p++)
      if (tlb_resp_[LD + p].valid) std::fprintf(stderr, " ms%d=%d/%x", p, tlb_code(tlb_resp_[LD + p].result), (unsigned)tlb_resp_[LD + p].paddr);
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
}

// wake-up pass: this cycle's STQ events and the MSHR fill wake the parked loads
void RealLsu::wake_loads() {
  const lp::LsuPipeState &pcur = pcur_;
  // wake-up pass
  uint32_t full_wakes = 0;
  // Registered STA/STD wake events (STAEV_REG): address / data events reach parked loads one cycle later
  const stq_mask_t ev_sta = ev_q_.sta, ev_data = ev_q_.data;
  ev_d_.sta = pw.ev_sta;
  ev_d_.data = pw.ev_data;
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
      { static const bool tr = std::getenv("PARK_TRACE") != nullptr;
        if (tr && e.rwait == 0) std::fprintf(stderr, "[WAKE] cyc=%lld rob=%d paddr=%x fill=%d rtype=%d\n", (long long)sim_time, (int)e.uop().rob_idx, (unsigned)e.paddr, (int)pw.fill, (int)e.rtype); }
      break;
    case RC_ADDR:
      if ((ev_sta | pw.ev_ret) & b) {  // STA broadcast: confirm against the store now (same decision as L2)
        const PStq &s = pcur.stq[e.blk_idx];   // registered store (no same-cycle bypass)
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
      if (ev_data & b)
        e.st = L_WBP;  // the STD write is forwarded straight to the write-back (decision was already made at L2)
      else if (pw.ev_ret & b)
        e.st = L_RDY;
      break;
    default: break;
    }
  }
}

// replay registers and the STA replay register: the next entries to re-enter L1
void RealLsu::fill_replay_registers() {
  const lp::LsuPipeState &pcur = pcur_;
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

