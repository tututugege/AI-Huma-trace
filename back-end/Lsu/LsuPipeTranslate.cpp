// The staged LSU: translate phase functions (see RealLsuPipe.cpp, docs/LSU_PIPE.md).
#include "LsuPipeInternal.h"

// ---------------------------------------------------------------------------------------------------------------
// AGU arrivals: STD write, STA (va), load (L1 input)
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::exe2lsu() {
  const lp::LsuPipeState &pcur = pcur_;
  const ExeLsuIO &x = (*in.exe2lsu);
  for (int i = 0; i < LSU_SDU_COUNT; i++) {
    if (!x.sdu_req[i].valid) continue;
    const MicroOp &inst = x.sdu_req[i].uop;
    if (inst.stq_idx >= STQ_SIZE) continue;
    PStq &e = pnxt.stq[inst.stq_idx];
    if (e.rob_flag != inst.rob_flag || e.rob_idx != inst.rob_idx) continue;
    e.data = inst.result;
    e.data_v = 1;
    pw.ev_data |= (stq_mask_t)1 << inst.stq_idx;
    if (e.pa_v && !e.wbp && !e.wbd) e.wbp = 1;
  }
  int ns = 0;
  int vio_new = 0;  // loads marked as violating in this cycle
  uint64_t vio_set = 0;
  for (int i = 0; i < LSU_AGU_COUNT; i++) {
    if (!(x.agu_req[i].valid && x.agu_req[i].uop.op == UOP_STA)) continue;
    const MicroOp &inst = x.agu_req[i].uop;
    if (inst.stq_idx >= STQ_SIZE) continue;
    PStq &e = pnxt.stq[inst.stq_idx];
    if (e.rob_flag != inst.rob_flag || e.rob_idx != inst.rob_idx) continue;
    e.va_v = 1;
    e.vaddr = inst.result;
    {  // a store whose base register was stale: its data goes to the stale address (never to the true one)
      const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx);
      e.m_atnt = f && f->is_store && f->vaddr != static_cast<xlen_t>(inst.result);
    }
    e.pa_v = 0;
    e.paddr = 0;
    e.func3 = inst->func3;
    e.dest_preg = inst.dest_preg;
    e.lrsc = inst->is_atomic && ((inst->func7 >> 2) == AmoOp::SC);
    e.trp = 0;
    LP_PORT_CHECK(ns + 1, LSU_STA_COUNT, "sta_agu");
    pw.s1[ns].v = true;
    pw.s1[ns].stq = inst.stq_idx;
    ns++;
    pw.ev_sta |= (stq_mask_t)1 << inst.stq_idx;  // the partial address is known from now on
    {  // memory-order check: younger loads that passed this store while its address was unknown
      const uint32_t j = inst.stq_idx;
      const uint32_t h = (pcur.stq_head_flag ? STQ_SIZE : 0) + pcur.stq_head;
      const uint32_t sw = ld_width(inst->func3);
      for (int i = 0; i < LDQ_SIZE; i++) {
        PLdq &l = pnxt.ldq[i];
        if (!l.spec || l.vio || l.second) continue;
        // L_RPW too: a load parked behind an older store (ADDR / HOLD / DATA) forwards from it when woken, without another CAM pass, and would
        // miss a younger store whose address it passed (found by the RTL Linux check, s06: stale byte forwarded)
        if (l.st != L_PIPE && l.st != L_DCW && l.st != L_WBP && l.st != L_WB3 && l.st != L_SPD && l.st != L_RPW) continue;
        const uint32_t b = (l.snap_flag ? STQ_SIZE : 0) + l.snap_idx;
        const uint32_t older = std::min<uint32_t>((b + 2 * STQ_SIZE - h) % (2 * STQ_SIZE), pcur.stq_count);
        if (stq_rank(j, pcur.stq_head) >= older) continue;  // the store is younger than the load
        if (range_rel(l.vaddr, ld_width(l.func3), inst.result, sw, 12) == DISJOINT) continue;
        vio_new++;
        vio_set |= 1ULL << i;
        l.vio = 1;
        if (l.st == L_SPD) {  // already written back: re-execute with the store visible
          l.second = 1;
          l.st = L_RDY;
        }
      }
    }
  }
  if (vio_new) {
    mdp_.trn |= vio_set;
  }
  int nl = 0;
  for (int i = 0; i < LSU_AGU_COUNT; i++) {
    if (!(x.agu_req[i].valid && x.agu_req[i].uop.op == UOP_LOAD)) continue;
    const MicroOp &inst = x.agu_req[i].uop;
    if (inst.ldq_idx >= LDQ_SIZE) continue;
    PLdq &e = pnxt.ldq[inst.ldq_idx];
    if (e.rob_flag != inst.rob_flag || e.rob_idx != inst.rob_idx) continue;
    e.vaddr = inst.result;
    {  // a load whose base register was stale reads the stale address
      const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx);
      e.atnt = f && f->is_load && f->vaddr != static_cast<xlen_t>(inst.result);
    }
    e.dest_preg = inst.dest_preg;
    e.pa_v = 0;
    e.paddr = 0;
    e.func3 = inst->func3;
    e.illegal = inst.illegal_inst;
    e.st = L_PIPE;
    e.snap_idx = inst.stq_idx;
    e.snap_flag = inst.stq_flag;
    e.lrsc = is_amo_lr_uop(inst);
    LP_PORT_CHECK(nl + 1, LD, "load_agu");
    pw.l1[nl].v = true;
    pw.l1[nl].need_tlb = true;
    pw.l1[nl].ldq = inst.ldq_idx;
    nl++;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// L1: DTLB request (new loads, replay registers fill the remaining load ports), STA translation
// ---------------------------------------------------------------------------------------------------------------
// L1 address translation, one phase: the requests of the load / STA pipes, the DTLB lookup (a fence flushes it, a flush
// or mispredict cancels its pending walk and returns nothing), the responses.
void RealLsu::tlb() {
  tlb_out();
  tlb_resp_ = {};
  if (dtlb != nullptr) {
    if (in.rob_bcast->fence) dtlb->flush();
    else if (in.rob_bcast->flush || in.dec_bcast->mispred) dtlb->cancel_pending_walk();
    else dtlb->translate_lsu_ports(tlb_req_, tlb_resp_);
  }
  tlb_in_ldq();
  tlb_in_stq();
}

void RealLsu::tlb_out() {
  const lp::LsuPipeState &pcur = pcur_;
  tlb_req_ = {};
  tlb_req_.csr_status = (*in.csr_status);

  int n = 0;
  while (n < LD && pw.l1[n].v) n++;
  for (int r = 0; r < lp_rp_ports; r++) {
    if (n >= LD) continue;
    int src = -1;
    if (pcur.rp[r].v) {
      src = pcur.rp[r].ldq;
      pnxt.rp[r].v = 0;
    }
    if (src < 0) continue;
    PLdq &e = pnxt.ldq[src];
    if (e.st != L_PIPE) continue;
    pw.l1[n].v = true;
    pw.l1[n].replay = true;
    pw.l1[n].need_tlb = !e.pa_v;
    pw.l1[n].ldq = src;
    n++;
  }
  for (int k = 0; k < LD; k++) {
    if (!pw.l1[k].v || !pw.l1[k].need_tlb) continue;
    auto &rq = tlb_req_.ldq_req[k];
    rq.valid = true;
    rq.vaddr = pnxt.ldq[pw.l1[k].ldq].vaddr;
    rq.entry_idx = pw.l1[k].ldq;
    rq.wait_idx = pw.l1[k].ldq;
  }

  int ns = 0;
  while (ns < STA && pw.s1[ns].v) ns++;
  if (ns < STA && pcur.srp.v) {  // STA replay register takes a free STA port
    const uint32_t j = pcur.srp.ldq;
    pnxt.srp.v = 0;
    if (stq_rank(j, pnxt.stq_head) < pnxt.stq_count && pnxt.stq[j].va_v && !pnxt.stq[j].pa_v && !pnxt.stq[j].pf) {
      pw.s1[ns].v = true;
      pw.s1[ns].replay = true;
      pw.s1[ns].stq = j;
      ns++;
    }
  }
  for (int k = 0; k < STA; k++) {
    if (!pw.s1[k].v) continue;
    auto &rq = tlb_req_.stq_req[k];
    rq.valid = true;
    rq.vaddr = pnxt.stq[pw.s1[k].stq].vaddr;
    rq.entry_idx = pw.s1[k].stq;
    rq.wait_idx = pw.s1[k].stq;
  }
}

void RealLsu::tlb_in_ldq() {
  for (int k = 0; k < LD; k++) {
    PL1 &l = pw.l1[k];
    if (!l.v) continue;
    PLdq &e = pnxt.ldq[l.ldq];
    if (!l.need_tlb) {  // replay with a known physical address
      l.hit = true;
      l.mmio = e.mmio;
      l.paddr = e.paddr;
      continue;
    }
    const auto &resp = tlb_resp_.ldq_resp[k];
    if (!resp.valid || resp.entry_idx != l.ldq) {
      l.tlb = 1;
    } else if (resp.result == MMUResultType::HIT) {
      e.pa_v = 1;
      e.paddr = resp.paddr;
      e.mmio = lsu_is_mmio_addr(resp.paddr);
      l.hit = true;
      l.mmio = e.mmio;
      l.paddr = resp.paddr;
    } else if (resp.result == MMUResultType::PAGE_FAULT) {
      e.pf = 1;
      e.result = e.vaddr;
      nxt.lrsc_unit.reserve_valid = false;
      l.tlb = 2;
    } else {
      l.tlb = 1;
    }
  }
}

void RealLsu::tlb_in_stq() {
  for (int k = 0; k < STA; k++) {
    PSta &s1 = pw.s1[k];
    if (!s1.v) continue;
    PStq &s = pnxt.stq[s1.stq];
    const auto &resp = tlb_resp_.stq_resp[k];
    if (!resp.valid || resp.entry_idx != s1.stq || resp.result == MMUResultType::MISS) {
      s.trp = 1;  // retry through the STA replay register
      continue;
    }
    if (resp.result == MMUResultType::HIT) {
      s.pa_v = 1;
      s.paddr = resp.paddr;
      s.mmio = lsu_is_mmio_addr(resp.paddr);
      if (s.data_v && !s.wbp && !s.wbd) s.wbp = 1;
    } else {  // page fault: completes without data, raised at commit
      s.pf = 1;
      nxt.lrsc_unit.reserve_valid = false;
      if (!s.wbp && !s.wbd) s.wbp = 1;
    }
    pw.ev_sta |= (stq_mask_t)1 << s1.stq;
    if (s1.replay) continue;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// L1: partial CAM over the STQ (parallel with the DCache read), MMIO issue
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::stlf() {
  const lp::LsuPipeState &pcur = pcur_;
  for (int k = 0; k < LD; k++) {
    PL1 &l = pw.l1[k];
    if (!l.v || !l.hit || l.mmio) continue;
    const PLdq &e = pnxt.ldq[l.ldq];
    PLdq tmp = e;
    tmp.paddr = l.paddr;
    stq_mask_t ov = 0;
    for (int q = 0; q < STA; q++)
      if (pw.s1[q].v && !pw.s1[q].replay) ov |= (stq_mask_t)1 << pw.s1[q].stq;
    const CamRes c = stq_cam(pcur, pnxt, ov, tmp, /*full=*/!l.need_tlb, /*spec_ok=*/!e.second && !e.lrsc && !e.wait);
    pnxt.ldq[l.ldq].spec = c.unk;
    pnxt.ldq[l.ldq].skip = c.skip;
    l.cand_v = c.v;
    l.cand = c.idx;
  }

  // MMIO loads: only the oldest instruction, with no older store left in the STQ, one on the bus at a time
  for (int i = 0; i < LDQ_SIZE; i++) {
    PLdq &e = pnxt.ldq[i];
    if (e.st != L_MMW || nxt.uncached_unit.valid) continue;
    if (pcur.ldq[i].st != L_MMW) continue;  // M: only entries that were already MMW at the start of the cycle
    if (!lsu_mmio_is_oldest_unfinished(in.rob_bcast, e.rob_idx)) continue;
    const uint32_t h = (pcur.stq_head_flag ? STQ_SIZE : 0) + pcur.stq_head;
    const uint32_t b = (e.snap_flag ? STQ_SIZE : 0) + e.snap_idx;
    if ((b + 2 * STQ_SIZE - h) % (2 * STQ_SIZE) != 0) continue;
    e.st = L_MMR;
    UncachedUnit &u = nxt.uncached_unit;
    u.valid = true;
    u.is_load = true;
    u.addr = e.paddr;
    u.func3 = e.func3;
    u.idx = i;
  }
}
