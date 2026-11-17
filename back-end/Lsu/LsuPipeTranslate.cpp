// The staged LSU: translate phase functions (see RealLsuPipe.cpp, docs/LSU_PIPE.md).
#include "LsuPipeInternal.h"
#include "AguPorts.h"

// ---------------------------------------------------------------------------------------------------------------
// AGU arrivals: STD write, STA (va), load (L1 input)
// ---------------------------------------------------------------------------------------------------------------
void RealLsu::exe2lsu() {
  const lp::LsuPipeState &pcur = pcur_;
  ExeLsuIO x = (*in.exe2lsu);
  if (CFG_LSU_AGU_REG) {   // the AGU results go through a register: this cycle's L1 input is last cycle's AGU output (if still alive)
    for (int i = 0; i < LSU_AGU_COUNT; i++) {
      agu_d_[i] = x.agu_req[i];
      x.agu_req[i] = agu_q_[i];
      if (!x.agu_req[i].valid) continue;
      const MicroOp &u = x.agu_req[i].uop;
      bool alive = false;
      if (u.op == UOP_LOAD) alive = u->ldq_idx < LDQ_SIZE && pcur.ldq[u->ldq_idx].st == L_ALLOC;
      else if (u.op == UOP_STA) alive = u->stq_idx < STQ_SIZE && stq_rank(u->stq_idx, pcur.stq_head) < pcur.stq_count;
      if (!alive || in.rob_bcast->flush) x.agu_req[i].valid = false;
    }
  }
  for (int i = 0; i < LSU_SDU_COUNT; i++) {
    if (!x.sdu_req[i].valid) continue;
    const MicroOp &inst = x.sdu_req[i].uop;
    if (inst->stq_idx >= STQ_SIZE) continue;
    PStq &e = pnxt.stq[inst->stq_idx];
    if (e.rob_flag != inst->rob_flag || e.rob_idx != inst->rob_idx) continue;
    e.data = inst.result;
    e.data_v = 1;
    pw.ev_data |= (stq_mask_t)1 << inst->stq_idx;
    if (e.pa_v && !e.wbp && !e.wbd) e.wbp = 1;
  }
  agu_raw_ = raw_ld_ = raw_st_ = 0;
  for (int i = 0; i < LSU_AGU_COUNT; i++) {
    if (!(CFG_LSU_SLOT_FIXED || CFG_LSU_FREEPORT_RAW ? x.agu_req[i].raw : x.agu_req[i].valid)) continue;   // (raw: occupied, dead or not)
    if (x.agu_req[i].uop.op == UOP_LOAD) { agu_raw_ |= 1u << agu::ld_port(i); raw_ld_ |= 1u << i; }
    else if (x.agu_req[i].uop.op == UOP_STA) { agu_raw_ |= 1u << agu::st_port(i - LSU_LDU_COUNT); raw_st_ |= 1u << (i - LSU_LDU_COUNT); }
  }
  int ns = 0;
  int vio_new = 0;  // loads marked as violating in this cycle
  uint64_t vio_set = 0;
  for (int i = 0; i < LSU_AGU_COUNT; i++) {
    if (!(x.agu_req[i].valid && x.agu_req[i].uop.op == UOP_STA)) continue;
    const MicroOp &inst = x.agu_req[i].uop;
    if (inst->stq_idx >= STQ_SIZE) continue;
    PStq &e = pnxt.stq[inst->stq_idx];
    if (e.rob_flag != inst->rob_flag || e.rob_idx != inst->rob_idx) continue;
    e.va_v = 1;
    e.vaddr = inst.result;
    {  // a store whose base register was stale: its data goes to the stale address (never to the true one)
      const tb::Fact *f = tbhook::fact_by_rob(inst->rob_idx);
      ms_stq_[inst->stq_idx].atnt = f && f->is_store && f->vaddr != static_cast<xlen_t>(inst.result);
    }
    e.pa_v = 0;
    e.paddr = 0;
    e.func3 = inst->func3;
    e.dest_preg = inst->dest_preg;
    e.id = inst.id;
    e.lrsc = inst->is_atomic && ((inst->func7 >> 2) == AmoOp::SC);
    e.trp = 0;
    LP_PORT_CHECK(ns + 1, LSU_STA_COUNT, "sta_agu");
    const int ss = CFG_LSU_SLOT_FIXED ? i - LSU_LDU_COUNT : ns;
    pw.s1[ss].v = true;
    pw.s1[ss].stq = inst->stq_idx;
    pw.s1[ss].phys = agu::st_port(i - LSU_LDU_COUNT);
    ns++;
    pw.ev_sta |= (stq_mask_t)1 << inst->stq_idx;  // the partial address is known from now on
    {  // memory-order check: younger loads that passed this store while its address was unknown
      const uint32_t j = inst->stq_idx;
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
        if (range_rel(l.vaddr, ld_width(l.uop().func3), inst.result, sw, 12) == DISJOINT) continue;
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
    if (inst->ldq_idx >= LDQ_SIZE) continue;
    PLdq &e = pnxt.ldq[inst->ldq_idx];
    if (e.uop().rob_flag != inst->rob_flag || e.uop().rob_idx != inst->rob_idx) continue;
    e.vaddr = inst.result;
    {  // a load whose base register was stale reads the stale address
      const tb::Fact *f = tbhook::fact_by_rob(inst->rob_idx);
      ms_ldq_[inst->ldq_idx].atnt = f && f->is_load && f->vaddr != static_cast<xlen_t>(inst.result);
    }
    e.id = inst.id;
    e.pa_v = 0;
    e.paddr = 0;
    e.illegal = inst.illegal_inst;
    e.st = L_PIPE;
    e.snap_idx = inst->stq_idx;
    e.snap_flag = inst->stq_flag;
    e.lrsc = is_amo_lr_uop(inst);
    LP_PORT_CHECK(nl + 1, LD, "load_agu");
    const int ls = CFG_LSU_SLOT_FIXED ? i : nl;
    pw.l1[ls].v = true;
    pw.l1[ls].need_tlb = true;
    pw.l1[ls].ldq = inst->ldq_idx;
    pw.l1[ls].phys = agu::ld_port(i);
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
  const bool fence = in.rob_bcast->fence, cancel = !fence && (in.rob_bcast->flush || in.dec_bcast->mispred);
  // the DTLB has one port per physical AGU: a request is on the port of its AGU, a replay on the free port it took; the ports are
  // served in port order (the priority of the walk picks) and every request carries its own type
  dtlb.comb(fence, cancel, tlb_req_, tlb_resp_, fence || cancel ? 0 : agu::PORTS);
  if (fence || cancel) for (auto &r : tlb_resp_) r = {};
  tlb_in_ldq();
  tlb_in_stq();
}

void RealLsu::tlb_out() {
  const lp::LsuPipeState &pcur = pcur_;
  for (auto &r : tlb_req_) r = {};

  // The DTLB ports the new requests of this cycle hold; a replay that needs the DTLB takes a free port it may use, otherwise its
  // replay register keeps it for the next cycle
  uint32_t used = (CFG_LSU_FREEPORT_RAW || CFG_LSU_SLOT_FIXED) ? agu_raw_ : 0;
  for (int k = 0; k < LD; k++) if (pw.l1[k].v) used |= 1u << pw.l1[k].phys;
  for (int k = 0; k < STA; k++) if (pw.s1[k].v) used |= 1u << pw.s1[k].phys;
  auto free_port = [&](bool ld) {
    for (int q = 0; q < agu::PORTS; q++)
      if (!(used >> q & 1) && (ld ? agu::ld_can(q) : agu::st_can(q))) return q;
    return -1;
  };
  int n = 0;
  while (n < LD && pw.l1[n].v) n++;
  uint32_t lslot = raw_ld_;   // CFG_LSU_SLOT_FIXED: the load slots taken (valid AGU requests, then the replays)
  for (int r = 0; r < lp_rp_ports; r++) {
    if (CFG_LSU_SLOT_FIXED) { n = 0; while (n < LD && (lslot >> n & 1)) n++; }
    if (n >= LD) continue;
    int src = -1, ph = -1;
    if (pcur.rp[r].v) {
      src = pcur.rp[r].ldq;
      if (pnxt.ldq[src].st == L_PIPE && !pnxt.ldq[src].pa_v) {
        ph = free_port(true);
        if (ph < 0) continue;   // no free DTLB port this cycle
        used |= 1u << ph;
      }
      pnxt.rp[r].v = 0;
    }
    if (src < 0) continue;
    PLdq &e = pnxt.ldq[src];
    if (e.st != L_PIPE) continue;
    pw.l1[n].v = true;
    pw.l1[n].phys = ph;
    pw.l1[n].replay = true;
    pw.l1[n].need_tlb = !e.pa_v;
    pw.l1[n].ldq = src;
    lslot |= 1u << n;
    n++;
  }
  for (int k = 0; k < LD; k++) {
    if (!pw.l1[k].v || !pw.l1[k].need_tlb) continue;
    tlb_req_[pw.l1[k].phys] = {true, pnxt.ldq[pw.l1[k].ldq].vaddr, 1};
  }

  int ns = 0;
  while (ns < STA && pw.s1[ns].v) ns++;
  if (CFG_LSU_SLOT_FIXED) { ns = 0; while (ns < STA && (raw_st_ >> ns & 1)) ns++; }
  int sph = -1;
  bool take_srp = ns < STA && pcur.srp.v;
  if (take_srp) {
    sph = free_port(false);
    if (sph < 0) take_srp = false;   // no free DTLB port: the replay register keeps the store
  }
  if (take_srp) {  // STA replay register takes a free STA port
    const uint32_t j = pcur.srp.ldq;
    pnxt.srp.v = 0;
    if (stq_rank(j, pnxt.stq_head) < pnxt.stq_count && pnxt.stq[j].va_v && !pnxt.stq[j].pa_v && !pnxt.stq[j].pf) {
      pw.s1[ns].v = true;
      pw.s1[ns].phys = sph;
      pw.s1[ns].replay = true;
      pw.s1[ns].stq = j;
      ns++;
    }
  }
  for (int k = 0; k < STA; k++) {
    if (!pw.s1[k].v) continue;
    tlb_req_[pw.s1[k].phys] = {true, pnxt.stq[pw.s1[k].stq].vaddr, 2};
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
    const TlbMmu::Resp &resp = tlb_resp_[l.phys];
    if (!resp.valid) {
      l.tlb = 1;
    } else if (resp.result == TlbMmu::Result::OK) {
      e.pa_v = 1;
      e.paddr = resp.paddr;
      e.mmio = lsu_is_mmio_addr(resp.paddr);
      l.hit = true;
      l.mmio = e.mmio;
      l.paddr = resp.paddr;
    } else if (resp.result == TlbMmu::Result::FAULT) {
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
    const TlbMmu::Resp &resp = tlb_resp_[s1.phys];
    if (!resp.valid || resp.result == TlbMmu::Result::RETRY) {
      s.trp = 1;  // retry through the STA replay register
      continue;
    }
    if (resp.result == TlbMmu::Result::OK) {
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
    ms_ldq_[l.ldq].skip = c.skip;
    l.cand_v = c.v;
    l.cand = c.idx;
  }

  // MMIO loads: only the oldest instruction, with no older store left in the STQ, one on the bus at a time
  for (int i = 0; i < LDQ_SIZE; i++) {
    PLdq &e = pnxt.ldq[i];
    if (e.st != L_MMW || nxt.uncached_unit.valid) continue;
    if (pcur.ldq[i].st != L_MMW) continue;  // M: only entries that were already MMW at the start of the cycle
    if (!lsu_mmio_is_oldest_unfinished(in.rob_bcast, e.uop().rob_idx)) continue;
    const uint32_t h = (pcur.stq_head_flag ? STQ_SIZE : 0) + pcur.stq_head;
    const uint32_t b = (e.snap_flag ? STQ_SIZE : 0) + e.snap_idx;
    if ((b + 2 * STQ_SIZE - h) % (2 * STQ_SIZE) != 0) continue;
    e.st = L_MMR;
    UncachedUnit &u = nxt.uncached_unit;
    u.valid = true;
    u.is_load = true;
    u.addr = e.paddr;
    u.func3 = e.uop().func3;
    u.idx = i;
  }
}
