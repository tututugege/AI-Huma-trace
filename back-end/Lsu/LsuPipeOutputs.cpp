// The staged LSU: outputs phase functions (see RealLsuPipe.cpp, docs/LSU_PIPE.md).
#include "LsuPipeInternal.h"
#include "DseCap.h"

void RealLsu::cal() {
  const lp::LsuPipeState &pcur = pcur_;
  pw = {};
  mdp_.write_pending();
  {  // train one flagged live entry (flags of earlier cycles); entries that died are dropped
    for (int i = 0; i < LDQ_SIZE && mdp_.trn; i++)  // the flag of an entry that died is dropped (the RTL clears it when the entry is EMPTY)
      if ((mdp_.trn >> i & 1) && pcur.ldq[i].st == L_EMPTY) mdp_.trn &= ~(1ULL << i);
    for (int i = 0; i < LDQ_SIZE && mdp_.trn; i++)
      if (mdp_.trn >> i & 1) { mdp_.trn &= ~(1ULL << i); mdp_.pend.push_back(MemDepPred::idx(pcur.ldq[i].pc())); break; }
  }
  for (int k = 0; k < LD; k++) pnxt.l12[k] = {};  // L1/L2a/L2b/L3 registers are rewritten every cycle
  for (int k = 0; k < LD; k++) pnxt.l2b[k] = {};
  for (int k = 0; k < LD; k++) pnxt.l3[k] = {};
  for (int k = 0; k < LD; k++) pnxt.cn[k] = {};
  for (auto &w : out.lsu_awake.wake) w = {};
  out.lsu_awake.spec_l1_mask = 0;
  for (int k = 0; k < LD; k++) {
    out.lsu_awake.cancel[k].valid = pcur.cn[k].v;
    out.lsu_awake.cancel[k].preg = pcur.cn[k].preg;
  }
  ms_capture_stores();
}

void RealLsu::lsu2rob() {
  const lp::LsuPipeState &pcur = pcur_;
  out.lsu2rob = {};
  out.lsu2rob.committed_store_pending = pcur.stq_ccount != 0;
  std::bitset<ROB_NUM> m;  // ROB_NUM bits: a 64-bit mask lost the entries above 63 
  for (const PLdq &l : pcur.ldq)
    if (l.miss && l.st != L_EMPTY) m.set(l.uop().rob_idx);
  out.lsu2rob.tma.miss_mask = m;
}

void RealLsu::lsu2dis() {
  const lp::LsuPipeState &pcur = pcur_;
  out.lsu2dis = {};
  const uint32_t head = pcur.stq_head, count = pcur.stq_count;
  out.lsu2dis.stq_tail = (head + count) % STQ_SIZE;
  out.lsu2dis.stq_tail_flag = stq_tail_flag(head, count, pcur.stq_head_flag);
  uint32_t empty = 0;
  for (const PLdq &l : pcur.ldq) empty += l.st == L_EMPTY;
  out.lsu2dis.ldq_free = dse_free(g_dse.ldq, LDQ_SIZE - int(empty));
  out.lsu2dis.stq_free = dse_free(g_dse.stq, count);
  uint32_t n = 0;
  for (int i = 0; i < g_dse.ldq && n < (uint32_t)MAX_LDQ_DISPATCH_WIDTH; i++)
    if (pcur.ldq[i].st == L_EMPTY) {
      out.lsu2dis.ldq_alloc_idx[n] = i;
      out.lsu2dis.ldq_alloc_valid[n++] = true;
    }
}

// LSU_MS_DATA: what the trace says the stores allocated in an earlier cycle write (the fact leaves the bridge when the store
// commits, so it is taken now).
void RealLsu::ms_capture_stores() {
  for (uint32_t k = 0; k < pnxt.stq_count; k++) {
    const uint32_t j = (pnxt.stq_head + k) % STQ_SIZE;
    MsStq &m = ms_stq_[j];
    if (m.init) continue;
    m.init = true;
    const tb::Fact *f = tbhook::fact_by_rob(pnxt.stq[j].rob_idx);
    if (!f || !f->is_store || !f->has_mem_write || f->trapped || f->blocked != tb::Blocked::None) continue;
    m.v = true;
    m.amo = (f->inst & 0x7f) == 0x2f;
    m.w = f->mem_waddr;
    m.old_ = f->mem_old;
    m.new_ = f->mem_wdata;
    m.rs2 = f->rs2_value;
    m.off = f->vaddr & (XLEN / 8 - 1);
    m.sz = f->size;
  }
}
