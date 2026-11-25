// The staged LSU (the only LSU). Design and rules: docs/LSU_PIPE.md.
//
// Hardware reading: pcur = registers, pnxt = next register values (pnxt == pcur at the start of a cycle, seq() copies
// pnxt to pcur). Each comb phase reads registers (pcur) and writes next values (pnxt) or per-cycle wires (pw).
// One file per phase (BackTop order), this file has the clock edge and the phases' composition:
//   comb_outputs    LsuPipeOutputs.cpp     cal / lsu2dis / lsu2rob: cycle-start views
//   comb_writeback  LsuPipeWriteback.cpp   ROB commit marks stores committed, MMIO response, DCache responses (STLF
//                                          confirm, hit / forward select, park / replay), write-back ports, STA completions
//   comb_translate  LsuPipeTranslate.cpp   AGU arrivals (STD write, STA, load), DTLB lookup, partial-address STQ CAM
//   comb_requests   LsuPipeRequests.cpp    allocation, flush / mispredict, DCache / MMIO requests, committed-store drain,
//                                          STQ retire, wake-up pass, replay-register fill
#include "LsuPipeInternal.h"

// pcur = pnxt. The staged LSU's registers are 12 KB, nearly all of it the 64 LDQ and 64 STQ entries, of which only a few are live.
//  * LDQ: an entry that is L_EMPTY in both states is dead; the only field read from a dead entry is `epoch` (allocation bumps it), so
//    that one is copied and nothing else.
//  * STQ: only the entries inside the ring window [head, head + count) of either state are live (allocation rewrites an entry completely).
// LSU_SEQ_POISON fills the skipped entries of both states with garbage: a read of a stale field then changes the result.
void RealLsu::seq_pipe() {
  static_assert(std::is_trivially_copyable_v<lp::LsuPipeState>);
  using namespace lp;
  const LsuPipeState &n = pnxt;
  LsuPipeState &c = pcur_;
  for (int i = 0; i < LDQ_SIZE; i++) {
    if (c.ldq[i].st != L_EMPTY || n.ldq[i].st != L_EMPTY) {
      c.ldq[i] = n.ldq[i];
    } else {
#ifdef LSU_SEQ_POISON
      const auto ep = n.ldq[i].epoch;
      std::memset(&c.ldq[i], 0xA5, sizeof(PLdq));
      std::memset(&pnxt.ldq[i], 0xA5, sizeof(PLdq));
      c.ldq[i].st = pnxt.ldq[i].st = L_EMPTY;
      c.ldq[i].epoch = pnxt.ldq[i].epoch = ep;
#else
      c.ldq[i].epoch = n.ldq[i].epoch;
#endif
    }
  }
#ifdef LSU_SEQ_POISON
  bool live[STQ_SIZE] = {};
  for (uint32_t k = 0; k < c.stq_count; k++) live[(c.stq_head + k) % STQ_SIZE] = true;
  for (uint32_t k = 0; k < n.stq_count; k++) live[(n.stq_head + k) % STQ_SIZE] = true;
  for (int i = 0; i < STQ_SIZE; i++) {
    if (live[i]) c.stq[i] = n.stq[i];
    else { std::memset(&c.stq[i], 0xA5, sizeof(PStq)); std::memset(&pnxt.stq[i], 0xA5, sizeof(PStq)); }
  }
#else
  for (uint32_t k = 0; k < c.stq_count; k++) { const uint32_t i = (c.stq_head + k) % STQ_SIZE; c.stq[i] = n.stq[i]; }
  for (uint32_t k = 0; k < n.stq_count; k++) { const uint32_t i = (n.stq_head + k) % STQ_SIZE; c.stq[i] = n.stq[i]; }
#endif
  // Update 256-bit registered STQ word bloom filter from cycle-start live STQ entries + accepted STAs (vaddr[10:3])
  pnxt.stq_bloom[0] = pnxt.stq_bloom[1] = pnxt.stq_bloom[2] = pnxt.stq_bloom[3] = 0;
  for (uint32_t k = 0; k < c.stq_count; k++) {
    const PStq &st = c.stq[(c.stq_head + k) % STQ_SIZE];
    if (st.va_v && !st.suppress) {
      const uint32_t b = (uint32_t(st.vaddr) >> 3) & 0xFFu;
      pnxt.stq_bloom[b >> 6] |= 1ULL << (b & 63u);
    }
  }
  for (int k = 0; k < STA; k++) {
    if (pw.s1[k].v && !pw.s1[k].replay) {
      const uint32_t b = (uint32_t(n.stq[pw.s1[k].stq].vaddr) >> 3) & 0xFFu;
      pnxt.stq_bloom[b >> 6] |= 1ULL << (b & 63u);
    }
  }
  // everything after the two big arrays (write-back buffer, L1/L2/L3 registers, replay registers, pointers)
  char *dp = reinterpret_cast<char *>(&c);
  const char *sp = reinterpret_cast<const char *>(&n);
  const size_t off = reinterpret_cast<const char *>(&c.wb) - dp;
  std::memcpy(dp + off, sp + off, sizeof(LsuPipeState) - off);
}

// outputs from the registers: wakeups, free counts to dispatch, completion to the ROB
void RealLsu::comb_outputs() {
  cal();
  lsu2dis();
  lsu2rob();
}

// committed stores, MMIO and DCache responses, load write-back to the EXU
void RealLsu::comb_writeback() {
  stq_commit();
  mmio_in();
  dcache2lsu();
  lsu2exe();
}

// AGU / STD results in, address translation, store-to-load forwarding
void RealLsu::comb_translate() {
  exe2lsu();
  tlb();
  stlf();
}

// allocation from dispatch, flush, the DCache / MMIO requests, ordering checks
void RealLsu::comb_requests() {
  dis2lsu();
  flush();
  lsu2dcache();
  mmio_out();
  check();
}
