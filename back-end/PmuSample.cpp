// PMU (EXP-013): builds one cycle of the event bus from the signals the modules already drive. Everything here only
// reads; it runs after comb_stage_fire_and_recover(), when the handshakes of the cycle are final.
#include <cstdlib>
#include <cstdio>
#include "BackTop.h"
#include "DataPrefetcher.h"
#include "Pmu.h"
#include "BackPorts.h"
#include "config.h"
#include "util.h"

// The DECODE_WIDTH dispatch slots, each in exactly one bucket (the identity checked in Pmu::accumulate):
//   fired -> FIRE;  killed by a flush/mispredict -> KILLED;  blocked by the backend -> BE_<cause>;  empty -> FE_*.
// As in Intel's top-down method, once a valid slot is blocked all slots from it on are backend-bound (even the empty
// ones: the frontend could not have been used), and the cause is the first resource that stopped it.
void Dispatch::pmu_slots(PmuVec &e, bool recovering, bool recovering_by_flush, int &be_slots) const {
  const Regs &q = q_;
  for (int q = 0; q < IQ_NUM; q++)  // entries in use = size - free (free_slots is what the IQ reports to dispatch); IQ_INT1 counts with IQ_INT
    e.add(static_cast<PmuEvent>(PMU_OCC_IQ_INT + (q == IQ_INT1 ? IQ_INT : q)), GLOBAL_IQ_CONFIG[q].size - in.iss2dis->ready_num[q]);
  const bool kill = in.rob_bcast->flush || in.dec_bcast->mispred;
  const bool recov = recovering || kill;
  const bool by_flush = kill ? static_cast<bool>(in.rob_bcast->flush) : recovering_by_flush;
  bool blocked = false, fired_before = false;
  PmuEvent cause = PMU_SLOT_BE_SERIAL;
  int iq_sub = -1;  // sub-event of an IQ stall
  be_slots = 0;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (out.dis2rob.dis_fire[i]) {
      e.add(PMU_SLOT_FIRE);
      fired_before = true;
      if (is_eliminated_move(q.inst_r[i])) e.add(PMU_MOVE_ELIMINATED);
      continue;
    }
    if (!blocked && q.inst_valid[i] && kill) { e.add(PMU_SLOT_KILLED); continue; }
    if (!blocked && q.inst_valid[i]) {  // first slot the backend did not take: find out why
      blocked = true;
      const InstType type = decode_inst_type(q.inst_r[i]->type);
      const bool serial = (is_CSR(type) || type == AMO) && (!in.rob2dis->empty || fired_before);
      if (in.rob2dis->stall) cause = PMU_SLOT_BE_FLUSH_WAIT;
      else if (!in.rob2dis->ready) cause = PMU_SLOT_BE_ROB_FULL;
      else if (serial) cause = PMU_SLOT_BE_SERIAL;
      else if (!out.dis2rob.valid[i] && is_load(q.inst_r[i]) && !iq_blk_width[i]) cause = PMU_SLOT_BE_LDQ_FULL;
      else if (!out.dis2rob.valid[i] && is_store(q.inst_r[i])) cause = PMU_SLOT_BE_STQ_FULL;
      else if (!fit_[i] || iq_blk_width[i]) cause = PMU_SLOT_BE_IQ_FULL;
      else cause = PMU_SLOT_BE_SERIAL;  // behind an exclusive instruction (AMO/WFI) that fired this cycle
      if (cause == PMU_SLOT_BE_IQ_FULL && iq_blk[i] >= 0)
        iq_sub = (iq_blk_width[i] ? PMU_SLOT_BE_IQ_WIDTH_INT : PMU_SLOT_BE_IQ_SPACE_INT) + (iq_blk[i] == IQ_INT1 ? IQ_INT : iq_blk[i]);
    }
    if (blocked) {
      e.add(cause);
      if (cause == PMU_SLOT_BE_IQ_FULL && iq_sub >= 0) e.add(static_cast<PmuEvent>(iq_sub));
      be_slots++;
      continue;
    }
    // an empty slot (no valid instruction, nothing blocked)
    e.add(PMU_SLOT_FE_EMPTY);
    if (recov) e.add(by_flush ? PMU_SLOT_FE_RECOV_FLUSH : PMU_SLOT_FE_RECOV_MISPRED);
    else if (g_pmu_front.icache_miss_busy) e.add(PMU_SLOT_FE_ICACHE);
    else if (g_pmu_front.itlb_miss) e.add(PMU_SLOT_FE_ITLB);
    else {
      e.add(PMU_SLOT_FE_OTHER);
      if (g_pmu_back.ren_no_preg) e.add(PMU_SLOT_FE_OTHER_NOPREG);
      else if (g_pmu_back.idu_no_brtag) e.add(PMU_SLOT_FE_OTHER_NOBRTAG);
    }
  }
}

void BackTop::pmu_sample() {
  Pmu &p = ctx->perf.pmu;
  if (!p.enabled || (ctx->is_ckpt && !ctx->perf.perf_start)) return;  // checkpoints: only the measurement window
  PmuVec &e = p.cur;
  e = PmuVec{};
  e.add(PMU_CYCLES);

  // ---- slots ----
  int be_slots = 0;
  dis->pmu_slots(e, p.recovering, p.recov_by_flush, be_slots);
  e.add(PMU_SLOT_BE, be_slots);
  Uop head{};
  int rob_occ = 0;
  const bool head_found = rob->pmu_head_incomplete(head, rob_occ);

  // ---- commit ----
  int committed = 0;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    const auto &ce = rob->out.rob_commit.commit_entry[i];
    if (!ce.valid || rob->out.rob_bcast.interrupt) continue;
    committed++;
    const Uop &u = ce.uop;
    const InstType t = decode_inst_type(u->type);
    const bool is_ret = u->tma.is_ret;
    if (t == LOAD || t == AMO) e.add(PMU_COMMIT_LOAD);
    if (t == STORE || t == AMO) e.add(PMU_COMMIT_STORE);
    if (t == BR) e.add(PMU_COMMIT_BRANCH);
    if (t == JAL || t == JALR) e.add(PMU_COMMIT_JUMP);
    if (is_ret) e.add(PMU_COMMIT_RET);
    if (u.mispred) e.add(t == BR ? PMU_COMMIT_MISPRED_BR : is_ret ? PMU_COMMIT_MISPRED_RET : PMU_COMMIT_MISPRED_JALR);
    if (p.pcprof) {
      PmuPcStat &s = p.pc_stat[u->dbg.pc];
      s.commits++;
      s.type = static_cast<uint8_t>(t);
      s.mispreds += u.mispred ? 1 : 0;
    }
  }
  e.add(PMU_INSTRET, committed);
  if (committed == 0) e.add(PMU_CYC_COMMIT_ZERO);
  if (committed == COMMIT_WIDTH) e.add(PMU_CYC_COMMIT_FULL);

  // ---- head-of-ROB view of the backend-stalled slots (and the per-PC blame) ----
  if (be_slots > 0) {
    const bool mem = head_found && rob->out.rob2dis.tma.head_is_memory, miss = head_found && rob->out.rob2dis.tma.head_is_miss;
    e.add(miss ? PMU_SLOT_BE_HEAD_MISS : mem ? PMU_SLOT_BE_HEAD_MEM : head_found ? PMU_SLOT_BE_HEAD_CORE : PMU_SLOT_BE_HEAD_DONE, be_slots);
  }
  if (p.pcprof && head_found && committed < COMMIT_WIDTH) {
    PmuPcStat &s = p.pc_stat[head->dbg.pc];
    s.head_cycles++;
    s.head_miss_cycles += rob->out.rob2dis.tma.head_is_miss ? 1 : 0;
    s.stall_slots += be_slots;
    s.type = static_cast<uint8_t>(decode_inst_type(head->type));
  }

  // ---- control flow / recovery ----
  if (idu->out.dec_bcast.mispred) e.add(PMU_BR_MISPRED);
  if (rob->out.rob_bcast.flush) e.add(rob->out.rob_bcast.exception ? PMU_FLUSH_EXCEPTION : PMU_FLUSH_OTHER);
  if (p.recovering) e.add(PMU_CYC_RECOVERING);
  // recovery state for the next cycle: a redirect starts it, the first valid instruction in the dispatch stage ends it
  bool any_valid = false;
  for (int i = 0; i < DECODE_WIDTH; i++) any_valid |= static_cast<bool>(dis->q().inst_valid[i]);
  if (rob->out.rob_bcast.flush || idu->out.dec_bcast.mispred) {
    p.recovering = true;
    p.recov_by_flush = rob->out.rob_bcast.flush;
  } else if (any_valid) {
    p.recovering = false;
  }

  // ---- frontend ----
  if (g_pmu_front.feo_ready) {
    e.add(PMU_FEO_CYC);
    if (g_pmu_front.feo_n > 0) { e.add(PMU_FEO_GROUPS); e.add(PMU_FEO_INSTS, g_pmu_front.feo_n); }
    if (g_pmu_front.feo_cause) e.add(static_cast<PmuEvent>(g_pmu_front.feo_cause), FETCH_WIDTH - g_pmu_front.feo_n);
  }
  if (g_pmu_front.icache_req_fire) e.add(PMU_ICACHE_ACCESS);
  if (g_pmu_front.icache_miss_issue) e.add(PMU_ICACHE_MISS);
  if (g_pmu_front.icache_miss_busy) e.add(PMU_CYC_ICACHE_MISS_BUSY);
  if (g_pmu_front.itlb_access) e.add(PMU_ITLB_ACCESS);
  if (g_pmu_front.itlb_miss) e.add(PMU_ITLB_MISS);

  // ---- memory ----
  static uint64_t prev_hit = 0, prev_miss = 0, prev_issued = 0, prev_useful = 0, prev_late = 0;
  const PfStats &pf = g_pf_stats;
  e.add(PMU_DC_LD_HIT, pf.demand_hit - prev_hit);
  e.add(PMU_DC_LD_MISS, pf.demand_miss - prev_miss);
  e.add(PMU_PF_ISSUED, pf.issued - prev_issued);
  e.add(PMU_PF_USEFUL, pf.useful - prev_useful);
  e.add(PMU_PF_LATE, pf.late - prev_late);
  prev_hit = pf.demand_hit; prev_miss = pf.demand_miss; prev_issued = pf.issued; prev_useful = pf.useful; prev_late = pf.late;
  const unsigned outstanding = static_cast<unsigned>(lsu->out.lsu2rob.tma.miss_mask.count());
  e.add(PMU_DC_LD_MISS_OUT, outstanding);
  if (outstanding) e.add(PMU_CYC_LD_MISS_OUT);

  // ---- occupancy ----
  const int ldq_occ = LDQ_SIZE - static_cast<int>(lsu->out.lsu2dis.ldq_free), stq_occ = STQ_SIZE - static_cast<int>(lsu->out.lsu2dis.stq_free);
  e.add(PMU_OCC_ROB, rob_occ);
  e.add(PMU_OCC_LDQ, ldq_occ);
  e.add(PMU_OCC_STQ, stq_occ);

  // ---- histograms (model-only) ----
  p.h_commit[committed]++;
  p.h_rob[std::min(16, rob_occ * 16 / ROB_NUM)]++;
  p.h_ldq[std::min(16, ldq_occ * 16 / LDQ_SIZE)]++;
  p.h_stq[std::min(16, stq_occ * 16 / STQ_SIZE)]++;
  p.h_mlp[std::min<unsigned>(LDQ_SIZE, outstanding)]++;

  p.accumulate();
}
