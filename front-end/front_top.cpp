#include REFCPU_RVC_H   // the function model's RVC table (Makefile: from the same tree as the librefcpu API header)
#include "fatal.h"
#include "trace_hook.h"
#include "BPU/BPU.h"
#include "BPU/BPU_pipe.h"
#include "Pmu.h"
#include "PreIduQueue.h"
#include "front_IO.h"
#include "front_core.h"
#include "front_module.h"
#include "predecode.h"
#include "predecode_checker.h"
#include "train_IO.h"
#include <RISCV.h>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <unordered_set>
#include <optional>
#include <vector>
#include <cstdlib>

static pc_t sequential_next_pc(pc_t fetch_pc, const wire<1> *inst_valid) {
  uint32_t valid_prefix = 0;
  while (valid_prefix < FETCH_WIDTH && inst_valid[valid_prefix]) {
    ++valid_prefix;
  }
  pc_t next_pc = fetch_pc + valid_prefix * SLOT_BYTES;   // RVC=1 (S2): next pc after the last valid instruction, gaps allowed
  const pc_t line_mask = ~static_cast<pc_t>(ICACHE_LINE_SIZE - 1u);
  if ((next_pc & line_mask) != (fetch_pc & line_mask)) {
    next_pc &= line_mask;
  }
  return next_pc;
}

// ---- RVC=1 head fix (C910-style carry); the RTL counterpart is rtl/frontend/front_head_fix.sv ----
// Slot / pc conventions of RVC=1: a slot is a 2-byte position (FETCH_WIDTH = 16 per 32-byte block), slot i of a group has pc base + 2 * i.  A 32-bit
// instruction that starts in the last slot of a group is not delivered by that group: its first halfword is carried (lh_data) and the next sequential
// group (pc[0] == lh_pc) assembles it in its slot 0, with pc = group pc - 2 (the only slot whose pc is not base + 2 * i) and is_rvc = 0; the instruction
// chain of that group then starts at slot 1 (start_b).  Slots that do not start an instruction are invalid (NON_BRANCH).
struct RvcHeadFix {
  bool carry_used = false;   // slot 0 was assembled from the carried halfword
  int n = 0;                 // slots of the group (valid prefix before the fix)
  bool straddle = false;     // the last slot starts a 32-bit instruction that this group cannot complete
  uint16_t low = 0;          // ... and this is its first halfword
};
[[maybe_unused]] static RvcHeadFix rvc_head_fix(InstFifoOut &head, PtabOut &ptab, bool lh_valid, pc_t lh_pc, uint16_t lh_data) {
  RvcHeadFix r;
  auto drop = [&](int i) { head.inst_valid[i] = false; head.predecode_type[i] = PREDECODE_NON_BRANCH; head.predecode_target_address[i] = 0; };
  r.carry_used = lh_valid && lh_pc == head.pc[0] && head.inst_valid[0];
  if (r.carry_used) {
    const pc_t ipc = head.pc[0] - SLOT_BYTES;
    const uint32_t word = (uint32_t(head.raw16[0]) << 16) | lh_data;
    head.instructions[0] = word;
    head.is_rvc[0] = false;
    head.pc[0] = ipc;
    ptab.predict_base_pc[0] = ipc;
    const PredecodeResult res = predecode(word, ipc);
    head.predecode_type[0] = res.type;
    head.predecode_target_address[0] = res.target_address;
  }
  const wire<1> *chain = r.carry_used ? head.start_b : head.start_a;
  while (r.n < FETCH_WIDTH && head.inst_valid[r.n]) r.n++;
  for (int i = 0; i < FETCH_WIDTH; i++) {
    if (!head.inst_valid[i] || chain[i] || (r.carry_used && i == 0)) continue;
    drop(i);
  }
  const int last = r.n - 1;   // the last slot of the group: a 32-bit start there lacks its upper half
  // a predicted-taken straddler whose upper half is in the same cache line is delivered complete: dropping it would lose the prediction (the fetch has
  // already moved to the target, nothing would ever complete the instruction) and cost a predecode flush
  if (last >= (r.carry_used ? 1 : 0) && head.inst_valid[last] && !head.is_rvc[last] && !head.page_fault_inst[last] && chain[last] &&
      ptab.predict_dir[last] && head.up_ok) {
    const uint32_t word = (uint32_t(head.up16) << 16) | head.raw16[last];
    head.instructions[last] = word;
    const PredecodeResult res = predecode(word, head.pc[last]);
    head.predecode_type[last] = res.type;
    head.predecode_target_address[last] = res.target_address;
    head.seq_next_pc = head.pc[last] + 2 * SLOT_BYTES;   // the instruction is complete now: the sequential continuation is behind it, not in the middle of it
  } else if (last >= (r.carry_used ? 1 : 0) && head.inst_valid[last] && !head.is_rvc[last] && !head.page_fault_inst[last] && chain[last]) {
    r.straddle = true;
    r.low = head.raw16[last];
    drop(last);
  }
  // Explicit 1-bit pf_inst_hi for cross-page 2nd-halfword instruction page fault (ROB stval = pc + 2)
  for (int i = 0; i < FETCH_WIDTH; i++) {
    head.pf_inst_hi[i] = head.inst_valid[i] && head.page_fault_inst[i] && (i == 0 && r.carry_used);
    if (head.inst_valid[i] && head.page_fault_inst[i]) head.instructions[i] = INST_NOP;
  }
  return r;
}

// ============================================================================
// Small combinational blocks
// ============================================================================


// Alignment fingerprint of a Unified FTQ S2 write (see BPU/fpr.h)
static void fpr_ptab(const FtqFrontS2Req &s2) {
  if (!fpr::on()) return;
  uint32_t m = 0;
  for (int i = 0; i < FETCH_WIDTH; i++) m |= uint32_t(s2.pred_dir[i]) << i;
  fpr::pt(static_cast<uint32_t>(s2.slot_pc[0]), static_cast<uint32_t>(s2.next_pc), m);
}

static void checker_input(const InstFifoOut &fifo_out, const PtabOut &ptab_out,
                                     PredecodeCheckerIn &checker_in) {
  checker_in = {};
  for (int i = 0; i < FETCH_WIDTH; i++) {
    checker_in.predict_dir[i] = ptab_out.predict_dir[i];
    checker_in.predecode_type[i] = fifo_out.predecode_type[i];
    checker_in.predecode_target_address[i] = fifo_out.predecode_target_address[i];
  }
  checker_in.seq_next_pc = fifo_out.seq_next_pc;
  checker_in.predict_next_fetch_address = ptab_out.predict_next_fetch_address;
}


// ---- the front end's control (front-end/front_ctrl.h) ----

// Flush / refetch of the cycle and the BPU pipeline's last-stage decision (from the registers).
void FrontControl::control() {
  out.front_state.valid = false;
  out.pipe_dec = {};
  out.global_reset = in.in->ctrl.reset;
  out.global_refetch = in.in->redirect.valid || in.rd->q.predecode_refetch;
  // BPU_PIPE (S2->S3 split): drop youngest squash_n (0..2) fetch addresses on S3 override;
  // ICache may pop ifu_ptr whenever fa_count() > bpu_pipe->hold_n() (0 extra refetch wait on post-refetch head!).
  bpu_pipe->decide(out.global_refetch, out.pipe_dec);
  out.pipe_head_ok = ftq ? (ftq->fa_count() > bpu_pipe->hold_n()) : true;
}

// The read enables of the Unified FTQ / inst_fifo and the S2 write enable (no ptab_bypass, no front2back_fifo).
static inline bool fe_icache_pipe() {
  static const bool on = (!std::getenv("FE_ICACHE_PIPE") || std::atoi(std::getenv("FE_ICACHE_PIPE")) != 0);
  return on;
}

void FrontControl::enables() {
  const bpuf::OutputPayload &bpu_output = *in.bpu_output;

  const bool icache_ready = icache->read_ready(in.in->ctrl.itlb_flush || in.in->ctrl.fence_i, out.global_reset);
  const int f1_inflight = fe_icache_pipe() ? int(in.rd->q.ic_f1_v) : 0;
  const bool inst_fifo_room = int(in.rd->fifo_rd_snapshot.size) + int(in.rd->q.pend_valid) + f1_inflight < INSTRUCTION_FIFO_SIZE;
  out.s2_wr_en = bpu_output.PTAB_write_enable && !out.global_reset && !out.global_refetch;
  out.fetch_addr_read_enable = icache_ready && ftq && !ftq->fa_empty() &&
                               !out.global_reset && !out.global_refetch && out.pipe_head_ok && inst_fifo_room;
  out.fetch_addr_head = {};
  if (out.fetch_addr_read_enable) {
    const FTQEntry &e = ftq->ifu_head();
    out.fetch_addr_head.read_valid = true;
    out.fetch_addr_head.fetch_address = e.fetch_addr;
    out.fetch_addr_head.pair = e.fetch_pair;
  }
  out.f2b_read_enable = in.in->ctrl.fifo_read_enable;
  out.fifo_read_enable = out.f2b_read_enable && !in.rd->q.fifo_empty && ftq && !ftq->s2_empty() &&
                         !out.global_reset && !out.global_refetch;
}

// The front end's registers of the next cycle, the Unified FTQ write requests, the output toward the back end, the PMU delivery split.
void FrontControl::finish() {
  const bpuf::OutputPayload &bpu_output = *in.bpu_output;
  const bool bpu_can_run = *in.bpu_can_run;
  const FetchAddrFifoOut &fetch_addr_head = out.fetch_addr_head;
  const InstFifoReadData &fifo_rd = *in.fifo_rd;
  const InstFifoOut &fifo_head = *in.fifo_head;
  const PtabOut &grp_ptab = *in.grp_ptab;
  const PredecodeCheckerOut &checker_out = *in.checker_out;
  const ICacheOut &ic_out = *in.ic_out;
  const ICacheOut &ic_k = *in.ic_k;
  const bool predecode_flush = *in.predecode_flush, lh_new_valid = *in.lh_new_valid, lh_updated = *in.lh_updated;
  const pc_t flush_address = *in.flush_address, lh_new_pc = *in.lh_new_pc;
  const uint16_t lh_new_data = *in.lh_new_data;
  const bpuf::PdInfo &pd_next = *in.pd_next;

  out.front_state.valid = true;
  const ICacheOut &ic_last = predecode_flush ? ic_k : ic_out;   // the I-cache's outputs of its last step
  {   // FE_PAIR: remember the second block of the ICache request that is still in flight
    bool nv = in.rd->q.pend_valid; fetch_addr_t na = in.rd->q.pend_addr; FetchPair np = in.rd->q.pend_pair;
    const bool head_done = ic_last.icache_read_complete && fetch_addr_head.read_valid && fetch_addr_head.fetch_address == ic_last.fetch_pc;
    if (ic_last.icache_read_complete && nv && in.rd->q.pend_addr == ic_last.fetch_pc) nv = false;
    if (fetch_addr_head.read_valid && !head_done) { nv = true; na = fetch_addr_head.fetch_address; np = fetch_addr_head.pair; }
    if (out.global_refetch || out.global_reset) nv = false;
    out.front_state.d.pend_valid = nv; out.front_state.d.pend_addr = na; out.front_state.d.pend_pair = np;
  }
  {   // RVC=1 last-half state
    bool nv = lh_updated ? lh_new_valid : in.rd->q.lh_valid;
    // a back-end redirect (or reset) drops the carry; a predecode flush drops it unless the flushed group itself set it (sequential continuation),
    // and the refetch cycle that follows a predecode flush keeps it
    if (out.global_reset || in.in->redirect.valid) nv = false;
    else if (predecode_flush && !(lh_updated && lh_new_valid)) nv = false;
    out.front_state.d.lh_valid = nv; out.front_state.d.lh_pc = lh_updated ? lh_new_pc : in.rd->q.lh_pc; out.front_state.d.lh_data = lh_updated ? lh_new_data : in.rd->q.lh_data;
  }
  out.front_state.d.predecode_refetch = predecode_flush;
  out.front_state.d.pd = pd_next;
  out.front_state.d.predecode_refetch_address = predecode_flush ? flush_address : 0;
  out.front_state.d.fifo_full = (fifo_rd.size == INSTRUCTION_FIFO_SIZE);
  out.front_state.d.fifo_empty = (fifo_rd.size == 0);
  out.front_state.d.fdip_s2_en = out.global_reset ? false : bool(bpu_output.PTAB_write_enable);
  out.front_state.d.fdip_s2_va = out.global_reset ? 0 : bpu_output.predict_next_fetch_address;
  out.front_state.d.fdip_s2_clear = out.global_reset ? false : (out.global_refetch || out.pipe_dec.ovr);
  if (fe_icache_pipe()) {
    out.front_state.d.ic_f1_v = !out.global_reset && !out.global_refetch && !predecode_flush && ic_out.icache_read_complete;
    out.front_state.d.ic_f1_entry = *in.f1_next_entry;
  } else {
    out.front_state.d.ic_f1_v = false;
  }

  FrontTopOut &fo = *out.out;
  fo = {};
  for (int i = 0; i < BACKEND_FETCH_WIDTH; ++i) {
    fo.instructions[i] = INST_NOP;
    fo.page_fault_inst[i] = false;
    fo.pf_inst_hi[i] = false;
    fo.inst_valid[i] = false;
  }

  // Unified FTQ-64 S0 allocate / S2-override tail drop / ICache issue pop
  fo.ftq_s0.drop_tail_n = (!out.global_reset && !out.global_refetch && out.pipe_dec.ovr) ? out.pipe_dec.squash_n : 0;
  if (bpu_output.icache_read_valid && bpu_can_run && !out.global_reset) {
    fo.ftq_s0.wr_en = true;
    fo.ftq_s0.addr = bpu_output.fetch_address;
    if (bpu_output.pair_valid) {
      fo.ftq_s0.pair.valid = true;
      fo.ftq_s0.pair.addr2 = bpu_output.pair_addr2;
      fo.ftq_s0.pair.a_len = bpu_output.pair_a_len;
      fo.ftq_s0.pair.b_len = bpu_output.pair_b_len;
    }
  }
  fo.ftq_s0.ifu_pop = out.fetch_addr_read_enable;

  // Unified FTQ-64 S2 fill (prediction + ftq_meta written once at s2_ptr)
  if (out.s2_wr_en) {
    fo.ftq_s2.wr_en = true;
    fo.ftq_s2.is_pair = bpu_output.ptab_pcs_valid;
    fo.ftq_s2.next_pc = bpu_output.predict_next_fetch_address;
    for (int i = 0; i < FETCH_WIDTH; i++) {
      fo.ftq_s2.pred_dir[i] = bpu_output.out_pred_dir[i];
      fo.ftq_s2.slot_pc[i] = bpu_output.ptab_pcs_valid ? bpu_output.out_pred_pc[i] : bpu_output.out_pred_base_pc + (i * SLOT_BYTES);
    }
    fo.ftq_s2.meta = bpu_output.meta;
    fo.ftq_s2.meta.mask();
    fpr_ptab(fo.ftq_s2);
  }

  // Unified FTQ-64 Predecode in-place update + direct delivery to Front2Back (no front2back_fifo)
  if (out.fifo_read_enable) {
    bool no_taken = true;
    int nslots = 0;
    for (int i = 0; i < FETCH_WIDTH; i++) {
      const bool raw_v = fifo_head.inst_valid[i] && i < int(checker_out.keep_n);
      const bool prev_raw_v = (i > 0) && fifo_head.inst_valid[i - 1] && (i - 1) < int(checker_out.keep_n);
      if (i > 0 && !no_taken && prev_raw_v && raw_v &&
          grp_ptab.predict_base_pc[i] != grp_ptab.predict_base_pc[i - 1] + ilen_of(fifo_head.is_rvc[i - 1]) &&
          checker_out.predict_dir_corrected[i - 1]) {
        no_taken = true;
      }
      const bool v = no_taken && raw_v;
      if (v && checker_out.predict_dir_corrected[i]) no_taken = false;
      fo.inst_valid[i] = v;
      fo.instructions[i] = fifo_head.instructions[i];
      fo.pc[i] = grp_ptab.predict_base_pc[i];
      fo.is_rvc[i] = fifo_head.is_rvc[i];
      fo.page_fault_inst[i] = fifo_head.page_fault_inst[i];
      fo.pf_inst_hi[i] = fifo_head.pf_inst_hi[i];
      fo.predict_dir[i] = checker_out.predict_dir_corrected[i];
      fo.ftq_pd.slot_pc[i] = grp_ptab.predict_base_pc[i];
      fo.ftq_pd.is_rvc[i] = fifo_head.is_rvc[i];
      fo.ftq_pd.pred_dir[i] = v && checker_out.predict_dir_corrected[i];
      if (v) nslots = i + 1;
    }
    fo.FIFO_valid = (nslots > 0);
    fo.ftq_idx = ftq ? ftq->pd_idx() : 0;
    fo.predict_next_fetch_address = checker_out.predict_next_fetch_address_corrected;
    fo.meta.assign_prefix(grp_ptab.meta);
    fo.ftq_pd.fire = true;
    fo.ftq_pd.flush = predecode_flush;
    fo.ftq_pd.empty_blk = (nslots == 0);
    fo.ftq_pd.next_pc = checker_out.predict_next_fetch_address_corrected;
    fo.ftq_pd.nslots = static_cast<uint8_t>(nslots);
  }

  fo.commit_stall = bpu_output.update_queue_full;
  fo.commit_upd_free = bpu_output.upd_free;
  fo.commit_tq_free = bpu_output.tq_free;
  // PMU frontend delivery split (EXP-025): how many instructions of this cycle's fetch group the back end gets, and why the other slots are empty
  {
    const uint64_t cyc = ctx ? ctx->perf.cycle : 0;
    const uint64_t last_refetch = out.global_refetch ? cyc : in.rd->q.pmu_last_refetch;
    out.front_state.d.pmu_last_refetch = last_refetch;
    PmuFrontSignals &pf = g_pmu_front;
    pf.feo_ready = in.in->ctrl.fifo_read_enable;
    pf.feo_n = 0;
    pf.feo_cause = 0;
    if (pf.feo_ready) {
      int last = -1;
      for (int j = 0; j < FETCH_WIDTH; j++) {
        if (fo.FIFO_valid && fo.inst_valid[j]) { pf.feo_n++; last = j; }
      }
      if (pf.feo_n > 0) {
        if (pf.feo_n < FETCH_WIDTH) {
          const uint32_t line = ICACHE_LINE_SIZE;
          if (fo.predict_dir[last]) pf.feo_cause = PMU_FEO_LOSS_TAKEN;
          else if (((fo.pc[last] + 4u) & ~(line - 1u)) != (fo.pc[last] & ~(line - 1u))) pf.feo_cause = PMU_FEO_LOSS_LINE;
          else pf.feo_cause = PMU_FEO_LOSS_SHORTOTH;
        }
      } else if (out.global_refetch || cyc - last_refetch <= 6) pf.feo_cause = PMU_FEO_LOSS_REFETCH;
      else if (ic_last.perf_miss_busy) pf.feo_cause = PMU_FEO_LOSS_ICMISS;
      else if (predecode_flush) pf.feo_cause = PMU_FEO_LOSS_PDFLUSH;
      else if ((!ftq || ftq->fa_empty()) && !bpu_output.icache_read_valid) {
        if (out.pipe_dec.ovr) pf.feo_cause = PMU_FEO_LOSS_OVR_SD;
        else if (*in.pmu_l1_ovr) pf.feo_cause = PMU_FEO_LOSS_OVR_L1;
        else if ((ftq && ftq->ftq_full()) || *in.pmu_ptab_stall) pf.feo_cause = PMU_FEO_LOSS_PTABSTALL;
        else pf.feo_cause = PMU_FEO_LOSS_BPUOTHER;
      }
      else if (!ic_last.icache_read_ready) pf.feo_cause = PMU_FEO_LOSS_ICNOTRDY;
      else if (in.rd->q.fifo_empty) pf.feo_cause = PMU_FEO_LOSS_IFIFO;
      else if (!ftq || ftq->s2_empty()) pf.feo_cause = PMU_FEO_LOSS_PTABLAG;
      else pf.feo_cause = PMU_FEO_LOSS_F2B;
    }
  }
}

// A predecode flush: the I-cache's invalidate step K, whose outputs replace the fetch step's (ic_k).
void ICacheTop::comb_invalidate() {
  out.ic_k = {};
  if (*in.predecode_flush) invalidate_step(&out.ic_k);
}

// ---- the instruction FIFO port (front-end/front_fifo_ports.h) ----

// The head of the instruction FIFO as the checker sees it: FE_PAIR keeps only the slots that agree with the Unified FTQ
// pd_ptr entry; RVC=1 joins the carried half of a straddling instruction (rvc_head_fix).
void InstPort::comb_read() {
  const bool global_reset = *in.global_reset, global_refetch = *in.global_refetch;
  out.fifo_in = {};
  out.fifo_rd = in.rd->fifo_rd_snapshot;
  out.fifo_head = {};
  out.fifo_req.clear_fifo = out.fifo_req.push_en = out.fifo_req.pop_en = false;
  out.multi = false;
  for (int i = 0; i < FETCH_WIDTH; i++) out.grp_in_range[i] = out.grp_seg_first[i] = false;
  out.rvc_n = 0;
  out.rvc_straddle = false;
  out.carry_low = 0;
  out.grp_ptab = {};
  if (ftq && !ftq->s2_empty()) {
    const FTQEntry &pe = ftq->pd_head();
    out.grp_ptab.is_pair = pe.s2_is_pair;
    out.grp_ptab.predict_next_fetch_address = pe.next_pc;
    for (int i = 0; i < FETCH_WIDTH; i++) {
      out.grp_ptab.predict_dir[i] = pe.pred_taken_mask[i];
      out.grp_ptab.predict_base_pc[i] = pe.slot_pc[i];
    }
    out.grp_ptab.meta.assign_prefix(ftq->pd_meta());
    out.grp_ptab.meta.mask();
  }
  out.fifo_in.reset = global_reset;
  out.fifo_in.refetch = global_refetch;
  out.fifo_in.read_enable = *in.fifo_read_enable;
  fifo->read(out.fifo_in, out.fifo_rd, out.fifo_head, out.fifo_req);
  if (*in.fifo_read_enable) {
    // S2-confirmed A/B mux: if S2 overrode A (d.pair=false), restore intact block A instead of cutting A at a_len
    if (out.fifo_head.has_alt_a && !out.grp_ptab.is_pair) {
      for (int i = 0; i < FETCH_WIDTH; i++) {
        out.fifo_head.instructions[i] = out.fifo_head.alt_instructions[i];
        out.fifo_head.pc[i] = out.fifo_head.alt_pc[i];
        out.fifo_head.page_fault_inst[i] = out.fifo_head.alt_page_fault_inst[i];
        out.fifo_head.inst_valid[i] = out.fifo_head.alt_inst_valid[i];
        out.fifo_head.predecode_type[i] = out.fifo_head.alt_predecode_type[i];
        out.fifo_head.predecode_target_address[i] = out.fifo_head.alt_predecode_target_address[i];
        out.fifo_head.is_rvc[i] = out.fifo_head.alt_is_rvc[i];
        out.fifo_head.start_a[i] = out.fifo_head.alt_start_a[i];
        out.fifo_head.start_b[i] = out.fifo_head.alt_start_b[i];
        out.fifo_head.raw16[i] = out.fifo_head.alt_raw16[i];
      }
      out.fifo_head.up16 = out.fifo_head.alt_up16;
      out.fifo_head.up_ok = out.fifo_head.alt_up_ok;
      out.fifo_head.seq_next_pc = out.fifo_head.alt_seq_next_pc;
    }
    // Only the slots whose pc agrees with the PTAB entry are usable
    bool cut = false, dropped_valid = false;
    for (int i = 0; i < FETCH_WIDTH; i++) {
      if (!cut && out.fifo_head.inst_valid[i] && out.fifo_head.pc[i] != out.grp_ptab.predict_base_pc[i]) cut = true;
      if (cut) {
        if (out.fifo_head.inst_valid[i]) dropped_valid = true;
        out.fifo_head.inst_valid[i] = false; out.fifo_head.predecode_type[i] = PREDECODE_NON_BRANCH; out.fifo_head.predecode_target_address[i] = 0;
      }
    }
    for (int i = 1; i < FETCH_WIDTH; i++) if (out.fifo_head.inst_valid[i] && out.fifo_head.pc[i] != out.fifo_head.pc[i - 1] + SLOT_BYTES) out.multi = true;
    if (dropped_valid) {  // the group lost valid slots: restore up16 from the unmasked new last slot & fix seq_next_pc
      int last = -1;
      for (int i = 0; i < FETCH_WIDTH; i++) if (out.fifo_head.inst_valid[i]) last = i;
      if (last >= 0) {
        out.fifo_head.seq_next_pc = out.fifo_head.pc[last] + SLOT_BYTES;
        out.fifo_head.up16 = uint16_t(out.fifo_head.instructions[last] >> 16);
        out.fifo_head.up_ok = true;
      }
    }
    for (int i = 0; i < FETCH_WIDTH; i++) {
      out.grp_in_range[i] = out.fifo_head.inst_valid[i];
      out.grp_seg_first[i] = i > 0 && out.fifo_head.inst_valid[i] && out.fifo_head.inst_valid[i - 1] && out.fifo_head.pc[i] != out.fifo_head.pc[i - 1] + SLOT_BYTES;
    }
    if constexpr (kRvc) {
      const RvcHeadFix hf = rvc_head_fix(out.fifo_head, out.grp_ptab, in.rd->q.lh_valid, in.rd->q.lh_pc, in.rd->q.lh_data);
      out.rvc_n = hf.n; out.rvc_straddle = hf.straddle; out.carry_low = hf.low;
    }

  }
}

void InstPort::comb_write() {
  const bool global_reset = *in.global_reset, global_refetch = *in.global_refetch;
  const bool pipe_ic = fe_icache_pipe();
  const ICacheOut &ic_out = *in.ic_out;

  if (pipe_ic) {
    out.f1_next_entry = {};
    if (in.rd->q.ic_f1_v && !in.rd->q.fifo_full && !global_reset && !global_refetch) {
      out.fifo_in.wdata = in.rd->q.ic_f1_entry;
      fifo->write(out.fifo_in, out.fifo_rd, out.fifo_req);
    }
    if (!ic_out.icache_read_complete || global_reset || global_refetch) return;
  } else {
    if (in.rd->q.fifo_full || !ic_out.icache_read_complete || global_reset || global_refetch) return;
  }
  InstFifoEntry entry{};
  // Same-Page Line-Straddle 2-Fetch: when a 32B block crosses a 64B cache-line boundary within the same 4KB page,
  // read the next sequential 64B line (always a different ICache bank, same physical page -> zero 2nd ITLB lookup!).
  inst_word_t grp[FETCH_WIDTH]; pc_t gpc[FETCH_WIDTH]; wire<1> gv[FETCH_WIDTH], gpf[FETCH_WIDTH];
  for (int i = 0; i < FETCH_WIDTH; i++) { grp[i] = ic_out.fetch_group[i]; gpc[i] = ic_out.fetch_pc + (i * SLOT_BYTES); gv[i] = ic_out.inst_valid[i]; gpf[i] = ic_out.page_fault_inst[i]; }
  const uint32_t off6 = static_cast<uint32_t>(ic_out.fetch_pc & (ICACHE_LINE_SIZE - 1u));
  const bool same_page_next = (((ic_out.fetch_pc >> 6) & 0x3fu) != 0x3fu);
  const bool straddle = (off6 > FETCH_WIDTH * SLOT_BYTES) && same_page_next;
  const bool tail_next = (off6 == FETCH_WIDTH * SLOT_BYTES) && same_page_next;
  const int str_a_len = straddle ? static_cast<int>((ICACHE_LINE_SIZE - off6) >> SLOT_SHIFT) : 0;
  const int str_b_len = straddle ? (FETCH_WIDTH - str_a_len) : (tail_next ? 1 : 0);
  const pc_t str_addr2 = (ic_out.fetch_pc & ~static_cast<pc_t>(ICACHE_LINE_SIZE - 1u)) + ICACHE_LINE_SIZE;
  const bool want_pair = straddle && str_a_len > 0 && str_a_len < FETCH_WIDTH && !ic_out.page_fault_inst[0] && !ic_out.fetch_single_word && ic_out.inst_valid[str_a_len - 1];
  const bool want_tail = tail_next && !ic_out.page_fault_inst[0] && !ic_out.fetch_single_word && ic_out.inst_valid[FETCH_WIDTH - 1];
  bool tail_ok = false;
  uint16_t tail_up16 = 0;
  if (want_pair) {
    uint32_t w2[FETCH_WIDTH];
    int actual_b = 0;
    const bool ok = icache->fetch_second(ic_out.fetch_pc, ic_out.fetch_paddr, str_addr2, str_b_len, w2, actual_b);
    if (ok && actual_b > 0) {
      grp[str_a_len - 1] = (grp[str_a_len - 1] & 0xffffu) | ((w2[0] & 0xffffu) << 16);
    }
    for (int i = str_a_len; i < FETCH_WIDTH; i++) {
      const int j = i - str_a_len;
      gpc[i] = str_addr2 + SLOT_BYTES * j;
      gv[i] = ok && j < actual_b; gpf[i] = false; grp[i] = ok && j < actual_b ? w2[j] : INST_NOP;
    }
    if (!ok) for (int i = str_a_len; i < FETCH_WIDTH; i++) gv[i] = false;
  } else if (want_tail) {
    uint32_t w2[FETCH_WIDTH];
    int actual_b = 0;
    const bool ok = icache->fetch_second(ic_out.fetch_pc, ic_out.fetch_paddr, str_addr2, 1, w2, actual_b);
    if (ok && actual_b > 0) {
      tail_ok = true;
      tail_up16 = uint16_t(w2[0] & 0xffffu);
    }
  }
  uint16_t rvc_up16 = 0; bool rvc_up_ok = false;
  if constexpr (kRvc) {   // C910-style carry: the window of the last slot of a group has no upper half (a 32-bit instruction starting there is carried to the next group)
    int nv = 0;
    while (nv < FETCH_WIDTH && gv[nv]) nv++;
    if (nv > 0) {
      // the upper half of the last window is in the same cache line (or next same-page line for off6==32): the fetch can deliver it with the group
      if (tail_ok && nv == FETCH_WIDTH) {
        rvc_up16 = tail_up16;
        rvc_up_ok = true;
      } else {
        rvc_up16 = uint16_t(grp[nv - 1] >> 16);
        rvc_up_ok = !ic_out.fetch_single_word && ((gpc[nv - 1] + SLOT_BYTES) & (ICACHE_LINE_SIZE - 1u)) != 0;
      }
      grp[nv - 1] &= 0xffffu;   // (an inner FE_PAIR block end keeps its window: the upper half is in the same fetched line)
    }
  }
  entry.up16 = rvc_up16; entry.up_ok = rvc_up_ok;
  for (int i = 0; i < FETCH_WIDTH; i++) {
    entry.raw16[i] = uint16_t(grp[i] & 0xffffu);
    entry.instructions[i] = grp[i];
    entry.pc[i] = gpc[i];
    entry.page_fault_inst[i] = gpf[i];
    entry.inst_valid[i] = gv[i];
    if (gv[i]) {
      if constexpr (kRvc) {   // the window at this slot is a 16-bit instruction when its low bits are not 11: expand it (an illegal encoding becomes 0)
        if ((grp[i] & 3u) != 3u) {
          uint32_t ex = 0;
          entry.is_rvc[i] = true;
          grp[i] = rvc::expand(grp[i] & 0xffffu, ex) ? ex : 0u;
          entry.instructions[i] = grp[i];
        }
      }
      const PredecodeResult result = predecode(grp[i], gpc[i]);
      entry.predecode_type[i] = result.type;
      entry.predecode_target_address[i] = result.target_address;
    } else {
      entry.predecode_type[i] = PREDECODE_NON_BRANCH;
      entry.predecode_target_address[i] = 0;
    }
  }
  if constexpr (kRvc) {   // instruction starts: from slot 0 (start_a) or, when slot 0 is the second half of the previous block's last instruction, from slot 1 (start_b)
    // FE_PAIR: the group may hold two fetch blocks; each block's first slot is an instruction start and a block's last instruction (a 32-bit branch whose
    // window is complete) does not run into the next block's first slot.
    int n = 0;
    while (n < FETCH_WIDTH && gv[n]) n++;
    auto boundary = [&](int k) { return k > 0 && k < n && gpc[k] != gpc[k - 1] + SLOT_BYTES; };
    for (int pass = 0; pass < 2; pass++) {
      wire<1> *st = pass ? entry.start_b : entry.start_a;
      int pos = pass;
      while (pos < n) {
        st[pos] = true;
        int nxt = pos + (entry.is_rvc[pos] ? 1 : 2);
        for (int k = pos + 1; k < nxt && k < n; k++) if (boundary(k)) { nxt = k; break; }
        pos = nxt;
      }
    }
  }
  if (want_pair) {   // 2-taken pair: next pc after the last valid slot
    int last = -1;
    for (int i = 0; i < FETCH_WIDTH; i++) if (entry.inst_valid[i]) last = i;
    entry.seq_next_pc = entry.pc[last] + SLOT_BYTES;   // the position after the last slot (RVC=1: a straddling last instruction continues there)
  } else
  entry.seq_next_pc = sequential_next_pc(ic_out.fetch_pc, ic_out.inst_valid);

  if (pipe_ic) {
    out.f1_next_entry = entry;
  } else {
    out.fifo_in.wdata = entry;
    fifo->write(out.fifo_in, out.fifo_rd, out.fifo_req);
  }
}

// The front end's BPU phase (front-end/BPU/BPU_pipe.h).
void bpuf::BpuPipe::comb_front() {
  out = {};
  const bool global_reset = *in.global_reset, global_refetch = *in.global_refetch;
  const Decision &pipe_dec = *in.pipe_dec;

  fpr::set_cycle(ctx ? ctx->perf.cycle : 0);
  const bool bpu_stall = ftq ? ftq->ftq_full() : false;
  out.bpu_can_run = (!bpu_stall || global_reset || global_refetch);

  bpuf::InputPayload bpu_input{};
  bpu_input.refetch = global_refetch;
  bpu_input.refetch_pd = global_refetch && !in.in->redirect.valid;
  bpu_input.rf_valid = in.in->redirect.valid && in.in->redirect.rf_valid;
  bpu_input.rf_blk = in.in->redirect.rf_blk; bpu_input.rf_slot = in.in->redirect.rf_slot; bpu_input.rf_type = in.in->redirect.rf_type; bpu_input.rf_pc = in.in->redirect.rf_pc; bpu_input.rf_is_rvc = in.in->redirect.rf_is_rvc;
  bpu_input.pd = in.rd->q.pd;
  bpu_input.refetch_address = in.in->redirect.valid ? in.in->redirect.address : in.rd->q.predecode_refetch_address;
  bpu_input.icache_read_ready = !bpu_stall;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    bpu_input.in_update_base_pc[i] = in.in->train[i].base_pc;
    bpu_input.in_update_is_rvc[i] = in.in->train[i].is_rvc;
    bpu_input.in_upd_valid[i] = in.in->train[i].valid;
    bpu_input.in_actual_dir[i] = in.in->train[i].actual_dir;
    bpu_input.in_actual_br_type[i] = in.in->train[i].br_type;
    bpu_input.in_actual_targets[i] = in.in->train[i].target;
    bpu_input.in_pred_dir[i] = in.in->train[i].pred_dir;
    bpu_input.in_alt_pred[i] = in.in->train[i].alt_pred;
    bpu_input.in_pcpn[i] = in.in->train[i].pcpn;
    bpu_input.in_blk_id[i] = in.in->train[i].blk_id;
    bpu_input.in_ftq_is_last[i] = in.in->train[i].ftq_is_last;
  }

  out.bpu_seq_txn.valid = true;
  out.bpu_seq_txn.reset = in.in->ctrl.reset;
  if (in.in->ctrl.reset) {
    out.bpu_output.fetch_address = RESET_PC;
  } else {
    comb_cycle(bpu_input, pipe_dec, out.bpu_output);
  }
  out.pmu_l1_ovr = txn_d_.pmu_l1_ovr;
  out.pmu_ptab_stall = txn_d_.pmu_ptab_stall;
}

// The front end's fetch step (front-end/icache/icache_top.h).
void ICacheTop::comb_fetch() {
  out.ic_in = {};   // (its ports to the AXI interconnect, the D-cache probe and the walker stay bound)
  out.ic_out = {};
  out.ic_k = {};
  const bool global_reset = *in.global_reset, global_refetch = *in.global_refetch;
  const FetchAddrFifoOut &fetch_addr_head = *in.fetch_addr_head;

  out.ic_in.reset = global_reset;
  out.ic_in.refetch = global_refetch;
  out.ic_in.itlb_flush = in.in->ctrl.itlb_flush;
  out.ic_in.fence_i = in.in->ctrl.fence_i;
  out.ic_in.invalidate_req = false;
  out.ic_in.icache_read_valid = fetch_addr_head.read_valid;
  out.ic_in.fetch_address = fetch_addr_head.read_valid ? fetch_addr_head.fetch_address : 0;
  comb_calc(&out.ic_in, &out.ic_out);
  g_pmu_front.icache_req_fire = out.ic_out.perf_req_fire;
  g_pmu_front.icache_miss_issue = out.ic_out.perf_miss_event;
  g_pmu_front.icache_miss_busy = out.ic_out.perf_miss_busy;
  g_pmu_front.itlb_access = out.ic_out.perf_itlb_hit || out.ic_out.perf_itlb_miss || out.ic_out.perf_itlb_fault;
  g_pmu_front.itlb_miss = out.ic_out.perf_itlb_miss;
}

// The predecode check (front-end/front_checker.h).
void FrontChecker::comb() {
  out = {};
  const bool fifo_read_enable = *in.fifo_read_enable;
  const InstFifoOut &fifo_head = *in.fifo_head;
  const PtabOut &grp_ptab = *in.grp_ptab;
  const bool multi = *in.multi;
  const wire<1> *grp_in_range = in.grp_in_range, *grp_seg_first = in.grp_seg_first;
  const int rvc_n = *in.rvc_n;
  const bool rvc_straddle = *in.rvc_straddle;
  const uint16_t carry_low = *in.carry_low;

  if (!fifo_read_enable) return;
  PredecodeCheckerIn checker_in;
  checker_input(fifo_head, grp_ptab, checker_in);
  checker_in.multi = multi;
  for (int i = 0; i < FETCH_WIDTH; i++) { checker_in.pc[i] = fifo_head.pc[i]; checker_in.inst_valid[i] = fifo_head.inst_valid[i]; checker_in.in_range[i] = grp_in_range[i]; checker_in.seg_first[i] = grp_seg_first[i]; checker_in.is_rvc[i] = fifo_head.is_rvc[i]; }
  predecode_check(checker_in, out.checker_out);
  if (out.checker_out.predecode_flush_enable) {
    out.predecode_flush = true;
    out.flush_address = out.checker_out.predict_next_fetch_address_corrected;
  }
  if constexpr (kRvc) {   // the next group starts with the second half of this group's last instruction if it straddles and the fetch continues sequentially
    bool any_taken = false;
    int last_seg = 0;   // FE_PAIR: only the last block of the group decides whether the fetch continues sequentially (a taken branch that ends the first block does not)
    for (int i = 0; i < FETCH_WIDTH; i++) if (grp_seg_first[i]) last_seg = i;
    for (int i = last_seg; i < FETCH_WIDTH; i++) if (fifo_head.inst_valid[i] && out.checker_out.predict_dir_corrected[i]) any_taken = true;
    out.lh_updated = true;
    out.lh_new_pc = fifo_head.pc[rvc_n > 0 ? rvc_n - 1 : 0] + (rvc_n > 0 ? SLOT_BYTES : 0);   // the position after the group's last slot
    // a predecode flush that continues sequentially (corrected target = the end of this group) keeps the carry: the refetched group still begins with the
    // second half of the straddling instruction
    out.lh_new_data = carry_low;
    out.lh_new_valid = rvc_straddle && !any_taken && (!out.checker_out.predecode_flush_enable || out.checker_out.predict_next_fetch_address_corrected == out.lh_new_pc);
  }
  {  // BPO_PDCKPT: report the check, and the history / RAS effect of the corrected block, to the BPU next cycle
    out.pd_next = bpuf::PdInfo{};
    out.pd_next.valid = true;
    out.pd_next.flush = out.predecode_flush;
    if (out.predecode_flush) {
      pc_t seg_base = grp_ptab.predict_base_pc[0];
      for (int i = 0; i < FETCH_WIDTH; i++) {   // a taken prediction on an instruction that is not a branch (a dropped slot is not one: the carried instruction may be)
        if (grp_seg_first[i]) seg_base = grp_ptab.predict_base_pc[i];
        if (fifo_head.inst_valid[i] && grp_ptab.predict_dir[i] && fifo_head.predecode_type[i] == PREDECODE_NON_BRANCH) {
          out.pd_next.nb_valid = true; out.pd_next.nb_pc = fifo_head.pc[i]; out.pd_next.nb_base = multi ? seg_base : 0; break;
        }
      }
      for (int i = 0; i < FETCH_WIDTH; i++) {
        if (!fifo_head.inst_valid[i]) continue;
        const predecode_type_t ty = fifo_head.predecode_type[i];
        if (ty == PREDECODE_NON_BRANCH) continue;
        const pc_t pc = fifo_head.pc[i]; const uint32_t inst = fifo_head.instructions[i];
        const bool taken = out.checker_out.predict_dir_corrected[i];
        pc_t tgt = fifo_head.predecode_target_address[i];
        if (ty == PREDECODE_JALR) {
          tgt = (multi && i + 1 < FETCH_WIDTH && grp_seg_first[i + 1]) ? fifo_head.pc[i + 1] : grp_ptab.predict_next_fetch_address;
        }
        if (multi && i >= out.checker_out.keep_n) break;
        if (!taken) continue;
        out.pd_next.bits |= uint8_t(bpuf::hist_sym(pc, tgt)) << out.pd_next.nbits; out.pd_next.nbits += bpuf::HSYM_BITS;
        const uint32_t rd_ = (inst >> 7) & 31, rs1 = (inst >> 15) & 31;
        uint8_t rop = 0; target_addr_t rval = 0;
        if (ty == PREDECODE_JAL && rd_ == 1) { rop = 1; rval = pc + ilen_of(fifo_head.is_rvc[i]); }
        else if (ty == PREDECODE_JALR && rd_ == 0 && rs1 == 1 && (inst >> 20) == 0) rop = 2;
        if (rop != 0) {
          if (out.pd_next.ras_op == 0) { out.pd_next.ras_op = rop; out.pd_next.ras_val = rval; }
          else if (out.pd_next.ras_op2 == 0) { out.pd_next.ras_op2 = rop; out.pd_next.ras_val2 = rval; }
        }
        if (!multi || i + 1 >= int(out.checker_out.keep_n)) break;
      }
    }
  }
}

// The modules' ports, bound once: each input at the output of the module that drives it (a port carries the name of
// its wire)
FrontCore::FrontCore() {
  inst_port.fifo = &inst_fifo;
  ctrl.bpu_pipe = &bpu_pipe;
  ctrl.icache = &icache;
  inst_port.icache = &icache;
  checker.in.fifo_read_enable = &ctrl.out.fifo_read_enable;
  checker.in.fifo_head = &inst_port.out.fifo_head;
  checker.in.grp_ptab = &inst_port.out.grp_ptab;
  checker.in.multi = &inst_port.out.multi;
  checker.in.grp_in_range = inst_port.out.grp_in_range;
  checker.in.grp_seg_first = inst_port.out.grp_seg_first;
  checker.in.rvc_n = &inst_port.out.rvc_n;
  checker.in.rvc_straddle = &inst_port.out.rvc_straddle;
  checker.in.carry_low = &inst_port.out.carry_low;
  icache.in.global_reset = &ctrl.out.global_reset;
  icache.in.global_refetch = &ctrl.out.global_refetch;
  icache.in.fetch_addr_head = &ctrl.out.fetch_addr_head;
  icache.in.predecode_flush = &checker.out.predecode_flush;
  bpu_pipe.in.global_reset = &ctrl.out.global_reset;
  bpu_pipe.in.global_refetch = &ctrl.out.global_refetch;
  bpu_pipe.in.pipe_dec = &ctrl.out.pipe_dec;
  inst_port.in.global_reset = &ctrl.out.global_reset;
  inst_port.in.global_refetch = &ctrl.out.global_refetch;
  inst_port.in.fifo_read_enable = &ctrl.out.fifo_read_enable;
  inst_port.in.ic_out = &icache.out.ic_out;
  ctrl.in.bpu_output = &bpu_pipe.out.bpu_output;
  ctrl.in.bpu_can_run = &bpu_pipe.out.bpu_can_run;
  ctrl.in.fifo_rd = &inst_port.out.fifo_rd;
  ctrl.in.fifo_head = &inst_port.out.fifo_head;
  ctrl.in.grp_ptab = &inst_port.out.grp_ptab;
  ctrl.in.checker_out = &checker.out.checker_out;
  ctrl.in.ic_out = &icache.out.ic_out;
  ctrl.in.ic_k = &icache.out.ic_k;
  ctrl.in.predecode_flush = &checker.out.predecode_flush;
  ctrl.in.lh_new_valid = &checker.out.lh_new_valid;
  ctrl.in.lh_updated = &checker.out.lh_updated;
  ctrl.in.flush_address = &checker.out.flush_address;
  ctrl.in.lh_new_pc = &checker.out.lh_new_pc;
  ctrl.in.lh_new_data = &checker.out.lh_new_data;
  ctrl.in.pd_next = &checker.out.pd_next;
  ctrl.in.f1_next_entry = &inst_port.out.f1_next_entry;
  ctrl.in.pmu_l1_ovr = &bpu_pipe.out.pmu_l1_ovr;
  ctrl.in.pmu_ptab_stall = &bpu_pipe.out.pmu_ptab_stall;
}
FrontCore::~FrontCore() = default;

void FrontCore::set_ftq(const PreIduQueue *ftq) {
  ctrl.ftq = ftq;
  bpu_pipe.ftq = ftq;
  inst_port.ftq = ftq;
}

// The front end's inputs, registers and output (FrontTop's) and the context
void FrontCore::bind(const FrontTopIn *in, const FrontReadData *rd, FrontTopOut *out, SimContext *c) {
  ctx = c;
  bpu_pipe.ctx = c;
  ctrl.ctx = c;
  icache.in.in = in;
  bpu_pipe.in.in = in;
  ctrl.in.in = in;
  bpu_pipe.in.rd = rd;
  inst_port.in.rd = rd;
  ctrl.in.rd = rd;
  ctrl.out.out = out;
}


// All phases of one cycle (init / step_bpu)
void FrontCore::comb() {
  // BEGIN GENERATED by scripts/comb/gen_top_order.py from the module contracts: do not edit
  ctrl.control();
  bpu_pipe.comb_front();
  ctrl.enables();
  inst_port.comb_read();
  icache.comb_fetch();
  inst_port.comb_write();
  checker.comb();
  icache.comb_invalidate();
  ctrl.finish();
  // END GENERATED
}

void FrontCore::read_q(FrontReadData &rd) const {
  rd.q = q_;
  inst_fifo.read_q(rd.fifo_rd_snapshot);
}

void FrontCore::seq(const FrontTopIn &inp) {
  (void)inp;
  icache.seq();   // first: the I-cache edge comes before everything below that reads the arrays / ITLB (FE_IPF)
  if (bpu_pipe.out.bpu_seq_txn.valid) {
    if (bpu_pipe.out.bpu_seq_txn.reset) bpu_pipe.reset();
    else bpu_pipe.seq();
    if (fpr::on() && fpr::in_dump()) bpu_pipe.dump_hist(static_cast<uint32_t>(fpr::cycle()));
  }
  inst_fifo.seq(inst_port.out.fifo_req);
  if (ctrl.out.front_state.valid) {
    pc_t s2c[1];
    int n = 0;
    if (q_.fdip_s2_en) s2c[n++] = q_.fdip_s2_va;
    icache.enqueue_fdip(s2c, n, q_.fdip_s2_clear);
    q_ = ctrl.out.front_state.d;
  }
}
