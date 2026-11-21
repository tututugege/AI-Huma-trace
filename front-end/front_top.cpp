#include REFCPU_RVC_H   // the function model's RVC table (Makefile: from the same tree as the librefcpu API header)
#include "fatal.h"
#include "trace_hook.h"
#include "BPU/BPU.h"
#include "BPU/BPU_pipe.h"
#include "Pmu.h"
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


// Alignment fingerprint of a PTAB write (see BPU/fpr.h)
static void fpr_ptab(const PtabIn &p) {
  if (!fpr::on()) return;
  uint32_t m = 0;
  for (int i = 0; i < FETCH_WIDTH; i++) m |= uint32_t(p.wdata.predict_dir[i]) << i;
  fpr::pt(static_cast<uint32_t>(p.wdata.predict_base_pc[0]), static_cast<uint32_t>(p.wdata.predict_next_fetch_address), m);
}

// PTAB write port from the BPU output.
static void ptab_write_data(const bpuf::OutputPayload &bpu_output, wire<1> global_reset,
                                  wire<1> global_refetch, PtabIn &ptab_in) {
  ptab_in = {};
  ptab_in.reset = global_reset;
  ptab_in.refetch = global_refetch;
  ptab_in.read_enable = false;
  ptab_in.write_enable = true;
  for (int i = 0; i < FETCH_WIDTH; i++) {
    ptab_in.wdata.predict_dir[i] = bpu_output.out_pred_dir[i];
    ptab_in.wdata.predict_base_pc[i] = bpu_output.ptab_pcs_valid ? bpu_output.out_pred_pc[i] : bpu_output.out_pred_base_pc + (i * SLOT_BYTES);
  }
  ptab_in.wdata.meta = bpu_output.meta;
  ptab_in.wdata.predict_next_fetch_address = bpu_output.predict_next_fetch_address;
  ptab_in.wdata.is_pair = bpu_output.ptab_pcs_valid;
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

// front2back FIFO write data (and, when the entry bypasses the FIFO, the output it would have produced).
static void f2b_write_data(const InstFifoOut &fifo_out, const PtabOut &ptab_out,
                                        const PredecodeCheckerOut &checker_out, bool bypass,
                                        F2bFifoIn &fifo_in, F2bFifoOut &bypass_out) {
  for (int i = 0; i < FETCH_WIDTH; i++) {
    fifo_in.wdata.fetch_group[i] = fifo_out.instructions[i];
    fifo_in.wdata.page_fault_inst[i] = fifo_out.page_fault_inst[i];
    fifo_in.wdata.pf_inst_hi[i] = fifo_out.pf_inst_hi[i];
    fifo_in.wdata.inst_valid[i] = fifo_out.inst_valid[i] && i < int(checker_out.keep_n);
    fifo_in.wdata.is_rvc[i] = fifo_out.is_rvc[i];
    fifo_in.wdata.predict_dir_corrected[i] = checker_out.predict_dir_corrected[i];
    fifo_in.wdata.predict_base_pc[i] = ptab_out.predict_base_pc[i];
    if (bypass) {
      bypass_out.fetch_group[i] = fifo_out.instructions[i];
      bypass_out.page_fault_inst[i] = fifo_out.page_fault_inst[i];
      bypass_out.pf_inst_hi[i] = fifo_out.pf_inst_hi[i];
      bypass_out.inst_valid[i] = fifo_out.inst_valid[i] && i < int(checker_out.keep_n);
      bypass_out.is_rvc[i] = fifo_out.is_rvc[i];
      bypass_out.predict_dir_corrected[i] = checker_out.predict_dir_corrected[i];
      bypass_out.predict_base_pc[i] = ptab_out.predict_base_pc[i];
    }
  }
  fifo_in.wdata.meta = ptab_out.meta;
  fifo_in.wdata.meta.mask();
  fifo_in.wdata.predict_next_fetch_address_corrected = checker_out.predict_next_fetch_address_corrected;
  if (bypass) {
    bypass_out.meta = ptab_out.meta;
    bypass_out.front2back_FIFO_valid = true;
    bypass_out.predict_next_fetch_address_corrected = checker_out.predict_next_fetch_address_corrected;
  }
}

// Output toward the back end: the FIFO head, or the bypassed entry when the FIFO was empty.
static void front_output(const F2bFifoOut &saved, const F2bFifoOut &bypass_out,
                              bool use_bypass, FrontTopOut &out) {
  out = {};
  const F2bFifoOut *src = &saved;
  if (!saved.front2back_FIFO_valid && use_bypass) src = &bypass_out;

  out.FIFO_valid = src->front2back_FIFO_valid;
  for (int i = 0; i < FETCH_WIDTH; i++) {
    out.instructions[i] = src->fetch_group[i];
    out.page_fault_inst[i] = src->page_fault_inst[i];
    out.pf_inst_hi[i] = src->pf_inst_hi[i];
    out.predict_dir[i] = src->predict_dir_corrected[i];
    out.pc[i] = src->predict_base_pc[i];
    out.inst_valid[i] = src->inst_valid[i];
    out.is_rvc[i] = src->is_rvc[i];
  }
  out.meta.assign_prefix(src->meta);
  for (int i = FETCH_WIDTH; i < BACKEND_FETCH_WIDTH; ++i) {
    out.instructions[i] = INST_NOP;
    out.page_fault_inst[i] = false;
    out.pf_inst_hi[i] = false;
    out.inst_valid[i] = false;
  }
  out.predict_next_fetch_address = src->predict_next_fetch_address_corrected;
}


// ---- the front end's control (front-end/front_ctrl.h) ----

// Flush / refetch of the cycle and the BPU pipeline's last-stage decision (from the registers).
void FrontControl::control() {
  out.front_state.valid = false;
  out.pipe_dec = {};
  out.global_reset = in.in->ctrl.reset;
  out.global_refetch = in.in->redirect.valid || in.rd->q.predecode_refetch;
  // BPU_PIPE: the pipeline's last stage may override an older block this cycle. The youngest squash_n fetch addresses are
  // dropped, and the FIFO head is not popped when it is one of them. An entry is popped only when it is at least
  // max(0, D-2) cycles old, so a block is never inside the ICache before the decision on its predecessor.
  bpu_pipe->decide(out.global_refetch, out.pipe_dec);
  const auto &fa = in.rd->fetch_addr_fifo_rd_snapshot;
  const int min_age = bpu_pipe->depth() > 2 ? bpu_pipe->depth() - 2 : 0;
  // Timing cut (EXP-044): the decision of the BPU's last stage does not gate the pop in the same cycle; an entry is
  // popped one cycle later, when every decision that can squash it has been made (and applied) in an earlier cycle.
  out.pipe_head_ok = fa.head_age >= (bpu_pipe->depth() >= 2 ? min_age + 1 : min_age);
}

// The read enables of the FIFOs and the PTAB write of the BPU's block (bypassed to the checker when the last stage
// decides this cycle).
void FrontControl::enables() {
  const bpuf::OutputPayload &bpu_output = *in.bpu_output;

  const bool icache_ready = icache->read_ready(in.in->ctrl.itlb_flush || in.in->ctrl.fence_i, out.global_reset);
  out.ptab_can_write = bpu_output.PTAB_write_enable && !out.global_reset && !out.global_refetch;
  out.ptab_bypass = out.ptab_can_write && out.pipe_dec.valid;
  out.fetch_addr_read_enable = icache_ready && !in.rd->q.fetch_addr_fifo_empty &&
                           !out.global_reset && !out.global_refetch && out.pipe_head_ok;
  out.fifo_read_enable = !in.rd->q.fifo_empty && (!in.rd->q.ptab_empty || out.ptab_bypass) &&
                     !in.rd->q.f2b_full && !out.global_reset && !out.global_refetch;
  out.f2b_read_enable = in.in->ctrl.fifo_read_enable;
}

// The front end's registers of the next cycle, the output toward the back end, the PMU delivery split.
void FrontControl::finish() {
  const bpuf::OutputPayload &bpu_output = *in.bpu_output;
  const FetchAddrFifoOut &fetch_addr_head = *in.fetch_addr_head;
  const FetchAddrFifoReadData &fetch_addr_rd = *in.fetch_addr_rd;
  const InstFifoReadData &fifo_rd = *in.fifo_rd;
  const PtabReadData &ptab_rd = *in.ptab_rd;
  const F2bFifoReadData &f2b_rd = *in.f2b_rd;
  const F2bFifoOut &f2b_head = *in.f2b_head;
  const F2bFifoOut &f2b_bypass = *in.f2b_bypass;
  const bool use_f2b_bypass = *in.use_f2b_bypass;
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
  out.front_state.d.fetch_addr_fifo_full = (fetch_addr_rd.size >= (FETCH_ADDR_FIFO_SIZE - 1));
  out.front_state.d.fetch_addr_fifo_empty = (fetch_addr_rd.size == 0);
  out.front_state.d.fifo_full = (fifo_rd.size == INSTRUCTION_FIFO_SIZE);
  out.front_state.d.fifo_empty = (fifo_rd.size == 0);
  out.front_state.d.ptab_empty = (ptab_rd.size == 0);
  out.front_state.d.f2b_full = (f2b_rd.size == FRONT2BACK_FIFO_SIZE);
  out.front_state.d.f2b_empty = (f2b_rd.size == 0);

  front_output(f2b_head, f2b_bypass, use_f2b_bypass, *out.out);
  out.out->commit_stall = bpu_output.update_queue_full;
  out.out->commit_upd_free = bpu_output.upd_free;
  out.out->commit_tq_free = bpu_output.tq_free;
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
      bool no_taken = true;
      for (int j = 0; j < FETCH_WIDTH; j++) {   // the same cut as SimCpu::front_comb_before_back
        if (j > 0 && !no_taken && out.out->inst_valid[j - 1] && out.out->inst_valid[j] && out.out->pc[j] != out.out->pc[j - 1] + ilen_of(out.out->is_rvc[j - 1]) && out.out->predict_dir[j - 1]) no_taken = true;
        const bool v = no_taken && out.out->FIFO_valid && out.out->inst_valid[j];
        if (v) { pf.feo_n++; last = j; if (out.out->predict_dir[j]) no_taken = false; }
      }
      if (pf.feo_n > 0) {
        if (pf.feo_n < FETCH_WIDTH) {
          const uint32_t line = ICACHE_LINE_SIZE;
          if (out.out->predict_dir[last]) pf.feo_cause = PMU_FEO_LOSS_TAKEN;
          else if (((out.out->pc[last] + 4u) & ~(line - 1u)) != (out.out->pc[last] & ~(line - 1u))) pf.feo_cause = PMU_FEO_LOSS_LINE;
          else pf.feo_cause = PMU_FEO_LOSS_SHORTOTH;
        }
      } else if (out.global_refetch || cyc - last_refetch <= 6) pf.feo_cause = PMU_FEO_LOSS_REFETCH;
      else if (ic_last.perf_miss_busy) pf.feo_cause = PMU_FEO_LOSS_ICMISS;
      else if (predecode_flush) pf.feo_cause = PMU_FEO_LOSS_PDFLUSH;
      else if (in.rd->q.fetch_addr_fifo_empty && !bpu_output.icache_read_valid) {
        if (out.pipe_dec.ovr) pf.feo_cause = PMU_FEO_LOSS_OVR_SD;
        else if (*in.pmu_l1_ovr) pf.feo_cause = PMU_FEO_LOSS_OVR_L1;
        else if (in.rd->q.fetch_addr_fifo_full || *in.pmu_ptab_stall) pf.feo_cause = PMU_FEO_LOSS_PTABSTALL;
        else pf.feo_cause = PMU_FEO_LOSS_BPUOTHER;
      }
      else if (!ic_last.icache_read_ready) pf.feo_cause = PMU_FEO_LOSS_ICNOTRDY;
      else if (in.rd->q.fifo_empty) pf.feo_cause = PMU_FEO_LOSS_IFIFO;
      else if (in.rd->q.ptab_empty) pf.feo_cause = PMU_FEO_LOSS_PTABLAG;
      else pf.feo_cause = PMU_FEO_LOSS_F2B;
    }
  }
}

// A predecode flush: the I-cache's invalidate step K, whose outputs replace the fetch step's (ic_k).
void ICacheTop::comb_invalidate() {
  out.ic_k = {};
  if (*in.predecode_flush) invalidate_step(&out.ic_k);
}

// ---- the FIFO ports (front-end/front_fifo_ports.h) ----

void FetchAddrPort::comb_read() {
  const bool global_reset = *in.global_reset, global_refetch = *in.global_refetch;
  const bpuf::Decision &pipe_dec = *in.pipe_dec;
  out.fetch_addr_in = {};
  out.fetch_addr_rd = in.rd->fetch_addr_fifo_rd_snapshot;
  out.fetch_addr_head = {};
  out.fetch_addr_fifo_req.clear_fifo = out.fetch_addr_fifo_req.push_en = out.fetch_addr_fifo_req.pop_en = false;
  out.fetch_addr_fifo_req.drop_tail_n = 0;
  out.fetch_addr_in.drop_tail_n = pipe_dec.ovr ? pipe_dec.squash_n : 0;
  out.fetch_addr_in.reset = global_reset;
  out.fetch_addr_in.refetch = global_refetch;
  out.fetch_addr_in.read_enable = *in.fetch_addr_read_enable;
  fifo->read(out.fetch_addr_in, out.fetch_addr_rd, out.fetch_addr_head, out.fetch_addr_fifo_req);
}

void FetchAddrPort::comb_write() {
  const bpuf::OutputPayload &bpu_output = *in.bpu_output;
  const bool bpu_can_run = *in.bpu_can_run, global_reset = *in.global_reset;

  if (!(bpu_output.icache_read_valid && bpu_can_run && !global_reset)) return;
  out.fetch_addr_in.fetch_address = bpu_output.fetch_address;
  out.fetch_addr_in.pair = FetchPair{};
  if (bpu_output.pair_valid) { out.fetch_addr_in.pair.valid = true; out.fetch_addr_in.pair.addr2 = bpu_output.pair_addr2; out.fetch_addr_in.pair.a_len = bpu_output.pair_a_len; out.fetch_addr_in.pair.b_len = bpu_output.pair_b_len; }
  fifo->write(out.fetch_addr_in, out.fetch_addr_rd, out.fetch_addr_fifo_req);
}

// The head of the instruction FIFO as the checker sees it: FE_PAIR keeps only the slots that agree with the PTAB entry;
// RVC=1 joins the carried half of a straddling instruction (rvc_head_fix).
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
  out.grp_ptab = *in.ptab_head;
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
  const ICacheOut &ic_out = *in.ic_out;
  const FetchAddrFifoOut &fetch_addr_head = *in.fetch_addr_head;

  if (in.rd->q.fifo_full || !ic_out.icache_read_complete || global_reset || global_refetch) return;
  // Second block of the request that completes now (if the entry had one and its line can be read in the same cycle)
  FetchPair pr{};
  if (fetch_addr_head.read_valid && fetch_addr_head.fetch_address == ic_out.fetch_pc) pr = fetch_addr_head.pair;
  else if (in.rd->q.pend_valid && in.rd->q.pend_addr == ic_out.fetch_pc) pr = in.rd->q.pend_pair;
  inst_word_t grp[FETCH_WIDTH]; pc_t gpc[FETCH_WIDTH]; wire<1> gv[FETCH_WIDTH], gpf[FETCH_WIDTH];
  for (int i = 0; i < FETCH_WIDTH; i++) { grp[i] = ic_out.fetch_group[i]; gpc[i] = ic_out.fetch_pc + (i * SLOT_BYTES); gv[i] = ic_out.inst_valid[i]; gpf[i] = ic_out.page_fault_inst[i]; }
  const bool want_pair = pr.valid && pr.a_len > 0 && pr.a_len < FETCH_WIDTH && !ic_out.page_fault_inst[0] && !ic_out.fetch_single_word && ic_out.inst_valid[pr.a_len - 1];
  if (want_pair) {
    // Save intact block A before fusing B, modelling the S2-confirmed A/B lane mux
    inst_word_t agrp[FETCH_WIDTH]; pc_t apc[FETCH_WIDTH]; wire<1> agv[FETCH_WIDTH], agpf[FETCH_WIDTH];
    for (int i = 0; i < FETCH_WIDTH; i++) { agrp[i] = grp[i]; apc[i] = gpc[i]; agv[i] = gv[i]; agpf[i] = gpf[i]; }
    uint16_t a_up16 = 0; bool a_up_ok = false;
    if constexpr (kRvc) {
      int anv = 0;
      while (anv < FETCH_WIDTH && agv[anv]) anv++;
      if (anv > 0) {
        a_up16 = uint16_t(agrp[anv - 1] >> 16);
        a_up_ok = ((apc[anv - 1] + SLOT_BYTES) & (ICACHE_LINE_SIZE - 1u)) != 0;
        agrp[anv - 1] &= 0xffffu;
      }
    }
    out.fifo_in.wdata.has_alt_a = true;
    out.fifo_in.wdata.alt_up16 = a_up16;
    out.fifo_in.wdata.alt_up_ok = a_up_ok;
    for (int i = 0; i < FETCH_WIDTH; i++) {
      out.fifo_in.wdata.alt_raw16[i] = uint16_t(agrp[i] & 0xffffu);
      out.fifo_in.wdata.alt_instructions[i] = agrp[i];
      out.fifo_in.wdata.alt_pc[i] = apc[i];
      out.fifo_in.wdata.alt_page_fault_inst[i] = agpf[i];
      out.fifo_in.wdata.alt_inst_valid[i] = agv[i];
      out.fifo_in.wdata.alt_is_rvc[i] = false;
      if (agv[i]) {
        if constexpr (kRvc) {
          if ((agrp[i] & 3u) != 3u) {
            uint32_t ex = 0;
            out.fifo_in.wdata.alt_is_rvc[i] = true;
            agrp[i] = rvc::expand(agrp[i] & 0xffffu, ex) ? ex : 0u;
            out.fifo_in.wdata.alt_instructions[i] = agrp[i];
          }
        }
        const PredecodeResult res = predecode(agrp[i], apc[i]);
        out.fifo_in.wdata.alt_predecode_type[i] = res.type;
        out.fifo_in.wdata.alt_predecode_target_address[i] = res.target_address;
      } else {
        out.fifo_in.wdata.alt_predecode_type[i] = PREDECODE_NON_BRANCH;
        out.fifo_in.wdata.alt_predecode_target_address[i] = 0;
      }
    }
    if constexpr (kRvc) {
      int an = 0;
      while (an < FETCH_WIDTH && agv[an]) an++;
      for (int pass = 0; pass < 2; pass++) {
        wire<1> *st = pass ? out.fifo_in.wdata.alt_start_b : out.fifo_in.wdata.alt_start_a;
        for (int i = 0; i < FETCH_WIDTH; i++) st[i] = false;
        int pos = pass;
        while (pos < an) {
          st[pos] = true;
          pos += (out.fifo_in.wdata.alt_is_rvc[pos] ? 1 : 2);
        }
      }
    }
    out.fifo_in.wdata.alt_seq_next_pc = sequential_next_pc(ic_out.fetch_pc, ic_out.inst_valid);

    uint32_t w2[FETCH_WIDTH];
    int actual_b = 0;
    const bool ok = icache->fetch_second(ic_out.fetch_pc, ic_out.fetch_paddr, pr.addr2, pr.b_len, w2, actual_b);
    for (int i = pr.a_len; i < FETCH_WIDTH; i++) {
      const int j = i - pr.a_len;
      gpc[i] = pr.addr2 + SLOT_BYTES * j;
      gv[i] = ok && j < actual_b; gpf[i] = false; grp[i] = ok && j < actual_b ? w2[j] : INST_NOP;
    }
    if (!ok) for (int i = pr.a_len; i < FETCH_WIDTH; i++) gv[i] = false;
  }
  uint16_t rvc_up16 = 0; bool rvc_up_ok = false;
  if constexpr (kRvc) {   // C910-style carry: the window of the last slot of a group has no upper half (a 32-bit instruction starting there is carried to the next group)
    int nv = 0;
    while (nv < FETCH_WIDTH && gv[nv]) nv++;
    if (nv > 0) {
      // the upper half of the last window is in the same cache line as the slot: the fetch can deliver it with the group
      rvc_up16 = uint16_t(grp[nv - 1] >> 16);
      rvc_up_ok = !ic_out.fetch_single_word && ((gpc[nv - 1] + SLOT_BYTES) & (ICACHE_LINE_SIZE - 1u)) != 0;
      grp[nv - 1] &= 0xffffu;   // (an inner FE_PAIR block end keeps its window: the upper half is in the same fetched line)
    }
  }
  out.fifo_in.wdata.up16 = rvc_up16; out.fifo_in.wdata.up_ok = rvc_up_ok;
  for (int i = 0; i < FETCH_WIDTH; i++) {
    out.fifo_in.wdata.raw16[i] = uint16_t(grp[i] & 0xffffu);
    out.fifo_in.wdata.instructions[i] = grp[i];
    out.fifo_in.wdata.pc[i] = gpc[i];
    out.fifo_in.wdata.page_fault_inst[i] = gpf[i];
    out.fifo_in.wdata.inst_valid[i] = gv[i];
    if (gv[i]) {
      if constexpr (kRvc) {   // the window at this slot is a 16-bit instruction when its low bits are not 11: expand it (an illegal encoding becomes 0)
        if ((grp[i] & 3u) != 3u) {
          uint32_t ex = 0;
          out.fifo_in.wdata.is_rvc[i] = true;
          grp[i] = rvc::expand(grp[i] & 0xffffu, ex) ? ex : 0u;
          out.fifo_in.wdata.instructions[i] = grp[i];
        }
      }
      const PredecodeResult result = predecode(grp[i], gpc[i]);
      out.fifo_in.wdata.predecode_type[i] = result.type;
      out.fifo_in.wdata.predecode_target_address[i] = result.target_address;
    } else {
      out.fifo_in.wdata.predecode_type[i] = PREDECODE_NON_BRANCH;
      out.fifo_in.wdata.predecode_target_address[i] = 0;
    }
  }
  if constexpr (kRvc) {   // instruction starts: from slot 0 (start_a) or, when slot 0 is the second half of the previous block's last instruction, from slot 1 (start_b)
    // FE_PAIR: the group may hold two fetch blocks; each block's first slot is an instruction start and a block's last instruction (a 32-bit branch whose
    // window is complete) does not run into the next block's first slot.
    int n = 0;
    while (n < FETCH_WIDTH && gv[n]) n++;
    auto boundary = [&](int k) { return k > 0 && k < n && gpc[k] != gpc[k - 1] + SLOT_BYTES; };
    for (int pass = 0; pass < 2; pass++) {
      wire<1> *st = pass ? out.fifo_in.wdata.start_b : out.fifo_in.wdata.start_a;
      int pos = pass;
      while (pos < n) {
        st[pos] = true;
        int nxt = pos + (out.fifo_in.wdata.is_rvc[pos] ? 1 : 2);
        for (int k = pos + 1; k < nxt && k < n; k++) if (boundary(k)) { nxt = k; break; }
        pos = nxt;
      }
    }
  }
  if (want_pair) {   // 2-taken pair: next pc after the last valid slot
    int last = -1;
    for (int i = 0; i < FETCH_WIDTH; i++) if (out.fifo_in.wdata.inst_valid[i]) last = i;
    out.fifo_in.wdata.seq_next_pc = out.fifo_in.wdata.pc[last] + SLOT_BYTES;   // the position after the last slot (RVC=1: a straddling last instruction continues there)
  } else
  out.fifo_in.wdata.seq_next_pc = sequential_next_pc(ic_out.fetch_pc, ic_out.inst_valid);

  fifo->write(out.fifo_in, out.fifo_rd, out.fifo_req);
}

// The block leaving S_D enters the PTAB; when the checker reads the PTAB in the same cycle (bypass) it is written
// before the read.
void PtabPort::comb_read() {
  const bool global_reset = *in.global_reset, global_refetch = *in.global_refetch;
  const bpuf::OutputPayload &bpu_output = *in.bpu_output;
  out.ptab_in = {};
  out.ptab_rd = in.rd->ptab_rd_snapshot;
  out.ptab_head = {};
  out.ptab_req.clear_fifo = out.ptab_req.push_en = out.ptab_req.pop_en = false;
  if (*in.ptab_bypass) { ptab_write_data(bpu_output, global_reset, global_refetch, out.ptab_in); out.ptab_in.write_enable = true; fpr_ptab(out.ptab_in); }
  out.ptab_in.reset = global_reset;
  out.ptab_in.refetch = global_refetch;
  out.ptab_in.read_enable = *in.fifo_read_enable;
  fifo->read(out.ptab_in, out.ptab_rd, out.ptab_head, out.ptab_req);
  if (*in.ptab_can_write && !*in.ptab_bypass) {   // the write port (after the read port)
    ptab_write_data(bpu_output, global_reset, global_refetch, out.ptab_in);
    fpr_ptab(out.ptab_in);
    fifo->write(out.ptab_in, out.ptab_rd, out.ptab_req);
  }
}

void F2bPort::comb_read() {
  out.f2b_in = {};
  out.f2b_rd = in.rd->front2back_fifo_rd_snapshot;
  out.f2b_head = {};
  out.f2b_bypass = {};
  out.use_f2b_bypass = false;
  out.front2back_fifo_req.clear_fifo = out.front2back_fifo_req.push_en = out.front2back_fifo_req.pop_en = false;
  out.f2b_in.reset = *in.global_reset;
  out.f2b_in.refetch = in.in->redirect.valid;
  out.f2b_in.read_enable = *in.f2b_read_enable;
  fifo->read(out.f2b_in, out.f2b_rd, out.f2b_head, out.front2back_fifo_req);
}

// The checked entry goes to the front2back FIFO, or straight to the output when the FIFO is empty and the back end is
// reading this cycle.
void F2bPort::comb_write() {
  const bool can_write = *in.fifo_read_enable && !in.rd->q.f2b_full && !*in.global_reset;
  out.use_f2b_bypass = *in.f2b_read_enable && in.rd->q.f2b_empty &&
                   !out.f2b_head.front2back_FIFO_valid && can_write;
  if (!can_write) return;

  f2b_write_data(*in.fifo_head, *in.grp_ptab, *in.checker_out, out.use_f2b_bypass, out.f2b_in, out.f2b_bypass);
  if (!out.use_f2b_bypass) fifo->write(out.f2b_in, out.f2b_rd, out.front2back_fifo_req);
}

// The front end's BPU phase (front-end/BPU/BPU_pipe.h).
void bpuf::BpuPipe::comb_front() {
  out = {};
  const bool global_reset = *in.global_reset, global_refetch = *in.global_refetch;
  const Decision &pipe_dec = *in.pipe_dec;

  fpr::set_cycle(ctx ? ctx->perf.cycle : 0);
  const bool bpu_stall = in.rd->q.fetch_addr_fifo_full;
  out.bpu_can_run = (!bpu_stall || global_reset || global_refetch);

  bpuf::InputPayload bpu_input{};
  bpu_input.refetch = global_refetch;
  bpu_input.refetch_pd = global_refetch && !in.in->redirect.valid;
  bpu_input.rf_valid = in.in->redirect.valid && in.in->redirect.rf_valid;
  bpu_input.rf_blk = in.in->redirect.rf_blk; bpu_input.rf_slot = in.in->redirect.rf_slot; bpu_input.rf_type = in.in->redirect.rf_type; bpu_input.rf_pc = in.in->redirect.rf_pc; bpu_input.rf_is_rvc = in.in->redirect.rf_is_rvc;
  bpu_input.pd = in.rd->q.pd;
  bpu_input.refetch_address = in.in->redirect.valid ? in.in->redirect.address : in.rd->q.predecode_refetch_address;
  bpu_input.icache_read_ready = out.bpu_can_run && !in.rd->q.fetch_addr_fifo_full;
  bpu_input.ptab_size = in.rd->ptab_rd_snapshot.size > 255 ? 255 : in.rd->ptab_rd_snapshot.size;
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
  fa_port.fifo = &fetch_addr_fifo;
  inst_port.fifo = &inst_fifo;
  ptab_port.fifo = &ptab;
  f2b_port.fifo = &f2b_fifo;
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
  icache.in.fetch_addr_head = &fa_port.out.fetch_addr_head;
  icache.in.predecode_flush = &checker.out.predecode_flush;
  bpu_pipe.in.global_reset = &ctrl.out.global_reset;
  bpu_pipe.in.global_refetch = &ctrl.out.global_refetch;
  bpu_pipe.in.pipe_dec = &ctrl.out.pipe_dec;
  fa_port.in.global_reset = &ctrl.out.global_reset;
  fa_port.in.global_refetch = &ctrl.out.global_refetch;
  fa_port.in.fetch_addr_read_enable = &ctrl.out.fetch_addr_read_enable;
  fa_port.in.bpu_can_run = &bpu_pipe.out.bpu_can_run;
  fa_port.in.pipe_dec = &ctrl.out.pipe_dec;
  fa_port.in.bpu_output = &bpu_pipe.out.bpu_output;
  inst_port.in.global_reset = &ctrl.out.global_reset;
  inst_port.in.global_refetch = &ctrl.out.global_refetch;
  inst_port.in.fifo_read_enable = &ctrl.out.fifo_read_enable;
  inst_port.in.ptab_head = &ptab_port.out.ptab_head;
  inst_port.in.ic_out = &icache.out.ic_out;
  inst_port.in.fetch_addr_head = &fa_port.out.fetch_addr_head;
  ptab_port.in.global_reset = &ctrl.out.global_reset;
  ptab_port.in.global_refetch = &ctrl.out.global_refetch;
  ptab_port.in.fifo_read_enable = &ctrl.out.fifo_read_enable;
  ptab_port.in.ptab_bypass = &ctrl.out.ptab_bypass;
  ptab_port.in.ptab_can_write = &ctrl.out.ptab_can_write;
  ptab_port.in.bpu_output = &bpu_pipe.out.bpu_output;
  f2b_port.in.global_reset = &ctrl.out.global_reset;
  f2b_port.in.f2b_read_enable = &ctrl.out.f2b_read_enable;
  f2b_port.in.fifo_read_enable = &ctrl.out.fifo_read_enable;
  f2b_port.in.fifo_head = &inst_port.out.fifo_head;
  f2b_port.in.grp_ptab = &inst_port.out.grp_ptab;
  f2b_port.in.checker_out = &checker.out.checker_out;
  ctrl.in.bpu_output = &bpu_pipe.out.bpu_output;
  ctrl.in.fetch_addr_head = &fa_port.out.fetch_addr_head;
  ctrl.in.fetch_addr_rd = &fa_port.out.fetch_addr_rd;
  ctrl.in.fifo_rd = &inst_port.out.fifo_rd;
  ctrl.in.ptab_rd = &ptab_port.out.ptab_rd;
  ctrl.in.f2b_rd = &f2b_port.out.f2b_rd;
  ctrl.in.f2b_head = &f2b_port.out.f2b_head;
  ctrl.in.f2b_bypass = &f2b_port.out.f2b_bypass;
  ctrl.in.use_f2b_bypass = &f2b_port.out.use_f2b_bypass;
  ctrl.in.ic_out = &icache.out.ic_out;
  ctrl.in.ic_k = &icache.out.ic_k;
  ctrl.in.predecode_flush = &checker.out.predecode_flush;
  ctrl.in.lh_new_valid = &checker.out.lh_new_valid;
  ctrl.in.lh_updated = &checker.out.lh_updated;
  ctrl.in.flush_address = &checker.out.flush_address;
  ctrl.in.lh_new_pc = &checker.out.lh_new_pc;
  ctrl.in.lh_new_data = &checker.out.lh_new_data;
  ctrl.in.pd_next = &checker.out.pd_next;
  ctrl.in.pmu_l1_ovr = &bpu_pipe.out.pmu_l1_ovr;
  ctrl.in.pmu_ptab_stall = &bpu_pipe.out.pmu_ptab_stall;
}
FrontCore::~FrontCore() = default;

// The front end's inputs, registers and output (FrontTop's) and the context
void FrontCore::bind(const FrontTopIn *in, const FrontReadData *rd, FrontTopOut *out, SimContext *c) {
  ctx = c;
  bpu_pipe.ctx = c;
  ctrl.ctx = c;
  icache.in.in = in;
  bpu_pipe.in.in = in;
  f2b_port.in.in = in;
  ctrl.in.in = in;
  bpu_pipe.in.rd = rd;
  fa_port.in.rd = rd;
  inst_port.in.rd = rd;
  ptab_port.in.rd = rd;
  f2b_port.in.rd = rd;
  ctrl.in.rd = rd;
  ctrl.out.out = out;
}


// All phases of one cycle (init / step_bpu)
void FrontCore::comb() {
  // BEGIN GENERATED by scripts/comb/gen_top_order.py from the module contracts: do not edit
  ctrl.control();
  bpu_pipe.comb_front();
  ctrl.enables();
  fa_port.comb_read();
  ptab_port.comb_read();
  inst_port.comb_read();
  f2b_port.comb_read();
  fa_port.comb_write();
  icache.comb_fetch();
  inst_port.comb_write();
  checker.comb();
  f2b_port.comb_write();
  icache.comb_invalidate();
  ctrl.finish();
  // END GENERATED
}

void FrontCore::read_q(FrontReadData &rd) const {
  rd.q = q_;
  fetch_addr_fifo.read_q(rd.fetch_addr_fifo_rd_snapshot);
  inst_fifo.read_q(rd.fifo_rd_snapshot);
  ptab.read_q(rd.ptab_rd_snapshot);
  f2b_fifo.read_q(rd.front2back_fifo_rd_snapshot);
}

void FrontCore::seq(const FrontTopIn &inp) {
  icache.seq();   // first: the I-cache edge comes before everything below that reads the arrays / ITLB (FE_IPF)
  if (bpu_pipe.out.bpu_seq_txn.valid) {
    if (bpu_pipe.out.bpu_seq_txn.reset) bpu_pipe.reset();
    else bpu_pipe.seq();
    if (fpr::on() && fpr::in_dump()) bpu_pipe.dump_hist(static_cast<uint32_t>(fpr::cycle()));
  }
  fetch_addr_fifo.seq(fa_port.out.fetch_addr_fifo_req);
  inst_fifo.seq(inst_port.out.fifo_req);
  ptab.seq(ptab_port.out.ptab_req);
  f2b_fifo.seq(f2b_port.out.front2back_fifo_req);
  if (ctrl.out.front_state.valid) {
    pc_t s2c[1];
    int n = 0;
    if (bpu_pipe.out.bpu_output.PTAB_write_enable) s2c[n++] = bpu_pipe.out.bpu_output.predict_next_fetch_address;
    icache.enqueue_fdip(s2c, n, ctrl.out.global_refetch || ctrl.out.pipe_dec.ovr);
    q_ = ctrl.out.front_state.d;
  }
}
