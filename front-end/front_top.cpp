#include "../function-model/include/rvc.h"
#include "trace_hook.h"
#include "BPU/BPU.h"
#include "BPU/BPU_pipe.h"
#include "Pmu.h"
#include "front_IO.h"
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
#include <vector>
#include <cstdlib>

static FrontRegs g_regs;   // the front end's own registers (q)
// ---- FE_IPF: I-cache prefetch driven by the BPU run-ahead. Every fetch address the BPU pushes into the fetch-address FIFO is a candidate: if its line
// is not in the I-cache, has been filled before (so it is assumed to sit in the LLC) and no prefetch of it is in flight, a prefetch is started and the
// line is installed into the I-cache after FE_IPF_LAT cycles (tag + valid only; LLC / bus traffic of the prefetch is not modelled).
bool icache_peek_translate(uint32_t v_addr, CsrStatusIO *status, uint32_t &p_addr);
bool icache_prefetch_install(uint32_t index, uint32_t ptag);
bool icache_peek_hit(uint32_t index, uint32_t ptag);
extern void (*g_icache_fill_hook)(uint32_t index, uint32_t tag);
namespace ipf {
static const int on = std::getenv("FE_IPF") ? std::atoi(std::getenv("FE_IPF")) : 0;
static const int lat = std::getenv("FE_IPF_LAT") ? std::atoi(std::getenv("FE_IPF_LAT")) : 12;
static const size_t maxfl = 4;   // in-flight instruction-prefetch fills
struct Fl { uint32_t index, tag; uint64_t ready; };
static std::vector<Fl> fl;
static std::unordered_set<uint32_t> seen;   // key = tag << 8 | index
static inline uint32_t key(uint32_t index, uint32_t tag) { return (tag << 8) | index; }
static void note_fill(uint32_t index, uint32_t tag) { seen.insert(key(index, tag)); }
static void init() {
  static bool done = false;
  if (done || !on) return;
  done = true;
  g_icache_fill_hook = note_fill;
}
static void tick(uint64_t now, const uint32_t *cand, int n, CsrStatusIO *csr) {
  if (!on) return;
  init();
  for (size_t i = 0; i < fl.size();) {
    if (fl[i].ready <= now) { icache_prefetch_install(fl[i].index, fl[i].tag); fl.erase(fl.begin() + i); }
    else i++;
  }
  for (int c = 0; c < n; c++) {
    uint32_t pa;
    if (!icache_peek_translate(cand[c], csr, pa)) continue;
    const uint32_t index = (pa >> ICACHE_OFFSET_BITS) & (ICACHE_SET_NUM - 1u), tag = pa >> 12;
    if (icache_peek_hit(index, tag) || !seen.count(key(index, tag))) continue;
    bool dup = false;
    for (const Fl &f : fl) if (f.index == index && f.tag == tag) dup = true;
    if (dup || fl.size() >= maxfl) continue;
    fl.push_back({index, tag, now + uint64_t(lat)});
  }
}
}  // namespace ipf


static SimContext *front_ctx = nullptr;

void front_set_context(SimContext *ctx) { front_ctx = ctx; }

// The pipelined predictor; its per-cycle decision and transaction are kept in statics (large).
static bpuf::BpuPipe bpu_pipe;
void front_warm_commit(pc_t pc, br_type_t type, bool dir, target_addr_t target, bool rvc) {  // fast-warmup replay (EXP-011)
  bpu_pipe.warm_commit(pc, type, dir, target, rvc);
}
void front_warm_finish() { bpu_pipe.warm_finish(); }

static pc_t sequential_next_pc(pc_t fetch_pc, const wire1_t *inst_valid) {
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
[[maybe_unused]] static RvcHeadFix rvc_head_fix(instruction_FIFO_out &head, PTAB_out &ptab, bool lh_valid, pc_t lh_pc, uint16_t lh_data) {
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
    predecode_in in{word, ipc};
    predecode_read_data rd{};
    PredecodeResult res{};
    predecode_seq_read(&in, &rd);
    predecode_comb(rd, res);
    head.predecode_type[0] = res.type;
    head.predecode_target_address[0] = res.target_address;
  }
  const wire1_t *chain = r.carry_used ? head.start_b : head.start_a;
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
    predecode_in in{word, head.pc[last]};
    predecode_read_data rd{};
    PredecodeResult res{};
    predecode_seq_read(&in, &rd);
    predecode_comb(rd, res);
    head.predecode_type[last] = res.type;
    head.predecode_target_address[last] = res.target_address;
    head.seq_next_pc = head.pc[last] + 2 * SLOT_BYTES;   // the instruction is complete now: the sequential continuation is behind it, not in the middle of it
  } else if (last >= (r.carry_used ? 1 : 0) && head.inst_valid[last] && !head.is_rvc[last] && !head.page_fault_inst[last] && chain[last]) {
    r.straddle = true;
    r.low = head.raw16[last];
    drop(last);
  }
  return r;
}

// ============================================================================
// Small combinational blocks
// ============================================================================


// Alignment fingerprint of a PTAB write (see BPU/fpr.h)
static void fpr_ptab(const PTAB_in &p) {
  if (!fpr::on()) return;
  uint32_t m = 0;
  for (int i = 0; i < FETCH_WIDTH; i++) m |= uint32_t(p.predict_dir[i]) << i;
  fpr::pt(static_cast<uint32_t>(p.predict_base_pc[0]), static_cast<uint32_t>(p.predict_next_fetch_address), m);
}

// PTAB write port from the BPU output.
static void front_ptab_write_comb(const bpuf::OutputPayload &bpu_output, wire1_t global_reset,
                                  wire1_t global_refetch, PTAB_in &ptab_in) {
  std::memset(&ptab_in, 0, sizeof(ptab_in));
  ptab_in.reset = global_reset;
  ptab_in.refetch = global_refetch;
  ptab_in.read_enable = false;
  ptab_in.write_enable = true;
  for (int i = 0; i < FETCH_WIDTH; i++) {
    ptab_in.predict_dir[i] = bpu_output.out_pred_dir[i];
    ptab_in.predict_base_pc[i] = bpu_output.ptab_pcs_valid ? bpu_output.out_pred_pc[i] : bpu_output.out_pred_base_pc + (i * SLOT_BYTES);
  }
  ptab_in.meta = bpu_output.meta;
  ptab_in.predict_next_fetch_address = bpu_output.predict_next_fetch_address;
}

static void front_checker_input_comb(const instruction_FIFO_out &fifo_out, const PTAB_out &ptab_out,
                                     predecode_checker_in &checker_in) {
  std::memset(&checker_in, 0, sizeof(checker_in));
  for (int i = 0; i < FETCH_WIDTH; i++) {
    checker_in.predict_dir[i] = ptab_out.predict_dir[i];
    checker_in.predecode_type[i] = fifo_out.predecode_type[i];
    checker_in.predecode_target_address[i] = fifo_out.predecode_target_address[i];
  }
  checker_in.seq_next_pc = fifo_out.seq_next_pc;
  checker_in.predict_next_fetch_address = ptab_out.predict_next_fetch_address;
}

// front2back FIFO write data (and, when the entry bypasses the FIFO, the output it would have produced).
static void front_front2back_write_comb(const instruction_FIFO_out &fifo_out, const PTAB_out &ptab_out,
                                        const predecode_checker_out &checker_out, bool bypass,
                                        front2back_FIFO_in &fifo_in, front2back_FIFO_out &bypass_out) {
  for (int i = 0; i < FETCH_WIDTH; i++) {
    fifo_in.fetch_group[i] = fifo_out.instructions[i];
    fifo_in.page_fault_inst[i] = fifo_out.page_fault_inst[i];
    fifo_in.inst_valid[i] = fifo_out.inst_valid[i] && i < int(checker_out.keep_n);
    fifo_in.is_rvc[i] = fifo_out.is_rvc[i];
    fifo_in.predict_dir_corrected[i] = checker_out.predict_dir_corrected[i];
    fifo_in.predict_base_pc[i] = ptab_out.predict_base_pc[i];
    if (bypass) {
      bypass_out.fetch_group[i] = fifo_out.instructions[i];
      bypass_out.page_fault_inst[i] = fifo_out.page_fault_inst[i];
      bypass_out.inst_valid[i] = fifo_out.inst_valid[i] && i < int(checker_out.keep_n);
      bypass_out.is_rvc[i] = fifo_out.is_rvc[i];
      bypass_out.predict_dir_corrected[i] = checker_out.predict_dir_corrected[i];
      bypass_out.predict_base_pc[i] = ptab_out.predict_base_pc[i];
    }
  }
  fifo_in.meta = ptab_out.meta;
  fifo_in.meta.mask();
  fifo_in.predict_next_fetch_address_corrected = checker_out.predict_next_fetch_address_corrected;
  if (bypass) {
    bypass_out.meta = ptab_out.meta;
    bypass_out.front2back_FIFO_valid = true;
    bypass_out.predict_next_fetch_address_corrected = checker_out.predict_next_fetch_address_corrected;
  }
}

// Output toward the back end: the FIFO head, or the bypassed entry when the FIFO was empty.
static void front_output_comb(const front2back_FIFO_out &saved, const front2back_FIFO_out &bypass_out,
                              bool use_bypass, front_top_out &out) {
  std::memset(&out, 0, sizeof(out));
  const front2back_FIFO_out *src = &saved;
  if (!saved.front2back_FIFO_valid && use_bypass) src = &bypass_out;

  out.FIFO_valid = src->front2back_FIFO_valid;
  for (int i = 0; i < FETCH_WIDTH; i++) {
    out.instructions[i] = src->fetch_group[i];
    out.page_fault_inst[i] = src->page_fault_inst[i];
    out.predict_dir[i] = src->predict_dir_corrected[i];
    out.pc[i] = src->predict_base_pc[i];
    out.inst_valid[i] = src->inst_valid[i];
    out.is_rvc[i] = src->is_rvc[i];
  }
  out.meta.assign_prefix(src->meta);
  for (int i = FETCH_WIDTH; i < BACKEND_FETCH_WIDTH; ++i) {
    out.instructions[i] = INST_NOP;
    out.page_fault_inst[i] = false;
    out.inst_valid[i] = false;
  }
  out.predict_next_fetch_address = src->predict_next_fetch_address_corrected;
}

// ============================================================================
// One front-end cycle
// ============================================================================
namespace {

// Working state of one front_comb_calc() call. Each FIFO's read data advances in place as that FIFO is read and
// written; the merged requests go straight into the FrontUpdateRequest.
// The phases' contracts (scripts/comb/gen_front_order.py checks them against the code and generates front_comb_calc()
// from them): signals are the members below (the wires of the cycle), in, rd (the registers, q), req.<x> (the write
// requests applied at the edge, d), out, bpu_pipe.txn (the BPU pipeline's transaction of the cycle). A FIFO's state is
// written by its read port and then its write port; every other signal has one writer.
//   CONTRACT fifo fetch_addr: fetch_addr_in fetch_addr_rd req.fetch_addr_fifo_req  ports: read_fifos write_fetch_addr
//   CONTRACT fifo instruction: fifo_in fifo_rd req.fifo_req  ports: read_fifos write_instruction_fifo
//   CONTRACT fifo front2back: f2b_in f2b_rd req.front2back_fifo_req  ports: read_fifos write_front2back
//   CONTRACT control  in: global_refetch in pipe_dec rd  out: global_refetch global_reset pipe_dec pipe_head_ok
//   CONTRACT run_bpu  in: bpu_can_run bpu_output global_refetch global_reset in pipe_dec rd  out: bpu_can_run bpu_output bpu_pipe.txn req.bpu_seq_txn
//   CONTRACT read_fifos  in: bpu_output f2b_head f2b_in f2b_rd f2b_read_enable fetch_addr_head fetch_addr_in fetch_addr_rd fifo_head fifo_in fifo_rd fifo_read_enable global_refetch global_reset in pipe_dec pipe_head_ok ptab_head ptab_in ptab_rd rd req.fetch_addr_fifo_req req.fifo_req req.front2back_fifo_req req.ptab_req  out: carry_low f2b_head f2b_in f2b_rd f2b_read_enable fetch_addr_head fetch_addr_in fetch_addr_rd fifo_head fifo_in fifo_rd fifo_read_enable grp_in_range multi ptab_head ptab_in ptab_rd req.fetch_addr_fifo_req req.fifo_req req.front2back_fifo_req req.ptab_req rvc_n rvc_straddle
//   CONTRACT write_fetch_addr  in: bpu_can_run bpu_output fetch_addr_in fetch_addr_rd global_reset req.fetch_addr_fifo_req  out: fetch_addr_in fetch_addr_rd req.fetch_addr_fifo_req req.ipf
//   CONTRACT run_icache  in: fetch_addr_head global_refetch global_reset ic_in ic_out in  out: ic_in ic_out
//   CONTRACT write_instruction_fifo  in: fetch_addr_head fifo_in fifo_rd global_refetch global_reset ic_out rd req.fifo_req  out: fifo_in fifo_rd req.fifo_req
//   CONTRACT check_group  in: carry_low checker_out fifo_head fifo_read_enable grp_in_range lh_new_pc multi pd_next predecode_flush ptab_head rvc_n rvc_straddle  out: checker_out flush_address lh_new_data lh_new_pc lh_new_valid lh_updated pd_next predecode_flush
//   CONTRACT write_front2back  in: checker_out f2b_bypass f2b_head f2b_in f2b_rd f2b_read_enable fifo_head fifo_read_enable global_reset ptab_head rd req.front2back_fifo_req use_f2b_bypass  out: f2b_bypass f2b_in f2b_rd req.front2back_fifo_req use_f2b_bypass
//   CONTRACT finish  in: bpu_output bpu_pipe.txn f2b_bypass f2b_head f2b_rd fetch_addr_head fetch_addr_rd fifo_rd flush_address global_refetch global_reset ic_k ic_out in lh_new_data lh_new_pc lh_new_valid lh_updated out pd_next pipe_dec predecode_flush ptab_rd rd use_f2b_bypass  out: ic_k out req.front_state
struct FrontComb {
  const front_top_in &in;
  const FrontReadData &rd;
  front_top_out &out;
  FrontUpdateRequest &req;

  fetch_address_FIFO_read_data fetch_addr_rd;
  instruction_FIFO_read_data fifo_rd;
  PTAB_read_data ptab_rd;
  front2back_FIFO_read_data f2b_rd;

  fetch_address_FIFO_in fetch_addr_in{};
  instruction_FIFO_in fifo_in{};
  PTAB_in ptab_in{};
  front2back_FIFO_in f2b_in{};
  icache_in ic_in{};
  icache_out ic_out{};

  // FIFO heads read this cycle.
  fetch_address_FIFO_out fetch_addr_head{};
  instruction_FIFO_out fifo_head{};
  PTAB_out ptab_head{};
  front2back_FIFO_out f2b_head{};
  front2back_FIFO_out f2b_bypass{};
  bool use_f2b_bypass = false;

  bool global_reset = false;
  bool global_refetch = false;
  bool fifo_read_enable = false;  // instruction FIFO and PTAB are read together
  bool f2b_read_enable = false;
  bpuf::OutputPayload bpu_output{};
  bool bpu_can_run = false;
  bool pipe_head_ok = true;   // the fetch-address FIFO head may be popped (BPU_PIPE: not a block the last stage overrides)
  bool multi = false;         // FE_PAIR: the group at the FIFO head spans two blocks
  wire1_t grp_in_range[FETCH_WIDTH] = {}, grp_seg_first[FETCH_WIDTH] = {};   // the group's slots and each block's first slot (before the RVC head fix)
  int rvc_n = 0;              // RVC=1: slots of the group at the head (before the head fix), its straddle and carried half
  bool rvc_straddle = false;
  uint16_t carry_low = 0;
  bool predecode_flush = false;   // the predecode check of this cycle flushes, to flush_address
  pc_t flush_address = 0;
  icache_out ic_k{};          // the I-cache's outputs of the invalidate step K (a predecode flush), which replace ic_out
  bpuf::PdInfo pd_next{};
  predecode_checker_out checker_out{};
  bool lh_new_valid = false;      // RVC=1: the "last half" state after the group at the FIFO head (set when a head is processed)
  pc_t lh_new_pc = 0;
  uint16_t lh_new_data = 0;
  bool lh_updated = false;
  bpuf::Decision pipe_dec{};      // the BPU pipeline's last-stage decision of this cycle (from its registers)

  FrontComb(const front_top_in &i, const FrontReadData &r, front_top_out &o, FrontUpdateRequest &q)
      : in(i), rd(r), out(o), req(q), fetch_addr_rd(r.fetch_addr_fifo_rd_snapshot), fifo_rd(r.fifo_rd_snapshot),
        ptab_rd(r.ptab_rd_snapshot), f2b_rd(r.front2back_fifo_rd_snapshot) {}

  void control();
  void run_bpu();
  void read_fifos();
  void write_fetch_addr();
  void run_icache();
  void write_instruction_fifo();
  void check_group();
  void write_front2back();
  void finish();

private:
  void fix_heads();
};

// Flush / refetch of the cycle and the BPU pipeline's last-stage decision (from the registers).
void FrontComb::control() {
  global_reset = in.reset;
  global_refetch = in.refetch || rd.q.predecode_refetch;
  // BPU_PIPE: the pipeline's last stage may override an older block this cycle. The youngest squash_n fetch addresses are
  // dropped, and the FIFO head is not popped when it is one of them. An entry is popped only when it is at least
  // max(0, D-2) cycles old, so a block is never inside the ICache before the decision on its predecessor.
  bpu_pipe.decide(global_refetch, pipe_dec);
  const auto &fa = rd.fetch_addr_fifo_rd_snapshot;
  const int min_age = bpu_pipe.depth() > 2 ? bpu_pipe.depth() - 2 : 0;
  pipe_head_ok = !(pipe_dec.ovr && fa.size <= pipe_dec.squash_n) && fa.head_age >= min_age;
}

// The read and write ports of the four FIFOs' heads: read enables, the heads (the fetch-address FIFO feeds the ICache,
// the other three feed the predecode checker and the back end), the PTAB write of the BPU's block, the head fix-ups.
void FrontComb::read_fifos() {
  const bool icache_ready = icache_read_ready(in.itlb_flush || in.fence_i, global_reset);
  fetch_addr_in.drop_tail_n = pipe_dec.ovr ? pipe_dec.squash_n : 0;
  // the block leaving S_D writes the PTAB and the checker may read it in the same cycle (PTAB bypass)
  const bool ptab_can_write = bpu_output.PTAB_write_enable && !global_reset && !global_refetch;
  const bool ptab_bypass = ptab_can_write && pipe_dec.valid;
  if (ptab_bypass) { front_ptab_write_comb(bpu_output, global_reset, global_refetch, ptab_in); ptab_in.write_enable = true; fpr_ptab(ptab_in); }
  const bool fetch_addr_read_enable = icache_ready && !rd.q.fetch_addr_fifo_empty &&
                                      !global_reset && !global_refetch && pipe_head_ok;
  fifo_read_enable = !rd.q.fifo_empty && (!rd.q.ptab_empty || ptab_bypass) &&
                     !rd.q.f2b_full && !global_reset && !global_refetch;
  f2b_read_enable = in.FIFO_read_enable;

  fetch_addr_in.reset = global_reset;
  fetch_addr_in.refetch = global_refetch;
  fetch_addr_in.read_enable = fetch_addr_read_enable;
  fifo_in.reset = global_reset;
  fifo_in.refetch = global_refetch;
  fifo_in.read_enable = fifo_read_enable;
  ptab_in.reset = global_reset;
  ptab_in.refetch = global_refetch;
  ptab_in.read_enable = fifo_read_enable;
  f2b_in.reset = global_reset;
  f2b_in.refetch = in.refetch;
  f2b_in.read_enable = f2b_read_enable;

  // the read ports: each FIFO's request of the cycle starts here
  fetch_address_FIFO_read(&fetch_addr_in, &fetch_addr_rd, &fetch_addr_head, &req.fetch_addr_fifo_req);
  instruction_FIFO_read(&fifo_in, &fifo_rd, &fifo_head, &req.fifo_req);
  PTAB_read(&ptab_in, &ptab_rd, &ptab_head, &req.ptab_req);
  front2back_FIFO_read(&f2b_in, &f2b_rd, &f2b_head, &req.front2back_fifo_req);
  if (ptab_can_write && !ptab_bypass) {   // the PTAB's write port (after its read port)
    front_ptab_write_comb(bpu_output, global_reset, global_refetch, ptab_in);
    fpr_ptab(ptab_in);
    PTAB_write(&ptab_in, &ptab_rd, &req.ptab_req);
  }
  if (fifo_read_enable) fix_heads();
}

// The group at the instruction FIFO head as the checker sees it: FE_PAIR keeps only the slots that agree with the PTAB
// entry; RVC=1 joins the carried half of a straddling instruction (rvc_head_fix).
void FrontComb::fix_heads() {
  static const bool pair_on = bpuf::cfg_pair() != 0;
  if (pair_on) {
    // FE_PAIR: only the slots whose pc agrees with the PTAB entry are usable (the BPU may have replaced the second block / the first block's end)
    bool cut = false;
    for (int i = 0; i < FETCH_WIDTH; i++) {
      if (!cut && fifo_head.inst_valid[i] && fifo_head.pc[i] != ptab_head.predict_base_pc[i]) cut = true;
      if (cut) { fifo_head.inst_valid[i] = false; fifo_head.predecode_type[i] = PREDECODE_NON_BRANCH; fifo_head.predecode_target_address[i] = 0; }
    }
    for (int i = 1; i < FETCH_WIDTH; i++) if (fifo_head.inst_valid[i] && fifo_head.pc[i] != fifo_head.pc[i - 1] + SLOT_BYTES) multi = true;
    if (cut) {  // the group lost slots: its sequential continuation is the pc after the last usable slot
      int last = -1;
      for (int i = 0; i < FETCH_WIDTH; i++) if (fifo_head.inst_valid[i]) last = i;
      if (last >= 0) fifo_head.seq_next_pc = fifo_head.pc[last] + SLOT_BYTES;
    }
  } else
  for (int i = 0; i < FETCH_WIDTH; i++) {
    if (fifo_head.pc[i] != ptab_head.predict_base_pc[i]) {
      printf("ERROR: fifo pc[%d]: %llx != ptab pc[%d]: %llx\n", i, (unsigned long long)fifo_head.pc[i], i, (unsigned long long)ptab_head.predict_base_pc[i]);
      exit(1);
    }
  }
  for (int i = 0; i < FETCH_WIDTH; i++) {
    grp_in_range[i] = fifo_head.inst_valid[i];
    grp_seg_first[i] = i > 0 && fifo_head.inst_valid[i] && fifo_head.inst_valid[i - 1] && fifo_head.pc[i] != fifo_head.pc[i - 1] + SLOT_BYTES;
  }
  if constexpr (kRvc) {
    const RvcHeadFix hf = rvc_head_fix(fifo_head, ptab_head, rd.q.lh_valid, rd.q.lh_pc, rd.q.lh_data);
    rvc_n = hf.n; rvc_straddle = hf.straddle; carry_low = hf.low;
  }
}

// The BPU predicts the next fetch group unless the fetch-address FIFO or PTAB is full (a flush always runs it).
void FrontComb::run_bpu() {
  fpr::set_cycle(front_ctx ? front_ctx->perf.cycle : 0);
  const bool bpu_stall = rd.q.fetch_addr_fifo_full;
  bpu_can_run = (!bpu_stall || global_reset || global_refetch);

  bpuf::InputPayload bpu_input{};
  bpu_input.refetch = global_refetch;
  bpu_input.refetch_pd = global_refetch && !in.refetch;
  bpu_input.rf_valid = in.refetch && in.rf_valid;
  bpu_input.rf_blk = in.rf_blk; bpu_input.rf_slot = in.rf_slot; bpu_input.rf_type = in.rf_type; bpu_input.rf_pc = in.rf_pc; bpu_input.rf_is_rvc = in.rf_is_rvc;
  bpu_input.pd = rd.q.pd;
  bpu_input.refetch_address = in.refetch ? in.refetch_address : rd.q.predecode_refetch_address;
  bpu_input.icache_read_ready = bpu_can_run && !rd.q.fetch_addr_fifo_full;
  bpu_input.ptab_size = rd.ptab_rd_snapshot.size > 255 ? 255 : rd.ptab_rd_snapshot.size;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    bpu_input.in_update_base_pc[i] = in.predict_base_pc[i];
    bpu_input.in_update_is_rvc[i] = in.update_is_rvc[i];
    bpu_input.in_upd_valid[i] = in.back2front_valid[i];
    bpu_input.in_actual_dir[i] = in.actual_dir[i];
    bpu_input.in_actual_br_type[i] = in.actual_br_type[i];
    bpu_input.in_actual_targets[i] = in.actual_target[i];
    bpu_input.in_pred_dir[i] = in.predict_dir[i];
    bpu_input.in_alt_pred[i] = in.alt_pred[i];
    bpu_input.in_pcpn[i] = in.pcpn[i];
    for (int j = 0; j < 4; j++) {
      bpu_input.in_tage_tags[i][j] = in.tage_tag[i][j];
      bpu_input.in_tage_idxs[i][j] = in.tage_idx[i][j];
    }
  }

  req.bpu_seq_txn.valid = true;
  req.bpu_seq_txn.reset = in.reset;
  if (in.reset) {
    bpu_output.fetch_address = RESET_PC;
  } else {
    bpu_pipe.comb_cycle(bpu_input, pipe_dec, bpu_output);
  }
}

// The predicted fetch address enters the fetch-address FIFO (the ICache reads it next cycle).
void FrontComb::write_fetch_addr() {
  if (!(bpu_output.icache_read_valid && bpu_can_run && !global_reset)) return;
  fetch_addr_in.fetch_address = bpu_output.fetch_address;
  req.ipf.ncand = 0;
  req.ipf.cand[req.ipf.ncand++] = bpu_output.fetch_address;
  if (bpu_output.pair_valid) req.ipf.cand[req.ipf.ncand++] = bpu_output.pair_addr2;
  fetch_addr_in.pair = FetchPair{};
  if (bpu_output.pair_valid) { fetch_addr_in.pair.valid = true; fetch_addr_in.pair.addr2 = bpu_output.pair_addr2; fetch_addr_in.pair.a_len = bpu_output.pair_a_len; fetch_addr_in.pair.b_len = bpu_output.pair_b_len; }
  fetch_address_FIFO_write(&fetch_addr_in, &fetch_addr_rd, &req.fetch_addr_fifo_req);
}

// The ICache takes the address popped from the fetch-address FIFO.
void FrontComb::run_icache() {
  ic_in.reset = global_reset;
  ic_in.refetch = global_refetch;
  ic_in.itlb_flush = in.itlb_flush;
  ic_in.fence_i = in.fence_i;
  ic_in.invalidate_req = false;
  ic_in.csr_status = in.csr_status;
  ic_in.icache_read_valid = fetch_addr_head.read_valid;
  ic_in.fetch_address = fetch_addr_head.read_valid ? fetch_addr_head.fetch_address : 0;
  icache_comb_calc(&ic_in, &ic_out);
  g_pmu_front.icache_req_fire = ic_out.perf_req_fire;
  g_pmu_front.icache_miss_issue = ic_out.perf_miss_event;
  g_pmu_front.icache_miss_busy = ic_out.perf_miss_busy;
  g_pmu_front.itlb_access = ic_out.perf_itlb_hit || ic_out.perf_itlb_miss || ic_out.perf_itlb_fault;
  g_pmu_front.itlb_miss = ic_out.perf_itlb_miss;
}

// A completed ICache read is predecoded and written to the instruction FIFO.
void FrontComb::write_instruction_fifo() {
  if (rd.q.fifo_full || !ic_out.icache_read_complete || global_reset || global_refetch) return;
  // FE_PAIR: the second block of the request that completes now (if the entry had one and its line can be read in the same cycle)
  FetchPair pr{};
  if (fetch_addr_head.read_valid && fetch_addr_head.fetch_address == ic_out.fetch_pc) pr = fetch_addr_head.pair;
  else if (rd.q.pend_valid && rd.q.pend_addr == ic_out.fetch_pc) pr = rd.q.pend_pair;
  inst_word_t grp[FETCH_WIDTH]; pc_t gpc[FETCH_WIDTH]; wire1_t gv[FETCH_WIDTH], gpf[FETCH_WIDTH];
  for (int i = 0; i < FETCH_WIDTH; i++) { grp[i] = ic_out.fetch_group[i]; gpc[i] = ic_out.fetch_pc + (i * SLOT_BYTES); gv[i] = ic_out.inst_valid[i]; gpf[i] = ic_out.page_fault_inst[i]; }
  bool fused = false;
  if (pr.valid && pr.a_len > 0 && pr.a_len < FETCH_WIDTH && !ic_out.page_fault_inst[0] && !ic_out.fetch_single_word && ic_out.inst_valid[pr.a_len - 1]) {
    uint32_t w2[FETCH_WIDTH];
    const bool ok = icache_fetch_second(ic_out.fetch_pc, ic_out.fetch_paddr, pr.addr2, pr.b_len, w2);
    for (int i = pr.a_len; i < FETCH_WIDTH; i++) {
      const int j = i - pr.a_len;
      gpc[i] = pr.addr2 + SLOT_BYTES * j;
      gv[i] = ok && j < pr.b_len; gpf[i] = false; grp[i] = ok && j < pr.b_len ? w2[j] : INST_NOP;
    }
    fused = ok;
    if (!ok) for (int i = pr.a_len; i < FETCH_WIDTH; i++) gv[i] = false;   // the group stays the first block only; the PTAB entry (with the second block) then fails the check below
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
  fifo_in.up16 = rvc_up16; fifo_in.up_ok = rvc_up_ok;
  for (int i = 0; i < FETCH_WIDTH; i++) {
    fifo_in.raw16[i] = uint16_t(grp[i] & 0xffffu);
    fifo_in.fetch_group[i] = grp[i];
    fifo_in.pc[i] = gpc[i];
    fifo_in.page_fault_inst[i] = gpf[i];
    fifo_in.inst_valid[i] = gv[i];
    if (gv[i]) {
      if constexpr (kRvc) {   // the window at this slot is a 16-bit instruction when its low bits are not 11: expand it (an illegal encoding becomes 0)
        if ((grp[i] & 3u) != 3u) {
          uint32_t ex = 0;
          fifo_in.is_rvc[i] = true;
          grp[i] = rvc::expand(grp[i] & 0xffffu, ex) ? ex : 0u;
          fifo_in.fetch_group[i] = grp[i];
        }
      }
      predecode_in predecode_inp{grp[i], gpc[i]};
      predecode_read_data predecode_rd{};
      PredecodeResult result{};
      predecode_seq_read(&predecode_inp, &predecode_rd);
      predecode_comb(predecode_rd, result);
      fifo_in.predecode_type[i] = result.type;
      fifo_in.predecode_target_address[i] = result.target_address;
    } else {
      fifo_in.predecode_type[i] = PREDECODE_NON_BRANCH;
      fifo_in.predecode_target_address[i] = 0;
    }
  }
  if constexpr (kRvc) {   // instruction starts: from slot 0 (start_a) or, when slot 0 is the second half of the previous block's last instruction, from slot 1 (start_b)
    // FE_PAIR: the group may hold two fetch blocks; each block's first slot is an instruction start and a block's last instruction (a 32-bit branch whose
    // window is complete) does not run into the next block's first slot.
    int n = 0;
    while (n < FETCH_WIDTH && gv[n]) n++;
    auto boundary = [&](int k) { return k > 0 && k < n && gpc[k] != gpc[k - 1] + SLOT_BYTES; };
    for (int pass = 0; pass < 2; pass++) {
      wire1_t *st = pass ? fifo_in.start_b : fifo_in.start_a;
      int pos = pass;
      while (pos < n) {
        st[pos] = true;
        int nxt = pos + (fifo_in.is_rvc[pos] ? 1 : 2);
        for (int k = pos + 1; k < nxt && k < n; k++) if (boundary(k)) { nxt = k; break; }
        pos = nxt;
      }
    }
  }
  if (pr.valid && pr.a_len > 0 && pr.a_len < FETCH_WIDTH && ic_out.inst_valid[pr.a_len - 1] && !ic_out.fetch_single_word) {   // FE_PAIR: next pc after the last valid slot
    int last = -1;
    for (int i = 0; i < FETCH_WIDTH; i++) if (fifo_in.inst_valid[i]) last = i;
    fifo_in.seq_next_pc = fifo_in.pc[last] + SLOT_BYTES;   // the position after the last slot (RVC=1: a straddling last instruction continues there)
    (void)fused;
  } else
  fifo_in.seq_next_pc = sequential_next_pc(ic_out.fetch_pc, ic_out.inst_valid);

  instruction_FIFO_write(&fifo_in, &fifo_rd, &req.fifo_req);
}

// The BPU's prediction enters the PTAB; the predecode checker compares the FIFO head against the PTAB head and may ask
// for a refetch.
// The predecode check of the group at the FIFO head against its PTAB entry; the RVC=1 carry and the check's report
// to the BPU for the next cycle.
void FrontComb::check_group() {
  if (!fifo_read_enable) return;
  predecode_checker_in checker_in;
  front_checker_input_comb(fifo_head, ptab_head, checker_in);
  checker_in.multi = multi;
  for (int i = 0; i < FETCH_WIDTH; i++) { checker_in.pc[i] = fifo_head.pc[i]; checker_in.inst_valid[i] = fifo_head.inst_valid[i]; checker_in.in_range[i] = grp_in_range[i]; checker_in.seg_first[i] = grp_seg_first[i]; checker_in.is_rvc[i] = fifo_head.is_rvc[i]; }
  predecode_checker_read_data checker_rd{};
  predecode_checker_seq_read(&checker_in, &checker_rd);
  predecode_checker_comb(checker_rd, checker_out);
  if (checker_out.predecode_flush_enable) {
    predecode_flush = true;
    flush_address = checker_out.predict_next_fetch_address_corrected;
  }
  if constexpr (kRvc) {   // the next group starts with the second half of this group's last instruction if it straddles and the fetch continues sequentially
    bool any_taken = false;
    int last_seg = 0;   // FE_PAIR: only the last block of the group decides whether the fetch continues sequentially (a taken branch that ends the first block does not)
    for (int i = 0; i < FETCH_WIDTH; i++) if (grp_seg_first[i]) last_seg = i;
    for (int i = last_seg; i < FETCH_WIDTH; i++) if (fifo_head.inst_valid[i] && checker_out.predict_dir_corrected[i]) any_taken = true;
    lh_updated = true;
    lh_new_pc = fifo_head.pc[rvc_n > 0 ? rvc_n - 1 : 0] + (rvc_n > 0 ? SLOT_BYTES : 0);   // the position after the group's last slot
    // a predecode flush that continues sequentially (corrected target = the end of this group) keeps the carry: the refetched group still begins with the
    // second half of the straddling instruction
    lh_new_data = carry_low;
    lh_new_valid = rvc_straddle && !any_taken && (!checker_out.predecode_flush_enable || checker_out.predict_next_fetch_address_corrected == lh_new_pc);
  }
  {  // BPO_PDCKPT: report the check, and the history / RAS effect of the corrected block, to the BPU next cycle
    pd_next = bpuf::PdInfo{};
    pd_next.valid = true;
    pd_next.flush = predecode_flush;
    if (predecode_flush) {
      for (int i = 0; i < FETCH_WIDTH; i++)   // a taken prediction on an instruction that is not a branch (a dropped slot is not one: the carried instruction may be)
        if (fifo_head.inst_valid[i] && ptab_head.predict_dir[i] && fifo_head.predecode_type[i] == PREDECODE_NON_BRANCH) { pd_next.nb_valid = true; pd_next.nb_pc = fifo_head.pc[i]; break; }
      for (int i = 0; i < FETCH_WIDTH; i++) {
        if (!fifo_head.inst_valid[i]) continue;
        const predecode_type_t ty = fifo_head.predecode_type[i];
        if (ty == PREDECODE_NON_BRANCH) continue;
        const pc_t pc = fifo_head.pc[i]; const uint32_t inst = fifo_head.instructions[i];
        const bool taken = checker_out.predict_dir_corrected[i];
        const pc_t tgt = ty == PREDECODE_JALR ? ptab_head.predict_next_fetch_address : fifo_head.predecode_target_address[i];   // XLEN wide (rv64: a uint32_t here dropped the sign extension of the history symbol's input)
        if (multi && i >= checker_out.keep_n) break;
        if (!taken) continue;
        pd_next.bits |= uint8_t(bpuf::hist_sym(pc, tgt)) << pd_next.nbits; pd_next.nbits += bpuf::HSYM_BITS;
        const uint32_t rd_ = (inst >> 7) & 31, rs1 = (inst >> 15) & 31;
        if (pd_next.ras_op == 0) {
          if (ty == PREDECODE_JAL && rd_ == 1) { pd_next.ras_op = 1; pd_next.ras_val = pc + ilen_of(fifo_head.is_rvc[i]); }
          else if (ty == PREDECODE_JALR && rd_ == 0 && rs1 == 1 && (inst >> 20) == 0) pd_next.ras_op = 2;
        }
        if (!multi || i + 1 >= int(checker_out.keep_n)) break;
      }
    }
  }
}

// The checked entry goes to the front2back FIFO, or straight to the output when the FIFO is empty and the back end is
// reading this cycle.
void FrontComb::write_front2back() {
  const bool can_write = fifo_read_enable && !rd.q.f2b_full && !global_reset;
  use_f2b_bypass = f2b_read_enable && rd.q.f2b_empty &&
                   !f2b_head.front2back_FIFO_valid && can_write;
  if (!can_write) return;

  front_front2back_write_comb(fifo_head, ptab_head, checker_out, use_f2b_bypass, f2b_in, f2b_bypass);
  if (!use_f2b_bypass) front2back_FIFO_write(&f2b_in, &f2b_rd, &req.front2back_fifo_req);
}

// Latches for the next cycle, the predecode-flush request, and the output toward the back end.
void FrontComb::finish() {
  req.front_state.valid = true;
  if (predecode_flush) icache_invalidate_step(in.csr_status, &ic_k);
  const icache_out &ic_last = predecode_flush ? ic_k : ic_out;   // the I-cache's outputs of its last step
  {   // FE_PAIR: remember the second block of the ICache request that is still in flight
    bool nv = rd.q.pend_valid; fetch_addr_t na = rd.q.pend_addr; FetchPair np = rd.q.pend_pair;
    const bool head_done = ic_last.icache_read_complete && fetch_addr_head.read_valid && fetch_addr_head.fetch_address == ic_last.fetch_pc;
    if (ic_last.icache_read_complete && nv && rd.q.pend_addr == ic_last.fetch_pc) nv = false;
    if (fetch_addr_head.read_valid && !head_done) { nv = true; na = fetch_addr_head.fetch_address; np = fetch_addr_head.pair; }
    if (global_refetch || global_reset) nv = false;
    req.front_state.d.pend_valid = nv; req.front_state.d.pend_addr = na; req.front_state.d.pend_pair = np;
  }
  {   // RVC=1 last-half state
    bool nv = lh_updated ? lh_new_valid : rd.q.lh_valid;
    // a back-end redirect (or reset) drops the carry; a predecode flush drops it unless the flushed group itself set it (sequential continuation),
    // and the refetch cycle that follows a predecode flush keeps it
    if (global_reset || in.refetch) nv = false;
    else if (predecode_flush && !(lh_updated && lh_new_valid)) nv = false;
    req.front_state.d.lh_valid = nv; req.front_state.d.lh_pc = lh_updated ? lh_new_pc : rd.q.lh_pc; req.front_state.d.lh_data = lh_updated ? lh_new_data : rd.q.lh_data;
  }
  req.front_state.d.predecode_refetch = predecode_flush;
  req.front_state.d.pd = pd_next;
  req.front_state.d.predecode_refetch_address = predecode_flush ? flush_address : 0;
  req.front_state.d.fetch_addr_fifo_full = (fetch_addr_rd.size >= (FETCH_ADDR_FIFO_SIZE - 1));
  req.front_state.d.fetch_addr_fifo_empty = (fetch_addr_rd.size == 0);
  req.front_state.d.fifo_full = (fifo_rd.size == INSTRUCTION_FIFO_SIZE);
  req.front_state.d.fifo_empty = (fifo_rd.size == 0);
  req.front_state.d.ptab_empty = (ptab_rd.size == 0);
  req.front_state.d.f2b_full = (f2b_rd.size == FRONT2BACK_FIFO_SIZE);
  req.front_state.d.f2b_empty = (f2b_rd.size == 0);

  front_output_comb(f2b_head, f2b_bypass, use_f2b_bypass, out);
  out.commit_stall = bpu_output.update_queue_full;
  out.commit_upd_free = bpu_output.upd_free;
  out.commit_tq_free = bpu_output.tq_free;
  // PMU frontend delivery split (EXP-025): how many instructions of this cycle's fetch group the back end gets, and why the other slots are empty
  {
    const uint64_t cyc = front_ctx ? front_ctx->perf.cycle : 0;
    const uint64_t last_refetch = global_refetch ? cyc : rd.q.pmu_last_refetch;
    req.front_state.d.pmu_last_refetch = last_refetch;
    PmuFrontSignals &pf = g_pmu_front;
    pf.feo_ready = in.FIFO_read_enable;
    pf.feo_n = 0;
    pf.feo_cause = 0;
    if (pf.feo_ready) {
      int last = -1;
      bool no_taken = true;
      for (int j = 0; j < FETCH_WIDTH; j++) {   // the same cut as SimCpu::front_comb_before_back
        if (j > 0 && !no_taken && out.inst_valid[j - 1] && out.inst_valid[j] && out.pc[j] != out.pc[j - 1] + ilen_of(out.is_rvc[j - 1]) && out.predict_dir[j - 1]) no_taken = true;
        const bool v = no_taken && out.FIFO_valid && out.inst_valid[j];
        if (v) { pf.feo_n++; last = j; if (out.predict_dir[j]) no_taken = false; }
      }
      if (pf.feo_n > 0) {
        if (pf.feo_n < FETCH_WIDTH) {
          const uint32_t line = ICACHE_LINE_SIZE;
          if (out.predict_dir[last]) pf.feo_cause = PMU_FEO_LOSS_TAKEN;
          else if (((out.pc[last] + 4u) & ~(line - 1u)) != (out.pc[last] & ~(line - 1u))) pf.feo_cause = PMU_FEO_LOSS_LINE;
          else pf.feo_cause = PMU_FEO_LOSS_SHORTOTH;
        }
      } else if (global_refetch || cyc - last_refetch <= 6) pf.feo_cause = PMU_FEO_LOSS_REFETCH;
      else if (ic_last.perf_miss_busy) pf.feo_cause = PMU_FEO_LOSS_ICMISS;
      else if (predecode_flush) pf.feo_cause = PMU_FEO_LOSS_PDFLUSH;
      else if (rd.q.fetch_addr_fifo_empty && !bpu_output.icache_read_valid) {
        if (pipe_dec.ovr) pf.feo_cause = PMU_FEO_LOSS_OVR_SD;
        else if (bpu_pipe.txn().pmu_l1_ovr) pf.feo_cause = PMU_FEO_LOSS_OVR_L1;
        else if (rd.q.fetch_addr_fifo_full || bpu_pipe.txn().pmu_ptab_stall) pf.feo_cause = PMU_FEO_LOSS_PTABSTALL;
        else pf.feo_cause = PMU_FEO_LOSS_BPUOTHER;
      }
      else if (!ic_last.icache_read_ready) pf.feo_cause = PMU_FEO_LOSS_ICNOTRDY;
      else if (rd.q.fifo_empty) pf.feo_cause = PMU_FEO_LOSS_IFIFO;
      else if (rd.q.ptab_empty) pf.feo_cause = PMU_FEO_LOSS_PTABLAG;
      else pf.feo_cause = PMU_FEO_LOSS_F2B;
    }
  }
}

} // namespace

// Precondition: req is zero-initialised (the merged requests accumulate into it).
void front_comb_calc(const struct front_top_in &inp, const FrontReadData &rd, struct front_top_out &out,
                     FrontUpdateRequest &req) {
  FrontComb c(inp, rd, out, req);
  // BEGIN GENERATED by scripts/comb/gen_front_order.py from the contracts above struct FrontComb: do not edit
  c.control();
  c.run_bpu();
  c.read_fifos();
  c.write_fetch_addr();
  c.run_icache();
  c.write_instruction_fifo();
  c.check_group();
  c.write_front2back();
  c.finish();
  // END GENERATED
}

namespace {

void front_seq_read(const struct front_top_in &inp, FrontReadData &rd) {
  rd.q = g_regs;

  fetch_address_FIFO_in fetch_addr_in;
  instruction_FIFO_in fifo_in;
  PTAB_in ptab_in;
  front2back_FIFO_in front2back_in;
  std::memset(&fetch_addr_in, 0, sizeof(fetch_addr_in));
  std::memset(&fifo_in, 0, sizeof(fifo_in));
  std::memset(&ptab_in, 0, sizeof(ptab_in));
  std::memset(&front2back_in, 0, sizeof(front2back_in));

  fetch_address_FIFO_seq_read(&fetch_addr_in, &rd.fetch_addr_fifo_rd_snapshot);
  instruction_FIFO_seq_read(&fifo_in, &rd.fifo_rd_snapshot);
  PTAB_seq_read(&ptab_in, &rd.ptab_rd_snapshot);
  front2back_FIFO_seq_read(&front2back_in, &rd.front2back_fifo_rd_snapshot);
  (void)inp;
}

void front_seq_write(const struct front_top_in &inp, const FrontUpdateRequest &req,
                     bool reset) {
  icache_seq();   // first: the I-cache edge comes before everything below that reads the arrays / ITLB (FE_IPF)
  (void)inp;
  (void)reset;
  if (req.bpu_seq_txn.valid) {
    if (req.bpu_seq_txn.reset) bpu_pipe.reset();
    else bpu_pipe.seq();
  }
  fetch_address_FIFO_seq_write(&req.fetch_addr_fifo_req);
  instruction_FIFO_seq_write(&req.fifo_req);
  PTAB_seq_write(&req.ptab_req);
  front2back_FIFO_seq_write(&req.front2back_fifo_req);
  predecode_checker_seq_write();

  ipf::tick(front_ctx ? front_ctx->perf.cycle : 0, req.ipf.cand, req.front_state.valid ? req.ipf.ncand : 0, inp.csr_status);
  if (req.front_state.valid) g_regs = req.front_state.d;
}

} // namespace

void front_top_seq_read(const struct front_top_in &inp, FrontReadData &rd) {
  front_seq_read(inp, rd);
}

void front_top_seq_write(const struct front_top_in &inp,
                         const FrontUpdateRequest &req, bool reset) {
  front_seq_write(inp, req, reset);
}
