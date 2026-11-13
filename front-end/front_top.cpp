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

// ============================================================================
// Latched front-end state (registers)
// ============================================================================
static bool predecode_refetch = false;
static pc_t predecode_refetch_address = 0;
static bpuf::PdInfo pd_latch{};
static uint32_t front_sim_time = 0;

// FIFO full/empty latches
static bool fetch_addr_fifo_full_latch = false;
static bool fetch_addr_fifo_empty_latch = true;
static bool fifo_full_latch = false;
static bool fifo_empty_latch = true;
static bool ptab_full_latch = false;
static bool ptab_empty_latch = true;
static bool front2back_fifo_full_latch = false;
static bool front2back_fifo_empty_latch = true;
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
static std::unordered_set<uint32_t> seen, pfd;   // key = tag << 8 | index
static uint64_t n_issue = 0, n_install = 0, n_res = 0, n_unseen = 0, n_noxl = 0, n_full = 0, n_dup = 0, n_useful = 0, n_demand = 0;
static inline uint32_t key(uint32_t index, uint32_t tag) { return (tag << 8) | index; }
static void note_fill(uint32_t index, uint32_t tag) { seen.insert(key(index, tag)); }
static void init() {
  static bool done = false;
  if (done || !on) return;
  done = true;
  g_icache_fill_hook = note_fill;
  std::atexit([] { std::printf("[FE_IPF] lat=%d max=%zu issued=%llu installed=%llu useful=%llu (demand line fetches %llu) | skipped: resident=%llu unseen=%llu dup=%llu full=%llu no-xlate=%llu\n", lat, maxfl,
    (unsigned long long)n_issue, (unsigned long long)n_install, (unsigned long long)n_useful, (unsigned long long)n_demand, (unsigned long long)n_res, (unsigned long long)n_unseen, (unsigned long long)n_dup, (unsigned long long)n_full, (unsigned long long)n_noxl); });
}
static void demand(uint32_t paddr) {
  if (!on) return;
  n_demand++;
  const uint32_t k = key((paddr >> ICACHE_OFFSET_BITS) & (ICACHE_SET_NUM - 1u), paddr >> 12);
  if (pfd.erase(k)) n_useful++;
}
static void tick(uint64_t now, const uint32_t *cand, int n, CsrStatusIO *csr) {
  if (!on) return;
  init();
  for (size_t i = 0; i < fl.size();) {
    if (fl[i].ready <= now) { if (icache_prefetch_install(fl[i].index, fl[i].tag)) { n_install++; pfd.insert(key(fl[i].index, fl[i].tag)); } fl.erase(fl.begin() + i); }
    else i++;
  }
  for (int c = 0; c < n; c++) {
    uint32_t pa;
    if (!icache_peek_translate(cand[c], csr, pa)) { n_noxl++; continue; }
    const uint32_t index = (pa >> ICACHE_OFFSET_BITS) & (ICACHE_SET_NUM - 1u), tag = pa >> 12;
    if (icache_peek_hit(index, tag)) { n_res++; continue; }
    if (!seen.count(key(index, tag))) { n_unseen++; continue; }
    bool dup = false;
    for (const Fl &f : fl) if (f.index == index && f.tag == tag) dup = true;
    if (dup) { n_dup++; continue; }
    if (fl.size() >= maxfl) { n_full++; continue; }
    fl.push_back({index, tag, now + uint64_t(lat)});
    n_issue++;
  }
}
}  // namespace ipf

// RVC=1 front-end statistics (printed at exit)
struct RvcFeStats {
  uint64_t groups = 0, slots_in_range = 0, insts = 0, insts_rvc = 0, carry_used = 0, straddle = 0, straddle_line_end = 0, taken_nonstart = 0;
  ~RvcFeStats() {
    if (!kRvc || groups == 0) return;
    std::printf("[RVC_FE] groups=%llu slots_in_range=%llu insts=%llu (16-bit %llu = %.1f%%) insts/group=%.2f | last-half carry used=%llu | straddling 4-byte insts at the group end=%llu (at a cache-line end=%llu) | predicted taken on a non-start slot=%llu\n",
                (unsigned long long)groups, (unsigned long long)slots_in_range, (unsigned long long)insts, (unsigned long long)insts_rvc,
                insts ? 100.0 * insts_rvc / insts : 0.0, double(insts) / double(groups), (unsigned long long)carry_used, (unsigned long long)straddle,
                (unsigned long long)straddle_line_end, (unsigned long long)taken_nonstart);
  }
};
static RvcFeStats g_rvc_stats;
static const bool g_rvc_pair_guard = [] {
  if (kRvc && bpuf::cfg_pair() != 0) { std::fprintf(stderr, "FE_PAIR is not supported with RVC=1\n"); std::exit(2); }
  return true;
}();
static bool lh_valid = false;             // RVC=1 "last half": the next group begins with the second half of an instruction that started in the last slot of the group before
static pc_t lh_pc = 0;                    // ... and that group starts at this pc
static bool pend_valid = false;           // FE_PAIR: ICache request in flight and its second block
static uint32_t pend_addr = 0;
static FetchPair pend_pair{};
static SimContext *front_ctx = nullptr;

void front_set_context(SimContext *ctx) { front_ctx = ctx; }

static bpuf::Bpu bpu_instance;
bpuf::Bpu &front_bpu_instance() { return bpu_instance; }  // fast-warmup replay (EXP-011)
// BPU_PIPE: pipelined predictor (default off). Its per-cycle decision and transaction are kept in statics (large, only used when on).
static bpuf::BpuPipe bpu_pipe;
static bpuf::BpuPipe::Txn g_pipe_txn;
static bpuf::Decision g_pipe_dec;
static bool g_pipe_txn_valid = false;
static const bool g_use_pipe = bpuf::cfg_pipe() > 0;
void front_warm_commit(pc_t pc, br_type_t type, bool dir, target_addr_t target, bool rvc) {  // fast-warmup replay (EXP-011)
  if (g_use_pipe) bpu_pipe.warm_commit(pc, type, dir, target, rvc); else bpu_instance.warm_commit(pc, type, dir, target, rvc);
}
void front_warm_finish() { if (g_use_pipe) bpu_pipe.warm_finish(); else bpu_instance.warm_finish(); }

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

// ============================================================================
// Small combinational blocks
// ============================================================================

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

// Each FIFO is read at most once and written at most once per cycle; the per-step requests merge here.
static void accumulate_fetch_addr_req(FetchAddrCombOut &acc,
                                      const FetchAddrCombOut &step) {
    acc.clear_fifo |= step.clear_fifo;
    acc.drop_tail_n |= step.drop_tail_n;
    if (step.pop_en) {
        if (acc.pop_en) {
            std::printf("[front_top] ERROR: fetch_address_FIFO pop requested multiple times in one cycle\n");
            std::exit(1);
        }
        acc.pop_en = true;
    }
    if (step.push_en) {
        if (acc.push_en) {
            std::printf("[front_top] ERROR: fetch_address_FIFO push requested multiple times in one cycle\n");
            std::exit(1);
        }
        acc.push_en = true;
        acc.push_data = step.push_data;
        acc.push_pair = step.push_pair;
    }
}

static void accumulate_instruction_fifo_req(InstructionCombOut &acc,
                                            const InstructionCombOut &step) {
    acc.clear_fifo |= step.clear_fifo;
    if (step.pop_en) {
        if (acc.pop_en) {
            std::printf("[front_top] ERROR: instruction_FIFO pop requested multiple times in one cycle\n");
            std::exit(1);
        }
        acc.pop_en = true;
    }
    if (step.push_en) {
        if (acc.push_en) {
            std::printf("[front_top] ERROR: instruction_FIFO push requested multiple times in one cycle\n");
            std::exit(1);
        }
        acc.push_en = true;
        acc.push_entry = step.push_entry;
    }
}

static void accumulate_ptab_req(PtabCombOut &acc, const PtabCombOut &step) {
    acc.clear_ptab |= step.clear_ptab;
    if (step.pop_en) {
        if (acc.pop_en) {
            std::printf("[front_top] ERROR: PTAB pop requested multiple times in one cycle\n");
            std::exit(1);
        }
        acc.pop_en = true;
    }
    if (step.push_write_en) {
        if (acc.push_write_en) {
            std::printf("[front_top] ERROR: PTAB write push requested multiple times in one cycle\n");
            std::exit(1);
        }
        acc.push_write_en = true;
        acc.push_write_entry = step.push_write_entry;
    }
}

static void accumulate_front2back_req(Front2BackCombOut &acc,
                                      const Front2BackCombOut &step) {
    acc.clear_fifo |= step.clear_fifo;
    if (step.pop_en) {
        if (acc.pop_en) {
            std::printf("[front_top] ERROR: front2back_FIFO pop requested multiple times in one cycle\n");
            std::exit(1);
        }
        acc.pop_en = true;
    }
    if (step.push_en) {
        if (acc.push_en) {
            std::printf("[front_top] ERROR: front2back_FIFO push requested multiple times in one cycle\n");
            std::exit(1);
        }
        acc.push_en = true;
        acc.push_entry = step.push_entry;
    }
}

// ============================================================================
// One front-end cycle
// ============================================================================
namespace {

// Working state of one front_comb_calc() call. Each FIFO's read data advances in place as that FIFO is read and
// written; the merged requests go straight into the FrontUpdateRequest.
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
  bool bpu_done = false;      // FE_RFAST bit1: run_bpu() already ran inside read_fifos()
  bool ptab_written = false;  // FE_RFAST bit1: the PTAB write was merged into the head read (write-to-read bypass)
  bpuf::PdInfo pd_next{};
  predecode_checker_out checker_out{};
  bool lh_new_valid = false;      // RVC=1: the "last half" state after the group at the FIFO head (set when a head is processed)
  pc_t lh_new_pc = 0;
  bool lh_updated = false;

  FrontComb(const front_top_in &i, const FrontReadData &r, front_top_out &o, FrontUpdateRequest &q)
      : in(i), rd(r), out(o), req(q), fetch_addr_rd(r.fetch_addr_fifo_rd_snapshot), fifo_rd(r.fifo_rd_snapshot),
        ptab_rd(r.ptab_rd_snapshot), f2b_rd(r.front2back_fifo_rd_snapshot) {}

  void read_fifos();
  void run_bpu();
  void write_fetch_addr();
  void run_icache();
  void write_instruction_fifo();
  void write_ptab_and_check(bool &predecode_flush, pc_t &flush_address);
  void write_front2back();
  void finish(bool predecode_flush, pc_t flush_address);
};

// Flush/refetch, read enables, and the FIFO heads (the fetch-address FIFO feeds the ICache, the other three feed the
// predecode checker and the back end).
void FrontComb::read_fifos() {
  global_reset = in.reset;
  global_refetch = in.refetch || rd.predecode_refetch_snapshot;

  ic_in.reset = global_reset;
  ic_in.refetch = global_refetch;
  ic_in.itlb_flush = in.itlb_flush;
  ic_in.fence_i = in.fence_i;
  ic_in.invalidate_req = false;
  ic_in.csr_status = in.csr_status;
  icache_peek_ready(&ic_in, &ic_out);

  // BPU_PIPE: the pipeline's last stage may override an older block this cycle. The youngest squash_n fetch addresses are
  // dropped, and the FIFO head is not popped when it is one of them. An entry is popped only when it is at least
  // max(0, D-2) cycles old, so a block is never inside the ICache before the decision on its predecessor.
  bool pipe_head_ok = true;
  fetch_addr_in.drop_tail_n = 0;
  if (g_use_pipe) {
    bpu_pipe.decide(global_refetch, g_pipe_dec);
    const auto &fa = rd.fetch_addr_fifo_rd_snapshot;
    const int min_age = bpu_pipe.depth() > 2 ? bpu_pipe.depth() - 2 : 0;
    pipe_head_ok = !(g_pipe_dec.ovr && fa.size <= g_pipe_dec.squash_n) && fa.head_age >= min_age;
    fetch_addr_in.drop_tail_n = g_pipe_dec.ovr ? g_pipe_dec.squash_n : 0;
  }
  // FE_RFAST bit1: the block leaving S_D writes the PTAB and the checker may read it in the same cycle (PTAB bypass)
  bool ptab_bypass = false;
  if (g_use_pipe && (bpuf::cfg_rfast() & 2) && !global_reset && !global_refetch && g_pipe_dec.valid) {
    run_bpu();
    bpu_done = true;
    ptab_bypass = bpu_output.PTAB_write_enable;
    if (ptab_bypass) { front_ptab_write_comb(bpu_output, global_reset, global_refetch, ptab_in); ptab_in.write_enable = true; ptab_written = true; }
  }
  const bool fetch_addr_read_enable = ic_out.icache_read_ready && !rd.fetch_addr_fifo_empty_latch_snapshot &&
                                      !global_reset && !global_refetch && pipe_head_ok;
  fifo_read_enable = !rd.fifo_empty_latch_snapshot && (!rd.ptab_empty_latch_snapshot || ptab_bypass) &&
                     !rd.front2back_fifo_full_latch_snapshot && !global_reset && !global_refetch;
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

  FetchAddrCombOut fetch_addr_step{};
  fetch_address_FIFO_comb_calc(&fetch_addr_in, &fetch_addr_rd, &fetch_addr_head, &fetch_addr_rd, &fetch_addr_step);
  accumulate_fetch_addr_req(req.fetch_addr_fifo_req, fetch_addr_step);

  InstructionCombOut fifo_step{};
  instruction_FIFO_comb_calc(&fifo_in, &fifo_rd, &fifo_head, &fifo_rd, &fifo_step);
  accumulate_instruction_fifo_req(req.fifo_req, fifo_step);

  PtabCombOut ptab_step{};
  PTAB_comb_calc(&ptab_in, &ptab_rd, &ptab_head, &ptab_rd, &ptab_step);
  accumulate_ptab_req(req.ptab_req, ptab_step);

  Front2BackCombOut f2b_step{};
  front2back_FIFO_comb_calc(&f2b_in, &f2b_rd, &f2b_head, &f2b_rd, &f2b_step);
  accumulate_front2back_req(req.front2back_fifo_req, f2b_step);
}

// The BPU predicts the next fetch group unless the fetch-address FIFO or PTAB is full (a flush always runs it).
void FrontComb::run_bpu() {
  if (bpu_done) return;
  const bool bpu_stall = rd.fetch_addr_fifo_full_latch_snapshot || (!g_use_pipe && rd.ptab_full_latch_snapshot);
  bpu_can_run = ((!bpu_stall && (g_use_pipe || !bpu_instance.bubble())) || global_reset || global_refetch);

  bpuf::InputPayload bpu_input{};
  bpu_input.refetch = global_refetch;
  bpu_input.refetch_pd = global_refetch && !in.refetch;
  bpu_input.rf_valid = in.refetch && in.rf_valid;
  bpu_input.rf_blk = in.rf_blk; bpu_input.rf_slot = in.rf_slot; bpu_input.rf_type = in.rf_type; bpu_input.rf_pc = in.rf_pc; bpu_input.rf_is_rvc = in.rf_is_rvc;
  bpu_input.pd = rd.pd_snapshot;
  bpu_input.refetch_address = in.refetch ? in.refetch_address : rd.predecode_refetch_address_snapshot;
  bpu_input.icache_read_ready = bpu_can_run && !rd.fetch_addr_fifo_full_latch_snapshot;
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
    bpu_input.in_loop_used[i] = in.loop_used[i];
  }

  PendingBpuSeqTxn &txn = req.bpu_seq_txn;
  txn.valid = true;
  txn.reset = in.reset;
  txn.inp = bpu_input;
  if (in.reset) {
    bpu_output.fetch_address = RESET_PC;
  } else if (g_use_pipe) {
    bpu_pipe.comb(bpu_input, g_pipe_dec, bpu_output, g_pipe_txn);
    g_pipe_txn_valid = true;
  } else {
    bpu_instance.comb(bpu_input, bpu_output, txn.req);
  }
}

// The predicted fetch address enters the fetch-address FIFO (the ICache reads it next cycle).
void FrontComb::write_fetch_addr() {
  if (!(bpu_output.icache_read_valid && bpu_can_run && !global_reset)) return;
  fetch_addr_in.reset = false;
  fetch_addr_in.refetch = false;
  fetch_addr_in.read_enable = false;
  fetch_addr_in.drop_tail_n = 0;
  fetch_addr_in.write_enable = true;
  fetch_addr_in.fetch_address = bpu_output.fetch_address;
  req.front_state.ipf_ncand = 0;
  req.front_state.ipf_cand[req.front_state.ipf_ncand++] = bpu_output.fetch_address;
  if (bpu_output.pair_valid) req.front_state.ipf_cand[req.front_state.ipf_ncand++] = bpu_output.pair_addr2;
  fetch_addr_in.pair = FetchPair{};
  if (bpu_output.pair_valid) { fetch_addr_in.pair.valid = true; fetch_addr_in.pair.addr2 = bpu_output.pair_addr2; fetch_addr_in.pair.a_len = bpu_output.pair_a_len; fetch_addr_in.pair.b_len = bpu_output.pair_b_len; }
  fetch_address_FIFO_out unused_out{};
  FetchAddrCombOut step{};
  fetch_address_FIFO_comb_calc(&fetch_addr_in, &fetch_addr_rd, &unused_out, &fetch_addr_rd, &step);
  accumulate_fetch_addr_req(req.fetch_addr_fifo_req, step);
}

// The ICache takes the address popped from the fetch-address FIFO.
void FrontComb::run_icache() {
  ic_in.reset = global_reset;
  ic_in.refetch = global_refetch;
  ic_in.itlb_flush = in.itlb_flush;
  ic_in.fence_i = in.fence_i;
  ic_in.invalidate_req = false;
  ic_in.csr_status = in.csr_status;
  ic_in.run_comb_only = false;
  ic_in.icache_read_valid = fetch_addr_head.read_valid;
  ic_in.fetch_address = fetch_addr_head.read_valid ? fetch_addr_head.fetch_address : 0;
  ic_in.icache_read_valid_2 = false;
  ic_in.fetch_address_2 = 0;
  icache_comb_calc(&ic_in, &ic_out);
  g_pmu_front.icache_req_fire = ic_out.perf_req_fire;
  g_pmu_front.icache_miss_issue = ic_out.perf_miss_event;
  g_pmu_front.icache_miss_busy = ic_out.perf_miss_busy;
  g_pmu_front.itlb_access = ic_out.perf_itlb_hit || ic_out.perf_itlb_miss || ic_out.perf_itlb_fault;
  g_pmu_front.itlb_miss = ic_out.perf_itlb_miss;
}

// A completed ICache read is predecoded and written to the instruction FIFO.
void FrontComb::write_instruction_fifo() {
  fifo_in.read_enable = false;
  fifo_in.write_enable = false;
  if (rd.fifo_full_latch_snapshot || !ic_out.icache_read_complete || global_reset || global_refetch) return;

  fifo_in.write_enable = true;
  // FE_PAIR: the second block of the request that completes now (if the entry had one and its line can be read in the same cycle)
  FetchPair pr{};
  if (fetch_addr_head.read_valid && fetch_addr_head.fetch_address == ic_out.fetch_pc) pr = fetch_addr_head.pair;
  else if (rd.pend_valid_snapshot && rd.pend_addr_snapshot == ic_out.fetch_pc) pr = rd.pend_pair_snapshot;
  inst_word_t grp[FETCH_WIDTH]; pc_t gpc[FETCH_WIDTH]; wire1_t gv[FETCH_WIDTH], gpf[FETCH_WIDTH];
  for (int i = 0; i < FETCH_WIDTH; i++) { grp[i] = ic_out.fetch_group[i]; gpc[i] = ic_out.fetch_pc + (i * SLOT_BYTES); gv[i] = ic_out.inst_valid[i]; gpf[i] = ic_out.page_fault_inst[i]; }
  ipf::demand(ic_out.fetch_paddr);
  bool fused = false;
  if (pr.valid && pr.a_len > 0 && pr.a_len < FETCH_WIDTH && !ic_out.page_fault_inst[0] && !ic_out.fetch_single_word && ic_out.inst_valid[pr.a_len - 1]) {
    uint32_t w2[FETCH_WIDTH];
    const bool ok = icache_fetch_second(ic_out.fetch_pc, ic_out.fetch_paddr, pr.addr2, pr.b_len, w2);
    { static uint64_t nok = 0, nbad = 0, npg = 0; static bool rg = false;
      if (!rg) { rg = true; std::atexit([] { std::printf("[FE_PAIR] second-block fetch: ok=%llu failed=%llu (different page: %llu)\n", (unsigned long long)nok, (unsigned long long)nbad, (unsigned long long)npg); }); }
      if (ok) nok++; else { nbad++; if ((ic_out.fetch_pc >> 12) != (pr.addr2 >> 12)) npg++; } }
    for (int i = pr.a_len; i < FETCH_WIDTH; i++) {
      const int j = i - pr.a_len;
      gpc[i] = pr.addr2 + SLOT_BYTES * j;
      gv[i] = ok && j < pr.b_len; gpf[i] = false; grp[i] = ok && j < pr.b_len ? w2[j] : INST_NOP;
    }
    fused = ok;
    if (!ok) for (int i = pr.a_len; i < FETCH_WIDTH; i++) gv[i] = false;   // the group stays the first block only; the PTAB entry (with the second block) then fails the check below
  }
  for (int i = 0; i < FETCH_WIDTH; i++) {
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
    int n = 0;
    while (n < FETCH_WIDTH && gv[n]) n++;
    for (int pass = 0; pass < 2; pass++) {
      wire1_t *st = pass ? fifo_in.start_b : fifo_in.start_a;
      for (int pos = pass; pos < n; pos += fifo_in.is_rvc[pos] ? 1 : 2) st[pos] = true;
    }
  }
  if (pr.valid && pr.a_len > 0 && pr.a_len < FETCH_WIDTH && ic_out.inst_valid[pr.a_len - 1] && !ic_out.fetch_single_word) {   // FE_PAIR: next pc after the last valid slot
    int last = -1;
    for (int i = 0; i < FETCH_WIDTH; i++) if (fifo_in.inst_valid[i]) last = i;
    fifo_in.seq_next_pc = fifo_in.pc[last] + 4u;
    (void)fused;
  } else
  fifo_in.seq_next_pc = sequential_next_pc(ic_out.fetch_pc, ic_out.inst_valid);

  instruction_FIFO_out unused_out{};
  InstructionCombOut step{};
  instruction_FIFO_comb_calc(&fifo_in, &fifo_rd, &unused_out, &fifo_rd, &step);
  accumulate_instruction_fifo_req(req.fifo_req, step);
}

// The BPU's prediction enters the PTAB; the predecode checker compares the FIFO head against the PTAB head and may ask
// for a refetch.
void FrontComb::write_ptab_and_check(bool &predecode_flush, pc_t &flush_address) {
  const bool ptab_can_write =
      bpu_output.PTAB_write_enable && (g_use_pipe || !rd.ptab_full_latch_snapshot) && !global_reset && !global_refetch;
  if (ptab_can_write && !ptab_written) {
    front_ptab_write_comb(bpu_output, global_reset, global_refetch, ptab_in);
    PTAB_out unused_out{};
    PtabCombOut step{};
    PTAB_comb_calc(&ptab_in, &ptab_rd, &unused_out, &ptab_rd, &step);
    accumulate_ptab_req(req.ptab_req, step);
  }

  if (!fifo_read_enable) return;
  static const bool pair_on = bpuf::cfg_pair() != 0;
  bool multi = false;
  if (pair_on) {
    // FE_PAIR: only the slots whose pc agrees with the PTAB entry are usable (the BPU may have replaced the second block / the first block's end)
    bool cut = false;
    for (int i = 0; i < FETCH_WIDTH; i++) {
      if (!cut && fifo_head.inst_valid[i] && fifo_head.pc[i] != ptab_head.predict_base_pc[i]) cut = true;
      if (cut) { fifo_head.inst_valid[i] = false; fifo_head.predecode_type[i] = PREDECODE_NON_BRANCH; fifo_head.predecode_target_address[i] = 0; }
    }
    for (int i = 1; i < FETCH_WIDTH; i++) if (fifo_head.inst_valid[i] && fifo_head.pc[i] != fifo_head.pc[i - 1] + 4) multi = true;
    if (cut) {  // the group lost slots: its sequential continuation is the pc after the last usable slot
      int last = -1;
      for (int i = 0; i < FETCH_WIDTH; i++) if (fifo_head.inst_valid[i]) last = i;
      if (last >= 0) fifo_head.seq_next_pc = fifo_head.pc[last] + 4u;
    }
  } else
  for (int i = 0; i < FETCH_WIDTH; i++) {
    if (fifo_head.pc[i] != ptab_head.predict_base_pc[i]) {
      printf("ERROR: fifo pc[%d]: %llx != ptab pc[%d]: %llx\n", i, (unsigned long long)fifo_head.pc[i], i, (unsigned long long)ptab_head.predict_base_pc[i]);
      exit(1);
    }
  }
  int rvc_n = 0, rvc_last = -1;   // RVC=1: slots in range, and the last slot that starts an instruction
  bool rvc_straddle = false;
  if constexpr (kRvc) {
    // Which in-range slots start an instruction: the chain from slot 0, or from slot 1 when slot 0 completes the previous group's last instruction.
    const bool use_b = rd.lh_valid_snapshot && rd.lh_pc_snapshot == fifo_head.pc[0];
    const wire1_t *m = use_b ? fifo_head.start_b : fifo_head.start_a;
    while (rvc_n < FETCH_WIDTH && fifo_head.inst_valid[rvc_n]) rvc_n++;
    for (int i = 0; i < FETCH_WIDTH; i++) {
      if (!fifo_head.inst_valid[i]) continue;
      if (m[i]) { rvc_last = i; continue; }
      if (ptab_head.predict_dir[i]) g_rvc_stats.taken_nonstart++;
      fifo_head.inst_valid[i] = false; fifo_head.predecode_type[i] = PREDECODE_NON_BRANCH; fifo_head.predecode_target_address[i] = 0;
    }
    g_rvc_stats.groups++; g_rvc_stats.slots_in_range += rvc_n; g_rvc_stats.carry_used += use_b;
    for (int i = 0; i < FETCH_WIDTH; i++) if (fifo_head.inst_valid[i]) { g_rvc_stats.insts++; g_rvc_stats.insts_rvc += fifo_head.is_rvc[i]; }
    rvc_straddle = rvc_last >= 0 && rvc_last == rvc_n - 1 && !fifo_head.is_rvc[rvc_last] && !fifo_head.page_fault_inst[rvc_last];
    if (rvc_straddle) { g_rvc_stats.straddle++; if (((fifo_head.pc[0] + rvc_n * SLOT_BYTES) & (ICACHE_LINE_SIZE - 1u)) == 0) g_rvc_stats.straddle_line_end++; }
  }
  predecode_checker_in checker_in;
  front_checker_input_comb(fifo_head, ptab_head, checker_in);
  checker_in.multi = multi;
  for (int i = 0; i < FETCH_WIDTH; i++) { checker_in.pc[i] = fifo_head.pc[i]; checker_in.inst_valid[i] = fifo_head.inst_valid[i]; }
  predecode_checker_read_data checker_rd{};
  predecode_checker_seq_read(&checker_in, &checker_rd);
  predecode_checker_comb(checker_rd, checker_out);
  if (checker_out.predecode_flush_enable) {
    predecode_flush = true;
    flush_address = checker_out.predict_next_fetch_address_corrected;
  }
  if (kRvc) { static const bool dbg = std::getenv("RVC_DBG") != nullptr; static int budget = 40;
    if (dbg && checker_out.predecode_flush_enable && budget-- > 0) {
      std::printf("[RVC_DBG] flush pc0=%x predicted_next=%x corrected=%x seq_next=%x n=%d valid=", (unsigned)fifo_head.pc[0], (unsigned)ptab_head.predict_next_fetch_address, (unsigned)checker_out.predict_next_fetch_address_corrected, (unsigned)fifo_head.seq_next_pc, rvc_n);
      for (int i = 0; i < FETCH_WIDTH; i++) std::printf("%d", (int)fifo_head.inst_valid[i]);
      std::printf(" dir=");
      for (int i = 0; i < FETCH_WIDTH; i++) std::printf("%d", (int)ptab_head.predict_dir[i]);
      std::printf(" type=");
      for (int i = 0; i < FETCH_WIDTH; i++) std::printf("%d", (int)fifo_head.predecode_type[i]);
      std::printf("\n");
    } }
  if constexpr (kRvc) {   // the next group starts with the second half of this group's last instruction if it straddles and the fetch continues sequentially
    bool any_taken = false;
    for (int i = 0; i < FETCH_WIDTH; i++) if (fifo_head.inst_valid[i] && checker_out.predict_dir_corrected[i]) any_taken = true;
    lh_updated = true;
    lh_new_pc = fifo_head.pc[0] + rvc_n * SLOT_BYTES;
    // a predecode flush that continues sequentially (corrected target = the end of this group) keeps the carry: the refetched group still begins with the
    // second half of the straddling instruction
    lh_new_valid = rvc_straddle && !any_taken && (!checker_out.predecode_flush_enable || checker_out.predict_next_fetch_address_corrected == lh_new_pc);
  }
  if (g_use_pipe) {  // BPO_PDCKPT: report the check, and the history / RAS effect of the corrected block, to the BPU next cycle
    bpuf::PdInfo &pd = pd_next;
    pd = bpuf::PdInfo{};
    pd.valid = true;
    pd.flush = predecode_flush;
    if (predecode_flush) {
      const bool hist = bpuf::cfg_hist();
      for (int i = 0; i < FETCH_WIDTH; i++) {
        if (!fifo_head.inst_valid[i]) continue;
        const predecode_type_t ty = fifo_head.predecode_type[i];
        if (ty == PREDECODE_NON_BRANCH) continue;
        const uint32_t pc = fifo_head.pc[i], inst = fifo_head.instructions[i];
        const bool taken = checker_out.predict_dir_corrected[i];
        const uint32_t tgt = ty == PREDECODE_JALR ? ptab_head.predict_next_fetch_address : fifo_head.predecode_target_address[i];
        if (multi && i >= checker_out.keep_n) break;
        if (ty == PREDECODE_DIRECT_JUMP_NO_JAL && !hist) { pd.bits |= uint8_t(taken) << pd.nbits; pd.nbits++; }
        if (!taken) continue;
        if (hist) { pd.bits |= uint8_t(bpuf::hist_sym(pc, tgt)) << pd.nbits; pd.nbits += bpuf::cfg_hsym(); }
        const uint32_t rd_ = (inst >> 7) & 31, rs1 = (inst >> 15) & 31;
        if (pd.ras_op == 0) {
          if (ty == PREDECODE_JAL && rd_ == 1) { pd.ras_op = 1; pd.ras_val = pc + ilen_of(fifo_head.is_rvc[i]); }
          else if (ty == PREDECODE_JALR && rd_ == 0 && rs1 == 1 && (inst >> 20) == 0) pd.ras_op = 2;
        }
        if (!multi || i + 1 >= int(checker_out.keep_n)) break;
      }
    }
  }
}

// The checked entry goes to the front2back FIFO, or straight to the output when the FIFO is empty and the back end is
// reading this cycle.
void FrontComb::write_front2back() {
  const bool can_write = fifo_read_enable && !rd.front2back_fifo_full_latch_snapshot && !global_reset;
  use_f2b_bypass = f2b_read_enable && rd.front2back_fifo_empty_latch_snapshot &&
                   !f2b_head.front2back_FIFO_valid && can_write;
  f2b_in.read_enable = false;
  f2b_in.write_enable = can_write && !use_f2b_bypass;
  if (!can_write) return;

  front_front2back_write_comb(fifo_head, ptab_head, checker_out, use_f2b_bypass, f2b_in, f2b_bypass);
  if (f2b_in.write_enable) {
    front2back_FIFO_out unused_out{};
    Front2BackCombOut step{};
    front2back_FIFO_comb_calc(&f2b_in, &f2b_rd, &unused_out, &f2b_rd, &step);
    accumulate_front2back_req(req.front2back_fifo_req, step);
  }
}

// Latches for the next cycle, the predecode-flush request, and the output toward the back end.
void FrontComb::finish(bool predecode_flush, pc_t flush_address) {
  { static const bool tr = std::getenv("FE_TRACE") != nullptr; static int budget = 24;
    if (tr && budget > 0 && front_ctx->perf.cycle > 40000 && (global_refetch || budget < 24)) { budget--;
      std::printf("[FE_TRACE] cyc=%llu refetch=%d bpu_issue=%d(pc=%x) ic_req=%d ic_done=%d fifo_w=%d f2b_w=%d out_valid=%d icrdy=%d faE=%d faSz=%d\n",
        (unsigned long long)front_ctx->perf.cycle, (int)global_refetch, (int)(bpu_output.icache_read_valid && bpu_can_run), bpu_output.fetch_address,
        (int)fetch_addr_head.read_valid, (int)ic_out.icache_read_complete, (int)fifo_in.write_enable, (int)f2b_in.write_enable || use_f2b_bypass, (int)(f2b_head.front2back_FIFO_valid || use_f2b_bypass), (int)ic_out.icache_read_ready, (int)rd.fetch_addr_fifo_empty_latch_snapshot, (int)rd.fetch_addr_fifo_rd_snapshot.size); } }
  PendingFrontState &state = req.front_state;
  state.valid = true;
  if (predecode_flush) {
    icache_in invalidate_in{};
    invalidate_in.invalidate_req = true;
    invalidate_in.csr_status = in.csr_status;
    icache_comb_calc(&invalidate_in, &ic_out);
  }
  {   // FE_PAIR: remember the second block of the ICache request that is still in flight
    bool nv = rd.pend_valid_snapshot; fetch_addr_t na = rd.pend_addr_snapshot; FetchPair np = rd.pend_pair_snapshot;
    const bool head_done = ic_out.icache_read_complete && fetch_addr_head.read_valid && fetch_addr_head.fetch_address == ic_out.fetch_pc;
    if (ic_out.icache_read_complete && nv && rd.pend_addr_snapshot == ic_out.fetch_pc) nv = false;
    if (fetch_addr_head.read_valid && !head_done) { nv = true; na = fetch_addr_head.fetch_address; np = fetch_addr_head.pair; }
    if (global_refetch || global_reset) nv = false;
    state.next_pend_valid = nv; state.next_pend_addr = na; state.next_pend_pair = np;
  }
  {   // RVC=1 last-half state
    bool nv = lh_updated ? lh_new_valid : rd.lh_valid_snapshot;
    // a back-end redirect (or reset) drops the carry; a predecode flush drops it unless the flushed group itself set it (sequential continuation),
    // and the refetch cycle that follows a predecode flush keeps it
    if (global_reset || in.refetch) nv = false;
    else if (predecode_flush && !(lh_updated && lh_new_valid)) nv = false;
    state.next_lh_valid = nv; state.next_lh_pc = lh_updated ? lh_new_pc : rd.lh_pc_snapshot;
  }
  state.next_predecode_refetch = predecode_flush;
  state.next_pd = pd_next;
  state.next_predecode_refetch_address = predecode_flush ? flush_address : 0;
  state.next_fetch_addr_fifo_full = (fetch_addr_rd.size >= (FETCH_ADDR_FIFO_SIZE - 1));
  state.next_fetch_addr_fifo_empty = (fetch_addr_rd.size == 0);
  state.next_fifo_full = (fifo_rd.size == INSTRUCTION_FIFO_SIZE);
  state.next_fifo_empty = (fifo_rd.size == 0);
  state.next_ptab_full = (ptab_rd.size >= (PTAB_SIZE - 1));
  state.next_ptab_empty = (ptab_rd.size == 0);
  state.next_front2back_fifo_full = (f2b_rd.size == FRONT2BACK_FIFO_SIZE);
  state.next_front2back_fifo_empty = (f2b_rd.size == 0);
  state.next_front_sim_time = rd.front_sim_time_snapshot + 1;

  front_output_comb(f2b_head, f2b_bypass, use_f2b_bypass, out);
  out.commit_stall = bpu_output.update_queue_full;
  out.commit_upd_free = bpu_output.upd_free;
  out.commit_tq_free = bpu_output.tq_free;
  // PMU frontend delivery split (EXP-025): how many instructions of this cycle's fetch group the back end gets, and why the other slots are empty
  {
    static uint64_t last_refetch = ~0ull;
    const uint64_t cyc = front_ctx ? front_ctx->perf.cycle : 0;
    if (global_refetch) last_refetch = cyc;
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
      else if (ic_out.perf_miss_busy) pf.feo_cause = PMU_FEO_LOSS_ICMISS;
      else if (predecode_flush) pf.feo_cause = PMU_FEO_LOSS_PDFLUSH;
      else if (rd.fetch_addr_fifo_empty_latch_snapshot && !bpu_output.icache_read_valid) {
        if (g_use_pipe && g_pipe_dec.ovr) pf.feo_cause = PMU_FEO_LOSS_OVR_SD;
        else if (g_use_pipe && g_pipe_txn.s_l1_ovr) pf.feo_cause = PMU_FEO_LOSS_OVR_L1;
        else if (rd.fetch_addr_fifo_full_latch_snapshot || (g_use_pipe && g_pipe_txn.s_stall_ptab)) pf.feo_cause = PMU_FEO_LOSS_PTABSTALL;
        else pf.feo_cause = PMU_FEO_LOSS_BPUOTHER;
      }
      else if (!ic_out.icache_read_ready) pf.feo_cause = PMU_FEO_LOSS_ICNOTRDY;
      else if (rd.fifo_empty_latch_snapshot) pf.feo_cause = PMU_FEO_LOSS_IFIFO;
      else if (rd.ptab_empty_latch_snapshot) pf.feo_cause = PMU_FEO_LOSS_PTABLAG;
      else pf.feo_cause = PMU_FEO_LOSS_F2B;
    }
  }
}

} // namespace

// Precondition: req is zero-initialised (the merged requests accumulate into it).
void front_comb_calc(const struct front_top_in &inp, const FrontReadData &rd, struct front_top_out &out,
                     FrontUpdateRequest &req) {
  FrontComb c(inp, rd, out, req);
  c.read_fifos();
  c.run_bpu();
  c.write_fetch_addr();
  c.run_icache();
  c.write_instruction_fifo();
  bool predecode_flush = false;
  pc_t flush_address = 0;
  c.write_ptab_and_check(predecode_flush, flush_address);
  c.write_front2back();
  c.finish(predecode_flush, flush_address);
}

namespace {

void front_seq_read(const struct front_top_in &inp, FrontReadData &rd) {
  rd.predecode_refetch_snapshot = predecode_refetch;
  rd.predecode_refetch_address_snapshot = predecode_refetch_address;
  rd.pd_snapshot = pd_latch;
  rd.front_sim_time_snapshot = front_sim_time;
  rd.fetch_addr_fifo_full_latch_snapshot = fetch_addr_fifo_full_latch;
  rd.fetch_addr_fifo_empty_latch_snapshot = fetch_addr_fifo_empty_latch;
  rd.fifo_full_latch_snapshot = fifo_full_latch;
  rd.fifo_empty_latch_snapshot = fifo_empty_latch;
  rd.ptab_full_latch_snapshot = ptab_full_latch;
  rd.ptab_empty_latch_snapshot = ptab_empty_latch;
  rd.front2back_fifo_full_latch_snapshot = front2back_fifo_full_latch;
  rd.front2back_fifo_empty_latch_snapshot = front2back_fifo_empty_latch;
  rd.lh_valid_snapshot = lh_valid; rd.lh_pc_snapshot = lh_pc;
  rd.pend_valid_snapshot = pend_valid; rd.pend_addr_snapshot = pend_addr; rd.pend_pair_snapshot = pend_pair;

  fetch_address_FIFO_in fetch_addr_in;
  instruction_FIFO_in fifo_in;
  PTAB_in ptab_in;
  front2back_FIFO_in front2back_in;
  icache_in icache_inp;
  icache_out icache_outp;
  std::memset(&fetch_addr_in, 0, sizeof(fetch_addr_in));
  std::memset(&fifo_in, 0, sizeof(fifo_in));
  std::memset(&ptab_in, 0, sizeof(ptab_in));
  std::memset(&front2back_in, 0, sizeof(front2back_in));
  std::memset(&icache_inp, 0, sizeof(icache_inp));
  icache_inp.csr_status = inp.csr_status;
  std::memset(&icache_outp, 0, sizeof(icache_outp));

  fetch_address_FIFO_seq_read(&fetch_addr_in, &rd.fetch_addr_fifo_rd_snapshot);
  instruction_FIFO_seq_read(&fifo_in, &rd.fifo_rd_snapshot);
  PTAB_seq_read(&ptab_in, &rd.ptab_rd_snapshot);
  front2back_FIFO_seq_read(&front2back_in, &rd.front2back_fifo_rd_snapshot);
  icache_seq_read(&icache_inp, &icache_outp);
  (void)inp;
}

void front_seq_write(const struct front_top_in &inp, const FrontUpdateRequest &req,
                     bool reset) {
  (void)inp;
  (void)reset;
  if (req.bpu_seq_txn.valid) {
    if (req.bpu_seq_txn.reset) { bpu_instance.reset(); if (g_use_pipe) bpu_pipe.reset(); }
    else if (g_use_pipe) { if (g_pipe_txn_valid) bpu_pipe.commit(g_pipe_txn); g_pipe_txn_valid = false; }
    else bpu_instance.commit(req.bpu_seq_txn.inp, req.bpu_seq_txn.req);
  }
  fetch_address_FIFO_seq_write(&req.fetch_addr_fifo_req);
  instruction_FIFO_seq_write(&req.fifo_req);
  PTAB_seq_write(&req.ptab_req);
  front2back_FIFO_seq_write(&req.front2back_fifo_req);
  predecode_checker_seq_write();
  icache_seq_write();

  ipf::tick(front_ctx ? front_ctx->perf.cycle : 0, req.front_state.ipf_cand, req.front_state.valid ? req.front_state.ipf_ncand : 0, inp.csr_status);
  if (req.front_state.valid) {
    front_sim_time = req.front_state.next_front_sim_time;
    predecode_refetch = req.front_state.next_predecode_refetch;
    predecode_refetch_address = req.front_state.next_predecode_refetch_address;
    pd_latch = req.front_state.next_pd;
    fetch_addr_fifo_full_latch = req.front_state.next_fetch_addr_fifo_full;
    fetch_addr_fifo_empty_latch = req.front_state.next_fetch_addr_fifo_empty;
    fifo_full_latch = req.front_state.next_fifo_full;
    fifo_empty_latch = req.front_state.next_fifo_empty;
    ptab_full_latch = req.front_state.next_ptab_full;
    ptab_empty_latch = req.front_state.next_ptab_empty;
    front2back_fifo_full_latch = req.front_state.next_front2back_fifo_full;
    front2back_fifo_empty_latch = req.front_state.next_front2back_fifo_empty;
    lh_valid = req.front_state.next_lh_valid; lh_pc = req.front_state.next_lh_pc;
    pend_valid = req.front_state.next_pend_valid; pend_addr = req.front_state.next_pend_addr; pend_pair = req.front_state.next_pend_pair;
  }
}

} // namespace

void front_top_seq_read(const struct front_top_in &inp, FrontReadData &rd) {
  front_seq_read(inp, rd);
}

void front_top_seq_write(const struct front_top_in &inp,
                         const FrontUpdateRequest &req, bool reset) {
  front_seq_write(inp, req, reset);
}

void front_top(struct front_top_in *in, struct front_top_out *out) {
  assert(in);
  assert(out);

  FrontReadData rd;
  FrontUpdateRequest req;
  front_seq_read(*in, rd);
  front_comb_calc(*in, rd, *out, req);
  front_seq_write(*in, req, in->reset);
}
