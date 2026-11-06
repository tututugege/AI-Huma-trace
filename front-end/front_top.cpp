#include "BPU/BPU.h"
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

// ============================================================================
// Latched front-end state (registers)
// ============================================================================
static bool predecode_refetch = false;
static uint32_t predecode_refetch_address = 0;
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
static SimContext *front_ctx = nullptr;

void front_set_context(SimContext *ctx) { front_ctx = ctx; }

static bpuf::Bpu bpu_instance;
bpuf::Bpu &front_bpu_instance() { return bpu_instance; }  // fast-warmup replay (EXP-011)

static uint32_t sequential_next_pc(uint32_t fetch_pc, const wire1_t *inst_valid) {
  uint32_t valid_prefix = 0;
  while (valid_prefix < FETCH_WIDTH && inst_valid[valid_prefix]) {
    ++valid_prefix;
  }
  uint32_t next_pc = fetch_pc + valid_prefix * 4u;
  const uint32_t line_mask = ~(ICACHE_LINE_SIZE - 1u);
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
    ptab_in.predict_base_pc[i] = bpu_output.out_pred_base_pc + (i * 4);
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
    fifo_in.inst_valid[i] = fifo_out.inst_valid[i];
    fifo_in.predict_dir_corrected[i] = checker_out.predict_dir_corrected[i];
    fifo_in.predict_base_pc[i] = ptab_out.predict_base_pc[i];
    if (bypass) {
      bypass_out.fetch_group[i] = fifo_out.instructions[i];
      bypass_out.page_fault_inst[i] = fifo_out.page_fault_inst[i];
      bypass_out.inst_valid[i] = fifo_out.inst_valid[i];
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
  predecode_checker_out checker_out{};

  FrontComb(const front_top_in &i, const FrontReadData &r, front_top_out &o, FrontUpdateRequest &q)
      : in(i), rd(r), out(o), req(q), fetch_addr_rd(r.fetch_addr_fifo_rd_snapshot), fifo_rd(r.fifo_rd_snapshot),
        ptab_rd(r.ptab_rd_snapshot), f2b_rd(r.front2back_fifo_rd_snapshot) {}

  void read_fifos();
  void run_bpu();
  void write_fetch_addr();
  void run_icache();
  void write_instruction_fifo();
  void write_ptab_and_check(bool &predecode_flush, uint32_t &flush_address);
  void write_front2back();
  void finish(bool predecode_flush, uint32_t flush_address);
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

  const bool fetch_addr_read_enable = ic_out.icache_read_ready && !rd.fetch_addr_fifo_empty_latch_snapshot &&
                                      !global_reset && !global_refetch;
  fifo_read_enable = !rd.fifo_empty_latch_snapshot && !rd.ptab_empty_latch_snapshot &&
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
  const bool bpu_stall = rd.fetch_addr_fifo_full_latch_snapshot || rd.ptab_full_latch_snapshot;
  bpu_can_run = !bpu_stall || global_reset || global_refetch;

  bpuf::InputPayload bpu_input{};
  bpu_input.refetch = global_refetch;
  bpu_input.refetch_address = in.refetch ? in.refetch_address : rd.predecode_refetch_address_snapshot;
  bpu_input.icache_read_ready = bpu_can_run && !rd.fetch_addr_fifo_full_latch_snapshot;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    bpu_input.in_update_base_pc[i] = in.predict_base_pc[i];
    bpu_input.in_upd_valid[i] = in.back2front_valid[i];
    bpu_input.in_actual_dir[i] = in.actual_dir[i];
    bpu_input.in_actual_br_type[i] = in.actual_br_type[i];
    bpu_input.in_actual_targets[i] = in.actual_target[i];
    bpu_input.in_pred_dir[i] = in.predict_dir[i];
    bpu_input.in_alt_pred[i] = in.alt_pred[i];
    bpu_input.in_pcpn[i] = in.pcpn[i];
    bpu_input.in_altpcpn[i] = in.altpcpn[i];
    for (int j = 0; j < 4; j++) {
      bpu_input.in_tage_tags[i][j] = in.tage_tag[i][j];
      bpu_input.in_tage_idxs[i][j] = in.tage_idx[i][j];
    }
    bpu_input.in_sc_used[i] = in.sc_used[i];
    bpu_input.in_loop_used[i] = in.loop_used[i];
  }

  PendingBpuSeqTxn &txn = req.bpu_seq_txn;
  txn.valid = true;
  txn.reset = in.reset;
  txn.inp = bpu_input;
  if (in.reset) {
    bpu_output.fetch_address = RESET_PC;
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
  fetch_addr_in.write_enable = true;
  fetch_addr_in.fetch_address = bpu_output.fetch_address;
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
}

// A completed ICache read is predecoded and written to the instruction FIFO.
void FrontComb::write_instruction_fifo() {
  fifo_in.read_enable = false;
  fifo_in.write_enable = false;
  if (rd.fifo_full_latch_snapshot || !ic_out.icache_read_complete || global_reset || global_refetch) return;

  fifo_in.write_enable = true;
  for (int i = 0; i < FETCH_WIDTH; i++) {
    fifo_in.fetch_group[i] = ic_out.fetch_group[i];
    fifo_in.pc[i] = ic_out.fetch_pc + (i * 4);
    fifo_in.page_fault_inst[i] = ic_out.page_fault_inst[i];
    fifo_in.inst_valid[i] = ic_out.inst_valid[i];
    if (ic_out.inst_valid[i]) {
      predecode_in predecode_inp{ic_out.fetch_group[i], ic_out.fetch_pc + (i * 4)};
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
  fifo_in.seq_next_pc = sequential_next_pc(ic_out.fetch_pc, ic_out.inst_valid);

  instruction_FIFO_out unused_out{};
  InstructionCombOut step{};
  instruction_FIFO_comb_calc(&fifo_in, &fifo_rd, &unused_out, &fifo_rd, &step);
  accumulate_instruction_fifo_req(req.fifo_req, step);
}

// The BPU's prediction enters the PTAB; the predecode checker compares the FIFO head against the PTAB head and may ask
// for a refetch.
void FrontComb::write_ptab_and_check(bool &predecode_flush, uint32_t &flush_address) {
  const bool ptab_can_write =
      bpu_output.PTAB_write_enable && !rd.ptab_full_latch_snapshot && !global_reset && !global_refetch;
  if (ptab_can_write) {
    front_ptab_write_comb(bpu_output, global_reset, global_refetch, ptab_in);
    PTAB_out unused_out{};
    PtabCombOut step{};
    PTAB_comb_calc(&ptab_in, &ptab_rd, &unused_out, &ptab_rd, &step);
    accumulate_ptab_req(req.ptab_req, step);
  }

  if (!fifo_read_enable) return;
  for (int i = 0; i < FETCH_WIDTH; i++) {
    if (fifo_head.pc[i] != ptab_head.predict_base_pc[i]) {
      printf("ERROR: fifo pc[%d]: %x != ptab pc[%d]: %x\n", i, fifo_head.pc[i], i, ptab_head.predict_base_pc[i]);
      exit(1);
    }
  }
  predecode_checker_in checker_in;
  front_checker_input_comb(fifo_head, ptab_head, checker_in);
  predecode_checker_read_data checker_rd{};
  predecode_checker_seq_read(&checker_in, &checker_rd);
  predecode_checker_comb(checker_rd, checker_out);
  if (checker_out.predecode_flush_enable) {
    predecode_flush = true;
    flush_address = checker_out.predict_next_fetch_address_corrected;
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
void FrontComb::finish(bool predecode_flush, uint32_t flush_address) {
  PendingFrontState &state = req.front_state;
  state.valid = true;
  if (predecode_flush) {
    icache_in invalidate_in{};
    invalidate_in.invalidate_req = true;
    invalidate_in.csr_status = in.csr_status;
    icache_comb_calc(&invalidate_in, &ic_out);
  }
  state.next_predecode_refetch = predecode_flush;
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
  uint32_t flush_address = 0;
  c.write_ptab_and_check(predecode_flush, flush_address);
  c.write_front2back();
  c.finish(predecode_flush, flush_address);
}

namespace {

void front_seq_read(const struct front_top_in &inp, FrontReadData &rd) {
  rd.predecode_refetch_snapshot = predecode_refetch;
  rd.predecode_refetch_address_snapshot = predecode_refetch_address;
  rd.front_sim_time_snapshot = front_sim_time;
  rd.fetch_addr_fifo_full_latch_snapshot = fetch_addr_fifo_full_latch;
  rd.fetch_addr_fifo_empty_latch_snapshot = fetch_addr_fifo_empty_latch;
  rd.fifo_full_latch_snapshot = fifo_full_latch;
  rd.fifo_empty_latch_snapshot = fifo_empty_latch;
  rd.ptab_full_latch_snapshot = ptab_full_latch;
  rd.ptab_empty_latch_snapshot = ptab_empty_latch;
  rd.front2back_fifo_full_latch_snapshot = front2back_fifo_full_latch;
  rd.front2back_fifo_empty_latch_snapshot = front2back_fifo_empty_latch;

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
    if (req.bpu_seq_txn.reset) bpu_instance.reset();
    else bpu_instance.commit(req.bpu_seq_txn.inp, req.bpu_seq_txn.req);
  }
  fetch_address_FIFO_seq_write(&req.fetch_addr_fifo_req);
  instruction_FIFO_seq_write(&req.fifo_req);
  PTAB_seq_write(&req.ptab_req);
  front2back_FIFO_seq_write(&req.front2back_fifo_req);
  predecode_checker_seq_write();
  icache_seq_write();

  if (req.front_state.valid) {
    front_sim_time = req.front_state.next_front_sim_time;
    predecode_refetch = req.front_state.next_predecode_refetch;
    predecode_refetch_address = req.front_state.next_predecode_refetch_address;
    fetch_addr_fifo_full_latch = req.front_state.next_fetch_addr_fifo_full;
    fetch_addr_fifo_empty_latch = req.front_state.next_fetch_addr_fifo_empty;
    fifo_full_latch = req.front_state.next_fifo_full;
    fifo_empty_latch = req.front_state.next_fifo_empty;
    ptab_full_latch = req.front_state.next_ptab_full;
    ptab_empty_latch = req.front_state.next_ptab_empty;
    front2back_fifo_full_latch = req.front_state.next_front2back_fifo_full;
    front2back_fifo_empty_latch = req.front_state.next_front2back_fifo_empty;
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
