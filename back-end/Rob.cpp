#include "Rob.h"
#include "Csr.h"
#include "PreIduQueue.h"
#include "IO.h"
#include "RealLsu.h"
#include "SimCpu.h"
#include "BackPorts.h"
#include "config.h"
#include "trace_hook.h"
#include "util.h"
#include "Pmu.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>


namespace {
inline bool rob_is_store(const Uop &u) { return u->tma.mem_commit_is_store; }
inline bool rob_is_load(const Uop &u) { return u->tma.mem_commit_is_load; }
inline bool rob_is_page_fault(const Uop &u) { return u.page_fault_inst || u.page_fault_load || u.page_fault_store; }
// RISC-V semihosting marker: slli x0,x0,0x1f ; ebreak ; srai x0,x0,7. OpenSBI's generic platform probes it and expects a breakpoint
// trap into M mode; every other ebreak still ends the simulation (the bare-metal tests' exit). Same rule as the function model.
inline bool is_semihost_marker(xlen_t pc) {
  return tbhook::mem_read(pc - 4) == 0x01f01013u && tbhook::mem_read(pc + 4) == 0x40705013u;
}
inline bool rob_is_complete(const Uop &u) { return u.cplt_mask == u->expect_mask; }
inline InstType rob_type(const Uop &u) { return decode_inst_type(u->type); }
inline bool rob_is_exception(const Uop &u) { return rob_is_page_fault(u) || u.illegal_inst || rob_type(u) == ECALL; }
inline bool rob_is_wfi(const Uop &u) { return rob_type(u) == WFI; }
inline bool rob_is_fence(const Uop &u) {
  const InstType t = rob_type(u);
  return t == FENCE || t == FENCE_I || t == SFENCE_VMA;
}
// Instructions that must retire alone and redirect the front-end.
inline bool rob_is_flush_inst(const Uop &u) {
  const InstType t = rob_type(u);
  return t == CSR || t == ECALL || t == MRET || t == SRET || t == SFENCE_VMA || t == FENCE_I || t == EBREAK ||
         rob_is_exception(u) || u.flush_pipe;
}
// ROBTR=<lo>:<hi> prints the ROB's dispatches and flushes of the cycles in the range (debug)
static long robtr_lo() { static long v = [] { long lo = -1, hi = -1; if (const char *e = std::getenv("ROBTR")) std::sscanf(e, "%ld:%ld", &lo, &hi); return lo; }(); return v; }
static long robtr_hi() { static long v = [] { long lo = -1, hi = -1; if (const char *e = std::getenv("ROBTR")) std::sscanf(e, "%ld:%ld", &lo, &hi); return hi; }(); return v; }
#define ROBTR_ON() (robtr_lo() >= 0 && (long)ctx->perf.cycle >= robtr_lo() && (long)ctx->perf.cycle <= robtr_hi())
// A completed load's address must be naturally aligned (only checked when the load did not fault).
inline void check_load_alignment(const Uop &u, const char *msg) {
  if (!rob_is_load(u) || !rob_is_complete(u) || rob_is_page_fault(u)) return;
  const uint32_t mask = (u->func3 & 3u) == 0 ? 0u : (u->func3 & 3u) == 1 ? 1u : 3u;
  Assert((u.diag_val & mask) == 0 && msg);
}
// Commit statistics and the checkpoint warm-up / measurement window.
void count_commit(SimContext *ctx, const Uop &u) {
  ctx->perf.commit_num++;
  if (is_load(u)) ctx->perf.commit_load_num++;
  if (is_store(u)) ctx->perf.commit_store_num++;
  if (!ctx->is_ckpt) return;
  if (ctx->front_door_pending) {
    // The restore program ends with mret: the measurement counts from the checkpoint's first instruction. The commit
    // counters restart; the cycle count is kept (the memory system uses it as a clock) and its baseline moves.
    if (decode_inst_type(u->type) == MRET) {
      ctx->front_door_pending = false;
      std::cout << "[front-door] restore program committed: " << std::dec << ctx->perf.commit_num << " instructions, "
                << ctx->perf.cycle << " cycles; measuring from here" << std::endl;
      ctx->perf.commit_num = ctx->perf.commit_load_num = ctx->perf.commit_store_num = 0;
      ctx->perf.pmu.reset();
      ctx->perf.cycle_base = ctx->perf.cycle;
      if (ctx->ckpt_warmup_commit_target == 0) ctx->perf.perf_start = true;
    }
    return;
  }
  if (!ctx->perf.perf_start && ctx->perf.commit_num >= ctx->ckpt_warmup_commit_target) {
    std::cout << "[CKPT] O3 warmup done at commit " << std::dec << ctx->perf.commit_num << "/"
              << ctx->ckpt_warmup_commit_target << ". Resetting perf counters and starting measure phase." << std::endl;
    ctx->perf.perf_reset();
    ctx->perf.perf_start = true;
  }
  if (ctx->perf.perf_start && ctx->perf.commit_num >= ctx->ckpt_measure_commit_target)
    ctx->exit_reason = ExitReason::SIMPOINT;
}
}  // namespace

void Rob::init() {
  q_.stall_cycle = 0;
  for (int i = 0; i < ROB_LINE_NUM; i++)
    for (int j = 0; j < ROB_BANK_NUM; j++) q_.entry[j][i].valid = false;
  q_.c_head = q_.c_count = 0;
  q_.c_hflag = false;
}

// Dispatch/CSR handshake: ready/empty, stall when a flush-type instruction heads the ROB, and the top-down
// attribution of the oldest incomplete instruction (core bound vs memory bound vs cache miss).
void Rob::ready() {
  const Regs &q = q_;
  // one scan of the head line (the ROB_BANK_NUM oldest entries): a flush-type instruction in it, its oldest incomplete entry
  bool flush_inst = false;
  head_incomplete_ = -1;
  for (int k = 0; k < ROB_BANK_NUM && k < q.c_count; k++) {
    const int idx = (q.c_head + k) % ROB_NUM;
    const Uop &u = q.cent(idx).uop;
    flush_inst = flush_inst || rob_is_flush_inst(u);
    if (head_incomplete_ < 0 && !rob_is_complete(u)) head_incomplete_ = idx;
  }
  out.rob2dis.stall = flush_inst;
  // top-down attribution: the oldest incomplete instruction is the bottleneck (memory, and a cache miss?)
  const bool mem = head_incomplete_ >= 0 && (rob_is_load(q.cent(head_incomplete_).uop) || rob_is_store(q.cent(head_incomplete_).uop));
  out.rob2dis.tma.head_is_memory = mem;
  out.rob2dis.tma.head_is_miss = mem && in.lsu2rob->tma.miss_mask.test(head_incomplete_);
  out.rob2dis.empty = is_empty();
  out.rob2dis.ready = !is_full();
}


// Commit: the longest completed prefix from the head (at most COMMIT_WIDTH entries), cut by the BPU commit
// credits (below). Flush-type instructions, fences, WFI and interrupts retire alone at
// the head; one of them inside the prefix ends it. Commit slot k is the k-th oldest entry.
void Rob::commit() {
  const Regs &q = q_;
  // the pc of the head (a trap / interrupt / flushing instruction redirects to it): the FTQ read port
  head_pc_ = is_empty() ? FtqPc{} : ftq->ftq_pc(q.cent(q.c_head).uop->ftq_idx, q.cent(q.c_head).uop->ftq_offset);
  auto &bc = out.rob_bcast;
  bc.flush = bc.exception = bc.mret = bc.sret = bc.ecall = bc.breakpoint = bc.fence = bc.fence_i = false;
  bc.interrupt = false;
  bc.page_fault_inst = bc.page_fault_load = bc.page_fault_store = bc.illegal_inst = false;
  out.rob2csr.commit = false;
  out.rob2csr.interrupt_resp = false;
  // BPU commit credits: the control-flow instructions of the prefix may not exceed the free BPU update-queue entries and the
  // fetch-block ends may not exceed the free uBTB train-queue entries (the BPU queues drop instead of back-pressuring). Both
  // come from registers only: the BPU occupancy at the start of the cycle minus what this ROB committed last cycle (those
  // commits reach the BPU queues this cycle); conservative, no cross-unit combinational path.

  bc.head_valid = q.c_count > 0;
  bc.head_rob_idx = q.c_count > 0 ? q.c_head : 0;
  bc.head_incomplete_valid = head_incomplete_ >= 0;
  bc.head_incomplete_rob_idx = head_incomplete_ >= 0 ? head_incomplete_ : 0;

  for (int k = 0; k < COMMIT_WIDTH; k++) out.rob_commit.commit_entry[k].valid = false;   // (the uops are written for the committing slots only)

  const bool front_stall = static_cast<bool>(in.top_in->front_stall);
  const bool mispred = in.dec_bcast->mispred;
  const bool intr = csr_state().interrupt_req();
  int n = 0;
  bool single = false;
  if (q.c_count > 0 && !mispred) {
    const Uop &h = q.cent(q.c_head).uop;
    const bool h_alone = rob_is_flush_inst(h) || rob_is_fence(h) || rob_is_wfi(h);
    if (intr) {
      single = true;  // the interrupt is taken on the head instruction (it retires with it)
    } else if (h_alone) {
      single = rob_is_complete(h) && !(rob_is_fence(h) && in.lsu2rob->committed_store_pending);
    } else {
      const int lim = front_stall ? 1 : COMMIT_WIDTH;  // BPU update back-pressure: one per cycle
      int blocks = 0, ctl = 0;
      const int upd_free = static_cast<int>(in.top_in->front_upd_free) - q.c_prev_ctl;
      const int tq_free = static_cast<int>(in.top_in->front_tq_free) - q.c_prev_blk;
      for (int k = 0; k < lim && k < q.c_count; k++) {
        const Uop &u = q.cent((q.c_head + k) % ROB_NUM).uop;
        if (!rob_is_complete(u) || rob_is_flush_inst(u) || rob_is_fence(u) || rob_is_wfi(u)) break;
        const InstType ty = rob_type(u);
        const bool is_ctl = ty == BR || ty == JAL || ty == JALR;
        if (k > 0 && is_ctl && ctl >= upd_free) break;               // the oldest one always goes (progress)
        if (k > 0 && u.ftq_is_last && blocks >= tq_free) break;
        blocks += u.ftq_is_last ? 1 : 0;
        ctl += is_ctl ? 1 : 0;
        n++;
      }
      cw_.prev_ctl = ctl; cw_.prev_blk = blocks;
    }
  }
  if (single) n = 1;
  if (single || n == 0) { cw_.prev_ctl = 0; cw_.prev_blk = 0; }
  if (single) { const Uop &h = q.cent(q.c_head).uop; const InstType ty = rob_type(h); cw_.prev_ctl = (ty == BR || ty == JAL || ty == JALR); cw_.prev_blk = h.ftq_is_last ? 1 : 0; }
  cw_.n = n;
  out.rob2csr.interrupt_resp = single && intr;
  bc.interrupt = single && intr;

  for (int k = 0; k < n; k++) {
    const int idx = (q.c_head + k) % ROB_NUM;
    const Uop &u = q.cent(idx).uop;
    check_load_alignment(u, "DUT: Load address misaligned at commit!");
    out.rob_commit.commit_entry[k].valid = true;
    out.rob_commit.commit_entry[k].uop = u;
    out.rob_commit.commit_extra[k].src1_areg = u->src1_areg;
    out.rob_commit.commit_extra[k].imm = u->imm;
  }
  if (single) single_actions(q.cent(q.c_head).uop);


  // the dispatch stage allocates from the tail of the registered state (entries freed this cycle are reused next cycle)
  const int tail = (q.c_head + q.c_count) % ROB_NUM;
  out.rob2dis.enq_idx = tail;
  out.rob2dis.rob_flag = q.c_hflag ^ (q.c_head + q.c_count >= ROB_NUM);
  out.rob2dis.free_cnt = ROB_NUM - q.c_count;

  cw_.stall_cycle = (n > 0 ? 0 : q.stall_cycle) + 1;
  if (cw_.stall_cycle > 50000) {
    std::fprintf(stderr, "ROB deadlock at cycle %llu: count=%d head=%d n=%d single=%d front_stall=%d\n",
                 (unsigned long long)ctx->perf.cycle, q.c_count, q.c_head, n, (int)single, (int)front_stall);
    for (int k = 0; k < q.c_count && k < 3; k++) {   // the oldest entries: what they wait for
      const Uop &h = q.cent((q.c_head + k) % ROB_NUM).uop;
      std::fprintf(stderr, "  rob[%d] idx=%d type=%d pc=%llx expect=%x cplt=%x flush_pipe=%d valid=%d\n", k, (q.c_head + k) % ROB_NUM, (int)rob_type(h),
                   (unsigned long long)h.diag_val, (unsigned)h->expect_mask, (unsigned)h.cplt_mask, (int)h.flush_pipe,
                   (int)q.entry[(q.c_head + k) % ROB_NUM % ROB_BANK_NUM][(q.c_head + k) % ROB_NUM / ROB_BANK_NUM].valid);
    }
    std::exit(1);   // (the watchdog: no commit for 50000 cycles)
  }
}

// What a single-retired instruction triggers: flush / trap / CSR / fence side effects.
void Rob::single_actions(const Uop &u) {
  auto &bc = out.rob_bcast;
  const bool flush_inst = rob_is_flush_inst(u), wfi = rob_is_wfi(u), intr = out.rob2csr.interrupt_resp;
  if (flush_inst || intr || wfi) {
    xlen_t pc = 0;
    if (flush_inst || intr) {
      Assert(!is_empty());
      pc = head_pc_.pc;
      bc.is_rvc = head_pc_.is_rvc;
      bc.flush = true;
      bc.exception = rob_is_exception(u) || intr || (!intr && rob_type(u) == EBREAK && is_semihost_marker(pc));
      bc.pc = pc;
    }
    const InstType t = rob_type(u);
    if (intr) {
      // an interrupt takes priority over whatever instruction retires with it
    } else if (t == ECALL) { bc.ecall = true; bc.pc = pc; }
    else if (t == MRET) bc.mret = true;
    else if (t == SRET) bc.sret = true;
    else if (u.page_fault_store) { bc.page_fault_store = true; bc.trap_val = u.diag_val; }
    else if (u.page_fault_load) { bc.page_fault_load = true; bc.trap_val = u.diag_val; }
    else if (u.page_fault_inst) { bc.page_fault_inst = true; bc.trap_val = pc; }
    else if (u.illegal_inst) { bc.illegal_inst = true; bc.trap_val = u.diag_val; }
    else if (t == EBREAK && is_semihost_marker(pc)) { bc.breakpoint = true; bc.trap_val = pc; }  // breakpoint exception, cause 3, tval = pc
    else if (t == EBREAK) ctx->exit_reason = ExitReason::EBREAK;
    else if (wfi) { /* retires without leaving the simulator */ }
    else if (t == CSR) {
      out.rob2csr.commit = true;
      const bool rw_family = u->func3 == 0b001 || u->func3 == 0b101;
      out.rob2csr.we = rw_family || u->imm != 0;
      out.rob2csr.idx = u->csr_idx;
      out.rob2csr.wcmd = u->func3 & 0b11;
      out.rob2csr.wdata = 0;
      if (const tb::Fact *f2 = tbhook::fact_by_rob(u->rob_idx)) out.rob2csr.wdata = f2->csr_wdata;
    }
    else if (t == SFENCE_VMA) bc.fence = true;
    else if (t == FENCE_I) bc.fence_i = true;
    else if (u.flush_pipe) {  // memory-order violator (EXP-018): no CSR/MMU action, refetch from pc+4
      bc.pc = pc;
    }
    else Assert(0 && "Rob: unexpected single-commit instruction");
  }
}

// Execution results mark their issue port's completion bit; load/store ports also record the address or the fault.
void Rob::complete() {
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    if (!in.exu2rob->entry[i].valid) continue;
    const Uop &wb = in.exu2rob->entry[i].uop;
    const int bank = get_rob_bank(wb->rob_idx), line = get_rob_line(wb->rob_idx);
    const auto bit = rob_cplt_mask_from_issue_port(i);
    Uop &u = q_.entry[bank][line].uop;
    if (wb.rob_defer) continue;  // speculative load write-back: the completion bit is set by its second write-back
    Assert((u.cplt_mask & bit) == 0 && "ROB: duplicate completion bit set");
    u.cplt_mask |= bit;
    Assert((u.cplt_mask & ~u->expect_mask) == 0 && "ROB: completion bit outside expected mask");
    Assert(rob_cplt_popcount(u.cplt_mask) <= rob_cplt_popcount(u->expect_mask) && "ROB: completion overflow");

    const bool fault = rob_is_page_fault(wb);
    const bool load_port = i >= IQ_LD_PORT_BASE && i < IQ_LD_PORT_BASE + LSU_LDU_COUNT;
    const bool sta_port = i >= IQ_STA_PORT_BASE && i < IQ_STA_PORT_BASE + LSU_STA_COUNT;
    if (load_port || sta_port) {
      u.diag_val = wb.diag_val;  // resolved address, used by commit-time checks
      if (fault) {
        u.diag_val = wb.result;
        u.page_fault_load |= wb.page_fault_load;
        u.page_fault_store |= wb.page_fault_store;
        u.page_fault_inst |= wb.page_fault_inst;
        if (sta_port && u.page_fault_store) u.page_fault_load = false;
      }
    }
    // an instruction can write back several uops (STA/STD): these flags only accumulate
    u.flush_pipe = u.flush_pipe || wb.flush_pipe;
    u.illegal_inst = u.illegal_inst || wb.illegal_inst;   // F/D: an FP uop found illegal at execute (FS = Off, reserved encoding)
    u.is_mmio = u.is_mmio || wb.is_mmio;
    if (is_branch_uop(wb.op)) {
      u.diag_val = wb.diag_val;
      u.mispred = wb.mispred;
      u.br_taken = wb.br_taken;
    }
  }
}

// Branch mispredict: the tail moves to the entry right after the branch; everything younger is invalidated (head /
// count: the registered ones).
void Rob::branch(int head, int count, int &c_count) {
  if (!in.dec_bcast->mispred || out.rob_bcast.flush || (CFG_FLUSH_REG && pend_.v)) return;   // (a flush waits for its delivery: the ROB is empty, a stale mispredict must not recover it)
  tbhook::on_mispredict((uint64_t)q_.entry[get_rob_bank(in.dec_bcast->redirect_rob_idx)][get_rob_line(in.dec_bcast->redirect_rob_idx)].uop->dbg.inst_idx);
  const int bidx = static_cast<int>(in.dec_bcast->redirect_rob_idx);
  const int keep = (bidx - head + ROB_NUM) % ROB_NUM + 1;  // entries up to and including the branch
  Assert(keep <= count && "ROB: mispredicted branch outside the valid range");
  q_.cent(bidx).uop.ftq_is_last = true;   // everything after the branch in its fetch block is flushed
  for (int k = keep; k < count; k++) q_.cent((head + k) % ROB_NUM).valid = false;
  c_count = keep;  // commit is blocked in a mispredict cycle, so the head does not move
}

// Dispatch enqueues at the tail: the fired slots are the first m valid slots; Dispatch numbered them tail, tail+1, ...
void Rob::fire(int &c_count) {
  if (!out.rob2dis.ready) return;
  int m = 0;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (!in.dis2rob->dis_fire[i]) continue;
    const int idx = (out.rob2dis.enq_idx + m) % ROB_NUM;
    Assert(static_cast<int>(in.dis2rob->uop[i]->rob_idx) == idx && "ROB: dispatch numbering mismatch");
    auto &e = q_.cent(idx);
    e.valid = true;
    e.uop = in.dis2rob->uop[i];
    e.uop.cplt_mask = 0;
    tbhook::on_dispatch((uint64_t)e.uop->dbg.inst_idx, e.uop->dbg.pc, e.uop->dbg.instruction, idx, ilen_of(e.uop->is_rvc));
    if (ROBTR_ON()) std::fprintf(stderr, "ROBTR %llu fire idx=%d type=%d pc=%llx\n", (unsigned long long)ctx->perf.cycle, idx, (int)rob_type(e.uop), (unsigned long long)e.uop->dbg.pc);
    m++;
  }
  c_count += m;
  Assert(c_count <= ROB_NUM && "ROB: overflow");
}

// A global flush empties the ROB and resets the pointers (overrides any recovery or enqueue above).
void Rob::flush(int &c_head, int &c_count, bool &c_hflag) {
  if (!out.rob_bcast.flush) return;
  if (ROBTR_ON()) std::fprintf(stderr, "ROBTR %llu FLUSH count=%d head=%d\n", (unsigned long long)ctx->perf.cycle, c_count, c_head);
  for (int i = 0; i < ROB_BANK_NUM; i++)
    for (int j = 0; j < ROB_LINE_NUM; j++) q_.entry[i][j].valid = false;
  c_head = c_count = 0;
  c_hflag = false;
}

// The clock edge: the commit decided by comb_commit, then the write-backs, the mispredict recovery, the enqueue and the
// flush of this cycle (the RTL's rob_entry always_ff and pointer registers).
void Rob::seq() {
  const int head = q_.c_head, count = q_.c_count;
  int c_head = head, c_count = count;
  bool c_hflag = q_.c_hflag;
  for (int k = 0; k < cw_.n; k++) q_.cent((head + k) % ROB_NUM).valid = false;
  if (cw_.n > 0) {
    c_head = (head + cw_.n) % ROB_NUM;
    if (head + cw_.n >= ROB_NUM) c_hflag = !c_hflag;
    c_count = count - cw_.n;
  }
  q_.c_prev_ctl = cw_.prev_ctl; q_.c_prev_blk = cw_.prev_blk; q_.stall_cycle = cw_.stall_cycle;
  complete();
  branch(head, count, c_count);
  fire(c_count);
  flush(c_head, c_count, c_hflag);
  q_.c_head = c_head; q_.c_count = c_count; q_.c_hflag = c_hflag;
  bc_prev_ = out.rob_bcast;
  pend_ = pend_nxt_;
}

// PMU (EXP-013): same "oldest incomplete instruction" as ready(); also the number of valid entries.
bool Rob::pmu_head_incomplete(Uop &out, int &occupied_entries) const {
  const Regs &q = q_;
  occupied_entries = 0;
  for (int l = 0; l < ROB_LINE_NUM; l++)
    for (int i = 0; i < ROB_BANK_NUM; i++) occupied_entries += q.entry[i][l].valid ? 1 : 0;
  if (head_incomplete_ < 0) return false;   // (comb_commit's scan of this cycle)
  out = q.cent(head_incomplete_).uop;
  return true;
}

// the head: commit and its broadcasts
uint64_t g_dbg_cycle = 0;   // (debug: the cycle number for the other modules' ROBTR traces)
void Rob::comb_commit() {
  g_dbg_cycle = ctx->perf.cycle;
  ready();
  commit();
  out.rob_bcast.flush_now = out.rob_bcast.flush;
  if (CFG_FLUSH_REG) {   // the flush / trap fields of the previous cycle, with the head information of this one
    RobBroadcastIO d = bc_prev_;
    const RobBroadcastIO &bc = out.rob_bcast;
    d.head_rob_idx = bc.head_rob_idx; d.head_valid = bc.head_valid;
    d.head_incomplete_rob_idx = bc.head_incomplete_rob_idx; d.head_incomplete_valid = bc.head_incomplete_valid;
    d.interrupt = bc.interrupt;   // (qualifies this cycle's commit: an instruction retiring with an interrupt does not commit)
    d.flush_now = bc.flush;
    out.rob_bcast_d = d;
  } else {
    out.rob_bcast_d = out.rob_bcast;
  }
}

// Where the front end goes: a flush from the commit (a trap or xRET target from the CSR file, else the next instruction)
// or a branch mispredict from the IDU (with what the BPU needs to repair its history); the flush is reported to the
// trace bridge.
void Rob::redirect() {
  BackOut &o = *out.top_out;
  const auto &bc = out.rob_bcast;
  if (CFG_FLUSH_REG) {   // the redirect of a global flush goes to the front end one cycle after the decision, with the back-end clear
    pend_nxt_ = {};
    if (pend_.v) {
      o.flush = true; o.fence_i = pend_.fence_i; o.itlb_flush = pend_.itlb; o.mispred = true; o.redirect_pc = pend_.pc;
      o.rf_valid = false;
      return;
    }
    o.flush = false; o.fence_i = false; o.itlb_flush = false;
    if (bc.flush) {
      int64_t committing = -1;
      for (int i = 0; i < COMMIT_WIDTH && committing < 0; i++)
        if (out.rob_commit.commit_entry[i].valid) committing = (int64_t)out.rob_commit.commit_entry[i].uop->dbg.inst_idx;
      tbhook::on_flush(committing, bc.interrupt);
      const Csr::TrapOut trap = csr_state().trap_out();
      pend_nxt_.v = true; pend_nxt_.fence_i = bc.fence_i; pend_nxt_.itlb = bc.fence;
      pend_nxt_.pc = (bc.mret || bc.sret) ? trap.epc : bc.exception ? trap.trap_pc : bc.pc + ilen_of(bc.is_rvc);
      o.mispred = false; o.rf_valid = false; o.stall = !in.pre2front->ready; o.redirect_pc = 0;
      return;
    }
  }
  o.flush = bc.flush;
  o.fence_i = bc.fence_i;
  o.itlb_flush = bc.fence;
  if (bc.flush) {
    int64_t committing = -1;
    for (int i = 0; i < COMMIT_WIDTH && committing < 0; i++)
      if (out.rob_commit.commit_entry[i].valid) committing = (int64_t)out.rob_commit.commit_entry[i].uop->dbg.inst_idx;
    tbhook::on_flush(committing, bc.interrupt);
    o.mispred = true;
    const Csr::TrapOut trap = csr_state().trap_out();   // (the CSR file: its registers and this cycle's commit)
    o.redirect_pc = (bc.mret || bc.sret) ? trap.epc : bc.exception ? trap.trap_pc : bc.pc + ilen_of(bc.is_rvc);
    return;
  }
  o.mispred = in.dec_bcast->mispred;
  o.stall = !in.pre2front->ready;
  o.redirect_pc = in.idu_br_latch->redirect_pc;
  { static const bool tr = std::getenv("MSD_TRACE") != nullptr;
    if (tr && o.mispred) std::fprintf(stderr, "[MP] cyc=%lld redirect_pc=%llx\n", (long long)sim_time, (unsigned long long)o.redirect_pc); }
  o.rf_valid = in.dec_bcast->mispred;
  if (o.rf_valid) {
    const uint32_t fi = in.idu_br_latch->ftq_idx;
    o.rf_slot = in.idu_br_latch->ftq_off;
    o.rf_blk = ftq->ftq_blk_id(fi, o.rf_slot);
    o.rf_type = in.idu_br_latch->br_type;
    o.rf_pc = ftq->ftq_slot_pc(fi, o.rf_slot);
    o.rf_is_rvc = ftq->ftq_slot_rvc(fi, o.rf_slot);
  }
}

// The commit as the back end's output (the uops with their FTQ data: the BPU training; on a flush their next pc is the
// redirect target), the commits reported to the trace bridge (and FS-dirty from their facts), the commit statistics and
// an interrupt taken at commit.
void Rob::report_commit() {
  BackOut &o = *out.top_out;
  const bool interrupt = out.rob_bcast.interrupt;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) o.fire[i] = in.pre2front->fire[i];
  out.rob2csr.fs_dirty = false;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    const auto &c = out.rob_commit.commit_entry[i];
    o.commit_entry[i].valid = c.valid;
    if (!c.valid) continue;
    o.commit_entry[i].uop = c.uop;
    o.commit_extra[i] = out.rob_commit.commit_extra[i];
    o.commit_ftq[i] = in.ftq_commit_info->resp[i];
    if (o.flush) o.commit_entry[i].uop.diag_val = o.redirect_pc;
    if (interrupt) continue;
    if (tbhook::fs_dirty_at_commit((uint64_t)c.uop->dbg.inst_idx)) out.rob2csr.fs_dirty = true;
    tbhook::on_commit((uint64_t)c.uop->dbg.inst_idx);
  }
  if (!interrupt)
    for (int i = 0; i < COMMIT_WIDTH; i++)
      if (out.rob_commit.commit_entry[i].valid) count_commit(ctx, out.rob_commit.commit_entry[i].uop);
  if (interrupt) ctx->run_interrupt_hook();   // (the trap of this cycle, to the trace bridge)
}

// the back end's outputs of the cycle: the redirect and the commit
void Rob::comb_redirect() {
  redirect();
  report_commit();
}
