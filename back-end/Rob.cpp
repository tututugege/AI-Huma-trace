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
#include <cstdio>
#include <cstdlib>


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
// A completed load's address must be naturally aligned (only checked when the load did not fault).
inline void check_load_alignment(const Uop &u, const char *msg) {
  if (!rob_is_load(u) || !rob_is_complete(u) || rob_is_page_fault(u)) return;
  const uint32_t mask = (u->func3 & 3u) == 0 ? 0u : (u->func3 & 3u) == 1 ? 1u : 3u;
  Assert((u.diag_val & mask) == 0 && msg);
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
  out.rob2dis.stall = false;
  out.rob2csr.commit = false;
  out.rob2csr.interrupt_resp = false;

  bool found = false, mem = false, miss = false;
  for (int k = 0; k < ROB_BANK_NUM && k < q.c_count; k++) {   // the COMMIT_WIDTH oldest entries
    const int idx = (q.c_head + k) % ROB_NUM;
    const auto &e = q.cent(idx);
    if (rob_is_flush_inst(e.uop)) out.rob2dis.stall = true;
    if (!found && !rob_is_complete(e.uop)) {  // the oldest incomplete instruction is the bottleneck
      found = true;
      mem = rob_is_load(e.uop) || rob_is_store(e.uop);
      miss = mem && in.lsu2rob->tma.miss_mask.test(idx);
    }
  }
  out.rob2dis.tma.head_is_memory = found && mem;
  out.rob2dis.tma.head_is_miss = found && miss;
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
  out.rob2csr.interrupt_resp = false;
  // BPU commit credits: the control-flow instructions of the prefix may not exceed the free BPU update-queue entries and the
  // fetch-block ends may not exceed the free uBTB train-queue entries (the BPU queues drop instead of back-pressuring). Both
  // come from registers only: the BPU occupancy at the start of the cycle minus what this ROB committed last cycle (those
  // commits reach the BPU queues this cycle); conservative, no cross-unit combinational path.

  bc.head_valid = bc.head_incomplete_valid = false;
  bc.head_rob_idx = bc.head_incomplete_rob_idx = 0;
  if (q.c_count > 0) {
    bc.head_valid = true;
    bc.head_rob_idx = q.c_head;
    for (int k = 0; k < ROB_BANK_NUM && k < q.c_count; k++) {
      const int idx = (q.c_head + k) % ROB_NUM;
      if (!rob_is_complete(q.cent(idx).uop)) { bc.head_incomplete_valid = true; bc.head_incomplete_rob_idx = idx; break; }
    }
  }

  for (int k = 0; k < COMMIT_WIDTH; k++) {
    const int idx = (q.c_head + k) % ROB_NUM;
    out.rob_commit.commit_entry[k].uop = q.cent(idx).uop;
    out.rob_commit.commit_extra[k].src1_areg = q.cent(idx).uop->src1_areg;
    out.rob_commit.commit_extra[k].imm = q.cent(idx).uop->imm;
    out.rob_commit.commit_entry[k].valid = false;
  }

  const bool front_stall = static_cast<bool>(in.top_in->front_stall);
  const bool mispred = in.dec_bcast->mispred;
  const bool intr = csr->interrupt_req();
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
    check_load_alignment(q.cent(idx).uop, "DUT: Load address misaligned at commit!");
    out.rob_commit.commit_entry[k].valid = true;
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
  if (!in.dec_bcast->mispred || out.rob_bcast.flush) return;
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
    m++;
  }
  c_count += m;
  Assert(c_count <= ROB_NUM && "ROB: overflow");
}

// A global flush empties the ROB and resets the pointers (overrides any recovery or enqueue above).
void Rob::flush(int &c_head, int &c_count, bool &c_hflag) {
  if (!out.rob_bcast.flush) return;
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
}

// PMU (EXP-013): same "oldest incomplete instruction" as ready(); also the number of valid entries.
bool Rob::pmu_head_incomplete(Uop &out, int &occupied_entries) const {
  const Regs &q = q_;
  occupied_entries = 0;
  for (int l = 0; l < ROB_LINE_NUM; l++)
    for (int i = 0; i < ROB_BANK_NUM; i++) occupied_entries += q.entry[i][l].valid ? 1 : 0;
  for (int k = 0; k < ROB_BANK_NUM && k < q.c_count; k++) {
    const auto &e = q.entry[((q.c_head + k) % ROB_NUM) % ROB_BANK_NUM][((q.c_head + k) % ROB_NUM) / ROB_BANK_NUM];
    if (!rob_is_complete(e.uop)) { out = e.uop; return true; }
  }
  return false;
}

// the head: commit and its broadcasts
void Rob::comb_commit() {
  ready();
  commit();
}

// the back end's outputs of the cycle: the redirect (a branch mispredict from the IDU, a flush from the commit, its
// target from the CSR), the fetch-group handshake, the commit with its BPU training data, the commit / flush reported to
// the trace bridge, the FS-dirty request to the CSR
void Rob::comb_redirect() {
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) out.top_out->fire[i] = in.pre2front->fire[i];
  out.top_out->flush = out.rob_bcast.flush;
  if (out.top_out->flush) {
    int64_t committing = -1;
    for (int i = 0; i < COMMIT_WIDTH; i++) {
      if (out.rob_commit.commit_entry[i].valid) {
        committing = (int64_t)out.rob_commit.commit_entry[i].uop->dbg.inst_idx;
        break;
      }
    }
    tbhook::on_flush(committing, out.rob_bcast.interrupt);
  }
  out.top_out->fence_i = out.rob_bcast.fence_i;
  out.top_out->itlb_flush = out.rob_bcast.fence;

  // 1. Normal case (No Rob flush)
  if (!out.rob_bcast.flush) {
    out.top_out->mispred = in.dec_bcast->mispred;
    out.top_out->stall = !in.pre2front->ready;
    out.top_out->redirect_pc = in.idu_br_latch->redirect_pc;
    out.top_out->rf_valid = in.dec_bcast->mispred;
    if (out.top_out->rf_valid) {
      const uint32_t fi = in.idu_br_latch->ftq_idx;
      out.top_out->rf_slot = in.idu_br_latch->ftq_off;
      out.top_out->rf_blk = ftq->ftq_blk_id(fi, out.top_out->rf_slot);
      out.top_out->rf_type = in.idu_br_latch->br_type;
      out.top_out->rf_pc = ftq->ftq_slot_pc(fi, out.top_out->rf_slot);
      out.top_out->rf_is_rvc = ftq->ftq_slot_rvc(fi, out.top_out->rf_slot);
    }
  } else {
    out.top_out->mispred = true;
    const Csr::TrapOut trap = csr->trap_out();   // (the CSR file: its registers and this cycle's commit)
    if (out.rob_bcast.mret || out.rob_bcast.sret) {
      out.top_out->redirect_pc = trap.epc;
    } else if (out.rob_bcast.exception) {
      out.top_out->redirect_pc = trap.trap_pc;
    } else {
      out.top_out->redirect_pc = out.rob_bcast.pc + ilen_of(out.rob_bcast.is_rvc);
    }
  }

  out.rob2csr.fs_dirty = false;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    out.top_out->commit_entry[i] =
        out.rob_commit.commit_entry[i].uop.to_inst_entry(
            out.rob_commit.commit_entry[i].valid);
    out.top_out->commit_extra[i] = out.rob_commit.commit_extra[i];
    out.top_out->commit_ftq[i] = in.ftq_commit_info->resp[i];
    if (out.rob_commit.commit_entry[i].valid &&
        !out.rob_bcast.interrupt) {
      if (tbhook::fs_dirty_at_commit((uint64_t)out.rob_commit.commit_entry[i].uop->dbg.inst_idx)) out.rob2csr.fs_dirty = true;
      tbhook::on_commit(
          (uint64_t)out.rob_commit.commit_entry[i].uop->dbg.inst_idx);
    }
    if (out.top_out->commit_entry[i].valid && out.top_out->flush) {
      // Flush: the next PC of every committing instruction is the redirect target (trap vector, epc, ...)
      out.top_out->commit_entry[i].uop.diag_val = out.top_out->redirect_pc;
    }
  }
}
