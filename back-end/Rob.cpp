#include "Rob.h"
#include "IO.h"
#include "RealLsu.h"
#include "SimCpu.h"
#include "Wires.h"
#include "config.h"
#include "trace_hook.h"
#include "util.h"
#include <cstdio>
#include <cstdlib>

void g_msp_flush_commit(unsigned rob_idx, long long younger);  // RealLsuPipe.cpp: violation penalty statistics (EXP-019)
void g_msp_tick(bool committed);

namespace {
inline bool rob_is_store(const Uop &u) { return u.tma.mem_commit_is_store; }
inline bool rob_is_load(const Uop &u) { return u.tma.mem_commit_is_load; }
inline bool rob_is_page_fault(const Uop &u) { return u.page_fault_inst || u.page_fault_load || u.page_fault_store; }
// RISC-V semihosting marker: slli x0,x0,0x1f ; ebreak ; srai x0,x0,7. OpenSBI's generic platform probes it and expects a breakpoint
// trap into M mode; every other ebreak still ends the simulation (the bare-metal tests' exit). Same rule as the function model.
inline bool is_semihost_marker(uint32_t pc) {
  return tbhook::mem_read(pc - 4) == 0x01f01013u && tbhook::mem_read(pc + 4) == 0x40705013u;
}
inline bool rob_is_complete(const Uop &u) { return u.cplt_mask == u.expect_mask; }
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
  deq_ptr = deq_ptr_1 = 0;
  enq_ptr = enq_ptr_1 = 0;
  enq_flag = enq_flag_1 = false;
  deq_flag = deq_flag_1 = false;
  stall_cycle = 0;
  for (int i = 0; i < ROB_LINE_NUM; i++)
    for (int j = 0; j < ROB_BANK_NUM; j++) {
      entry[j][i].valid = false;
      entry_1[j][i] = entry[j][i];  // entry_1 equals entry at the start of every cycle
    }
  std::memset(dirty_, 0, sizeof(dirty_));
  dirty_all_ = false;
  c_head = c_head_1 = c_count = c_count_1 = 0;
  c_hflag = c_hflag_1 = false;
}

// Working pointers start from the registered ones; entry_1 already equals entry (seq() copies back the touched slots).
void Rob::comb_begin() {
  enq_ptr_1 = enq_ptr;
  deq_ptr_1 = deq_ptr;
  enq_flag_1 = enq_flag;
  deq_flag_1 = deq_flag;
  c_head_1 = c_head; c_count_1 = c_count; c_hflag_1 = c_hflag;
}

// Dispatch/CSR handshake: ready/empty, stall when a flush-type instruction heads the ROB, and the top-down
// attribution of the oldest incomplete instruction (core bound vs memory bound vs cache miss).
void Rob::comb_ready() {
  Wires &w = *wires;
  w.rob2dis.stall = false;
  w.rob2csr.commit = false;
  w.rob2csr.interrupt_resp = false;

  bool found = false, mem = false, miss = false;
  if (rob_compact()) {  // the COMMIT_WIDTH oldest entries play the role of the head line
    for (int k = 0; k < ROB_BANK_NUM && k < c_count; k++) {
      const int idx = (c_head + k) % ROB_NUM;
      const auto &e = cent(idx);
      if (rob_is_flush_inst(e.uop)) w.rob2dis.stall = true;
      if (!found && !rob_is_complete(e.uop)) {
        found = true;
        mem = rob_is_load(e.uop) || rob_is_store(e.uop);
        miss = mem && w.lsu2rob.tma.miss_mask.test(idx);
      }
    }
  } else
  for (int i = 0; i < ROB_BANK_NUM; i++) {
    const auto &e = entry[i][deq_ptr];
    if (!e.valid) continue;
    if (rob_is_flush_inst(e.uop)) w.rob2dis.stall = true;
    if (!found && !rob_is_complete(e.uop)) {  // the oldest incomplete instruction is the bottleneck
      found = true;
      mem = rob_is_load(e.uop) || rob_is_store(e.uop);
      const uint32_t idx = i + deq_ptr * ROB_BANK_NUM;
      miss = mem && idx < ROB_NUM && w.lsu2rob.tma.miss_mask.test(idx);
    }
  }
  w.rob2dis.tma.head_not_ready = found;
  w.rob2dis.tma.head_is_memory = found && mem;
  w.rob2dis.tma.head_is_miss = found && miss;
  w.rob2dis.empty = is_empty();
  w.rob2dis.ready = !is_full();
}

// PC lookup for the oldest valid instruction of the head line.
void Rob::comb_ftq_pc_req() {
  auto &req = wires->ftq_rob_pc_req.req[0];
  req = {};
  if (is_empty()) return;
  if (rob_compact()) {
    req.valid = true;
    req.ftq_idx = cent(c_head).uop->ftq_idx;
    req.ftq_offset = cent(c_head).uop->ftq_offset;
    return;
  }
  for (int i = 0; i < ROB_BANK_NUM; i++)
    if (entry[i][deq_ptr].valid) {
      req.valid = true;
      req.ftq_idx = entry[i][deq_ptr].uop->ftq_idx;
      req.ftq_offset = entry[i][deq_ptr].uop->ftq_offset;
      return;
    }
}

// Commit arbitration. A whole head line retires at once when every valid entry is complete; flush-type instructions,
// fences, WFI and interrupts retire alone (precisely), and a ready non-flush entry may retire alone when the line is
// blocked by a younger entry so older stores can drain.
void Rob::comb_commit() {
  if (rob_compact()) return comb_commit_compact();
  Wires &w = *wires;
  auto &bc = w.rob_bcast;
  bc.flush = bc.exception = bc.mret = bc.sret = bc.ecall = bc.breakpoint = bc.fence = bc.fence_i = false;
  bc.interrupt = false;
  bc.page_fault_inst = bc.page_fault_load = bc.page_fault_store = bc.illegal_inst = false;
  w.rob2csr.interrupt_resp = false;

  // head info for the LSU (MMIO issue order): first valid entry and first incomplete entry of the head line
  bc.head_valid = bc.head_incomplete_valid = false;
  bc.head_rob_idx = bc.head_incomplete_rob_idx = 0;
  bool head_line_empty = true;
  for (int i = 0; i < ROB_BANK_NUM; i++) {
    const auto &e = entry[i][deq_ptr];
    if (!e.valid) continue;
    head_line_empty = false;
    if (!is_empty()) {
      if (!bc.head_valid) { bc.head_valid = true; bc.head_rob_idx = make_rob_idx(deq_ptr, i); }
      if (!bc.head_incomplete_valid && !rob_is_complete(e.uop)) {
        bc.head_incomplete_valid = true;
        bc.head_incomplete_rob_idx = make_rob_idx(deq_ptr, i);
      }
    }
  }

  const bool front_stall = static_cast<bool>(w.in.front_stall);  // BPU update queue back-pressure
  const bool mispred = w.dec_bcast.mispred;
  bool commit = !is_empty() && !mispred && !(!head_line_empty && front_stall);
  for (int i = 0; i < ROB_BANK_NUM; i++)
    if (entry[i][deq_ptr].valid) commit = commit && rob_is_complete(entry[i][deq_ptr].uop);

  bool single = false, progress = false, interrupt_fire = false;
  int single_idx = 0;
  if (!mispred) {
    const bool intr = w.csr2rob.interrupt_req;
    for (int i = 0; i < ROB_BANK_NUM && !single; i++) {
      const auto &e = entry[i][deq_ptr];
      single = intr || (e.valid && (rob_is_flush_inst(e.uop) || rob_is_fence(e.uop) || rob_is_wfi(e.uop)));
    }
    bool has_head = false;
    for (int i = 0; i < ROB_BANK_NUM && !has_head; i++) {
      if (!entry[i][deq_ptr].valid) continue;
      has_head = true;
      single_idx = i;
      const Uop &h = entry[i][deq_ptr].uop;
      if (!intr && !rob_is_complete(h)) single = false;
      if (!intr && rob_is_fence(h) && w.lsu2rob.committed_store_pending) {  // fences wait for committed stores to drain
        single = false;
        commit = false;
      }
      interrupt_fire = single && intr;
    }
    if (!has_head) single = false;  // an empty line pops through the group path

    if (!commit && !single && !intr) {  // forward progress: retire the oldest ready non-flush entry alone
      for (int i = 0; i < ROB_BANK_NUM; i++) {
        if (!entry[i][deq_ptr].valid) continue;
        const Uop &u = entry[i][deq_ptr].uop;
        if (rob_is_complete(u) && !rob_is_flush_inst(u)) { single = progress = true; single_idx = i; }
        break;
      }
    }
  }
  (void)progress;
  w.rob2csr.interrupt_resp = interrupt_fire;
  bc.interrupt = interrupt_fire;

  for (int i = 0; i < ROB_BANK_NUM; i++) {
    w.rob_commit.commit_entry[i].uop = entry[i][deq_ptr].uop;
    out.commit_extra[i].src1_areg = entry[i][deq_ptr].uop->src1_areg;
    out.commit_extra[i].imm = entry[i][deq_ptr].uop->imm;
    w.rob_commit.commit_entry[i].valid = false;
  }

  auto advance_head_if_drained = [&]() {
    for (int i = 0; i < ROB_BANK_NUM; i++)
      if (entry_1[i][deq_ptr].valid) return;
    LOOP_INC(deq_ptr_1, ROB_LINE_NUM);
    if (deq_ptr_1 == 0) deq_flag_1 = !deq_flag;
  };

  if (commit && !single) {
    for (int i = 0; i < ROB_BANK_NUM; i++) {
      const auto &e = entry[i][deq_ptr];
      if (!e.valid) continue;
      check_load_alignment(e.uop, "DUT: Load address misaligned at commit!");
      w.rob_commit.commit_entry[i].valid = true;
      entry_1[i][deq_ptr].valid = false;
      touch(i, deq_ptr);
    }
    stall_cycle = 0;
    advance_head_if_drained();
  } else if (single) {
    stall_cycle = 0;
    w.rob_commit.commit_entry[single_idx].valid = true;
    const Uop &u = entry[single_idx][deq_ptr].uop;
    if (entry[single_idx][deq_ptr].valid) check_load_alignment(u, "DUT: Load address misaligned at single commit!");
    entry_1[single_idx][deq_ptr].valid = false;
    touch(single_idx, deq_ptr);
    advance_head_if_drained();

    single_actions(u);
  }

  {
    bool any = false;
    for (int i = 0; i < ROB_BANK_NUM; i++) any |= (bool)w.rob_commit.commit_entry[i].valid;
    g_msp_tick(any);
  }

  w.rob2dis.enq_idx = enq_ptr;
  w.rob2dis.rob_flag = enq_flag;

  if (++stall_cycle > 50000) {
    std::fprintf(stderr, "ROB deadlock at cycle %llu: empty=%d deq_ptr=%d enq_ptr=%d commit=%d single=%d front_stall=%d head_line_empty=%d\n",
                 (unsigned long long)ctx->perf.cycle, (int)is_empty(), (int)deq_ptr, (int)enq_ptr, (int)commit, (int)single,
                 (int)front_stall, (int)head_line_empty);
    Assert(0 && "ROB Deadlock detected (stall_cycle > 50000)");
    std::exit(1);
  }
}

// Compact mode commit: the longest completed prefix from the head (at most COMMIT_WIDTH entries), cut by the BPU commit
// credits (below). Flush-type instructions, fences, WFI and interrupts still retire alone at
// the head; one of them inside the prefix ends it. Commit slot k is the k-th oldest entry.
void Rob::comb_commit_compact() {
  Wires &w = *wires;
  auto &bc = w.rob_bcast;
  bc.flush = bc.exception = bc.mret = bc.sret = bc.ecall = bc.breakpoint = bc.fence = bc.fence_i = false;
  bc.interrupt = false;
  bc.page_fault_inst = bc.page_fault_load = bc.page_fault_store = bc.illegal_inst = false;
  w.rob2csr.interrupt_resp = false;
  // BPU commit credits: the control-flow instructions of the prefix may not exceed the free BPU update-queue entries and the
  // fetch-block ends may not exceed the free uBTB train-queue entries (the BPU queues drop instead of back-pressuring). Both
  // come from registers only: the BPU occupancy at the start of the cycle minus what this ROB committed last cycle (those
  // commits reach the BPU queues this cycle); conservative, no cross-unit combinational path.

  bc.head_valid = bc.head_incomplete_valid = false;
  bc.head_rob_idx = bc.head_incomplete_rob_idx = 0;
  if (c_count > 0) {
    bc.head_valid = true;
    bc.head_rob_idx = c_head;
    for (int k = 0; k < ROB_BANK_NUM && k < c_count; k++) {
      const int idx = (c_head + k) % ROB_NUM;
      if (!rob_is_complete(cent(idx).uop)) { bc.head_incomplete_valid = true; bc.head_incomplete_rob_idx = idx; break; }
    }
  }

  for (int k = 0; k < COMMIT_WIDTH; k++) {
    const int idx = (c_head + k) % ROB_NUM;
    w.rob_commit.commit_entry[k].uop = cent(idx).uop;
    out.commit_extra[k].src1_areg = cent(idx).uop->src1_areg;
    out.commit_extra[k].imm = cent(idx).uop->imm;
    w.rob_commit.commit_entry[k].valid = false;
  }

  const bool front_stall = static_cast<bool>(w.in.front_stall);
  const bool mispred = w.dec_bcast.mispred;
  const bool intr = w.csr2rob.interrupt_req;
  int n = 0;
  bool single = false;
  if (c_count > 0 && !mispred) {
    const Uop &h = cent(c_head).uop;
    const bool h_alone = rob_is_flush_inst(h) || rob_is_fence(h) || rob_is_wfi(h);
    if (intr) {
      single = true;  // the interrupt is taken on the head instruction (it retires with it, as in line mode)
    } else if (h_alone) {
      single = rob_is_complete(h) && !(rob_is_fence(h) && w.lsu2rob.committed_store_pending);
    } else {
      const int lim = front_stall ? 1 : COMMIT_WIDTH;  // BPU update back-pressure: one per cycle (as the line mode's progress path)
      int blocks = 0, ctl = 0;
      const int upd_free = static_cast<int>(w.in.front_upd_free) - c_prev_ctl;
      const int tq_free = static_cast<int>(w.in.front_tq_free) - c_prev_blk;
      for (int k = 0; k < lim && k < c_count; k++) {
        const Uop &u = cent((c_head + k) % ROB_NUM).uop;
        if (!rob_is_complete(u) || rob_is_flush_inst(u) || rob_is_fence(u) || rob_is_wfi(u)) break;
        const InstType ty = rob_type(u);
        const bool is_ctl = ty == BR || ty == JAL || ty == JALR;
        if (k > 0 && is_ctl && ctl >= upd_free) break;               // the oldest one always goes (progress)
        if (k > 0 && u.ftq_is_last && blocks >= tq_free) break;
        blocks += u.ftq_is_last ? 1 : 0;
        ctl += is_ctl ? 1 : 0;
        n++;
      }
      c_prev_ctl_1 = ctl; c_prev_blk_1 = blocks;
    }
  }
  if (single) n = 1;
  if (single || n == 0) { c_prev_ctl_1 = 0; c_prev_blk_1 = 0; }
  if (single) { const Uop &h = cent(c_head).uop; const InstType ty = rob_type(h); c_prev_ctl_1 = (ty == BR || ty == JAL || ty == JALR); c_prev_blk_1 = h.ftq_is_last ? 1 : 0; }
  w.rob2csr.interrupt_resp = single && intr;
  bc.interrupt = single && intr;

  for (int k = 0; k < n; k++) {
    const int idx = (c_head + k) % ROB_NUM;
    check_load_alignment(cent(idx).uop, "DUT: Load address misaligned at commit!");
    w.rob_commit.commit_entry[k].valid = true;
    cent_1(idx).valid = false;
    ctouch(idx);
  }
  if (n > 0) {
    stall_cycle = 0;
    c_head_1 = (c_head + n) % ROB_NUM;
    if (c_head + n >= ROB_NUM) c_hflag_1 = !c_hflag;
    c_count_1 = c_count - n;
  }
  if (single) single_actions(cent(c_head).uop);

  g_msp_tick(n > 0);

  // the dispatch stage allocates from the tail of the registered state (entries freed this cycle are reused next cycle)
  const int tail = (c_head + c_count) % ROB_NUM;
  w.rob2dis.enq_idx = tail;
  w.rob2dis.rob_flag = c_hflag ^ (c_head + c_count >= ROB_NUM);
  w.rob2dis.free_cnt = ROB_NUM - c_count;

  if (++stall_cycle > 50000) {
    std::fprintf(stderr, "ROB deadlock (compact) at cycle %llu: count=%d head=%d n=%d single=%d front_stall=%d\n",
                 (unsigned long long)ctx->perf.cycle, c_count, c_head, n, (int)single, (int)front_stall);
    Assert(0 && "ROB Deadlock detected (stall_cycle > 50000)");
    std::exit(1);
  }
}

// What a single-retired instruction triggers: flush / trap / CSR / fence side effects (shared by both ROB modes).
void Rob::single_actions(const Uop &u) {
  Wires &w = *wires;
  auto &bc = w.rob_bcast;
  const bool flush_inst = rob_is_flush_inst(u), wfi = rob_is_wfi(u), intr = w.rob2csr.interrupt_resp;
  if (flush_inst || intr || wfi) {
    uint32_t pc = 0;
    if (flush_inst || intr) {
      Assert(w.ftq_rob_pc_resp.resp[0].valid);
      pc = w.ftq_rob_pc_resp.resp[0].pc;
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
    else if (t == CSR) w.rob2csr.commit = true;
    else if (t == SFENCE_VMA) bc.fence = true;
    else if (t == FENCE_I) bc.fence_i = true;
    else if (u.flush_pipe) {  // memory-order violator (LSU_MEMSPEC): no CSR/MMU action, refetch from pc+4
      bc.pc = pc;
      long long younger = -1;
      for (int b = 0; b < ROB_BANK_NUM; b++)
        for (int l = 0; l < ROB_LINE_NUM; l++) younger += entry[b][l].valid;
      g_msp_flush_commit(u.rob_idx, younger);
    }
    else Assert(0 && "Rob: unexpected single-commit instruction");
  }
}

// Execution results mark their issue port's completion bit; load/store ports also record the address or the fault.
void Rob::comb_complete() {
  Wires &w = *wires;
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    if (!w.exu2rob.entry[i].valid) continue;
    const Uop &wb = w.exu2rob.entry[i].uop;
    const int bank = get_rob_bank(wb.rob_idx), line = get_rob_line(wb.rob_idx);
    const auto bit = rob_cplt_mask_from_issue_port(i);
    Uop &u = entry_1[bank][line].uop;
    touch(bank, line);
    if (wb.rob_defer) continue;  // speculative load write-back: the completion bit is set by its second write-back
    Assert((u.cplt_mask & bit) == 0 && "ROB: duplicate completion bit set");
    u.cplt_mask |= bit;
    Assert((u.cplt_mask & ~u.expect_mask) == 0 && "ROB: completion bit outside expected mask");
    Assert(rob_cplt_popcount(u.cplt_mask) <= rob_cplt_popcount(u.expect_mask) && "ROB: completion overflow");

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
    u.is_mmio = u.is_mmio || wb.is_mmio;
    if (is_branch_uop(wb.op)) {
      u.diag_val = wb.diag_val;
      u.mispred = wb.mispred;
      u.br_taken = wb.br_taken;
    }
  }
}

// Branch mispredict: pull the tail back to just after the branch and invalidate every younger entry, across lines.
void Rob::comb_branch() {
  const Wires &w = *wires;
  if (!w.dec_bcast.mispred || w.rob_bcast.flush) return;
  const int bank = get_rob_bank(w.dec_bcast.redirect_rob_idx), line = get_rob_line(w.dec_bcast.redirect_rob_idx);
  if (tbhook::enabled) tbhook::on_mispredict((uint64_t)entry[bank][line].uop->dbg.inst_idx);
  if (rob_compact()) {  // the tail moves to the entry right after the branch; everything younger is invalidated
    const int bidx = static_cast<int>(w.dec_bcast.redirect_rob_idx);
    const int keep = (bidx - c_head + ROB_NUM) % ROB_NUM + 1;  // entries up to and including the branch
    Assert(keep <= c_count && "ROB compact: mispredicted branch outside the valid range");
    cent_1(bidx).uop.ftq_is_last = true;
    ctouch(bidx);
    for (int k = keep; k < c_count; k++) {
      const int idx = (c_head + k) % ROB_NUM;
      cent_1(idx).valid = false;
      ctouch(idx);
    }
    c_count_1 = keep;  // commit is blocked in a mispredict cycle, so the head does not move
    return;
  }

  enq_ptr_1 = (line + 1) % ROB_LINE_NUM;
  if (enq_ptr_1 > enq_ptr) enq_flag_1 = !enq_flag;

  // everything after the branch in its fetch block is flushed, so it is now the last instruction of that block
  entry_1[bank][line].uop.ftq_is_last = true;
  touch(bank, line);

  for (int i = bank + 1; i < ROB_BANK_NUM; i++) {
    entry_1[i][line].valid = false;
    touch(i, line);
  }
  for (int ptr = (line + 1) % ROB_LINE_NUM; ptr != enq_ptr; ptr = (ptr + 1) % ROB_LINE_NUM)
    for (int i = 0; i < ROB_BANK_NUM; i++) {
      entry_1[i][ptr].valid = false;
      touch(i, ptr);
    }
}

// Dispatch enqueues into the tail line; the tail advances one line when any slot was written.
void Rob::comb_fire() {
  Wires &w = *wires;
  if (!w.rob2dis.ready) return;
  if (rob_compact()) {  // the fired slots are the first m valid slots; Dispatch numbered them tail, tail+1, ...
    int m = 0;
    for (int i = 0; i < DECODE_WIDTH; i++) {
      if (!w.dis2rob.dis_fire[i]) continue;
      const int idx = (w.rob2dis.enq_idx + m) % ROB_NUM;
      Assert(static_cast<int>(w.dis2rob.uop[i].rob_idx) == idx && "ROB compact: dispatch numbering mismatch");
      auto &e = cent_1(idx);
      e.valid = true;
      e.uop = w.dis2rob.uop[i];
      e.uop.cplt_mask = 0;
      ctouch(idx);
      if (tbhook::enabled) {
        const auto &d = e.uop->dbg;
        tbhook::on_dispatch((uint64_t)d.inst_idx, d.pc, d.instruction, idx);
      }
      m++;
    }
    c_count_1 += m;
    Assert(c_count_1 <= ROB_NUM && "ROB compact: overflow");
    return;
  }
  bool enq = false;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (!w.dis2rob.dis_fire[i]) continue;
    auto &e = entry_1[i][enq_ptr];
    e.valid = true;
    touch(i, enq_ptr);
    e.uop = w.dis2rob.uop[i];
    e.uop.cplt_mask = 0;
    if (tbhook::enabled) {
      const auto &d = e.uop->dbg;
      tbhook::on_dispatch((uint64_t)d.inst_idx, d.pc, d.instruction, make_rob_idx(enq_ptr, i));
    }
    enq = true;
  }
  if (enq) {
    LOOP_INC(enq_ptr_1, ROB_LINE_NUM);
    if (enq_ptr_1 == 0) enq_flag_1 = !enq_flag;
  }
}

// A global flush empties the ROB and resets the pointers (overrides any recovery or enqueue above).
void Rob::comb_flush() {
  if (!wires->rob_bcast.flush) return;
  for (int i = 0; i < ROB_BANK_NUM; i++)
    for (int j = 0; j < ROB_LINE_NUM; j++) entry_1[i][j].valid = false;
  touch_all();
  enq_ptr_1 = deq_ptr_1 = 0;
  enq_flag_1 = deq_flag_1 = false;
  c_head_1 = c_count_1 = 0;
  c_hflag_1 = false;
}

void Rob::seq() {
  if (dirty_all_) {
    for (int i = 0; i < ROB_BANK_NUM; i++)
      for (int j = 0; j < ROB_LINE_NUM; j++) entry[i][j] = entry_1[i][j];
  } else {
    for (int w = 0; w < kDirtyWords; w++)
      for (uint64_t m = dirty_[w]; m; m &= m - 1) {
        const int k = w * 64 + __builtin_ctzll(m);
        entry[k / ROB_LINE_NUM][k % ROB_LINE_NUM] = entry_1[k / ROB_LINE_NUM][k % ROB_LINE_NUM];
      }
  }
  std::memset(dirty_, 0, sizeof(dirty_));
  dirty_all_ = false;
  deq_ptr = deq_ptr_1;
  enq_ptr = enq_ptr_1;
  enq_flag = enq_flag_1;
  deq_flag = deq_flag_1;
  c_head = c_head_1; c_count = c_count_1; c_hflag = c_hflag_1;
  c_prev_ctl = c_prev_ctl_1; c_prev_blk = c_prev_blk_1;
}

// PMU (EXP-013): same "oldest incomplete instruction of the head line" as comb_ready(); also the number of valid entries.
bool Rob::pmu_head_incomplete(Uop &out, int &occupied_entries) const {
  occupied_entries = 0;
  for (int l = 0; l < ROB_LINE_NUM; l++)
    for (int i = 0; i < ROB_BANK_NUM; i++) occupied_entries += entry[i][l].valid ? 1 : 0;
  if (rob_compact()) {
    for (int k = 0; k < ROB_BANK_NUM && k < c_count; k++) {
      const auto &e = entry[((c_head + k) % ROB_NUM) % ROB_BANK_NUM][((c_head + k) % ROB_NUM) / ROB_BANK_NUM];
      if (!rob_is_complete(e.uop)) { out = e.uop; return true; }
    }
    return false;
  }
  for (int i = 0; i < ROB_BANK_NUM; i++) {
    const auto &e = entry[i][deq_ptr];
    if (e.valid && !rob_is_complete(e.uop)) {
      out = e.uop;
      return true;
    }
  }
  return false;
}
