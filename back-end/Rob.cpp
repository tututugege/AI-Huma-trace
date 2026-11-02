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

namespace {
inline bool rob_is_store(const Uop &u) { return u.tma.mem_commit_is_store; }
inline bool rob_is_load(const Uop &u) { return u.tma.mem_commit_is_load; }
inline bool rob_is_page_fault(const Uop &u) { return u.page_fault_inst || u.page_fault_load || u.page_fault_store; }
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
}

// Working pointers start from the registered ones; entry_1 already equals entry (seq() copies back the touched slots).
void Rob::comb_begin() {
  enq_ptr_1 = enq_ptr;
  deq_ptr_1 = deq_ptr;
  enq_flag_1 = enq_flag;
  deq_flag_1 = deq_flag;
}

// Dispatch/CSR handshake: ready/empty, stall when a flush-type instruction heads the ROB, and the top-down
// attribution of the oldest incomplete instruction (core bound vs memory bound vs cache miss).
void Rob::comb_ready() {
  Wires &w = *wires;
  w.rob2dis.stall = false;
  w.rob2csr.commit = false;
  w.rob2csr.interrupt_resp = false;

  bool found = false, mem = false, miss = false;
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
  Wires &w = *wires;
  auto &bc = w.rob_bcast;
  bc.flush = bc.exception = bc.mret = bc.sret = bc.ecall = bc.fence = bc.fence_i = false;
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

    // what a single-retired instruction triggers
    const bool flush_inst = rob_is_flush_inst(u), wfi = rob_is_wfi(u), intr = w.rob2csr.interrupt_resp;
    if (flush_inst || intr || wfi) {
      uint32_t pc = 0;
      if (flush_inst || intr) {
        Assert(w.ftq_rob_pc_resp.resp[0].valid);
        pc = w.ftq_rob_pc_resp.resp[0].pc;
        bc.flush = true;
        bc.exception = rob_is_exception(u) || intr;
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
      else if (t == EBREAK) ctx->exit_reason = ExitReason::EBREAK;
      else if (wfi) { /* retires without leaving the simulator */ }
      else if (t == CSR) w.rob2csr.commit = true;
      else if (t == SFENCE_VMA) bc.fence = true;
      else if (t == FENCE_I) bc.fence_i = true;
      else if (u.flush_pipe) bc.pc = pc;  // MMIO-triggered flush: no CSR/MMU action
      else Assert(0 && "Rob: unexpected single-commit instruction");
    }
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
}
