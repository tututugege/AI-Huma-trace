#include "BackPorts.h"
#include "PreIduQueue.h"
#include "types.h"

#include <cstring>
#include <cstdlib>


FtqPc PreIduQueue::ftq_pc(uint32_t idx, uint32_t off) const {
  const Regs &q = q_;
  const FTQEntry &entry = q.ftq_lookup_entries[idx];
  FtqPc r;
  r.pc = entry.slot_pc[off];
  r.is_rvc = entry.slot_rvc[off];
  r.pred_taken = entry.pred_taken_mask[off];
  r.next_pc = entry.next_pc;
  // FE_PAIR: a taken branch followed by a slot of a different fetch block (the group continues at its target)
  if (entry.pred_taken_mask[off] && off + 1 < entry.nslots && entry.slot_pc[off + 1] != entry.slot_pc[off] + ilen_of(entry.slot_rvc[off]))
    r.next_pc = entry.slot_pc[off + 1];
  return r;
}

// Unified FTQ-64: single 64-entry ring shared by BPU S0 (bpu_ptr), ICache (ifu_ptr),
// BPU S2 (s2_ptr), Predecode (pd_ptr), and Commit (comm_ptr).

// Drop every block younger than the one holding the mispredicted branch.
void PreIduQueue::ftq_recover(int new_tail) {
  const uint32_t tail = static_cast<uint32_t>(new_tail) & (FTQ_SIZE - 1);
  const uint32_t discarded = (q_.bpu_ptr - tail) & (FTQ_SIZE - 1);
  for (uint32_t i = 0; i < discarded; i++) {
    const uint32_t idx = (tail + i) & (FTQ_SIZE - 1);
    q_.ftq_valid[idx] = false;
    q_.ftq_lookup_entries[idx].empty_blk = false;
  }
  q_.pd_ptr = q_.s2_ptr = q_.ifu_ptr = q_.bpu_ptr = tail;
  q_.fresh_v = false;
}

void PreIduQueue::ftq_flush() {
  q_.comm_ptr = q_.pd_ptr = q_.s2_ptr = q_.ifu_ptr = q_.bpu_ptr = 0;
  q_.fresh_v = false;
  q_.fresh_slot = 0;
  for (int i = 0; i < FTQ_SIZE; i++) {
    q_.ftq_valid[i] = false;
    q_.ftq_lookup_entries[i].empty_blk = false;
  }
}

void PreIduQueue::init() {
  q_.ibuf.init();
  q_.flush_q = false;
  q_.comm_ptr = q_.pd_ptr = q_.s2_ptr = q_.ifu_ptr = q_.bpu_ptr = 0;
  q_.fresh_v = false;
  q_.fresh_slot = 0;
  push_count = 0;
  push_bypass = false;
  for (int i = 0; i < FTQ_SIZE; i++) {
    q_.ftq_lookup_entries[i] = FTQEntry();
    q_.ftq_train_meta_fifo[i] = FTQTrainMetaEntry();
    q_.ftq_valid[i] = false;
  }
}

// Publish the oldest DECODE_WIDTH buffered instructions to decode.
void PreIduQueue::begin_cycle() {
  const Regs &q = q_;

  const int n = q.ibuf.count() < DECODE_WIDTH ? q.ibuf.count() : DECODE_WIDTH;
  for (int i = 0; i < DECODE_WIDTH; i++) out.pre_issue.entries[i] = i < n ? q.ibuf.peek(i) : InstructionBufferEntry{};

  out.pre2front.ready = false;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) out.pre2front.fire[i] = false;
  push_count = 0;
  push_bypass = false;
}

// Accept one checked fetch block per cycle if the instruction buffer has room for all its instructions;
// its FTQ entry already lives at in.top_in->ftq_idx (allocated at BPU S0, filled at S2, updated at Predecode).
void PreIduQueue::comb_accept_front() {
  const Regs &q = q_;
  begin_cycle();   // the first phase of the cycle: default outputs
  if (in.idu_br_latch->mispred || q.flush_q) return;  // the cycle after a mispredict / flush: ready stays false

  int incoming = 0;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) incoming += in.top_in->valid[i] ? 1 : 0;

  out.pre2front.ready = q.ibuf.can_accept(incoming);
  if (!out.pre2front.ready || incoming == 0) return;

  const uint32_t ftq_idx = in.top_in->ftq_idx;

  int last = -1;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) {
    out.pre2front.fire[i] = in.top_in->valid[i];
    if (in.top_in->valid[i]) last = i;
  }
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) {
    if (!in.top_in->valid[i]) continue;
    InstructionBufferEntry &e = push_entries[push_count++];
    e.valid = true;
    e.inst = in.top_in->inst[i];
    e.pc = in.top_in->pc[i];
    e.is_rvc = in.top_in->is_rvc[i];
    e.page_fault_inst = in.top_in->page_fault_inst[i];
    e.pf_inst_hi = in.top_in->pf_inst_hi[i];
    e.ftq_idx = ftq_idx;
    e.ftq_offset = i;
    e.ftq_is_last = (i == last);
    e.uop_id = uop_alloc();
  }
  if (q.ibuf.count() == 0 && push_count > 0) {
    push_bypass = true;
    for (int i = 0; i < DECODE_WIDTH; i++) out.pre_issue.entries[i] = i < push_count ? push_entries[i] : InstructionBufferEntry{};
  }
}

// The clock edge: update Unified FTQ-64 pointers/arrays (flush > mispred > S0/IFU/S2/PD/Commit) and ibuf.
void PreIduQueue::seq() {
  const bool flush = in.rob_bcast->flush, mispred = in.idu_br_latch->mispred;
  q_.flush_q = flush;   // the next cycle sees this cycle's ROB flush (accept_front stays closed)
  if (flush) {
    ftq_flush();
    q_.ibuf.clear();
    return;
  }
  if (mispred) {
    ftq_recover((in.idu_br_latch->ftq_idx + 1) & (FTQ_SIZE - 1));
    q_.ibuf.clear();
    return;
  }

  // ---- 1. S0 allocate / S2-override tail drop / ICache issue pop ----
  const FtqFrontS0Req &s0 = in.top_in->ftq_s0;
  if (s0.drop_tail_n > 0) {
    const uint32_t avail = (q_.bpu_ptr - q_.ifu_ptr) & (FTQ_SIZE - 1);
    const uint32_t n = s0.drop_tail_n < avail ? s0.drop_tail_n : avail;
    for (uint32_t i = 0; i < n; i++) {
      const uint32_t idx = (q_.bpu_ptr - 1 - i) & (FTQ_SIZE - 1);
      q_.ftq_valid[idx] = false;
      q_.ftq_lookup_entries[idx].empty_blk = false;
    }
    q_.bpu_ptr = (q_.bpu_ptr - n) & (FTQ_SIZE - 1);
  }
  if (s0.wr_en) {
    Assert(((q_.bpu_ptr - q_.comm_ptr) & (FTQ_SIZE - 1)) < (FTQ_SIZE - 1) && "Unified FTQ-64 overflow");
    const uint32_t idx = q_.bpu_ptr;
    FTQEntry &e = q_.ftq_lookup_entries[idx];
    e = FTQEntry();
    e.fetch_addr = s0.addr;
    e.fetch_pair = s0.pair;
    e.s2_is_pair = s0.pair.valid;
    e.empty_blk = false;
    q_.ftq_valid[idx] = true;
    q_.fresh_v = true;
    q_.fresh_slot = idx;
    q_.bpu_ptr = (idx + 1) & (FTQ_SIZE - 1);
  } else {
    q_.fresh_v = false;
  }
  if (s0.ifu_pop) {
    Assert(q_.ifu_ptr != q_.bpu_ptr && "Unified FTQ-64 ifu_ptr underflow");
    q_.ifu_ptr = (q_.ifu_ptr + 1) & (FTQ_SIZE - 1);
  }

  // ---- 2. BPU S2 fill (writes prediction + ftq_meta once at s2_ptr) ----
  const FtqFrontS2Req &s2 = in.top_in->ftq_s2;
  if (s2.wr_en) {
    const uint32_t idx = q_.s2_ptr;
    FTQEntry &e = q_.ftq_lookup_entries[idx];
    e.s2_is_pair = s2.is_pair;
    e.next_pc = s2.next_pc;
    for (int i = 0; i < FETCH_WIDTH; i++) {
      e.slot_pc[i] = s2.slot_pc[i];
      e.pred_taken_mask[i] = s2.pred_dir[i];
    }
    q_.ftq_train_meta_fifo[idx] = FTQTrainMetaEntry();
    q_.ftq_train_meta_fifo[idx].meta.assign_prefix(s2.meta);
    q_.ftq_train_meta_fifo[idx].meta.mask();
    q_.s2_ptr = (idx + 1) & (FTQ_SIZE - 1);
  }

  // ---- 3. Predecode in-place update at pd_ptr (+ frontend-only rollback on predecode flush) ----
  const FtqFrontPdReq &pd = in.top_in->ftq_pd;
  if (pd.fire) {
    const uint32_t idx = q_.pd_ptr;
    FTQEntry &e = q_.ftq_lookup_entries[idx];
    e.next_pc = pd.next_pc;
    e.nslots = pd.nslots;
    e.empty_blk = pd.empty_blk;
    for (int i = 0; i < FETCH_WIDTH; i++) {
      e.slot_pc[i] = pd.slot_pc[i];
      e.slot_rvc[i] = pd.is_rvc[i];
      e.pred_taken_mask[i] = pd.pred_dir[i];
    }
    q_.pd_ptr = (idx + 1) & (FTQ_SIZE - 1);
    if (pd.flush) {
      // Frontend-only rollback: preserve entries < pd_ptr (already in ibuf / backend), squash >= pd_ptr
      const uint32_t disc = (q_.bpu_ptr - q_.pd_ptr) & (FTQ_SIZE - 1);
      for (uint32_t i = 0; i < disc; i++) {
        const uint32_t d = (q_.pd_ptr + i) & (FTQ_SIZE - 1);
        q_.ftq_valid[d] = false;
        q_.ftq_lookup_entries[d].empty_blk = false;
      }
      q_.bpu_ptr = q_.ifu_ptr = q_.s2_ptr = q_.pd_ptr;
      q_.fresh_v = false;
    }
  }

  // ---- 4. InstructionBuffer pop / push ----
  int pop_count = 0;
  while (pop_count < DECODE_WIDTH && in.idu_consume->fire[pop_count]) pop_count++;
  int first_push = 0;
  if (push_bypass) {  // the first pop_count pushed entries were consumed by decode in the cycle they arrived
    first_push = pop_count < push_count ? pop_count : push_count;
    pop_count = 0;
    push_bypass = false;
  }
  if (pop_count > 0) q_.ibuf.pop_front(pop_count);
  for (int i = first_push; i < push_count; i++) q_.ibuf.push_back(push_entries[i]);

  // ---- 5. ROB commit retire (comm_ptr jumps past highest committed ftq_is_last + skips leading empty_blk) ----
  int last_cm = -1;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    if (in.rob_commit->commit_entry[i].valid && in.rob_commit->commit_entry[i].uop.ftq_is_last) last_cm = i;
  }
  uint32_t next_comm = q_.comm_ptr;
  if (last_cm >= 0) {
    next_comm = (in.rob_commit->commit_entry[last_cm].uop->ftq_idx + 1) & (FTQ_SIZE - 1);
  }
  while (q_.comm_ptr != next_comm) {
    q_.ftq_valid[q_.comm_ptr] = false;
    q_.ftq_lookup_entries[q_.comm_ptr].empty_blk = false;
    q_.comm_ptr = (q_.comm_ptr + 1) & (FTQ_SIZE - 1);
  }
  // Skip at most one leading empty_blk (two consecutive empty straddlers cannot exist)
  if (q_.comm_ptr != q_.pd_ptr && q_.ftq_lookup_entries[q_.comm_ptr].empty_blk) {
    q_.ftq_valid[q_.comm_ptr] = false;
    q_.ftq_lookup_entries[q_.comm_ptr].empty_blk = false;
    q_.comm_ptr = (q_.comm_ptr + 1) & (FTQ_SIZE - 1);
  }
  Assert(!(q_.comm_ptr != q_.pd_ptr && q_.ftq_lookup_entries[q_.comm_ptr].empty_blk) &&
         "Two consecutive empty straddler FTQ blocks");
}

// What the predictor needs to train on each committing instruction: the fetched PC / direction and the prediction
// metadata of its block (guaranteed within 8-bank window [comm_ptr .. comm_ptr + 7]).
void PreIduQueue::comb_commit_info() {
  const Regs &q = q_;

  for (int i = 0; i < COMMIT_WIDTH; i++) {
    auto &resp = out.ftq_commit_info.resp[i];
    const auto &commit = in.rob_commit->commit_entry[i];
    resp = {};
    if (!commit.valid) continue;

    const uint32_t idx = commit.uop->ftq_idx, off = commit.uop->ftq_offset;
    Assert(((idx - q.comm_ptr) & (FTQ_SIZE - 1)) < COMMIT_WIDTH && "commit ftq_idx outside 8-bank window");
    if (idx < FTQ_SIZE && q.ftq_valid[idx]) {
      resp.pc = q.ftq_lookup_entries[idx].slot_pc[off];
      resp.is_rvc = q.ftq_lookup_entries[idx].slot_rvc[off];
      resp.pred_taken = q.ftq_lookup_entries[idx].pred_taken_mask[off];
      const auto &m = q.ftq_train_meta_fifo[idx].meta;
      resp.alt_pred = m.alt_pred[off];
      resp.pcpn = m.pcpn[off];
      resp.ps_info = m.ps_info[off];
      resp.blk_id = m.blk_id[off];
      resp.ftq_is_last = commit.uop.ftq_is_last;
    }
  }
}

