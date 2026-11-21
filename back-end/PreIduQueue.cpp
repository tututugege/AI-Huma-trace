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

// FTQ: ring of fetch blocks, allocated at the tail when a block is accepted, freed at the head when its last
// instruction commits, trimmed at the tail on a mispredict (the clock edge: the registers in place).

void PreIduQueue::ftq_pop(int pop_cnt) {
  for (int i = 0; i < pop_cnt && q_.ftq_count > 0; i++) {
    q_.ftq_valid[q_.ftq_head] = false;
    q_.ftq_head = (q_.ftq_head + 1) % FTQ_SIZE;
    q_.ftq_count--;
  }
}

// Drop every block younger than the one holding the mispredicted branch.
void PreIduQueue::ftq_recover(int new_tail) {
  const int tail = ((new_tail % FTQ_SIZE) + FTQ_SIZE) % FTQ_SIZE;
  const int discarded = (q_.ftq_tail - tail + FTQ_SIZE) % FTQ_SIZE;
  for (int i = 0; i < discarded; i++) q_.ftq_valid[(tail + i) % FTQ_SIZE] = false;
  q_.ftq_tail = tail;
  q_.ftq_count -= discarded;
}

void PreIduQueue::ftq_flush() {
  q_.ftq_head = q_.ftq_tail = q_.ftq_count = 0;
  for (int i = 0; i < FTQ_SIZE; i++) q_.ftq_valid[i] = false;
}

void PreIduQueue::init() {
  q_.ibuf.init();
  q_.ftq_head = q_.ftq_tail = q_.ftq_count = 0;
  push_count = 0;
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
  accept_ = false;
}

// Accept one fetch block per cycle if the FTQ and the instruction buffer have room for all its instructions; the block
// gets an FTQ entry and each instruction a buffer entry (and a uop-pool block).
void PreIduQueue::comb_accept_front() {
  const Regs &q = q_;
  begin_cycle();   // the first phase of the cycle: default outputs
  if (in.idu_br_latch->mispred || q.flush_q) return;  // the cycle after a mispredict / flush: ready stays false

  int incoming = 0;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) incoming += in.top_in->valid[i] ? 1 : 0;

  out.pre2front.ready = q.ftq_count < FTQ_SIZE && q.ibuf.can_accept(incoming);
  if (!out.pre2front.ready || incoming == 0) return;

  const int ftq_idx = q.ftq_tail;   // the block's FTQ entry (written at the edge)
  accept_ = true;
  FTQEntry &block = accept_block_;
  block = FTQEntry();
  block.next_pc = in.top_in->predict_next_fetch_address[0];
  block.nslots = 0;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) {
    block.slot_pc[i] = in.top_in->pc[i];
    block.slot_rvc[i] = in.top_in->is_rvc[i];
    block.pred_taken_mask[i] = in.top_in->predict_dir[i];
    if (in.top_in->valid[i]) block.nslots = i + 1;
  }
  accept_meta_ = in.top_in->meta;

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

// The clock edge. The accepted block takes the FTQ tail. Instruction buffer: decode consumes from the front (a leading
// run of fired slots), the accepted block is appended; flush and mispredict empty it (and the FTQ: everything, or what
// is younger than the branch's block). Committed blocks leave the FTQ head.
void PreIduQueue::seq() {
  if (accept_) {
    const int idx = q_.ftq_tail;
    q_.ftq_tail = (q_.ftq_tail + 1) % FTQ_SIZE;
    q_.ftq_count++;
    q_.ftq_lookup_entries[idx] = accept_block_;
    q_.ftq_train_meta_fifo[idx] = FTQTrainMetaEntry();
    q_.ftq_train_meta_fifo[idx].meta = accept_meta_;
    q_.ftq_valid[idx] = true;
  }
  const bool flush = in.rob_bcast->flush, mispred = in.idu_br_latch->mispred;
  q_.flush_q = flush;   // the next cycle sees this cycle's ROB flush (accept_front stays closed)
  if (flush) {
    ftq_flush();
  } else if (mispred) {
    ftq_recover((in.idu_br_latch->ftq_idx + 1) % FTQ_SIZE);
  }

  if (flush || mispred) {
    q_.ibuf.clear();
    return;
  }

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

  int committed_blocks = 0;
  for (int i = 0; i < COMMIT_WIDTH; i++)
    if (in.rob_commit->commit_entry[i].valid && in.rob_commit->commit_entry[i].uop.ftq_is_last) committed_blocks++;
  ftq_pop(committed_blocks);
}



// What the predictor needs to train on each committing instruction: the fetched PC / direction and the prediction
// metadata of its block. Blocks commit in order, so the metadata cursor starts at the FTQ head and advances after the
// last instruction of a block.
void PreIduQueue::comb_commit_info() {
  const Regs &q = q_;
  uint32_t cursor = static_cast<uint32_t>(q.ftq_head);
  bool cursor_valid = q.ftq_count > 0;

  for (int i = 0; i < COMMIT_WIDTH; i++) {
    auto &resp = out.ftq_commit_info.resp[i];
    const auto &commit = in.rob_commit->commit_entry[i];
    resp = {};
    if (!commit.valid) continue;

    const uint32_t idx = commit.uop->ftq_idx, off = commit.uop->ftq_offset;
    if (idx < FTQ_SIZE && q.ftq_valid[idx]) {
      resp.pc = q.ftq_lookup_entries[idx].slot_pc[off];
      resp.is_rvc = q.ftq_lookup_entries[idx].slot_rvc[off];
      resp.pred_taken = q.ftq_lookup_entries[idx].pred_taken_mask[off];
    }

    if (cursor_valid) {
      Assert(idx == cursor);
      const auto &m = q.ftq_train_meta_fifo[cursor].meta;
      resp.alt_pred = m.alt_pred[off];
      resp.pcpn = m.pcpn[off];
      resp.ps_info = m.ps_info[off];
      resp.blk_id = m.blk_id[off];
      resp.ftq_is_last = commit.uop.ftq_is_last;
    }

    if (commit.uop.ftq_is_last && cursor_valid) {
      const uint32_t next = (cursor + 1) % FTQ_SIZE;
      if (next == static_cast<uint32_t>(q.ftq_tail)) cursor_valid = false;
      else cursor = next;
    }
  }
}

