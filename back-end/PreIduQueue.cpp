#include "Wires.h"
#include "PreIduQueue.h"
#include "types.h"

#include <cstring>
#include <cstdlib>

// Instructions accepted from the front end this cycle (comb_accept_front), pushed into the buffer in comb_fire.
static InstructionBufferEntry push_entries[BACKEND_FETCH_WIDTH];
static int push_count = 0;
// With an empty instruction buffer the accepted block goes straight to decode (buffer bypass)
static bool g_bypass = false;

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
// instruction commits, trimmed at the tail on a mispredict. All of it works on the *_1 copies.
int PreIduQueue::ftq_alloc() {
  if (d.ftq_count >= FTQ_SIZE) return -1;
  const int idx = d.ftq_tail;
  d.ftq_tail = (d.ftq_tail + 1) % FTQ_SIZE;
  d.ftq_count++;
  return idx;
}

void PreIduQueue::ftq_pop(int pop_cnt) {
  for (int i = 0; i < pop_cnt && d.ftq_count > 0; i++) {
    d.ftq_valid[d.ftq_head] = false;
    d.ftq_head = (d.ftq_head + 1) % FTQ_SIZE;
    d.ftq_count--;
    ftq_dirty = true;
  }
}

// Drop every block younger than the one holding the mispredicted branch.
void PreIduQueue::ftq_recover(int new_tail) {
  const int tail = ((new_tail % FTQ_SIZE) + FTQ_SIZE) % FTQ_SIZE;
  const int discarded = (d.ftq_tail - tail + FTQ_SIZE) % FTQ_SIZE;
  for (int i = 0; i < discarded; i++) d.ftq_valid[(tail + i) % FTQ_SIZE] = false;
  if (discarded > 0) ftq_dirty = true;
  d.ftq_tail = tail;
  d.ftq_count -= discarded;
}

void PreIduQueue::ftq_flush() {
  d.ftq_head = d.ftq_tail = d.ftq_count = 0;
  for (int i = 0; i < FTQ_SIZE; i++) d.ftq_valid[i] = false;
  ftq_dirty = true;
}

void PreIduQueue::init() {
  q_.ibuf.init();
  d.ibuf.init();
  q_.ftq_head = q_.ftq_tail = q_.ftq_count = 0;
  d.ftq_head = d.ftq_tail = d.ftq_count = 0;
  push_count = 0;
  for (int i = 0; i < FTQ_SIZE; i++) {
    q_.ftq_lookup_entries[i] = d.ftq_lookup_entries[i] = FTQEntry();
    q_.ftq_train_meta_fifo[i] = d.ftq_train_meta_fifo[i] = FTQTrainMetaEntry();
    q_.ftq_valid[i] = d.ftq_valid[i] = false;
  }
}

// Publish the oldest DECODE_WIDTH buffered instructions to decode.
void PreIduQueue::begin_cycle() {
  const Regs &q = q_;
  d.ftq_head = q.ftq_head;
  d.ftq_tail = q.ftq_tail;
  d.ftq_count = q.ftq_count;

  const int n = q.ibuf.count() < DECODE_WIDTH ? q.ibuf.count() : DECODE_WIDTH;
  for (int i = 0; i < DECODE_WIDTH; i++) wires->pre_issue.entries[i] = i < n ? q.ibuf.peek(i) : InstructionBufferEntry{};

  wires->pre2front.ready = false;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) wires->pre2front.fire[i] = false;
  push_count = 0;
  g_bypass = false;
}

// Accept one fetch block per cycle if the FTQ and the instruction buffer have room for all its instructions; the block
// gets an FTQ entry and each instruction a buffer entry (and a uop-pool block).
void PreIduQueue::comb_accept_front() {
  const Regs &q = q_;
  begin_cycle();   // the first phase of the cycle: default outputs, working copies (x_1 = x)
  if (wires->idu_br_latch->mispred || q.flush_q) return;  // the cycle after a mispredict / flush: ready stays false

  int incoming = 0;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) incoming += wires->in.valid[i] ? 1 : 0;

  wires->pre2front.ready = q.ftq_count < FTQ_SIZE && q.ibuf.can_accept(incoming);
  if (!wires->pre2front.ready || incoming == 0) return;

  const int ftq_idx = ftq_alloc();
  Assert(ftq_idx >= 0 && "PreIduQueue: ready with a full FTQ");  // ready required a free slot and nothing allocated yet
  FTQEntry &block = d.ftq_lookup_entries[ftq_idx];
  block = FTQEntry();
  block.next_pc = wires->in.predict_next_fetch_address[0];
  block.nslots = 0;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) {
    block.slot_pc[i] = wires->in.pc[i];
    block.slot_rvc[i] = wires->in.is_rvc[i];
    block.pred_taken_mask[i] = wires->in.predict_dir[i];
    if (wires->in.valid[i]) block.nslots = i + 1;
  }
  d.ftq_train_meta_fifo[ftq_idx] = FTQTrainMetaEntry();
  d.ftq_train_meta_fifo[ftq_idx].meta = wires->in.meta;
  d.ftq_valid[ftq_idx] = true;
  ftq_alloc_mask |= 1ull << ftq_idx;
  ftq_dirty = true;

  int last = -1;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) {
    wires->pre2front.fire[i] = wires->in.valid[i];
    if (wires->in.valid[i]) last = i;
  }
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) {
    if (!wires->in.valid[i]) continue;
    InstructionBufferEntry &e = push_entries[push_count++];
    e.valid = true;
    e.inst = wires->in.inst[i];
    e.pc = wires->in.pc[i];
    e.is_rvc = wires->in.is_rvc[i];
    e.page_fault_inst = wires->in.page_fault_inst[i];
    e.ftq_idx = ftq_idx;
    e.ftq_offset = i;
    e.ftq_is_last = (i == last);
    e.uop_id = uop_alloc();
  }
  if (q.ibuf.count() == 0 && push_count > 0) {
    g_bypass = true;
    for (int i = 0; i < DECODE_WIDTH; i++) wires->pre_issue.entries[i] = i < push_count ? push_entries[i] : InstructionBufferEntry{};
  }
}

// Instruction buffer: decode consumes from the front (a leading run of fired slots), the accepted block is appended;
// flush and mispredict empty it (and the FTQ: everything, or what is younger than the branch's block). Committed blocks
// leave the FTQ head.
void PreIduQueue::comb_fire() {
  const bool flush = wires->rob_bcast.flush, mispred = wires->idu_br_latch->mispred;
  d.flush_q = flush;   // the next cycle sees this cycle's ROB flush (accept_front stays closed)
  if (flush) {
    ftq_flush();
  } else if (mispred) {
    ftq_recover((wires->idu_br_latch->ftq_idx + 1) % FTQ_SIZE);
  }

  if (flush || mispred) {
    d.ibuf.clear();
    ibuf_dirty = true;
    return;
  }

  int pop_count = 0;
  while (pop_count < DECODE_WIDTH && wires->idu_consume.fire[pop_count]) pop_count++;
  int first_push = 0;
  if (g_bypass) {  // the first pop_count pushed entries were consumed by decode in the cycle they arrived
    first_push = pop_count < push_count ? pop_count : push_count;
    pop_count = 0;
    g_bypass = false;
  }
  if (pop_count > 0) d.ibuf.pop_front(pop_count);
  for (int i = first_push; i < push_count; i++) d.ibuf.push_back(push_entries[i]);
  if (pop_count > 0 || push_count > first_push) ibuf_dirty = true;

  int committed_blocks = 0;
  for (int i = 0; i < COMMIT_WIDTH; i++)
    if (wires->rob_commit.commit_entry[i].valid && wires->rob_commit.commit_entry[i].uop.ftq_is_last) committed_blocks++;
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
    auto &resp = wires->ftq_commit_info.resp[i];
    const auto &commit = wires->rob_commit.commit_entry[i];
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
      for (int k = 0; k < 4; k++) {
        resp.tage_idx[k] = m.tage_idx[off][k];
        resp.tage_tag[k] = m.tage_tag[off][k];
      }
    }

    if (commit.uop.ftq_is_last && cursor_valid) {
      const uint32_t next = (cursor + 1) % FTQ_SIZE;
      if (next == static_cast<uint32_t>(q.ftq_tail)) cursor_valid = false;
      else cursor = next;
    }
  }
}

void PreIduQueue::seq() {
  q_.flush_q = d.flush_q;
  if (ibuf_dirty) {
    q_.ibuf.sync_from(d.ibuf);
    ibuf_dirty = false;
  }
  q_.ftq_head = d.ftq_head;
  q_.ftq_tail = d.ftq_tail;
  q_.ftq_count = d.ftq_count;
  if (ftq_dirty) {
    // Entry payloads only change at the allocated slot; valid bits are tiny.
    for (uint64_t m = ftq_alloc_mask; m; m &= m - 1) {
      const int i = __builtin_ctzll(m);
      q_.ftq_lookup_entries[i] = d.ftq_lookup_entries[i];
      q_.ftq_train_meta_fifo[i] = d.ftq_train_meta_fifo[i];
    }
    ftq_alloc_mask = 0;
    std::memcpy(q_.ftq_valid, d.ftq_valid, sizeof(q_.ftq_valid));
    ftq_dirty = false;
  }
}
