#include "Wires.h"
#include "BeginCheck.h"
#include "PreIduQueue.h"
#include "types.h"

#include <cstring>
#include <cstdlib>

// Instructions accepted from the front end this cycle (comb_accept_front), pushed into the buffer in comb_fire.
static InstructionBufferEntry push_entries[BACKEND_FETCH_WIDTH];
static int push_count = 0;
// FE_RFAST bit3: with an empty instruction buffer the accepted block goes straight to decode (buffer bypass)
static bool g_bypass = false;
static const bool g_bypass_on = ((std::getenv("FE_RFAST") ? std::atoi(std::getenv("FE_RFAST")) : CFG_FE_RFAST) & 8) != 0;

static void fill_ftq_pc_resp(FtqPcReadResp &resp, const FTQEntry &entry, wire<1> entry_valid, const FtqPcReadReq &req) {
  resp = {};
  if (!req.valid) return;
  resp.valid = true;
  resp.entry_valid = entry_valid;
  resp.pc = entry.slot_pc[req.ftq_offset];
  resp.pred_taken = entry.pred_taken_mask[req.ftq_offset];
  resp.next_pc = entry.next_pc;
  // FE_PAIR: a taken branch followed by a slot of a different fetch block (the group continues at its target)
  if (entry.pred_taken_mask[req.ftq_offset] && req.ftq_offset + 1 < entry.nslots && entry.slot_pc[req.ftq_offset + 1] != entry.slot_pc[req.ftq_offset] + 4)
    resp.next_pc = entry.slot_pc[req.ftq_offset + 1];
}

// FTQ: ring of fetch blocks, allocated at the tail when a block is accepted, freed at the head when its last
// instruction commits, trimmed at the tail on a mispredict. All of it works on the *_1 copies.
int PreIduQueue::ftq_alloc() {
  if (ftq_count_1 >= FTQ_SIZE) return -1;
  const int idx = ftq_tail_1;
  ftq_tail_1 = (ftq_tail_1 + 1) % FTQ_SIZE;
  ftq_count_1++;
  return idx;
}

void PreIduQueue::ftq_pop(int pop_cnt) {
  for (int i = 0; i < pop_cnt && ftq_count_1 > 0; i++) {
    ftq_valid_1[ftq_head_1] = false;
    ftq_head_1 = (ftq_head_1 + 1) % FTQ_SIZE;
    ftq_count_1--;
    ftq_dirty = true;
  }
}

// Drop every block younger than the one holding the mispredicted branch.
void PreIduQueue::ftq_recover(int new_tail) {
  const int tail = ((new_tail % FTQ_SIZE) + FTQ_SIZE) % FTQ_SIZE;
  const int discarded = (ftq_tail_1 - tail + FTQ_SIZE) % FTQ_SIZE;
  for (int i = 0; i < discarded; i++) ftq_valid_1[(tail + i) % FTQ_SIZE] = false;
  if (discarded > 0) ftq_dirty = true;
  ftq_tail_1 = tail;
  ftq_count_1 -= discarded;
}

void PreIduQueue::ftq_flush() {
  ftq_head_1 = ftq_tail_1 = ftq_count_1 = 0;
  for (int i = 0; i < FTQ_SIZE; i++) ftq_valid_1[i] = false;
  ftq_dirty = true;
}

void PreIduQueue::init() {
  ibuf.init();
  ibuf_1.init();
  ftq_head = ftq_tail = ftq_count = 0;
  ftq_head_1 = ftq_tail_1 = ftq_count_1 = 0;
  push_count = 0;
  for (int i = 0; i < FTQ_SIZE; i++) {
    ftq_lookup_entries[i] = ftq_lookup_entries_1[i] = FTQEntry();
    ftq_train_meta_fifo[i] = ftq_train_meta_fifo_1[i] = FTQTrainMetaEntry();
    ftq_valid[i] = ftq_valid_1[i] = false;
  }
}

// Publish the oldest DECODE_WIDTH buffered instructions to decode.
void PreIduQueue::comb_begin() {
  BEGIN_EQ(ftq_lookup_entries_1, ftq_lookup_entries);
  BEGIN_EQ(ftq_train_meta_fifo_1, ftq_train_meta_fifo);
  BEGIN_EQ(ftq_valid_1, ftq_valid);
  ftq_head_1 = ftq_head;
  ftq_tail_1 = ftq_tail;
  ftq_count_1 = ftq_count;

  const int n = ibuf.count() < DECODE_WIDTH ? ibuf.count() : DECODE_WIDTH;
  for (int i = 0; i < DECODE_WIDTH; i++) wires->pre_issue.entries[i] = i < n ? ibuf.peek(i) : InstructionBufferEntry{};

  wires->pre2front.ready = false;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) wires->pre2front.fire[i] = false;
  push_count = 0;
  g_bypass = false;
}

// Accept one fetch block per cycle if the FTQ and the instruction buffer have room for all its instructions; the block
// gets an FTQ entry and each instruction a buffer entry (and a uop-pool block).
void PreIduQueue::comb_accept_front() {
  if (wires->idu_br_latch->mispred || wires->rob_bcast.flush) return;  // ready stays false

  int incoming = 0;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) incoming += wires->in.valid[i] ? 1 : 0;

  wires->pre2front.ready = ftq_count < FTQ_SIZE && ibuf.can_accept(incoming);
  if (!wires->pre2front.ready || incoming == 0) return;

  const int ftq_idx = ftq_alloc();
  Assert(ftq_idx >= 0 && "PreIduQueue: ready with a full FTQ");  // ready required a free slot and nothing allocated yet
  FTQEntry &block = ftq_lookup_entries_1[ftq_idx];
  block = FTQEntry();
  block.next_pc = wires->in.predict_next_fetch_address[0];
  block.nslots = 0;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) {
    block.slot_pc[i] = wires->in.pc[i];
    block.pred_taken_mask[i] = wires->in.predict_dir[i];
    if (wires->in.valid[i]) block.nslots = i + 1;
  }
  ftq_train_meta_fifo_1[ftq_idx] = FTQTrainMetaEntry();
  ftq_train_meta_fifo_1[ftq_idx].meta = wires->in.meta;
  ftq_valid_1[ftq_idx] = true;
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
    e.page_fault_inst = wires->in.page_fault_inst[i];
    e.ftq_idx = ftq_idx;
    e.ftq_offset = i;
    e.ftq_is_last = (i == last);
    e.uop_id = uop_alloc();
  }
  if (g_bypass_on && ibuf.count() == 0 && push_count > 0) {
    g_bypass = true;
    for (int i = 0; i < DECODE_WIDTH; i++) wires->pre_issue.entries[i] = i < push_count ? push_entries[i] : InstructionBufferEntry{};
  }
}

// Instruction buffer: decode consumes from the front (a leading run of fired slots), the accepted block is appended;
// flush and mispredict empty it (and the FTQ: everything, or what is younger than the branch's block). Committed blocks
// leave the FTQ head.
void PreIduQueue::comb_fire() {
  const bool flush = wires->rob_bcast.flush, mispred = wires->idu_br_latch->mispred;
  if (flush) {
    ftq_flush();
  } else if (mispred) {
    ftq_recover((wires->idu_br_latch->ftq_idx + 1) % FTQ_SIZE);
  }

  if (flush || mispred) {
    ibuf_1.clear();
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
  if (pop_count > 0) ibuf_1.pop_front(pop_count);
  for (int i = first_push; i < push_count; i++) ibuf_1.push_back(push_entries[i]);
  if (pop_count > 0 || push_count > first_push) ibuf_dirty = true;

  int committed_blocks = 0;
  for (int i = 0; i < COMMIT_WIDTH; i++)
    if (wires->rob_commit.commit_entry[i].valid && wires->rob_commit.commit_entry[i].uop.ftq_is_last) committed_blocks++;
  ftq_pop(committed_blocks);
}

void PreIduQueue::comb_ftq_lookup_prf() {
  for (int i = 0; i < FTQ_PRF_PC_PORT_NUM; i++) {
    const FtqPcReadReq &req = wires->ftq_prf_pc_req.req[i];
    wires->ftq_prf_pc_resp.resp[i] = {};
    if (req.valid) fill_ftq_pc_resp(wires->ftq_prf_pc_resp.resp[i], ftq_lookup_entries[req.ftq_idx], ftq_valid[req.ftq_idx], req);
  }
}

void PreIduQueue::comb_ftq_lookup_rob() {
  for (int i = 0; i < FTQ_ROB_PC_PORT_NUM; i++) {
    const FtqPcReadReq &req = wires->ftq_rob_pc_req.req[i];
    wires->ftq_rob_pc_resp.resp[i] = {};
    if (req.valid) fill_ftq_pc_resp(wires->ftq_rob_pc_resp.resp[i], ftq_lookup_entries[req.ftq_idx], ftq_valid[req.ftq_idx], req);
  }
}

// What the predictor needs to train on each committing instruction: the fetched PC / direction and the prediction
// metadata of its block. Blocks commit in order, so the metadata cursor starts at the FTQ head and advances after the
// last instruction of a block.
void PreIduQueue::comb_commit_info() {
  uint32_t cursor = static_cast<uint32_t>(ftq_head);
  bool cursor_valid = ftq_count > 0;

  for (int i = 0; i < COMMIT_WIDTH; i++) {
    auto &resp = wires->ftq_commit_info.resp[i];
    const auto &commit = wires->rob_commit.commit_entry[i];
    resp = {};
    if (!commit.valid) continue;

    const uint32_t idx = commit.uop->ftq_idx, off = commit.uop->ftq_offset;
    if (idx < FTQ_SIZE && ftq_valid[idx]) {
      resp.pc = ftq_lookup_entries[idx].slot_pc[off];
      resp.pred_taken = ftq_lookup_entries[idx].pred_taken_mask[off];
    }

    if (cursor_valid) {
      Assert(idx == cursor);
      const auto &m = ftq_train_meta_fifo[cursor].meta;
      resp.alt_pred = m.alt_pred[off];
      resp.altpcpn = m.altpcpn[off];
      resp.pcpn = m.pcpn[off];
      resp.ps_info = m.ps_info[off];
      resp.sc_used = m.sc_used[off];
      resp.sc_pred = m.sc_pred[off];
      resp.sc_sum = m.sc_sum[off];
      for (int t = 0; t < BPU_SCL_META_NTABLE; t++) resp.sc_idx[t] = m.sc_idx[off][t];
      resp.loop_used = m.loop_used[off];
      resp.loop_hit = m.loop_hit[off];
      resp.loop_pred = m.loop_pred[off];
      resp.loop_idx = m.loop_idx[off];
      resp.loop_tag = m.loop_tag[off];
      for (int k = 0; k < 4; k++) {
        resp.tage_idx[k] = m.tage_idx[off][k];
        resp.tage_tag[k] = m.tage_tag[off][k];
      }
    }

    if (commit.uop.ftq_is_last && cursor_valid) {
      const uint32_t next = (cursor + 1) % FTQ_SIZE;
      if (next == static_cast<uint32_t>(ftq_tail)) cursor_valid = false;
      else cursor = next;
    }
  }
}

void PreIduQueue::seq() {
  if (ibuf_dirty) {
    ibuf.sync_from(ibuf_1);
    ibuf_dirty = false;
  }
  ftq_head = ftq_head_1;
  ftq_tail = ftq_tail_1;
  ftq_count = ftq_count_1;
  if (ftq_dirty) {
    // Entry payloads only change at the allocated slot; valid bits are tiny.
    for (uint64_t m = ftq_alloc_mask; m; m &= m - 1) {
      const int i = __builtin_ctzll(m);
      ftq_lookup_entries[i] = ftq_lookup_entries_1[i];
      ftq_train_meta_fifo[i] = ftq_train_meta_fifo_1[i];
    }
    ftq_alloc_mask = 0;
    std::memcpy(ftq_valid, ftq_valid_1, sizeof(ftq_valid));
    ftq_dirty = false;
  }
}
