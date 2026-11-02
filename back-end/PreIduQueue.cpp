#include "Wires.h"
#include <cstdlib>
#include <cstdio>
#include "PreIduQueue.h"
#include "types.h"

#include <cstring>

static InstructionBufferEntry push_entries[BACKEND_FETCH_WIDTH];
static int push_count = 0;

static void fill_ftq_pc_resp(FtqPcReadResp &resp, const FTQEntry &entry,
                             wire<1> entry_valid, const FtqPcReadReq &req) {
  resp = {};
  if (!req.valid) {
    return;
  }

  resp.valid = true;
  resp.entry_valid = entry_valid;
  resp.pc = entry.slot_pc[req.ftq_offset];
  resp.pred_taken = entry.pred_taken_mask[req.ftq_offset];
  resp.next_pc = entry.next_pc;
}

int PreIduQueue::ftq_alloc() {
  if (ftq_count_1 >= FTQ_SIZE) {
    return -1;
  }
  int idx = ftq_tail_1;
  ftq_tail_1 = (ftq_tail_1 + 1) % FTQ_SIZE;
  ftq_count_1++;
  return idx;
}

void PreIduQueue::ftq_pop(int pop_cnt) {
  if (pop_cnt <= 0) {
    return;
  }
  for (int i = 0; i < pop_cnt; i++) {
    if (ftq_count_1 <= 0) {
      break;
    }
    ftq_valid_1[ftq_head_1] = false;
    ftq_dirty = true;
    ftq_head_1 = (ftq_head_1 + 1) % FTQ_SIZE;
    ftq_count_1--;
  }
}

void PreIduQueue::ftq_recover(int new_tail) {
  int normalized_tail = ((new_tail % FTQ_SIZE) + FTQ_SIZE) % FTQ_SIZE;
  int discarded = (ftq_tail_1 - normalized_tail + FTQ_SIZE) % FTQ_SIZE;
  for (int i = 0; i < discarded; i++) {
    int idx = (normalized_tail + i) % FTQ_SIZE;
    ftq_valid_1[idx] = false;
    ftq_dirty = true;
  }
  ftq_tail_1 = normalized_tail;
  ftq_count_1 -= discarded;
}

void PreIduQueue::ftq_flush() {
  ftq_head_1 = 0;
  ftq_tail_1 = 0;
  ftq_count_1 = 0;
  for (int i = 0; i < FTQ_SIZE; i++) {
    ftq_valid_1[i] = false;
  }
  ftq_dirty = true;
}

void PreIduQueue::init() {
  ibuf.init();
  ibuf_1.init();
  ftq_head = ftq_tail = ftq_count = 0;
  ftq_head_1 = ftq_tail_1 = ftq_count_1 = 0;
  push_count = 0;
  for (int i = 0; i < FTQ_SIZE; i++) {
    ftq_lookup_entries[i] = FTQEntry();
    ftq_lookup_entries_1[i] = FTQEntry();
    ftq_train_meta_fifo[i] = FTQTrainMetaEntry();
    ftq_train_meta_fifo_1[i] = FTQTrainMetaEntry();
    ftq_valid[i] = false;
    ftq_valid_1[i] = false;
  }
}

void PreIduQueue::comb_begin() {
#ifdef IQ_SELFCHECK
  if (std::memcmp(&ibuf, &ibuf_1, sizeof(ibuf)) != 0 ||
      std::memcmp(ftq_lookup_entries, ftq_lookup_entries_1, sizeof(ftq_lookup_entries)) != 0 ||
      std::memcmp(ftq_train_meta_fifo, ftq_train_meta_fifo_1, sizeof(ftq_train_meta_fifo)) != 0 ||
      std::memcmp(ftq_valid, ftq_valid_1, sizeof(ftq_valid)) != 0) {
    std::fprintf(stderr, "[IQ_SELFCHECK] PreIduQueue committed != working\n");
    std::abort();
  }
#endif
  ftq_head_1 = ftq_head;
  ftq_tail_1 = ftq_tail;
  ftq_count_1 = ftq_count;

  for (auto &e : wires->pre_issue.entries) {
    e = {};
  }
  int n = ibuf.count() < DECODE_WIDTH ? ibuf.count() : DECODE_WIDTH;
  for (int i = 0; i < n; i++) {
    wires->pre_issue.entries[i] = ibuf.peek(i);
  }

  wires->pre2front.ready = false;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) {
    wires->pre2front.fire[i] = false;
  }

  push_count = 0;
}

void PreIduQueue::comb_accept_front() {
  if ((*wires->idu_br_latch).mispred || wires->rob_bcast.flush) {
    wires->pre2front.ready = false;
    return;
  }

  int incoming_valid_num = 0;
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) {
    incoming_valid_num += wires->in.valid[i] ? 1 : 0;
  }

  bool ftq_ok = (ftq_count < FTQ_SIZE);
  bool ib_ok = ibuf.can_accept(incoming_valid_num);
  wires->pre2front.ready = ftq_ok && ib_ok;
  if (ctx != nullptr && incoming_valid_num > 0 && !wires->pre2front.ready) {
    if (!ib_ok) {

    }
    if (!ftq_ok) {

    }
  }
  if (!wires->pre2front.ready || incoming_valid_num == 0) {
    return;
  }

  FTQEntry ftq_lookup_entry;
  FTQTrainMetaEntry ftq_train_meta_entry;
  ftq_lookup_entry.next_pc = wires->in.predict_next_fetch_address[0];
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) {
    ftq_lookup_entry.slot_pc[i] = wires->in.pc[i];
    ftq_lookup_entry.pred_taken_mask[i] = wires->in.predict_dir[i];
  }
  ftq_train_meta_entry.meta = wires->in.meta;
  const int ftq_alloc_idx = ftq_alloc();
  if (ftq_alloc_idx < 0) {
    wires->pre2front.ready = false;
    return;
  }
  ftq_lookup_entries_1[ftq_alloc_idx] = ftq_lookup_entry;
  ftq_train_meta_fifo_1[ftq_alloc_idx] = ftq_train_meta_entry;
  ftq_valid_1[ftq_alloc_idx] = true;
  ftq_alloc_mask |= 1ull << ftq_alloc_idx;
  ftq_dirty = true;

  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) {
    wires->pre2front.fire[i] = wires->in.valid[i];
  }

  int last_fire_idx = -1;
  for (int i = BACKEND_FETCH_WIDTH - 1; i >= 0; i--) {
    if (wires->pre2front.fire[i]) {
      last_fire_idx = i;
      break;
    }
  }
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) {
    if (!wires->pre2front.fire[i]) {
      continue;
    }
    auto &e = push_entries[push_count++];
    e.valid = true;
    e.inst = wires->in.inst[i];
    e.pc = wires->in.pc[i];
    e.page_fault_inst = wires->in.page_fault_inst[i];
    e.ftq_idx = ftq_alloc_idx;
    e.ftq_offset = i;
    e.ftq_is_last = (i == last_fire_idx);
    e.uop_id = uop_alloc();
  }

  if (ctx != nullptr && push_count > 0) {


  }
}

void PreIduQueue::comb_fire() {
  int pop_count = 0;
  bool ftq_flush_req = false;
  bool ftq_recover_req = false;
  int ftq_recover_tail = 0;

  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (wires->idu_consume.fire[i]) {
      pop_count++;
    } else {
      break;
    }
  }

  if (wires->rob_bcast.flush) {
    ftq_flush_req = true;
  } else if ((*wires->idu_br_latch).mispred) {
    ftq_recover_req = true;
    ftq_recover_tail = ((*wires->idu_br_latch).ftq_idx + 1) % FTQ_SIZE;
  }

  if (ftq_flush_req) {
    ftq_flush();
  } else if (ftq_recover_req) {
    ftq_recover(ftq_recover_tail);
  }

  if (wires->rob_bcast.flush) {
    ibuf_1.clear();
    ibuf_dirty = true;
  } else if ((*wires->idu_br_latch).mispred) {
    ibuf_1.clear();
    ibuf_dirty = true;
  } else {
    if (pop_count > 0) {
      ibuf_1.pop_front(pop_count);
      ibuf_dirty = true;
    }
    if (push_count > 0) {
      ibuf_dirty = true;
      for (int i = 0; i < push_count; i++) {
        ibuf_1.push_back(push_entries[i]);
      }
    }
  }

  if (ftq_flush_req || ftq_recover_req) {
    return;
  }
  int pop_cnt = 0;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    if (wires->rob_commit.commit_entry[i].valid &&
        wires->rob_commit.commit_entry[i].uop.ftq_is_last) {
      pop_cnt++;
    }
  }
  ftq_pop(pop_cnt);
}

void PreIduQueue::comb_ftq_lookup_prf() {
  for (int i = 0; i < FTQ_PRF_PC_PORT_NUM; i++) {
    wires->ftq_prf_pc_resp.resp[i] = {};
    if (wires->ftq_prf_pc_req.req[i].valid) {
      fill_ftq_pc_resp(wires->ftq_prf_pc_resp.resp[i],
                       ftq_lookup_entries[wires->ftq_prf_pc_req.req[i].ftq_idx],
                       ftq_valid[wires->ftq_prf_pc_req.req[i].ftq_idx],
                       wires->ftq_prf_pc_req.req[i]);
    }
  }
}

void PreIduQueue::comb_ftq_lookup_rob() {
  for (int i = 0; i < FTQ_ROB_PC_PORT_NUM; i++) {
    wires->ftq_rob_pc_resp.resp[i] = {};
    if (wires->ftq_rob_pc_req.req[i].valid) {
      fill_ftq_pc_resp(wires->ftq_rob_pc_resp.resp[i],
                       ftq_lookup_entries[wires->ftq_rob_pc_req.req[i].ftq_idx],
                       ftq_valid[wires->ftq_rob_pc_req.req[i].ftq_idx],
                       wires->ftq_rob_pc_req.req[i]);
    }
  }
}

void PreIduQueue::comb_commit_info() {
  uint32_t train_meta_cursor = static_cast<uint32_t>(ftq_head);
  bool train_meta_cursor_valid = ftq_count > 0;

  for (int i = 0; i < COMMIT_WIDTH; i++) {
    auto &resp = wires->ftq_commit_info.resp[i];
    const auto &commit = wires->rob_commit.commit_entry[i];
    resp = {};
    if (!commit.valid) {
      continue;
    }

    const uint32_t ftq_idx = commit.uop->ftq_idx;
    const uint32_t ftq_offset = commit.uop->ftq_offset;
    if (ftq_idx < FTQ_SIZE && ftq_valid[ftq_idx]) {
      resp.pc = ftq_lookup_entries[ftq_idx].slot_pc[ftq_offset];
      resp.pred_taken = ftq_lookup_entries[ftq_idx].pred_taken_mask[ftq_offset];
    }

    if (train_meta_cursor_valid) {
      Assert(ftq_idx == train_meta_cursor);
      const auto &meta = ftq_train_meta_fifo[train_meta_cursor];
      resp.alt_pred = meta.meta.alt_pred[ftq_offset];
      resp.altpcpn = meta.meta.altpcpn[ftq_offset];
      resp.pcpn = meta.meta.pcpn[ftq_offset];
      resp.sc_used = meta.meta.sc_used[ftq_offset];
      resp.sc_pred = meta.meta.sc_pred[ftq_offset];
      resp.sc_sum = meta.meta.sc_sum[ftq_offset];
      for (int t = 0; t < BPU_SCL_META_NTABLE; ++t) {
        resp.sc_idx[t] = meta.meta.sc_idx[ftq_offset][t];
      }
      resp.loop_used = meta.meta.loop_used[ftq_offset];
      resp.loop_hit = meta.meta.loop_hit[ftq_offset];
      resp.loop_pred = meta.meta.loop_pred[ftq_offset];
      resp.loop_idx = meta.meta.loop_idx[ftq_offset];
      resp.loop_tag = meta.meta.loop_tag[ftq_offset];
      for (int k = 0; k < 4; k++) {
        resp.tage_idx[k] = meta.meta.tage_idx[ftq_offset][k];
        resp.tage_tag[k] = meta.meta.tage_tag[ftq_offset][k];
      }
    }

    if (commit.uop.ftq_is_last && train_meta_cursor_valid) {
      const uint32_t next = (train_meta_cursor + 1) % FTQ_SIZE;
      if (next == static_cast<uint32_t>(ftq_tail)) {
        train_meta_cursor_valid = false;
      } else {
        train_meta_cursor = next;
      }
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
