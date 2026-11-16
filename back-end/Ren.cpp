#include "Wires.h"
#include "Ren.h"
#include "Pmu.h"
#include "MsData.h"
#include "config.h"
#include <algorithm>
#include <cstdlib>
#include <cstring>

#include "util.h"

// Shared by several phases of a cycle.
static wire<1> fire[DECODE_WIDTH];
static wire<PRF_IDX_WIDTH> alloc_reg[DECODE_WIDTH];
static bool elim[DECODE_WIDTH];  // move elimination: renamed away, takes no free register

static inline int ring_distance(int from, int to) { return to >= from ? to - from : to - from + PRF_NUM; }

static inline int fp_ring_distance(int from, int to) { return to >= from ? to - from : to - from + FPRF_NUM; }
// FP free list invariant: free slots + registers held by in-flight instructions == the managed pool (no move elimination here)
static inline void fp_freelist_sanity(int spec_head, int commit_head, int tail) {
  constexpr int kManagedPool = FPRF_NUM - FP_ARF_NUM;
  Assert(spec_head >= 0 && spec_head < FPRF_NUM && commit_head >= 0 && commit_head < FPRF_NUM && tail >= 0 && tail < FPRF_NUM &&
         "Ren: FP free list pointer out of range");
  Assert(fp_ring_distance(spec_head, tail) + fp_ring_distance(commit_head, spec_head) == kManagedPool &&
         "Ren: FP free slots + pending spec must equal the managed pool (FP register leak)");
}

// Free list invariant: free slots + registers held by in-flight instructions == the managed pool.
void Ren::init() {
  Regs &r = q_;
  r = Regs{};
  int free_count = 0;
  for (int i = 0; i < PRF_NUM; i++) {
    if (i < ARF_NUM + 1) {
      r.spec_RAT[i] = r.arch_RAT[i] = i;
      r.arch_cnt[i] = 1;
    } else {
      r.arch_cnt[i] = 0;
      r.free_list[free_count++] = i;
    }
  }
  r.free_head = r.free_head_commit = 0;
  r.free_tail = free_count;
  for (int i = 0; i < MAX_BR_NUM; i++) r.alloc_checkpoint_head[i] = r.free_head;
  int fp_free_count = 0;
  for (int i = 0; i < FPRF_NUM; i++) {
    if (i < FP_ARF_NUM) r.fp_spec_RAT[i] = r.fp_arch_RAT[i] = FP_TAG_BASE + i;
    else r.fp_free_list[fp_free_count++] = FP_TAG_BASE + i;
  }
  r.fp_free_head = r.fp_free_head_commit = 0;
  r.fp_free_tail = fp_free_count;
  for (int i = 0; i < MAX_BR_NUM; i++) r.fp_alloc_checkpoint_head[i] = r.fp_free_head;
  fp_freelist_sanity(r.fp_free_head, r.fp_free_head_commit, r.fp_free_tail);
  d = q_;
  ckpt_dirty = ckpt_dirty_fp = 0;
  cnt_dirty = false;
}


// Move elimination bound. The architectural state after a move commits equals the speculative RAT right after its
// rename, so the architectural reference count of the source register never exceeds the speculative one. A move is
// eliminated only when its source comes straight from the speculative RAT (no older instruction of the group writes
// it) and  (RAT entries naming it) + (older instructions of the group with a destination) + 1  <= MOVE_ELIM_MAX_REFS;
// older writers can add at most one reference each, so the bound holds without looking at what they write.
bool Ren::elim_refs_ok(int i) const {
  const Regs &q = q_;
  const int s1 = q.inst_r[i]->src1_areg;
  int older = 0;
  for (int j = 0; j < i; j++) {
    if (!q.inst_valid[j] || !q.inst_r[j].dest_en) continue;
    if (q.inst_r[j]->dest_areg != 0 && !q.inst_r[j]->dest_fp && q.inst_r[j]->dest_areg == s1) return false;  // source bypassed inside the group
    older++;
  }
  int refs = 0;
  for (int e = 0; e < ARF_NUM + 1; e++)
    if (q.spec_RAT[e] == q.spec_RAT[s1]) refs++;
  const bool ok = refs + older + 1 <= MOVE_ELIM_MAX_REFS;
  return ok;
}

// Pick destination physical registers from the free list, in order. The first instruction that finds it empty stalls
// itself and everything behind it (ren2dis.valid = 0).
void Ren::alloc() {
  const Regs &q = q_;
  int head = q.free_head;
  int left = ring_distance(q.free_head, q.free_tail);
  int fhead = q.fp_free_head;
  int fleft = fp_ring_distance(q.fp_free_head, q.fp_free_tail);
  fp_freelist_sanity(q.fp_free_head, q.fp_free_head_commit, q.fp_free_tail);
  bool stall = false;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    alloc_reg[i] = 0;
    elim[i] = false;
    wires->ren2dis.valid[i] = false;
    if (!q.inst_valid[i]) continue;
    if (is_move_shape(q.inst_r[i]) && elim_refs_ok(i)) {
      elim[i] = true;
      wires->ren2dis.valid[i] = !stall;
    } else if (!q.inst_r[i].dest_en) {
      wires->ren2dis.valid[i] = !stall;
    } else if (q.inst_r[i]->dest_fp) {  // each domain stalls on its own free list; everything behind a stalled slot waits
      if (!stall && fleft > 0) {
        alloc_reg[i] = q.fp_free_list[fhead];
        wires->ren2dis.valid[i] = true;
        LOOP_INC(fhead, FPRF_NUM);
        fleft--;
      } else {
        stall = true;
      }
    } else if (!stall && left > 0) {
      alloc_reg[i] = q.free_list[head];
      wires->ren2dis.valid[i] = true;
      LOOP_INC(head, PRF_NUM);
      left--;
    } else {
      stall = true;
    }
  }
  g_pmu_back.ren_no_preg = stall;
}

// Source/old-destination lookup in q.spec_RAT; an older instruction of the same group that writes the same architectural
// register (x0 excluded) takes precedence, the youngest such writer wins.
void Ren::rename() {
  const Regs &q = q_;
  wire<PRF_IDX_WIDTH> dest_of[DECODE_WIDTH];  // final destination register of each slot (an eliminated move: its source)
  for (int i = 0; i < DECODE_WIDTH; i++) {
    wires->ren2dis.uop[i] = q.inst_r[i];
    wires->ren2dis.uop[i].dest_preg = alloc_reg[i];
    dest_of[i] = alloc_reg[i];
    // Static fields live in the uop pool: an invalid slot still holds the handle of an instruction that has long
    // left the stage, so it must not be written.
    if (!q.inst_valid[i]) continue;
    // EXP-020: a newly allocated physical register holds no stale value; the taint of a squashed wrong-path producer (a mispredict does not clear the table) must not leak to its next owner
    if (q.inst_r[i].dest_en && !elim[i] && alloc_reg[i] != 0 && alloc_reg[i] < TAG_NUM) msd::taint[alloc_reg[i]] = msd::TV{};
    auto lookup = [&](int areg, bool fp) {
      wire<PRF_IDX_WIDTH> p = 0;
      if (fp) p = q.fp_spec_RAT[areg]; else
      p = q.spec_RAT[areg];
      for (int j = 0; j < i; j++) {
        if (!q.inst_valid[j] || !q.inst_r[j].dest_en || bool(q.inst_r[j]->dest_fp) != fp) continue;
        if (fp ? q.inst_r[j]->dest_areg == areg : (q.inst_r[j]->dest_areg != 0 && q.inst_r[j]->dest_areg == areg)) p = dest_of[j];
      }
      return p;
    };
    wires->ren2dis.uop[i]->src1_preg = lookup(q.inst_r[i]->src1_areg, q.inst_r[i]->src1_fp);
    wires->ren2dis.uop[i]->src2_preg = lookup(q.inst_r[i]->src2_areg, q.inst_r[i]->src2_fp);
    if (q.inst_r[i]->src3_en) wires->ren2dis.uop[i]->src3_preg = lookup(q.inst_r[i]->src3_areg, q.inst_r[i]->src3_fp);
    wires->ren2dis.uop[i]->old_dest_preg = lookup(q.inst_r[i]->dest_areg, q.inst_r[i]->dest_fp);
    if (elim[i]) {
      dest_of[i] = wires->ren2dis.uop[i]->src1_preg;
      wires->ren2dis.uop[i].dest_preg = dest_of[i];
    }
  }
}

namespace {
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

// Rename state updates: fired instructions take their register and checkpoint q.spec_RAT at branches, commits free old
// registers and update q.arch_RAT, then flush (restore from q.arch_RAT) or mispredict (restore a checkpoint) override
// the next-cycle spec state.
void Ren::handshake() {
  const Regs &q = q_;
  const bool flush = wires->rob_bcast.flush, mispred = wires->dec_bcast.mispred;
  for (int i = 0; i < DECODE_WIDTH; i++) fire[i] = wires->ren2dis.valid[i] && wires->dis2ren.ready;

  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (!fire[i]) continue;
    if (wires->ren2dis.uop[i].dest_en) {
      if (q.inst_r[i]->dest_fp) {
        d.fp_spec_RAT[q.inst_r[i]->dest_areg] = wires->ren2dis.uop[i].dest_preg;
        Assert(d.fp_free_head != d.fp_free_tail && "Ren: allocate when the FP free list is empty");
        LOOP_INC(d.fp_free_head, FPRF_NUM);
      } else
      {
        d.spec_RAT[q.inst_r[i]->dest_areg] = wires->ren2dis.uop[i].dest_preg;
        if (!elim[i]) {
          Assert(d.free_head != d.free_tail && "Ren: allocate when free list is empty");
          LOOP_INC(d.free_head, PRF_NUM);
        }
      }
    }
    if (is_branch(q.inst_r[i]->type)) {  // the checkpoint includes this instruction's own rename, not younger ones
      const auto br_id = q.inst_r[i]->br_id;
      Assert(br_id < MAX_BR_NUM && "Ren: branch id out of range");
      memcpy(d.RAT_checkpoint[br_id], d.spec_RAT, sizeof(d.spec_RAT));
      ckpt_dirty |= uint64_t(1) << br_id;
      d.alloc_checkpoint_head[br_id] = d.free_head;
      ckpt_dirty_fp |= uint64_t(1) << br_id;
      memcpy(d.fp_RAT_checkpoint[br_id], d.fp_spec_RAT, sizeof(d.fp_spec_RAT));
      d.fp_alloc_checkpoint_head[br_id] = d.fp_free_head;
    }
  }

  wires->ren2dec.ready = !flush && !mispred;
  for (int i = 0; i < DECODE_WIDTH; i++) wires->ren2dec.ready &= fire[i] || !q.inst_valid[i];

  const bool interrupt = wires->rob_bcast.interrupt;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    if (!wires->rob_commit.commit_entry[i].valid) continue;
    const auto &commit_uop = wires->rob_commit.commit_entry[i].uop;
    if (!interrupt) count_commit(ctx, commit_uop);

    InstEntry commit_entry = commit_uop.to_inst_entry(wires->rob_commit.commit_entry[i].valid);
    InstInfo *inst = &commit_entry.uop;
    // An exception instruction does not free its old destination: it looks as if it never committed.
    if (inst->dest_en && !is_exception(*inst) && !interrupt && (*inst)->dest_fp) {  // FP destination: no move elimination
      Assert(d.fp_free_head_commit != d.fp_free_head && "Ren: FP commit reclaim when no pending spec allocation");
      d.fp_free_list[d.fp_free_tail] = (*inst)->old_dest_preg;
      LOOP_INC(d.fp_free_tail, FPRF_NUM);
      LOOP_INC(d.fp_free_head_commit, FPRF_NUM);
      d.fp_arch_RAT[(*inst)->dest_areg] = inst->dest_preg;
    } else
    if (inst->dest_en && !is_exception(*inst) && !interrupt) {
      // move elimination: the old register is freed when no architectural register names it any more; an eliminated
      // move allocated nothing, so it does not advance the commit head
      Assert(!mispred && "Ren(ME): commit together with mispredict");
      if (!is_eliminated_move(*inst)) {
        Assert(d.free_head_commit != d.free_head && "Ren: commit reclaim when no pending spec allocation");
        LOOP_INC(d.free_head_commit, PRF_NUM);
      }
      d.arch_cnt[inst->dest_preg]++, cnt_dirty = true;
      const auto old = (*inst)->old_dest_preg;
      if (--d.arch_cnt[old] == 0) {
        d.free_list[d.free_tail] = old;
        LOOP_INC(d.free_tail, PRF_NUM);
      }
      d.arch_RAT[(*inst)->dest_areg] = inst->dest_preg;
    }
    if (!interrupt) ctx->run_commit_inst(&commit_entry, i);
  }
  if (interrupt) ctx->run_interrupt_hook();

  if (flush) {
    memcpy(d.spec_RAT, d.arch_RAT, sizeof(d.spec_RAT));
    d.free_head = d.free_head_commit;
    memcpy(d.fp_spec_RAT, d.fp_arch_RAT, sizeof(d.fp_spec_RAT));
    d.fp_free_head = d.fp_free_head_commit;
  } else if (mispred) {  // never together with flush
    const auto br = wires->dec_bcast.br_id;
    Assert(br < MAX_BR_NUM && br != 0 && "Ren: mispred br_id out of range");
    memcpy(d.spec_RAT, q.RAT_checkpoint[br], sizeof(d.spec_RAT));
    d.free_head = q.alloc_checkpoint_head[br];
    d.free_tail = q.free_tail;  // reclaims of this cycle are dropped, as in the original
    memcpy(d.fp_spec_RAT, q.fp_RAT_checkpoint[br], sizeof(d.fp_spec_RAT));
    d.fp_free_head = q.fp_alloc_checkpoint_head[br];
    d.fp_free_tail = q.fp_free_tail;
  }
  fp_freelist_sanity(d.fp_free_head, d.fp_free_head_commit, d.fp_free_tail);
}

// Pipeline register: emptied by flush/mispredict, reloaded from decode when ready, else keeps unfired instructions.
// Bits of branches resolved this cycle are cleared from every mask that stays.
void Ren::pipeline() {
  const Regs &q = q_;
  const wire<BR_MASK_WIDTH> clear = wires->dec_bcast.clear_mask;
  const bool drain = wires->rob_bcast.flush || wires->dec_bcast.mispred;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (drain) {
      d.inst_valid[i] = false;
    } else if (wires->ren2dec.ready) {
      d.inst_valid[i] = wires->dec2ren.valid[i];
      d.inst_r[i] = wires->dec2ren.uop[i];
      d.inst_r[i].br_mask &= ~clear;
    } else {
      d.inst_valid[i] = q.inst_valid[i] && !fire[i];
      if (d.inst_valid[i]) d.inst_r[i].br_mask &= ~clear;
    }
  }
}

// The clock edge: q = d. The checkpoint rows and the reference counts change rarely: only the dirty ones are copied
// (d == q holds again afterwards).
void Ren::seq() {
  Regs &q = q_;
  memcpy(q.inst_r, d.inst_r, sizeof(q.inst_r));
  memcpy(q.inst_valid, d.inst_valid, sizeof(q.inst_valid));
  memcpy(q.spec_RAT, d.spec_RAT, sizeof(q.spec_RAT));
  memcpy(q.arch_RAT, d.arch_RAT, sizeof(q.arch_RAT));
  memcpy(q.free_list, d.free_list, sizeof(q.free_list));
  q.free_head = d.free_head;
  q.free_head_commit = d.free_head_commit;
  q.free_tail = d.free_tail;
  for (uint64_t m = ckpt_dirty; m != 0; m &= m - 1) {
    const int row = __builtin_ctzll(m);
    memcpy(q.RAT_checkpoint[row], d.RAT_checkpoint[row], sizeof(q.RAT_checkpoint[row]));
  }
  ckpt_dirty = 0;
  if (cnt_dirty) memcpy(q.arch_cnt, d.arch_cnt, sizeof(q.arch_cnt));
  cnt_dirty = false;
  memcpy(q.alloc_checkpoint_head, d.alloc_checkpoint_head, sizeof(q.alloc_checkpoint_head));
  memcpy(q.fp_spec_RAT, d.fp_spec_RAT, sizeof(q.fp_spec_RAT));
  memcpy(q.fp_arch_RAT, d.fp_arch_RAT, sizeof(q.fp_arch_RAT));
  memcpy(q.fp_free_list, d.fp_free_list, sizeof(q.fp_free_list));
  q.fp_free_head = d.fp_free_head;
  q.fp_free_head_commit = d.fp_free_head_commit;
  q.fp_free_tail = d.fp_free_tail;
  for (uint64_t m = ckpt_dirty_fp; m != 0; m &= m - 1) {
    const int row = __builtin_ctzll(m);
    memcpy(q.fp_RAT_checkpoint[row], d.fp_RAT_checkpoint[row], sizeof(q.fp_RAT_checkpoint[row]));
  }
  ckpt_dirty_fp = 0;
  memcpy(q.fp_alloc_checkpoint_head, d.fp_alloc_checkpoint_head, sizeof(q.fp_alloc_checkpoint_head));
}

// allocate the destination registers and rename the decoded group
void Ren::comb_rename() {
  alloc();
  rename();
}

// dispatch handshake: commit the rename, the next pipeline register
void Ren::comb_fire() {
  handshake();
  pipeline();
}
