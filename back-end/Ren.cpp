#include "Wires.h"
#include "BeginCheck.h"
#include "Ren.h"
#include "config.h"
#include <cstdlib>
#include <cstring>

#include "util.h"

// Shared by several phases of a cycle.
static wire<1> fire[DECODE_WIDTH];
static wire<PRF_IDX_WIDTH> alloc_reg[DECODE_WIDTH];

static inline int ring_distance(int from, int to) { return to >= from ? to - from : to - from + PRF_NUM; }

// Free list invariant: free slots + registers held by in-flight instructions == the managed pool.
static inline void ren_freelist_sanity(int spec_head, int commit_head, int tail) {
  constexpr int kManagedPool = PRF_NUM - (ARF_NUM + 1);
  Assert(spec_head >= 0 && spec_head < PRF_NUM && commit_head >= 0 && commit_head < PRF_NUM && tail >= 0 &&
         tail < PRF_NUM && "Ren: free list pointer out of range");
  Assert(ring_distance(spec_head, tail) + ring_distance(commit_head, spec_head) == kManagedPool &&
         "Ren: free slots + pending spec must equal managed pool");
}

void Ren::init() {
  int free_count = 0;
  for (int i = 0; i < PRF_NUM; i++) {
    if (i < ARF_NUM + 1) {
      spec_RAT[i] = arch_RAT[i] = i;
    } else {
      free_list[free_count++] = i;
    }
  }
  free_head = free_head_commit = 0;
  free_tail = free_count;
  for (int i = 0; i < MAX_BR_NUM; i++) alloc_checkpoint_head[i] = free_head;
  ren_freelist_sanity(free_head, free_head_commit, free_tail);

  for (int i = 0; i < DECODE_WIDTH; i++) {
    inst_r[i] = inst_r_1[i] = {};
    inst_valid[i] = inst_valid_1[i] = false;
  }
  memcpy(arch_RAT_1, arch_RAT, sizeof(arch_RAT));
  memcpy(spec_RAT_1, spec_RAT, sizeof(spec_RAT));
  memcpy(free_list_1, free_list, sizeof(free_list));
  free_head_1 = free_head;
  free_head_commit_1 = free_head_commit;
  free_tail_1 = free_tail;
  memcpy(alloc_checkpoint_head_1, alloc_checkpoint_head, sizeof(alloc_checkpoint_head));
}

// The working copies already equal the registered state (seq() copies them back).
void Ren::comb_begin() {
  BEGIN_EQ(inst_r_1, inst_r);
  BEGIN_EQ(inst_valid_1, inst_valid);
  BEGIN_EQ(spec_RAT_1, spec_RAT);
  BEGIN_EQ(arch_RAT_1, arch_RAT);
  BEGIN_EQ(RAT_checkpoint_1, RAT_checkpoint);
  BEGIN_EQ(free_list_1, free_list);
  BEGIN_EQ(free_head_1, free_head);
  BEGIN_EQ(free_head_commit_1, free_head_commit);
  BEGIN_EQ(free_tail_1, free_tail);
  BEGIN_EQ(alloc_checkpoint_head_1, alloc_checkpoint_head);
}

// Pick destination physical registers from the free list, in order. The first instruction that finds it empty stalls
// itself and everything behind it (ren2dis.valid = 0).
void Ren::comb_alloc() {
  ren_freelist_sanity(free_head, free_head_commit, free_tail);
  int head = free_head;
  int left = ring_distance(free_head, free_tail);
  bool stall = false;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    alloc_reg[i] = 0;
    wires->ren2dis.valid[i] = false;
    if (!inst_valid[i]) continue;
    if (!inst_r[i].dest_en) {
      wires->ren2dis.valid[i] = !stall;
    } else if (!stall && left > 0) {
      alloc_reg[i] = free_list[head];
      wires->ren2dis.valid[i] = true;
      LOOP_INC(head, PRF_NUM);
      left--;
    } else {
      stall = true;
    }
  }
}

// Source/old-destination lookup in spec_RAT; an older instruction of the same group that writes the same architectural
// register (x0 excluded) takes precedence, the youngest such writer wins.
void Ren::comb_rename() {
  for (int i = 0; i < DECODE_WIDTH; i++) {
    wires->ren2dis.uop[i] = inst_r[i];
    wires->ren2dis.uop[i].dest_preg = alloc_reg[i];
    // Static fields live in the uop pool: an invalid slot still holds the handle of an instruction that has long
    // left the stage, so it must not be written.
    if (!inst_valid[i]) continue;
    auto lookup = [&](int areg) {
      wire<PRF_IDX_WIDTH> p = spec_RAT[areg];
      for (int j = 0; j < i; j++)
        if (inst_valid[j] && inst_r[j].dest_en && inst_r[j]->dest_areg != 0 && inst_r[j]->dest_areg == areg)
          p = alloc_reg[j];
      return p;
    };
    wires->ren2dis.uop[i]->src1_preg = lookup(inst_r[i]->src1_areg);
    wires->ren2dis.uop[i]->src2_preg = lookup(inst_r[i]->src2_areg);
    wires->ren2dis.uop[i]->old_dest_preg = lookup(inst_r[i]->dest_areg);
  }
}

namespace {
// Commit statistics and the checkpoint warm-up / measurement window.
void count_commit(SimContext *ctx, const Uop &u) {
  ctx->perf.commit_num++;
  if (is_load(u)) ctx->perf.commit_load_num++;
  if (is_store(u)) ctx->perf.commit_store_num++;
  if (!ctx->is_ckpt) return;
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

// Rename state updates: fired instructions take their register and checkpoint spec_RAT at branches, commits free old
// registers and update arch_RAT, then flush (restore from arch_RAT) or mispredict (restore a checkpoint) override
// the next-cycle spec state.
void Ren::comb_fire() {
  const bool flush = wires->rob_bcast.flush, mispred = wires->dec_bcast.mispred;
  for (int i = 0; i < DECODE_WIDTH; i++) fire[i] = wires->ren2dis.valid[i] && wires->dis2ren.ready;

  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (!fire[i]) continue;
    if (wires->ren2dis.uop[i].dest_en) {
      Assert(free_head_1 != free_tail_1 && "Ren: allocate when free list is empty");
      spec_RAT_1[inst_r[i]->dest_areg] = wires->ren2dis.uop[i].dest_preg;
      LOOP_INC(free_head_1, PRF_NUM);
    }
    if (is_branch(inst_r[i]->type)) {  // the checkpoint includes this instruction's own rename, not younger ones
      const auto br_id = inst_r[i]->br_id;
      Assert(br_id < MAX_BR_NUM && "Ren: branch id out of range");
      memcpy(RAT_checkpoint_1[br_id], spec_RAT_1, sizeof(spec_RAT_1));
      alloc_checkpoint_head_1[br_id] = free_head_1;
    }
  }

  wires->ren2dec.ready = !flush && !mispred;
  for (int i = 0; i < DECODE_WIDTH; i++) wires->ren2dec.ready &= fire[i] || !inst_valid[i];

  const bool interrupt = wires->rob_bcast.interrupt;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    if (!wires->rob_commit.commit_entry[i].valid) continue;
    const auto &commit_uop = wires->rob_commit.commit_entry[i].uop;
    if (!interrupt) count_commit(ctx, commit_uop);

    InstEntry commit_entry = commit_uop.to_inst_entry(wires->rob_commit.commit_entry[i].valid);
    InstInfo *inst = &commit_entry.uop;
    // An exception instruction does not free its old destination: it looks as if it never committed.
    if (inst->dest_en && !is_exception(*inst) && !interrupt) {
      Assert(free_head_commit_1 != free_head_1 && "Ren: commit reclaim when no pending spec allocation");
      free_list_1[free_tail_1] = (*inst)->old_dest_preg;
      LOOP_INC(free_tail_1, PRF_NUM);
      LOOP_INC(free_head_commit_1, PRF_NUM);
      arch_RAT_1[(*inst)->dest_areg] = inst->dest_preg;
    }
    BE_LOG("ROB commit PC=0x%08x Inst=0x%08x idx=%ld", (*inst)->dbg.pc, (*inst)->dbg.instruction,
           (*inst)->dbg.inst_idx);
    if (!interrupt) ctx->run_commit_inst(&commit_entry, i);
  }
  if (interrupt) ctx->run_interrupt_hook();

  if (flush) {
    memcpy(spec_RAT_1, arch_RAT_1, sizeof(spec_RAT_1));
    free_head_1 = free_head_commit_1;
  } else if (mispred) {  // never together with flush
    const auto br = wires->dec_bcast.br_id;
    Assert(br < MAX_BR_NUM && br != 0 && "Ren: mispred br_id out of range");
    memcpy(spec_RAT_1, RAT_checkpoint[br], sizeof(spec_RAT_1));
    free_head_1 = alloc_checkpoint_head[br];
    free_tail_1 = free_tail;  // reclaims of this cycle are dropped, as in the original
  }
  ren_freelist_sanity(free_head_1, free_head_commit_1, free_tail_1);
}

// Pipeline register: emptied by flush/mispredict, reloaded from decode when ready, else keeps unfired instructions.
// Bits of branches resolved this cycle are cleared from every mask that stays.
void Ren::comb_pipeline() {
  const wire<BR_MASK_WIDTH> clear = wires->dec_bcast.clear_mask;
  const bool drain = wires->rob_bcast.flush || wires->dec_bcast.mispred;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (drain) {
      inst_valid_1[i] = false;
    } else if (wires->ren2dec.ready) {
      inst_valid_1[i] = wires->dec2ren.valid[i];
      inst_r_1[i] = wires->dec2ren.uop[i];
      inst_r_1[i].br_mask &= ~clear;
    } else {
      inst_valid_1[i] = inst_valid[i] && !fire[i];
      if (inst_valid_1[i]) inst_r_1[i].br_mask &= ~clear;
    }
  }
}

void Ren::seq() {
  memcpy(inst_r, inst_r_1, sizeof(inst_r));
  memcpy(inst_valid, inst_valid_1, sizeof(inst_valid));
  memcpy(spec_RAT, spec_RAT_1, sizeof(spec_RAT));
  memcpy(arch_RAT, arch_RAT_1, sizeof(arch_RAT));
  memcpy(free_list, free_list_1, sizeof(free_list));
  free_head = free_head_1;
  free_head_commit = free_head_commit_1;
  free_tail = free_tail_1;
  memcpy(RAT_checkpoint, RAT_checkpoint_1, sizeof(RAT_checkpoint));
  memcpy(alloc_checkpoint_head, alloc_checkpoint_head_1, sizeof(alloc_checkpoint_head));
}
